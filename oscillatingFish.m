classdef oscillatingFish < handle
    % This class provides controls for coordinating circular group motion
    % with oscillatory speed. The controled robots simulate the motion of a
    % school of fish. Currently implemented using all-to-all communication,
    % neighboring robots stabilize to circular motion about the group
    % center of mass. Controling heading angles allows for either splayed,
    % synchronized, or arbitrary spacing about the overall circle. Speed
    % phase is also controlled to allow for alternating spatial patterns
    % within clusters of robots. 
    %
    % SYNTAX
    %
    % school = oscillatingFish(initial_positions, options)
    %
    % INPUTS
    %
    % initial_poses: n_robots X [x y z theta] matrix of the initial
    % positions and headings of the robots.
    %
    % OPTIONS
    %
    % 'headings' - Choose between 'splay', 'sync', or 'none' to set
    % control law coefficients to stabilize either splayed, synchronized, 
    % or arbitrary robot positioning about overall orbit. Default: sync.
    %
    % 'collision_avoidance' - Not yet implemented. Logical. 
    % True = enable collision avoidance, false = disable collision avoidance.
    %
    % PROPERTIES
    %
    % omega - natural turning rate for robots. In rad/s.
    %
    % Omega - natural oscillating forward speed freqency. In rad/s.
    %
    % mu - Amplitude of forward speed oscillation.
    %
    % k - control constant for turning rate adjustment.
    %
    % k_phi - control constant for forward speed phase adjustment. 
    %
    % scale - Speed conversion factor. (scale) # units/s = 1 meter/s.
    %
    %
    % METHODS
    %
    % fishControlLaw(obj, t, states) - takes the oscillating fish object,
    % current time, and current states of the robots and returns an N x 3 
    % matrix of commands for new forward velocity, turning rate, and 
    % vertical velocity (always 0).
    %
    % simulate(obj, runTime, options) - simulates trajectory of robots for specified 
    % runtime (seconds). 
    %       'animate' - logical. Draw particle simulation of robots moving
    %                   along the trajectories. 
    %       'animation_speed' - numerical. Pause time between point
    %                           updates. Default: 0.2.
    %        Graph options - 'headings', 'phases', 'trajectory', 'ellipse',
    %                        'graph_all'
    %                        logical. User selects which plots to display.
    %                        'graph_all' overides and displays all graphs. 
    %
    % demoConditions(obj, runTime, option) - sets up good demonstration
    % conditions for the Miabot robots in current DCSL floor robot
    % setup.
    %       'n_robots' - Change number of robots. Default is original
    %                    number oscillatingFish was initialized to. Currently
    %                    limited to n = 2 or 3 robots.
    %
    %       'headings' - Choose between 'sync' or 'splay'. Default is
    %                    'sync'. Currently 3 robots only supports splay state. 
    %**********************************************************************
    
    properties (Access = public)
        omega = .8;           % Natural heading turning frequency
        Omega = .8*1.5;       % Natural speed phase frequency
        mu = 0.5;             % Speed oscillation parameter
        k = 1;                % Steering control parameter
        k_phi = 1;            % Speed phase control parameter
        scale = 5;            % Commands scaling  1 meter: scale
        initial_poses;        % initial robot positions
    end % end public properties
    
    properties (Access = private)
        P_matrix;             % The P matrix, I(n) - ones(n)
        N;                    % Number of robots
        phi;                  % N x 1 matrix of speed phase angles
        time_step = 1/15;     % Default time step between commands
        time = 0;             % Current running time
        phi_last;             % Last set of speed phases 
        theta_state;          % Control for heading alignments
        collisions;           % Collision control enabled or disabled 
        coeff;                % Heading control coefficient
    end % end private properties
       
    methods (Access = public)
        
%************************************************************************
%  Object contructor. Initialize object properties.
%************************************************************************
        function obj = oscillatingFish(initial_poses, varargin)
            
            p = inputParser;
            
            defaultState = 'sync';
            expected_states = {'sync', 'splay', 'none'};
            defaultCollision_Avoidance = false;
            
            addRequired(p, 'initial_poses', @(x) ismatrix(x) && isnumeric(x) && (size(x, 2)==4));
            addOptional(p, 'headings', defaultState, @(x) any(validatestring(x, expected_states)));
            addOptional(p, 'collision_avoidance', defaultCollision_Avoidance, @islogical);
            
            parse(p, initial_poses, varargin{:});           
            obj.initial_poses = p.Results.initial_poses;
            obj.N = size(obj.initial_poses, 1);
            obj.P_matrix = eye(obj.N) - 1/obj.N*ones(obj.N);
            obj.phi = zeros(obj.N, 1);
            obj.theta_state = p.Results.headings;
            obj.collisions = p.Results.collision_avoidance;
            
            % Set sign of gradient control based on goal equilibrium
            % If the coefficient is > 0 the splay state is stabilized.
            % If it is > 0 the synchronized state is stabilized.
            if strcmp(obj.theta_state,'splay')    
                obj.coeff = -1;
            elseif strcmp(obj.theta_state,'sync') 
                obj.coeff = 1;
            else
                obj.coeff = 0;
            end
            
        end % end constructor
        
%************************************************************************
        function [ commands ] = fishControlLaw(obj, ~, states)
            % fishControlLaw Main for oscillating fish control law
            %
            % SYNOPSIS  Computes velocity and steering commands 
            %
            % INPUT     obj: the object, current, states, system time
            %
            % OUTPUT    N x 3 commands vector
            
            %dt = t - obj.time; 
            %obj.time = t;
            dt = obj.time_step;
              
            commands = zeros(obj.N, 3);     % init. command matrix as 0's
            
            % Convert units obj states (e.g. m -> cm)
            states(:,1) = states(:,1)*obj.scale;
            states(:,2) = states(:,2)*obj.scale;
            states(:,4) = states(:,4)*obj.scale;
            
            % Compute particle dynamics
            E = obj.createE(obj.phi);       % create ellipse matrix E(phi)
            r = obj.createR(states);        % create complex vector matrix
            s = obj.createS(r, E, states);  % create S matrix
                         
            % loop to set commands for robots
            for j = 1:obj.N
                
                phi_j = obj.phi(j, 1);  % Get current speed phase
                
                % compute forward speed (u_x) and turning rate (u_theta) 
                u_x = obj.forwardControl(phi_j);
                u_theta = obj.headingControl(states, s, obj.coeff, j); 
                                
                commands(j,1) = u_x/obj.scale;
                commands(j,2) = u_theta;
                
                obj.phi_last = obj.phi;
                obj.updatePhi(u_theta, phi_j, dt, j); % set new speed phase
                
            end % end for loop
        end % end control law
        
 %************************************************************************       
        function [] = simulate(obj, runTime, varargin)
            % SIMULATE Simulate robot motion
            %
            % SYNOPSIS Built in simulator. Same basic function as Miabots.m
            %          simulate method. No max speed cutoff or differential
            %          drive control.
            %
            % INPUT    obj: the object
            %          runTime: Number of seconds to simulate for
            %          Animation options: 
            %                'animate' - logical. Moving dots trace out path. 
            %                'animation_speed' - numerical. Default: 0.2.
            %          Graph options: 
            %                'headings', 'phases', 'trajectory', 'ellipse','graph_all'
            %                logical. User selects which plots to display.
            %                'graph_all' overides and displays all graphs.
            %
            % OUTPUT   None
            
            close all
            clear trajectory current_position
            
            % Check input arguments, and set options
            p = inputParser;
            defaultAnimate = false;
            defaultAnimationSpeed = 0.2;
            
            defaultHeadings = false;
            defaultPhases = false;
            defaultTrajectory = true;
            defaultEllipseLocus = false;
            defaultGraphAll = false; 
            
            addRequired(p, 'obj', @isobject);
            addRequired(p, 'runTime', @isnumeric);
            addOptional(p, 'animate', defaultAnimate, @islogical);
            addOptional(p, 'animation_speed', defaultAnimationSpeed, @isnumeric);
            addOptional(p, 'headings', defaultHeadings, @islogical);
            addOptional(p, 'phases', defaultPhases, @islogical);
            addOptional(p, 'trajectory', defaultTrajectory, @islogical);
            addOptional(p, 'ellipse', defaultEllipseLocus, @islogical);
            addOptional(p, 'graph_all', defaultGraphAll, @islogical);
            
            parse(p, obj, runTime, varargin{:});
            
            animate = p.Results.animate;
            animationSpeed = p.Results.animation_speed;
            graphHeadings = p.Results.headings;
            graphPhases = p.Results.phases;
            graphTrajectory = p.Results.trajectory;
            graphEllipseLocus = p.Results.ellipse;
            graphAll = p.Results.graph_all;
            
            if graphAll
                [graphHeadings, graphPhases, graphTrajectory, graphEllipseLocus]...
                    = deal(true);
            end
                 
            % initialize states
            states = zeros(obj.N, 7);
            states(:, 1) = obj.initial_poses(:, 1);
            states(:, 2) = obj.initial_poses(:, 2);
            states(:, 3) = obj.initial_poses(:, 3);
            states(:, 6) = obj.initial_poses(:, 4);
            
            % keep track obj robot position history
            [x_history, y_history, theta_history, phi_history] ...
                = deal(zeros(runTime + 1, obj.N));
            
            for robot = 1:obj.N
                x_history(1, robot) = obj.initial_poses(robot, 1);
                y_history(1, robot) = obj.initial_poses(robot, 2);
                theta_history(1, robot) = obj.initial_poses(robot, 4);
            end % end initial positions loop
            
            for t = 1:floor(runTime/obj.time_step)
                x_old = states(:, 1);
                y_old = states(:, 2);
                theta_old = states(:, 6);
                
                % Get control law
                commands = obj.fishControlLaw(t, states);
                
                vx = commands(:, 1);
                utheta = commands(:, 2);
                
                % Compute new positions
                theta_new = wrapToPi(theta_old + utheta*obj.time_step);
                [dx, dy] = deal(zeros(obj.N, 1));
                
                for robot = 1:obj.N
                    if utheta(robot) == 0
                        dx(robot) = vx(robot)*cos(theta_old(robot))*obj.time_step;
                        dy(robot) = vx(robot)*sin(theta_old(robot))*obj.time_step;
                    else
                        dx(robot) = vx(robot)/utheta(robot) ...
                            *(sin(theta_new(robot)) - sin(theta_old(robot)));
                        dy(robot) = -vx(robot)/utheta(robot) ...
                            *(cos(theta_new(robot)) - cos(theta_old(robot)));
                    end 
                end % end new positions loop
                
                x_new = x_old + dx;
                y_new = y_old + dy;
                
                % Update states
                states(:, 1) = x_new;
                states(:, 2) = y_new;
                states(:, 4) = vx;
                states(:, 6) = theta_new;
                states(:, 7) = utheta;
   
                % Add to history
                for robot = 1:obj.N
                    x_history(t + 1, robot) = x_new(robot);
                    y_history(t + 1, robot) = y_new(robot);
                    theta_history(t + 1, robot) = theta_new(robot);
                    phi_history(t + 1, robot) = 1 + obj.mu*cos(obj.phi(robot));
                end % history update loop
                
            end % end runtime loop
            
            % Plot trajectories
            if graphHeadings
                figure
                plot(theta_history);
                ylabel('Heading Angle (radians)');
                xlabel('Time Steps');
                legend('Robot 1', 'Robot 2');
                title('Robot Heading Angle History');
            end
            
            if graphPhases
                figure
                plot(phi_history);
                ylabel('Speed Phase (radians)');
                xlabel('Time Steps');
                legend('Robot 1', 'Robot 2');
                title('Robot Speed Phase History');
            end
            
            if graphTrajectory
                figure
                hold on;
                plot(x_history,y_history);
                axis('equal');
                ylabel('Y (meters)');
                xlabel('X (meters)');
                legend('Robot 1', 'Robot 2');
                title('Robot Trajectories');
            end
         
            % Animation obj trajectories
            if animate
                current_position = zeros(robot);
                for robot = 1:obj.N
                    current_position(robot) = plot(x_history(1, robot), ...
                        y_history(1, robot),'Marker','.','markersize', 20);
                end
                
                for i = 1:runTime/obj.time_step
                    for robot = 1:obj.N
                        set(current_position(robot), 'xdata', x_history(i, robot), ...
                            'ydata', y_history(i, robot));
                    end             
                    pause(animationSpeed)
                end
                
            end % end animation
            
            % Graph ellipse locus and overall circle
            if graphEllipseLocus

                r = obj.createR(states);
                E = (1/obj.scale).*obj.createE(obj.phi_last);
                
                % Get average center and ellipse locus
                center = 0;
                [x, y] = deal(zeros(floor(2*pi/0.1 + 1), obj.N));
                
                for robot = 1:obj.N
                    theta = states(robot, 6);
                    center  = center + (r(robot) - (1)*obj.mu*exp(1i*theta)*E(robot) + ...
                              (1/obj.scale)*1i/obj.omega*exp(1i*theta))/obj.N;
                    
                    for phase = floor(0:2*pi/.01);
                        ellipse = (1/obj.scale)*1/(obj.Omega^2 - obj.omega^2) ...
                            .*(obj.Omega.*sin(phase*.01) + 1i.*obj.omega.*cos(phase*.01));
                        x(phase + 1, robot) = real(r(robot) - obj.mu*exp(1i*theta)*E(robot) ...
                            + obj.mu*exp(1i*theta)*ellipse);
                        y(phase + 1, robot) = imag(r(robot) - obj.mu*exp(1i*theta)*E(robot) ...
                            + obj.mu*exp(1i*theta)*ellipse);
                    end
                end
                
                % Get last full orbit
                revolutionT = floor(2*pi/obj.omega/obj.time_step);
                if runTime > revolutionT
                    start = floor(runTime/obj.time_step - revolutionT);
                else start = 1;
                end
                endPoint = floor(runTime/obj.time_step + 1);
                last_x = x_history(start:endPoint, :);
                last_y = y_history(start:endPoint, :);
                
                % Plot last full orbits, greater circle and circle center,
                % ellipses. 
                figure
                hold on;
                plot(last_x, last_y);                
                for robot = 1:obj.N
                    plot(x(:, robot), y(:, robot),'--r');
                end

                for robot = 1:obj.N
                    current_position(robot) = plot(x_history(endPoint, robot), ...
                        y_history(endPoint, robot),'Marker','.','markersize', 20);   
                end
                
                R = zeros(ceil(2*pi/.01), 2);
                for heading = ceil(0:2*pi/.01);
                    R(heading + 1, 1) = real(center - (1/obj.scale)*1i/obj.omega*exp(1i*heading*.01));
                    R(heading + 1, 2) = imag(center - (1/obj.scale)*1i/obj.omega*exp(1i*heading*.01));
                end
                
                plot(R(:,1), R(:,2),'--r');
                plot(real(center), imag(center),'Marker','.','markersize', 20, 'color', 'r');
                axis('equal');
                
            end % end graph ellipse locus

        end % end simulate        
 %************************************************************************          
        function [] = demoConditions(obj, runTime, varargin)
            % DEMOCONDITIONS Setup demonstration conditions
            %
            % SYNOPSIS Initializes object properties for good
            %          demonstration conditions for use in current DCSL lab
            %          setup.             
            %
            % INPUT    obj: the object
            %          runTime: Number of seconds to demo for
            %          'n_robots' - Change number of robots. Current limited
            %                       to 2 or 3 robots.
            %          'headings' - 'sync' or 'splay'. Currently 3 robots 
            %                       only supports splay state. 
            %
            % OUTPUT   None
            
            clear obj.N obj.phi obj.initial_poses obj.P_matrix obj.theta_state
            
            % Check input arguments, and set options
            p = inputParser;
            defaultN = obj.N;
            defaultState = 'sync';
            expected_states = {'sync', 'splay'};
            
            addRequired(p, 'obj', @isobject);
            addRequired(p, 'runTime', @isnumeric);
            addOptional(p, 'n_robots', defaultN, @isnumeric);
            addOptional(p, 'headings', defaultState, @(x) any(validatestring(x, expected_states)));
            
            parse(p, obj, runTime, varargin{:});
            obj.N = p.Results.n_robots;
            obj.phi = zeros(obj.N);
            obj.initial_poses = zeros(obj.N, 4);
            obj.P_matrix = eye(obj.N) - 1/obj.N*ones(obj.N);
            obj.theta_state = p.Results.headings;
            
            % 2 robots, synchronized state, oscillation
            if obj.N == 2 && strcmp(obj.theta_state, 'sync')
                obj.phi(1) = -.2137;
                obj.phi(2) = 2.8722;
                obj.scale = 6;
                obj.mu = .5;
                obj.omega = 0.8;
                obj.Omega = 0.8*1.5;
                obj.coeff = 1;
                obj.initial_poses = [.0503 -.5130 0 -.2331; .0682 -.6709 0 -.2244];
            end
            
            % 2 robots, splay state, no oscillation
            if obj.N == 2 && strcmp(obj.theta_state, 'splay')
                obj.phi(1) = 0;
                obj.phi(2) = 0;
                obj.scale = 6;
                obj.mu = 0;
                obj.omega = 0.8;
                obj.Omega = 0.8*1.5;
                obj.coeff = -1;
                obj.initial_poses = [-.25 -.5 0 0; .25 -.5 0 0];
            end
            
            % 3 robots, splay state, no oscillation
            if obj.N == 3 && strcmp(obj.theta_state, 'splay')
                obj.phi(1) = 0;
                obj.phi(2) = 0;
                obj.scale = 6;
                obj.mu = 0;
                obj.omega = 0.8;
                obj.Omega = 0.8*1.5;
                obj.coeff = -1;
                obj.initial_poses = [-.25 -.5 0 -pi/2; .25 -.5 0 pi/2; 0 -.25 0 pi];
            end
        end
        
    end % end  public methods
    
    methods (Access = private)
        
%************************************************************************
        function [ E_matrix ] = createE(obj, phi)
            % createE  E-matrix constructor method
            %
            % SYNOPSIS Defines elliptical locus of particle positions
            %          about current spot on greater circular orbit.             
            %
            % INPUT    obj: the object
            %          phi: The N x 1 matrix of robot speed phase angles
            %
            % OUTPUT   N x 1 matrix. E(phi_j), j = 1 to N 
            
            E_matrix = zeros(obj.N, 1);
            for j = 1:obj.N
                phi_j = phi(j, 1);
                E_matrix(j, 1) = 1/(obj.Omega^2 - obj.omega^2) ...
                    .*(obj.Omega.*sin(phi_j) + 1i.*obj.omega.*cos(phi_j));      
            end 
            
        end % end createE
        
%************************************************************************
        function [ R_matrix ] = createR(obj, states)
            % createR  R-matrix constructor method
            %
            % SYNOPSIS Converts cartesian input states to complex plane
            %          coordinates.             
            %
            % INPUT    obj: the object
            %          states: N x 7 matrix of current conditions
            %
            % OUTPUT   Returns a length N vector of complex positions.
            
            R_matrix = zeros(obj.N, 1);
            for j = 1:obj.N
                x = states(j, 1);
                y = states(j, 2);
                R_matrix(j, 1) = x + 1i.*y;      
            end 
            
        end % end createR
        
%************************************************************************       
        function [ s_matrix ] = createS(obj, R, E, states)
            % createS  S-matrix constructor method
            %
            % SYNOPSIS Computes and stores components of the s-matrix             
            %
            % INPUT    obj: the object
            %          R: N length vector of complex plane coordinates
            %          E: N x 1 E-matrix
            %          states: N x 7 matrix of current conditions
            %
            % OUTPUT   Returns a length N vector. nth component is iwc(n),
            %          c(n) being the current trajectory center.
            
            s_matrix = zeros(obj.N, 1);
            
            for j = 1:obj.N
                a = exp(1i.*states(j, 6));
                b = 1i.*obj.omega.*R(j, 1);
                c = 1i.*obj.omega*obj.mu.*exp(1i*states(j, 6)).*E(j, 1);
                s_matrix(j, 1) = a - b + c;   
            end
        end % end createS
        
%************************************************************************
        function [ u_x] = forwardControl(obj,phi_j)
            % FORWARDCONTROL forward velocity control
            %
            % SYNOPSIS computes new forward (body frame) velocity             
            %
            % INPUT    obj: the object
            %          phi_j: speed phase of specific robot
            %
            % OUTPUT   Returns a scalar.
            
            u_x = 1 + obj.mu*cos(phi_j);
            
        end
 
%************************************************************************      
        function [ u_theta ] = headingControl(obj, states, s_matrix, coeff, j)
            % FORWARDCONTROL steering (heading) control
            %
            % SYNOPSIS computes new angular velocity             
            %
            % INPUT    obj: the object
            %          states: N x 7 matrix of current conditions
            %          s_matrix: N x 1 s-matrix
            %          coeff: gradient control coefficient
            %          j: specific robot index
            %
            % OUTPUT   Returns a scalar.

            a = obj.P_matrix(j, :)*s_matrix;
            b = 1i*exp(1i*states(j, 6));
            p_theta = obj.orderParameter(states(:,6));
            dU_dtheta = coeff*3*real(p_theta'*1i*exp(1i*states(j, 6)));
            
            u_theta = obj.omega - obj.k*real(a'*b) + dU_dtheta;
            
        end % end headingControl
        
%************************************************************************          
        function [] = updatePhi(obj, u_theta, phi_j, dt, index)
            % UPDATEPHI speed phase control
            %
            % SYNOPSIS computes new speed phase             
            %
            % INPUT    obj: the object
            %          u_theta: current steering control (rad/s)
            %          phi_j: specific robot speed phase
            %          dt: command time step
            %          index: specific robot index
            %          
            %
            % OUTPUT   None.
           
            p1_phi = obj.orderParameter(obj.phi);
            dU1_dphi = real(p1_phi'*1i*exp(1i*phi_j));
            
%            % Not sure why this part of the control doesn't seem to work.            
%
%             dU_dphi = 0; 
%             for m = 1:floor(obj.N)
%                 p2_phi = obj.orderParameter(m*obj.phi);
%                 dU_dphi = dU_dphi + ...
%                     2*obj.k_phi/m*real(p2_phi'*1i*m/obj.N*exp(1i*m*phi_j));
%             end
%             
%             phi_j_dot = obj.Omega/obj.omega*u_theta - (dU_dphi - dU1_dphi); 
            
            phi_j_dot = obj.Omega/obj.omega*u_theta - (dU1_dphi);
                   
            % Update phi
            obj.phi(index, 1) = wrapToPi(phi_j + phi_j_dot*dt);
            
        end % end updatePhi

%************************************************************************
        function [ order_parameter] = orderParameter(obj, psi)
            % ORDERPARAMETER Compute complex order parameter
            %
            % SYNOPSIS complex order parameter for given set of angles. 
            %          Returns a complex number, with magnitude and angle =
            %          average of group.             
            %
            % INPUT    obj: the object
            %          psi: set of angles
            %
            % OUTPUT   Returns a complex scalar.
            
            order_parameter = 0;
            for j = 1:obj.N
                order_parameter = order_parameter + 1/obj.N*exp(1i*psi(j));
            end
            
        end % end orderParameter  

%************************************************************************
        function [ neighbor ] = nearestNeighbor(obj, r, j)
            % NEARESTNEIGHBOR Find nearest neighbor
            %
            % SYNOPSIS Find nearest neighbor of given robot.
            %
            % INPUT    obj: the object
            %          r: complex plane coordinates of robots
            %          j: specific robot index
            %
            % OUTPUT   Return 2 x 1 vector containing distance to nearest 
            %          neighbor (1) and the index of the nearest neighbor (2).
            
            minDistance = Inf;
            nearest = 1; 
            for robot = 1:obj.N
                distance = abs(r(j) - r (robot));
                if (distance < minDistance && robot ~= j)
                    nearest = robot;
                    minDistance = distance;
                end
            end
            neighbor(1) = minDistance;
            neighbor(2) = nearest;
        end
             
    end % end private methods
        
end % end class

