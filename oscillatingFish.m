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
    % fishControlLaw(OF, t, states) - takes the oscillating fish object,
    % current time, and current states of the robots and returns an N x 3 
    % matrix of commands for new forward velocity, turning rate, and 
    % vertical velocity (always 0).
    %
    % simulate(OF, runTime, options) - simulates trajectory of robots for specified 
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
    %**********************************************************************
    
    properties (Access = public)
        omega = .8;            % Natural heading turning frequency
        Omega = .8*1.5;        % Natural speed phase frequency
        mu = 0.5;             % Speed oscillation parameter
        k = 1;                % Steering control parameter
        k_phi = 1;            % Speed phase control parameter
        scale = 5;           % Commands scaling  1 meter: scale
        initial_poses;        % initial robot positions

    end % end public properties
    
    properties (Access = private)
        P_matrix;             % The P matrix, I(n) - ones(n)
        N;                    % Number of robots
        phi;                  % N x 1 matrix of speed phase angles
        time_step = 1/15;    % Default time step between commands
        time = 0;             % Current running time
        phi_last;             % Last set of speed phases 
        theta_state;          % Control for heading alignments
        collisions;           % Collision control enabled or disabled
        
    end % end private properties
       
    methods (Access = public)
        
%************************************************************************
%  Object contructor. Initialize object properties.
%************************************************************************
        function OF = oscillatingFish(initial_poses, varargin)
            
            p = inputParser;
            
            defaultState = 'sync';
            expected_states = {'sync', 'splay', 'none'};
            defaultCollision_Avoidance = false;
            
            addRequired(p, 'initial_poses', @(x) ismatrix(x) && isnumeric(x) && (size(x, 2)==4));
            addOptional(p, 'headings', defaultState, @(x) any(validatestring(x, expected_states)));
            addOptional(p, 'collision_avoidance', defaultCollision_Avoidance, @islogical);
            
            parse(p, initial_poses, varargin{:});
            
            OF.initial_poses = p.Results.initial_poses;
            OF.N = size(OF.initial_poses, 1);
            OF.P_matrix = eye(OF.N) - 1/OF.N*ones(OF.N);
            OF.phi = zeros(OF.N, 1);


% three

% OF.phi(1) = -.0346;
% OF.phi(2) = -2.0876;
% OF.phi(3) = 2.0396;

            OF.theta_state = p.Results.headings;
            OF.collisions = p.Results.collision_avoidance;
            
        end % end constructor
        
%************************************************************************
%  Main for oscillating fish control law.
%************************************************************************
        function [ commands ] = fishControlLaw(OF, t, states)
            
            %dt = t - OF.time; 
            %OF.time = t;
            dt = OF.time_step;
              
            commands = zeros(OF.N, 3);    % init. command matrix as 0's
            
            % Convert units of states (e.g. m -> cm)
            states(:,1) = states(:,1)*OF.scale;
            states(:,2) = states(:,2)*OF.scale;
            states(:,4) = states(:,4)*OF.scale;
            
            % Compute particle dynamics
            E = OF.createE(OF.phi);       % create ellipse matrix E(phi)
            r = OF.createR(states);       % create complex vector matrix
            s = OF.createS(r, E, states); % create S matrix
                         
            % Set sign of gradient control based on goal equilibrium
            % If the coefficient is > 0 the splay state is stabilized.
            % If it is > 0 the synchronized state is stabilized.
            if strcmp(OF.theta_state,'splay')    
                coeff = -1;
            elseif strcmp(OF.theta_state,'sync') 
                coeff = 1;
            else
                coeff = 0;
            end

            % loop to set commands for robots
            for j = 1:OF.N
                
                phi_j = OF.phi(j, 1);  % Get current speed phase
                
                % compute forward speed (u_x) and turning rate (u_theta) 
                u_x = OF.forwardControl(states, r, phi_j, j);
                u_theta = OF.headingControl(states, s, coeff, r, j); 
                                
                commands(j,1) = u_x/OF.scale;
                commands(j,2) = u_theta;
                
                OF.phi_last = OF.phi;
                OF.updatePhi(u_theta, phi_j, dt, j); % set new speed phase
                
            end % end for loop
        end % end control law
        

%************************************************************************
%  Built in simulator. Same basic function as Miabots.m simulate method.
%  No max speed cutoff or differential drive control.
%************************************************************************

        function [] = simulate(OF, runTime, varargin)
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
            
            addRequired(p, 'OF', @isobject);
            addRequired(p, 'runTime', @isnumeric);
            addOptional(p, 'animate', defaultAnimate, @islogical);
            addOptional(p, 'animation_speed', defaultAnimationSpeed, @isnumeric);
            addOptional(p, 'headings', defaultHeadings, @islogical);
            addOptional(p, 'phases', defaultPhases, @islogical);
            addOptional(p, 'trajectory', defaultTrajectory, @islogical);
            addOptional(p, 'ellipse', defaultEllipseLocus, @islogical);
            addOptional(p, 'graph_all', defaultGraphAll, @islogical);
            
            parse(p, OF, runTime, varargin{:});
            
            animate = p.Results.animate;
            animationSpeed = p.Results.animation_speed;
            graphHeadings = p.Results.headings;
            graphPhases = p.Results.phases;
            graphTrajectory = p.Results.trajectory;
            graphEllipseLocus = p.Results.ellipse;
            graphAll = p.Results.graph_all;
            
            if graphAll
                [graphHeadings, graphPhases, graphTrajectory, graphEllipseLocus] = deal(true);
            end
            
                
            % initialize states
            states = zeros(OF.N, 7);
            states(:, 1) = OF.initial_poses(:, 1);
            states(:, 2) = OF.initial_poses(:, 2);
            states(:, 3) = OF.initial_poses(:, 3);
            states(:, 6) = OF.initial_poses(:, 4);
            
            % keep track of robot position history
            [x_history, y_history, theta_history, phi_history] ...
                                          = deal(zeros(runTime + 1, OF.N));
            
            for robot = 1:OF.N
                x_history(1, robot) = OF.initial_poses(robot, 1);
                y_history(1, robot) = OF.initial_poses(robot, 2);
                theta_history(1, robot) = OF.initial_poses(robot, 4);
            end % end initial positions loop
            
            for t = 1:floor(runTime/OF.time_step)
                x_old = states(:, 1);
                y_old = states(:, 2);
                theta_old = states(:, 6);
                
                % Get control law
                commands = OF.fishControlLaw(t, states);
                
                vx = commands(:, 1);
                utheta = commands(:, 2);
                
                % Compute new positions
                theta_new = wrapToPi(theta_old + utheta*OF.time_step);
                [dx, dy] = deal(zeros(OF.N, 1));
                
                for robot = 1:OF.N
                    if utheta(robot) == 0
                        dx(robot) = vx(robot)*cos(theta_old(robot))*OF.time_step;
                        dy(robot) = vx(robot)*sin(theta_old(robot))*OF.time_step;
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
                for robot = 1:OF.N
                    x_history(t + 1, robot) = x_new(robot);
                    y_history(t + 1, robot) = y_new(robot);
                    theta_history(t + 1, robot) = theta_new(robot);
                    phi_history(t + 1, robot) = 1 + OF.mu*cos(OF.phi(robot));
                end % history update loop
                
            end % end runtime loop
            
            % Plot trajectories
            if graphHeadings
                figure
                plot(theta_history);
            end
            
            if graphPhases
                figure
                plot(phi_history);
            end
            
            if graphTrajectory
                figure
                hold on;
                plot(x_history,y_history);
                axis('equal');
            end
            
            
            % Animation of trajectories
            if animate
                current_position = zeros(robot);
                for robot = 1:OF.N
                    current_position(robot) = plot(x_history(1, robot), ...
                        y_history(1, robot),'Marker','.','markersize', 20);
                end
                
                for i = 1:runTime/OF.time_step
                    for robot = 1:OF.N
                        set(current_position(robot), 'xdata', x_history(i, robot), ...
                            'ydata', y_history(i, robot));
                    end
                    
                    pause(animationSpeed)
                end
                
            end % end animation
            
            % Graph ellipse locus and overall circle
            if graphEllipseLocus

                r = OF.createR(states);
                E = (1/OF.scale).*OF.createE(OF.phi_last);
                
                % Get average center and ellipse locus
                center = 0;
                [x, y] = deal(zeros(floor(2*pi/0.1 + 1), OF.N));
                
                for robot = 1:OF.N
                    theta = states(robot, 6);
                    center  = center + (r(robot) - (1)*OF.mu*exp(1i*theta)*E(robot) + ...
                              (1/OF.scale)*1i/OF.omega*exp(1i*theta))/OF.N;
                    
                    for phase = floor(0:2*pi/.01);
                        ellipse = (1/OF.scale)*1/(OF.Omega^2 - OF.omega^2) ...
                            .*(OF.Omega.*sin(phase*.01) + 1i.*OF.omega.*cos(phase*.01));
                        x(phase + 1, robot) = real(r(robot) - OF.mu*exp(1i*theta)*E(robot) ...
                            + OF.mu*exp(1i*theta)*ellipse);
                        y(phase + 1, robot) = imag(r(robot) - OF.mu*exp(1i*theta)*E(robot) ...
                            + OF.mu*exp(1i*theta)*ellipse);
                    end
                end
                
                % Get last full orbit
                revolutionT = floor(2*pi/OF.omega/OF.time_step);
                start = floor(runTime/OF.time_step - revolutionT);
                endPoint = floor(runTime/OF.time_step + 1);
                last_x = x_history(start:endPoint, :);
                last_y = y_history(start:endPoint, :);
                
                % Plot last full orbits, greater circle and circle center,
                % ellipses. 
                figure
                hold on;
                plot(last_x, last_y);                
                for robot = 1:OF.N
                    plot(x(:, robot), y(:, robot),'--r');
                end

                for robot = 1:OF.N
                    current_position(robot) = plot(x_history(endPoint, robot), ...
                        y_history(endPoint, robot),'Marker','.','markersize', 20);
                    disp(x_history(endPoint, robot));
                    disp(y_history(endPoint, robot));
                    disp(theta_history(endPoint, robot));
                    disp(OF.phi_last(robot));
                    
                end
                
                for heading = 0:2*pi/.01;
                    R(heading + 1, 1) = real(center - (1/OF.scale)*1i/OF.omega*exp(1i*heading*.01));
                    R(heading + 1, 2) = imag(center - (1/OF.scale)*1i/OF.omega*exp(1i*heading*.01));
                end
                
                plot(R(:,1), R(:,2),'--r');
                plot(real(center), imag(center),'Marker','.','markersize', 20, 'color', 'r');
                axis('equal');
                
            end % end graph ellipse locus

        end % end simulate
        
        function [] = miabotSim(OF, runTime, varargin)
            
            % Check input arguments, and set options
            p = inputParser;
            defaultAnimate = false;
            defaultAnimationSpeed = 0.2;
            
            addRequired(p, 'OF', @isobject);
            addRequired(p, 'runTime', @isnumeric);
            addOptional(p, 'animate', defaultAnimate, @islogical);
            addOptional(p, 'animation_speed', defaultAnimationSpeed, @isnumeric);     
            parse(p, OF, runTime, varargin{:});        
            animate = p.Results.animate;
            animationSpeed = p.Results.animation_speed;

            % initialize states
            states = zeros(OF.N, 7);
            states(:, 1) = OF.initial_poses(:, 1);
            states(:, 2) = OF.initial_poses(:, 2);
            states(:, 3) = OF.initial_poses(:, 3);
            states(:, 6) = OF.initial_poses(:, 4);
            
            % keep track of robot position history
            [x_history, y_history, theta_history, phi_history] ...
                                          = deal(zeros(runTime + 1, OF.N));
            
            for robot = 1:OF.N
                x_history(1, robot) = OF.initial_poses(robot, 1);
                y_history(1, robot) = OF.initial_poses(robot, 2);
                theta_history(1, robot) = OF.initial_poses(robot, 4);
            end % end initial positions loop
            
            for t = 1:floor(runTime/OF.time_step)
                
                % Get control law
                commands = OF.fishControlLaw(t, states);
                
                [output, crap] = propagate(OF, states, commands, 1/15, [0 0 0 0], OF.N); 
                states = output;
                
                % Add to history
                for robot = 1:OF.N
                    x_history(t + 1, robot) = states(robot, 1);
                    y_history(t + 1, robot) = states(robot, 2);
                    theta_history(t + 1, robot) = states(robot, 4);
                    phi_history(t + 1, robot) = 1 + OF.mu*cos(OF.phi(robot));
                end % history update loop
                
                
            end % end runtime loop
            
            % Plot trajectories

            figure
            plot(theta_history);
                      
            figure
            plot(phi_history);
            
            figure
            hold on;
            plot(x_history,y_history);
            axis('equal');
            
            
            
            % Animation of trajectories
            if animate
                current_position = zeros(robot);
                for robot = 1:OF.N
                    current_position(robot) = plot(x_history(1, robot), ...
                        y_history(1, robot),'Marker','.','markersize', 20);
                end
                
                for i = 1:runTime/OF.time_step
                    for robot = 1:OF.N
                        set(current_position(robot), 'xdata', x_history(i, robot), ...
                            'ydata', y_history(i, robot));
                    end
                    
                    pause(animationSpeed)
                end
                
            end % end animation
        end
        
        function [] = demo(OF, runTime, varargin)
            % Check input arguments, and set options
            p = inputParser;
            defaultN = 2;
            defaultState = 'sync';
            expected_states = {'sync', 'splay'};
            addRequired(p, 'OF', @isobject);
            addRequired(p, 'runTime', @isnumeric);
            addOptional(p, 'n_robots', defaultN, @isnumeric);
            addOptional(p, 'headings', defaultState, @(x) any(validatestring(x, expected_states)));    
            parse(p, OF, runTime, varargin{:});        
            
            OF.N = p.Results.n_robots;
            OF.phi = zeros(OF.N);
            OF.P_matrix = eye(OF.N) - 1/OF.N*ones(OF.N);
            OF.theta_state = p.Results.headings;
            
            if OF.N == 2 && strcmp(OF.theta_state, 'sync')
            OF.phi(1) = -.2137;
            OF.phi(2) = 2.8722;
            OF.scale = 6;
            OF.initial_poses = [.0503 -.5130 0 -.2331; .0682 -.6709 0 -.2244];      
            OF.simulate(runTime);
            control_law = @(t, x) OF.fishControlLaw(t,x);    
            % calls new Miabot object that actuates robot motion
            m = Miabots(OF.initial_poses, control_law, 'velocity', runTime,...
                'sim', true);
            m.start
            end
            
            if OF.N == 2 && strcmp(OF.theta_state, 'splay')
            OF.phi(1) = 0;
            OF.phi(2) = 0;
            OF.scale = 6;
            OF.mu = 0;
            OF.initial_poses = [-.25 -.5 0 0; .25 -.5 0 0];      
            OF.simulate(runTime);
            control_law = @(t, x) OF.fishControlLaw(t,x);    
            % calls new Miabot object that actuates robot motion
            m = Miabots(OF.initial_poses, control_law, 'velocity', runTime,...
                'sim', true);
            m.start
            end
            
        end
        
    end % end  public methods
    

    methods (Access = private)
        
%************************************************************************
%  E matrix constructor. Defines elliptical locus of particle positions
%  about current spot on greater circular orbit.
%************************************************************************
        function [ E_matrix ] = createE(OF, phi)
            
            E_matrix = zeros(OF.N, 1);
            for j = 1:OF.N
                phi_j = phi(j, 1);
                E_matrix(j, 1) = 1/(OF.Omega^2 - OF.omega^2) ...
                    .*(OF.Omega.*sin(phi_j) + 1i.*OF.omega.*cos(phi_j));      
            end 
            
        end % end createE
        
%************************************************************************
%  R matrix constructor method. Converts cartesian input to complex plane
%  coordinates. Returns a length N vector of complex positions.
%************************************************************************
        function [ R_matrix ] = createR(OF, states)
            
            R_matrix = zeros(OF.N, 1);
            for j = 1:OF.N
                x = states(j, 1);
                y = states(j, 2);
                R_matrix(j, 1) = x + 1i.*y;      
            end 
            
        end % end createR
        
%************************************************************************
%  S matrix constructor method. Returns a length N vector.
%  nth component is iwc(n), c(n) the current trajectory center.
%************************************************************************
        
        function [ s_matrix ] = createS(OF, R, E, states)
            
            s_matrix = zeros(OF.N, 1);
            
            for j = 1:OF.N
                a = exp(1i.*states(j, 6));
                b = 1i.*OF.omega.*R(j, 1);
                c = 1i.*OF.omega*OF.mu.*exp(1i*states(j, 6)).*E(j, 1);
                s_matrix(j, 1) = a - b + c;   
            end
        end % end createS
        
%************************************************************************
%  Helper method to construct steering (heading) control
%************************************************************************ 

        function [ u_x] = forwardControl(OF, states, r, phi_j, j)
            
            u_x = 1 + OF.mu*cos(phi_j);
            
%             % Collision avoidance stuff. Work in progress. 
%
%             nearestNeighbor = OF.nearestNeighbor(r, j);
%             distanceTo = nearestNeighbor(1);
%             theta_j = states(j, 6);
%             theta_near = states(nearestNeighbor(2), 6);
%             u_x = 1 + OF.mu*cos(phi_j) + ...
%                 OF.collisions*(1 + cos(theta_j - theta_near))/distanceTo;   
        end
 
%************************************************************************
%  Helper method to construct steering (heading) control
%************************************************************************        
        function [ u_theta ] = headingControl(OF, states, s_matrix, coeff, r, j)

            a = OF.P_matrix(j, :)*s_matrix;
            b = 1i*exp(1i*states(j, 6));
            p_theta = OF.orderParameter(states(:,6));
            dU_dtheta = coeff*3*real(p_theta'*1i*exp(1i*states(j, 6)));
            
            u_theta = OF.omega - OF.k*real(a'*b) + dU_dtheta;
            
%             % Collision avoidance stuff. Work in progress.
%             nearestNeighbor = OF.nearestNeighbor(r, j);
%             neighbor = nearestNeighbor(2);
%             avoidance = 0*sin((-states(j, 6) + angle(r(j) - r(neighbor)))/2);
%             u_theta = OF.omega - OF.k*real(a'*b) + dU_dtheta ...
%                       + avoidance/nearestNeighbor(1)^2;
            
        end % end headingControl
        
%************************************************************************
%  Helper method to construct speed phase control
%************************************************************************           
        function [] = updatePhi(OF, u_theta, phi_j, dt, index)
           
            p1_phi = OF.orderParameter(OF.phi);
            dU1_dphi = real(p1_phi'*1i*exp(1i*phi_j));
            
%            % Not sure why this part of the control doesn't seem to work.            
%
%             dU_dphi = 0; 
%             for m = 1:floor(OF.N)
%                 p2_phi = OF.orderParameter(m*OF.phi);
%                 dU_dphi = dU_dphi + ...
%                     2*OF.k_phi/m*real(p2_phi'*1i*m/OF.N*exp(1i*m*phi_j));
%             end
%             
%             phi_j_dot = OF.Omega/OF.omega*u_theta - (dU_dphi - dU1_dphi); 
            
            phi_j_dot = OF.Omega/OF.omega*u_theta - (dU1_dphi);
                   
            % Update phi
            OF.phi(index, 1) = wrapToPi(phi_j + phi_j_dot*dt);
            
        end % end updatePhi

%************************************************************************
%  Compute complex order parameter for given set of angles. Returns a
%  complex number, with magnitude and angle = average of group. 
%************************************************************************

        function [ order_parameter] = orderParameter(OF, psi)
            order_parameter = 0;
            for j = 1:OF.N
                order_parameter = order_parameter + 1/OF.N*exp(1i*psi(j));
            end
            
        end % end orderParameter  

%************************************************************************
%  Find nearest neighbor of given robot. Return 2 x 1 vector containing
%  distance to nearest neighbor and the index of the nearest neighbor.
%************************************************************************
        
        function [ neighbor ] = nearestNeighbor(OF, r, j)
            minDistance = Inf;
            nearest = 1; 
            for robot = 1:OF.N
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

