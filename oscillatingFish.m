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
    % school = oscillatingFish(initial_positions)
    %
    % INPUTS
    %
    % initial_poses: n_robots X [x y z theta] matrix of the initial
    % positions and headings of the robots.
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
    % theta_state - 'splay', 'sync', or neither sets control law
    % coefficients to stabilize either splayed, synchronized, or arbitrary
    % robot positioning about overall orbit. 
    %
    % METHODS
    %
    % fishControlLaw(OF, t, states) - takes the oscillating fish object,
    % current time, and current states of the robots and returns an N x 3 
    % matrix of commands for new forward velocity, turning rate, and 
    % vertical velocity (always 0).
    %
    % simulate(OF, runTime) - simulates trajectory of robots for specified 
    % runtime (seconds). 
    %
    %**********************************************************************
    
    properties (Access = public)
        omega = 1;           % Natural heading turning frequency
        Omega = 1*2.5;         % Natural speed phase frequency
        mu = 0.5;              % Speed oscillation parameter
        k = 1;                 % Steering control parameter
        k_phi = 1;             % Speed phase control parameter
        scale = 1;             % Commands scaling  1 meter: scale
        theta_state = 'sync';  % Control for heading alignments
        initial_poses;        % initial robot positions
        collision = 0;       
        c;
    end % end public properties
    
    properties (Access = private)
        P_matrix;             % The P matrix, I(n) - ones(n)
        N;                    % Number of robots
        phi;                  % N x 1 matrix of speed phase angles
        
        time_step = 1/7.5;    % Default time step between commands
        time = 0;             % Current running time
        phi_last;
    end % end private properties
       
    methods (Access = public)
        
%************************************************************************
%  Object contructor. Initialize object properties.
%************************************************************************
        function OF = oscillatingFish(initial_poses)
            OF.initial_poses = initial_poses;
            OF.N = size(initial_poses, 1);
            OF.P_matrix = eye(OF.N) - 1/OF.N*ones(OF.N);
            OF.phi = zeros(OF.N, 1);
            
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
                % u_x = (1 + OF.mu*cos(phi_j));  
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

        function [] = simulate(OF, runTime, animate)
            close all
            clear trajectory current_position
            
            % initialize states
            states = zeros(OF.N, 7);
            states(:, 1) = OF.initial_poses(:, 1);
            states(:, 2) = OF.initial_poses(:, 2);
            states(:, 3) = OF.initial_poses(:, 3);
            states(:, 6) = OF.initial_poses(:, 4);
            
            % keep track of robot position history
            x_history = zeros(runTime + 1, OF.N);
            y_history = zeros(runTime + 1, OF.N);
            theta_history = zeros(runTime + 1, OF.N);
            
            phi_history = zeros(runTime+1, OF.N);
            
            for robot = 1:OF.N
                x_history(1, robot) = OF.initial_poses(robot, 1);
                y_history(1, robot) = OF.initial_poses(robot, 2);
                theta_history(1, robot) = OF.initial_poses(robot, 4);
                phi_history(1, robot) = 1+OF.mu*cos(OF.phi(robot));
            end % end initial positions loop
            
            for t = 1:(runTime/OF.time_step)
                x_old = states(:, 1);
                y_old = states(:, 2);
                theta_old = states(:, 6);
                
                % Get control law
                commands = OF.fishControlLaw(t, states);
                vx = commands(:, 1);
                utheta = commands(:, 2);
                
                % Compute new positions
                theta_new = wrapToPi(theta_old + utheta*OF.time_step);
                dx = zeros(OF.N, 1);
                dy = zeros(OF.N, 1);
                
                for robot = 1:OF.N
                    if utheta(robot) == 0
                        dx(robot) = vx(robot)*cos(theta_old(robot))*OF.time_step;
                        dy(robot) = vx(robot)*sin(theta_old(robot))*OF.time_step;
                    else
                        dx(robot) = vx(robot)/utheta(robot) ...
                            *(sin(theta_new(robot)) - sin(theta_old(robot)));
                        dy(robot) = -vx(robot)/utheta(robot) ...
                            *(cos(theta_new(robot)) - cos(theta_old(robot)));
                    end % end if
                end % end loop
                
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
                    phi_history(t + 1, robot) = 1+OF.mu*cos(OF.phi(robot));
                end % history update loop
                
            end % end runtime loop
            
            % Plot trajectories
            plot(theta_history);
            figure
            plot(phi_history);
            figure
            hold on;
            plot(x_history,y_history);
            axis('equal');
            
            current_position = zeros(robot);
            %Animate
            
            if strcmp(animate, 'animate')
            for robot = 1:OF.N
                current_position(robot) = plot(x_history(1, robot), ...
                    y_history(1, robot),'Marker','.','markersize', 20);
            end
            
            for i = 1:runTime/OF.time_step
                for robot = 1:OF.N
                    set(current_position(robot), 'xdata', x_history(i, robot), ...
                        'ydata', y_history(i, robot));
                end
                
                pause(.2)
            end
            end
            
            %*************************************************************
            
            % Get average center  
            r = OF.createR(states);
            E = OF.createE(OF.phi_last);
            center = 0;
            for robot = 1:OF.N
                theta = states(robot, 6);
                center  = center + (r(robot) - OF.mu*exp(1i*theta)*E(robot) + ...
                    1i/OF.omega*exp(1i*theta))/OF.N;
                
                for phase = 0:2*pi/.01;
                    ellipse = 1/(OF.Omega^2 - OF.omega^2) ...
                        .*(OF.Omega.*sin(phase*.01) + 1i.*OF.omega.*cos(phase*.01));
                    x(phase + 1, robot) = real(r(robot) - OF.mu*exp(1i*theta)*E(robot) ...
                        + OF.mu*exp(1i*theta)*ellipse);
                    y(phase + 1, robot) = imag(r(robot) - OF.mu*exp(1i*theta)*E(robot) ...
                        + OF.mu*exp(1i*theta)*ellipse);
                end
            end
            
      
           revolutionT = floor(2*pi/OF.omega/OF.time_step);

            start = runTime/OF.time_step - revolutionT;
            endP = runTime/OF.time_step + 1;
           last_x = x_history(start:endP, :);
           last_y = y_history(start:endP, :);
           
           figure
           hold on;
           
           plot(last_x, last_y);
           
           for robot = 1:OF.N
            plot(x(:, robot), y(:, robot),'r');
           end
           
           
      
            for robot = 1:OF.N
                current_position(robot) = plot(x_history(endP, robot), ...
                    y_history(endP, robot),'Marker','.','markersize', 20);
            end
            
            for v = 0:2*pi/.01;
              
            R(v + 1, 1) = real(center -1i/OF.omega*exp(1i*v*.01));
            R(v + 1, 2) = imag(center -1i/OF.omega*exp(1i*v*.01));
            end

           
            plot(R(:,1), R(:,2),'g');

            plot(real(center), imag(center),'Marker','.','markersize', 20, 'color', 'g');
            thetas = theta_history(endP, :)
            
            
            E = OF.createE(OF.phi);       % create ellipse matrix E(phi)
            r = OF.createR(states);       % create complex vector matrix
            s = OF.createS(r, E, states); % create S matrix
            
            abs(OF.P_matrix(1, :)*s)
            abs(OF.P_matrix(2, :)*s)
            %abs(OF.P_matrix(3, :)*s)
            
            commands = OF.fishControlLaw(t, states)
        end % end simulate
        
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
            
            nearestNeighbor = OF.nearestNeighbor(r, j);
            distanceTo = nearestNeighbor(1);
            theta_j = states(j, 6);
            theta_near = states(nearestNeighbor(2), 6);
            
            u_x = 1 + OF.mu*cos(phi_j) + ...
                OF.collision*(1 + cos(theta_j - theta_near))/distanceTo;   
        end
 
%************************************************************************
%  Helper method to construct steering (heading) control
%************************************************************************        
        function [ u_theta ] = headingControl(OF, states, s_matrix, coeff, r, j)

            a = OF.P_matrix(j, :)*s_matrix;
            b = 1i*exp(1i*states(j, 6));
            p_theta = OF.orderParameter(states(:,6));
            dU_dtheta = coeff*3*real(p_theta'*1i*exp(1i*states(j, 6)));
            
            nearestNeighbor = OF.nearestNeighbor(r, j);
            neighbor = nearestNeighbor(2);
            avoidance = 0*sin((-states(j, 6) + angle(r(j) - r(neighbor)))/2);
            
            u_theta = OF.omega - OF.k*real(a'*b) + dU_dtheta ...
                      + avoidance/nearestNeighbor(1)^2;
            
        end % end headingControl
        
%************************************************************************
%  Helper method to construct speed phase control
%************************************************************************           
        function [] = updatePhi(OF, u_theta, phi_j, dt, index)
           
            p1_phi = OF.orderParameter(OF.phi);
            dU1_dphi = real(p1_phi'*1i*exp(1i*phi_j));
            
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
            OF.phi(index, 1) = (phi_j + phi_j_dot*dt);
            
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
%  Find nearest neighbor of given robot
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

