classdef oscillatingFish < handle
    
    properties
        omega = .75;        % Natural heading turning frequency
        Omega = .5*(pi);     % Natural speed phase frequency
        mu = 0.5;           % Speed oscillation parameter
        k = 1;            % Steering control parameter
        time_step = 1/15;   % Discrete time step between commands
        initial_poses;      % initial robot positions
        scale = 5;         % Commands scaling  1 meter: scale
        phi;
        last;
    end % end public properties
    
    properties (Access = private)
        P_matrix;           % The P matrix, I(n) - ones(n)
        N;                  % Number of robots
       % phi;                % N x 1 matrix of speed phase angles
        R0 = 0;
    end % end private properties
    
    methods
        
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
            
            commands = zeros(OF.N, 3);    % init. command matrix as 0's
            
            % Convert units of states (e.g. m -> cm)
            states(:,1) = states(:,1)*OF.scale;
            states(:,2) = states(:,2)*OF.scale;
            states(:,4) = states(:,4)*OF.scale;
            
            % Compute particle dynamics
            E = OF.createE();             % create ellipse matrix E(phi)
            r = OF.createR(states);       % create complex vector matrix
            %s = OF.createS(r, E, states); % create S matrix
            
            % loop to set positions for robots
            for j = 1:OF.N
                
                phi_j = OF.phi(j, 1);         % Get current speed phase
                u_x = (1 + OF.mu*cos(phi_j)); % Compute new forward speed
                
                % a = OF.P_matrix(j, :)*s;
                % b = 1i*exp(1i*states(j, 6));
                % u_theta = OF.omega - OF.k*real(a'*b);  % Heading control
                
                u_theta = OF.thetaControl(r, E, states, j);
                
                
                p_phi = OF.OrderParameter(OF.phi);
                a = -1*real(p_phi'*1i*exp(1i*phi_j));
                phi_j_dot = OF.Omega/OF.omega*u_theta + a; % Phi control
                
                
                % Update phi, send new commands
                OF.phi(j, 1) = phi_j + phi_j_dot*OF.time_step;
                commands(j,1) = u_x/OF.scale;
                commands(j,2) = u_theta;
            end % end for loop
        end % end control law
        
%************************************************************************
%  E matrix constructor. Defines elliptical locus of particle positions
%  about current spot on greater circular orbit.
%************************************************************************
        function [ E_matrix ] = createE(OF)
            
            E_matrix = zeros(OF.N, 1);
            for j = 1:OF.N
                phi_j = OF.phi(j, 1);
                E_matrix(j, 1) = 1/(OF.Omega^2 - OF.omega^2) ...
                    .*(OF.Omega.*sin(phi_j) + 1i.*OF.omega.*cos(phi_j));
            end % end loop
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
            end % end for loop
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
            end % end for loop
        end % end createS
        
%************************************************************************
%  Theta control law.
%************************************************************************
        function [ u_theta ] = thetaControl(OF, R, E, states, j)
            thetas = states(:, 6);
            p_theta = OF.OrderParameter(thetas);
            
            %r_tilde = OF.P_matrix(j, :)*R;
            r_tilde = R(j) - OF.R0; 
            e = OF.getE(E, states);
            e_tilde = OF.P_matrix(j, :)*e;            
            
            a = OF.k*real(p_theta'*1i*exp(1i*states(j, 6)));
            
            b = OF.omega*(1 + OF.k*real(r_tilde'*exp(1i*states(j, 6))));
            
            c = OF.mu*OF.k*OF.omega*real(e_tilde'*exp(1i*states(j, 6)));
            
            u_theta = a + b + c;
        end
        
        function [ order_parameter] = OrderParameter(OF, psi)
            order_parameter = 0;
            for j = 1:OF.N
                order_parameter = order_parameter + 1/OF.N*exp(1i*psi(j));
            end
        end
        
        function [ e ] = getE(OF, E, states)
            e = zeros(OF.N, 1);
            for j = 1:OF.N;
                e(j) = exp(1i*states(j, 6))*E(j);
            end         
        end
        
        
        
%************************************************************************
%  Built in simulator. Same basic function as Miabots.m simulate method.
%  No max speed cutoff or differential drive control.
%************************************************************************
        function [] = simulate(OF, runTime)
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
            
            for robot = 1:OF.N
                x_history(1, robot) = OF.initial_poses(robot, 1);
                y_history(1, robot) = OF.initial_poses(robot, 2);
                theta_history(1, robot) = OF.initial_poses(robot, 4);
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
                OF.last = states;
                
                % Add to history
                for robot = 1:OF.N
                    x_history(t + 1, robot) = x_new(robot);
                    y_history(t + 1, robot) = y_new(robot);
                    theta_history(t + 1, robot) = theta_new(robot);
                end % history update loop
                
            end % end runtime loop
            
            % Plot trajectories
            plot(theta_history);
            figure
            plot(0,0,'Marker','.','markersize',30);
            hold on;
            plot(x_history,y_history);
            center = plot(real(OF.R0),imag(OF.R0),'Marker','.','markersize',20);
            
            axis('equal');
            
            current_position = zeros(robot);
            % Animate
            for robot = 1:OF.N
                current_position(robot) = plot(x_history(1, robot), ...
                    y_history(1, robot),'Marker','.','markersize', 20);
            end
            
            for i = 1:runTime/OF.time_step
                for robot = 1:OF.N
                    set(current_position(robot), 'xdata', x_history(i, robot), ...
                        'ydata', y_history(i, robot));
                end
                
                pause(OF.time_step)
            end
            
        end % end simulate
        
    end % end methods
    
end % end class

