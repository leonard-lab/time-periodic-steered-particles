classdef oscillatingFish < handle
    
    
    properties
        
        omega = .25;        % Natural heading turning frequency
        Omega = .25*pi;       % Natural speed phase frequency
        mu = 0;               % Speed oscillation parameter
        k = 15;                % Steering control parameter
        time_step = 0.04;       % Discrete time step between commands
        initial_poses;          % initial robot positions
        scale = 50;
        
    end % end public properties
    
    properties (Access = private)
        
        P_matrix;      % The P matrix, I(n) - ones(n)
        N;             % Number of robots
        phi;           % N x 1 matrix of speed phase angles
        start = 0;
    end % end private properties
    
    methods
        
        % Constructor
        function OF = oscillatingFish(initial_poses)
            OF.initial_poses = initial_poses;
            OF.N = size(initial_poses, 1);
            OF.P_matrix = eye(OF.N) - 1/OF.N*ones(OF.N);
            OF.phi = zeros(OF.N, 1);
            
        end % end constructor
        
        function [ commands ] = fishControlLaw(OF, t, states)
            if OF.start == 0
                t_step = .04;
                OF.start = 1;
            else
                t_step = .04;
            end
            commands = zeros(OF.N, 3);    % init. command matrix as 0's
            states(:,1) = states(:,1)*OF.scale;
            states(:,2) = states(:,2)*OF.scale;
            states(:,4) = states(:,4)*OF.scale;
            E = OF.createE();             % create ellipse matrix E(phi)
            r = OF.createR(states);       % create complex vector matrix
            s = OF.createS(r, E, states); % create S matrix
            
            % loop to set positions for robots
            for j = 1:OF.N
                phi_j = OF.phi(j, 1);
                
                u_x = (1 + OF.mu*cos(phi_j));
                
                a = OF.P_matrix(j, :)*s;
                b = 1i*exp(1i*states(j, 6));
                u_theta = OF.omega - OF.k*real(a'*b);
                
                phi_j_dot = OF.Omega/OF.omega*u_theta;
                %phi_j_dot = OF.Omega;
                OF.phi(j, 1) = phi_j + phi_j_dot*t_step;
                
                commands(j,1) = u_x/OF.scale;
                commands(j,2) = u_theta;
                tic;
            end % end for loop
        end % end control law
        
        function [ E_matrix ] = createE(OF)
            
            E_matrix = zeros(OF.N, 1);
            for j = 1:OF.N
                phi_j = OF.phi(j, 1);
                E_matrix(j, 1) = 1/(OF.Omega^2 - OF.omega^2) ...
                    .*(OF.Omega.*sin(phi_j) + 1i.*OF.omega.*cos(phi_j));
                
            end % end loop
            
        end % end createS
        
        function [ R_matrix ] = createR(OF, states)
            
            R_matrix = zeros(OF.N, 1);
            for j = 1:OF.N
                x = states(j, 1);
                y = states(j, 2);
                R_matrix(j, 1) = x + 1i.*y;
                
            end % end for loop
            
        end % end createR
        
        function [ s_matrix ] = createS(OF, R, E, states)
            
            s_matrix = zeros(OF.N, 1);
            
            for j = 1:OF.N
                a = exp(1i.*states(j, 6));
                b = 1i.*OF.omega.*R(j, 1);
                c = 1i.*OF.omega*OF.mu.*exp(1i*states(j, 6)).*E(j, 1);
                s_matrix(j, 1) = a - b + c;
                
            end % end for loop
            
        end % end createS
        
        function [] = simulate(OF, runTime)
            close all
            clear trajectory current_position
            
            states = zeros(OF.N, 7);
            states(:, 1) = OF.initial_poses(:, 1);
            states(:, 2) = OF.initial_poses(:, 2);
            states(:, 3) = OF.initial_poses(:, 3);
            states(:, 6) = OF.initial_poses(:, 4);
            
            x_history = zeros(runTime + 1, OF.N);
            y_history = zeros(runTime + 1, OF.N);
            theta_history = zeros(runTime + 1, OF.N);
            
            for robot = 1:OF.N
                x_history(1, robot) = OF.initial_poses(robot, 1);%/OF.scale;
                y_history(1, robot) = OF.initial_poses(robot, 2);%/OF.scale;
                theta_history(1, robot) = OF.initial_poses(robot, 4);
            end % initial positions loop
            
            for t = 1:runTime
                x_old = states(:, 1); %/OF.scale;
                y_old = states(:, 2); %/OF.scale;
                theta_old = states(:, 6);
                
                commands = OF.fishControlLaw(t, states);
                %commands = OF.testControlLaw(states);
                vx = commands(:, 1); %/OF.scale;
                utheta = commands(:, 2);
                
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
                
                states(:, 1) = x_new; %*OF.scale;
                states(:, 2) = y_new; %*OF.scale;
                states(:, 4) = vx; %*OF.scale;
                states(:, 6) = theta_new;
                states(:, 7) = utheta;
                
                for robot = 1:OF.N
                    x_history(t + 1, robot) = x_new(robot);
                    y_history(t + 1, robot) = y_new(robot);
                    theta_history(t + 1, robot) = theta_new(robot);
                    
                end % history update loop
                
            end % end for loop
            
            figure
            hold on
            plot(x_history,y_history);
            axis('equal');
            for robot = 1:OF.N
                current_position(robot) = plot(x_history(1, robot), ...
                    y_history(1, robot),'Marker','.','markersize', 20);
            end
            
            
            for i = 1:runTime
                for robot = 1:OF.N
                    set(current_position(robot), 'xdata', x_history(i, robot), ...
                        'ydata', y_history(i, robot));
                end
                
                pause(.001)
            end
            
        end % end simulate
        
        function commands = testControlLaw(OF, states)
            n_robots = size(states, 1);
            n = OF.N;
            commands  = zeros(n_robots, 3);
            commands(:,1) = ones(n_robots, 1);
            commands(:,2) = ones(n_robots, 1);
        end
        
        
    end % end methods
    
end % end class

