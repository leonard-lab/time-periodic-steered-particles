        function [states_out, measurements_out] = propagate(obj, states_in, commands_in, dt, noise, N)
            % PROPAGATE Takes in current state, commands, time duration,
            % and measurement noises and propagates kinematics forward and
            % returns states and states with measurement noise.
            %
            % SYNOPSIS [states_out, measurements_out] = propagate(obj, states_in, commands_in, dt, noise)
            %
            % INPUTS obj: the object
            % states_in: an n_robots X 7 matrix with the second dimension
            % as the state of the robot [x y z vx vz theta theta_dot]
            % commands_in: an n_robots X M inputs matrix. If velocity or
            % direct control format of 2nd dimension is [ux utheta uz]; if
            % it is waypoint control the format is [x y z theta] of the
            % waypoint
            % dt: the time to propagate the system in seconds
            % noise: a length 4 vector containing the standard deviation of
            % the guassian noise to be applied to the measurement in the
            % format [x y z theta].
            
            states_out = zeros(N, 7);
            measurements_out = zeros(N, 7);
            for i=1:N
                diffConversionFactor = 0.0667;
                motorScaleFactor = 501;
                max_motor_speed = 1000;
                
                eps = 0.001;
                
                x = states_in(i,1);
                y = states_in(i,2);
                z = states_in(i,3);
                v_x = states_in(i,4);
                v_z = states_in(i,5);
                theta = states_in(i,6);
                omega = states_in(i,7);
                
                

                        u_x = commands_in(i,1);
                        u_omega = commands_in(i,2);
                        u_z = commands_in(i,3);
 
                
                v_right = (u_x + u_omega*diffConversionFactor/(2))*motorScaleFactor;
                v_left = (u_x - u_omega*diffConversionFactor/(2))*motorScaleFactor;
                
                if v_right > max_motor_speed
                    v_right = max_motor_speed;
                elseif v_right < -max_motor_speed
                    v_right = -max_motor_speed;
                end
                
                if v_left > max_motor_speed
                    v_left = max_motor_speed;
                elseif v_left < -max_motor_speed
                    v_left = -max_motor_speed;
                end
                
                u_x = ((v_left + v_right)/2)/motorScaleFactor;
                u_omega = (v_right - v_left)/(diffConversionFactor*motorScaleFactor);
                
                if abs(u_omega) < eps
                    theta_out = theta;
                    x_out = x + u_x*dt*cos(theta);
                    y_out = y + u_x*dt*sin(theta);
                else
                    theta_out = theta + u_omega*dt;
                    radius = u_x/u_omega;
                    x_out = x + radius*(sin(theta_out) - sin(theta));
                    y_out = y + radius*(cos(theta) - cos(theta_out));
                    theta_out = wrapToPi(theta_out);
                end
                states_out(i,:) = [x_out y_out z u_x v_z theta_out u_omega];
                measurements_out(i, :) = [x_out+normrnd(0, noise(1)) y_out+normrnd(0, noise(2)) z u_x v_z wrapToPi(theta_out + normrnd(0, noise(4))) u_omega];
            end
        end


