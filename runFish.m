clc
clear all
close all

c = oscillatingFish([1 1 0 0; 0 0 0 0]);
runTime = 200;
            
% call control law for robot motion
control_law = @(t,x) c.fishControlLaw(t,x); 
            
% calls new Miabot object that actuates robot motion
m = Miabots(c.initial_poses, control_law, 'velocity', runTime,...
    'sim', true);
m.start
u=0:.04:runTime;
            
% plots the resulting path of the two robots against the ideal
figure
hold on
plot(m.get_history(1,'y'), m.get_history(1, 'x'),'b');
plot(m.get_history(2,'y'), m.get_history(2, 'x'),'r');
figure
  plot(m.get_history(1,'state_times'), m.get_history(1,'theta_dot'), 'r')
 % m.get_history(2,'state_times'), m.get_history(2,'theta_dot'), 'g')
%             
% xlabel('Time (s)');
% ylabel('X-position (m)');
% title('Trajectory of Miabots Simulating a Coupled Pendulum');