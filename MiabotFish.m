%%
%************************************************************************
%  Sets up oscillatingFish object with random intial conditions.
%************************************************************************
clear all
clc
close all

N = 2;
positions = zeros(N, 4);

for i = 1:N
    
    positions(i, 1) = -.25*rand(1);
    positions(i, 2) = -.25*rand(1);
    positions(i,4) = wrapToPi(2*pi*rand(1));
end

fish = oscillatingFish(positions, 'headings','sync','collision_avoidance',false);

runTime = 20;
%%
%************************************************************************
%  Simulates oscillating fish behavior based on initial conditions.
%  Options are: 
%            'animate' - logical. Draw particle simulation of robots moving
%                        along the trajectories. 
%           'animation_speed' - numerical. Pause time between point
%                               updates. Default: 0.2.
%            Graph options - 'headings', 'phases', 'trajectory', 'ellipse',
%                            'graph_all'
%                            logical. User selects which plots to display.
%                            'graph_all' overides and displays all graphs.
%************************************************************************
simulate(fish, runTime, 'graph_all', true, 'animate',true,'animation_speed', 0.1);

%%
%************************************************************************
%  Sets up good demonstration conditions for the Miabot robots in current 
%  DCSL floor robot setup. Options are:
%            'n_robots' - Change number of robots. Default is original
%                         number oscillatingFish was initialized to. Currently
%                         limited to n = 2 or 3 robots.
%     
%            'headings' - Choose between 'sync' or 'splay'. Default is
%                         'sync'. Currently 3 robots only supports splay state. 
%************************************************************************ 

fish.demoConditions(runTime,'n_robots', 3, 'headings', 'splay');

%%
%************************************************************************
% Sets up and sends commands to Miabots.
%************************************************************************
control_law = @(t, x) fish.fishControlLaw(t,x);

% calls new Miabot object that actuates robot motion
m = Miabots(fish.initial_poses, control_law, 'velocity', runTime,...
    'sim', true,'sim_noise', [.00 .00 .00 .00]);
m.start

%%
%************************************************************************
% Plots Miabot trajectories
%************************************************************************
figure
hold on
N = size(fish.initial_poses,1);
for robot = 1:N
x_history(:, robot) = m.get_history(robot, 'x');
y_history(:, robot) = m.get_history(robot, 'y');
end
plot(x_history, y_history);
axis('equal');

%%
%************************************************************************
% Animates Miabot trajectories
%************************************************************************
colors = ['r';'b';'g';'k';'y';'m'];
a = size(m.get_history(1,'x'));
position = zeros(N);
for robot = 1:N
    statex = m.get_history(robot, 'x');
    statey = m.get_history(robot, 'y');
    position(robot) = plot(statex(1), ...
        statey(1),'Marker','.','markersize', 20,'Color', colors(robot));
end

for i = 1:a(2)
    for robot = 1:N
        clear statex statey
        statex = m.get_history(robot, 'x');
        statey = m.get_history(robot, 'y');
        set(position(robot), 'xdata', statex(i), ...
            'ydata', statey(i));
    end
    
    pause(.05)
end
