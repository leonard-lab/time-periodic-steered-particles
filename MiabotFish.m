%%
clear all
clc
%close all
%pointGenerator;

%positions = [-0.2577 -0.0514 0 -2.2921; -0.1788 0.1275 0 -2.2745];
%positions = [-0.5459 0.1304 0 -1.6687; -0.5483 -0.0426 0 -1.6747];
%positions = [-0.0103 -.4119 0 -.0184; -.2836 -0.4010 0 -0.0095];

%positions = [-.6245 0.0139 0 -1.9812; -0.3620 0.2621 0 -1.9948];

%two good
%positions = [.0503 -.5130 0 -.2331; .0682 -.6709 0 -.2244];

% three

%positions = [.25 -.55 0 0; -.25 -.55 0 0; 0 -.55 0 0];

positions = [0.0605 -.4995 0 0.3998; 0.0131 -.6810 0 .4093; .2437 -.5797 0 .4010];

fish = oscillatingFish(positions, 'headings','sync','collision_avoidance',false);

runTime = 10;
%%
simulate(fish, runTime, 'graph_all', true, 'animate',true,'animation_speed', 0.1);

%%
miabotSim(fish, runTime, 'animate', false);

%%

% call control law for robot motion
control_law = @(t, x) fish.fishControlLaw(t,x);

% calls new Miabot object that actuates robot motion
m = Miabots(fish.initial_poses, control_law, 'velocity', runTime,...
    'sim', true,'sim_noise', [.00 .00 .00 .00]);
m.start
%%
% plots the resulting path of the two robots against the ideal
figure
hold on
N = size(fish.initial_poses,1);
for robot = 1:N
%      plot(m.get_history(robot,'x'), m.get_history(robot,'y'));
%      axis('equal')

x_history(:, robot) = m.get_history(robot, 'x');
y_history(:, robot) = m.get_history(robot, 'y');
end
plot(x_history, y_history);
axis('equal');
% colors = ['r';'b';'g';'k';'y';'m'];
% a = size(m.get_history(1,'x'));
% position = zeros(N);
% for robot = 1:N
%     statex = m.get_history(robot, 'x');
%     statey = m.get_history(robot, 'y');
%     position(robot) = plot(statex(1), ...
%         statey(1),'Marker','.','markersize', 20,'Color', colors(robot));
% end
% 
% for i = 1:a(2)
%     for robot = 1:N
%         clear statex statey
%         statex = m.get_history(robot, 'x');
%         statey = m.get_history(robot, 'y');
%         set(position(robot), 'xdata', statex(i), ...
%             'ydata', statey(i));
%     end
%     
%     pause(.05)
% end


% 
