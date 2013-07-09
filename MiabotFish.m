clear all
clc
close all
pointGenerator;
fish = oscillatingFish(positions);

runTime = 100;
simulate(fish, runTime);

% call control law for robot motion
control_law = @(t, x) fish.fishControlLaw(t,x);

% calls new Miabot object that actuates robot motion
m = Miabots(fish.initial_poses, control_law, 'velocity', runTime,...
    'sim', true,'sim_noise', [.00 .00 .00 .00]);
m.start


% plots the resulting path of the two robots against the ideal
figure
hold on
N = size(fish.initial_poses,1);
for robot = 1:N
    plot(m.get_history(robot,'x'), m.get_history(robot,'y'));
end
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
    
    pause(.002)
end


% 
