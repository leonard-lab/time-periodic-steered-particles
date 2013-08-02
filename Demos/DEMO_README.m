% DEMO README
%
% There are two folders for two main demonstrations - synchronized
% formation and splayed formation. The tests were run with two robots.
% Because of limited conditions where the Miabots don't collide with each
% other, only a few sets of initial conditions were recorded. 
%
% Each run has saved graphs of the trajectories, as well as the data saved
% in .mat matrix files. For each run there are two saved matrices of data.
% The first has the state data - [x y theta]. The second has the command
% data - [u_x u_theta]. For each robot they are labeled with either an R1
% or R2 in the file name. 
%
% A trial was run in the splay state where one of the robots was picked up
% and moved to demonstrate robustness to disturbances. That only has a
% graph (and movie) saved with it.
%
% Conditions: 
% Splay state trials: scale = 4, omega = .8, initial poses = [-.25 -.45 0 0; .25 -.45 0 0]
%
% Sync state trials: 
% Demo conditions - scale = 6, mu = .5, omega = .8, Omega = 1.5*omega. 
% Imperfect - scale = 5, mu = .5, omega = .8, Omega = 1.5*omega
% initial poses = [-.2 -.55 0 0; .1 -.5 0 0].
%