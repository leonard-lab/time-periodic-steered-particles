N = 4; 

positions = zeros(N, 4);
% theta = 0;
% for i = 1:N
%     
%     positions(i, 1) = 2*cos(theta);
%     positions(i, 2) = 2*sin(theta);
%     positions(i, 4) = theta + pi/2;
%     
%     theta = theta + 2*pi/N;
% end


for i = 1:N
    
    positions(i, 1) = .25*rand(1);
    positions(i, 2) = .25*rand(1);
    positions(i,4) = 2*pi*rand(1);
end
