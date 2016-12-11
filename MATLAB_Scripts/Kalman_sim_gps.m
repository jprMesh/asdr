clear all;
close all;

numTrials = 5;
dt = 0.01;
%A = [1.23 0.968 0.05 0.7; 0 1 0 0; 0 0 1 0; 0 0 0 1];
A = [1 0 dt 0; 0 1 0 dt; 0 0 1 0; 0 0 0 1];
%Initialize x to 0;
x = [0;0;0;0;];

%Initialize input
z = zeros(4, numTrials);
for i = 1:numTrials
    z(:,i) = [i i 1 1];
end
var_dx = 1;
var_dy = 1;
var_vx = 1;
var_vy = 1;
p= [var_dx 0 0 0; 0 var_dy 0 0; 0 0 var_vx 0; 0 0 0 var_vy];
%initialize H
    H = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1];
    %Q is noise
    Q = [0.01 0 0 0; 0 0.01 0 0; 0 0 0.01 0; 0 0 0 0.01];
    R = zeros(4,4);
for i = 1:numTrials
    
    %predict Step
    x = A*x;
    p = A*p*A.'+Q;
    %correct step
    K = p*H.'\(H*p*H.' + R);
    x = x + K*(z(:,i)-H*x);
    p = (eye(4)-K*H)*p;
end



