clear all;
close all;

numTrials = 100;
dt = 0.01;
%A = [1.23 0.968 0.05 0.7; 0 1 0 0; 0 0 1 0; 0 0 0 1];
A = [1 0 dt 0; 0 1 0 dt; 0 0 1 0; 0 0 0 1];
%Initialize x to 0;
x = [0;0;0;0;];

%Initialize input
z = zeros(4, numTrials);
for i = 1:numTrials
    xrand = rand()/2;
    yrand = rand()/2;
    z(:,i) = [i+xrand i+yrand 1+xrand 1+yrand];
end
var_dx = 1;
var_dy = 1;
var_vx = 1;
var_vy = 1;
p= [var_dx 0 0 0; 0 var_dy 0 0; 0 0 var_vx 0; 0 0 0 var_vy];
%initialize H
    H = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1];
    %Q is noise
    %Q = [0.01 0.01 0.011 0.1; 0.01 0.01 0.1 0.01; 0.3 0.02 0.01 0.23; 0.054 0.13 0.1 0.01];
    Q = [0.25 0 0.5 0; 0 0.25 0 0.5; 0.5 0 1 0; 0 0.5 0 1];
    %R = zeros(4,4);
    R = eye(4);
    
for i = 1:numTrials
    %predict Step
    x = A*x;
    p = A*p*A.'+ Q;
    %correct step
    K = p*H.'\(H*p*H.' + R);
    x = x + K*(z(:,i)-H*x);
    p = (eye(4)-K*H)*p;
end



