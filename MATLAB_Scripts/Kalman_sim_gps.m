clear all;
close all;

numTrials = 10000;
dt = 0.001;
%A = [1.23 0.968 0.05 0.7; 0 1 0 0; 0 0 1 0; 0 0 0 1];
A = [1 0 dt 0; 0 1 0 dt; 0 0 1 0; 0 0 0 1];
%Initialize x to 0;
x = [0;0;0;0;];
q = 1;
%q = 4;
sd_r = 0.5;
%Initialize input
z = zeros(4, numTrials);
for i = 1:numTrials
    xrand = rand()/2;
    yrand = rand()/2;
    if i == 1
        z(:,i) = [i+xrand i+yrand 1 1];
    else
        z(:,i) = [i+xrand i+yrand 0 0];
        z(3,i) = z(1,i)-z(1,i-1);
        z(4,i) = z(2,i)-z(1,i-1);
    end
end
var_dx = 2;
var_dy = 2;
var_vx = 4;
var_vy = 4;
p= [var_dx 0 0 0; 0 var_dy 0 0; 0 0 var_vx 0; 0 0 0 var_vy];
%initialize H
    H = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1];
    %Q is noise covariance
    %Q = [0.01 0.01 0.011 0.1; 0.01 0.01 0.1 0.01; 0.3 0.02 0.01 0.23; 0.054 0.13 0.1 0.01];
    Q = q*[1/4*dt^4 0 0.5*dt^3 0; 0 1/4*dt^4 0 0.5*dt^3; 0.5*dt^3 0 dt^2 0; 0 0.5*dt^3 0 dt^2];
    %Q = zeros(4,4);
    %Q(4,4) = 1;
    
    %R = zeros(4,4);
    %R is model covariance?
    R = sd_r * [1/4*dt^4 0 0.5*dt^3 0; 0 1/4*dt^4 0 0.5*dt^3; 0.5*dt^3 0 dt^2 0; 0 0.5*dt^3 0 dt^2];
    
for i = 1:numTrials
    %predict Step
    x = A*x;
    p = A*p*A.'+ Q;
    %correct step
    K = p*H.'\(H*p*H.' + R);
    x = x + K*(z(:,i)-H*x);
    p = (eye(4)-K*H)*p;
end



