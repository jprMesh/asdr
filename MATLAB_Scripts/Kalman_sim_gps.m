clear all;
close all;

numTrials = 10000;
dt = 0.001;

%State transition Matrix
F = [1 0 dt 0; 0 1 0 dt; 0 0 1 0; 0 0 0 1];
%Initialize state to 0;
x=zeros(4,numTrials+1);
x(:,1) = [0;0;0;0;];

%Initialize state covariance matrix
var_dx = 2;
var_dy = 2;
var_vx = 4;
var_vy = 4;
p= [var_dx 0 0 0; 0 var_dy 0 0; 0 0 var_vx 0; 0 0 0 var_vy];

%initialize H
H = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1];

%Initialize noise covariance matrices
sd_q = 1;
sd_r = 0.5;    
%Q is piecewise white noise.
Q = sd_q*[1/4*dt^4 0 0.5*dt^3 0; 0 1/4*dt^4 0 0.5*dt^3; 0.5*dt^3 0 dt^2 0; 0 0.5*dt^3 0 dt^2];
%R is piecwise white noise
R = sd_r * [1/4*dt^4 0 0.5*dt^3 0; 0 1/4*dt^4 0 0.5*dt^3; 0.5*dt^3 0 dt^2 0; 0 0.5*dt^3 0 dt^2];

%Initialize input
M = zeros(4, numTrials);
for i = 1:numTrials
    xrand = rand()/2;
    yrand = rand()/2;
    if i == 1
        M(:,i) = [i+xrand i+yrand 1 1];
    else
        M(:,i) = [i+xrand i+yrand 0 0];
        M(3,i) = M(1,i)-M(1,i-1); %Velocity isn't directly measured.
        M(4,i) = M(2,i)-M(1,i-1);
    end
end

for i = 1:numTrials
    %predict Step
    x(:,i) = F*x(:,i);
    p = F*p*F.'+ Q;
    %correct step
    K = p*H.'\(H*p*H.' + R);
    x(:,i+1) = x(:,i) + K*(M(:,i)-H*x(:,i));
    p = (eye(4)-K*H)*p;
end



