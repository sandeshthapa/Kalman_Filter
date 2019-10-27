% True system 
close all;
x0 = [0;3]; % initial position + velocity
T = 0.1; % sampling time
F = [1 T; 0 1]; % state transition model
B = [1/2*T^2;T]; % control matrix
su = 0.1; % standard deviation of the acceleration
Q = su^2*[1/4*T^4 1/2*T^3;1/2*T^3 T^2]; % covariance of the acceleration
H = [ 1 0]; % measurement equation
sv = .5; % standard deviation of sensor noise 
R = sv^2; % covariance of the sensor noise
% propagate the true system
N = 100; % time steps
x = [];
z = [];
x_k1 = x0;
for i = 1:N
    uk = su*randn; % process noise 
    vk = sv*randn; % sensor noise
    x_k = F*x_k1 + B*uk; % Dynamics
    zk = H*x_k + vk;  % 
    x = [x x_k];
    z = [z zk];
    x_k1 = x_k;
end

%% Kalman filter
xhat = [2;0]; % wrong estimate
P = [10 0; 0 5];% initial covariance
X = [];
for i = 1:N
    % prediction
    uk = 0; % because the actual u is zero mean
    xhat = F*xhat + B*uk; 
    P = F*P*F' + Q; % predicted covariance includes the process noise covariance
    % update
    ytilde = z(i) - H*xhat;
    S = R + H*P*H';
    K = P*H'*inv(S);
    xhat = xhat + K*ytilde;
    P = (eye(2) - K*H)*P;
    X = [X xhat];
end

%% plots
figure;
plot(x(1,:))
hold on
plot(X(1,:));
legend('true position','estimated position');
xlabel('time step');
ylabel('position (meters)');
figure;
plot(x(2,:))
hold on
plot(X(2,:));
legend('true velocity','estimated velocity');
xlabel('time step');
ylabel('velocity (m/s)');


%% error plot
figure; plot(x(1,:) - X(1,:))
legend('estimation error for position');
ylabel('position error (meters)');
xlabel('time step');
