close all;
clear all;
clc;
 
global A;
global B;
global C;
global Q;
global R;
global T;
global F;

global g;

% Define endtime, V(initial velocity), a(gravity acceleration), T (Sampling time)
V_0_magnitude = 100;
V = V_0_magnitude * [cos(pi/4);sin(pi/4)];
g = -9.81;
T = 0.1;

% Define initial state x= four dimensional
x_t = [0;V(1);0;V(2)];
x_vec = [x_t];

%Define noise covariance Q and R
Q = 100*eye(2);
mu_acc = [0 0];
R = 0.01*eye(2);
mu_pos_meas = [0 0];

% Define system matrices A, B, C
A = eye(4);
A(1,2) = T;
A(3,4) = T;

B = zeros(4,1);
B(4,1) = T;

C = zeros(2,4);
C(1,1) = 1;
C(2,3) = 1;

F = zeros(4,2);
F(1,1) = 0.5*T^2;
F(2,1) = T;
F(3,2) = 0.5*T^2;
F(4,2) = T;

% endtime step
endtime = 145;

% Define the initial state of the system
mu = zeros(4,endtime+1);
mu(2,1) = V(1);
mu(4,1) = V(2);
cov = 0.1*eye(4);

z_t = [0;0];
z_t_vec = [z_t];

for t=1:endtime
    % state transition
    epsilon_t = mvnrnd(mu_acc,Q);
    x_t = A*x_t + B*g + F*epsilon_t';
    x_vec = [x_vec x_t];
    % measurement
    delta_t = mvnrnd(mu_pos_meas,R);
    z_t = C*x_t + delta_t';
    z_t_vec = [z_t_vec z_t];
    
    [mu(:,t+1),cov] = kf(mu(:,t),cov,z_t);
end


plot(x_vec(1,:),x_vec(3,:),'r','Linewidth',3);
hold on;
plot(z_t_vec(1,:),z_t_vec(2,:),'o');
plot(mu(1,2:endtime+1),mu(3,2:endtime+1),'g','Linewidth',2);
xlabel('x'); ylabel('y');
legend('Real trajectory','Measurment', 'Predicted trajectory');
axis([0 1000 0 300]);
rmse_result = RMSE(x_vec,mu)
