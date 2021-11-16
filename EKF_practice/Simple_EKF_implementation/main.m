close all
clear all
dt = 0.01;
t = 0:dt:6;
x0 = [0;0];
ref = [10;0];
m_plant = 5;
F = 5;

K = [10 3];
[t,x] = ode45(@(t,x) plant_dynamics(m_plant,x,-K*m_plant*(x-ref)+m_plant*9.81),t,x0);

N = length(x(:,1));

state = zeros(3,N);
state(3,1) = 3;
P = 0.01*eye(3);
P(3,3) = 1;

for i = 1:N-1
    [state_temp,P_temp]=predictEKF(state(:,i),P,-K*m_plant*(x(i,:)'-ref) + m_plant*9.81);
    [state(:,i+1),P]=measModelEKF(state_temp,P_temp,-K*(x(i,:)'-ref) + m_plant*9.81,x(i,1:2)');
end

subplot(2,2,1)
plot(t,state(1,:))
hold on;
grid on;
plot(t,x(:,1))
title('x-t')
legend('x_{est}','x_{ground truth}')

subplot(2,2,2)
plot(t,state(2,:))
hold on;
grid on;
plot(t,x(:,2))
title('v-t')
legend('v_{est}','v_{ground truth}')

subplot(2,2,3)
plot(t,state(3,:))
hold on
grid on
% x1 = [0 t(end)];
% y1 = [m_plant m_plant];
% line(x1,y1,'LineWidth',2)
% ylim([0 m_plant*1.2])
title('m_{est} - t')
P