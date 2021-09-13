close all
clear all
dt = 0.01;
t = 0:dt:60;
x0 = [0;0];
m_plant = 6;
F = 5;

[t,x] = ode45(@(t,x) plant_dynamics(m_plant,x,F),t,x0);

N = length(x(:,1));

for i = 1:N-1
    a(i,1) = (x(i+1,2)-x(i,2))/dt;
end

state = zeros(3,N);
state(3,1) = 2;
P = 0.1;

for i = 1:N-1
    [state_temp,P_temp]=predictEKF(state(:,i),P,F);
    [state(:,i+1),P]=measModelEKF(state_temp,P_temp,F,x(i,1));
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
x1 = [0 t(end)];
y1 = [m_plant m_plant];
line(x1,y1,'LineWidth',2)
ylim([0 m_plant*1.2])
title('m_{est} - t')