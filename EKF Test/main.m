close all
clear all
dt = 0.01;
t = 0:dt:1;
x0 = [0;0];
m_plant = 8;
F = 1;

[t,x] = ode45(@(t,x) plant_dynamics(m_plant,x,F),t,x0);

N = length(x(:,1));

for i = 1:N-1
    a(i,1) = (x(i+1,2)-x(i,2))/dt;
end

m(1) = 4;
v(1) = 0;
P = 1*eye(2);
R = 0.01*eye(2);
Q = 10*eye(2);
for i = 1:N-1
    % Dynamics Model Update
    v(i+1) = v(i) + (awgn(F,1e-10,'measured'))/m(i)*dt;
    m(i+1) = m(i);
    Gt = [1 -F/m(i)^2*dt;
        0 1];
    P_ = Gt*P*Gt' + R;

    % Measurement Update - Velocity from Camera Sensor    
    H = [1 0;
        0 1];
    K = P_*H'*inv(H*P_*H'+Q);
    meas = [x(i,2);F/awgn(a(i),50,'measured')];
    % Sensor Model - acceleration from IMU
    sensor_model = [v(i) + awgn(a(i),50,'measured')*dt;m(i)];
    z = meas - sensor_model;
    mu(i,1:2) = [v(i+1);m(i+1)]+K*z;
    v(i+1) = mu(i,1);
    m(i+1) = mu(i,2);
    P = (eye(2)-K*H)*P_;
end


subplot(2,2,1);
plot(t,x(:,1));
title('x - t');
xlabel('t (sec)')
ylabel('x (m)')
grid on;

subplot(2,2,2)
plot(t,x(:,2))
hold on;
plot(t,v)
title('v - t')
xlabel('t (sec)')
ylabel('v (m/s)')
grid on;

subplot(2,2,3)
plot(t(1:N-1),a)
ylim([0 1.2*a(1,1)]);
title('a - t (Ideal IMU)')
xlabel('t (sec)')
ylabel('a (m/s^2)')
grid on;

subplot(2,2,4)
plot(t(1:N-1),mu(:,2))
hold on;
x1 = [0 t(end)];
y1 = [m_plant m_plant];
line(x1,y1,'LineWidth',2)
ylim([0 mu(N-1,2)*1.2])
title('m_{est} (kg)')
grid on;
legend('Ground truth','m_{est}')

P
K