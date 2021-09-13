function [state_temp,cov_temp] = predictEKF(state_old,cov_old,input)
dt = 0.01;

A = [1 dt 0;
    0 1 0;
    0 0 1];
B = [1/2*dt^2;dt;0];

state_temp = A*state_old+B*input/state_old(3);

Ft = [1 dt -input/2/state_old(3)^2*dt^2;
    0 1 -input/state_old(3)^2*dt;
    0 0 1];
Rt = 0.01*eye(3);

cov_temp = Ft*cov_old*Ft' + Rt;

end

