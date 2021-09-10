function [dxdt, z]= motor_dynamics(x,K,d)
A = [0 1;
    0 -10];

B1 = [0;1];

B2 = [0;1];

D1 = 0;

D2 = 0;

z = motor_observation(x);


dxdt = (A+B1*K)*z+B2*d;
    
end