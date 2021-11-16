function [y] = plant_dynamics(m,x,u)
A = [0 1;
    0 0];
B = [0;1/m];
y = A*x+B*u - [0;9.81];

end

