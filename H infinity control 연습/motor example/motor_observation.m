function z = motor_observation(x)
A = [0 1;
    0 -10];

B1 = [0;1];

B2 = [0;1];

C = eye(2);


z = C*x;

end

