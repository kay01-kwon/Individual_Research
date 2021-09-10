function [mu_next_step, cov_next_step] = kf(mu_current_step,cov_current_step,z_t)

global A
global B;
global C;
global F;

global Q;
global R;

global g;

mu_temp = A*mu_current_step + B*g;
cov_temp = A*cov_current_step*A' + F*Q*F';
K = cov_temp*C'*inv(C*cov_temp*C' + R);
mu_next_step = mu_temp + K*(z_t - C*mu_temp);
cov_next_step = (eye(4)-K*C)*cov_temp;

end