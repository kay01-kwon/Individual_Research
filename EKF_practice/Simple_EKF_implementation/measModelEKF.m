function [state_est,cov_est] = measModelEKF(state_temp,cov_temp,input,obs)

Ht = [1 0 0;
    0 1 0];
Qt = 0.01*eye(2);
sensor_model = [state_temp(1);state_temp(2)];
innovation = obs - sensor_model;
Kt = cov_temp*Ht'*inv(Ht*cov_temp*Ht'+Qt);
state_est = state_temp + Kt*innovation;
cov_est = (eye(3) - Kt*Ht)*cov_temp;

end