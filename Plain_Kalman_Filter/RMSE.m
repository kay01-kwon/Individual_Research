function rmse_result = RMSE( ground_truth, estimated_state )

n = length(ground_truth);

sum = 0;

for i=1:n
    error_x(i) = ground_truth(1,i) - estimated_state(1,i);
    error_y(i) = ground_truth(3,i) - estimated_state(3,i);
    sum = sum + error_x(i)^2 + error_y(i)^2;
end

rmse_result = sqrt(sum/n);

