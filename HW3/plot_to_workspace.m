%plot

figure;
subplot(2, 1, 1);
plot(out.desired_yaw_rate);
grid on;
xlabel('Time (s)');
ylabel('Yaw rate (deg/s)');
title('Desired Yaw Rate');

subplot(2, 1, 2);
plot(out.actual_yaw_rate);
grid on;
xlabel('Time (s)');
ylabel('Yaw rate (deg/s)');
title('Actual Yaw Rate');
