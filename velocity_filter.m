clear all
close all
load('data/odrive_position_data.mat')

tdiff = diff(ts);
pdiff = diff(ps);
v_est = pdiff./tdiff;

alpha = 0.1; %update rate
initial_v = 5000; %initialize at v setpoint

v_filtered = zeros(size(pdiff));
v_filtered(1) = initial_v;
for i = 2:length(pdiff)
    v_filtered(i) = alpha * v_est(i-1) + (1 - alpha) * v_filtered(i-1); 
end

figure(1)
hold on
plot(ts(1:end-1), v_est)
plot(ts(1:end-1), v_filtered)
title('Velocity Estimation');
xlabel('Time t (s)');
ylabel('Velocity Estimate')
legend('Unfiltered', 'Filtered');
