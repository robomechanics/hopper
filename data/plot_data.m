clear all
close all
load('odrive_pv_control.mat');

%% 0.1 hz
figure(1)
subplot(2,1,1)
hold on
plot(ts_0_1_pv, ps_0_1_pv)
plot(ts_0_1_pv, pds_0_1_pv)
legend('Measured', 'Desired')
title("Position 0.1 hz");
subplot(2,1,2)
plot(ts_0_1_pv, pds_0_1_pv - ps_0_1_pv)
title("Position Error 0.1 hz");
saveas(gcf,'0.1hz.png')

%% 1 hz
ts = ts_1_pv(1:1500);
ps = ps_1_pv(1:1500);
pds = pds_1_pv(1:1500);

figure(2)
subplot(2,1,1)
hold on
plot(ts, ps)
plot(ts, pds)
legend('Measured', 'Desired')
title("Position 1 hz");
subplot(2,1,2)
plot(ts, pds - ps)
title("Position Error 1 hz");
saveas(gcf,'1hz.png')

%% 5 hz
ts = ts_5_pv(1:300);
ps = ps_5_pv(1:300);
pds = pds_5_pv(1:300);
figure(3)
subplot(2,1,1)
hold on
plot(ts, ps)
plot(ts, pds)
legend('Measured', 'Desired')
title("Position 5 hz");
subplot(2,1,2)
plot(ts, pds - ps)
title("Position Error 5 hz");
saveas(gcf,'5hz.png')