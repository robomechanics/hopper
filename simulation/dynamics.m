% Made up figures
params.m = 0.4;
params.Ilink = 0.01;
params.Iwheel = 0.02;
params.I2 = 0.01;
params.g = 9.81;

q0 = [1 0 0 0]'; %theta1, theta1dot, theta2, theta2dot
trange = [0 10];

I_big = params.Ilink + params.Iwheel - params.I2;

A = [0 1 0 0; params.m*params.g/I_big 0 0 0; 0 0 0 1; -params.m*params.g/I_big 0 0 0];
B = [0; -1/I_big; 0; 1/params.I2+1/I_big];
Q = eye(4);
R = 1;

kc = lqr(A,B,Q,R);
%kc = zeros(4);

[T,Y] = ode45(@(t,q) ground_state_update(t,q,kc, params), trange, q0);

figure(1)
subplot(2,2,1)
plot(T,Y(:,1));
title("Theta1");
subplot(2,2,2)
plot(T,Y(:,3));
title("Theta2");
subplot(2,2,3)
plot(T,Y(:,2));
title("Theta1dot");
subplot(2,2,4)
plot(T,Y(:,4));
title("Theta2dot");