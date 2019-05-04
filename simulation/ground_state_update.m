function qdot = ground_state_update(t,q,kc,params)
% ground_state_update 1D state update for inverted pendulum

% Made up figures
m = params.m;
Ilink = params.Ilink;
Iwheel = params.Iwheel;
I2 = params.I2;
g = params.g;

u = -kc*q;
qdot = [q(2); (m*g*sin(q(1)) - u(1))/(Ilink+Iwheel+I2); q(4) ; u(1)/I2 - (m*g*sin(q(1)) - u(1))/(Ilink+Iwheel+I2)];


end

