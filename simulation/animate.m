function animate(q, fps)
% Animates 2D hopper given state variables in q
% q should be 6xn where n = number of state vars

n = size(q,2);
r = robotics.Rate(fps); % Set loop rate

r.reset(); % reset rate counter to 0
for i = 1:n
    
    hfig = plotHopper(q(:,i));
   
end

end

function hfig = plotHopper(qi)
xb = qi(1); yb = qi(2); thetab = qi(3); thetal = qi(4); thetae = qi(5); thetai = qi(6);
w = 0.12; %width of square in m
l1 = 0.2;
l2 = 0.2;
rw_diam = 0.2;

hfig = figure(2);


% Draw square body

rotationMatBody = [cos(thetab - pi/2) -sin(thetab-pi/2); sin(thetab-pi/2) cos(thetab-pi/2)];
p1 = [xb;yb] + rotationMatBody*[-w/2;-w/2];
p2 = [xb;yb] + rotationMatBody*[w/2;-w/2];
p3 = [xb;yb] + rotationMatBody*[w/2;w/2];
p4 = [xb;yb] + rotationMatBody*[-w/2;w/2];

line([p1(1) p2(1)], [p1(2), p2(2)]);
hold on
line([p2(1) p3(1)], [p2(2), p3(2)]);
line([p3(1) p4(1)], [p3(2), p4(2)]);
line([p4(1) p1(1)], [p4(2), p1(2)]);

% Plot linkages
theta1 = thetal + thetab + pi/2;
theta2 = thetal + thetab + thetae + pi/2;
xe = xb + l1*cos(theta1);
ye = yb + l1*sin(theta1);
line([xb, xe], [yb, ye]);
xf = xe + l2 * cos(theta2);
yf = xe + l2 * sin(theta2);
line([xe, xf], [ye, yf]);

% Reaction wheel
rectangle('Position',[xb - rw_diam/2 yb - rw_diam/2 rw_diam rw_diam ],'Curvature',[1,1], 'EdgeColor', 'r');
rotationMatRW = [cos(thetai - pi/2) -sin(thetai-pi/2); sin(thetai-pi/2) cos(thetai-pi/2)];
p5 = [xb;yb] + rotationMatRW*[0;rw_diam/2];
p6 = [xb;yb] + rotationMatRW*[0;-rw_diam/2];
p7 = [xb;yb] + rotationMatRW*[rw_diam/2; 0];
p8 = [xb;yb] + rotationMatRW*[-rw_diam/2; 0];
line([xb p5(1)], [yb p5(2)], 'Color', 'r');
line([xb p6(1)], [yb p6(2)], 'Color', 'r');
line([xb p7(1)], [yb p7(2)], 'Color', 'r');
line([xb p8(1)], [yb p8(2)], 'Color', 'r');


hold off
axis([0, 1, 0, 1])
axis('equal')


end


