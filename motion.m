% Simulate motion from the example

[length,dim] = size(Q1);
xA = a1*cos(Y0(1));
yA = a1*sin(Y0(1));
xB = xA + a2*cos(Y0(1) + Y0(2));
yB = yA + a2*sin(Y0(1) + Y0(2));
xC = xB - a3*cos(Y0(1) + Y0(2) + Y0(3));
yC = yB + a3*sin(Y0(1) + Y0(2) + Y0(3));

figure
h = plot([0,xA],[0,yA],[xA,xB],[yA,yB],[xB,xC],[yB,yC]);
axis([-2.5,2.5,-2.5,2.5]);
set(h,'linewidth',10);

for i = 2:length
    xA = a1*cos(Q1(i));
    yA = a1*sin(Q1(i));
    xB = xA + a2*cos(Q1(i) + Q2(i));
    yB = yA + a2*sin(Q1(i) + Q2(i));
    xC = xB - a3*cos(Q1(i) + Q2(i) + Q3(i));
    yC = yB + a3*sin(Q1(i) + Q2(i) + Q3(i));
    h = plot([0,xA],[0,yA],[xA,xB],[yA,yB],[xB,xC],[yB,yC]);
    axis([-2.5,2.5,-2.5,2.5]);
    set(h,'linewidth',10);
    drawnow
    pause(0.1)
end