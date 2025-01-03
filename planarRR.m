% This file calculate the accelerations fpr the three-link planar robot.

function Ydot = planarRR(t,Y);

% Make robot parameters global the link three link function
global a1 a2 a3 m1 m2 m3 g

% What are the control torque?
t1 = 10; % Nm
t2 = 5;  % Nm
t3 = 5;  % Nm

% input states
q1  = Y(1);
q2  = Y(2);
q3  = Y(3);
q1d = Y(4);
q2d = Y(5);
q3d = Y(6);

q12 = q1 + q2;
q23 = q2 + q3;
q123 = q1 + q2 + q3;

% D matrix
d11 = (m1 + m2 + m3)*a1^2 + 2*(m2 + m3)*a1*a2*a3 + (m2 + m3)*a2^2 + m3*a3^2 + 2*(m3*a1*a3*cos(q1)*cos(q123)) - 2*(m3*a1*a3*sin(q1)*sin(q123)) + 2*(m2*a2*a3*cos(q123));
d12 = (m2 + m2)*(a1*a2) + (m2+m3)*(a2^2) + m3*a3^2 + 2*(m3*a1*a3*cos(q1)*cos(q123)) - 2*(m3*a1*a3*sin(q1)*sin(q123)) - 2*(m3*a2*a3*cos(q123));
d13 = m2*a3^2 + 2*m3*a1*a3*cos(q1)*cos(q123) - 2*m3*a1*a3*sin(q1)*sin(q123) + 2*m1*a2*a3*cos(q123);
d21 = (m1 + m2)*(a1*a2*a3) + (m1 +m2)*a2^2 +  m2*a3^2 + m3*a3*cos(q1)*cos(q123) - m3*a3*sin(q1)*sin(q123) + 2*m3*a2*a3*cos(q123);
d22 = 2*(m1 + m2)*a2^2 + m3*a3^2 + 2*m3*a2*a3*cos(q123);
d23 = m3*a3^2 + 2*m3*a2*a3*cos(q123);
d31 = m2*a2^2 + m3*a1*a3*cos(q1)*cos(q123) - m3*a1*a3*sin(q1)*sin(q123);
d32 = m3*a2^2 + m3*a2*a3*cos(q123);
d33 = m3*a2^2;

D = [d11 d12 d13;d21 d22 d23;d31 d32 d33];

% matrix for qxdot^2
a11 = -2*m3*a1*a3*sin(q1)*cos(q123) - 2*m3*cos(q1)*sin(q123) - m3*a1*a3*cos(q1)*cos(q123) - 2*m3*a2*a3*sin(123);
a12 = -m3*a1*a3*sin(q1)*cos(q123) - m3*a1*a3*cos(q1)*sin(q123) -2*m3*a2*a3*sin(q123);
a13 = -m3*a1*a3*sin(q1)*cos(q123) - m3*a1*a3*cos(q1)*sin(q123) - 2*m3*a2*a3*sin(q123);
a21 = -m3*a3*cos(q1)*sin(q123) - m3*a2*sin(q1)*cos(q123) - m3*a1*a3*sin(q23) - 2*m2*a2*sin(q123) - (m1 + m2)*(a1*a2*sin(q2));
a22 = -2*m2*a2*a3*sin(q123);
a23 = -2*m2*a2*a3*sin(q123);
a31 = -2*m3*a1*a3*sin(q1)*cos(q123) - m3*a1*a3*cos(q1)*sin(q123) - 2*m3*a2*a3*sin(q123) - m3*a1*a3 - m3*a2*a3;
a32 = -m3*a2*a3;
a33 = 0;

% matrix for qxdot*qydot
b11 = -4*m3*a1*a3*cos(q1)*sin(q123) - 4*m3*a1*a3*sin(q1)*cos(q123);
b12 = b11;
b13 = -2*m3*a1*a3*cos(q1)*sin(q123) - 2*m3*a1*a3*sin(q1)*cos(q123);
b21 = -(m1+m2)*(a1*a2*a3) - m3*a3*cos(q1)*sin(q123) - m3*a3*sin(q1)*cos(q123) - 4*m2*a2*a3*sin(q123) - (m2 + m3)*(a1*a2*a3) - m3*a1*a3*sin(q123);
b22 = -m3*a3*cos(q1)*sin(q123) - m3*a3*sin(q1)*cos(q123) - 4*m2*a2*a3*sin(q123) - m3*a1*a3*sin(q23);
b23 = -4*m3*a2*a3*sin(q123);
b31 = -2*m3*a1*a3*cos(q1)*sin(q123) - 2*m3*a1*a3*sin(q1)*cos(q123) -m3*a2*a3 - m3*a1*a3;
b32 = -4*a2*a3*sin(q123) - m3*a2*a3 - m3*a1*a3;
b33 = -m3*a2*a3*sin(q123) - m3*a2*a3;

% C*qd matrix
C = [a11 a12 a13;a21 a22 a23;a31 a32 a33] * [q1d^2;q2d^2;q3d^2] + [b11 b12 b13; b21 b22 b23; b31 b32 b33]*[q1d*q2d;q1d*q3d;q2d*q3d];

% phi
phi1 = -(m1 + m2 + m3)*g*a1*cos(q1) - (m2 + m3)*g*a2*cos(q12) + m3*g*a3*cos(q123);
phi2 = -(m2*m3)*g*a2*cos(q12) + m3*g*cos(q123);
phi3 = -m3*g*cos(q123);
phi = [phi1 phi2 phi3]';

% calculate the generalized accelerations
tau = [t1 t2 t3]';
qdd = inv(D) * (tau - C- phi);

% generate the change in the state vector
Ydot(1) = q1d;
Ydot(2) = q2d;
Ydot(3) = q3d;
Ydot(4) = qdd(1);
Ydot(5) = qdd(2);
Ydot(6) = qdd(3);

% Column vector
Ydot = Ydot';
