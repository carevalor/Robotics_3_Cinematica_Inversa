function [q1, q2, q3, q4] = inverKinematics(offset, poseTCP, L)
q1 = atan2(poseTCP(2,4),poseTCP(1,4));

w = poseTCP(1:3,4) - L(4)*poseTCP(1:3,3)- L(1)*[0 0 1]';

q_3 = acos((w'*w -(L(2)^2+L(3)^2))/(2*L(2)*L(3)));

q_2 = acos(-(L(3)^2-(w'*w+L(2)^2))/(2*sqrt(w'*w)*L(2)));


q3 = -q_3+offset;
q2 = atan2(w(3),norm(w(1:2)))+q_2-offset;
q4 = acos([cos(q1) sin(q1) 0]*poseTCP(1:3,3))-q3-q2;
end

