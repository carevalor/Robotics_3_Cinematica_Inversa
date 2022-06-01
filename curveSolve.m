function [q] = curveSolve(r)
l(1) = 89.45;
l(2) = 105.95;
l(3) = 100;
l(4) = 107.6;
offset = atan(100/35);
q1 = zeros(max(size(r))/4,1);
q2 = zeros(max(size(r))/4,1);
q3 = zeros(max(size(r))/4,1);
q4 = zeros(max(size(r))/4,1);
for i = 1:max(size(r))/4
    [q1(i) q2(i) q3(i) q4(i)] = inverKinematics(offset, r(4*i-3:4*i,:),l);
end
q = [q1 q2 q3 q4];
end