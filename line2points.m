function [tcp] = line2points(r1, r2, n)
    t = 0:1/n:1;
    v = r2(1:3,4) - r1(1:3,4);
    
    line = v*t + r1(1:3,4);
    
    tcp = [];
    aux = [r1(1:3,1:3) zeros(3,1); 0 0 0 1];
    for i = 1:(n+1)
        aux(1:3,4) = line(:,i);
        tcp = [tcp; aux];
    end
end