rosshutdown
rosinit;
%%
motorSvcClient = rossvcclient('/dynamixel_workbench/dynamixel_command');
motorCommandMsg= rosmessage(motorSvcClient);
%%
motorCommandMsg.AddrName="Goal_Position";
%%
t=1;
Init = rpy2tr(0,90,180,'deg');
start = [1 0 0 189.452;0 1 0 0; 0 0 1 242.601;0 0 0 1];
start = Init*start;
tr=0;
MTH = troty(5,'deg');
MTH2= troty(-5,'deg');
q=curveSolve(start)*180/pi;
moveid(-q,motorCommandMsg,motorSvcClient)
while(t>0)

    prompt = 'Funciona ';
    str = input(prompt,'s');

    if str=='x'
        t=0;
    end
   
    if str=='a'
        if tr == 1
            start(1,4)=start(1,4)-50
        elseif tr == 2
            start(2,4)=start(2,4)-50;
       
        elseif tr == 3
            start(3,4)=start(3,4)-50;
        
        elseif tr == 4
            start(:,3)=MTH*start(:,3);
        end
        q=curveSolve(start);
        q=q*180/pi
        moveid(-q,motorCommandMsg,motorSvcClient)

    end

    if str=='s'
        tr=tr-1;
        if tr<1
            tr=4;
        end
        check(tr);
    end

    if str=='d'
        if tr == 1
          start(1,4)=start(1,4)+50
        end
        if tr == 2
          start(2,4)=start(2,4)+50;
        end
        if tr == 3
          start(3,4)=start(3,4)+50;
        end
        if tr == 4
          start(:,3)=MTH2*start(:,3);
        end
        q=curveSolve(start);
        q=q*180/pi
        moveid(-q,motorCommandMsg,motorSvcClient)
    end

    if str=='w'
        tr=tr+1;
        if tr>4
            tr=1;
        end
        check(tr)
    end
end