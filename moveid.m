function [] = moveid(q, motorCommandMsg,motorSvcClient)
    t=q;
    for i=1:length(t)
        motorCommandMsg.Id=i;
        motorCommandMsg.Value=round(mapfun(t(i),-180,180,0,4096));%bits
        call(motorSvcClient,motorCommandMsg);
    end
end