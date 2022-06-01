rosshutdown
rosinit;
%%
motorSvcClient = rossvcclient('/dynamixel_workbench/dynamixel_command');
motorCommandMsg= rosmessage(motorSvcClient);
%%
motorCommandMsg.AddrName="Goal_Position";
%% Cinemática inversa
r1 = transl(130,0,30)*r2t([0 1 0;1 0 0; 0 0 -1]);
r2 = transl(130,0,100)*r2t([0 1 0;1 0 0; 0 0 -1]);
r = line2points(r2,r1,20);
q = curveSolve(r);
q = -180/pi*q;
%% PICK AND PLACE
k=21; %Número de punto en la interpolación
q(:,4) = q(:,4)-10; % Error de posción de los motores
%Va a la posición de la primera pieza
moveid([90 q(1,2:4) 30],motorCommandMsg,motorSvcClient)
%Ruta de bajada
for i = 1:k
    moveid([90 q(i,2:4),30],motorCommandMsg,motorSvcClient)
end
%griping
moveid([90 q(k,2:4) 15],motorCommandMsg,motorSvcClient)
qup = flipud(q);
%Ruta de subida
for i = 1:k
    moveid([90 qup(i,2:4),15],motorCommandMsg,motorSvcClient)
end
%Va a la posición sobre el soporte
moveid([0 qup(k,2:4),15],motorCommandMsg,motorSvcClient)
%Ruta de inserción
for i = 1:k
    moveid([0 q(i,2:4),15],motorCommandMsg,motorSvcClient)
end
%Libera la pieza y Sube
for i = 1:k
    moveid([0 qup(i,2:4),30],motorCommandMsg,motorSvcClient)
end
%Se posiciona sobre la segunda pieza
moveid([-90 qup(k,2:4),30],motorCommandMsg,motorSvcClient)
%Ruta de bajada
for i = 1:k
    moveid([-90 q(i,2:4),30],motorCommandMsg,motorSvcClient)
end
%Gripping y subida
for i = 1:k
    moveid([-90 qup(i,2:4),15],motorCommandMsg,motorSvcClient)
end
%Va a la posición sobre el soporte
moveid([0 qup(k,2:4),15],motorCommandMsg,motorSvcClient)
%Ruta de inserción
for i = 1:k
    moveid([0 q(i,2:4),15],motorCommandMsg,motorSvcClient)
end
%Libera la pieza y Sube
for i = 1:k
    moveid([0 qup(i,2:4),30],motorCommandMsg,motorSvcClient)
end
%Fin del proceso
%%
Sub=rossubscriber('/dynamixel_workbench/joint_states');
Sub.LatestMessage.Position;
%%
rosshutdown;