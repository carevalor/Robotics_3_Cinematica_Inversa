# Robotics_3_Cinemática_Inversa
## Matlab + Toolbox
### Cinemática Inversa
El modelo de cinemática inversa se realiza utilizando el método geométrico. Para el caso del robot Px100 se debe tener especial cuidado con las restricciones geométricas derivadas de la construcción propia del robot. 
En particular se resalta el hecho de que el px100 solo funciona en configuración codo arriba y además que posee un offset angular en el brazo.

#### Solución para la primera articulación
Se busca ubicar el plano de trabajo de la cadena cinemática formada por las articulaciones 2,3 y 4 de forma tal que el punto objetivo $\vec{p}$ esté contenido. Para esto se sabe que el plano de trabajo debe ser generado por una linea que va del origen a la proyección de $\vec{p}$ sobre el plano xy, esto es equivalente a decir que $\theta_1$ debe ser igual al ángulo que forma la proyección mencionada con el el eje x del marco fijo.

$\theta_1 =  tan^{-1}(\dfrac{p_y}{p_x})$

#### Solución para la segunda y tercera articulación
Ya ubicado el plano de trabajo se puede solucionar el problema de posción análizando el RR formado ṕor la segunda y tercera articulación y luego solucionar el problema de orientación del eje de aproach con la última articulación. 

Para hacer este desacople se obtiene la posición del marco de referencia de la articulación 4 dada una orientación y posición del tcp. 
En este caso la posción del tcp es $\vec{p}$ y la orientción del tcp queda definida para esta aplicación por la dirección de aproah $\hat{a}$

El nuevo objetivo parcial sería un nuevo vector $\vec{w}$ de la posición del marco de referencia de la articulación 4 con respecto al marco de refrencia de la articulación 2. Se obtiene $\vec{w}$ como sigue:

$\vec{w} = \vec{p} - L_4\hat{a}-L_1\hat{z_0}$

Donde $L_4$ es la distancia desde el origen de la articualción 4 hasta el tcp del Px100 y $L_1$ es la distancia desde el suelo o base del robot hasta el origen de la articulación 2. Por ser un 2R basta con utilizar identidades trigonométricas para un triangulo en codo arriba para así obtener las siguientes expresiones para $\theta_2$ y $\theta_3$.

$\theta_2 = atan2(\dfrac{w_y}{\sqrt{w_x^2 + w_y^2}}) - \theta_0 + acos(-\dfrac{L_3^2-(w^Tw+L_2^2)}{2\sqrt{w^Tw}L_2)})$

$\theta_3 = \theta_0-acos(\dfrac{w^Tw-(L_2^2+L_3^2)}{2L_2L_3})$


Nota: $\theta_0 = atan(\dfrac{100}{35})$ es el angulo de offset que tiene el PX100.


#### Solución para la articulación 4
Al resolver el 2R, se sabe que la suma $\theta_2 + theta_3 + theta_4$ debe ser igual al angulo que forma el vector $\hat{a}$ con la proyección de $\vec{p}$ sobre el plano xy, o  bien el angulo entre el vector  $\vec{u} = [cos(\theta_1),sin(\theta_1),0]^T$ y $\hat{a}$. De esta manera, $\theta_4$ queda como sigue:

$\theta_4 = Iacos(\vec{u}^T\hat{a})-\theta_3-\theta_2$

Donde I es una variable indicadora que vale -1 cuando la dirección de aproach es exaxtamente igual a $-\hat{z_0}$ y 1 para cualquier otro caso, esto es necesario pues la función acos no es capaz de detectar el signo de giro cuando esa proyección es cero.

#### Implementación en Matlab
Se crea una función en matlab que recibe el offset del robot, la pose del TCP que se quiere lograr y la longitud de los eslabones del Px100 y devuleve el vector q con los valores articulares.
``` matlab
function [q1, q2, q3, q4] = inverKinematics(offset, poseTCP, L)
        q1 = atan2(poseTCP(2,4),poseTCP(1,4));

        w = poseTCP(1:3,4) - L(4)*poseTCP(1:3,3)- L(1)*[0 0 1]';

        q_3 = acos((w'*w -(L(2)^2+L(3)^2))/(2*L(2)*L(3)));

        q_2 = acos(-(L(3)^2-(w'*w+L(2)^2))/(2*sqrt(w'*w)*L(2)));


        q3 = -q_3+offset;
        q2 = atan2(w(3),norm(w(1:2)))+q_2-offset;
        q4 = acos([cos(q1) sin(q1) 0]*poseTCP(1:3,3))-q3-q2;
end
``` 
El uso de esta función es de especial cuidado pues las soluciones q que determina son a -q' soluciones del robot real. Es decir, que el robot real recibe como solución a -q donde q es la obtenida con esta función. Esto puede pasar por la forma en la que se definió el urdf del px100 pues no es sencible al uso de D-H mientras que la implemntación en matlab se realiza teniendo encuenta los marcos de referencia D-H.

La cinemática inversa se realiza y verifica para este modelo del Px100 con D-Hstd.
``` matlab
l(1) = 89.45;
l(2) = 105.95;
l(3) = 100;
l(4) = 107.6;

L(1) = Link('revolute','alpha',pi/2,'a',0,   'd',l(1),'offset',0,   'qlim',[-3*pi/4 3*pi/4]);
L(2) = Link('revolute','alpha',0,'a',l(2),   'd',0,'offset',atan(100/35),   'qlim',[-3*pi/4 3*pi/4]);
L(3) = Link('revolute','alpha',0,'a',l(3),   'd',0,'offset',-atan(100/35),   'qlim',[-3*pi/4 3*pi/4]);
L(4) = Link('revolute','alpha',0,'a',l(4),   'd',0,'offset',0,   'qlim',[-3*pi/4 3*pi/4]);
PhantomX = SerialLink(L,'name','Px');

PhantomX.tool = trotx(pi/2)*troty(pi/2);
``` 
### Espacio de Trabajo
El espacio de trabajo se aproxima teniendo en cuenta el dominio de las expresiones halladas para la cinemática inversa, en particula, si se toma la expresión de $\theta_3$ se obtiene la siguiente inecuación.

$\dfrac{||w||^2-(L_2^2 + L_3^2)}{2L_2L_3} \leq 1$

De donde se obtiene que $||w|| \leq 205.95 mm$
Teniendo encuenta la exteción que proporciona la última articulación se puede decir que la envolvente es aproximadamente la esfera $||w|| \leq 313 mm$

[![Screenshot-from-2022-06-01-18-55-26.png](https://i.postimg.cc/X7JW3BwZ/Screenshot-from-2022-06-01-18-55-26.png)](https://postimg.cc/s1tq4XHy)

Este bosquejo corresponde con el espacio alcanzable del Px100 y no incluye la posibilidad de llegar en cualquier orientación.

### Cinemática inversa con el Toolbox







## ROS - Aplicación Pick and place

Para la aplicación de Pick and Place se crea una función que interpola lineas rectas entre dos puntos, sin alterar la orientación de aproach, esta función se muestra acontinuación:
``` matlab
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
``` 
Esta recibe la postura de cada punto en el espacio, que por defecto debe ser la misma pues como se menciona la linea recta se recorre sin cambiar la orientación, y el número de puntos que se quieren tener en la interpolación.
Esta función devuelve una matriz de matrices de transformación homogenea con las posturas del manipulador durante la trayectoria.

Se utiliza una función auxiliar que recorre la matriz anterior y va resolviendo la cinemática inversa en cada postura.
``` matlab
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
``` 

Se define una sola trayectoria vertical compuesta por una rutina de bajada y una rutina de subida, esta se utiliza tanto para el pick como para el place y el pocisionamiento sobre los objetivos se hace con una rotación de la primera articulación, es decir, que las piezas objetivo y el soporte se encuentran sobre una circunferencia centrada en el origen del robot.

```matlab
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
```
## ROS - Aplicación de movimiento en el espacio de la tarea

Se creó un script en matlab que lee las teclas A,W,S y D que ingresa el usuario. Con la letra W el programa cambia el tipo de movimiento, siendo este translación en x y o z y rotación en la dirección de open de la herramienta. Las letras A y D realizan cambios positivos y negativos respectivamente en estos movimiento. Utilizando la cinemática inversa previamente descrita se obtienen los ángulos de cada articulación de acuerdo a dichas posiciones para ser enviadas al robot.

Para realizar la comunicación con el robot se utiliza ROS. Para conectar matlab con ROS se debe crear un nodo inicial. También es necesario crear un cliente para el servicio 'dynamixel_command' que permitira mover cada articulación. De la misma manera al igual que en los procesos anteriores, se realiza la creación de el mensaje que será enviado al cliente y el nombre del mansaje.

``` matlab
rosinit;
%%
motorSvcClient = rossvcclient('/dynamixel_workbench/dynamixel_command');
motorCommandMsg= rosmessage(motorSvcClient);
%%
motorCommandMsg.AddrName="Goal_Position";
```
Para leer la tecla que ingresa el usuario se hace uso de la función 'input', la cuál envía el mensaje guardado en la variable 'prompt' y luego guarda el string ingresado en la variable 'str'. En este caso se envía un mensaje de funcionamiento para avisar que el programa está esperando correctamente a que el usuario ingrese una tecla.

```matlab
prompt = 'Funciona ';
str = input(prompt,'s');
```
La línea anterior se encuentra dentro de un while infinito para que el programa siga esperando la siguiente tecla. Cuando la variable 'str' reciba el caracter 'x' el ciclo se rompera y el programa finalizará. La estructura general de esta lógica se muestra a continuación.

```matlab
while(t>0)
    if str=='x'
        t=0;
    end
end
```
Dentro del ciclo while previamente descrito, se revisa todo el tiempo que caracter fue guardado por el usuario dentro de la variable 'str'. De esta forma, si la variable es 'w' o 's' se va a cambiar el tipo de movimiento a realizar. Con las variables 'a' y 'd' se realiza el movimiento ya sea en positivo y en negativo.


```matlab
if str=='a'
        if tr == 1
            start(1,4)=start(1,4)-5;
        elseif tr == 2
            start(2,4)=start(2,4)-5;
       
        elseif tr == 3
            start(3,4)=start(3,4)-5;
        
        elseif tr == 4
            start=start*MTH;
        end
        q=curveSolve(start);
        q=q*180/pi
        moveid(q)
    end
 
    if str=='s'
        tr=tr-1;
        if tr<1
            tr=4;
        end
        check(tr);
    end
```
Para la tecla 'd' la estructura es similar pero se cambian las direcciones de los cambios. Nótese que se utilizan varios if anidados para revisar en que valor está la variable 'tr' la cuál indica que movimiento se está realizando. Si la variables es 1, sera translación en x, 2 para translación en y, 3 para la translación en z y 4 para rotación en Open. Para las teclas 's' y 'w' se cambian estos valores para cambiar el tipo de movimiento.

La función curvesolve() recibe la matriz homogenea que representa la pose del robot y realiza la cinemática inversa desarrollada previamente para obtener cada ángulo. Por otra parte, la función moveid() envía los valores al servicio para mover el robot. Dentro de esta función se utiliza la función mapfun() utilizada previamente que realiza un mapeo de los ángulos posibles del motor y la resolución del encoder.  

La función check utilizada es para revisar en que valor se encuentra la variable 'tr' y de acuerdo a dicho valor se imprima en consola que tipo de movimiento se va a realizar

```matlab
function c = check(tr)

        if tr==1
        disp('trax')
        elseif tr ==2
        disp('tray')
        elseif tr ==3
        disp('traz')
        elseif tr ==4
        disp('rot')
        end

end
```
Para realizar el movimiento en x y o z se modifican las posiciones (1,4),(2,4) y (3,4) de la matriz homogenea que describe la pose del robot. Dicha matríz 'start' será sobreescrita para que el movimiento se de respecto a la posición inmediatamente anterior


```matlab
start(1,4)=start(1,4)+5; % Para x
start(2,4)=start(2,4)+5; % Para y
start(3,4)=start(3,4)+5; % Para z
```
El valor que se le va a sumar o restar a la matriz es la distancia en milímetros que se va a transladar el robot en cada dirección.

## Análisis
Durante el desarrollo de la práctica se pudo evidenciar el funcionamiento de los robots phantom x. Debido al funcionamiento de los servicios solo se puede mover una articulación a la vez. En este comportamiento radica la importancia de la interpolación en la cinemática inversa. Al tener varios puntos cercanos para unir dos posiciones, se puede garantizar que la ruta que sigue el manipulador sea en línea recta. También se observó que la velocidad con la que se realiza la ruta depende de la cantidad de puntos de la interpolación. A mayor cantidad de puntos la velocidad disminuye. Esto se debe a que la cantidad de mensajes enviados al servicio para cada articulación aumenta.

También se observó que el robot presenta cierto error de posición. Sin embargo, debido a que cada punto de la interpolación es enviado a las articulaciones como posiciones absolutas, por lo que el dicho error de posición no se acumula en cada posición enviada.
## Conclusiones
 + Para asegurar que la trayectoria del robot sea en línea recta, es necesario realizar una interpolación que una el punto final e inicial.
 + Cada punto de la interpolación representa una matriz homogenea, debido a que cada punto no solo implica una posición sino también una orientación predeterminada.
 + Para los movimientos en z del pick and place que deben ser en linea recta, se mantiene la misma orientación de la herramienta para asegurar el correcto posicionamiento de la pieza.
 + Para la aplicación de las teclas no se utilizó interpolación, debido a que los cambios realizados en cada dirección eran pequeños en relación a los movimientos realizados en la aplicación de Pick and Place.


