# Robotics_3_Cinematica_Inversa
## Matlab + Toolbox
### Cinemática Inversa
### Espacio de Trabajo
### Cinemática inversa con el Toolbox

### Análisis
 + De acuerdo a la configuración del último grado de libertad del manipulador, se identifica que el ángulo que gira dicha articulación es Pitch.
 + Debido a que la parte final del manipulador serian phantom x es un robot 2 R, las dos soluciones posibles corresponden a las configuraciones de codo arriba y de codo abajo.
 + El espacio diestro del manipulador corresponde al espacion dentro del cual el robot puede llegar en varias configuraciones de posición. En el espacio que no es diestro, el robot solo podrá llegar con una única orientación.


## ROS - Aplicación Pick and place
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

Para realizar la rotación en Open de la herramienta se realiza una rotación en la dirección de y. Para ello se crean dos matrices de rotación para las direcciones positiva y negativa respectivamente.

```matlab
MTH = troty(5,'deg');
MTH2= troty(-5,'deg');
```
Estas matrices se multiplican por la orientación actual del robot para obtener la nueva orientación.

```matlab
elseif tr == 4
   start(:,3)=MTH*start(:,3);
end
```
## Análisis

Durante el desarrollo de la práctica se pudo evidenciar el funcionamiento de los robots phantom x. Debido al funcionamiento de los servicios solo se puede mover una articulación a la vez. En este comportamiento radica la importancia de la interpolación en la cinemática inversa. Al tener varios puntos cercanos para unir dos posiciones, se puede garantizar que la ruta que sigue el manipulador sea en línea recta. También se observó que la velocidad con la que se realiza la ruta depende de la cantidad de puntos de la interpolación. A mayor cantidad de puntos la velocidad disminuye. Esto se debe a que la cantidad de mensajes enviados al servicio para cada articulación aumenta.

También se observó que el robot presenta cierto error de posición. Sin embargo, debido a que cada punto de la interpolación es enviado a las articulaciones como posiciones absolutas, por lo que el dicho error de posición no se acumula en cada posición enviada.

## Conclusiones
 + Para asegurar que la trayectoria del robot sea en línea recta, es necesario realizar una interpolación que una el punto final e inicial.
 + Cada punto de la interpolación representa una matriz homogenea, debido a que cada punto no solo implica una posición sino también una orientación predeterminada.
 + Para los movimientos en z del pick and place que deben ser en linea recta, se mantiene la misma orientación de la herramienta para asegurar el correcto posicionamiento de la pieza.
 + Para la aplicación de las teclas no se utilizó interpolación, debido a que los cambios realizados en cada dirección eran pequeños en relación a los movimientos realizados en la aplicación de Pick and Place.
