# Robotics_3_Cinematica_Inversa
## Matlab + Toolbox
### Análisis
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
            start=start*MTH;
        end
        q=curveSolve(start);
        q=q*180/pi
        move(q)
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
