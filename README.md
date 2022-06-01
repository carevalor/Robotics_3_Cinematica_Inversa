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
Dentro del ciclo while previamente descrito, se revisa todo el tiempo que caracter fue guardado por el usuario dentro de la variable 'str'. De esta forma, si la variable es 'd' 
