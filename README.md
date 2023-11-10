# Robot-Modular-de-Articulaciones-Oblicuas
<Pre>
Trabajo de fin de grado en el que se ha desarrollado un brazo modular con 6 grados de libertad

Programas de Arduino:
Control: bucle de control de un encoder.
i2cScan: Barrido de direcciones I2C para verificar la detección del multiplexor.
magnet_status: Programa para la validación de la distancia del imán.
MultiplexerScan: Barrido de las direcciones del mux para verificar la conexión de los encoders.
ProgramaArduino: Programa final para el control del brazo.
Readings: Programa para la calibración del encoder.

Distintos ensayos de funcionamiento:
test
test2gdl
test2gdl2
test2gdl2matlab 

Matlab app designer:
App: Primera propuesta de app, tiene funcionalidades que no se llegaron a implementar.
GUI: Interfaz de control del robot. Tiene el puerto de arduino estático, hay que escribirlo a mano en cada caso.
modelo_simulink: cadena cinemática en simulink para implementar simulaciones.

Piezas:
El diseño de las piezas se ha hecho sin tolerancias por cuestiones de tiempo. 
En el caso del rodamiento es recomendable añadirle una expansión horizontal entre -0,1 y -0,2 (según la máquina)
Para el resto de piezas no debería ser necesario añadir expansión horizontal. En cualquier caso yo le puse -0,1 a 
los segmentos y a la base para que el rodamiento entrase sin problemas.
</Pre>

# License
Hardware: CC-BY-SA

Software: GPLv3
