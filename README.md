Este proyecto tiene como objetivo desarrollar una computadora de vuelo básica para aeronaves VTOL (despegue y aterrizaje vertical), utilizando un ESP32 como plataforma principal y un MPU6050 para la medición de la orientación y movimientos de la aeronave. La computadora de vuelo incluye control de estabilidad a través de algoritmos PID, que gestionan los servos para controlar el alabeo (roll), el cabeceo (pitch) y el motor.

Características principales:

Lectura de sensores: Utiliza un MPU6050 para obtener datos de aceleración y giroscopio, calculando los ángulos de Euler (yaw, pitch, roll) mediante el uso del DMP (Digital Motion Processor) integrado en el sensor.

Control PID: Implementa controladores PID para estabilizar la aeronave, ajustando los servos en función de los errores en los ángulos de pitch y roll, lo que permite un vuelo estable.

Control de servos: Los servos son controlados para manejar los alerones, timón y motor, además de permitir la operación de periféricos adicionales como tren de aterrizaje o luces.

Receptor de radiocontrol: Se incorpora un receptor de 5 canales, que permite al piloto controlar el vuelo de la aeronave a través de un transmisor de radiocontrol.

Conexión I2C: La comunicación con el MPU6050 y otros dispositivos periféricos se maneja mediante el protocolo I2C.

Este sistema está diseñado como una solución flexible para aeronaves de radiocontrol en el contexto de proyectos de hobby o investigación, y es completamente personalizable para adaptarse a diferentes configuraciones de aeronaves y periféricos.

Licencia de uso: Este código es libre para usar, modificar y distribuir, siempre y cuando se mantenga el aviso legal y las condiciones del proyecto original.

Advertencia: Este sistema ha sido desarrollado para uso en aeronaves de radiocontrol y no debe ser utilizado en aeronaves tripuladas sin la evaluación y certificación adecuada.
