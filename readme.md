# Controlador PID - STM32F411CEU6

Este proyecto implementa un controlador PID utilizando un microcontrolador STM32F411CEU6. El sistema se organiza en tres módulos principales:

- **Salida PWM**: Controla la velocidad del motor a través de una señal PWM.
- **Entrada ADC**: Monitorea un sensor de entrada y realiza acciones como encender o apagar un LED según el valor del ADC.
- **Comunicación USB**: Permite la comunicación con una consola a través de USB (UART), para recibir datos y controlar el PWM.

## Versiones

Las siguientes versiones se irán actualizando conforme se añadan nuevas funcionalidades:

### v0.0
Estas etapas no se reflejan en el repositorio `.git`, ya que el repositorio fue inicializado después de estas etapas.

### v0.1
- **Salida PWM**: Controla el driver de potencia y regula la velocidad del motor. El circuito utilizado será subido en futuras actualizaciones.
- **Entrada ADC**: Controla el encendido y apagado de un LED basado en el valor del ADC. Si el valor es mayor a 512 (umbral de 50% en 10 bits), el LED se enciende; si es menor, el LED se apaga. Se diseñó para probar con potenciómetro.

- **Comunicación USB (UART 9600 baud)**: Permite recibir datos desde una consola para ajustar el valor de PWM%. El valor recibido es un carácter entre 0 y 9, el cual ajusta el PWM entre 0% y 90%.

### v0.2 (futura versión)
- Se añadirá la capacidad de recibir más de un carácter desde la consola y reflejar este dato en el PWM%, con un rango de 0% a 100%.
- Ajustar los límites de PWM para que se adapten a un rango de 0% a 100%, ya que actualmente el sistema no funciona fuera de este rango.


## Licencia
Este proyecto está bajo la licencia MIT. El código es de uso libre. Para más detalles, consulta el archivo [LICENSE](./LICENSE).
