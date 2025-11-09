Descripción:

Este programa implementa una comunicación serial por software (Soft UART) y salidas PWM por hardware en el microcontrolador PIC16F887, usando el compilador XC8 en MPLAB.
El sistema también incluye detección de reset por WDT, manejo de entradas digitales, y control de LEDs y salidas de habilitación.

Configuración general:

Reloj interno: 8 MHz

Baud rate Soft UART: 9600 bps

PWM CCP1 (RC2) y PWM CCP2 (RC1)

WDT: activado

Compilador: MPLAB X + XC8

Pines principales:
Función	Pin	Tipo	Descripción
TX Soft UART	RD5	Salida	Comunicación serial (9600 bps)
CCP1 PWM	RC2	Salida	Canal PWM 1
CCP2 PWM	RC1	Salida	Canal PWM 2
DIn1	RD1	Entrada	Señal de control 1
DIn2	RD2	Entrada	Señal de control 2
EN_Out1	RD3	Salida	Habilitación activa en bajo
EN_Out2	RD4	Salida	Habilitación activa en bajo
LED_RA6 / RA7	RA6 / RA7	Salida	Indicadores de estado
LED_RD0	RD0	Salida	Parpadeo de inicio
LED_RE0	RE0	Salida	Indicador de reset por WDT

Funcionamiento:

Al iniciar, configura el reloj, UART por software, PWM y pines.

Si el reinicio fue por WDT, enciende RE0 y envía WDT_RST por UART.

Realiza un parpadeo inicial y activa los PWM al 50 % de ciclo útil.

Dependiendo del estado de las entradas RD1 y RD2, cambia las salidas EN_Out_RD3/RD4, los LEDs, y habilita/deshabilita los canales PWM.

Se limpia el WDT en cada ciclo principal (CLRWDT()).

Comunicación serial (debug)

Envía mensajes a 9600 bps desde RD5.

Versión: 1.0
Autor: Johann Haeussler
