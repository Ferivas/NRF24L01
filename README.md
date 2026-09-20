# NRF24L01
Módulos de tx y rx wireless

<img width="600" alt="Módulo" src="https://github.com/Ferivas/NRF24L01/blob/main/DOCS/Pinout-nRF24L01-Wireless-Transceiver-Module.png">

VCC funciona entre 1.9V y 3.6V (3.3V nominal)
Si se utiliza un micro 5V se deben utilizar resistencias de 2K entre los IO del MCU y el nRF24L01 para operación normal.


CE es pin de salida<br>
CSN es el pin SS del SPI<br>
IRQ es un pin de entrada<br>

Para el Arduino Pro Mini con el ATMEGA328P los pines que se utilizan son:

* PB2 SS (CSN)
* PB5 SCK
* PB3 MOSI
* PB4 MISO
* Pb0 CE
* PD2 IRQ

  ## PRUEBAS BASADAS EN TARJETA BASEDEV

  Para utilizar la tarjeta BASEDEV es necesario hacer los siguientes ajustes en el HW para trabajar a 3.3V con un módulo ARduino micro pro.

  1. Cerrar SB1
  2. Montar regulador de voltaje de 3.3V (BU33SD5WG-TR)
  3. Cerrar SB3
  4. Unir líneas de interfaz del ESP32 para tener TX y RX directamnete en H3
  5. Desoldar Resistencia de 0 ohm en Arduino (esto desconecta los 5V al regulador del Arduino)
 
  Estos cambios se muestran a continuación:

  <img width="1000" alt="Bugs HW" src="https://github.com/Ferivas/NRF24L01/blob/main/DOCS/BASEDEV.jpg">

 

  
  


