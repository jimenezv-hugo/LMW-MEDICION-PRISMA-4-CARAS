# Guia de uso y explicacion

## Hardware y orientacion

- Monta el HC-SR04 en el lado derecho del carrito, apuntando perpendicularmente
  hacia la cara que se va a medir.
- Conecta `Trig` a A3 y `Echo` a A2. El Bluetooth usa A0 (RX) y A1 (TX).
- El carrito debe rodar paralelo a la cara, con espacio libre para avanzar y
  retroceder.
- Mantén la cara aproximadamente entre 5 y 60 cm del sensor. Unos 15 cm es un
  buen punto de partida.

El ultrasonido mide la separacion lateral entre el carrito y la caja. La longitud
de la cara se estima con el desplazamiento del carrito mientras el sensor detecta
la caja.

## Rama `main`: medir una cara

Abre `Mecanum_Wheels_Robot_Car_medicion_prisma_Medicion_V2.ino` y envia `%W#`
por Bluetooth.

1. Deja el carrito junto a la cara, paralelo a ella, con el sensor apuntando a
   la superficie y detectando la caja.
2. El programa retrocede hasta dejar de detectar la caja: asi busca el primer
   borde.
3. Avanza hasta volver a detectarla y despues sigue avanzando hasta perderla en
   el borde opuesto.
4. Calcula el ancho estimado y lo envia por Bluetooth y por el monitor serial.

La estimacion usa `ancho = tiempo_detectado * FORWARD_SPEED_CM_S -
EDGE_CORRECTION_CM`. No usa encoders; por eso depende de la bateria, el piso, la
carga y la velocidad real de los motores.

## Rama `4-CARAS`: medir las cuatro caras

Esta rama agrega el sketch
`Mecanum_Wheels_Robot_Car_medicion_prisma_4_Caras.ino`. Coloca el carrito antes
de la primera esquina, paralelo a la primera cara y con el sensor apuntando a la
caja. Envia `%W#`.

Para cada cara, el carrito avanza mientras confirma el primer borde, mantiene la
medicion mientras detecta la superficie y confirma el segundo borde al perderla.
Los ecos muy breves (menos de `MIN_FACE_SPAN_MS`) se descartan como posibles
reflejos de esquina. Despues de las primeras tres caras, el carrito avanza para
despejar la esquina y gira a la derecha antes de buscar la cara siguiente.

Los resultados se envian por Bluetooth y por el monitor serial como `Cara 1`,
`Cara 2`, `Cara 3` y `Cara 4`.

## Ajustes principales

| Constante | Uso |
| --- | --- |
| `SCAN_PWM` | Velocidad de avance/retroceso durante el escaneo |
| `FORWARD_SPEED_CM_S` | Velocidad lineal medida para convertir tiempo a centimetros |
| `EDGE_CORRECTION_CM` | Correccion aproximada por la deteccion acustica del borde |
| `TARGET_MIN_CM`, `TARGET_MAX_CM` | Rango valido de distancia al objeto |
| `EDGE_CONFIRMATIONS` | Lecturas consecutivas para confirmar un borde |
| `TURN_90_MS` | Duracion del giro a la derecha en la version de cuatro caras |
| `CLEAR_CORNER_MS` | Tiempo de avance antes de girar |
| `MIN_FACE_SPAN_MS` | Duracion minima para aceptar una deteccion como cara completa |

Calibra `FORWARD_SPEED_CM_S` recorriendo una distancia conocida con el mismo
`SCAN_PWM`, bateria y superficie de uso. En la rama de cuatro caras, calibra
`TURN_90_MS` por separado: el giro es temporizado, no utiliza un giroscopio, y
puede variar con el suelo y la bateria.

## Bluetooth y seguridad

- `%W#`: iniciar la medicion correspondiente a la rama.
- `%S#`: detener motores cuando el programa esta atendiendo comandos.

La medicion es bloqueante: mientras se ejecuta `%W#`, el programa no procesa
`%S#`. Haz las pruebas en un area despejada y mantén accesible el interruptor de
alimentacion.
