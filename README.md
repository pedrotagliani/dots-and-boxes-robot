<h1 align="center">D&B Robot</h1>

<p>
El proyecto consiste en el desarrollo de un brazo robótico de cuatro grados de libertad diseñado para competir contra un oponente humano en el juego de lápiz y papel "Puntos y Cajas". Utiliza visión artificial con marcadores ArUco, algoritmos de inteligencia artificial para estrategias de juego, y un sistema de control para mover el brazo de forma precisa. El robot puede identificar líneas dibujadas por el jugador, analizar el estado del tablero y realizar movimientos autónomos utilizando un marcador integrado en su gripper.
</p>

---

## Diseño mecánico

El diseño mecánico del brazo robótico se centró en garantizar precisión y estabilidad para las tareas requeridas. Los principales componentes son:

1. **Articulaciones del brazo**:
   - **Base**: Soporta el brazo y aloja los componentes electrónicos. Incluye un soporte para el motor paso a paso y puntos de fijación para garantizar la estabilidad. Además, se incorporó una crapodina para disminuir el rozamiento y mejorar la fijación del motor.
   - **Hombro y codo**: Incorporan soportes para servomotores MG996R, espacios para encoders y soportes para resortes que reducen la carga sobre los motores.
   - **Gripper**: Diseñado para manejar un marcador (fibrón), incluye un sistema deslizante y un resorte amortiguador para asegurar un contacto adecuado con la superficie de dibujo.

2. **Mesa y Topes**: 
   - Proporcionan soporte adicional, fijando el brazo a la mesa donde se encuentra la pizarra.

3. **Materiales y Diseño**: 
   - Todas las piezas fueron modeladas en SolidWorks, priorizando un diseño modular y funcional.

---

## Diseño eléctrico

El sistema eléctrico asegura el funcionamiento coordinado de motores, sensores y el control del brazo. Los principales elementos son:

1. **Fuente de Alimentación**:
   - Una fuente de 12V y 3A alimenta el sistema.
   - Un regulador step-down reduce el voltaje a 5V para los servomotores.

2. **Controladores**:
   - **PCA9685**: Gestiona los servomotores mediante señales PWM.
   - **A4988**: Controla el motor paso a paso NEMA 17.

3. **Sensores**:
   - **Encoders magnéticos**: Miden las posiciones angulares de las articulaciones.
   - **Switch final de carrera**: Establece la posición inicial del motor paso a paso.

4. **Microcontrolador ESP32**:
   - Actúa como el cerebro del sistema.
   - Coordina la comunicación y el control de motores y sensores mediante bus I2C y señales digitales.

5. **Cableado**:
   - Se utilizó un diseño modular para facilitar la organización y el mantenimiento del sistema.

6. **Motores**:
   - **MG996R**: Se agregaron servomotores en las articulaciones del hombro, codo y efector final.
   - **NEMA 17**: Se agregó el paso a paso en la base del robot. 

El diseño eléctrico es robusto y escalable, permitiendo ajustes futuros según las necesidades del sistema.

<img src="https://github.com/pedrotagliani/dots-and-boxes-robot/blob/main/Images/Esquema%20Brazo%20Robot.png" alt="Esquema de conexión del robot"/>

---

## Diseño de software

### Calibración de la Cámara
El proceso de calibración de la cámara se realizó utilizando la librería OpenCV y un tablero ChArUco. Este patrón híbrido combina características de los tableros de ajedrez y los marcadores ArUco, permitiendo una detección precisa y robusta. Para la calibración, se tomaron múltiples imágenes del tablero desde diferentes ángulos y distancias, las cuales se procesaron para calcular parámetros intrínsecos de la cámara, como la distancia focal, el centro óptico y los coeficientes de distorsión. Estos parámetros resultaron esenciales para corregir distorsiones en las imágenes capturadas y calcular con precisión la pose de los marcadores en operaciones posteriores.

### Detección del Tablero
La detección del tablero de juego es una de las etapas clave del sistema. Los cuatro marcadores ArUco ubicados en las esquinas de la pizarra delimitan su área de interés. Cada marcador tiene un ID único, lo que permite identificarlos correctamente y evitar falsos positivos. Una vez detectados, se recorta la imagen del tablero y se calculan las coordenadas de todos los puntos de la cuadrícula, referenciándolos a las posiciones de los marcadores. Para mejorar la precisión, se utilizó un promedio de mediciones obtenidas en capturas consecutivas, reduciendo así el impacto de variaciones en las condiciones de detección, como la iluminación o el ruido en la imagen.

### Detección de las Líneas
La detección de líneas se realiza utilizando los puntos del tablero previamente calculados. Para cada línea, se seleccionan los dos puntos que la delimitan y se recorta la región de interés correspondiente en la imagen. Este recorte se procesa mediante técnicas de thresholding y se aplica la función HoughLinesP de OpenCV para identificar la línea. En el caso de las líneas dibujadas por el robot, que son de color rojo, se utilizan máscaras de color para garantizar su correcta detección. Este enfoque también permite adaptar el sistema a otros colores en caso de necesitarlo. La precisión de esta etapa es fundamental para garantizar que el juego se desarrolle sin interrupciones.

### Motor de Juego
El motor del juego fue implementado utilizando la librería `dnbpy`, que gestiona la lógica del juego "Puntos y Cajas". Esta librería incluye diferentes niveles de dificultad para el oponente robótico: desde movimientos aleatorios hasta estrategias avanzadas basadas en el algoritmo Minimax con poda alfa-beta. Para integrar la librería al sistema, se adaptaron las convenciones de referencia de las líneas del tablero a los estándares del motor de juego. Esto permitió controlar el flujo de la partida, consultar el estado del tablero y realizar movimientos de manera estratégica.

### Comunicación con el Brazo Robótico
La comunicación entre el software en Python y el microcontrolador ESP32 se logró mediante un enlace serial. Desde Python se generaron comandos que el ESP32 interpretó para ejecutar movimientos o realizar lecturas de sensores. Este sistema de comunicación bidireccional permitió enviar instrucciones al robot y recibir confirmaciones de ejecución o datos de estado, asegurando una sincronización efectiva entre las capas de software y hardware.

### Control del Robot
El brazo robótico inicia su funcionamiento posicionándose en "home". Para los servomotores, esta posición se establece mediante encoders internos que proporcionan la retroalimentación necesaria. El motor paso a paso, en cambio, utiliza un switch final de carrera para identificar su punto de referencia inicial. Desde esta posición, el robot espera recibir instrucciones desde el software. Los motores son controlados utilizando librerías especializadas como `AccelStepper`, que gestiona el motor paso a paso, y `ServoEasing`, que facilita interpolaciones suaves en los movimientos de los servomotores. Además, se implementaron mecanismos de verificación con encoders externos para garantizar la precisión en el posicionamiento inicial.

### Funcionamiento y Uso
El sistema fue diseñado para ser intuitivo y fácil de usar. Al inicio del juego, se solicita al usuario que verifique las condiciones de iluminación y que los marcadores ArUco sean visibles. Una vez confirmada la detección de los marcadores, el tablero debe estar limpio para comenzar la partida. El juego se desarrolla alternando turnos entre el jugador humano y el robot, quien actualiza el marcador en tiempo real tras cada movimiento. Al finalizar la partida, el sistema imprime el resultado final, proporcionando un resumen del desempeño de ambos participantes.

Este sistema integra todas las etapas de software en una solución robusta, combinando visión artificial, lógica de juego y control robótico para garantizar una experiencia interactiva y eficiente.

<img src="https://github.com/pedrotagliani/dots-and-boxes-robot/blob/main/Images/Line%20detection.png" alt="Detección de líneas"/>
