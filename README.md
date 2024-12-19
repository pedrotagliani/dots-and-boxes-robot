<h1 align="center">D&B Robot</h1>

<p>
El proyecto consiste en el desarrollo de un brazo robótico de cuatro grados de libertad diseñado para competir contra un oponente humano en el juego de lápiz y papel "Puntos y Cajas". Utiliza visión artificial con marcadores ArUco, algoritmos de inteligencia artificial para estrategias de juego, y un sistema de control para mover el brazo de forma precisa. El robot puede identificar líneas dibujadas por el jugador, analizar el estado del tablero y realizar movimientos autónomos utilizando un marcador integrado en su gripper.
</p>

---

## Diseño mecánico

El diseño mecánico del brazo robótico se centró en garantizar precisión y estabilidad para las tareas requeridas. Los principales componentes son:

1. **Articulaciones**:
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





