import cv2
import numpy as np

# Crear una imagen en negro
height, width = 300, 500  # Tamaño de la ventana
image = np.zeros((height, width, 3), dtype=np.uint8)

def update_color(h, s, v):
    """Función para actualizar el color del rectángulo según los valores HSV ingresados"""
    # Crear un color BGR a partir de HSV
    hsv_color = np.uint8([[[h, s, v]]])  # Matriz de 1x1 en HSV
    bgr_color = cv2.cvtColor(hsv_color, cv2.COLOR_HSV2BGR)  # Convertir a BGR
    b, g, r = int(bgr_color[0][0][0]), int(bgr_color[0][0][1]), int(bgr_color[0][0][2])
    
    # Actualizar la imagen con el color
    image[:] = [b, g, r]
    return b, g, r

# Inicializar con valores de ejemplo
h, s, v = 120, 255, 255  # Un azul brillante
b, g, r = update_color(h, s, v)

while True:

    print(f"Current HSV: {h}, {s}, {v} - Current BGR: {b}, {g}, {r}")
    print("Enter new HSV values (0-179 for H, 0-255 for S and V). Type 'exit' to quit.")
    
    # Pedir al usuario los valores
    user_input = input("Enter HSV values (comma-separated): ")
    if user_input.lower() == "exit":
        break

    try:
        h, s, v = map(int, user_input.split(','))
        if not (0 <= h <= 179 and 0 <= s <= 255 and 0 <= v <= 255):
            print("Values out of range. H: 0-179, S: 0-255, V: 0-255")
            continue

        # Actualizar el color en la imagen
        b, g, r = update_color(h, s, v)

    except ValueError:
        print("Invalid input. Please enter three integers separated by commas.")

    # Actualizar la imagen con el nuevo color
    cv2.imshow("HSV to Color", image)
    cv2.waitKey(1)

cv2.destroyAllWindows()
