"""En este codigo se realizo una integracion del codigo HSVColorBlueCalibartion con el IdTestCamera 1.2
En el codigo primero se realiza la deteccion del tono o color HSV y se actualizan los valores de upper y lower color
En la segund aparte del codigo utiliza el color seleccionado para buscar los bordes del objeto y enmarcarlo
El texto impreso solo esta delimitado a credencial TEC pero el colo si detecta el que sea"""

import cv2
import numpy as np

# Variables globales para valores HSV seleccionados
lower_color = np.array([110, 50, 50])
upper_color = np.array([130, 255, 255])

# Función para detectar objetos del color seleccionado
def detect_color(frame):
    # Convertir de BGR a HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # Threshold la imagen HSV para obtener solo los colores en el rango seleccionado
    mask = cv2.inRange(hsv, lower_color, upper_color)
    
    # Convertir la máscara de un solo canal a tres canales
    mask = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)

    # Asegurarse de que la máscara sea un entero sin signo de 8 bits
    mask = np.uint8(mask)
    
    # Bitwise-AND entre la máscara y la imagen original
    res = cv2.bitwise_and(frame, frame, mask=mask[:,:,0])
    
    return res, mask

# Función para seleccionar el color HSV haciendo doble clic
def on_double_click(event, x, y, flags, param):
    global lower_color, upper_color
    if event == cv2.EVENT_LBUTTONDBLCLK:
        frame = param  # Obtener el frame actual
        # Convertir el píxel seleccionado a HSV
        hsv_pixel = cv2.cvtColor(np.uint8([[frame[y, x]]]), cv2.COLOR_BGR2HSV)[0][0]
        print("Color seleccionado/Selected HSV:", hsv_pixel)
        # Actualizar los valores de color
        hue_adjustment = 20  # Ajuste de valor H, puedes ajustar este valor según tu necesidad
        lower_color = np.array([max(hsv_pixel[0] - hue_adjustment, 0), 50, 50])
        upper_color = np.array([min(hsv_pixel[0] + hue_adjustment, 179), 255, 255])

# Función para calcular la distancia basada en el tamaño aparente
def calculate_distance(apparent_size, real_size_width, real_size_height, focal_length_width, focal_length_height):
    distance_width = (real_size_width * focal_length_width) / apparent_size[0]
    distance_height = (real_size_height * focal_length_height) / apparent_size[1]
    distance = (distance_width + distance_height) / 2  # Distancia promedio entre ancho y alto
    return distance

# Función principal
def main():
    cap = cv2.VideoCapture(0)

    # Real-world size of the object (in inches)
    real_object_width = 3.3  # Example: 3.3 inches
    real_object_height = 2  # Example: 2 inches

    # Focal length of the camera (in pixels) - You need to calibrate your camera to obtain this value
    focal_length_width = 1000  # Example: 1000 pixels
    focal_length_height = 1000  # Example: 1000 pixels
    
    # Load face detection classifier
    face_cascade = cv2.CascadeClassifier('C:/Users/rodri/anaconda3/pkgs/libopencv-4.9.0-qt6_py312hd35d245_612/Library/etc/haarcascades/haarcascade_frontalface_default.xml')

    while True:
        ret, frame = cap.read()
        
        # Mostrar la imagen en la ventana para seleccionar el color
        cv2.imshow('Select Color', frame)
        
        # Esperar a que el usuario seleccione el color y ajustar los valores
        cv2.setMouseCallback('Select Color', on_double_click, frame)
        
        if cv2.waitKey(1) & 0xFF == ord(' '):
            break
    
    # Segunda parte: Deteción de credenciales y caras
    while True:
        # Capture frame-by-frame
        ret, frame = cap.read()
        
        # Detectar objetos del color seleccionado
        color_detected, color_mask = detect_color(frame)
        
        # Encontrar contornos
        contours, _ = cv2.findContours(cv2.cvtColor(color_mask, cv2.COLOR_BGR2GRAY), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Filtrar contornos basados en el área
        filtered_contours = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 1000:  # Ajustar este umbral según el tamaño esperado de la credencial
                filtered_contours.append(contour)
        
        # Dibujar rectángulos alrededor de los objetos y calcular la distancia
        for contour in filtered_contours:
            x,y,w,h = cv2.boundingRect(contour)
            cv2.rectangle(frame,(x,y),(x+w,y+h),(255,0,0),2)
            
            # Calcular el tamaño aparente
            apparent_size = (w, h)
            
            # Calcular la distancia
            distance = calculate_distance(apparent_size, real_object_width, real_object_height, focal_length_width, focal_length_height)
            
            # Mostrar distancia
            cv2.putText(frame, f"Distancia: {distance:.2f} pulgadas", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
            
            # Detectar caras dentro del rectángulo
            roi_frame = frame[y:y+h, x:x+w]
            gray_roi = cv2.cvtColor(roi_frame, cv2.COLOR_BGR2GRAY)
            faces = face_cascade.detectMultiScale(gray_roi, 1.3, 5)
            for (fx, fy, fw, fh) in faces:
                cv2.rectangle(roi_frame, (fx, fy), (fx+fw, fy+fh), (0, 255, 0), 2)
                
                # Verificar si se detecta una cara dentro del rectángulo
                if len(faces) > 0:
                    # Mostrar texto "Estudiante TEC"
                    cv2.putText(frame, "Estudiante TEC", (x, y - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        # Mostrar el marco resultante
        cv2.imshow('Object Detection', frame)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Liberar la captura cuando se termine
    cap.release()
    cv2.destroyAllWindows()

if _name_ == "_main_":
    main()
