import tkinter as tk
import cv2
from PIL import Image, ImageTk
import time
import serial
import board
import busio
import firebase_admin
from firebase_admin
import credentials, db, firestore, storage
import os
from adafruit_vl53l0x import VL53L0X
from threading import Thread
import smbus2
import math

# Configuracion de la camara y el modelo de deteccion
SelectCamara = 0
Camera = cv2.VideoCapture(SelectCamara)
net = cv2.dnn.readNetFromCaffe("deploy.prototxt","res10_300x300_ssd_iter_140000_fp16.caffemodel")

# Parametros del modelo
in_width = 200
in_height = 200
mean = [104, 117, 123]
conf_threshold = 0.7

# ==============================
# I2C BUS
# ==============================

bus = smbus2.SMBus(1)

# ==============================
# MAGNETOMETRO AK8963
# ==============================

MAG_ADDR = 0x0C
AK8963_CNTL1 = 0x0A
AK8963_XOUT_L = 0x03

# ---- CALIBRACION ----
mx_offset = -229
my_offset = 492

scale_x = 1.038
scale_y = 0.964

# Encender magnetometro
bus.write_byte_data(MAG_ADDR, AK8963_CNTL1, 0x16)
time.sleep(0.1)

# ==============================
# SENSOR VL53L0X
# ==============================

i2c = busio.I2C(board.SCL, board.SDA)
vl53 = VL53L0X(i2c)

# modo largo alcance
vl53.signal_rate_limit = 0.1
vl53.measurement_timing_budget = 200000

print("VL53L0X modo largo alcance activado")
#====================================
# Inicializar Firebase
#====================================
cred = credentials.Certificate("/home/pi/Documents/App/clav.json")
firebase_admin.initialize_app(cred, {
    'databaseURL': 'https://miamav-6adc2-default-rtdb.firebaseio.com/',
    'storageBucket': 'miamav-6adc2.appspot.com' })

# ==============================
# FILTROS
# ==============================

alpha_dist = 0.3
distancia_filtrada = 0

alpha_angle = 0.90
filtered_heading = 0
first = True
   
#========================================================
# Configuracion de comunicacion serial con Arduino
arduino = serial.Serial('/dev/ttyACM0', 9600, timeout=1)  # Puerto serial de la Raspberry Pi
def adelante_balancing():
    # Envia el comando 'F:front' al Arduino 
    arduino.write(b'F\n')
    print("Enviando comando 'Adelante' al Arduino...")

def detener_balancing():

    # Envia el comando 'S: stop' al Arduino para detener
    arduino.write(b'S\n')
    print("Enviando comando 'Stop' al Arduino...")
def giro_derecha_balancing():
    # Envia el comando 'R:Right' al Arduino 
    arduino.write(b'R\n')
    print("Enviando comando 'Giro Derecha' al Arduino...")

def giro_izquierda_balancing():
    # Envia el comando 'L: Left' al Arduino para detener
    arduino.write(b'L\n')
    print("Enviando comando 'Giro Izquierda' al Arduino...")
def atras_balancing():
    # Envia el comando 'B:Back' al Arduino para detener
    arduino.write(b'B\n')
    print("Enviando comando 'Atras' al Arduino...")
    
# ==============================
# FUNCIONES MAGNETOMETRO
# ==============================

def read_mag_raw():

    data = bus.read_i2c_block_data(MAG_ADDR, AK8963_XOUT_L, 7)

    x = (data[1] << 8) | data[0]
    y = (data[3] << 8) | data[2]
    z = (data[5] << 8) | data[4]

    if x >= 0x8000: x -= 65536
    if y >= 0x8000: y -= 65536
    if z >= 0x8000: z -= 65536

    return x, y, z


def get_heading():

    mx, my, mz = read_mag_raw()

    # Hard iron correction
    mx = (mx - mx_offset) * scale_x
    my = (my - my_offset) * scale_y

    yaw = math.degrees(math.atan2(my, mx))

    if yaw < 0:
        yaw += 360

    return yaw


# ==============================
# FILTRO ANGULO CIRCULAR
# ==============================

def circular_complementary_filter(prev_angle, new_angle, alpha=0.90):

    diff = new_angle - prev_angle

    if diff > 180:
        new_angle -= 360
    elif diff < -180:
        new_angle += 360

    filtered = prev_angle * alpha + new_angle * (1 - alpha)

    filtered %= 360

    return filtered


# ==============================
# LECTURA DE SENSORES
# ==============================
def Sensores():
    global distancia_filtrada
    global filtered_heading
    global first
    while True:

        # ---------- MAGNETOMETRO ----------
        natural = get_heading()

        if first:
            filtered_heading = natural
            first = False

        filtered_heading = circular_complementary_filter(filtered_heading, natural)

        # ---------- SENSOR DISTANCIA ----------
        distancia_mm = vl53.range

        if distancia_mm > 2000:
            continue

        distancia_cm = distancia_mm / 10

        distancia_filtrada = alpha_dist * distancia_cm + (1 - alpha_dist) * distancia_filtrada
        
        if distancia_filtrada < 40:  # Usar un margen de error para la comparacion
        print("Proceso Reconocimiento")
        time.sleep(0.5)
        Detectar()
        
        # ---------- MOSTRAR DATOS ----------

        yaw_label.config(text=f"Angulo Yaw:{filtered_heading:.2f} ")
        distancia_label.config(text=f"Distancia:{distancia_filtrada:.2f} cm")
        time.sleep(0.1)
        
    #------------------ CONDICIONES DE NAVEGACION -----------------
    def nav_inercial(yaw, target_angle, tolerance=5):
    "tolerance: margen para considerar alineado"
    # Normaliza la diferencia entre -180 y 180
        diff = (target_angle - yaw + 360) % 360
        if diff > 180:
        diff -= 360
        adelante_balancing()
        time.sleep(0.5)
        # Decidir accion
        if abs(diff) <= tolerance:
            adelante_balancing()
        elif diff > 0:
            giro_derecha_balancing()
        else:
            giro_izquierda_balancing()


def Detectar():
    global camara_activa
    #Condiciones para activar camara
    if not camara_activa:
        return
        
    success, frame = Camera.read()
    if success:
        frame = cv2.flip(frame, 1)
        frame_height, frame_width = frame.shape[:2]
        blob = cv2.dnn.blobFromImage(frame, 1.0, (in_width, in_height), mean, swapRB=False, crop=False)
        net.setInput(blob)
        detections = net.forward()
        confidence = detections[0, 0, 0, 2]
        if confidence > conf_threshold:
            x_left_bottom = int(detections[0, 0, 0, 3] * frame_width)
            y_left_bottom = int(detections[0, 0, 0, 4] * frame_height)
            x_right_top = int(detections[0, 0, 0, 5] * frame_width)
            y_right_top = int(detections[0, 0, 0, 6] * frame_height)
            cv2.rectangle(frame, (x_left_bottom, y_left_bottom), (x_right_top, y_right_top), (0, 255, 0), 2)
            label = "Confianza: %.2f" % confidence
            cv2.putText(frame, label, (x_left_bottom, y_left_bottom - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        img = Image.fromarray(img)
        imgtk = ImageTk.PhotoImage(image=img)
        camera_label.imgtk = imgtk
        camera_label.configure(image=imgtk)
    
    
        if confidence >= 0.94659567:
        cv2.imwrite('C:/Users/elica/Documents/TesisEnProceso/Final/Detect.jpg', frame)
        # Actualizar la etiqueta de deteccion
            estado_label.config(text="Persona Detectada. Deteniendo Robot")
            detect= "Persona detectada"
            camara_activa = True  # sigue activa
            # Llamar a la funcion detener_balancing
            detener_balancing()
            sleep(0.5)
            atras_balancing()
            sleep(0.1)
        
            
        elif confidence <= 0.93109874:        
            cv2.imwrite('C:/Users/elica/Documents/TesisEnProceso/Final/NoDetect.jpg', frame)
            print("Persona no Detectada. Continuando navegacion.")
            nav_inercial(yaw, target_angle, tolerance=5)
            sleep(0.5)
            estado_label.config(text="Persona no Detectada. Continuando navegacion")
            camara_activa = False
            Camera.release()  # Apaga la camara
            return
            
    
    ventana.after(10, Detectar)

def Presentacion():
    global ventana
    global camera_label
    global distancia_label
    global yaw_label
    ventana = tk.Tk()
    ventana.title("Reconocimiento de Persona")
    ventana.geometry('1100x550')
    Sensores()
    
    # Label para mostrar la imagen de la camara
    camera_label = tk.Label(ventana)
    camera_label.place(x=400, y=10)
    
    
    # Crear la etiqueta de "Resultados"
    etq = tk.Label(ventana, text="Resultados", foreground="#FFFFFF", font=14, height=2, width=32, bg='#002051')
    etq.place(x=50, y=50)
    # Etiqueta para mostrar el Angulo yaw
    yaw_label = tk.Label(ventana, text="Angulo Yaw: ", font=12, bg='#002051', fg='#FFFFFF')
    yaw_label.place(x=50, y=120)
    # Crear la etiqueta para mostrar la distancia
    distancia_label = tk.Label(ventana, text="Distancia Robot-Persona: ", font=12, bg='#002051', fg='#FFFFFF')
    distancia_label.place(x=50, y=150)  # Posicionar la etiqueta debajo de "Resultados" 
    
    # Crear la etiqueta para mostrar estado
    estado_label = tk.Label(ventana, text="", font=14, bg='#002051', fg='#FFFFFF')
    estado_label.place(x=50, y=250)
    Thread(target=Sensores, daemon=True).start()
    ventana.protocol("WM_DELETE_WINDOW", on_closing)
    ventana.mainloop()

def on_closing():
    Camera.release()
    cv2.destroyAllWindows()
    ventana.destroy()
# -----------------------------
# ENVIAR DATOS FIREBASE
# -----------------------------
def send_status(distancia, estado):
    ref = db.reference('deteccion')
    data = {
        'estado': estado, # "Persona detectada"
        'distancia': distancia, # ejemplo: 120 cm
        'timestamp': int(time.time())
        }
    ref.push(data)
    print("Datos enviados:", data)
# -----------------------------
# SUBIR IMAGEN A STORAGE
# -----------------------------
def upload_image(image_path):
    bucket = storage.bucket()
    blob = bucket.blob(os.path.basename(image_path))
    blob.upload_from_filename(image_path)
    blob.make_public()
    print("Imagen subida:", blob.public_url)
    return blob.public_url
# -----------------------------
# GUARDAR URL EN REALTIME DB
# -----------------------------
def save_image_url(image_url):
    ref = db.reference('images')
    ref.push({
        'url': image_url,
        'timestamp': int(time.time())
    })
# -----------------------------
# GUARDAR URL EN FIREBASE
# -----------------------------
def save_image_url_firestore(image_url):
    firestore_db = firestore.client()
    firestore_db.collection('images').add({
        'url': image_url,
        'timestamp': int(time.time()) })
# -----------------------------
distancia = distancia_filtrada 
estado = detec
# Enviar estado y distancia
send_status(distancia, estado)
# Subir imagen
image_url = upload_image('/home/pi/Desktop/Detect.jpg')
# Guardar URL
save_image_url(image_url)
save_image_url_firestore(image_url)
#----------------------------------
target_angle = 0 "angulo deseado (0-360 grados)"
nav_inercial(yaw, target_angle, tolerance=5)
    
Presentacion()