from picamera import PiCamera #importa paquete
from time import sleep

camera = PiCamera() #Primero, importamos la clase PiCamera del módulo picamera.
   #Usaremos esa clase para tener acceso a la cámara física.

camera.start_preview() #método para iniciar la visualización de la entrada de la cámara.
sleep(5) #tiempo que esta abierta la visualización
camera.stop_preview() #método para cerrar la visualización de la entrada de la cámara.