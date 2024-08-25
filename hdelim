from time import sleep, time
import random
import win32api
import win32gui
import win32ui
import win32con
import numpy as np
import cv2 as cv
from PIL import Image
import os
import time

class WindowCapture:
    w = 0
    h = 0
    hwnd = None
    # scale_factor = 1.25 # Defina o fator de escala conforme a configuração do Windows

    def __init__(self, window_name):
        self.hwnd = win32gui.FindWindow(None, window_name)
        if not self.hwnd:
            raise Exception('Window not found: {}'.format(window_name))

        window_rect = win32gui.GetWindowRect(self.hwnd) #auto size
        self.w = window_rect[2] - window_rect[0] #/self.scale_factor Ajusta a largura pela escala
        self.h = window_rect[3] - window_rect[1] #/self.scale_factor
        #self.w = int(self.w)  # Converte para inteiro pq qualquer operação int gera float
        #self.h = int(self.h)  # Converte para inteiro

        # self.w =  1366
        # self.h =  768

        #realmente só tira borda, não atrapalha
        # border_pixels = 8 #border_pixels: Representa a largura da borda da janela em pixels (8 pixels de cada lado).
        # titlebar_pixels = 30 #titlebar_pixels: Representa a altura da barra de título da janela em pixels (30 pixels).
        # self.w = self.w - (border_pixels * 2)
        # self.h = self.h - titlebar_pixels - border_pixels
        # self.cropped_x = border_pixels
        # self.cropped_y = titlebar_pixels

    def get_screenshot(self):
        wDC = win32gui.GetWindowDC(self.hwnd)
        dcObj = win32ui.CreateDCFromHandle(wDC)
        cDC = dcObj.CreateCompatibleDC()
        dataBitMap = win32ui.CreateBitmap()
        dataBitMap.CreateCompatibleBitmap(dcObj, self.w, self.h)
        cDC.SelectObject(dataBitMap)
        #cDC.BitBlt((0, 0), (self.w, self.h), dcObj, (self.cropped_x, self.cropped_y), win32con.SRCCOPY)
        #captura toda janela, mas ainda assim img ta ampliada! nao resolve
        cDC.BitBlt((0, 0), (self.w, self.h), dcObj, (0, 0), win32con.SRCCOPY)

#problema de distorção mais não poderia estar nessa parte. resolvido com resolução windows
        signedIntsArray = dataBitMap.GetBitmapBits(True)
        img = np.frombuffer(signedIntsArray, dtype='uint8')
        img.shape = (self.h, self.w, 4)

        dcObj.DeleteDC()
        cDC.DeleteDC()
        win32gui.ReleaseDC(self.hwnd, wDC)
        win32gui.DeleteObject(dataBitMap.GetHandle())

        img = img[...,:3]
        img = np.ascontiguousarray(img) 
            
        return img

    def generate_image_dataset(self):
        if not os.path.exists("images"):
            os.mkdir("images")
        while(True):
            img = self.get_screenshot()
            im = Image.fromarray(img[..., [2, 1, 0]])
            im.save(f"./images/img_{len(os.listdir('images'))}.jpeg")
            sleep(1)
    
    def get_window_size(self):
        return (self.w, self.h)
    
# # Testando a captura de tela
# window_name = "Exiled Kingdoms"  # Nome da janela a ser capturada
# wc = WindowCapture(window_name)
# screenshot = wc.get_screenshot()  # Captura de tela
# cv.imshow('Screenshot Test', screenshot)
# # Aguarda uma tecla para fechar a janela de visualização
# cv.waitKey(0)
# cv.destroyAllWindows()

class ImageProcessor:
    W = 0
    H = 0
    net = None
    ln = None
    classes = {}
    colors = []

    def __init__(self, img_size, cfg_file, weights_file):
        np.random.seed(42)
        self.net = cv.dnn.readNetFromDarknet(cfg_file, weights_file)
        self.net.setPreferableBackend(cv.dnn.DNN_BACKEND_OPENCV)
        self.ln = self.net.getLayerNames()
        self.ln = [self.ln[i-1] for i in self.net.getUnconnectedOutLayers()]
        self.W = img_size[0] #1280 mexe na posição do quadrado com base no tamanho da imagem, mas nao manipula a imagem
        self.H = img_size[1] #720
        
        with open('yolov4-tiny/obj.names', 'r') as file:
            lines = file.readlines()
        for i, line in enumerate(lines):
            self.classes[i] = line.strip()
        
        # If you plan to utilize more than six classes, please include additional colors in this list.
        self.colors = [
            (0, 0, 255), 
            (0, 255, 0), 
            (255, 0, 0), 
            (255, 255, 0), 
            (255, 0, 255), 
            (0, 255, 255)
        ]
        
    #window size define above shoul be something like:
        #self.window_size = (1280, 720)  # Change this to your desired size

    def proccess_image(self, img):

        blob = cv.dnn.blobFromImage(img, 1/255.0, (416, 416), swapRB=True, crop=False)
        self.net.setInput(blob)
        outputs = self.net.forward(self.ln)
        outputs = np.vstack(outputs)
        
        coordinates = self.get_coordinates(outputs, 0.5)

        self.draw_identified_objects(img, coordinates)

        return coordinates

    def get_coordinates(self, outputs, conf):

        boxes = []
        confidences = []
        classIDs = []

        for output in outputs:
            scores = output[5:]
            
            classID = np.argmax(scores)
            confidence = scores[classID]
            if confidence > conf:
                x, y, w, h = output[:4] * np.array([self.W, self.H, self.W, self.H])
                p0 = int(x - w//2), int(y - h//2)
                boxes.append([*p0, int(w), int(h)])
                confidences.append(float(confidence))
                classIDs.append(classID)

        indices = cv.dnn.NMSBoxes(boxes, confidences, conf, conf-0.1)

        if len(indices) == 0:
            return []

        coordinates = []
        for i in indices.flatten():
            (x, y) = (boxes[i][0], boxes[i][1])
            (w, h) = (boxes[i][2], boxes[i][3])

            coordinates.append({'x': x, 'y': y, 'w': w, 'h': h, 'class': classIDs[i], 'class_name': self.classes[classIDs[i]]})
        return coordinates

    def draw_identified_objects(self, img, coordinates):
        for coordinate in coordinates:
            x = coordinate['x']
            y = coordinate['y']
            w = coordinate['w']
            h = coordinate['h']
            classID = coordinate['class']
            
            color = self.colors[classID]
            
            cv.rectangle(img, (x, y), (x + w, y + h), color, 2)
            cv.putText(img, self.classes[classID], (x, y - 10), cv.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        
        #resize image to window        
        #img_resized = cv.resize(img, self.window_size)
        #cv.imshow('window', img_resized)

        cv.imshow('window',  img)
#mouse control ##clique ainda errado
from pynput.mouse import Button, Controller


# Classe WindowCapture e ImageProcessor devem estar definidas aqui

# Iniciar o controlador do mouse
mouse = Controller()

# Definindo o nome da janela e arquivos de configuração
window_name = "Exiled Kingdoms"
cfg_file_name = "./yolov4-tiny/yolov4-tiny-custom.cfg"
weights_file_name = "yolov4-tiny-custom_last.weights"

# Inicializar captura de janela e processador de imagem
wincap = WindowCapture(window_name)
improc = ImageProcessor(wincap.get_window_size(), cfg_file_name, weights_file_name)
 
while True:
    # Captura de tela da janela do jogo
    ss = wincap.get_screenshot()
    
    if cv.waitKey(1) == ord('q'):
        cv.destroyAllWindows()
        break

    # Processa a imagem para obter as coordenadas dos mobs detectados
    coordinates = improc.proccess_image(ss)
    
     # Mostrar coordenadas detectadas
    for coordinate in coordinates:
        print(coordinate)
    print()
    
    # If you have limited computer resources, consider adding a sleep delay between detections.
    time.sleep(0.2)

    # Se nenhuma coordenada foi detectada, continue o loop
    if len(coordinates) == 0:
        continue

    # Obtendo a primeira coordenada detectada
    attack_mob = coordinates[0]

        #necessário para identificar coordenadas relativa a janela do jogo. antes disso estava considerando tela do windows e clicando fora. maior robustez
    window_rect = win32gui.GetWindowRect(wincap.hwnd)
    window_x = window_rect[0]
    window_y = window_rect[1]

        # Calculando o centro do retângulo detectado dicionar metade da largura (w/2) e metade da altura (h/2) do retângulo à coordenada inicial (x, y)
    center_x = attack_mob['x'] + attack_mob['w'] // 2
    center_y = attack_mob['y'] + attack_mob['h'] // 2

 # Convertendo coordenadas da janela para coordenadas da tela
    screen_x = window_x + center_x
    screen_y = window_y + center_y

        # Definindo a posição do mouse para a coordenada do mob (somente x e y). não funcionava, coordenadas erradas
        #mouse.position = (atack_mob['x'], atack_mob['y'])
    mouse.position = (screen_x, screen_y)
    mouse.click(Button.left)
        
    # Espera de 5 segundos antes de continuar o loop
    time.sleep(5)
    

print('Finished.')
