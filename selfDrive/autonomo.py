#!/usr/bin/env python3
import numpy as np
import cv2
import math
import time

import RPi.GPIO as GPIO

GPIO.cleanup()
GPIO.setmode(GPIO.BCM)

steering=18
front_RightWheel=19
back_RightWheel=13
front_LeftWheel=6
back_LeftWheel=5
contServo=6.5

GPIO.setup(steering, GPIO.OUT)
GPIO.setup(front_RightWheel, GPIO.OUT)
GPIO.setup(front_LeftWheel, GPIO.OUT)
GPIO.setup(back_RightWheel, GPIO.OUT)
GPIO.setup(back_LeftWheel, GPIO.OUT)

servo = GPIO.PWM(steering, 50)  # channel=18 frequency=50Hz
servo.start(contServo)

GPIO.output(front_RightWheel,False)
GPIO.output(front_LeftWheel,False)
GPIO.output(back_RightWheel,False)
GPIO.output(back_LeftWheel,False)


#Select a region of interest
def region_of_interest(img, vertices):
    #defining a blank mask to start with
    mask = np.zeros_like(img)

    #defining a 3 channel or 1 channel color to fill the mask with depending on the input image
    if len(img.shape) > 2:
        channel_count = img.shape[2]  # i.e. 3 or 4 depending on your image
        ignore_mask_color = (255,) * channel_count
    else:
        ignore_mask_color = 255

    #filling pixels inside the polygon defined by "vertices" with the fill color
    cv2.fillPoly(mask, vertices, ignore_mask_color)

    #returning the image only where mask pixels are nonzero
    masked_image = cv2.bitwise_and(img, mask)
    return masked_image


#Funcion principal
def run_detection(id_cam,param):
    
    cap = cv2.VideoCapture(id_cam)

    start_time = time.time()
    counter=0
    #Cambiar el tamaño de la imagen
    width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)   
    height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT) 
    #cap.set(cv2.CAP_PROP_FPS, 10)

    # Check if camera opened successfully
    if (cap.isOpened()== False): 
        print("Error opening video stream or file")

    # Read until video is completed
    while(cap.isOpened()):
        # Capture frame-by-frame
        ret, img_colour = cap.read()

        if ret == True:


            img_colour_rgb = cv2.cvtColor(img_colour, cv2.COLOR_BGR2RGB)
            img_colour_rgb = cv2.resize(img_colour_rgb, (int(width*param['scale']),int(height*param['scale'])), 
                                                                                interpolation = cv2.INTER_AREA)
            grey = cv2.cvtColor(img_colour_rgb, cv2.COLOR_RGB2GRAY)
            ############################################################
            #Apply Gaussuan smoothing
            kernel_size = (param['kernelSize'],param['kernelSize'])
            #Aplicamos una vez
            blur_grey = cv2.GaussianBlur(grey, kernel_size, sigmaX=0, sigmaY=0)
            #Aplicamos el resto de iteraciones
            for i in range(0, param['iterationKernel']-1):
                blur_grey = cv2.GaussianBlur(blur_grey, kernel_size, sigmaX=0, sigmaY=0)

            ##############################################################
            #Apply Canny edge detector
            low_threshold = param['LowCannyThreshold']
            high_threshold = param['HighCannyThreshold']
            edges = cv2.Canny(blur_grey, low_threshold, high_threshold, apertureSize=3)

            ###############################################################
            #ROI
            vertices = np.dot(param['vertices'],param['scale'])
            masked_edges = region_of_interest(edges, [vertices.astype(int)])
        
            #################################################################
            #Apply Hough transform for lane lines detection
            rho = param['rho']                      # distance resolution in pixels of the Hough grid
            theta = param['theta']                  # angular resolution in radians of the Hough grid
            threshold = param['threshold']          # minimum number of votes (intersections in Hough grid cell)
            min_line_len = param['min_line_len']    # minimum number of pixels making up a line
            max_line_gap = param['max_line_gap']    # maximum gap in pixels between connectable line segments
            hough_lines = cv2.HoughLinesP(masked_edges, rho, theta, threshold, np.array([]), minLineLength=min_line_len, maxLineGap=max_line_gap)

            ##################################################################
            #Separar lineas que pertenecen a carril derecho y al izquierda, tomando en cuenta las pendientes
            if hough_lines is not None:
                #creamos los arreglos para guardar los puntos de las lineas izquierda y derecha
                left_line_x = []
                left_line_y = []
                right_line_x = []
                right_line_y = []

                #For para calcular la pendiente de todas las lineas
                for line in hough_lines:
                    for x1, y1, x2, y2 in line:
                        slope = (y2 - y1) / (x2 - x1) #Pendiente
                        if math.fabs(slope) < param['extremeSlope']: # <-- Only consider extreme slope
                            continue   #Continue para ignorar esta pendiente y seguir con la siguiente
                        if slope <= 0: # <-- Si es negativo es linea del lado izquierda
                            left_line_x.extend([x1, x2])
                            left_line_y.extend([y1, y2])
                        else: # <-- Si no es de la derecha
                            right_line_x.extend([x1, x2])
                            right_line_y.extend([y1, y2])
            
                ##########################################################################################            
                #Crear lineas de ajuste

                if len(left_line_x)>0 and len(left_line_y)>0 and len(right_line_x)>0 and len(right_line_x)>0:
                
                    #Definimos lo minimo y lo maximo que pueden llegar las lineas en Y
                    min_y = int(param['min']*param['scale']) # <-- Limite superior
                    max_y = int(param['max']*param['scale']) # <-- Limite inferior
                    
                    
                    #Obtenemos la linea izquierda que coinciden con las x y las y, mediante una funcion lineal
                    poly_left = np.poly1d(np.polyfit(
                        left_line_y,
                        left_line_x,
                        deg=1
                    ))
                    
                    #Obtener punto de inicio y de final
                    left_x_start = int(poly_left(max_y))
                    left_x_end = int(poly_left(min_y))
                    
                    #Obtenemos la linea derecha que coinciden con las x y las y, mediante una funcion lineal
                    poly_right = np.poly1d(np.polyfit(
                        right_line_y,
                        right_line_x,
                        deg=1
                    ))
                    
                    #Obtener punto de inicio y de final
                    right_x_start = int(poly_right(max_y))
                    right_x_end = int(poly_right(min_y))              
                    

                    #####################################################################################
                    #Dibujar todas las lineas que obtenemos de hough
                    img_colour_with_lines = img_colour_rgb.copy()
                    for line in hough_lines:
                        for x1, y1, x2, y2 in line:
                            cv2.line(img_colour_with_lines, (x1, y1), (x2, y2), (0,255,0), 2)
                            
                    #####################################################################################
                    #Dibujar las lineas definidas
                    img_colour_with_Definelines = img_colour_rgb.copy()
                    cv2.line(img_colour_with_Definelines, (left_x_start, max_y), 
                                                          (left_x_end, min_y), (255,0,0), 5)

                    cv2.line(img_colour_with_Definelines, (right_x_start, max_y), 
                                                          (right_x_end, min_y), (255,0,0), 5)

                    ######################################################################################
                    #Definir trayectoria Desesada
                    startTrayectoria=left_x_start+((right_x_start-left_x_start)/2)
                    endTrayectoria=left_x_end+((right_x_end-left_x_end)/2)

                    cv2.line(img_colour_with_Definelines, (int(startTrayectoria), int(height*param['scale'])), 
                                                          (int(endTrayectoria), min_y),(255,125,0), 5)

                    ######################################################################################
                    #Definir trayectoria Actual
                    endTrayectoriaActual=int(
                          ((width*param['scale'])/2)*np.sin(np.deg2rad(0))+(width*param['scale'])/2
                        )

                    cv2.line(img_colour_with_Definelines, (int((width*param['scale'])/2), int(height*param['scale'])), 
                                                          (endTrayectoriaActual, min_y),(255,125,255), 5)

                   
                    error=(int(endTrayectoria-endTrayectoriaActual))
                    print(error)
                    
                    if(error > 18):
                        print ("Derecha") 
                        servo.ChangeDutyCycle(9.5)
                        GPIO.output(front_RightWheel,True)
                        GPIO.output(front_LeftWheel,True)
                        GPIO.output(back_RightWheel,False)
                        GPIO.output(back_LeftWheel,False) 
                        time.sleep(.1)
                        servo.ChangeDutyCycle(6.5)
                        
                        GPIO.output(front_RightWheel,False)
                        GPIO.output(front_LeftWheel,False)
                        GPIO.output(back_RightWheel,False)
                        GPIO.output(back_LeftWheel,False)
                    else:
                        if(error < -18):
                            print ("izquierda") 
                            servo.ChangeDutyCycle(5)
                            GPIO.output(front_RightWheel,True)
                            GPIO.output(front_LeftWheel,True)
                            GPIO.output(back_RightWheel,False)
                            GPIO.output(back_LeftWheel,False) 
                            time.sleep(.1)
                            servo.ChangeDutyCycle(6.5)
                            
                            GPIO.output(front_RightWheel,False)
                            GPIO.output(front_LeftWheel,False)
                            GPIO.output(back_RightWheel,False)
                            GPIO.output(back_LeftWheel,False)
                        else:
                            print ("Front")
                            servo.ChangeDutyCycle(6.5)
                            GPIO.output(front_RightWheel,True)
                            GPIO.output(front_LeftWheel,True)
                            GPIO.output(back_RightWheel,False)
                            GPIO.output(back_LeftWheel,False) 
                            
                    
                    # Display the resulting frame
                    cv2.namedWindow('masked_edges', cv2.WINDOW_NORMAL)
                    cv2.imshow('masked_edges', masked_edges)
                    cv2.namedWindow('img_colour_with_lines', cv2.WINDOW_NORMAL)
                    cv2.imshow('img_colour_with_lines', cv2.cvtColor(img_colour_with_lines, cv2.COLOR_BGR2RGB))
                    cv2.namedWindow('img_colour_with_Definelines', cv2.WINDOW_NORMAL)
                    cv2.imshow('img_colour_with_Definelines', cv2.cvtColor(img_colour_with_Definelines, cv2.COLOR_BGR2RGB))
                    

            else:
                print('=========')
                print('= Frame Perdido =')
                print('=========')
                
                servo.ChangeDutyCycle(6.5)
                GPIO.output(front_RightWheel,False)
                GPIO.output(front_LeftWheel,False)
                GPIO.output(back_RightWheel,True)
                GPIO.output(back_LeftWheel,True)
                
                time.sleep(.8)
                
                GPIO.output(front_RightWheel,True)
                GPIO.output(front_LeftWheel,True)
                GPIO.output(back_RightWheel,False)
                GPIO.output(back_LeftWheel,False) 
                


            counter+=1
            if (time.time() - start_time) > 1 :
                print("FPS: ", counter / (time.time() - start_time))
                counter = 0
                start_time = time.time()

            # Press Q on keyboard to  exit
            if cv2.waitKey(25) & 0xFF == ord('q'):
                break
 
        # Break the loop
        else: 
            break
 
    # When everything done, release the video capture object
    cap.release()
 
    # Closes all the frames
    cv2.destroyAllWindows()
        
    return None

id_cam = 0
              
parameters={
            'rho':1,    # distance resolution in pixels of the Hough grid
            'theta':np.pi/180, # angular resolution in radians of the Hough grid
            'threshold':50, # minimum number of votes (intersections in Hough grid cell)
            'min_line_len':4,# minimum number of pixels making up a line
            'max_line_gap':120,# maximum gap in pixels between connectable line segments
            'vertices': np.array([(0, 327),(190, 243),(431, 243), (635, 327)], dtype=np.int32), #Vertices del ROI
            'min':243,      #Limite min de las lineas
            'max':327,      #Límite max de las lineas
            'scale':(1/2),    #Escala de la imagen
            'kernelSize':3,  #Tamaño del filtro gausiano
            'iterationKernel':3, #Número de iteraciones del filtro gausiano
            'LowCannyThreshold':10,
            'HighCannyThreshold':70, 
            'extremeSlope':0.1 #Threshold de pendientes para las lineas definidas, 0.1 empieza a funcionar
            } 

run_detection(id_cam,parameters)
