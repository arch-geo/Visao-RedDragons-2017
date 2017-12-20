# -*- coding: utf-8 -*-
"""
Created on Thu Jan 19 19:53:02 2017

@author: geo
"""

import cv2
import imutils
import numpy as np
import copy
import os.path
import filtro
import time


##############
# NO MAIN, ADQUIRIMOS O ORIGINAL 'SNAP' 
# NA CALIBRAÇÃO, TRABALHAMOS COM 'SNAPE', UMA CÓPIA DE 'SNAP' QUE PODEMOS ALTERAR 
#For HSV, Hue range is [0,179], Saturation range is [0,255] and Value range is [0,255]
##############

#==== [FUNÇÃO AUXILIAR, RETORNA NADA] =========================================
def nothing(x):     # Essa função é necessária para construir os trackbars
    pass


#==== [PERMITE VISUALIZAR IMAGENS NA INTERFACE GRÁFICA] =======================
def visualizacao(snap):    
    global snape
    snape = copy.deepcopy(snap)        # Usado para desenhar as referencias visuais

#==== [SALVA POSICÃO COM CLIQUE DUPLO DO MOUSE] ===============================
def mouse_click(event,x,y,flags,param):     
    global mouseX,mouseY
    if event == cv2.EVENT_LBUTTONDBLCLK:
        cv2.circle(snape,(x,y),3,(255,255,0),-1)
        mouseX,mouseY = x,y


#==== [ABRE A CAMERA E TIRA FOTO PARA TESTAR] =================================
def snapshot(video, M):
    cv2.namedWindow('Câmera')
    cv2.moveWindow('Câmera',200,200)
    cap = cv2.VideoCapture(video)
    frame_counter = 0
    ret, snap = cap.read()
    
    while M==False:        
        
        ret, snap = cap.read()
        
        frame_counter += 1
        
        # Permite rodar arquivos de vídeo em loop caso não use a câmera.
        if imutils.is_cv2() and frame_counter == cap.get(cv2.cv.CV_CAP_PROP_FRAME_COUNT)-3:   #Conta frames para recomeçar o vídeo quando acabar
            frame_counter = 0
            cap.set(cv2.cv.CV_CAP_PROP_POS_FRAMES, 0)  
        
        elif imutils.is_cv3() and frame_counter == cap.get(cv2.CAP_PROP_FRAME_COUNT)-3:
            frame_counter = 0
            cap.set(cv2.CAP_PROP_POS_FRAMES, 1) 

        if ret == False:
            print ('Falha no Vídeo')
            print ('Carregando último Snapshot')
            break    
        
        if (cv2.waitKey(1) & 0xFF == ord('f')):
            cv2.imwrite('snap.png', snap)
            break
        
        if (cv2.waitKey(1) & 0xFF == ord('m')):
            cv2.imwrite('snap.png', snap)
            M = True
            print M
            break
        
        cv2.imshow('Câmera', snap)
        
    cv2.destroyAllWindows()
    return snap, M

    

#==== [ADQUIRE PARÂMETROS DO CAMPO] ===========================================
def parametro(snap, flag_parametros):

    n=7     #Numero de parâmetros
    
    global snape
    #snape = copy.deepcopy(snap)        #Calibração de Cores de amostra

    parametros = np.zeros([n,2],dtype=int)  #Inicializa vetor nulo pra receber parâmetros do campo
    
    if flag_parametros == True:
        
        cv2.namedWindow('Calibracao de Parametros do Campo')
        cv2.moveWindow('Calibracao de Parametros do Campo', 200, 200)
        cv2.setMouseCallback('Calibracao de Parametros do Campo',mouse_click)
 
       
        for i in xrange(0,n):    
            if i==0:
                print('Selecione a borda SUPERIOR ESQUERDA do campo e aperte s')
            if i==1:
                print('Selecione a borda SUPERIOR DIREITA do campo  e aperte s')
            if i==2:
                print('Selecione a borda INFERIOR ESQUERDA do campo  e aperte s')
            if i==3:
                print('Selecione a borda INFERIOR DIREITA do campo  e aperte s')
             
            if i==4:
                snape = planificar(snape,parametros)
            while(1):
                
                cv2.imshow('Calibracao de Parametros do Campo', snape)
                                
                k = cv2.waitKey(20) & 0xFF
                #print i
                if (k == ord('f') or i==n):     #Aperte 'f' para fechar a janela                
                    if os.path.isfile("parametros.npy") == True:
                        parametros = np.load('parametros.npy')
                        cv2.destroyAllWindows()
                        flag_parametros = False
                        break
                    else:
                        print('É necessário selecionar os parâmetros do campo!')
                
                elif k == ord('s') and i<n:
                    parametros[i] = (mouseX,mouseY)
                    print(parametros[i])
                    break
        
                if i == n:
                    break
            if flag_parametros == False:
                break
        cv2.destroyAllWindows()
    
    else:
        parametros = np.load('parametros.npy')
    
    origem = parametros[0]
    np.save('parametros',parametros)
    return parametros, origem


#==== [CORTA A IMAGEM] ========================================================
def crop(img,parametros,mostrar):
    imagem=img[parametros[0,1]:parametros[2,1], parametros[0,0]:parametros[1,0]]
    if mostrar == True:
        cv2.imshow('image',imagem)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    return imagem


#==== [ CALIBRAÇÃO ] =========================================================
def color(snap_amostra,n_amostras,ch,parametros,video,area_max):
    
    a = n_amostras    
    pixel_pos = np.zeros([a,2], dtype=np.int16)                 #Matriz com a posicao das amostras de pixel
    pixel_color = np.zeros([a,6], dtype=np.int16)               #Matriz com range de cor default dos pixels
    lower = np.zeros([a,ch], dtype=np.int16)                    #treshold inferior das amostras
    upper = np.zeros([a,ch], dtype=np.int16)                    #treshold superior das amostras
    calib = np.zeros([a,9], dtype=np.int16)                     #Matriz que guarda todos os dados de calibração individuais pra cada amostra
    mask1 = np.zeros([snap_amostra.shape[0],snap_amostra.shape[1]])
    img = np.zeros([snap_amostra.shape[0],snap_amostra.shape[1], ch])  
    frame_counter = 0
    ed = 1**6
    
    cap = cv2.VideoCapture(video)
    
    visualizacao(snap_amostra)
    snap_hsv_amostra = cv2.cvtColor(snap_amostra,cv2.COLOR_BGR2HSV)    

    # Carrega calibração anterior como um preset para a calibração atual
    if os.path.isfile('calib.npy'):            
        cali = np.load('calib.npy')
        if len(cali) <= len(calib):
            z = len(cali)
        else:
            z = len(calib)
            
        for j in xrange (0,z):
            calib[j] = cali[j]
            pixel_color[j] = cali[j,0:6]
    
    
    # ------------------[ LOOP PRINCIPAL ]-------------------------------------
    for i in xrange (0,n_amostras):
        
        cv2.namedWindow('Selecione o ponto de amostragem de cor')
        cv2.moveWindow('Selecione o ponto de amostragem de cor', 200, 200)
        cv2.setMouseCallback('Selecione o ponto de amostragem de cor',mouse_click)
        
    # Seleciona amostra de cor -----------------------------------------------
        while(1):
            
            cv2.imshow('Selecione o ponto de amostragem de cor',snape)
            
            k = cv2.waitKey(20) & 0xFF
            if (k == ord('f')):             #Apertando f destroi a janela
                cv2.destroyAllWindows()    
                break                
    
            elif k == ord('s'):             
                pixel_pos[i] = (mouseX,mouseY)
                (Hi,Si,Vi) = snap_hsv_amostra[pixel_pos[i,1],pixel_pos[i,0]]        # Armazena valores inciais de H, S e V das trackbars
                pixel_color[i] = [Hi-30, Hi+30, Si-50, Si+50, Vi-60, Vi+60]         # Define os ranges das barras
                print('Posição')
                print pixel_pos[i]
                print ('Cor')
                print pixel_color[i]
                
                cv2.destroyAllWindows()
                break
                
        cv2.destroyAllWindows()
        
    # Ajuste fino da amostra de cor -------------------------------------------
        
        cv2.namedWindow('Ajuste fino')
        cv2.moveWindow('Ajuste fino', 100, 100)
        cv2.namedWindow('Máscara')
        cv2.moveWindow('Máscara', 500, 100)
            
        
        #calib[i,0:6] = [pixel_color[i,0]-20, pixel_color[i,0]+20, pixel_color[i,1]-20, pixel_color[i,1]+20, pixel_color[i,2]-20, pixel_color[i,2]+20]
        cv2.createTrackbar('Hmin', 'Ajuste fino', pixel_color[i,0],179, nothing)        
        cv2.createTrackbar('Hmax', 'Ajuste fino', pixel_color[i,1],179, nothing)        
        cv2.createTrackbar('Smin', 'Ajuste fino', pixel_color[i,2],255, nothing)
        cv2.createTrackbar('Smax', 'Ajuste fino', pixel_color[i,3],255, nothing)
        cv2.createTrackbar('Vmin', 'Ajuste fino', pixel_color[i,4],255, nothing)   
        cv2.createTrackbar('Vmax', 'Ajuste fino', pixel_color[i,5],255, nothing)   
        cv2.createTrackbar('Tipo do Filtro', 'Ajuste fino', calib[i,6],3,nothing)
        cv2.createTrackbar('Força do Filtro', 'Ajuste fino', calib[i,7] ,50, nothing)
        cv2.createTrackbar('Estático', 'Ajuste fino', calib[i,8] ,1, nothing)
        cv2.createTrackbar('0 : OFF \n1 : ON', 'Ajuste fino',1,1,nothing)
        
        
        view = copy.deepcopy(snap_amostra)
        
        while(1):
            
            cv2.imshow('Máscara',view)
            
            
            if calib[i,8] == 1:         #Calibra mostrando video em tempo real
                ret, view = cap.read()
                view = planificar(view,parametros)
                snap_hsv = cv2.cvtColor(view,cv2.COLOR_BGR2HSV)
                
                frame_counter += 1
    
                # Permite rodar arquivos de vídeo em loop caso não use a câmera.
                if imutils.is_cv2() and frame_counter == cap.get(cv2.cv.CV_CAP_PROP_FRAME_COUNT)-10:   #Conta frames para recomeçar o vídeo quando acabar
                    frame_counter = 0
                    cap.set(cv2.cv.CV_CAP_PROP_POS_FRAMES, 0)  
                
                elif imutils.is_cv3() and frame_counter == cap.get(cv2.CAP_PROP_FRAME_COUNT)-10:
                    frame_counter = 0
                    cap.set(cv2.CAP_PROP_POS_FRAMES, 1) 
                
            else:
                    snap_hsv = snap_hsv_amostra[:]
                    view = snap_amostra[:]
            st = time.time()
            

            # Atualiza HSV em relação à posição dos trackbars em tempo real
            Hmin = cv2.getTrackbarPos('Hmin', 'Ajuste fino')
            Hmax = cv2.getTrackbarPos('Hmax', 'Ajuste fino')
            Smin = cv2.getTrackbarPos('Smin', 'Ajuste fino')
            Smax = cv2.getTrackbarPos('Smax', 'Ajuste fino')
            Vmin = cv2.getTrackbarPos('Vmin', 'Ajuste fino')
            Vmax = cv2.getTrackbarPos('Vmax', 'Ajuste fino')
            fi = cv2.getTrackbarPos('Tipo do Filtro', 'Ajuste fino')
            ke = cv2.getTrackbarPos('Força do Filtro', 'Ajuste fino')
            est = cv2.getTrackbarPos('Estático', 'Ajuste fino')
            show = cv2.getTrackbarPos('0 : OFF \n1 : ON','Ajuste fino')
            
            
            calib[i,0:6] = [Hmin, Hmax, Smin, Smax, Vmin, Vmax]

            img = snap_hsv[:,:,0:ch]
            inf = np.array([Hmin, Smin, Vmin]) 
            lower[i] = inf[0:ch]
            sup = np.array([Hmax, Smax, Vmax]) 
            upper[i] = sup[0:ch]
                
            if show == 1:

                ####### CORREÇÃO DA DESOCONTINUIDADE DO CANAL H  ##############
                
                if lower[i,0] >= upper[i,0]:
                    lower_aux = np.copy(lower[i])
                    upper_aux = np.copy(upper[i])
                    lower_aux[0] = 0
                    upper_aux[0] = 179
                    mask_aux1 = cv2.inRange(img, lower[i], upper_aux)
                    mask_aux2 = cv2.inRange(img, lower_aux, upper[i])
                    mask1 = cv2.bitwise_or(mask_aux1,mask_aux2)

                ###############################################################
                else:
                    # Segmentação para intervalo contínuo de H
                    mask1 = cv2.inRange(img, lower[i], upper[i])  
                    
                # Filtragem de ruído e visualização
                mask1 = filtro.morph(mask1,fi,ke,iterations=1)
                view = cv2.bitwise_and(view,view,mask=mask1)
                
                
            calib[i] = [Hmin, Hmax, Smin, Smax, Vmin, Vmax, fi, ke, est]        #Salva parâmetros de calibração
            
            ed = time.time()
            
            k = cv2.waitKey(int(30-1000*(ed-st))) & 0xFF
            if (k == ord('f')):     #Aperte 'f' para fechar a janela
                calib_old = np.load('calib.npy')
                calib[i] = calib_old[i]
                cv2.destroyAllWindows()
                break
            
            elif k == ord('s'):
                np.save('calib',calib)
                cv2.destroyAllWindows()
                break
            
            
    
    # Processamento -----------------------------------------------------------
    
            cnts = cv2.findContours(mask1.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cnts = cnts[0] if imutils.is_cv2() else cnts[1]
            # Varre os contornos encontrados
            for c in cnts:
            	# Achar centroide de cada contorno
                M = cv2.moments(c)
                
                if  M["m00"] <= area_max and M["m00"]!=0:                
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])
             
            	# draw the contour and center of the shape on the image
                    #cv2.drawContours(view, [c], -1, (0, 255, 0), 2)
                    cv2.circle(view, (cX, cY), 3, (255, 255, 255), -1)
                    #cv2.putText(view, "center", (cX - 20, cY - 20),
                    #cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
    
    print ('-----------------------------')   
    print('Amostras de cor:')
    print (pixel_color)

    
    np.save('lower', lower)
    np.save('upper', upper)    
    np.save('calib', calib)
    
    return calib, upper, lower, mask1

	
 
def planificar(frame,parametros):
    
    # Aplica transformação linear que corrige paralaxe do campo
    dy = parametros[3,1]-parametros[0,1]    # Organiza dados em y
    dx = parametros[1,0]-parametros[0,0]    # Organiza dados em x
#    dx = 640    # Tamanho desejado para a janela final
#    dy = 480    # Tamanho desejado para a janela final
    dx = 480    # Tamanho desejado para a janela final
    dy = 390    # Tamanho desejado para a janela final

    pts1 = np.float32([parametros[0],parametros[1],parametros[2],parametros[3]])
    pts2 = np.float32([[0,0], [dx,0], [0,dy],[dx,dy]])

#    s = time.time()
    M = cv2.getPerspectiveTransform(pts1,pts2)
    frame_t = cv2.warpPerspective(frame,M,(dx,dy))
#    e = time.time()
#    print 'Tempo de transformação'
#    print (e-s)
    
    return frame_t
