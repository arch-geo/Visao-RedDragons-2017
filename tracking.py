# -*- coding: utf-8 -*-
"""
Created on Tue May  9 23:56:25 2017

@author: geo
"""
import cv2
import numpy as np
import filtro
from math import pi
import imutils
#import time
    
#==== [ TRACKING ] =========================================================
def tracking(lower,upper,frame, calib,area_max,ch):
    
    ##################### DEFINIÇÃO DAS VARIÁVEIS ##########################
    iterations = 1;                              # Número de iterações do filtro
    n_max = 10                                   # Número máximo de centroides a ser armazenado
    [y,x,_] = frame.shape                        # Pega medidas da imagem para criação de máscaras
    a = len(calib)                               # Armazena o número de amostras
    mask1 = np.zeros([y,x], dtype=np.uint8)      # Cria mascara 1
    bola = np.zeros([1,2])                       # Aloca espaço para o vetor de posição da bola
    centroid_cores = np.zeros([a,n_max,3])       # Aloca espaço para criação do pão da visão
    view = frame.copy()
    
    ###################### CONVERSÃO PARA HSV ################################
    frame_hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
    img = frame_hsv[:,:,0:ch]
    
    
    ##################### VARREDURA DAS CORES CALIBRADAS  ####################
    for i in xrange (0,a):
        mask1 = cv2.inRange(img,lower[i],upper[i])  ## Segmentação das cores
        
        
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
            
        mask1 = filtro.morph(mask1,calib[i,6],calib[i,7],iterations)  ## Filtro de ruído


        ############## CALCULANDO CENTROIDS DE TODOS OS OBJETOS #############
        centroids = np.empty([0,3])
        cnts = cv2.findContours(mask1.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)   # Acha contornos da mascara
        cnts = cnts[0] if imutils.is_cv2() else cnts[1]                                     # Compatibiliza com opencv 2.x e 3.x
        
        # VARRE OS CONTORNOS ENCONTRADOS
        for c in cnts:
        	# ACHAR CENTROIDE DE CADA CONTORNO
            M = cv2.moments(c)
            
            if  M["m00"] < area_max and M["m00"] != 0:                
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
    
                centroids = np.append(centroids,[[cX,cY,M["m00"]]], axis = 0)
                cv2.circle(view, (cX, cY), 2, (255, 255, 255), -1)                
        
        ## INDEXANDO N CORES ENCONTRADAS EM MATRIZ DE N DIMENSÕES [X,Y,AREA]        
        num_centroids = len(centroids)
        
        if num_centroids == 0:
            centroids = np.zeros([1,3])  
                    
        if num_centroids <= n_max:
            centroid_cores[i,0:num_centroids,:] = centroids
        else:
            cv2.putText(view,"Centroids demais!", (25,25), cv2.FONT_HERSHEY_COMPLEX, 1, [0,255,0])
            centroid_cores[i] = centroids[0:n_max,:]
            
    
    ## Define bola    
    bola = centroid_cores[0,0,0:2]
    
    if np.sum(bola) == 0:
        cv2.putText(view,"SEM BOLA!", (25,25), cv2.FONT_HERSHEY_COMPLEX, 1, [0,255,0])
    
    return bola, view, centroid_cores

def pao_da_visao(centroid_cores,dist_max,ang_corr):
    (a,n_max,_) = centroid_cores.shape
    robot = np.zeros([3,5], dtype=np.float16)
    if a > 2 :
        for j in xrange (0,3):                   # Varre as cores de time
            for k in xrange (2,a):               # Varre as cores de jogador
                for l in xrange(0,n_max):        # Varre as cores repetidas de jogador
                    if sum(centroid_cores[k,l,:]) != 0:     # Pula linhas nulas
                        
                        # CALCULANDO AS DISTANCIAS ENTRE OS MARCADORES CANDIDATOS A FORMAR UM ROBÔ
                        dx = centroid_cores[1,j,0]-centroid_cores[k,l,0]
                        dy = centroid_cores[1,j,1]-centroid_cores[k,l,1]
                        dist = (dx**2 + dy**2)
        
                        # SE A DISTÂNCIA FOR COMPATÍVEL, CONSIDERE UM ROBÔ
                        if dist != 0 and dist < dist_max:
                            # Calculando o centroide do robo pela média dos centroides dos marcadores
                            robot[k-2,0:2] = (centroid_cores[1,j,0]+centroid_cores[k,l,0])/2, (centroid_cores[1,j,1]+centroid_cores[k,l,1])/2
                            angulo = np.arctan2(dy,dx) + ang_corr
                            
                            # CORREÇÃO DA DESCONTINUIDADE DO ANGULO PARA O CONTROLE
                            if (angulo >= -pi and angulo <= pi):
                                pass
                            elif (angulo > pi):
                                angulo = angulo - 2*pi
                            else:
                                print("NÂO CUIDADO!!!!!!!!!!!!!!!!!!!!!!!!!")           # Avisa sobre a ocorrência de uma condição não prevista na correção do ângulo
                                
                            # PREENCHIMENTO DA MATRIZ ROBOT    
                            robot[k-2,2] = angulo
                            robot[k-2,3:5] = (centroid_cores[k,l,0],centroid_cores[k,l,1])
                            
                            # robot = [ CentroideX, CentroideY, Angulo, CentroideAzulX, CentroideAzulY]
            
    return robot

    


    
            
              