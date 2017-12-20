# -*- coding: utf-8 -*-
"""
Created on Thu Jan 19 19:42:28 2017

@author: geo
"""

import cv2
import numpy as np
import calibracao
import tracking
import time
import math
import imutils
import desempenho


import serial
from estrategias import estrategia
from controle import serialEscreverPorta

pi = math.pi

# =========================[ VARIÁVEIS DA VISÃO]==============================
aquisicao = -1        # Entrada de Vídeo: 0=Camera interna, 1=Camera USB, -1=Template
a = 6                # Numero de amostras 
d_cent = 5.0         # Distância máxima entre o centroide dos marcadores (cm)
area_max = 350.0     # Detecção de áreas acima desse valor são ignoradas (área em pixel^2)
adv = True           # Ativa ou desativa detecção de adversários
#adv = False
ch = 3               # Numero de canais de vídeo a serem considerados (H,S,V)
param = True         # Habilita a configuração dos parametros do campo.
ang_corr = pi/4      # Habilitar no caso de Marcadores diagonais (Videos atuais)
#ang_corr = 0        # Habilitar no caso de Marcadores alinhados com a frente do robô (Video de Uberlandia)

M = True             # Variável do Marcos, quando ativada pula a calibração, carregando padrão anterior
M = False

gravar_video = True    # Permite gravar vídeo do jogo enquanto roda a visão (só funciona com câmera).
#gravar_video = False

robot = np.zeros([3,5])     # Alocação de espaço para matriz robot: [ ] 
tempo = np.empty([0,1])     # Alocação de espaço para matriz tempo, usada para análise de desempenho
counter = 0                 # Inicializa Contador
vec_len = 55
if adv == False:
    n_robos = a-2 
else:
    n_robos = a-3

# =========================[ VARIÁVEIS DO CONTROLE]============================
Gleyson = True
Gleyson = False
bola = np.zeros([1,2])
RD = np.zeros([3])
angulo_d = np.zeros(3)


debug = True
debug = False
Majin = False
duasFaces = False
if(Gleyson):
    porta = "/dev/ttyUSB0"            #Xbee
    velocidade = 57600
    
    ser = serial.Serial(porta, velocidade)
else:
    ser = 0

# =========================[ PRÉ PROCESSAMENTO]================================

# MODO -1: SEM CÂMERA (processa arquivo de vídeo)
if aquisicao == -1:
    gravacao = "teste_visao.mp4"
    gravacao = 'visao3.mp4'
#    gravacao = 'pequi.avi'
else:
    gravacao = aquisicao

snap,M = calibracao.snapshot(gravacao,M)

# CASO SEJA NECESSÁRIO CALIBRAR
if M == False:
    calibracao.visualizacao(snap)                           # Permite visualizar imagem da câmera
    parametros, origem = calibracao.parametro(snap,param)    # Rotina de calibração dos parametros espaciais do campo (cantos e cruzetas)  
    print('Parâmetros:')
    print parametros

    calibracao.visualizacao(snap)                           # Permite visualizar imagem da câmera
    #snap = calibracao.crop(snap,parametros,False)          # Antiga função crop que corta a imagem, muito rápida mas que não corrige campo torto nem redimensiona
    snap = calibracao.planificar(snap,parametros)           # Função 'planificar' bem mais lenta, mas corrige campo torto e redimensiona a imagem

    calib, upper, lower, mask1 = calibracao.color(snap,a,ch,parametros,gravacao,area_max)  # Rotina de calibração das cores

# CASO NAO SEJA NECESSÁRIO CALIBRAR, PULA ESSA ETAPA E CARREGA CALIBRAÇÃO ANTERIOR
else:
    parametros = np.load('parametros.npy')
    lower = np.load('lower.npy')
    upper = np.load ('upper.npy')
    calib = np.load('calib.npy')


# ORGANIZA VARIÁVEIS DOS PARÂMETROS ESPACIAIS OBTIDOS
campo_maxY = parametros[2, 1]
campo_maxX = parametros[1, 0]
origem = (parametros[0, 0],parametros[0, 1])
cruzetas_x = [parametros[4,0],parametros[5,0]];
cruzetas_y = [parametros[4,1],parametros[6,1]];

distCruzX = abs(cruzetas_x[0] - cruzetas_x[1])
distCruzY = abs(cruzetas_y[0] - cruzetas_y[1])

# CALCULA A ESCALA CM/PIXEL LEVANDO EM CONTA A DISTÂNCIA DAS CRUZETAS
if (distCruzX > distCruzY):

#    constX = 69/distCruzX      # Campo velho
#    constY = 32.0/distCruzY    # Campo velho         
    
    constX = 70.0/distCruzX     # Campo novo
    constY = 37.5/distCruzY     # Campo novo
else:
#    constX = 32.0/distCruzY    # Campo velho     
#    constY = 69/distCruzX      # Campo velho
    
    constX = 37.5/distCruzY     # Campo novo
    constY = 70.0/distCruzX     # Campo novo

d_pixel = (d_cent/constX)**2 +  (d_cent/constY)**2      # Distância quadrática entre os marcadores em pixel

# =====[ TRACKING ]=====
cap = cv2.VideoCapture(gravacao)

# INICIALIZA GRAVADOR DE VÍDEO
if gravar_video == True:
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    out = cv2.VideoWriter('out.avi',fourcc, 30.0, (640,480))

# COMPATIBILIZA OPENCV 2.X E OPENCV 3.X PARA REPETIÇÃO DO VÍDEO GRAVADO
if imutils.is_cv2 and aquisicao == -1:
    frame_max = cap.get(cv2.CAP_PROP_FRAME_COUNT)
    opencv_version = 2
if imutils.is_cv3 and aquisicao == -1:
    frame_max = cap.get(cv2.CAP_PROP_FRAME_COUNT)
    opencv_version = 3
frame_counter = 0  
cv2.namedWindow("Tracking")
cv2.moveWindow("Tracking",200,200)  
media = 0
bola_sumiu = 0
AD_sumiu = 0
robot_sumiu = np.zeros([3])
percent_ball = 0
percent_AD = 0
percent_robot = np.zeros([3])
s = 3

# =============================[ LOOP PRINCIPAL ]==============================
while (cap.isOpened()):   
    # ==============================[ VISAO ]==================================
    
    # CAPTURA FRAMES E RECORTA A IMAGEM
    ret,frame = cap.read()      # Variável ret verifica se a câmera funciona, frame guarda a imagem
    frame_counter += 1
            

    startv = time.time()        # Começa a contar o tempo para o desempenho da visão
    if ret == True:
        
        # GRAVA VÍDEO ENQUANTO RODA A VISÃO:
        if aquisicao !=-1 and gravar_video == True:
            out.write(frame)
            

#        frame = calibracao.crop(frame,parametros,False)    # Antiga função crop, que recorta o campo.
        frame = calibracao.planificar(frame,parametros)     # Recorta, desentorta e redimensiona o campo
        
        
        # OBTENÇÃO DOS CENTROIDES
        bola, view, centroid_cores = tracking.tracking(lower,upper,frame, calib,area_max,ch)        
        bola_sumiu, percent_ball = desempenho.detect_ball(frame_counter,frame_max,bola,bola_sumiu,percent_ball,s)
        
        # CASO QUEIRA DETECTAR OS ADVERSÁRIOS
        if adv == True:
            aux = centroid_cores[a-1,:,:]
            AD = aux[~np.all(aux == 0, axis=1)]
            centroid_cores = centroid_cores[0:a-1,:,:]
            AD_sumiu, percent_AD = desempenho.detect_AD(frame_counter,frame_max,AD,AD_sumiu,percent_AD,s)

        
        # RASTREIO DOS robôs
        robot_ant = robot                                                   # Salva posição anterior dos robôs para fazer um filtro e diminuir tremida dos centroides 
        robot = tracking.pao_da_visao(centroid_cores,d_pixel,ang_corr)      # Adquirie posição atual dos robôs
        #robot = (robot*0.8 + robot_ant*0.2)                                # Filtro da média ponderada dos centroides atuais com os centroides do frame anterior    
        robot_sumiu, percent_robot = desempenho.detect_robot(frame_counter,frame_max,robot,robot_sumiu,percent_robot,s)
        
        # PLOTAGEM DOS CENTROIDES E VETORES SOBRE A IMAGEM
        cv2.circle(view, (int(bola[0]),int(bola[1])), 5, (255, 0, 255), -1)  # Plota centroide da bola
        
        # Plota, um a um, os centroides e angulos dos robôs
        for r in xrange(3):
     
            x_view = int(robot[r,3] + vec_len*math.cos(robot[r,2]))         # Cálculo do ponto extremidade da reta visual em x do ângulo do robô usando coordenadas polares
            y_view = int(robot[r,4] + vec_len*math.sin(robot[r,2]))         # Cálculo do ponto extremidade da reta visual em y do ângulo do robô usando coordenadas polares
            
            mark = frame[int(robot[r,4]),int(robot[r,3])]
            
            cv2.circle(view, (int(robot[r,0]),int(robot[r,1])), 5, (0, 255, 255), -1)          ## Centroide de cada robô aliado (Amarelo)
            cv2.line(view, (int(robot[r,0]),int(robot[r,1])), (x_view,y_view), (int(mark[0]),int(mark[1]),int(mark[2])), 3)   ## Orientação de cada robô (Vermelho)
        
        if adv == True:
            for r in xrange(len(AD)):
                cv2.circle(view, (int(AD[r,0]),int(AD[r,1])), 5, (255, 0, 0), -1)  ## Centroide de cada robô adversário (Azul)
        
        
        # PERMITE RODAR ARQUIVOS DE VÍDEO EM LOOP CASO NÃO USE A CÂMERA.
        # CASO USE OPENCV 2.4
        if opencv_version == 2 and frame_counter == frame_max-s:   #Conta frames para recomeçar o vídeo quando acabar
            frame_counter = 0
            tempo = np.empty([0,1])
            cap.set(cv2.cv.CV_CAP_PROP_POS_FRAMES, 0)  
        
        # CASO USE OPENCV 3.0
        elif opencv_version == 3 and frame_counter == frame_max-s:
            
            # MOSTRA DESEMPENHO DA VISÃO AO FINAL DE CADA LOOP DE VÍDEO
            
            desv_pad, media = desempenho.loop_time(tempo)
            
            print '\n=========== DESEMPENHO VISÃO ==========='
            print 'Tempo Médio:',media, 'ms'
            print 'Desvio Padrão:',desv_pad, 'ms'    
            print 'Frames Sem Bola:',bola_sumiu
            for h in xrange(3):
                print 'Robo',h,':', round(percent_robot[h],2),'%'
            print 'Detecção dos Adversários:',round(percent_AD,2),'%'
            print 'Detecção de Bola:',round(percent_ball,2),'%'
            print '========================================='
            
            media = 0
            frame_counter = 0
            bola_sumiu = 0
            AD_sumiu = 0
            robot_sumiu = np.zeros([3])
            percent_ball = 0
            percent_AD = 0
            percent_robot = np.zeros([3])            
            tempo = np.empty([0,1])
            cap.set(cv2.CAP_PROP_POS_FRAMES, 0) 
        #print robot
        
        # ============================[ CONTROLE ]=============================
        startc = time.time()
        
        angulo_d = estrategia(robot, bola, ser, view, constX, constY, Gleyson, debug, Majin,angulo_d)
        
        
       ##PLOTAGEM DO ANGULO DESEJADO, DEPENDE DO MÓDULO DE CONTROLE
#        for r in xrange(3):
#            x_ang_d = int(robot[r,3] + vec_len*math.cos(angulo_d))
#            y_ang_d = int(robot[r,4] + vec_len*math.sin(angulo_d))
#   
#            cv2.line(view, (int(robot[r,0]),int(robot[r,1])), (x_ang_d,y_ang_d), (0,255,255), 3)  ## Orientação de cada robô (vermelho)
#       
        endc = time.time()

        # ===================== [ CONDIÇÃO DE PARADA] =========================
        flag = 0    # Flag de finalização do software, inicia abaixada.
        
        # CASO APERTE 'S' PAUSA O JOGO, SE APERTAR 'F' EM SEGUIDA, FINALIZA PROGRAMA
        if cv2.waitKey(1) & 0xFF == ord('s'):
            
            desv_pad, media = desempenho.loop_time(tempo)

            print '\n=========== DESEMPENHO VISÃO ==========='
            print 'Tempo Médio:',media, 'ms'
            print 'Desvio Padrão:',desv_pad, 'ms'    
            print '========================================='
            
            
            if Gleyson:
                for i in xrange(10):
                    serialEscreverPorta(ser, '&0,0000,0,0000#', Gleyson)
                    serialEscreverPorta(ser, '@0,0000,0,0000#', Gleyson)
                    serialEscreverPorta(ser, '$0,0000,0,0000#', Gleyson)
                
            # ESPERA APERTAR 'R' PARA RETORNAR OU 'F' PARA FECHAR
            while flag == 0:            
                if cv2.waitKey(20) & 0xFF == ord('r'):
                    break
                    
                if cv2.waitKey(20) & 0xFF == ord('f'):
                    flag = 1    # Ativa flag de finalização do software
                    break
        
        cv2.imshow("Tracking",view)
        
        endv = time.time()      #Encerra contagem de tempo do loop da visão
        
        ## ANÁLISE DE DESEMPENHO =============================================
        total_time = 1000*(endv-startv)
        tempo = np.append(tempo,[[total_time]], axis = 0)
        
        
        if flag == 1:
            print 'ALGORITMO ENCERRADO'
            break
    else:
        break

cap.release()
cv2.destroyAllWindows()
if gravar_video == True:
    out.release()

