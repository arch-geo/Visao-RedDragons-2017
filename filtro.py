# -*- coding: utf-8 -*-
"""
Created on Mon Oct  3 21:00:10 2016

@author: GEO
"""

import cv2
import numpy as np

# APLICAÇÃO DE FILTROS MORFOLÓGICOS
def morph(img,filtro,ke,iterations):
    
#    if ke == 0:
#        ke =1
    ke = 2*ke    # Coeficiente do kernel precisa ser par
    
    kernel = np.ones((ke,ke),np.float32)/(ke*ke)    #Criação do kernel
    
    
    # Sem filtro
    if filtro == 0:
        res = img
    
    # Filtro de Erosão
    if filtro == 1:
        res = cv2.erode(img,kernel)
    
    # Filtro de Abertura (Erosão seguido de Dilatação)
    if filtro == 2:
        res = cv2.morphologyEx(img, cv2.MORPH_OPEN, kernel,iterations)
        
    # Filtro de Fechamento (Dilatação seguido de Erosão)    
    if filtro == 3:
        res = cv2.morphologyEx(img, cv2.MORPH_CLOSE, kernel,iterations)
        
    return res

    