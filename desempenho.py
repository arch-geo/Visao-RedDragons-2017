#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Fri Oct 27 20:50:41 2017

@author: geo
"""

import numpy as np

def loop_time(tempo):
    soma_tempo = np.sum(tempo)
    n_tempo = float(len(tempo))
    media = soma_tempo/n_tempo
    var = np.sum((media - tempo)**2)/(n_tempo)
    desv_pad = var**(1.0/2)
    return desv_pad,media


def detect_ball(frame_counter,frame_max,bola,bola_sumiu,percent_ball,s):
    if np.sum(bola) == 0:
        bola_sumiu += 1
        
    if frame_counter == frame_max-s-1:
        percent_ball = 100*(frame_max-s-1 - bola_sumiu) / (frame_max-s-1)
    
    return bola_sumiu,percent_ball


def detect_robot(frame_counter,frame_max,robot,robot_sumiu,percent_robot,s):
    for i in xrange(3):
        if np.sum(robot[i,:]) == 0:
            robot_sumiu[i] += 1
            
        if frame_counter == frame_max-s-1:
            percent_robot[i] = 100*(frame_max-s-1 - robot_sumiu[i]) / (frame_max-s-1)
        
    return robot_sumiu,percent_robot

def detect_AD(frame_counter,frame_max,AD,AD_sumiu,percent_AD,s):
    if len(AD) != 3:
        AD_sumiu += 1
        
    if frame_counter == frame_max-s-1:
        percent_AD = 100*(frame_max-s-1 - AD_sumiu) / (frame_max-s-1)
    
    return AD_sumiu,percent_AD