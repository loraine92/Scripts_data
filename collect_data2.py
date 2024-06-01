# -*- coding: utf-8 -*-
#
#     ||          ____  _ __
#  +------+      / __ )(_) /_______________ _____  ___
#  | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#   ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Copyright (C) 2019 Bitcraze AB
#
#  Crazyflie Nano Quadcopter Client
#
#  This program is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public License
#  as published by the Free Software Foundation; either version 2
#  of the License, or (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software
#  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
#  MA  02110-1301, USA.
#3. Code used to gather data from positioning systems.
#(https://github.com/bitcraze/positioning_dataset)
#(https://github.com/bitcraze/positioning_dataset/blob/master/collect_data.py)
#This code sends data to position the drone and saves the position values from 
#the lighthouse system to a microSD memory (loraine92)
"""
Change the URI variable to your Crazyflie configuration.
"""
import logging
import time
import os
import numpy as np
import argparse

import cflib.crtp
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.utils.power_switch import PowerSwitch
from cflib.positioning.position_hl_commander import PositionHlCommander

URI = 'radio://0/80/2M/E7E7E7E7E7'
usdCanLog = None

def consoleReceived(data):
    print(data, end='')

def paramReceived(name, value):
    global usdCanLog
    if name == "usd.canLog":
        usdCanLog = int(value)
#La variable usdCanLog que corresponde a una de las variables de la tarjeta microSD,
#indica que si es distinto de cero es posible iniciar sesión, 
# 0 indica que puede existir un problema con la configuración de registro. 

if __name__ == '__main__':

    # Only output errors from the logging framework
    logging.basicConfig(level=logging.ERROR)
    #mediante el parámetro de nivel establece que desde el nivel ERROR se desea que se registren los mensajes de registro.
    # Initialize the low-level drivers (don't list the debug drivers)
    cflib.crtp.init_drivers(enable_debug_driver=False)

    s = PowerSwitch(URI)
    s.stm_power_cycle()
    s.close()
    time.sleep(5) 


    #logging framework
    lg = LogConfig(name='Battery', period_in_ms=10)
    lg.add_variable('pm.vbat', 'float')
    lg.add_variable('lighthouse.status', 'uint8_t')
    #La variable ‘lighthouse.status’ indica el estado general de los faros, si el valor es 0 indica 
    # que no se reciben datos de las estaciones de faros, si el valor es 1 indica que se reciben una
    #  o más estaciones base pero faltan datos de calibración o geometría, si es 2 significa que los 
    # datos de la estación base se envían al estimador de estado. La variable ‘pm.vbat’ indica el voltaje de la batería. 

    cf = cflib.crazyflie.Crazyflie()
    cf.console.receivedChar.add_callback(consoleReceived) #llama a la funcion consoleReceived

    with SyncCrazyflie(URI, cf) as scf:

        # check usd Deck
        cf.param.add_update_callback(group='usd', name='canLog', cb=paramReceived)
        cf.param.request_param_update('usd.canLog')

       #Verifica si la tarjeta microSD está conectada.
       #usd.canLog: La variable usdCanLog indica que si es distinto de cero es posible iniciar sesión, 
       # 0 indica que puede existir un problema con la configuración de registro.
     
        time.sleep(2)

        # check battery voltage
        with SyncLogger(scf, lg) as logger:
            for _, data, _ in logger:
                vbat = data['pm.vbat'] # pm.vbat es el voltaje de la bateria
                lhStatus = data['lighthouse.status']
                break

        print("Battery voltage: {:.2f} V".format(vbat))
        print("LightHouse Status: {}".format(lhStatus))

        if vbat < 3.6:   # es el voltaje de la bateria
            exit("Battery too low!")

        if lhStatus != 2 :
            exit("LightHouse not working!")

        if usdCanLog != 1:
            exit("Can't log to USD!")

        # if args.estimation_mode == 'crossingBeam':
            # enable lighthouse crossing beam method
        #cf.param.set_value('lighthouse.method', 0)
        # elif args.estimation_mode == 'kalman':
        cf.param.set_value('lighthouse.method', 1)
        cf.param.set_value('kalman.initialX', 0)
        cf.param.set_value('kalman.initialY', 0)
        cf.param.set_value('kalman.initialZ', 0)
        cf.param.set_value('kalman.resetEstimation', 1)
        # else:
        #    exit("Unknown mode", args.estimation_mode)

        # Realiza la configuración de los parámetros:
        #Lighthouse.method: Configura el método de estimación, si es 0 utiliza el método CrossingBeam, 
        # si es 1 utiliza el método Sweep en el filtro extendido de Kalman, el predeterminado es 1.
        #kalman.initialX: Es el valor inicial de x en metros, después de un reinicio.
        #kalman.initialY: Es el valor inicial de y en metros, después de un reinicio
        #kalman.initialZ: Es el valor inicial de z en metros, después de un reinicio
        #kalman.resetEstimation: Resetea el estimador de Kalman.
            

        # start logging in uSD card
        cf.param.set_value('usd.logging', 1)
        time.sleep(10)

        #Usd.logging controla si el registro en la tarjeta SD está activo. 
        # Se configura en 1 para iniciar el registro y en 0 para detener el registro, esta predeterminado en cero.
        # stop logging in uSD card
#---------------------------------------------------------------------
#prueba para mover el Crazyflie a la posición seleccionada usando 
#positionHLcommander

with PositionHlCommander(scf, controller=PositionHlCommander.CONTROLLER_PID) as pc:
            # Go to the coordinate (0, 0, 1)
            pc.go_to(0.0, 0.0, 1.0)
            # The Crazyflie lands when leaving this "with" section
        # When leaving this "with" section, the connection is automatically closed


#---------------------------------------------------------------------
        cf.param.set_value('usd.logging', 0)
        time.sleep(2)

    s = PowerSwitch(URI)
    s.stm_power_down()
    s.close()


