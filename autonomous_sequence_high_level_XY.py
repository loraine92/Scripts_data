# -*- coding: utf-8 -*-
#
#     ||          ____  _ __
#  +------+      / __ )(_) /_______________ _____  ___
#  | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#   ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Copyright (C) 2018 Bitcraze AB
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
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <https://www.gnu.org/licenses/>.
"""
Simple example that connects to one crazyflie (check the address at the top
and update it to your crazyflie address) and uses the high level commander
to send setpoints and trajectory to fly a figure 8.

This example is intended to work with any positioning system (including LPS).
It aims at documenting how to set the Crazyflie in position control mode
and how to send setpoints using the high level commander.
"""
"""
Make the following changes: This program performs a trajectory in the x, y axes using the lighthouse positioning system,
verifies the status of the modules and the battery level, and records the position and
velocity data on a microSD memory. loraine92
"""
import sys
import time
import logging #todos los datos de 09/10 los añadi para grabar en la microSD
import os #09/10
import numpy as np #09/10
import argparse #09/10


#--------------------------------------------------
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.mem import MemoryElement
from cflib.crazyflie.mem import Poly4D
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger
#from cflib.utils import uri_helper
#-----------09/10--------------------------------------
from cflib.utils.power_switch import PowerSwitch
from cflib.positioning.position_hl_commander import PositionHlCommander

#from PRUEBAS.collecData import URI
#----------------------------------------------------------

# URI to the Crazyflie to connect to
URI = 'radio://0/80/2M/E7E7E7E7E7'
#uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')

#----------------*****----09/10   microSD-----------*****----------------------
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
#Funciones para leer variables de registro de manera asincrona
# https://www.bitcraze.io/documentation/repository/crazyflie-lib-python/master/user-guides/sbs_connect_log_param/
def log_stab_callback(timestamp, data, logconf):
    print('[%d][%s]: %s' % (timestamp, logconf.name, data))

def simple_log_async(scf, logconf):
    cf = scf.cf
    cf.log.add_config(logconf)
    logconf.data_received_cb.add_callback(log_stab_callback)
    logconf.start()
    time.sleep(5)
    logconf.stop()
#------------------*****------------------*****--------------------------------------------------------------
# The trajectory to fly
# Duration,x^0,x^1,x^2,x^3,x^4,x^5,x^6,x^7,y^0,y^1,y^2,y^3,y^4,y^5,y^6,y^7,z^0,z^1,z^2,z^3,z^4,z^5,z^6,z^7,yaw^0,yaw^1,yaw^2,yaw^3,yaw^4,yaw^5,yaw^6,yaw^7
#Polinomios para formar figura de mariposa en los ejes x,y:
figure8 = [
  [1, 0.999999674, 3.64481231e-05, -0.000857618496, 0.00796107669, -0.0353570572, 0.0797246678, -0.0877133347, 0.0372697267, 0, 0, 0, 0, 0, 0, 0, 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, ],  # noqa
  [1, 1.0010635829171, 0.0140076301010987, 0.0750941865740025, 0.213953266389993, 0.296563294579443, -0.178451450594032, -0.28654965200665, 0.163419462039008, 0, 0.00040449667, -0.000138287695, 0.0609918575, -0.148982, 0.812903587, -0.760986364, 0.202139039, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, ],  # noqa
  [1, 1.29910031999996, 0.524690049999851, -0.154616830000261, -0.395619930000251, -0.65884052999197, 1.47190314998314, -1.02126636998682, 0.268120569996387, 0.166332328475, 0.50074851778, 0.248105513804999, -0.550761187499999, -0.125523480275008, 0.331035433100027, -0.189852820405018, 0.0529356350200044, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, ],  # noqa
  [1, 1.33347043000004, -0.498003999999815, -0.263951879999606, 0.646942000000408, -0.143180400010543, 0.0873051900217341, -0.29930771001676, 0.140467920004556, 0.433019940000005, -0.270808249999973, -0.583108569999954, 0.313190040000029, -0.0345047900012938, 0.816241070002717, -0.944817980002128, 0.29577871000058, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 , 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, ],  # noqa
  [1, 1.00374155000001, -0.0338482299999727, 0.151054289999976, -0.122504700000178, -0.537744210000056, 1.30115628000079, -1.06667956000081, 0.291249660000262, 0.024990169999986, -0.152725990000069, 0.35092671999986, -0.206523170000155, -0.489576809996116, 1.030926419992, -0.927552399993822, 0.303179259998317, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, ],  # noqa
  [1, 0.986425080000021, -0.105778929999855, -0.315312809999647, -0.401771839999537, 0.348304029990871, 0.329665580018309, -0.32289841001397, 0.0703716300037778, -0.066355799999999, -0.317176779999991, -0.443360989999974, 0.204659890000032, -0.019124600000616, 0.665313510001234, -0.689386560000933, 0.203978950000252, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, ],  # noqa
  [1, 0.58900432999997, -0.344965100000107, 0.500179729999839, 0.293138929999866, -0.12076302999467, -0.125437630011353, -0.0435979299911038, 0.0515297299975523, -0.461452379999995, -0.048316649999971, 0.651765730000063, 0.132828640000072, -0.467707510001633, 0.0819307000033476, -0.00497785000256545, 0.0279530500006977, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, ],  # noqa
  [1, 0.799089029999994, 0.523691409999968, -0.171202580000074, -0.5127075400001, 0.0646572800019669, 0.940692069996022, -0.91562313999694, 0.27229644999917, -0.0879762699999841, 0.358328440000068, -0.424340109999874, -0.0398946499998775, 0.248923599996347, 0.193780210007626, -0.38747925000592, 0.138558560001618, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 , 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, ],  # noqa
  [1, 1.00089298000001, 0.0175894100000473, 0.0694175300001092, 0.370755230000135, -0.346553500002884, 0.81852846000578, -0.970430230004427, 0.338902510001198, 9.94699999966e-05, -0.0004058599999885, -0.0151393499999865, -0.0064335499999995, 0.087363808029764, -0.800315192501524, 0.779589568264644, -0.210882855282763, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, ],  # noqa
  [1, 1.29910238999997, 0.524854649999858, -0.151853920000239, -0.377190920000153, -0.763550119993035, 1.64531628998524, -1.14249655998846, 0.299288619996833, -0.166123961489857, -0.500748517780001, -0.248105513805004, 0.55076118749999, 0.118230635795074, -0.313532606348133, 0.1752671314451, -0.0487682953171672, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, ],  # noqa
  [1, 1.33347043000001, -0.498003999999948, -0.263951879999915, 0.646942000000074, -0.143180400002649, 0.0873051900056332, -0.299307710004402, 0.140467920001202, -0.433019939999998, 0.270808250000022, 0.583108570000068, -0.313190039999901, 0.0345047899984143, -0.816241069996905, 0.944817979997664, -0.295778709999374, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, ],  # noqa
  [1, 1.00374155000001, -0.0338482299999852, 0.151054289999951, -0.12250470000015, -0.537744209999374, 1.3011562799992, -1.06667955999955, 0.291249659999899, -0.0249901700000097, 0.152725989999953, -0.350926720000093, 0.206523169999896, 0.500670410002622, -1.06637568000541, 0.956165220004176, -0.310561140001138, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 , 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, ],  # noqa
  [1, 0.986425080000001, -0.105778930000037, -0.315312810000115, -0.401771840000208, 0.348304030002705, 0.329665579994835, -0.322898409996164, 0.0703716299989755, 0.0632310799999963, 0.304308639999983, 0.429602809999968, -0.200887490000026, 0.508344800000883, -1.75938549000185, 1.56265310000144, -0.446415070000394, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 , 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, ],  # noqa
  [1, 0.589004329999993, -0.344965100000052, 0.500179729999866, 0.293138929999824, -0.120763029996645, -0.12543763000672, -0.0435979299948839, 0.0515297299986146, 0.46145238, 0.0483166500000052, -0.651765729999987, -0.132828639999989, 0.467707509999682, -0.0819306999993703, 0.00497784999952477, -0.027953049999871, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 , 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, ],  # noqa
  [1, 0.799089029999997, 0.52369140999997, -0.171202580000084, -0.512707540000122, 0.0901738300020227, 0.845664779996003, -0.795284589996979, 0.220407949999185, 0.0879762699999947, -0.358328440000008, 0.424340110000015, 0.0398946500000488, -0.268461150000007, -0.159438140000197, 0.377667660000227, -0.143657250000074, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 , 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, ],  # noqa
  [1, 0.999832289999993, 0.00333095999996402, -0.0123363200000846, 0.113222029999875, -0.394827829997702, 0.582975439995348, -0.391541279996445, 0.0993452399990333, -6.2900000005e-06, -0.0005943600000006, -0.0129115899999996, -0.102981899999999, 0.553150850000022, -0.903397760000056, 0.627438710000047, -0.160697660000014, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 , 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, ],  # noqa
   
]


class Uploader:
    def __init__(self):
        self._is_done = False
        self._sucess = True

    def upload(self, trajectory_mem):
        print('Uploading data')
        trajectory_mem.write_data(self._upload_done,
                                  write_failed_cb=self._upload_failed)

        while not self._is_done:
            time.sleep(0.2)

        return self._sucess

    def _upload_done(self, mem, addr):
        print('Data uploaded')
        self._is_done = True
        self._sucess = True

    def _upload_failed(self, mem, addr):
        print('Data upload failed')
        self._is_done = True
        self._sucess = False


def wait_for_position_estimator(scf):
    print('Waiting for estimator to find position...')

    log_config = LogConfig(name='Kalman Variance', period_in_ms=500)
    log_config.add_variable('kalman.varPX', 'float')
    log_config.add_variable('kalman.varPY', 'float')
    log_config.add_variable('kalman.varPZ', 'float')

    var_y_history = [1000] * 10
    var_x_history = [1000] * 10
    var_z_history = [1000] * 10

    threshold = 0.001

    with SyncLogger(scf, log_config) as logger:
        for log_entry in logger:
            data = log_entry[1]

            var_x_history.append(data['kalman.varPX'])
            var_x_history.pop(0)
            var_y_history.append(data['kalman.varPY'])
            var_y_history.pop(0)
            var_z_history.append(data['kalman.varPZ'])
            var_z_history.pop(0)

            min_x = min(var_x_history)
            max_x = max(var_x_history)
            min_y = min(var_y_history)
            max_y = max(var_y_history)
            min_z = min(var_z_history)
            max_z = max(var_z_history)

            # print("{} {} {}".
            #       format(max_x - min_x, max_y - min_y, max_z - min_z))

            if (max_x - min_x) < threshold and (
                    max_y - min_y) < threshold and (
                    max_z - min_z) < threshold:
                break


def reset_estimator(cf):
    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    cf.param.set_value('kalman.resetEstimation', '0')

    wait_for_position_estimator(cf)


def activate_high_level_commander(cf):
    cf.param.set_value('commander.enHighLevel', '1')


def activate_mellinger_controller(cf):
    cf.param.set_value('stabilizer.controller', '2')


def upload_trajectory(cf, trajectory_id, trajectory):
    trajectory_mem = cf.mem.get_mems(MemoryElement.TYPE_TRAJ)[0]

    total_duration = 0
    for row in trajectory:
        duration = row[0]
        x = Poly4D.Poly(row[1:9])
        y = Poly4D.Poly(row[9:17])
        z = Poly4D.Poly(row[17:25])
        yaw = Poly4D.Poly(row[25:33])
        trajectory_mem.trajectory.append(Poly4D(duration, x, y, z, yaw))
        total_duration += duration

    upload_result = Uploader().upload(trajectory_mem)
    if not upload_result:
        print('Upload failed, aborting!')
        sys.exit(1)
    cf.high_level_commander.define_trajectory(trajectory_id, 0, len(trajectory_mem.trajectory))
    return total_duration


def run_sequence(cf, trajectory_id, duration):
    commander = cf.high_level_commander

    commander.takeoff(0.5, 2.0)  #(altura absoluta en metros, duración en segundos )
    time.sleep(3.0)
    relative = True
    commander.start_trajectory(trajectory_id, 1.0, relative) # el 1 es la escala (time_scale)
    time.sleep(duration)
    commander.land(0.0, 2.0)
    time.sleep(2)
    commander.stop()


if __name__ == '__main__':
    
    #cflib.crtp.init_drivers()
    #----------------*****-----09/10 MicroSD----*****----------------------------------
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
    
    #----------------------------------------------------------------------------------------------
    #logging framework para lectura asincrona de la posicion, usando la funcion simple_log_async 
    lg_stab = LogConfig(name='stateEstimate', period_in_ms=10)
    lg_stab.add_variable('stateEstimate.x', 'float')
    lg_stab.add_variable('stateEstimate.y', 'float')
    lg_stab.add_variable('stateEstimate.z', 'float')
    #----------------------------------------------------------------------------------------------
    cf = cflib.crazyflie.Crazyflie()
    cf.console.receivedChar.add_callback(consoleReceived) #llama a la funcion consoleReceived
    #----------------*****----------------------*****--------------------------
    
    with SyncCrazyflie(URI, cf) as scf:

        #-----------------***** 09/10 MicroSD *****----------------------------
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

        #------------para la lectura de variables de registro de manera asincrona-----------
       # simple_log_async(scf, lg_stab)
        #-----------------------------------------------------------------------------------

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
        time.sleep(5)

        #Usd.logging controla si el registro en la tarjeta SD está activo. 
        # Se configura en 1 para iniciar el registro y en 0 para detener el registro, esta predeterminado en cero.
        # stop logging in uSD card
        #--------------------------seguimiento de trayectoria-------------
        cf = scf.cf
        trajectory_id = 1

        activate_high_level_commander(cf)
        ## activate_mellinger_controller(cf)
        duration = upload_trajectory(cf, trajectory_id, figure8)
        print('The sequence is {:.1f} seconds long'.format(duration))
        reset_estimator(cf)
        run_sequence(cf, trajectory_id, duration)
        #---------------------------------------------------------------------
        #--------------------------------------------------------------------------------------------------
        cf.param.set_value('usd.logging', 0)
        time.sleep(5)

    # Turn CF off
    s = PowerSwitch(URI)
    s.stm_power_down()
    s.close()              
       
