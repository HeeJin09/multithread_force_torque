from math import sin, cos, tan, pi
import time
import os
import sys
import can
import numpy as np
from multiprocessing import Process, Manager, freeze_support


#############################################################################
################## Manager for global data multiprocessing ##################
#############################################################################


def ft_sensor_run(force):
    init_time = time.time()
    DF = 50
    DT = 2000
    RFT_frq = 500
    channel = 0
    
    frq_cutoff = 5
    alpha = (frq_cutoff * (1 / RFT_frq)) / (1 + frq_cutoff * (1 / RFT_frq))
    F_ext_buf = np.array([0.0] * 6)
    F_ext_off = np.array([0.0] * 6) 
    
    bus = can.interface.Bus(bustype='kvaser', channel=0, bitrate=1000000)

    # Set filter
    tx_message = can.Message(arbitration_id=0x64, is_extended_id=False, data=[0x08, 0x01, 0x09, 0x01, 0x01, 0x01, 0x01, 0x01])
    bus.send(tx_message, timeout=0.5)
    bus.recv()

    tx_message = can.Message(arbitration_id=0x64, is_extended_id=False, data=[0x11, 0x01, 0x09, 0x01, 0x01, 0x01, 0x01, 0x01])
    bus.send(tx_message, timeout=0.5)
    bus.recv()
    
    while True:
        g_time = time.time()
        # read once
        tx_message = can.Message(arbitration_id=0x64, is_extended_id=False, data=[0x0A, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01])
        bus.send(tx_message, timeout=0.5)

        rx_message_1 = bus.recv()
        rx_message_2 = bus.recv()

        fx = ((rx_message_1.data[1] << 8) + rx_message_1.data[2])
        fy = ((rx_message_1.data[3] << 8) + rx_message_1.data[4])
        fz = ((rx_message_1.data[5] << 8) + rx_message_1.data[6])
        signed_fx = (-(fx & 0x8000) | (fx & 0x7fff)) / DF
        signed_fy = (-(fy & 0x8000) | (fy & 0x7fff)) / DF
        signed_fz = (-(fz & 0x8000) | (fz & 0x7fff)) / DF
        
        tx = ((rx_message_1.data[7] << 8) + rx_message_2.data[0])
        ty = ((rx_message_2.data[1] << 8) + rx_message_2.data[2])
        tz = ((rx_message_2.data[3] << 8) + rx_message_2.data[4])
        signed_tx = (-(tx & 0x8000) | (tx & 0x7fff)) / DT
        signed_ty = (-(ty & 0x8000) | (ty & 0x7fff)) / DT
        signed_tz = (-(tz & 0x8000) | (tz & 0x7fff)) / DT
        
        F_ext_tmp = np.array([signed_fx, signed_fy, signed_fz] + [signed_tx, signed_ty, signed_tz])
        F_ext = alpha * (F_ext_tmp - F_ext_off) + (1 - alpha) * F_ext_buf
        F_ext_buf = F_ext
        
        force_data = np.array([[F_ext[0], F_ext[1], F_ext[2]], [F_ext[3], F_ext[4], F_ext[5]]])
        force[0] = force_data
      
        print("hello")
        while time.time() - g_time < (1 / RFT_frq):
            time.sleep(1 / 100000000)
            glob_time_buf = time.time()
            init_time_buf = init_time
        #print(time.time()-g_time)


def worker(force):
    while True:
        if 0 in force:
            print(force[0])

        
if __name__ == '__main__':
    freeze_support()
    manager = Manager()
    force = manager.dict()
    
    ft_sensor_task = Process(target=ft_sensor_run, args=(force,))
    process = Process(target=worker, args=(force,))
    
    ft_sensor_task.start()
    process.start()

    ft_sensor_task.join()
    process.join()
