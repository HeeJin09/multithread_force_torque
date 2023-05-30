from math import sin, cos, tan, pi
import time
import os
import sys
import can
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))
import numpy as np
from multiprocessing import Process, Manager, freeze_support
import pybullet as p
from math import *
from modern_robotics import *
import serial
from scipy.spatial.transform import Rotation
import matplotlib.pyplot as plt

############################################################################
########################save txt ####################
#############################################################################
def save_list_to_txt(lst, filename):
    with open(filename, 'w') as f:
        for item in lst:
            f.write(str(item) + '\n')


#############################################################################
############################### cross product ############################
#############################################################################
def cross_prod(a, b):
    result = [a[1]*b[2] - a[2]*b[1],
            a[2]*b[0] - a[0]*b[2],
            a[0]*b[1] - a[1]*b[0]]

    return result

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
        
        force_data = np.array([F_ext[3], F_ext[4], F_ext[5], F_ext[0], F_ext[1], F_ext[2]])
        force[0] = force_data
      
       


def worker(force):
    CONTROL_FREQ = 240.0
    dt = 1.0/CONTROL_FREQ
    
    eul_x = 0.0
    eul_y = 0.0
    eul_z = 0.0

    quat = p.getQuaternionFromEuler([eul_x,eul_y,eul_z])
    p.connect(p.GUI)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
    p.configureDebugVisualizer(p.COV_ENABLE_TINY_RENDERER,1)
    p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS,0)

    p.resetDebugVisualizerCamera(cameraDistance=0.15, cameraYaw=0.0, cameraPitch=-40, cameraTargetPosition=[0.0,0,0.05])
    p.setTimeStep(dt)
    p.setPhysicsEngineParameter(dt)
    p.setGravity(0, 0, -9.8)
    boxId = p.loadURDF("box2.urdf",[0.0, 0.0, 0.0],quat,useFixedBase=True)
    planeID = p.loadURDF("plane.urdf")
    arrow = p.loadURDF("abc.urdf",[1.0, 1.0, 10.0],quat)
    t = 0
    t_list = []
    r_Test_list_1 = []
    r_Test_list_2 = []
    r_Test_list_3 = []
    force_list = []
    moment_list = []
    
    

    R = p.getMatrixFromQuaternion(quat)
    R = np.reshape(R,[3,3])



    p.enableJointForceTorqueSensor(boxId,0,True)


    p.stepSimulation()
    #jointState = np.array(p.getJointState(boxId,0)[2]).T
    R_ = np.array([[0, 0, 1],[0, 1, 0],[1, 0, 0]])
    r = [0,0,0]
    init_quaternion = np.array([0.0, 0.0, 0.0, 1])
    delta_quat = np.array([0.0, 0.0, 0.0, 1])
    count = 0

    print("loop star")
    count_Test_1 = 0
    filtered_value_fx = 0.0
    filtered_value_fy = 0.0 
    filtered_value_fz = 0.0
    filtered_value_mx = 0.0
    filtered_value_my = 0.0
    filtered_value_mz = 0.0
    input_fx = []
    input_fy = []
    input_fz = []
    input_mx = []
    input_my = []
    input_mz = []
    Wrench_init = np.array([0.0, 0.0, 0.0, 0.0, 0.0, -5.3])
    result_Wrench = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    p_b = np.array([0,0, 0.060])
    r_Test_pre = np.array([0.0, 0.0, 0.0])
    euler_filter_pre = np.array([0.0, 0.0, 0.0])

    n1 = R@np.array([0,0,1]).T
    n2 = R@np.array([0,-1,0]).T
    n3 = R@np.array([0,1,0]).T
    n4 = R@np.array([-1,0,0]).T
    n5 = R@np.array([1,0,0]).T

    plane1 = R@np.array([0, 0, 0.145])
    plane2 = R@np.array([0, -0.06, 0.085])
    plane3 = R@np.array([0, 0.06, 0.085])
    plane4 = R@np.array([-0.06, 0, 0.085])
    plane5 = R@np.array([0.06, 0, 0.085])


    d1 = -(n1[0]*plane1[0]+n1[1]*plane1[1]+n1[2]*plane1[2])
    d2 = -(n2[0]*plane2[0]+n2[1]*plane2[1]+n2[2]*plane2[2])
    d3 = -(n3[0]*plane3[0]+n3[1]*plane3[1]+n3[2]*plane3[2])
    d4 = -(n4[0]*plane4[0]+n4[1]*plane4[1]+n4[2]*plane4[2])
    d5 = -(n5[0]*plane5[0]+n5[1]*plane5[1]+n5[2]*plane5[2])
   
    max_values = [-float('inf')] * 5
    min_values = [float('inf')] * 5
    different = np.array([0.0] * 5)
    max_min = [0.0] * 5
    r_norm = [0.0] * 5
    result_test_r =10.0
    min_index = 0
    test_count = 0


    current_norm = np.array([0.0] * 5)
    prev_norm = np.array([0.0] * 5)
    two_prev_norm = np.array([0.0] * 5)
    for t in np.arange(0,10,dt):
        if 0 in force:
            wrench_sensor = np.array(force[0])
            
            ##########init calculate wrench bias ############
            if (count_Test_1 < 101) :
                window_size = 100
                input_fx.append(wrench_sensor[3])
                input_fy.append(wrench_sensor[4])
                input_fz.append(wrench_sensor[5])
                input_mx.append(wrench_sensor[0])
                input_my.append(wrench_sensor[1])
                input_mz.append(wrench_sensor[2])

                if len(input_fx) >= window_size:
                    sum_fx = sum(input_fx[-window_size:])
                    sum_fy = sum(input_fy[-window_size:])
                    sum_fz = sum(input_fz[-window_size:])
                    sum_mx = sum(input_mx[-window_size:])
                    sum_my = sum(input_my[-window_size:])
                    sum_mz = sum(input_mz[-window_size:])


                    filtered_value_fx = sum_fx / window_size
                    filtered_value_fy = sum_fy / window_size
                    filtered_value_fz = sum_fz / window_size
                    filtered_value_mx = sum_mx / window_size
                    filtered_value_my = sum_my / window_size
                    filtered_value_mz = sum_mz / window_size
                elif  len(input_fx) < window_size:
                    filtered_value_fx = wrench_sensor[3]
                    filtered_value_fy = wrench_sensor[4]
                    filtered_value_fz = wrench_sensor[5]
                    filtered_value_mx = wrench_sensor[0]
                    filtered_value_my = wrench_sensor[1]
                    filtered_value_mz = wrench_sensor[2]

            if count_Test_1 == 101:
                print(" #############################")
                print(" ######## START WRENCH #########")
                print(" #############################")
            filter_Wrench_base = np.array([filtered_value_mx, filtered_value_my, filtered_value_mz, filtered_value_fx, filtered_value_fy, filtered_value_fz + 5.3])
            filter_Wrench = wrench_sensor -  filter_Wrench_base
            
            
            count_Test_1 = count_Test_1 + 1
            
            ##############calcuate adjoint body wrech init #########################
            adjoint_Wrench = Adjoint(RpToTrans(R, p_b).T) @ Wrench_init.T
            adjoint_force = np.array([adjoint_Wrench[3], adjoint_Wrench[4], adjoint_Wrench[5]])
            adjoint_moment = cross_prod(p_b, adjoint_force)

            result_Wrench_body = np.array([adjoint_moment[0], adjoint_moment[1], adjoint_moment[2], adjoint_force[0], adjoint_force[1], adjoint_force[2]])
            result_Wrench = filter_Wrench - result_Wrench_body

            n1 = np.array([0,0,1]).T
            n2 = np.array([0,-1,0]).T
            n3 = np.array([0,1,0]).T
            n4 = np.array([-1,0,0]).T
            n5 = np.array([1,0,0]).T


            plane1 = np.array([0, 0, 0.145])
            plane2 = np.array([0, -0.06, 0.085])
            plane3 = np.array([0, 0.06, 0.085])
            plane4 = np.array([-0.06, 0, 0.085])
            plane5 = np.array([0.06, 0, 0.085])

            d1 = -(n1[0]*plane1[0]+n1[1]*plane1[1]+n1[2]*plane1[2])
            d2 = -(n2[0]*plane2[0]+n2[1]*plane2[1]+n2[2]*plane2[2])
            d3 = -(n3[0]*plane3[0]+n3[1]*plane3[1]+n3[2]*plane3[2])
            d4 = -(n4[0]*plane4[0]+n4[1]*plane4[1]+n4[2]*plane4[2])
            d5 = -(n5[0]*plane5[0]+n5[1]*plane5[1]+n5[2]*plane5[2])
        
            ################calculate A matrix ########################
            f = np.array([result_Wrench[3],result_Wrench[4],result_Wrench[5]])
            norm_f= np.linalg.norm(f)
            u = f/norm_f

            
            U1 = np.array([[0 ,-u[2] ,u[1]],[u[2] ,0 ,-u[0]],[-u[1], u[0], 0],[n1[0], n1[1], n1[2]]])
            U2 = np.array([[0 ,-u[2] ,u[1]],[u[2] ,0 ,-u[0]],[-u[1], u[0], 0],[n2[0], n2[1], n2[2]]])
            U3 = np.array([[0 ,-u[2] ,u[1]],[u[2] ,0 ,-u[0]],[-u[1], u[0], 0],[n3[0], n3[1], n3[2]]]) 
            U4 = np.array([[0 ,-u[2] ,u[1]],[u[2] ,0 ,-u[0]],[-u[1], u[0], 0],[n4[0], n4[1], n4[2]]]) 
            U5 = np.array([[0 ,-u[2] ,u[1]],[u[2] ,0 ,-u[0]],[-u[1], u[0], 0],[n5[0], n5[1], n5[2]]])

            ################[r1 : +Z], [r2 : -Y], [r2 : +Y], [r2 : -X], [r5 : +x] 
            ################calculate r (distance) ########################
            try:
                r1 = np.linalg.inv(U1.T @ U1)@U1.T @ np.array([-result_Wrench[0]/norm_f,-result_Wrench[1]/norm_f,-result_Wrench[2]/norm_f,-d1 ])
                #if r1[0] > 0.065 or r1[0] < -0.065 or r1[1] > 0.06 or r1[1] < -0.06 or r1[2] > 0.145  or r1[2] < 0.145 : r1[0] = [0,0,0] 
                if norm_f < 1.0 :  r1[0] = [0,0,0] 
                
            except:
                r1 = [0,0,0]

            try:
                r2 = np.linalg.inv(U2.T @ U2)@U2.T @np.array([-result_Wrench[0]/norm_f,-result_Wrench[1]/norm_f,-result_Wrench[2]/norm_f,-d2 ])
                #if r2[0] > 0.065 or r2[0] < -0.065 or r2[1] > -0.06 or r2[1] < -0.06 or r2[2] > 0.146 or r2[2] < 0.005  : r2 = [0,0,0]
                if norm_f < 1.0 :  r2[0] = [0,0,0] 
            except:
                r2 = [0,0,0]
                
            try:
                r3 = np.linalg.inv(U3.T @ U3)@U3.T @np.array([-result_Wrench[0]/norm_f,-result_Wrench[1]/norm_f,-result_Wrench[2]/norm_f,-d3 ])
                #if r3[0] > 0.065 or r3[0] < -0.065 or r3[1] > 0.06 or r3[1] < 0.06  or r3[2] > 0.146 or r3[2] < 0.005: r3 = [0,0,0]
                if norm_f < 1.0 :  r3[0] = [0,0,0] 
            except:
                r3 = [0,0,0]	

            try:
                r4 = np.linalg.inv(U4.T @ U4)@U4.T @np.array([-result_Wrench[0]/norm_f,-result_Wrench[1]/norm_f,-result_Wrench[2]/norm_f,-d4 ])
                #if r4[0] < -0.06 or r4[0] > -0.06 or r4[1] > 0.065 or r4[1] < -0.065 or r4[2] > 0.146 or r4[2] < 0.025 : r4 = [0,0,0]
                if norm_f < 1.0 :  r4[0] = [0,0,0]
            except:
                r4 = [0,0,0]

            try:
                r5 = np.linalg.inv(U5.T @ U5)@U5.T @np.array([-result_Wrench[0]/norm_f,-result_Wrench[1]/norm_f,-result_Wrench[2]/norm_f,-d5 ])
                #if r5[0] > 0.06 or r5[0] < 0.06 or r5[1] > 0.065 or r5[1] < -0.065 or r5[2] > 0.146 or r5[2] < 0.025 : r5 = [0,0,0]
                if norm_f < 1.0 :  r5[0] = [0,0,0]
            except:
                r5 = [0,0,0]

            ########## determine r from force vector ###########
            VV = np.array([0,1,0])
            QQ = np.cross([0,1,0],u)
            QQ_norm = np.linalg.norm(QQ)
            QQ = QQ/QQ_norm

            innerAB = np.dot(VV,u)
            AB = np.linalg.norm(VV) * np.linalg.norm(u)
            Angle = np.arccos(innerAB/AB)
            QQ = np.array([np.sin(Angle/2)*QQ[0], np.sin(Angle/2)*QQ[1], np.sin(Angle/2)*QQ[2], np.cos(Angle/2)])
            

            r_Test = np.array([r1, r2, r3, r4, r5])

                
            if norm_f > 1.0:
                for i in range(5):
                    r_norm[i] = np.linalg.norm(np.array([r_Test[i][0], r_Test[i][1], r_Test[i][2]]))
                    if test_count >= 3:  # 두 번째 값부터 변화량 계산
                        current_norm[i] = r_norm[i]
                        different[i] = np.linalg.norm(current_norm[i] - two_prev_norm[i])
                        if r_norm[i] > max_values[i]:
                            max_values[i] = r_norm[i]
                        if r_norm[i] < min_values[i]:
                            min_values[i] = r_norm[i]
                        max_min[i] = np.array(max_values - min_values)[i]
                    two_prev_norm[i] = prev_norm[i]
                    prev_norm[i] = current_norm[i]
                test_count += 1

                # p.resetBasePositionAndOrientation(arrow, r, QQ)
            else:
                max_values = np.array([-float('inf')] * 5)
                min_values = np.array([float('inf')] * 5)
                current_norm = np.array([0.0] * 5)
                prev_norm = np.array([0.0] * 5)
                two_prev_norm = np.array([0.0] * 5)
                different = np.array([0.0] * 5)
                min_index = None  # None으로 초기화
                result_test_r = None
                max_min = np.array([0.0] * 5)
                test_count = 0
                r = [1, 1, 1]

            p.resetBasePositionAndOrientation(arrow, r, QQ)

            for j, num in enumerate(max_min):
                if num != 0.0 and (min_index is None or num < different[min_index]):
                    min_index = j

            if min_index is not None:
                r = r_Test[min_index]
            print("different : " , different )
            print("test_count : " , test_count )
            nonzero_rows = ~np.all(r_Test == 0, axis=1)
            r_Test_list_1.append(r_Test[3])
            r_Test_list_2.append(r_Test[4])

            r_Test_list_3.append(max_min)
            force_list.append(f)
            moment_list.append(np.array([result_Wrench[0], result_Wrench[1], result_Wrench[2]]))
            #print(r_Test)
            t_list.append(t)
            t = t + dt

        time.sleep(dt)
    
    

    
    save_list_to_txt(np.array([elem[2] for elem in r_Test_list_2]), 'r+.txt')
    save_list_to_txt(np.array([elem[2] for elem in r_Test_list_1]), 'r-.txt')
    save_list_to_txt(t_list, 'time.txt')
    ax = plt.subplot(311)
    plt.plot(t_list, np.array([elem[2] for elem in r_Test_list_3]), label='rx')
    plt.plot(t_list, np.array([elem[3] for elem in r_Test_list_3]), label='ry')
    plt.plot(t_list, np.array([elem[4] for elem in r_Test_list_3]), label='rz')    
    plt.xlabel('time[s]')
    plt.ylabel('[r]')  
    plt.title("r")
    plt.grid(linestyle = ':', linewidth = 1)
    plt.grid(True)
    plt.legend()

    
    ax = plt.subplot(312)

    plt.plot(t_list, np.array([elem[0] for elem in moment_list]), label='mx')
    plt.plot(t_list, np.array([elem[1] for elem in moment_list]), label='my')
    plt.plot(t_list, np.array([elem[2] for elem in moment_list]), label='mz')
    plt.xlabel('time[s]')
    plt.ylabel('[fx]')
    plt.title("measured moment")
    plt.grid(linestyle = ':', linewidth = 1)
    plt.grid(True)
    plt.legend()

    ax = plt.subplot(313)
    plt.plot(t_list, np.array([elem[0] for elem in force_list]), label='fx')
    plt.plot(t_list, np.array([elem[1] for elem in force_list]), label='fy')
    plt.plot(t_list, np.array([elem[2] for elem in force_list]), label='fz')
    plt.xlabel('time[s]')
    plt.ylabel('[fx]')
    plt.title("measured force")
    plt.grid(linestyle = ':', linewidth = 1)
    plt.legend()
    

    '''
    ax = plt.subplot(413)

    plt.plot(t_list, np.array([elem[0] for elem in force_list]), label='fx')
    plt.plot(t_list, np.array([elem[1] for elem in force_list]), label='fy')
    plt.plot(t_list, np.array([elem[2] for elem in force_list]), label='fz')
    plt.xlabel('time[s]')
    plt.ylabel('[fx]')
    plt.title("measured moment")
    plt.grid(linestyle = ':', linewidth = 1)
    plt.grid(True)
    plt.legend()

    ax = plt.subplot(414)
    plt.plot(t_list, np.array([elem[0] for elem in moment_list]), label='mx')
    plt.plot(t_list, np.array([elem[1] for elem in moment_list]), label='my')
    plt.plot(t_list, np.array([elem[2] for elem in moment_list]), label='mz')
    plt.xlabel('time[s]')
    plt.ylabel('[mx]')
    plt.title("measured moment")
    plt.grid(linestyle = ':', linewidth = 1)
    plt.legend()
    '''
    plt.grid(True)
    plt.show()

        
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
    
