import csv
import numpy as np
import quaternion


def angle2quat(V,a):
    sin_a = np.sin(np.radians(a / 2))
    return np.quaternion(np.cos(np.radians(a / 2)),V[0]*sin_a,V[1]*sin_a,V[2]*sin_a)


def ComplementaryFilter(Q_gyro,V_accel,V_mag,K_accel,K_mag,Gravity_global = [0,0,-1],Geomag_global = [0,0.7071,-0.7071]):
    Gravity_local = quaternion.as_vector_part(Q_gyro * quaternion.from_vector_part(V_accel) * Q_gyro.conj())
    cos_Gravity_local_global = np.inner(Gravity_local, Gravity_global) / (np.linalg.norm(Gravity_local) * np.linalg.norm(Gravity_global))
    if (cos_Gravity_local_global > 1) or (cos_Gravity_local_global < -1):
        cos_Gravity_local_global = round(cos_Gravity_local_global,10)
    angle_Gravity_local_global = np.degrees(np.arccos(cos_Gravity_local_global))
    cross_Gravity_local_global = np.cross(Gravity_local,Gravity_global)
    cross_Gravity_local_global = cross_Gravity_local_global / np.linalg.norm(cross_Gravity_local_global)
    Q_filter_from_accel = angle2quat(cross_Gravity_local_global,angle_Gravity_local_global * K_accel)
    Q_return = Q_filter_from_accel * Q_gyro
    Geomag_local  = quaternion.as_vector_part(Q_return * quaternion.from_vector_part(V_mag) * Q_return.conj())
    cos_Geomag_local_global = np.inner(Geomag_local, Geomag_global) / (np.linalg.norm(Geomag_local) * np.linalg.norm(Geomag_global))
    if (cos_Geomag_local_global > 1) or (cos_Geomag_local_global < -1):
        cos_Geomag_local_global = round(cos_Geomag_local_global,10)
    angle_Geomag_local_global  = np.degrees(np.arccos(cos_Geomag_local_global))
    cross_Geomag_local_global  = np.cross(Geomag_local,Geomag_global)
    cross_Geomag_local_global  = cross_Geomag_local_global / np.linalg.norm(cross_Geomag_local_global)
    Q_filter_from_mag   = angle2quat(cross_Geomag_local_global,angle_Geomag_local_global * K_mag)
    Q_return = Q_filter_from_mag * Q_return
    return Q_return


def VectorNormalizer(V):
    return (V / np.linalg.norm(V))


calculation_times = 10 #to be squared

for number in range(1,100):
    results_path = r"A:\results-mpu9250-" + "{}.csv".format(number)
    save_detail_path = r"A:\results-experiments-detail.csv"
    save_path = r"A:\results-experiments-" + "{}.csv".format(number)
    
    with open(save_detail_path,'a',encoding='UTF-8') as s:
        s.write('K_accel,K_mag,gap,ans_quat_w,ans_quat_x,ans_quat_y,ans_quat_z,crc_quat_w,crc_quat_x,crc_quat_y,crc_quat_z\n')
    
    with open(save_path,'a',encoding='UTF-8') as t:
        for c in range(calculation_times + 1):
            t.write(',' + str(c / calculation_times))
        t.write('\n')
    
    for i in range(calculation_times + 1):
        results = []
        for j in range(calculation_times + 1):
            K_accel = i / calculation_times
            K_mag = j / calculation_times
            print('------------------------------------------------------------------------------------------------------------')
            print('K_accel={},K_mag={}'.format(K_accel,K_mag))
            f = open(results_path,'r')
            reader = csv.reader(f)
            firstLoop = True
            for row in reader:
                if firstLoop:
                    previous_values = [float(v) for v in row]
                    # quat_from_gyro = angle2quat([1,0,0],0)
                    quat_from_gyro = np.quaternion(1,0,0,0)
                    quat_filtered  = np.quaternion(1,0,0,0)
                    V_gravity_global = VectorNormalizer(np.array([previous_values[1],previous_values[2],previous_values[3]]))
                    V_geomag_global = np.array([previous_values[7],previous_values[8],previous_values[9]])
                    V_geomag_global = V_geomag_global / np.linalg.norm(V_geomag_global)
                    local_x_axis = np.array([1,0,0])
                    local_y_axis = np.array([0,1,0])
                    local_z_axis = np.array([0,0,1])
                    firstLoop = False
                elif "fall!" in row:
                    continue
                elif "Turn off the accels" in row:
                    quat_answer = quat_filtered
                    quat_from_gyro = np.quaternion(1,0,0,0)
                else:
                    values = [float(v) for v in row]
                    interval = values[0] - previous_values[0]
                    angle_from_gyro_x = ((values[4] + previous_values[4]) * interval / 2) #trapezoid approximate
                    angle_from_gyro_y = ((values[5] + previous_values[5]) * interval / 2) #trapezoid approximate
                    angle_from_gyro_z = ((values[6] + previous_values[6]) * interval / 2) #trapezoid approximate
                    V_gravity_local = VectorNormalizer(np.array([values[1],values[2],values[3]]))
                    V_geomag_local  = VectorNormalizer(np.array([values[7],values[8],values[9]]))
                    mag_is_normal = values[10]
                    if mag_is_normal != 16:
                        print('mag is not normal')
                        break
                    relative_quat_from_gyro = (angle2quat(local_z_axis,angle_from_gyro_z) * angle2quat(local_y_axis,angle_from_gyro_y) * angle2quat(local_x_axis,angle_from_gyro_x)).normalized() #x -> y -> z
                    quat_from_gyro = relative_quat_from_gyro * quat_from_gyro
                    local_x_axis = quaternion.as_vector_part(relative_quat_from_gyro * quaternion.from_vector_part(local_x_axis) * relative_quat_from_gyro.conj())
                    local_y_axis = quaternion.as_vector_part(relative_quat_from_gyro * quaternion.from_vector_part(local_y_axis) * relative_quat_from_gyro.conj())
                    local_z_axis = quaternion.as_vector_part(relative_quat_from_gyro * quaternion.from_vector_part(local_z_axis) * relative_quat_from_gyro.conj())
                    quat_filtered = ComplementaryFilter(relative_quat_from_gyro,V_gravity_local,V_geomag_local,K_accel,K_mag,Gravity_global=VectorNormalizer([previous_values[1],previous_values[2],previous_values[3]]),Geomag_global=VectorNormalizer([previous_values[7],previous_values[8],previous_values[9]])) * quat_filtered
                    previous_values = values
            quat_correct = quat_from_gyro.conj()
            gap_answer_correct = (quat_answer - quat_correct).absolute()
            results.append(gap_answer_correct)
            print(gap_answer_correct)
            print(quat_answer)
            print(quat_correct)
            with open(save_detail_path,'a',encoding='UTF-8') as s:
                s.write('{},{},{},{},{},{},{},{},{},{},{}\n'.format(K_accel,K_mag,gap_answer_correct,quat_answer.w,quat_answer.x,quat_answer.y,quat_answer.z,quat_correct.w,quat_correct.x,quat_correct.y,quat_correct.z,))
            f.close()
        with open(save_path,'a',encoding='UTF-8') as t:
            results = [str(v) for v in results]
            t.write(str(K_accel) + ',' + ','.join(results) + '\n')