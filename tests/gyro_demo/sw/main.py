import serial_json

import numpy





if __name__ == "__main__":


    # Example loop
    roll = pitch = yaw = 0
  
    port_name   = "/dev/tty.usbserial-110"
    json_loader = serial_json.SerialJson(port_name)

    time_start = None
    time_end   = 600*1000 # 600 seconds in ms

    angle_start = None

    t_result    = []
    g_dps       = []
    g_angle     = []
    
    while True:
        parsed, success, dt = json_loader.get_json()

        if success:
            time_ms = float(parsed["time"])
            gyro = numpy.array(parsed["gyro"])

            dps   = gyro[0]
            angle = gyro[1]

            
            if time_start is None:
                time_start = time_ms    

            if angle_start is None:
                angle_start = angle

            t_tmp = time_ms - time_start


            print("time (ms):", time_ms)    
            print("angle (deg): ", angle - angle_start)
            print()


            t_result.append(t_tmp)
            g_dps.append(dps)
            g_angle.append(angle - angle_start)

            if t_tmp > time_end:
                break
    
    t_result = numpy.array(t_result)
    g_dps = numpy.array(g_dps)
    g_angle = numpy.array(g_angle)

    
    print("gyro integrated angle in degrees")
    print("\t\t\tmean\t\tstd\t\tmax")

    indices = numpy.where(t_result < 1000)[0]
    g_angle_1s = g_angle[indices]
    print("mean_angle_1s  ", "\t", round(g_angle_1s.mean(), 5), "\t", round(g_angle_1s.std(), 5), "\t", round(numpy.abs(g_angle_1s).max(), 5))


    indices = numpy.where(t_result < 10000)[0]
    g_angle_10s = g_angle[indices]
    print("mean_angle_10s  ", "\t", round(g_angle_10s.mean(), 5), "\t", round(g_angle_10s.std(), 5), "\t", round(numpy.abs(g_angle_10s).max(), 5))


    indices = numpy.where(t_result < 60000)[0]
    g_angle_60s = g_angle[indices]
    print("mean_angle_60s  ", "\t", round(g_angle_60s.mean(), 5), "\t", round(g_angle_60s.std(), 5), "\t", round(numpy.abs(g_angle_60s).max(), 5))

    indices = numpy.where(t_result < 600000)[0]
    g_angle_600s = g_angle[indices]
    print("mean_angle_600s ", "\t", round(g_angle_600s.mean(), 5), "\t", round(g_angle_600s.std(), 5), "\t", round(numpy.abs(g_angle_600s).max(), 5))

    
    # plot histogram of g_angle
    import matplotlib.pyplot as plt 
    plt.hist(g_angle, bins=20)
    plt.xlabel("angle (deg)")
    plt.ylabel("count")
    plt.title("Histogram of angle measurements")
    plt.show()
                


    print("\n\ndone")