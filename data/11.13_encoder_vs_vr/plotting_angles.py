import matplotlib.pyplot as plt
import csv

def main():
    with open('test5_no_motion.csv') as csvfile:
        csv_reader = csv.reader(csvfile, delimiter=',')
        line_count = 0
        time = []
        current = []
        torque = []
        theta_vr = []
        theta_encoder = []

        offset = 0
        controller_ready = False

        for row in csv_reader:
            #skip first line
            if (line_count == 0):
                k = row[-1]
                line_count += 1

            elif (len(row) == 5):
                
                #skip data while theta_vr is 90
                if (not controller_ready and float(row[3]) != 90):
                    offset = float(row[3])-float(row[4])
                    controller_ready = True
                
                #store data in arrays
                elif (controller_ready):
                    time.append(float(row[0]))
                    current.append(float(row[1]))
                    torque.append(float(row[2]))
                    theta_vr.append(float(row[3]))
                    theta_encoder.append(float(row[4])+offset)
                    
                line_count += 1
    
    fig1 = plt.figure()
    plt.plot(time,theta_encoder)
    plt.plot(time,theta_vr)
    plt.xlabel('Time [s]')
    plt.ylabel('Angle [deg]')
    plt.legend(['VR Angle','Encoder Angle'])
    plt.title(k)
    plt.show()
    return

main()