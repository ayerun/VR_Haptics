import matplotlib.pyplot as plt
import csv

def main():
    with open('encoder_test4.csv') as csvfile:
        csv_reader = csv.reader(csvfile, delimiter=',')
        line_count = 0
        time = []
        current = []
        torque = []
        theta = []
        for row in csv_reader:
            if (line_count == 0):
                line_count += 1
            elif (len(row) == 4):
                time.append(float(row[0]))
                current.append(float(row[1]))
                torque.append(float(row[2]))
                theta.append(float(row[3]))
                line_count += 1
    
    fig1 = plt.figure()
    plt.plot(time,current)
    plt.plot(time,torque)
    plt.plot(time,theta)
    plt.xlabel('Time [s]')
    plt.legend(['Current [A]','Torque [Nm]', 'Angle [Degrees]'])
    plt.title("k = 6")
    plt.show()
    return

main()
# if __name__ == '__main__':
#     try:
#         main()
#         print("hi")
#     except:
#         pass