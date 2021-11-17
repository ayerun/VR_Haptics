import matplotlib.pyplot as plt
import csv

def main():
    with open('test11.csv') as csvfile:
        csv_reader = csv.reader(csvfile, delimiter=',')
        line_count = 0
        time = []
        height = []
        last_time = 0
        last_height = 0
        for row in csv_reader:
            if (line_count == 0):
                line_count += 1
            elif (len(row) == 2):
                speed = (float(row[1])-last_height)/(float(row[0])-last_time)
                if abs(speed) < 1:
                    time.append(float(row[0]))
                    height.append(float(row[1]))
                last_time = float(row[0])
                last_height = float(row[1])
                line_count += 1
    print(len(time))
    fig1 = plt.figure()
    plt.plot(time,height)
    plt.xlabel('Time [s]')
    plt.legend(['Height [m]'])
    plt.title("Controller Z Coordinate")
    plt.show()
    return

main()