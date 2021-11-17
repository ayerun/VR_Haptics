import matplotlib.pyplot as plt
import csv

def main():
    with open('test6.csv') as csvfile:
        csv_reader = csv.reader(csvfile, delimiter=',')
        line_count = 0
        i = 0
        time_sum = 0
        height_sum = 0
        time = []
        height = []
        for row in csv_reader:
            if (line_count == 0):
                line_count += 1
            elif (len(row) == 2):
                if i == 3:
                    time.append(time_sum/3)
                    height.append(height_sum/3)
                    time_sum = 0
                    height_sum = 0
                    i = 0
                else:
                    time_sum += float(row[0])
                    height_sum += float(row[1])
                i += 1
                line_count += 1
    
    fig1 = plt.figure()
    plt.plot(time,height)
    plt.xlabel('Time [s]')
    plt.legend(['Height [m]'])
    plt.title("Controller Z Coordinate")
    plt.show()
    return

main()