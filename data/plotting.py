import matplotlib.pyplot as plt
import csv

def main():
    with open('test9.csv') as csvfile:
        csv_reader = csv.reader(csvfile, delimiter=',')
        line_count = 0
        time = []
        height = []
        for row in csv_reader:
            if (line_count == 0):
                line_count += 1
            elif (len(row) == 2):
                time.append(float(row[0]))
                height.append(float(row[1]))
                line_count += 1
    
    fig1 = plt.figure()
    plt.plot(time,height)
    plt.xlabel('Time [s]')
    plt.legend(['Height [m]'])
    plt.title("Controller Z Coordinate")
    plt.show()
    return

main()