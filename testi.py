import matplotlib.pyplot as plt
import math
import helpers as hp

def calculate_distance_from_beacon(beacon_median_rssis):
    distance_multiplier = 0
    offset = 66
    power = 2.2
    distances = []

    for rssi_abs in beacon_median_rssis:
        offset_rssi = hp.clamp_value(rssi_abs - offset, min_value=2)
        if distance_multiplier == 0:
            distance = hp.clamp_value(offset_rssi ** power, max_value=3000)
        else:
            distance = hp.clamp_value((offset_rssi * distance_multiplier) ** power, max_value=3000)
        distances.append(distance)

    return distances

def custom_rising_function(x):
    steepness_at_start = 0.2
    steepness_at_end = 0.2
    middle_point = 0.5
    middle_steepness = 10

    # Calculate the output using the Sigmoid function
    return 1.0 / (1.0 + math.exp(-steepness_at_start * (x - middle_point))) - 1.0 / (1.0 + math.exp(steepness_at_end * (x - middle_point))) + middle_steepness * x

def plot_rssi_distance_graph(rssi_values, distances):
    plt.plot(rssi_values, distances, 'bo')
    plt.xlabel('RSSI')
    plt.ylabel('Distance')
    plt.title('Relationship between RSSI and distance from beacon')
    plt.grid(True)
    plt.gca().invert_xaxis()
    plt.show()

if __name__ == "__main__":
    beacon_median_rssis = [round(x, 1) for x in [i/10 for i in range(0, 1000, 1)]]
    print(beacon_median_rssis)
    distances = calculate_distance_from_beacon(beacon_median_rssis)
    beacon_median_rssis_negative = [-x for x in beacon_median_rssis]
    plot_rssi_distance_graph(beacon_median_rssis_negative, distances)
