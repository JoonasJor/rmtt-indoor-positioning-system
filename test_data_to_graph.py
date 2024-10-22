import matplotlib.pyplot as plt
import helpers as hp
import ast
import re
import numpy as np
from easy_trilateration.model import *  
from easy_trilateration.least_squares import easy_least_squares
import multiprocessing
from tqdm.contrib.concurrent import process_map
from collections import deque

def calculate_distance(rssi, multiplier, offset, power):
    offset_rssi = hp.clamp_value(rssi - offset, min_value=2)
    if multiplier == 0:
        distance = hp.clamp_value(offset_rssi ** power, max_value=1600)
    else:
        distance = hp.clamp_value((offset_rssi * multiplier) ** power, max_value=1600)
    return distance


def interpolate_along_path(points, num_points):
    if len(points) <= 1:
        return points * num_points
    
    path_length = 0

    for i in range(len(points) - 1):
        x1, y1 = points[i]
        x2, y2 = points[i + 1]
        segment_length = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)
        path_length += segment_length
    
    interpolated_points = []
    accumulated_distance = 0
    
    for i in range(len(points) - 1):
        x1, y1 = points[i]
        x2, y2 = points[i + 1]
        segment_length = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)
        num_interpolated_points = int(np.round(num_points * segment_length / path_length))
        
        for j in range(num_interpolated_points):
            t = j / (num_interpolated_points - 1)
            x_interp = x1 + t * (x2 - x1)
            y_interp = y1 + t * (y2 - y1)
            interpolated_points.append((x_interp, y_interp))
        
        accumulated_distance += segment_length
    
    return interpolated_points


def calculate_difference(drone_x, drone_y, points):
    differences_x, differences_y = [], []
    differences_euclidean = []

    if len(points) != 1:
        for i, point in enumerate(points):
            differences_x.append(abs(point[0] - drone_x[hp.clamp_value(i - 1, min_value=1)]))
            differences_y.append(abs(point[1] - drone_y[hp.clamp_value(i - 1, min_value=1)]))
            difference_euclidean = np.sqrt(differences_x[-1]**2 + differences_y[-1]**2)
            differences_euclidean.append(difference_euclidean)
    else:
        for i in range(len(drone_x)):
            differences_x.append(abs(points[0][0] - drone_x[hp.clamp_value(i - 1, min_value=1)]))
            differences_y.append(abs(points[0][1] - drone_y[hp.clamp_value(i - 1, min_value=1)]))
            difference_euclidean = np.sqrt(differences_x[-1]**2 + differences_y[-1]**2)
            differences_euclidean.append(difference_euclidean)
    
    return differences_x, differences_y, differences_euclidean

def calculate_weight(difference, screen_size, weight_power):
    weight = hp.clamp_value((difference / screen_size) ** weight_power, max_value=1)
    return weight

def weighted_midpoint_coordinates(x_A, y_A, x_B, y_B, beacon_distances, weight_power):
    distances_sorted = sorted(beacon_distances)
    lowest_distance = distances_sorted[0]
    second_lowest_distance = distances_sorted[1]
    
    difference = second_lowest_distance - lowest_distance
    screen_size = (2800, 2100)
    weight_x = calculate_weight(difference, screen_size[0], weight_power)
    weight_y = calculate_weight(difference, screen_size[1], weight_power)

    midpoint_x =hp.clamp_value(x_A + (x_B - x_A) * weight_x, max_value=screen_size[0])
    midpoint_y = hp.clamp_value(y_A + (y_B - y_A) * weight_y, max_value=screen_size[1])

    return midpoint_x, midpoint_y

def get_point_between_triresult_closest_beacon(beacon_distances: list, trilateration_result: Circle, beacon_locations, weight_power):
    closest_beacon_index = beacon_distances.index(min(beacon_distances))
    closest_beacon_location = beacon_locations[closest_beacon_index]
    
    mid_point = weighted_midpoint_coordinates(
                trilateration_result.center.x, trilateration_result.center.y, 
                closest_beacon_location[0], closest_beacon_location[1],
                beacon_distances, weight_power)
    return mid_point

def get_filtered_average_rssis(rolling_rssis):
    filtered_average_rssis = []
    try:
        for rssis in rolling_rssis:
            # remove lowest and highest values
            # and take average of remaining 3 values
            sorted_rssis = sorted(rssis)[1:-1] 
            avg_rssi = sum(sorted_rssis) / len(sorted_rssis)
            filtered_average_rssis.append(avg_rssi)    
    except Exception as e:
        print(f"get_filtered_average_rssis(): {e}")

    return filtered_average_rssis

def recalculate_coordinates(multiplier, offset, power, all_parsed_rssis, weight_power, o1, o2, o3, o4):
    beacon_locations = hp.get_beacon_locations("beacon_locations.txt")  
    beacon_locations = [[x * 2 for x in beacon_location] for beacon_location in beacon_locations] 
    tri_x, tri_y = [], []
    reprocessed_x, reprocessed_y = [], []
    rssi_individual_offsets = [o1, o2, o3, o4]

    for rssis in all_parsed_rssis:
        circles = []
        calculated_distances = []
        for i, rssi in enumerate(rssis):
            calculated_distance = calculate_distance(abs(rssi) - rssi_individual_offsets[i], multiplier, offset, power)
            calculated_distances.append(calculated_distance)
            circle = Circle(beacon_locations[i][0], beacon_locations[i][1], calculated_distance)
            circles.append(circle)
        trilateration_result, meta = easy_least_squares(circles, guess=Circle(1000, 1000, 100))
        tri_x.append(trilateration_result.center.x)
        tri_y.append(trilateration_result.center.y)
        re_x, re_y = get_point_between_triresult_closest_beacon(calculated_distances, trilateration_result, beacon_locations, weight_power)
        reprocessed_x.append(re_x)
        reprocessed_y.append(re_y)

    return tri_x, tri_y, reprocessed_x, reprocessed_y

def generate_all_possible_values(index, step_sizes, max_multipliers, current_values, initial_values):
    if index == len(current_values):
        # sum initial values to the list before returning it
        current_values = [current_values[i] + initial_values[i] for i in range(len(current_values))]
        print(current_values)
        return [current_values[:]]

    possible_combinations = []

    for multiplier in range(0, max_multipliers[index]):
        current_values[index] = round(multiplier * step_sizes[index], 3)
        possible_combinations.extend(generate_all_possible_values(index + 1, step_sizes, max_multipliers, current_values, initial_values))

    return possible_combinations

'''
def optimize_parameters(interpolated_points):
    parameters = [0.1, 0, 0.1] #multiplier, offset, power
    avg_difference = 10000
    recalc_tri_x, recalc_tri_y = [], []
    smallest_mae = [parameters, avg_difference, recalc_tri_x, recalc_tri_y]

    parameter_step_sizes = [0.15, 2, 0.15]
    max_multiplier = 20
    possible_values = generate_all_possible_values(0, parameter_step_sizes, max_multiplier, parameters)
    number_of_values = len(possible_values)

    for i, value in enumerate(possible_values):   
        recalc_tri_x, recalc_tri_y, recalc_reprocessed_x, recalc_reprocessed_y = recalculate_coordinates(value[0], value[1], value[2])
        recalc_differences_x, recalc_differences_y, recalc_differences_euclidean = calculate_difference(recalc_tri_x, recalc_tri_y, interpolated_points)
        avg_difference = sum(recalc_differences_euclidean) / len(recalc_differences_euclidean)

        if avg_difference < smallest_mae[1]:
            smallest_mae[0] = value
            smallest_mae[1] = avg_difference
            smallest_mae[2] = recalc_tri_x
            smallest_mae[3] = recalc_tri_y    
        
        print(f"{i+1}/{number_of_values} \tCurrent: {value} {avg_difference} \tBest: {smallest_mae[0]}{smallest_mae[1]}")
    
    return smallest_mae, recalc_differences_euclidean
'''
def get_rolling_average(x: list, window_size: int):
    rolling_average = []

    half_window = window_size // 2
    for i in range(half_window, len(x) - half_window):
        average = np.mean(x[i - half_window : i + half_window + 1])
        rolling_average.append(average)

    return rolling_average


def calculate_average_error(recalc_differences_euclidean, error_calculation_method):
    average_error = 0

    if error_calculation_method == 0: # Mean Average Error (MAE)
        average_error = np.mean(recalc_differences_euclidean)
    elif error_calculation_method == 1: # Root Mean Squared Error (RMSE)
        squared_differences_euclidian = [x ** 2 for x in recalc_differences_euclidean]
        average_error = np.mean(squared_differences_euclidian) ** 0.5
    elif error_calculation_method == 2: # Mean Squared Logarithmic Error (MSLE)
        average_error = np.mean(np.log1p(np.array(recalc_differences_euclidean)) ** 2)

    return average_error

def worker(params):
    value, interpolated_points, all_parsed_rssis, result_type, error_calculation_method = params
    recalc_tri_x, recalc_tri_y, recalc_reprocessed_x, recalc_reprocessed_y = recalculate_coordinates(value[0], value[1], value[2], all_parsed_rssis, 
                                                                                                     value[3], value[4], value[5], value[6], value[7])
    if result_type == 0:
        recalc_differences_x, recalc_differences_y, recalc_differences_euclidean = calculate_difference(recalc_tri_x, recalc_tri_y, interpolated_points)
    else:
        recalc_differences_x, recalc_differences_y, recalc_differences_euclidean = calculate_difference(recalc_reprocessed_x, recalc_reprocessed_y, interpolated_points)
    
    average_error = calculate_average_error(recalc_differences_euclidean, error_calculation_method)

    return value, average_error, recalc_tri_x, recalc_tri_y, recalc_differences_euclidean, recalc_reprocessed_x, recalc_reprocessed_y
    

def optimize_parameters_multiprocess(interpolated_points, all_parsed_rssis, result_type, error_calculation_method):
    # multiplier, offset, power, weight_power, individual_rssi_offsets1,2,3,4
    parameter_initial_values =  [0.0, 52, 1.8, 0.25, 0.0, 0.0, 0.0, 0.0]
    parameter_step_sizes =      [0.25, 3, 0.2, 0.2, 2, 2, 2, 2]
    parameter_max_multipliers = [1, 6, 5, 4, 3, 3, 3, 3]
    #parameter_max_multipliers = [1, 1, 1, 1, 1, 1, 1, 1]

    possible_values = generate_all_possible_values(0, parameter_step_sizes, parameter_max_multipliers, 
                                                  [0, 0, 0, 0, 0, 0, 0, 0], parameter_initial_values)
    # parameters for worker processes
    params_list = [(value, interpolated_points, all_parsed_rssis, result_type, error_calculation_method) for value in possible_values]
    # parallel process all possible combinations
    results = list(process_map(worker, params_list, max_workers=multiprocessing.cpu_count(), chunksize=1))
    # Find the best result among all processes
    smallest_error = min(results, key=lambda x: x[1])

    return smallest_error


def main():
    file_path = 'test_rssis_sisapiha_neliö.txt'

    calculated_distances, times, actual_distances = [[], [], [], []], [], []
    triresult_drone_x, triresult_drone_y = [], []
    reprocessed_drone_x, reprocessed_drone_y = [], []
    all_filttered_parsed_rssis = []
    all_parsed_rssis = []
    all_rolling_rssis = []
    result_type = 1 # 0 = raw trilateration 1 = reprocessed trilateration
    error_calculation_method = 1 # 0=MAE 1=RMSE 2=MSLE
    filter_rssis = False

    with open(file_path, 'r') as file:
        lines = file.readlines()
        beacon_locations = lines.pop()
        for line in lines:
            values = re.split(r'(?<![:,])\s(?!:)', line.strip())
            filttered_parsed_rssis = ast.literal_eval(values[0])
            all_filttered_parsed_rssis.append(filttered_parsed_rssis)
            for i, rssi in enumerate(filttered_parsed_rssis):
                calculated_distance = calculate_distance(abs(rssi), 0.325, 43, 2.8)
                calculated_distances[i].append(calculated_distance)
            parsed_rssis = ast.literal_eval(values[6])
            all_parsed_rssis.append(parsed_rssis)
            times.append(float(values[2]))
            coordinates = ast.literal_eval(values[4])
            triresult_drone_x.append(coordinates[0] * 2)
            triresult_drone_y.append(coordinates[1] * 2)
            reprocessed_coordinates = ast.literal_eval(values[3])
            reprocessed_drone_x.append(reprocessed_coordinates[0] * 2)
            reprocessed_drone_y.append(reprocessed_coordinates[1] * 2)
            rolling_rssis = eval(values[5])
            all_rolling_rssis.append(rolling_rssis)

    # Actual route taken (cm)
    points = [(250, 2000), (250, 500), (1800, 500), (1800, 1600), (250, 1600), (250, 2000)]

    cut = round(len(times)/2)
    times = times[:cut]
    all_rolling_rssis = all_rolling_rssis[cut:]
    all_parsed_rssis = all_parsed_rssis[cut:]
    reprocessed_drone_x = reprocessed_drone_x[cut:]
    reprocessed_drone_y = reprocessed_drone_y[cut:]  

    # Extract x and y coordinates from the points
    actual_x, actual_y = zip(*points)
    # Generate points between the given points
    number_of_points = len(reprocessed_drone_x)
    interpolated_points = interpolate_along_path(points, number_of_points)
    if result_type == 0:
        differences_x, differences_y, differences_euclidean = calculate_difference(triresult_drone_x, triresult_drone_y, interpolated_points)
    else:
        differences_x, differences_y, differences_euclidean = calculate_difference(reprocessed_drone_x, reprocessed_drone_y, interpolated_points)

    original_average_error = calculate_average_error(differences_euclidean, error_calculation_method)
    print(interpolated_points)
    if filter_rssis:
        recalc_filttered_rssis = [get_filtered_average_rssis(x) for x in all_rolling_rssis]
        smallest_error = optimize_parameters_multiprocess(interpolated_points, recalc_filttered_rssis, result_type, error_calculation_method)
    else:
        smallest_error = optimize_parameters_multiprocess(interpolated_points, all_parsed_rssis, result_type, error_calculation_method)
    print(f"Best result: {smallest_error[0]} {smallest_error[1]}\nOriginal: [0.325, 43, 2.8, 0.55] {original_average_error}")
    #Best result: [0.7, 33.5, 2.125, 0.7] 220.70350466191002
    #Best result: [0.0, 66, 2.2, 0.5, 1.5, 1.5, 1.5, 0.0]

    # get largest value among both lists
    plot_y_max = max([max(differences_euclidean), max(smallest_error[4])]) + 50
    #plot_y_max = max(smallest_error[4]) + 50

    if len(times) > len(differences_euclidean): times.pop() #nonono
    elif len(times) < len(differences_euclidean): times.append(times[-1] + 0.2) #end me

    rolling_avg_window_size = 30
    times_index = round(rolling_avg_window_size / 2)

    #value, average_error, recalc_tri_x, recalc_tri_y, recalc_differences_euclidean, recalc_reprocessed_x, recalc_reprocessed_y
    plt.subplot(2, 2, 1)
    plt.plot(times, differences_euclidean, label="Difference (cm)")
    plt.plot(times[times_index:-times_index], 
             get_rolling_average(differences_euclidean, rolling_avg_window_size), label="Rolling Average")
    plt.xlabel('Time (s)')
    plt.ylabel('Difference (cm)')
    plt.title('Difference between Estimated and Real Location (Original)')
    plt.legend()
    plt.ylim(0, plot_y_max)
    plt.grid(True)

    plt.subplot(2, 2, 2)
    plt.plot(times, smallest_error[4], label="Difference (cm)")
    plt.plot(times[times_index:-times_index], 
             get_rolling_average(smallest_error[4], rolling_avg_window_size), label="Rolling Average")
    plt.xlabel('Time (s)')
    plt.ylabel('Difference (cm)')
    plt.title('Difference between Estimated and Real Location (Recalculated)')
    plt.legend()
    plt.ylim(0, plot_y_max)
    plt.grid(True)

    plt.subplot(2, 2, 3)
    if result_type == 0:
        plt.scatter(triresult_drone_x, triresult_drone_y, color='green', marker='x', label="Estimated Location")      
    else:
        plt.scatter(reprocessed_drone_x, reprocessed_drone_y, color='green', marker='x', label="Estimated Location")
    plt.xlabel('X (cm)')
    plt.ylabel('Y (cm)')
    plt.title('Estimated vs Real Location (Original)')
    if len(actual_x) != 1:
        plt.plot(actual_x, actual_y, color='blue', linestyle='-', linewidth=2, label='Real Location')
    else: 
        plt.scatter(actual_x, actual_y, color='blue', marker='x', label='Real Location')
    plt.gca().set_aspect('equal', adjustable='box')  # Ensure equal aspect ratio
    plt.xlim(0, 2800)  # Set x-axis limits
    plt.ylim(2100, 0)  # Set y-axis limits in pygame-style
    plt.legend()

    plt.subplot(2, 2, 4)
    if result_type == 0:
        plt.scatter(smallest_error[2], smallest_error[3], color='green', marker='x', label="Estimated Location")
    else:
        plt.scatter(smallest_error[5], smallest_error[6], color='green', marker='x', label="Estimated Location")
    plt.xlabel('X (cm)')
    plt.ylabel('Y (cm)')
    plt.title('Estimated vs Real Location (Recalculated)')
    #plt.title('Estimated vs Real Location')
    if len(actual_x) != 1:
        plt.plot(actual_x, actual_y, color='blue', linestyle='-', linewidth=2, label='Real Location')
    else: 
        plt.scatter(actual_x, actual_y, color='blue', marker='x', label='Real Location')
    plt.gca().set_aspect('equal', adjustable='box')  # Ensure equal aspect ratio
    plt.xlim(0, 2800)  # Set x-axis limits
    plt.ylim(2100, 0)  # Set y-axis limits in pygame-style
    plt.legend()

    # Adjust the layout to avoid overlap
    plt.tight_layout()
    plt.show()


if __name__ == '__main__':
    main()
    