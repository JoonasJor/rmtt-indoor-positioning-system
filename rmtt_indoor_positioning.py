import math
from threading import Thread
from time import sleep
import pygame
import helpers as hp
import knn as k
import socket_helper as sh
from collections import deque
import socket
import py_window
import json
import rmtt_accuracy_test as test
from easy_trilateration.model import *  
from easy_trilateration.least_squares import easy_least_squares  
from easy_trilateration.graph import * 
import statistics 
import traceback

data_dict = [] # All ble advertisement data

beacon_current_rssis = [-92.25, -78.75, -83.6, -90.75] # dBm
rolling_rssis = []
beacon_addresses = [] # Mac address
beacon_locations = [] # x, y

# Estimated locations from fingerprinting, first element being the closest match
fp_estimated_locations = []
current_tof = 0 # Time of flight sensor value

sock = sh.initialize_socket()
fingerprint_locations = {}

# Pygame
font, screen, background_image, clock, target_fps = py_window.initialize_pygame()
beacon_to_move = 1

# Settings
do_test = False # accuracy testing
mode = 1 # 0 = fingerprint, 1 = trilateration
number_of_beacons_fp, number_of_beacons_tri = 0, 0

# Trilateration
beacon_distances = {0: 100, 1: 100, 2: 100, 3: 100}
individual_rssi_offsets = [0, 0, 0, 0]
trilateration_result = None
tri_test_rssis = []

# Drone
SPEED = 32
for_back_velocity = 0
left_right_velocity = 0
up_down_velocity = 0
yaw_velocity = 0
send_rc = False
drone_x = 500
drone_y = 500

stop_program = False

# Handle user input
def keydown(key):
    global beacon_to_move, for_back_velocity, left_right_velocity, up_down_velocity, yaw_velocity, send_rc, mode, rolling_rssis, do_test, stop_program

    if key == pygame.K_0:
        beacon_to_move = 0
    elif key == pygame.K_1:
        beacon_to_move = 1
    elif key == pygame.K_2:
        beacon_to_move = 2
    elif key == pygame.K_3:
        beacon_to_move = 3
    elif key == pygame.K_4:
        beacon_to_move = 4
    elif key == pygame.K_p:
        hp.save_beacon_locations(beacon_locations)
    elif key == pygame.K_UP:
        for_back_velocity = SPEED
    elif key == pygame.K_DOWN:
        for_back_velocity = -SPEED
    elif key == pygame.K_LEFT:
        left_right_velocity = -SPEED
    elif key == pygame.K_RIGHT:
        left_right_velocity = SPEED
    elif key == pygame.K_w:
        up_down_velocity = SPEED
    elif key == pygame.K_s:
        up_down_velocity = -SPEED
    elif key == pygame.K_a:
        yaw_velocity = -60
    elif key == pygame.K_d:
        yaw_velocity = 60
    elif key == pygame.K_r:
        send_rc = not send_rc
        sleep(1)
    elif key == pygame.K_t:
        sh.send_command("takeoff", True)
    elif key == pygame.K_l:
        sh.send_command("land", True)
    elif key == pygame.K_n:
        test.next_test_location()
    elif key == pygame.K_b:
        do_test = not do_test
    elif key == pygame.K_ESCAPE:
        stop_program = True
    elif key == pygame.K_f:
        # LENNOSTA VAIHTO EI TOIMI
        if mode == 0:
            mode = 1
            rolling_rssis = [deque([0] * number_of_beacons_fp, maxlen=5) for _ in range(number_of_beacons_fp)]
        elif mode == 1:
            mode = 0
            rolling_rssis = [deque([0] * number_of_beacons_tri, maxlen=5) for _ in range(number_of_beacons_tri)]

def keyup(key):
    global for_back_velocity, left_right_velocity, up_down_velocity, yaw_velocity

    if key == pygame.K_UP or key == pygame.K_DOWN:
        for_back_velocity = 0
    elif key == pygame.K_LEFT or key == pygame.K_RIGHT:
        left_right_velocity = 0
    elif key == pygame.K_w or key == pygame.K_s:
        up_down_velocity = 0
    elif key == pygame.K_a or key == pygame.K_d:
        yaw_velocity = 0

def set_current_rssis(dict):
    global beacon_current_rssis, rolling_rssis

    try:
        address = dict["Address"]
        rssi = int(dict["RSSI"])

        for i, beacon_address in enumerate(beacon_addresses):
            if address == beacon_address:
                print(address)
                rssi += individual_rssi_offsets[i]
                rolling_rssis[i].append(rssi)
                beacon_current_rssis[i] = sum(rolling_rssis[i]) / len(rolling_rssis[i])
        print(beacon_current_rssis)

    except Exception as e:
        print(f"set_current_rssis(): {e}")

def calculate_weight(difference, screen_size):
    power = 0.55
    weight = hp.clamp_value((difference / screen_size) ** power, max_value=1)
    return weight

def weighted_midpoint_coordinates(x_A, y_A, x_B, y_B):
    abs_distances_sorted = sorted([abs(distance) for distance in beacon_distances.values()])
    lowest_distance = abs_distances_sorted[0]
    second_lowest_distance = abs_distances_sorted[1]
    
    difference = second_lowest_distance - lowest_distance
    screen_size = screen.get_size()
    weight_x = calculate_weight(difference, screen_size[0])
    weight_y = calculate_weight(difference, screen_size[1])

    midpoint_x =hp.clamp_value(x_A + (x_B - x_A) * weight_x, max_value=screen_size[0])
    midpoint_y = hp.clamp_value(y_A + (y_B - y_A) * weight_y, max_value=screen_size[1])

    return midpoint_x, midpoint_y

def get_point_between_triresult_closest_beacon(beacon_distances: dict, trilateration_result: Circle):
    closest_beacon_key = min(beacon_distances, key=beacon_distances.get)
    closest_beacon_location = beacon_locations[closest_beacon_key]
    
    mid_point = weighted_midpoint_coordinates(
                trilateration_result.center.x, trilateration_result.center.y, 
                closest_beacon_location[0], closest_beacon_location[1])
    return mid_point

def write_data_to_file(data_list: list):
    data_list.append(beacon_locations)
    with open('test_rssis_sisapiha.txt', 'w') as file:
        for data in data_list:
            file.write(f"{data}\n")

def get_estimated_drone_location(knn = None, data = None):
    global fp_estimated_locations, beacon_distances, trilateration_result, drone_x, drone_y, tri_test_rssis
    # keep last x locations in deque
    rolling_location = deque([fp_estimated_locations], maxlen=1)

    time = 0

    while True:
        # Fingerprinting with k nearest algorithm
        if mode == 0:  
            try:                     
                filtered_avg_rssis = get_filtered_average_rssis()
                filtered_avg_rssis = [-92.25,  -78.75, -83.6, -90.75] # TESTI

                estimated_location_indexes = k.estimate_location(knn, data, [filtered_avg_rssis])
                estimated_location_indexes = [int(x) for x in estimated_location_indexes]
                estimated_locations = [fingerprint_locations[index] for index in estimated_location_indexes]

                rolling_location.append(estimated_locations)
                fp_estimated_locations = get_location_avgs(rolling_location)

                if do_test:
                    test.accuracy_test(estimated_locations, fingerprint_locations)
                    print(estimated_locations, fingerprint_locations)
                    sleep(0.2)
            except Exception as e:
                print(f"get_estimated_drone_location(knn, data) mode 0: {e}")
        # Trilateration
        elif mode == 1:
            try:
                filtered_avg_rssis = get_filtered_average_rssis() #EI KÄYTÖSSÄ
                circles = []
                distance_multiplier = 0.325
                rssi_offset = 43
                power = 2.8
                scale_to_screen_multiplier = 0.5 # 1 pixel is 2cm

                for key in beacon_distances:           
                    rssi_abs = abs(filtered_avg_rssis[key])
                    distance_from_beacon = hp.clamp_value((rssi_abs - rssi_offset), min_value=2) * distance_multiplier
                    distance_from_beacon = hp.clamp_value(distance_from_beacon ** power * scale_to_screen_multiplier, max_value=2400)
                    beacon_distances[key] = distance_from_beacon

                    circle = Circle(beacon_locations[key][0], beacon_locations[key][1], beacon_distances[key])
                    circles.append(circle)

                trilateration_result, meta = easy_least_squares(circles)
                drone_x, drone_y = get_point_between_triresult_closest_beacon(beacon_distances, trilateration_result)
                print(beacon_distances)

                #TESTI
                if do_test:
                    test_beacon = 2
                    tri_x = trilateration_result.center.x
                    tri_y = trilateration_result.center.y
                    #beacon_distances_scaled = [beacon_distance * 2 for beacon_distance in beacon_distances.values()]
                    tri_test_rssis.append(f"{filtered_avg_rssis} {beacon_distances} {time} {[drone_x, drone_y]} {[tri_x, tri_y]} {rolling_rssis} {beacon_current_rssis} {SPEED}")
                    time += 0.2
                #TESTI

                sleep(0.2)
            except Exception as e:
                print(f"get_estimated_drone_location(knn, data) mode 1: {e}")
                traceback.print_exc()

def get_filtered_average_rssis():
    filtered_average_rssis = []
    try:
        for rssis in rolling_rssis:
            # Remove lowest and highest values and take average of remaining 3 values
            sorted_rssis = sorted(rssis)[1:-1] 
            avg_rssi = sum(sorted_rssis) / len(sorted_rssis)
            filtered_average_rssis.append(avg_rssi)    
    except Exception as e:
        print(f"get_filtered_average_rssis(): {e}")

    return filtered_average_rssis

def get_location_avgs(rolling_location):
    drone_location_avg = []
    for locations in zip(*rolling_location):
        avg_values = tuple(sum(dim_values) // len(dim_values) for dim_values in zip(*locations))
        drone_location_avg.append(avg_values)
    return drone_location_avg

# Receive drone responses
def recv(sock: socket.socket):
    global current_tof

    while True:
        temp_data = " "
        while temp_data[-1] != "\n":
            try:
                data, address = sock.recvfrom(1024)
                temp_data += data.decode('utf-8')
                if temp_data.strip() in ["ok", "error"]: 
                    break 
            except socket.timeout:
                print("Timeout occurred")
                break
        data = temp_data[1:]
        if data[0:3] == "ble":
            dict = hp.str_to_dict(data[3:])
            if dict != None: 
                data_dict.append(dict)
                set_current_rssis(dict)  
        elif data[0:3] == "tof":
            current_tof = data[3:].strip("tof").strip()
        elif data.strip() == "user button":
            #test.next_test_location()
            #print(sum(tri_test_rssis) / len(tri_test_rssis))
            sleep(5)
        elif data:
            print(f"data received: {data}")

def draw_beacons():  
    for i, beacon_location in enumerate(beacon_locations):     
        pygame.draw.circle(screen, (200, 0, 0), beacon_location, 10)
        text = font.render(f"{i}: {beacon_current_rssis[i]}", True, (0, 0, 0))
        screen.blit(text, beacon_location)

        # Draw beacon range
        if i in beacon_distances and mode == 1:
                pygame.draw.circle(screen, (200, 0, 0), beacon_location, beacon_distances[i], width=1)

    if not trilateration_result:
        return   
    
    circle_radius = hp.clamp_value(abs(trilateration_result.radius), min_value=10, max_value=20)
    pygame.draw.circle(screen, (200, 100, 0), (drone_x, drone_y), circle_radius)
    pygame.draw.circle(screen, (200, 0, 0), (trilateration_result.center.x, trilateration_result.center.y), 5)
    

def draw_fp_locations(fp_estimated_locations):
    size = 40
    for location in fp_estimated_locations:
        pygame.draw.circle(screen, (0, 0, 200), location, size)
        size *= 0.8

def move_beacon_to_mouse(beacon_locations, mouse_pos):
    beacon_locations[beacon_to_move] = mouse_pos

def get_fingerprint_locations_from_file():
    with open("fingerprint_locations.txt", "r") as file:
        fp_locations_str = json.load(file)
        fp_locations = {int(key): value for key, value in fp_locations_str.items()}
    print(f"Retrieved fingerprint locations from file: fingerprint_locations.txt")
    print(fp_locations)
    return fp_locations

def update():
    if send_rc:
        sh.send_command(f"rc {left_right_velocity} {for_back_velocity} {up_down_velocity} {yaw_velocity}")
    #self.tello.send_command_with_return("EXT tof?")

def main():
    global beacon_addresses, rolling_rssis, beacon_locations, fingerprint_locations, number_of_beacons_fp, number_of_beacons_tri

    fingerprint_locations = get_fingerprint_locations_from_file()
    beacon_addresses = hp.get_beacon_addresses()
    beacon_locations = hp.get_beacon_locations()

    if mode == 0:
        knn, formatted_data = k.fit_data()
        drone_location_thread = Thread(target=get_estimated_drone_location, args=(knn, formatted_data))
        number_of_beacons_fp = len(formatted_data[0][1]) 
    else:
        drone_location_thread = Thread(target=get_estimated_drone_location)
    drone_location_thread.daemon = True
    drone_location_thread.start()
       
    number_of_beacons_tri = len(beacon_addresses)

    if mode == 0:       
        rolling_rssis = [deque([0] * number_of_beacons_fp, maxlen=4) for _ in range(number_of_beacons_fp)]
    elif mode == 1:
        rolling_rssis = [deque([0] * number_of_beacons_tri, maxlen=4) for _ in range(number_of_beacons_tri)]   

    response_receiver_thread = Thread(target=recv, args=(sock,))
    response_receiver_thread.daemon = True
    response_receiver_thread.start()   

    keep_drone_alive_thread = Thread(target=sh.keep_drone_alive)
    keep_drone_alive_thread.daemon = True
    keep_drone_alive_thread.start()

    sleep(1)
    sh.send_command("battery?", True) 
    
    while not stop_program:
        try:
            events = pygame.event.get()
            for event in events:
                if event.type == pygame.MOUSEBUTTONDOWN:
                    pos = pygame.mouse.get_pos()
                    move_beacon_to_mouse(beacon_locations, pos)
                elif event.type == pygame.KEYUP:
                    keyup(event.key)
                elif event.type == pygame.KEYDOWN:
                    keydown(event.key)
            update()
            screen.blit(background_image, (0,0))
            draw_beacons()
            if mode == 0: 
                draw_fp_locations(fp_estimated_locations)
            pygame.display.update()
            clock.tick(target_fps)
        except KeyboardInterrupt:
            #sock.shutdown(1)
            #sock.close()
            break

    write_data_to_file(tri_test_rssis)

if __name__ == "__main__":
    main()
