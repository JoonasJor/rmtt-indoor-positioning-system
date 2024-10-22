from threading import Thread
from time import sleep
import pygame
import socket_helper as sh
import helpers as hp
import traceback
import json
import os
import socket
import py_window

FOLDER_PATH = ".\calibration-data"
FP_FILENAME = "fingerprint_locations.txt"

sock = sh.initialize_socket()
font, screen, background_image, clock, target_fps = py_window.initialize_pygame()
beacon_addresses = hp.get_beacon_addresses()
print(beacon_addresses)
beacon_locations = hp.get_beacon_locations()

beacon_rssis = [] # [(beaconaddress1, [rssi1, rssi2...]), (beaconaddress2, [rssi1, rssi2...])...]
beacon_average_rssis = {} # [(beaconaddress1, avg_rssi1), (beaconaddress2, avg_rssi2)...]
calibration_length = 30
calibration_done = True
display_fp_values = True
current_location = 26 # location currently being calibrated
fingerprint_start = (300, 300)
fingerprint_locations_dict = {} # all calibrated locations
step_size = 200 # in cm/pixels
columns = 5
rows = 4
counter = 0
beacon_to_display = 1

# drone
SPEED = 100
for_back_velocity = 0
left_right_velocity = 0
up_down_velocity = 0
yaw_velocity = 0
send_rc = False

stop_program = False

def fit_fingerprints_into_map(start = None):
    global fingerprint_start, fingerprint_locations_dict

    fingerprint_locations_dict = {}
    if start: 
        fingerprint_start = start

    for column in range(columns):
        for row in range(rows):
            fingerprint_locations_dict[rows * column + row] = (
                fingerprint_start[0] + row * step_size, 
                fingerprint_start[1] + column * step_size
            )

def save_fingerprint_locations_to_file():
    with open(FP_FILENAME, "w") as file:
        json.dump(fingerprint_locations_dict, file)
    print(f"Saved fingerprint locations to file: {FP_FILENAME}")
    print(fingerprint_locations_dict)

def get_fingerprint_locations_from_file():
    with open(FP_FILENAME, "r") as file:
        fp_locations_str = json.load(file)
        fp_locations = {int(key): value for key, value in fp_locations_str.items()}
    print(f"Retrieved fingerprint locations from file: {FP_FILENAME}")
    print(fp_locations)
    return fp_locations


def calibrate_fingerprint(dict):
    global calibration_done, beacon_rssis
    try:
        for i, beacon in enumerate(beacon_rssis):
            if dict["Address"] == beacon[0]:
                rssi = int(dict["RSSI"].strip())
                beacon_rssis[i][1].append(rssi)  
    except:
        print(f"calibrate_fingerprint() error:")
        traceback.print_exc()

    if(counter > calibration_length):
        save_beacon_avg_rssis_to_file()
        print(f"Location {current_location} calibration done")
        calibration_done = True

def save_beacon_avg_rssis_to_file(): 
    global beacon_average_rssis

    avg_rssis = []
    for single_beacon_rssis in beacon_rssis:
        if single_beacon_rssis[1]:
            average = sum(single_beacon_rssis[1]) / len(single_beacon_rssis[1])
            avg_rssis.append([single_beacon_rssis[0], average])
        else:
            print(f"beacon {single_beacon_rssis[0]} not found")
    beacon_average_rssis[str(current_location)] = avg_rssis

    file_name = f"{current_location}.json"
    file_path = os.path.join(FOLDER_PATH, file_name)

    json_data = json.dumps(beacon_average_rssis[str(current_location)])
    with open(file_path, "w") as file:
        file.write(json_data)

def next_location():
    global current_location, calibration_done, beacon_rssis, counter
  
    current_location += 1
    counter = 0
    if current_location  < len(fingerprint_locations_dict):       
        beacon_rssis = []
        for address in beacon_addresses:
            beacon_rssis.append((address, []))
        calibration_done = False
        print(f"started calibration on location {current_location}")
    
def get_data_from_file():
    data = {}

    for file_name in os.listdir(FOLDER_PATH):
        if file_name.endswith(".json"):
            file_path = os.path.join(FOLDER_PATH, file_name)
            location_name = file_name[:-5]  # Extract the location name from the file name

            with open(file_path, "r") as file:
                json_data = file.read()
                data[location_name] = json.loads(json_data)  # Convert JSON data to dictionary
    return data

def draw_fingerprint_locations():
    pygame.draw.circle(screen, (200, 0, 0), beacon_locations[beacon_to_display], 20)
    for fp in fingerprint_locations_dict:
        if fp == current_location: color = (0,0,200)
        else: color = (200,0,0)

        location = fingerprint_locations_dict[fp]
        pygame.draw.circle(screen, color, location, 10)
        if fp < len(beacon_average_rssis):
            for beacon in beacon_average_rssis[str(fp)]:
                if beacon[0] == beacon_addresses[beacon_to_display]:
                    average_rssi = round(beacon[1], 1)
                    text = font.render(f"{str(fp)}: {average_rssi}", True, (0, 0, 0))
                else:
                    text = font.render(str(fp), True, (0, 0, 0))
        else:
            text = font.render(str(fp), True, (0, 0, 0))
        screen.blit(text, location)
    info_texts = []  
    text_x = 50
    text_y = 50
    info_texts.append(font.render(str(step_size), True, (0, 0, 0)))
    info_texts.append(font.render(str(calibration_done), True, (0, 0, 0)))
    for text in info_texts:
        screen.blit(text, (text_x, text_y))
        text_y += 20

def recv():
    global counter

    while True:
        temp_data = " "
        while temp_data[-1] != "\n":
            try:
                data, address = sock.recvfrom(1024)
                temp_data += data.decode('utf-8')
                if(temp_data.strip() == "ok"): 
                    break 
            except socket.timeout:
                print("Timeout occurred")
                break
        data = temp_data[1:]

        # ble announcements
        if data[0:3] == "ble":
            dict = hp.str_to_dict(data[3:])
            if(dict == None):
                continue 
            if not calibration_done:
                calibrate_fingerprint(dict)
                if dict["Address"] in beacon_addresses:
                    counter += 1
        # when user button pressed
        elif data.strip() == "user button" and calibration_done:
            next_location()
        if data:
            pass
           #print(f"data received: {data}")

# handle user input
def keydown(key):
    global rows, columns, step_size, fingerprint_locations_dict, for_back_velocity, left_right_velocity, up_down_velocity, yaw_velocity, send_rc, beacon_to_display, stop_program

    mods = pygame.key.get_mods()

    if key == pygame.K_KP_MINUS or key == pygame.K_MINUS:
        if mods & pygame.KMOD_SHIFT:
            columns -= 1
        elif mods & pygame.KMOD_CTRL:
            rows -= 1
        else:
            step_size -= 10
        fit_fingerprints_into_map()
    elif key == pygame.K_KP_PLUS or key == pygame.K_PLUS:
        if mods & pygame.KMOD_SHIFT:
            columns += 1
        elif mods & pygame.KMOD_CTRL:
            rows += 1
        else:
            step_size += 10
        fit_fingerprints_into_map()
    elif key == pygame.K_w:
        if mods & pygame.KMOD_CTRL:
            save_fingerprint_locations_to_file()
        else:
            up_down_velocity = SPEED
    elif key == pygame.K_r:
        if mods & pygame.KMOD_CTRL:
            fingerprint_locations_dict = get_fingerprint_locations_from_file() 
        else:
            send_rc = not send_rc
            sleep(1)
    elif key == pygame.K_n:
        next_location()
    elif key == pygame.K_UP:  # set forward velocity
        for_back_velocity = SPEED
    elif key == pygame.K_DOWN:  # set backward velocity
        for_back_velocity = -SPEED
    elif key == pygame.K_LEFT:  # set left velocity
        left_right_velocity = -SPEED
    elif key == pygame.K_RIGHT:  # set right velocity
        left_right_velocity = SPEED
    elif key == pygame.K_s:  # set down velocity
        up_down_velocity = -SPEED
    elif key == pygame.K_a:  # set yaw counter clockwise velocity
        yaw_velocity = -SPEED
    elif key == pygame.K_d:  # set yaw clockwise velocity
        yaw_velocity = SPEED
    elif key == pygame.K_t:
        sh.send_command("takeoff", True)
    elif key == pygame.K_l:
        sh.send_command("land", True)
    elif key == pygame.K_1:
        beacon_to_display = 1
    elif key == pygame.K_2:
        beacon_to_display = 2
    elif key == pygame.K_3:
        beacon_to_display = 3
    elif key == pygame.K_4:
        beacon_to_display = 4
    elif key == pygame.K_ESCAPE:
        stop_program = True

def keyup(key):
    global for_back_velocity, left_right_velocity, up_down_velocity, yaw_velocity

    if key == pygame.K_UP or key == pygame.K_DOWN:  # set zero forward/backward velocity
        for_back_velocity = 0
    elif key == pygame.K_LEFT or key == pygame.K_RIGHT:  # set zero left/right velocity
        left_right_velocity = 0
    elif key == pygame.K_w or key == pygame.K_s:  # set zero up/down velocity
        up_down_velocity = 0
    elif key == pygame.K_a or key == pygame.K_d:  # set zero yaw velocity
        yaw_velocity = 0

def update():
    if send_rc:
        sh.send_command(f"rc {left_right_velocity} {for_back_velocity} {up_down_velocity} {yaw_velocity}")

def main():
    global fingerprint_locations_dict, beacon_average_rssis

    beacon_average_rssis = get_data_from_file()
    print(beacon_average_rssis)
    for address in beacon_addresses:
        beacon_rssis.append((address, []))
    print(beacon_rssis)

    response_receiver_thread = Thread(target=recv)
    response_receiver_thread.daemon = True
    response_receiver_thread.start()

    response_receiver_thread = Thread(target=sh.keep_drone_alive)
    response_receiver_thread.daemon = True
    response_receiver_thread.start()

    fingerprint_locations_dict = get_fingerprint_locations_from_file() 

    sh.send_command("battery?", True)
    sleep(1)

    while not stop_program:
        try:
            if current_location >= len(fingerprint_locations_dict):
                print("all locations calibrated")
                break
            events = pygame.event.get()
            for event in events:
                if event.type == pygame.MOUSEBUTTONDOWN:
                    pos = pygame.mouse.get_pos()
                    fit_fingerprints_into_map(pos)
                elif event.type == pygame.KEYUP:
                    keyup(event.key)
                elif event.type == pygame.KEYDOWN:
                    keydown(event.key)
            update()
            screen.blit(background_image, (0,0))
            draw_fingerprint_locations()
            pygame.display.update()
            clock.tick(target_fps)
        except KeyboardInterrupt:
            break

if __name__ == "__main__":
    main()
    