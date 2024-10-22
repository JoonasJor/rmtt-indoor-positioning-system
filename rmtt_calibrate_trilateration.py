calibration_length = 25 
calibration_done = True
current_distance = 0

average_rssis = []

def recv():
    global counter

    while True:
        temp_data = " "
        while temp_data[-1] != "\n":
            try:
                data, address = sock.recvfrom(1024)
                #print(data)
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
                counter += 1
        # when user button pressed
        elif data.strip() == "user button" and calibration_done:
            next_location()
        if data:
           print(f"data received: {data}")