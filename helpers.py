import sys

beacon_locations_filename = "beacon_locations.txt"
beacon_addresses_filename = "beacon_addresses.txt"

def str_to_dict(data):
    string = str(data)
    pairs = string.split(', ')
    
    try:
        return {key: value for key, value in (pair.split(': ') for pair in pairs)}
    except Exception as e:
        print(f"str_to_dict() error: {e} \n string: {string}")  
        return None 
    
def clamp_value(value, min_value = None, max_value = None):
    if min_value is not None and value < min_value:
        value = min_value
    elif max_value is not None and value > max_value:
        value = max_value
    return value

# get beacon mac addresses from text file
def get_beacon_addresses():
    addresses = []
    
    try:
        f = open(beacon_addresses_filename, "r")
        for line in f:
            line = line.strip()
            addresses.append(line)

        return addresses
    except Exception as e:
        print(f"get_beacon_addresses() error: {e}")
        return None
    
def get_beacon_locations(filename = None):
    if filename is None:
        filename = "beacon_locations.txt"
    locations = []
    
    try:
        f = open(filename, "r")
        for line in f:
            location = [int(x) for x in line.split(",")]
            locations.append(location)
        return locations
    except Exception as e:
        print(f"get_beacon_locations() error: {e}")
        return None

def save_beacon_locations(locations):
    lines = [f"{x[0]},{x[1]}\n" for x in locations]
    print(lines)

    try:
        f = open(beacon_locations_filename, "w")
        for line in lines:
            f.write(line)
        print(f"Beacon locations saved to file: {beacon_locations_filename}")
    except Exception as e:
        print(f"save_beacon_locations() error: {e}") 

def printf(text):
    sys.stdout.write(text)
    sys.stdout.flush()
