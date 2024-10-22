from sklearn.neighbors import NearestNeighbors
import os
import json

folder_path = ".\calibration-data-4beacons-sisäpiha"

def get_data_from_file():
    data = {}

    for file_name in os.listdir(folder_path):
        if file_name.endswith(".json"):  # Only consider JSON files
            file_path = os.path.join(folder_path, file_name)
            location_name = file_name[:-5]  # Extract the location name from the file name

            with open(file_path, "r") as file:
                json_data = file.read()
                data[location_name] = json.loads(json_data)  # Convert JSON data to dictionary
    return data

def fit_data():
    data = get_data_from_file() # Get dict from json file
    # Convert to list, remove addresses and join rssi values
    formatted_data = []
    for key, inner_list in data.items():
        values = [value for _, value in inner_list]
        formatted_data.append((key, values))    

    knn = NearestNeighbors(n_neighbors=len(formatted_data))
    knn.fit([fp[1] for fp in formatted_data])

    return knn, formatted_data

def estimate_location(knn: NearestNeighbors, formatted_data, current_rssi):
    distances, indices = knn.kneighbors(current_rssi)
    matching_locations = [formatted_data[index][0] for index in indices[0]]

    return matching_locations