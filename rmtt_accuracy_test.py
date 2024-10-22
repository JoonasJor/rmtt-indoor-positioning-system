import math
import pandas as pd
from time import sleep

FILENAME = "accuracy_test_results.xlsx"
TEXT_FILENAME = "accuracy_test_results.txt"  # New text file name
location_done = True
test_real_location = 1
test_results = []
df = pd.DataFrame()
data_points = 150

# Open the text file in write mode at the beginning
text_file = open(TEXT_FILENAME, "w")

def accuracy_test(estimated_locations, fingerprint_locations):
    global test_results, location_done

    if test_real_location >= len(fingerprint_locations):
        print("Test finished")
        sleep(0.5)
        return 
    
    if location_done:
        print("Waiting for user button")
        sleep(0.5)
        return 

    difference_cm = calculate_difference(estimated_locations[0], fingerprint_locations[test_real_location])
    test_results.append(difference_cm)
    print(test_results)
    
    # Append the result to the text file
    text_file.write(f"{test_real_location}: {difference_cm}\n")
    
    if len(test_results) >= data_points:
        location_done = True
        save_test_results_to_file()
    
def calculate_difference(estimated_location_xy, real_location_xy):
    x1, y1 = real_location_xy[0], real_location_xy[1]
    x2, y2 = estimated_location_xy[0], estimated_location_xy[1]
    difference = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
    return difference

def save_test_results_to_file():   
    df = pd.read_excel(FILENAME)
    df[test_real_location] = test_results   
    df.to_excel(FILENAME, index=False)    
    print(f"Saved results to file: {FILENAME}")

    # Close the text file after saving
    text_file.close()
    print(f"Saved results to text file: {TEXT_FILENAME}")

def next_test_location():
    global test_real_location, location_done, test_results

    test_real_location += 1
    location_done = False
    test_results = []
