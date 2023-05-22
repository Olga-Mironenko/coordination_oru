import os
import pandas as pd
import csv

# Define the root directory
root_directory = './results/lookAheadPaper_2023/'

# Walk through all subdirectories in the root directory
for directory, subdirectories, files in os.walk(root_directory):
    # Initialize a DataFrame to store all results
    all_results = pd.DataFrame()

    # Loop over all files
    for file in files:
        # Skip the results.csv file
        if file == 'results.csv':
            continue

        # Create the full file paths
        input_file = os.path.join(directory, file)

        # Ensure the file is a CSV before processing
        if not input_file.endswith('.csv'):
            continue

        # Remove existing result file if exists
        output_file = os.path.join(directory, 'results.csv')
        if os.path.exists(output_file):
            os.remove(output_file)

        # Read in the data
        df = pd.read_csv(input_file, delimiter=';', quoting=csv.QUOTE_NONE)

        # Get the file ID (without extension) from the file name
        # Remove 'Robot_' from the file_id
        file_id = os.path.splitext(file)[0].replace('Robot_', '')

        # Initialize a counter for the cycles
        cycles = 0

        # Initialize a variable to keep track of the last row's 'DistanceTraveled' value
        last_distance = df['DistanceTraveled'][0]

        # Iterate through 'DistanceTraveled' column in the DataFrame
        for distance in df['DistanceTraveled'][1:]:
            # If the last distance was > 0 and the current distance is 0, increment the cycle count
            if last_distance > 0 and distance == 0:
                cycles += 1

            # Update the last distance
            last_distance = distance

        # Append the result to the all_results DataFrame, creating a new column with a name based on the file_id
        all_results[file_id + '_cycles'] = [cycles]

    # Skip if the DataFrame is empty
    if all_results.empty:
        continue

    # Sort the DataFrame columns in ascending order
    all_results = all_results.sort_index(axis=1)

    # Save the result to a CSV file in the same subdirectory
    results_file = os.path.join(directory, 'results.csv')

    # If a results file already exists, remove it before saving the new one
    if os.path.exists(results_file):
        os.remove(results_file)

    all_results.to_csv(results_file, index=False, sep=';')
