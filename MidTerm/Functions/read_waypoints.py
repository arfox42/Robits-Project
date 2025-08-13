import os
import numpy as np
from AriaPy import *

class read_waypoints:
    def __init__(self):
        pass

    @staticmethod
    def read_waypoints_from_file(file_name):
        """
        Reads waypoints from a text file and returns a NumPy array of coordinates.
        Each line in the file should contain two comma-separated floats: X, Y.
        """
        waypoints_list = []  # Use a temporary list to store the coordinates
        try:
            if not os.path.exists(file_name):
                print("Error: Waypoints file not found at %s" % file_name)
                return np.array([])

            with open(file_name, 'r') as f:
                for line in f:
                    line = line.strip()
                    if line and not line.startswith('#'):
                        try:
                            x_str, y_str = line.split(',')
                            x = float(x_str.strip())
                            y = float(y_str.strip())
                            # Append the coordinates as a list to the temporary list
                            waypoints_list.append([x, y])
                        except ValueError as e:
                            print("Error parsing line: %s. Details: %s" % (line.strip(), e))
        except IOError as e:
            print("Error reading file: %s. Details: %s" % (file_name, e))

        # Convert the list of coordinates into a NumPy array and return it
        return np.array(waypoints_list)