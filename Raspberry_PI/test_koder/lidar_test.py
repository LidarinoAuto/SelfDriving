# -*- coding: utf-8 -*-

# Filename: lidar_test.py

# Simple script to test RPLidar connectivity and scanning
 
from rplidar import RPLidar

import time

import sys # Import sys for exit
 
PORT_NAME = "/dev/rplidar" # Adjust if your port is different

BAUD_RATE = 115200 # Standard baud rate
 
def test_lidar():

    print(f"Attempting to connect to Lidar at {PORT_NAME} with baud rate {BAUD_RATE}...")

    lidar = None # Initialize lidar variable
 
    try:

        # Attempt connection

        lidar = RPLidar(PORT_NAME, baudrate=BAUD_RATE)

        print("Lidar connected successfully.")
 
        # --- Optional: Check Lidar Health and Info ---

        # These calls often trigger the "too many values to unpack" error if there's a basic communication problem

        try:

            print("Checking Lidar health...")

            health = lidar.get_health()

            print(f"Lidar Health: Status={health[0]}, Error Code={health[1]}")

            # print("Lidar Info:", lidar.get_info()) # Uncomment if you want to see info
 
        except Exception as e:

            print(f"Error getting Lidar health or info: {e}")

            # Decide if you want to exit here or try scanning anyway

            # For debugging 'too many values to unpack', this is a key test point.

            # Let's try scanning even if health/info fails, as the error might happen later.
 
 
        # --- Test Scanning ---

        print("Starting scan test (will read a few scans)...")

        count = 0

        scans_to_read = 10 # Try to read 10 scans

        start_time = time.time()
 
        # This loop is where the 'too many values to unpack' error often occurs

        for i, scan in enumerate(lidar.iter_scans()):

            if i >= scans_to_read:

                print(f"Successfully read {scans_to_read} test scans. Stopping scan test.")

                break # Stop after reading the desired number of scans
 
            count += len(scan) # Count data points

            print(f"Scan {i+1} received with {len(scan)} data points.")

            # Optional: print some data points from the scan (uncomment if needed for debugging data format)

            # if len(scan) > 0:

            #    # Print first and last point angle and distance (convert distance to cm)

            #    print(f"  Scan {i+1} Example points: First=({scan[0][1]:.2f}deg, {scan[0][2]/10.0:.2f}cm), Last=({scan[-1][1]:.2f}deg, {scan[-1][2]/10.0:.2f}cm)")
 
 
        end_time = time.time()

        print(f"Total data points read in test scans: {count}")

        if (i + 1) > 0 and (end_time - start_time) > 0:

             print(f"Scan rate approx: {i+1} scans / {end_time - start_time:.2f} seconds")
 
    except Exception as e:

        # Catch any exception during connection or scanning

        print(f"An error occurred during Lidar test: {e}")

        # This will print the 'too many values to unpack' error if it happens here
 
    finally:

        # --- Clean up ---

        if lidar:

            print("Stopping Lidar motor and disconnecting...")

            try:

                lidar.stop() # Stop scanning motor

                lidar.disconnect() # Close serial port connection

                print("Lidar disconnected.")

            except Exception as e:

                 print(f"Error during Lidar cleanup: {e}")

        print("Lidar test finished.")
 
 
# --- Program Entry Point ---

if __name__ == "__main__":

    test_lidar()

    sys.exit(0) # Exit cleanly
 