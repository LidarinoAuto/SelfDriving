# Filename: check_us_module.py
import ultrasound_module
import os
import sys

print("--- Ultrasound Module Diagnostic Test ---")
print(f"Running script: {__file__}")
print(f"Current working directory: {os.getcwd()}")

try:
    print(f"Attempting to import 'ultrasound_module'...")
    # Denne linjen pr�ver � importere modulen
    # import ultrasound_module # Denne importen skjer allerede �verst

    print("'ultrasound_module' imported successfully.")

    print(f"Path to loaded ultrasound_module: {ultrasound_module.__file__}")

    print(f"Checking for attribute 'check_all_ultrasound_sensors'...")
    if hasattr(ultrasound_module, 'check_all_ultrasound_sensors'):
        print("Result: 'ultrasound_module' module HAS the attribute 'check_all_ultrasound_sensors'.")
    else:
        print("Result: 'ultrasound_module' module DOES NOT have the attribute 'check_all_ultrasound_sensors'.")
        print("Please verify the content and location of your ultrasound_module.py file.")

except ImportError as e:
    print(f"Error: Could not import 'ultrasound_module'. Make sure ultrasound_module.py is in the same directory.")
    print(f"ImportError details: {e}")
except Exception as e:
    print(f"An unexpected error occurred during the test: {e}")
    print(f"Error details: {e}")

print("--- Test Complete ---")