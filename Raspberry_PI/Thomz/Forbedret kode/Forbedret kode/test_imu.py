import imu_module
import time
from logging_utils import skriv_logg # S�rg for at logging_utils er tilgjengelig

if imu_module.init_imu():
    skriv_logg("IMU is ready for testing.")
    try:
        while True:
            imu_data = imu_module.read_imu_data()
            # skriv_logg(f"IMU Raw Z Vel (Test): {imu_module.read_raw_gyro_z()}") # Valgfri: se r�data
            # skriv_logg(f"IMU Processed Z Vel (Test): {imu_data:.2f}") # Debug loggen i funksjonen tar seg av dette n�
            time.sleep(0.05) # Les ca 20 ganger i sekundet
    except KeyboardInterrupt:
        skriv_logg("IMU test stopped by user.")
    except Exception as e:
        skriv_logg(f"Error during IMU test: {e}")
else:
    skriv_logg("IMU initialization failed.")