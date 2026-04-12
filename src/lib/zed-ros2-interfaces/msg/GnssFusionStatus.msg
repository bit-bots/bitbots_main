# GNSS_FUSION_STATUS
# Represents the current state of GNSS fusion for global localization.
# OK - The GNSS fusion module is calibrated and working successfully.
# OFF - The GNSS fusion module is not enabled.
# CALIBRATION_IN_PROGRESS - Calibration of the GNSS/VIO fusion module is in progress.
# RECALIBRATION_IN_PROGRESS- Re-alignment of GNSS/VIO data is in progress, leading to potentially inaccurate global position.
uint8 OK = 0
uint8 OFF = 1
uint8 CALIBRATION_IN_PROGRESS = 2
uint8 RECALIBRATION_IN_PROGRESS = 3

uint8 gnss_fusion_status