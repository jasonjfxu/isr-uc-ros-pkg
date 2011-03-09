# Single gas sensor msg

Header header			# timestamp and frame_id

string sensor_model

# Gases enums
uint8 ORGANIC_SOLVENTS=0
uint8 AIR_CONTAMINANTS=1
uint8 GENERAL_COMBUSTIBLE=3
uint8 NATURAL_GAS_METHANE=4

uint8[] gas_type    	# the types of gases detected by the sensor

float32 reading			# ppm

float32 min_reading		# ppm
float32 max_reading		# ppm

float32 clean_air		# ppm

uint16 raw_data			# mV
