/*
 *  iRobot_OI.h
 *  
 *
 *  Created by Gonçalo Cabrita on 19/05/2010.
 *  Copyright 2010 ISR. All rights reserved.
 *
 *	Comments:
 *	Low level communication layer for the iRobot OI
 *
 */


typedef enum _OI_Opcode {

	// Command opcodes
	OI_OPCODE_START = 128,
	OI_OPCODE_BAUD = 129,
	OI_OPCODE_CONTROL = 130,
	OI_OPCODE_SAFE = 131,
	OI_OPCODE_FULL = 132,
	OI_OPCODE_POWER = 133,
	OI_OPCODE_SPOT = 134,
	OI_OPCODE_CLEAN = 135,
	OI_OPCODE_MAX = 136,
	OI_OPCODE_DRIVE = 137,
	OI_OPCODE_MOTORS = 138,
	OI_OPCODE_LEDS = 139,
	OI_OPCODE_SONG = 140,
	OI_OPCODE_PLAY = 141,
	OI_OPCODE_SENSORS = 142,
	OI_OPCODE_FORCE_DOCK = 143,
	OI_OPCODE_PWM_MOTORS = 144,
	OI_OPCODE_DRIVE_DIRECT = 145,
	OI_OPCODE_DRIVE_PWM = 146,
	OI_OPCODE_STREAM = 148,
	OI_OPCODE_QUERY = 149,
	OI_OPCODE_PAUSE_RESUME_STREAM = 150,
	OI_OPCODE_SCHEDULE_LEDS = 162,
	OI_OPCODE_DIGIT_LEDS_RAW = 163,
	OI_OPCODE_DIGIT_LEDS_ASSCI = 164,
	OI_OPCODE_BUTTONS = 165,
	OI_OPCODE_SCHEDULE = 167,
	OI_OPCODE_SET_DAY_TIME = 168

} OI_Opcode;


typedef enum _OI_Packet_ID {
	
	// Sensor Packets
	OI_PACKET_GROUP_0 = 0,			// PACKETS 7-26
	OI_PACKET_GROUP_1 = 1,			// PACKETS 7-16
	OI_PACKET_GROUP_2 = 2,			// PACKETS 17-20
	OI_PACKET_GROUP_3 = 3,			// PACKETS 21-26
	OI_PACKET_GROUP_4 = 4,			// PACKETS 27-34
	OI_PACKET_GROUP_5 = 5,			// PACKETS 35-42
	OI_PACKET_GROUP_6 = 6,			// PACKETS 7-42
	OI_PACKET_GROUP_100 = 100,		// PACKETS 7-58
	OI_PACKET_GROUP_101 = 101,		// PACKETS 43-58
	OI_PACKET_GROUP_106 = 106,		// PACKETS 46-51
	OI_PACKET_GROUP_107 = 107,		// PACKETS 54-58
	OI_PACKET_BUMPS_DROPS = 7,
	OI_PACKET_WALL = 8,
	OI_PACKET_CLIFF_LEFT = 9,
	OI_PACKET_CLIFF_FRONT_LEFT = 10,
	OI_PACKET_CLIFF_FRONT_RIGHT = 11,
	OI_PACKET_CLIFF_RIGHT = 12,
	OI_PACKET_VIRTUAL_WALL = 13,
	OI_PACKET_WHEEL_OVERCURRENTS = 14,
	OI_PACKET_DIRT_DETECT = 15,
	OI_PACKET_IR_CHAR_OMNI = 17,
	OI_PACKET_BUTTONS = 18,
	OI_PACKET_DISTANCE = 19,
	OI_PACKET_ANGLE = 20,
	OI_PACKET_CHARGING_STATE = 21,
	OI_PACKET_VOLTAGE = 22,
	OI_PACKET_CURRENT = 23,
	OI_PACKET_TEMPERATURE = 24,
	OI_PACKET_BATTERY_CHARGE = 25,
	OI_PACKET_BATTERY_CAPACITY = 26,
	OI_PACKET_WALL_SIGNAL = 27,
	OI_PACKET_CLIFF_LEFT_SIGNAL = 28,
	OI_PACKET_CLIFF_FRONT_LEFT_SIGNAL = 29,
	OI_PACKET_CLIFF_FRONT_RIGHT_SIGNAL = 30,
	OI_PACKET_CLIFF_RIGHT_SIGNAL = 31,
	OI_PACKET_CHARGE_SOURCES = 34,
	OI_PACKET_OI_MODE = 35,
	OI_PACKET_SONG_NUMBER = 36,
	OI_PACKET_SONG_PLAYING = 37,
	OI_PACKET_STREAM_PACKETS = 38,
	OI_PACKET_REQ_VELOCITY = 39,
	OI_PACKET_REQ_RADIUS = 40,
	OI_PACKET_REQ_RIGHT_VELOCITY = 41,
	OI_PACKET_REQ_LEFT_VELOCITY = 42,
	OI_PACKET_RIGHT_ENCODER = 43,
	OI_PACKET_LEFT_ENCODER = 44,
	OI_PACKET_LIGHT_BUMPER = 45,
	OI_PACKET_LIGHT_BUMPER_LEFT = 46,
	OI_PACKET_LIGHT_BUMPER_FRONT_LEFT = 47,
	OI_PACKET_LIGHT_BUMPER_CENTER_LEFT = 48,
	OI_PACKET_LIGHT_BUMPER_CENTER_RIGHT = 49,
	OI_PACKET_LIGHT_BUMPER_FRONT_RIGHT = 50,
	OI_PACKET_LIGHT_BUMPER_RIGHT = 51,
	OI_PACKET_IR_CHAR_LEFT = 52,
	OI_PACKET_IR_CHAR_RIGHT = 53,
	OI_PACKET_LEFT_MOTOR_CURRENT = 54,
	OI_PACKET_RIGHT_MOTOR_CURRENT = 55,
	OI_PACKET_BRUSH_MOTOR_CURRENT = 56,
	OI_PACKET_SIDE_BRUSH_MOTOR_CURRENT = 57,
	OI_PACKET_STASIS = 58
	
} OI_Packet_ID;


// Packets sizes
#define OI_PACKET_GROUP_0_SIZE				26
#define OI_PACKET_GROUP_1_SIZE				10
#define OI_PACKET_GROUP_2_SIZE				6
#define OI_PACKET_GROUP_3_SIZE				10
#define OI_PACKET_GROUP_4_SIZE				14
#define OI_PACKET_GROUP_5_SIZE				12
#define OI_PACKET_GROUP_6_SIZE				52
#define OI_PACKET_GROUP_100_SIZE			80
#define OI_PACKET_GROUP_101_SIZE			28
#define OI_PACKET_GROUP_106_SIZE			12
#define OI_PACKET_GROUP_107_SIZE			9
#define OI_PACKET_BUMPS_DROPS_SIZE			1
#define OI_PACKET_WALL_SIZE				1
#define OI_PACKET_CLIFF_LEFT_SIZE			1
#define OI_PACKET_CLIFF_FRONT_LEFT_SIZE			1
#define OI_PACKET_CLIFF_FRONT_RIGHT_SIZE		1
#define OI_PACKET_CLIFF_RIGHT_SIZE			1
#define OI_PACKET_VIRTUAL_WALL_SIZE			1
#define OI_PACKET_WHEEL_OVERCURRENTS_SIZE		1
#define OI_PACKET_DIRT_DETECT_SIZE			1	
#define OI_PACKET_IR_CHAR_OMNI_SIZE			1
#define OI_PACKET_IR_CHAR_LEFT_SIZE			1
#define OI_PACKET_IR_CHAR_RIGHT_SIZE			1
#define OI_PACKET_BUTTONS_SIZE				1
#define OI_PACKET_DISTANCE_SIZE				2
#define OI_PACKET_ANGLE_SIZE				2
#define OI_PACKET_CHARGING_STATE_SIZE			1
#define OI_PACKET_VOLTAGE_SIZE				2
#define OI_PACKET_CURRENT_SIZE				2
#define OI_PACKET_TEMPERATURE_SIZE			1
#define OI_PACKET_BATTERY_CHARGE_SIZE			2
#define OI_PACKET_BATTERY_CAPACITY_SIZE			2
#define OI_PACKET_WALL_SIGNAL_SIZE			2
#define OI_PACKET_CLIFF_LEFT_SIGNAL_SIZE		2
#define OI_PACKET_CLIFF_FRONT_LEFT_SIGNAL_SIZE		2
#define OI_PACKET_CLIFF_FRONT_RIGHT_SIGNAL_SIZE		2
#define OI_PACKET_CLIFF_RIGHT_SIGNAL_SIZE		2
#define OI_PACKET_CHARGE_SOURCES_SIZE			1
#define OI_PACKET_OI_MODE_SIZE				1
#define OI_PACKET_SONG_NUMBER_SIZE			1
#define OI_PACKET_SONG_PLAYING_SIZE			1
#define OI_PACKET_STREAM_PACKETS_SIZE			1
#define OI_PACKET_REQ_VELOCITY_SIZE			2
#define OI_PACKET_REQ_RADIUS_SIZE			2
#define OI_PACKET_REQ_RIGHT_VELOCITY_SIZE		2
#define OI_PACKET_REQ_LEFT_VELOCITY_SIZE		2
#define OI_PACKET_RIGHT_ENCODER_SIZE			2
#define OI_PACKET_LEFT_ENCODER_SIZE			2
#define OI_PACKET_LIGHT_BUMPER_SIZE			1
#define OI_PACKET_LIGHT_BUMPER_LEFT_SIZE		2
#define OI_PACKET_LIGHT_BUMPER_FRONT_LEFT_SIZE		2
#define OI_PACKET_LIGHT_BUMPER_CENTER_LEFT_SIZE		2
#define OI_PACKET_LIGHT_BUMPER_CENTER_RIGHT_SIZE	2
#define OI_PACKET_LIGHT_BUMPER_FRONT_RIGHT_SIZE		2
#define OI_PACKET_LIGHT_BUMPER_RIGHT_SIZE		2
#define OI_PACKET_LEFT_MOTOR_CURRENT_SIZE		2	
#define OI_PACKET_RIGHT_MOTOR_CURRENT_SIZE		2
#define OI_PACKET_BRUSH_MOTOR_CURRENT_SIZE		2
#define OI_PACKET_SIDE_BRUSH_MOTOR_CURRENT_SIZE		2
#define OI_PACKET_STASIS_SIZE				1

// OI Modes
#define OI_MODE_OFF			0
#define OI_MODE_PASSIVE			1
#define OI_MODE_SAFE			2
#define OI_MODE_FULL			3

// Delay after mode change
#define OI_DELAY_MODECHANGE_MS		20

// Charging states
#define OI_CHARGING_NO			0
#define OI_CHARGING_RECOVERY		1
#define OI_CHARGING_CHARGING		2
#define OI_CHARGING_TRICKLE		3
#define OI_CHARGING_WAITING		4
#define OI_CHARGING_ERROR		5

// Charging source
#define OI_NO_CHARGING_SOURCES		0
#define OI_INTERNAL_CHARGER		1
#define OI_HOME_BASE			2
#define OI_BOTH_CHARGING_SOURCES	3

// Positions
#define LEFT				0
#define RIGHT				1
#define FRONT_LEFT			2
#define FRONT_RIGHT			3
#define CENTER_LEFT			4
#define CENTER_RIGHT			5
#define OMNI				2
#define MAIN_BRUSH			2
#define SIDE_BRUSH			3

// Buttons
#define BUTTON_CLOCK			7
#define BUTTON_SCHEDULE			6
#define BUTTON_DAY			5
#define BUTTON_HOUR			4
#define BUTTON_MINUTE			3
#define BUTTON_DOCK			2
#define BUTTON_SPOT			1
#define BUTTON_CLEAN			0

// Roomba Geometry
#define ROOMBA_BUMPER_X_OFFSET		0.050
#define ROOMBA_DIAMETER			0.330
#define ROOMBA_AXLE_LENGTH		0.235

#define ROOMBA_MAX_LIN_VEL_MM_S		500
#define ROOMBA_MAX_ANG_VEL_RAD_S	2  
#define ROOMBA_MAX_RADIUS_MM		2000

// Roomba Odometry
#define ROOMBA_MAX_ENCODER_COUNTS	65535
#define ROOMBA_PULSES_TO_M		0.000445558279992234

#define MAX_PATH 32


#ifndef MIN
#define MIN(a,b) ((a < b) ? (a) : (b))
#endif
#ifndef MAX
#define MAX(a,b) ((a > b) ? (a) : (b))
#endif
#ifndef NORMALIZE
#define NORMALIZE(z) atan2(sin(z), cos(z))
#endif


// *****************************************************************************
// The class for the communication layer
class iRobot_OI
{
	public:
	
	// Constructor; need that
	iRobot_OI(const char * newSerialPort);
	~iRobot_OI();
	
	// Serial port
	int openSerialPort(bool fullControl);
	int closeSerialPort();
	
	// Power Down
	int powerDown();
	
	// Sensor Packets
	int setSensorPackets(OI_Packet_ID * newSensorPackets, int newNumOfPackets, size_t newBufferSize);
	int getSensorPackets(int timeout);
	
	// Calculate Odometry
	void calculateOdometry();
	
	// Motor control
	int drive(double linearSpeed, double angularSpeed);
	int driveDirect(int leftSpeed, int rightSpeed);
	int drivePWM(int leftPWM, int rightPWM);
	
	// Brushes control
	int brushes();
	int brushesPWM();
	
	// Actuate on the robot
	int clean();
	int max();
	int spot();
	int goDock();
	
	int setSong(unsigned char songNumber, unsigned char songLength, unsigned char *notes, unsigned char *noteLengths);
	
	int playSong(unsigned char songNumber);
	
	int setLeds(unsigned char checkRobot, unsigned char dock, unsigned char spot, unsigned char debris, unsigned char powerColor, unsigned char powerIntensity);
	
	int setSchedulingLeds(unsigned char sun, unsigned char mon, unsigned char tue, unsigned char wed, unsigned char thu, unsigned char fri, unsigned char sat, unsigned char colon, unsigned char pm, unsigned char am, unsigned char clock, unsigned char schedule);
	
	int setDigitLeds(unsigned char digit3, unsigned char digit2, unsigned char digit1, unsigned char digit0);
	
	unsigned char OImode;	// Current operation mode, one of ROOMBA_MODE_*
	
	void resetOdometry();
	void setOdometry(double newX, double newY, double newYaw);
	
	double odometryX;
	double odometryY;
	double odometryYaw;
	
	bool wall;
	bool virtualWall;
	bool cliff[4];				// LEFT FRONT_LEFT FRONT_RIGHT RIGHT
	bool bumper[2];				// LEFT RIGHT
	bool irBumper[6];			// LEFT FRONT_LEFT CENTER_LEFT CENTER_RIGHT FRONT_RIGHT RIGHT
	bool wheelDrop[2];			// LEFT RIGHT
	int wallSignal;
	int cliffSignal[4];			// LEFT FRONT_LEFT FRONT_RIGHT RIGHT
	int irBumperSignal[6];			// LEFT FRONT_LEFT CENTER_LEFT CENTER_RIGHT FRONT_RIGHT RIGHT
	unsigned char irChar[3];		// OMNI LEFT RIGHT
	
	bool buttons[8];
	
	unsigned char dirtDetect;
	
	int motorCurrent[4];			// LEFT RIGHT MAIN_BRUSH SIDE_BRUSH
	bool overcurrent[4];			// LEFT RIGHT MAIN_BRUSH SIDE_BRUSH
	
	unsigned char chargingState;		// One of OI_CHARGING_*
	int chargingSource;			// Source of charge
	float voltage;				// Volts
	float current;				// Amps
	char temperature;			// Degrees C
	float charge;				// Ah
	float capacity;				// Capacity Ah
	
	int stasis;				// 1 when the robot is going forward, 0 otherwise

	private:
	
	// Sensor Parsing
	int parseSensorPackets(unsigned char * buffer, size_t bufferLength);
	
	int parseBumpersAndWheeldrops(unsigned char * buffer, int index);
	int parseWall(unsigned char * buffer, int index);
	int parseLeftCliff(unsigned char * buffer, int index);
	int parseFrontLeftCliff(unsigned char * buffer, int index);
	int parseFrontRightCliff(unsigned char * buffer, int index);
	int parseRightCliff(unsigned char * buffer, int index);	
	int parseVirtualWall(unsigned char * buffer, int index);
	int parseOvercurrents(unsigned char * buffer, int index);
	int parseDirtDetector(unsigned char * buffer, int index);
	int parseIrOmniChar(unsigned char * buffer, int index);
	int parseButtons(unsigned char * buffer, int index);
	int parseDistance(unsigned char * buffer, int index);
	int parseAngle(unsigned char * buffer, int index);
	int parseChargingState(unsigned char * buffer, int index);
	int parseVoltage(unsigned char * buffer, int index);
	int parseCurrent(unsigned char * buffer, int index);
	int parseTemperature(unsigned char * buffer, int index);
	int parseBatteryCharge(unsigned char * buffer, int index);
	int parseBatteryCapacity(unsigned char * buffer, int index);
	int parseWallSignal(unsigned char * buffer, int index);
	int parseLeftCliffSignal(unsigned char * buffer, int index);
	int parseFrontLeftCliffSignal(unsigned char * buffer, int index);
	int parseFontRightCliffSignal(unsigned char * buffer, int index);
	int parseRightCliffSignal(unsigned char * buffer, int index);
	int parseChargingSource(unsigned char * buffer, int index);
	int parseRightEncoderCounts(unsigned char * buffer, int index);
	int parseLeftEncoderCounts(unsigned char * buffer, int index);
	int parseLightBumper(unsigned char * buffer, int index);
	int parseLightBumperLeftSignal(unsigned char * buffer, int index);
	int parseLightBumperFrontLeftSignal(unsigned char * buffer, int index);
	int parseLightBumperCenterLeftSignal(unsigned char * buffer, int index);
	int parseLightBumperCenterRightSignal(unsigned char * buffer, int index);
	int parseLightBumperFrontRightSignal(unsigned char * buffer, int index);
	int parseLightBumperRightSignal(unsigned char * buffer, int index);
	int parseIrCharLeft(unsigned char * buffer, int index);
	int parseIrCharRight(unsigned char * buffer, int index);
	int parseLeftMotorCurrent(unsigned char * buffer, int index);
	int parseRightMotorCurrent(unsigned char * buffer, int index);
	int parseMainBrushMotorCurrent(unsigned char * buffer, int index);
	int parseSideBrushMotorCurrent(unsigned char * buffer, int index);
	int parseStasis(unsigned char * buffer, int index);
	
	int buffer2signed_int(unsigned char * buffer, int index);
	int buffer2unsigned_int(unsigned char * buffer, int index);
	
	int startOI(bool fullControl);
	int sendOpcode(OI_Opcode code);
	
	// Serial port to which the robot is connected
	char serialPort[MAX_PATH];
	// File descriptor associated with serial connection (-1 if no valid connection)
	int fileDescriptor;
	
	int numOfPackets;
	OI_Packet_ID * sensorPackets;
	size_t packetsSize;
	
	int distance;
	int angle;
	int encoderCounts[2];
	uint16_t lastEncoderCounts[2];
};

// EOF
