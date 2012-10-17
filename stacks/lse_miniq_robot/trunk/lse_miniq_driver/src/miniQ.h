/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2012, ISR University of Coimbra.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the ISR University of Coimbra nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Gonçalo Cabrita on 20/08/2012
*********************************************************************/

#include <cereal_port/CerealPort.h>

#define MSG_LENGTH 64

#define MINIQ_RATE 3.0

class miniQ
{
public:
    
    //! miniQ actions
    typedef enum _miniQaction {
        
        // Get version
    	MQ_ACTION_GET_VERSION = 1,
    	// Actuate the robot
    	MQ_ACTION_DRIVE = 2,
    	MQ_ACTION_DRIVE_DIRECT = 3,
    	// Get robot sensors
    	MQ_ACTION_GET_ODOMETRY = 4,
    	MQ_ACTION_GET_ENCODER_PULSES = 5,
	MQ_ACTION_GET_WHEEL_VELOCITIES = 6,
   	MQ_ACTION_GET_GAS_SENSOR = 7,
    	MQ_ACTION_GET_IR_BUMPER = 8,
    	MQ_ACTION_GET_LINE_SENSOR = 9,
    	MQ_ACTION_GET_BATTERY = 10,
    	// Debug mode
    	MQ_ACTION_GET_DEBUG = 11,
    	MQ_ACTION_SET_DEBUG = 12,
    	// Configuration
    	MQ_ACTION_SET_PID_GAINS = 13,
    	MQ_ACTION_GET_PID_GAINS = 14,
    	MQ_ACTION_SET_ODOMETRY_CALIBRATION = 15,
    	MQ_ACTION_GET_ODOMETRY_CALIBRATION = 16,
    	MQ_ACTION_SET_ID = 17,
    	MQ_ACTION_GET_ID = 18,
   	MQ_ACTION_SET_MODE = 19,
    	MQ_ACTION_GET_MODE = 20,
    	MQ_ACTION_SET_GAS_CALIBRATION = 21,
   	MQ_ACTION_GET_GAS_CALIBRATION = 22,
    	MQ_ACTION_SET_BATTERY_TYPE = 23,
    	MQ_ACTION_GET_BATTERY_TYPE = 24,
    	// Group messages
    	// Group 1 - Odometry, gas sensor
    	MQ_ACTION_GET_GROUP_1 = 25,
    	// Group 2 - Odometry, gas sensor, IR bumper
    	MQ_ACTION_GET_GROUP_2 = 26,
    	MQ_ACTION_COUNT = 27
        
    } miniQaction;

    typedef enum _miniQPID
    {
	LeftStartingPID = 0,
	LeftRunningPID = 1,
	RightStartingPID = 2,	
	RightRunningPID = 3

    } miniQPID;
    
    miniQ();
    ~miniQ();
    
    /*!
	 *
	 * \fn bool openPort(char * port, int baudrate)
	 * \brief This function opens the virtual comm to the miniQ robot.
     * \param port Serial port name.
     * \param baudrate The name is self explanatory.
     * \return True if port is open, false otherwise.
	 *
	 */
    static bool openPort(char * port, int baudrate);
    
    /*!
	 *
	 * \fn bool checkVersion()
	 * \brief This function asks the firmware version from the miniQ robot.
     * \return True if the version is compatible, false otherwise.
	 *
	 */
    bool checkVersion();
    
    /*!
	 *
	 * \fn void setVelocities(double linear_velocity, double angular_velocity)
	 * \brief This function allows to send velocity commands to the velocity controllers.
     * \param linear_velocity Linear velocity to the robot in m/s.
     * \param angular_velocity Angular velocity to the robot in m/s.
	 *
	 */
    void setVelocities(double linear_velocity, double angular_velocity);
    
    /*!
	 *
	 * \fn bool update()
	 * \brief This function communicates with the robot to update the odometry and gas sensor data.
     * \return True if update was successful, false otherwise.
	 *
	 */
    bool update();
    
    /*!
	 *
	 * \fn double getPositionX()
	 * \brief Getter for the robot x position.
     * \return The robot x position.
	 *
	 */
    double getX(){ return x_; }
    /*!
	 *
	 * \fn double getPositionY()
	 * \brief Getter for the robot y position.
     * \return The robot y position.
	 *
	 */
    double getY(){ return y_; }
    /*!
	 *
	 * \fn double getYay()
	 * \brief Getter for the robot yaw.
     * \return The robot yaw.
	 *
	 */
    double getYaw(){ return yaw_; }
    /*!
	 *
	 * \fn double getLinearVelocity()
	 * \brief Getter for the robot linear velocity.
     * \return The robot linear velocity.
	 *
	 */
    double getLinearVelocity(){ return linear_velocity_; }
    /*!
	 *
	 * \fn double getAngularVelocity()
	 * \brief Getter for the robot angular velocity.
     * \return The robot angular velocity.
	 *
	 */
    double getAngularVelocity(){ return angular_velocity_; }
    /*!
	 *
	 * \fn double getLeftWheelVelocity()
	 * \brief Getter for the left wheel velocity.
     * \return The left wheel velocity.
	 *
	 */
    double getLeftWheelVelocity(){ return left_wheel_velocity_; }
    /*!
	 *
	 * \fn double getRightWheelVelocity()
	 * \brief Getter for the right wheel velocity.
     * \return The right wheel velocity.
	 *
	 */
    double getRightWheelVelocity(){ return right_wheel_velocity_; }
    /*!
	 *
	 * \fn double getGas()
	 * \brief Getter for the gas sensor reading.
     * \return The gas sensor reading in ppm.
	 *
	 */
    double getGas(){ return gas_; }
    /*!
	 *
	 * \fn double getRawGas()
	 * \brief Getter for the gas sensor reading.
     * \return The gas sensor reading in raw form.
	 *
	 */
    double getRawGas(){ return gas_raw_; }

    static int scanForId(int id);

    void setId(int id);
    int getID(){ return id_; };

    bool setMode(int mode);
    bool setBatteryType(int battery_type);
    bool setPIDGains(int kp, int ki, int kd, miniQPID pid);
    bool setOdometryCallibration(double odometry_d, double odometry_yaw);
    bool setGasSensorCallibration(double a, double b);

    void setPWM(int left_pwm, int left_dir, int right_pwm, int right_dir);

    bool updateWheelVelocities();
    
private:
    
    // RobChair variables
    //! Position in x
    double x_;
    //! Position in y
    double y_;
    //! Yaw
    double yaw_;
    //! Velocity in x
    double linear_velocity_;
    //! Angular velocity
    double angular_velocity_;
    //! Left wheel velocity
    double left_wheel_velocity_;
    //! Right wheel velocity
    double right_wheel_velocity_;

    //! Gas
    double gas_;
    //! Raw gas
    double gas_raw_;

    //! Robot id
    int id_;
    
    static cereal::CerealPort serial_port;
};

// EOF
