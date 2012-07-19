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
* Author: Gonçalo Cabrita on 28/05/2012
*********************************************************************/

#include <ros/ros.h>
#include <can_msgs/CANFrame.h>

// Masks
#define ROBCHAIR_FUNCTION_MASK      0x007
#define ROBCHAIR_SOURCE_MASK        0x078
#define ROBCHAIR_DESTINATION_MASK   0x780

// Robot constants
#define ROBCHAIR_AXLE_LENGTH        0.61492
#define ROBCHAIR_WHEEL_RADIUS       0.175

#define ROCHAIR_ENCODER_PULSES      2000
#define ROBCHAIR_GEARBOX            10

#define ROBCHAIR_CONTROL_PERIOD     0.005

#define ROBCHAIR_K ((ROBCHAIR_CONTROL_PERIOD*ROCHAIR_ENCODER_PULSES*ROBCHAIR_GEARBOX)/(2* M_PI))  

class RobChair
{
public:
    
    //! RobChair destinations and sources
    typedef enum _RobChairDS {
        
        PC = 0,
        RightPDrive = 1,
        LeftPDrive = 2,
        BothPDrives = 3,
        RightEncoder = 4,
        LeftEncoder = 5,
        BothEncoders = 6,
        Joystick = 10,
        SyncMCU = 15
        
    } RobChairDS;
    
    //! Common functions
    typedef enum _CommonFunction {
        
        TurnOff = 0,
        TurnOn = 1
        
    } CommonFunction;
    
    //! PDrive functions
    typedef enum _PDriveFunction {
        
        SetDACCommand = 2,
        SetPDriveControlMode = 3,
        DataFromMotor = 4
        
    } PDriveFunction;
    
    //! Encoder functions
    typedef enum _EncoderFunction {
        
        SetVelocityValue = 2,
        SetEncoderControlMode = 3,
        SetPIControlValues = 4,
        ResetEncoderInformation = 5,
        DataFromEncoder = 6
        
    } EncoderFunction;
    
    //! Trigger functions
    typedef enum _TriggerFunction {
        
        SyncronizeAllNodes = 15
        
    } TriggerFunction;
    
    /*!
	 *
	 * \fn RobChair()
	 * \brief RobChair constructor. This is the class constructor.
	 * \return none.
	 *
	 */
    RobChair();
    //! RobChair destructor.
    /*!
	 * This is the class destructor.
     */
    ~RobChair();
    
    /*!
	 *
	 * \fn void initializeRobChair()
	 * \brief This function initializes the PDrives, the Encoders and the Trigger in the RobChair robot.
     * \param can_pub The ROS publisher for the CAN topic.
     * \param kp The proportional gain of the speed controller.
     * \param ki The integral gain of the speed controller.
	 *
	 */
    void initializeRobChair(ros::Publisher * can_pub, double kp=0.0, double ki=0.0);
    
    /*!
	 *
	 * \fn void setPIControlValues(RobChairDS destination, double kp, double ki, double kd)
	 * \brief This function allows to set the PID control gains for the RobChair speed controller.
     * \param destination The destination PDrive, accepted values are LeftPDrive, RightPDrive and BothPDrives.
     * \param kp The proportional gain of the speed controller.
     * \param ki The integral gain of the speed controller.
	 *
	 */
    void setPIControlValues(RobChairDS destination, double kp, double ki);
    
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
	 * \fn void resetEncoders()
	 * \brief This function resets both encoders.
	 *
	 */
    void resetEncoders(){ sendCANFrame(RobChair::BothEncoders, RobChair::ResetEncoderInformation, NULL, 0); }
    
    /*!
	 *
	 * \fn void receivedCANFrame(const can_msgs::CANFrame::ConstPtr& frame_msg)
	 * \brief Callback for incoming CAN frames.
     * \param msg The incoming CAN frame.
	 *
	 */
    void receivedCANFrame(const can_msgs::CANFrame::ConstPtr& msg);
    
    /*!
	 *
	 * \fn double getPositionX()
	 * \brief Getter for the robot x position.
     * \return The robot x position.
	 *
	 */
    double getPositionX(){ return x_; }
    /*!
	 *
	 * \fn double getPositionY()
	 * \brief Getter for the robot y position.
     * \return The robot y position.
	 *
	 */
    double getPositionY(){ return y_; }
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
    
private:
    
    /*!
	 *
	 * \fn void sendCANFrame(RobChairDS destination, char function, char * data, int data_count)
	 * \brief This functions sends CAN frames to the RobChair CAN bus.
     * \param destination The destination device RobChairDS.
     * \param function The desired function, PDriveFunction, EncoderFunction or TriggerFunction.
     * \param data The data to be sent.
     * \param data_count The number of bytes to be sent.
     * \param n The trigger time slot to send the frame.
	 *
	 */
    void sendCANFrame(RobChairDS destination, char function, char * data, int data_count, char n=0);
    
    /*!
	 *
	 * \fn void setVelocity(RobChairDS destination, double velocity)
	 * \brief This function allows to set the velocity reference on the RobChair motor drivers.
     * \param destination The destination device RobChairDS, valid values are LeftEncoder, RightEncoder and BothEncoders.
     * \param velocity The reference velocity to the controller.
	 *
	 */
    void setVelocity(RobChairDS destination, double velocity);
    
    /*!
	 *
	 * \fn void decodeEncoderData(RobChairDS encoder)
	 * \brief This function decodes the data sent from the encoders to the PC.
     * \param encoder The source of the data, valid values are LeftEncoder and RightEncoder.
     * \param msg The data to be decoded.
	 *
	 */
    void decodeEncoderData(RobChairDS encoder, const can_msgs::CANFrame::ConstPtr& msg);
    
    
    /*!
	 *
	 * \fn void turnPDrivesOn()
	 * \brief This functions turns the PDrives on the RobChair ON - Running Mode.
	 *
	 */
    void turnPDrivesOn(){ sendCANFrame(BothPDrives, TurnOn, NULL, 0); }
    /*!
	 *
	 * \fn void turnPDrivesOff()
	 * \brief This functions turns the PDrives on the RobChair OFF - Idle Mode.
	 *
	 */
    void turnPDrivesOff(){ sendCANFrame(BothPDrives, TurnOff, NULL, 0); }
    
    /*!
	 *
	 * \fn void turnEncodersOn()
	 * \brief This functions turns the Encoders on the RobChair ON - Running Mode.
	 *
	 */
    void turnEncodersOn(){ sendCANFrame(BothEncoders, TurnOn, NULL, 0); }
    /*!
	 *
	 * \fn void turnEncodersOff()
	 * \brief This functions turns the Encoders on the RobChair OFF - Idle Mode.
	 *
	 */
    void turnEncodersOff(){ sendCANFrame(BothEncoders, TurnOff, NULL, 0); }
    
    /*!
	 *
	 * \fn void turnTriggerOn()
	 * \brief This functions turns the Trigger on the RobChair ON - Running Mode.
	 *
	 */
    void turnTriggerOn(){ sendCANFrame(SyncMCU, TurnOn, NULL, 0); }
    /*!
	 *
	 * \fn void turnTriggerOff()
	 * \brief This functions turns the Trigger on the RobChair OFF - Running Mode.
	 *
	 */
    void turnTriggerOff(){ sendCANFrame(SyncMCU, TurnOff, NULL, 0); }
    
    
    //! CANopen bus ROS publisher
    ros::Publisher * can_pub_;
    
    //! Current time slot sent by the trigger
    char n_;
    
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
    
    //! Left wheel position
    double left_wheel_position_;
    //! Left wheel velocity
    double left_wheel_velocity_;
    //! Right wheel position
    double right_wheel_position_;
    //! Right wheel velocity
    double right_wheel_velocity_;
    
    // Aux variables
    //! The last left wheel position
    double last_left_wheel_position_;
    //! The last right wheel position
    double last_right_wheel_position_;
    //! Flag for left encoder data
    bool got_left_encoder_data_;
    //! Flag for right encoder data
    bool got_right_encoder_data_;

    /*!
	 *
	 * \fn double limitAngle(double angle)
	 * \brief The angles must be contained between [-Pi, Pi]
     * \param angle The angle to be limited.
	 *
	 */
    double limitAngle(double angle);
};

// EOF
