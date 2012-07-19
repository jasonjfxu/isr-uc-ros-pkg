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
* Author: Gonçalo Cabrita on 17/04/2012
*********************************************************************/

#include <boost/function.hpp>
#include <boost/thread/thread.hpp>

#include <ros/ros.h>
#include <can_msgs/CANFrame.h>

#define ID_MASK             0x7F

//! COB IDs of all communication objects.
#define COB_NMT_SERVICE     0x0
#define COB_SYNC            0x80
#define COB_EMCY            0x80
#define COB_T_PDO1          0x180
#define COB_R_PDO1          0x200
#define COB_T_PDO2          0x280
#define COB_R_PDO2          0x300
#define COB_T_PDO3          0x380
#define COB_R_PDO3          0x400
#define COB_T_SDO           0x580
#define COB_R_SDO           0x600
#define COB_NMT_ERROR       0x700

//! CCD codings, depends on the message type and the transmitted data length.
#define WRITE_REQUEST_4BYTE 0x23
#define WRITE_REQUEST_3BYTE 0x27
#define WRITE_REQUEST_2BYTE 0x2B
#define WRITE_REQUEST_1BYTE 0x2F
#define WRITE_RESPONSE      0x60
#define READ_REQUEST_4BYTE  0x43
#define READ_REQUEST_3BYTE  0x47
#define READ_REQUEST_2BYTE  0x4B
#define READ_REQUEST_1BYTE  0x4F
#define READ_RESPONSE       0x40
#define ERROR_RESPONSE      0x80

namespace canopen
{
	//! Macro for defining an exception with a given parent (std::runtime_error should be top parent)
	/*#define DEF_EXCEPTION(name, parent) \
	class name : public parent { \
  		public: \
    	name(const char* msg) : parent(msg) {} \
  	}
  
  	//! A standard exception
  	DEF_EXCEPTION(Exception, std::runtime_error);

	#undef DEF_EXCEPTION*/
    
    //! Homing methods.
    typedef enum _HomingMethod
    {
        method_1 = 1,
        method_2 = 2,
        method_3 = 3,
        method_4 = 4,
        method_5 = 5,
        method_6 = 6,
        method_7 = 7,
        method_8 = 8,
        method_9 = 9,
        method_10 = 10,
        method_11 = 11,
        method_12 = 12,
        method_13 = 13,
        method_14 = 14,
        method_17 = 17,
        method_18 = 18,
        method_19 = 19,
        method_20 = 20,
        method_21 = 21,
        method_22 = 22,
        method_23 = 23,
        method_24 = 24,
        method_25 = 25,
        method_26 = 26,
        method_27 = 27,
        method_28 = 28,
        method_29 = 29,
        method_30 = 30,
        method_33 = 33,
        method_34 = 34,
        method_35 = 35,
        
    } HomingMethod;

	/*! \class CANopenMotorDriver CANopenMotorDriver.h "inc/CANopenMotorDriver.h"
	 *  \brief C++ CANopenMotorDriver class for ROS.
	 *
	 * This class was based on Technosoft CANopen programming guide.
	 */
	class CANopenMotorDriver
	{
        
    public:
		//! Constructor
		CANopenMotorDriver(unsigned char axis_id, ros::NodeHandle * nh);
		//! Destructor
		~CANopenMotorDriver();
        
        
        /***************************************** SCALING FACTORS STUFF *****************************************/
        
        void setupDCmotorWithEncoder(unsigned int num_of_encoder_lines, double Tr, double T);
        
        void setupStepperMotor(unsigned int num_of_steps, unsigned int num_of_usteps, double Tr, double T);
        
        
        /***************************************** GENERAL STUFF *****************************************/
         
        //! Start remote node.
		/*! 
         * This starts up the remote node.
         * 
         */
        void startNode();
        
        //! Ready to switch on.
		/*! 
         * Ready to switch on.
         * 
         */
        void readyToSwitchOn();
        
        //! Switch on.
		/*! 
         * Switch on.
         * 
         */
        void switchOn();
        
        //! Enable operation.
		/*! 
         * Enable operation.
         * 
         */
        void enableOperation();
        
        //! Run.
		/*! 
         * Run the last defined commands.
         * 
         */
        void run();
        
        
        
        /***************************************** HOMING MODE *****************************************/
         
		//! Setup homing
		/*! 
		* Homing is the method by which a drive seeks the home position.
        * There are various methods to achieve this position using the four available sources for the homing signal:
        * limit switches (negative and positive), home switch and index pulse.
		* 
		* \param method         The homing method determines the method that will be used during homing (1 to 35).
		* \param fast_speed     The slower speed is used to find the index pulse.
        * \param slow_speed     The faster speed is used to find the home switch.
        * \param acceleration   The acceleration to be used for all the accelerations and decelerations with the standard homing modes.
        * \param offset         The difference between the zero position for the application and the machine home position.
		*
		*/
        void setupHomingMode(HomingMethod method, double fast_speed, double slow_speed, double acceleration, double offset=0.0);
        
        
        
        /***************************************** POSITION MODE *****************************************/
         
        //! Setup position trapezoidal profile mode
		/*! 
         * Trapezoidal profile. The built-in reference generator computes the position profile with a trapezoidal shape of the speed,
         * due to a limited acceleration. The CANopen master specifies the absolute or relative Target Position, the Profile Velocity
         * and the Profile Acceleration.
         * 
         * \param target         The position that the drive should move to.
         * \param speed          The maximum speed to reach at the end of the acceleration ramp.
         * \param acceleration   The acceleration and deceleration rates used to change the speed between 2 levels.
         *
         */
        void setupPositionTrapezoidalProfileMode(double target, double speed, double acceleration);
        
        //! Setup position S-curve profile mode
		/*! 
         * S-curve profile The built-in reference generator computes a position profile with an S-curve shape of the speed.
         * This shape is due to the jerk limitation, leading to a trapezoidal or triangular profile for the acceleration and
         * an S-curve profile for the speed. The CANopen master specifies the absolute or relative Target Position, the Profile Velocity,
         * the Profile Acceleration and the jerk rate. The jerk rate is set indirectly via the Jerk time, which represents the time needed to reach the
         * maximum acceleration starting from zero.
         * 
         * \param target         The position that the drive should move to.
         * \param speed          The maximum speed to reach at the end of the acceleration ramp.
         * \param acceleration   The acceleration and deceleration rates used to change the speed between 2 levels.
         * \param jerk_time      The acceleration and deceleration rates used to change the speed between 2 levels.
         *
         */
        void setupPositionScurveProfileMode(double target, double speed, double acceleration, double jerk_time);
        
        //! Set quick stop deceleration
		/*! 
         * The quick stop deceleration is the deceleration used to stop the motor if the Quick Stop command is received.
         * 
         * \param deceleration  Quick stop deceleration.
         *
         */
        void setQuickStopDeceleration(double deceleration);
        
        // TODO: Implement getPositionActualValue();
        
        // TODO: Implement setFollowingErrorWindow();
        // TODO: Implement setFollowingErrorTimeOut();
        
        // TODO: Implement setPositionWindow();
        // TODO: Implement setPositionTimeOut();
        
        // TODO: Implement getFollowingErrorActualValue();
        
        // TODO: Implement getPositionDemandValue();
        
        // TODO: Implement getControlEffort();
        
        
        
        /***************************************** VELOCITY MODE *****************************************/
         
        //! Setup velocity profile mode
		/*! 
         * In the Velocity Profile Mode the drive performs speed control.
         * The built-in reference generator computes a speed profile with a trapezoidal shape, due to a limited acceleration.
         * The Target Velocity object specifies the jog speed (speed sign specifies the direction) and the Profile
         * Acceleration object the acceleration/deceleration rate.
         * 
         * \param target_speed        The target speed.
         * \param acceleration        The acceleration and deceleration rates used to change the speed between 2 levels.
         *
         */
        void setupVelocityProfileMode(double target_speed, double acceleration);
        
        // TODO: Implement getVelocitySensorActualValue();
        
        // TODO: Implement getVelocityDemandValue();
        
        // TODO: Implement getVelocityActualValue();
        
        // TODO: Implement setVelocityThreshold();
        
        // TODO: Implement setMaxSlippage();
        // TODO: Implement setSlippageTimeOut();
        
        
        
    private:
        
        //! ID of the CAN device being used
        unsigned char id_;
        
        
        /***************************************** SCALING FACTORS STUFF *****************************************/
        
        //! Time convertion factor
        double T_;
        //! Position units
        double load_to_motor_position_;
        //! Speed units
        double load_to_motor_speed_;
        //! Acceleration units
        double load_to_motor_acceleration_;
        //! Jerk units
        double load_to_motor_jerk_;
        
        
        
        /***************************************** ROS STUFF *****************************************/
        //ros::NodeHandle nh_;
        //! CAN bus publisher
        ros::Publisher can_pub_;
        //! CAN bus subscriber
        ros::Subscriber can_sub_;
        
        //! Received CANframe
		/*! 
         * This is the callback function for incoming CAN frames.
         * 
         * \param msg        ROS can_msgs/CANFrame message.
         *
         */
        void receivedCANframe(const can_msgs::CANFrame::ConstPtr& msg);
        
        
        
        /***************************************** CALLBACKS *****************************************/
        
        //! Callback boost function for limit switches
		boost::function<void(char*, int)> limitSwitchesCallback_;
	};

}

// EOF
