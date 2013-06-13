/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2013, ISR University of Coimbra.
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
* Author: Gonçalo Cabrita on 06/06/2013
*********************************************************************/

#include <cereal_port/CerealPort.h>

#define SQUIRTLE_MAX_MOTOR 100
#define SQUIRTLE_MIN_MOTOR -100
#define SQUIRTLE_MAX_LINEAR_ACTUATOR 100
#define SQUIRTLE_MIN_LINEAR_ACTUATOR -100

class Squirtle
{
public:
    
    Squirtle();
    ~Squirtle();
    
    bool openPort(char * port, int baudrate);
    bool initialize();
    bool startStreaming(int motors, int fluorometer, int pm);
    void streamCallback(std::string * msg);

    bool parseMotorData(const char * msg);
    bool parseChemicalSensorData(const char * msg);
    bool parsePowerData(const char * msg);
    bool parseLM35Data(const char * msg);

    void drive(int left_motor, int right_motor, int linear_actuator);

    double getBatteryVoltage(){ return battery_voltage_; }
    double getBatteryCurrent(){ return battery_current_; }
    double getBatteryTemperature(){ return battery_temperature_; }
    double getSolarPanelVoltage(){ return solar_panel_voltage_; }
    double getSolarPanelCurrent(){ return solar_panel_current_; }
    double getLeftMotorCurrent(){ return left_motor_current_; }
    double getRightMotorCurrent(){ return right_motor_current_; }
    double getMotorDriverTemperature(){ return motor_driver_temperature_; }

    double getChemicalSensorReading(){ return chemical_sensor_reading_; }

    bool isReady(){ return arduino_ready_; }

private:
    
    static cereal::CerealPort serial_port;

    double battery_voltage_;
    double battery_current_;
    double battery_temperature_;
    double solar_panel_voltage_;
    double solar_panel_current_;
    double left_motor_current_;
    double right_motor_current_;
    double motor_driver_temperature_;

    double chemical_sensor_reading_;

    bool arduino_ready_;
};

// EOF

