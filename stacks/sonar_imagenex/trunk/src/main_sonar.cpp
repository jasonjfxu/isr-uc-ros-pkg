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
* Author: João Sousa on 06/06/2013
*********************************************************************/

#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/ByteMultiArray.h>
#include <tf/transform_broadcaster.h>
#include <Eigen/Eigen>
#include <cereal_port/CerealPort.h>

#include "sonar_param_class.h"

#define DEG2RAD(a)              ((a) * M_PI / 180.0)
#define RAD2DEG(a)              ((a) * 180.0 / M_PI)
#define SONAR_MAX_RATE          20

int val1 = 0, val2 = 2, gain = 3, step_lenght = 10;
int train_angle = 0, sector_width = 90, initial_range = 5;

double loopr = 10;
bool first_read = true;

using namespace std;
using namespace Eigen;

int main(int argc, char **argv)
{	
	ros::init(argc, argv, "serial_sonar");
	
    ros::NodeHandle n;
    ros::NodeHandle pn("~");

    pn.param("train_angle", train_angle, 90);
    pn.param("sector_width", sector_width, 45);
    pn.param("initial_range", initial_range, 5);
    pn.param("step_lenght", step_lenght, 10);
    pn.param("frequency", loopr, 10.0);
    pn.param("gain", gain, 5);

    std::string serial_path;
    pn.param<std::string>("port", serial_path, "/dev/ttyUSB0");
    int baudrate;
    pn.param("baudrate", baudrate, 115200);

    std::string sonar_frame_id;
    pn.param<std::string>("sonar_frame_id", sonar_frame_id, "sonar");
    std::string sonar_transducer_frame_id;
    pn.param<std::string>("sonar_transducer_frame_id", sonar_transducer_frame_id, "sonar_transducer");

    ros::Publisher sender_sonar = n.advertise<std_msgs::ByteMultiArray>("sonar_raw_data", 10);
    // range msg
    ros::Publisher sonar_range_pub = n.advertise<sensor_msgs::Range>("sonar_range", 10);

    ros::Rate loop_rate(loopr);
    static tf::TransformBroadcaster tf_sonar_br;
    tf::Transform tf_sonar_transducer;

    cereal::CerealPort sonar_serial;
	
	Imagenex IGX;

    sonar_serial.open(serial_path.c_str(), baudrate);
	
    if(!sonar_serial.portOpen())
    {
        ROS_INFO("serial port busy\nTry another serial port...\n");
    }

    // configure sonar
	IGX.change_range(initial_range);
	IGX.change_train_angle(train_angle);
	IGX.change_sector_width(sector_width);
	IGX.change_frequency(850);
    IGX.change_step_length(step_lenght);
    IGX.change_gain(gain);
	IGX.sonar_msg_build();	

	//ROS
    std_msgs::ByteMultiArray raw_msg;
    raw_msg.data.clear();

    VectorXf conf, peaks;
	
    while(ros::ok())
    {
        sonar_serial.write(IGX.sCmd,27);
		
        sonar_serial.readBytes(IGX.sRec,IGX.Data_points+13,3000);
				
        IGX.sonar_rec_parse();

        static VectorXf current_read, previous_read;
        current_read = IGX.get_raw_data();
        float echo = -1;

        if(first_read)
        {
            first_read = false;
            previous_read = current_read;
        }

        peaks = obstacle_interpretation(current_read, previous_read, conf);

        if(peaks.rows() > 0)
        {
            int ind_max_conf;
            conf.maxCoeff(&ind_max_conf);
            echo = IGX.get_range() * peaks(ind_max_conf) / (float)IGX.get_data_bytes();

            if(echo > 0.3)
            {
                sensor_msgs::Range sonar_range;
                sonar_range.header.frame_id = sonar_transducer_frame_id;
                sonar_range.header.stamp = ros::Time::now();
                sonar_range.radiation_type = sensor_msgs::Range::ULTRASOUND;
                sonar_range.max_range = IGX.get_range();
                sonar_range.min_range = 0.3;
                sonar_range.range = echo;
                sonar_range.field_of_view = 0.17;

                sonar_range_pub.publish(sonar_range);

            }
        }
        tf_sonar_transducer.setOrigin(tf::Vector3(0,0,0));
        tf_sonar_transducer.setRotation(tf::createQuaternionFromRPY(0.0, 0.0, DEG2RAD(IGX.get_head_pos())));
        tf_sonar_br.sendTransform(tf::StampedTransform(tf_sonar_transducer, ros::Time::now(), sonar_frame_id, sonar_transducer_frame_id));


        // envia a mensagem completa
        raw_msg.data.resize(0);         //set vector to 0 lenght

        for(int ind = 0; ind < IGX.get_data_bytes()+12; ind++)
        {
            raw_msg.data.push_back(IGX.get_sRec(ind));
        }

        sender_sonar.publish(raw_msg);

        ros::spinOnce();
        loop_rate.sleep();

    }

    return(0);
}
