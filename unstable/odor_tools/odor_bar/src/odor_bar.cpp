/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010, ISR University of Coimbra.
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
* Author: Pedro Sousa on 27/08/2010
*********************************************************************/
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/ColorRGBA.h>
#include <lse_sensor_msgs/Nostril.h>
#include <lse_sensor_msgs/TPA.h>

#define NODE_VERSION 0.2

#define PIXEL_TO_M 20.0


//! flag for verbose mode
bool verbose = true;

//nav_msgs::Odometry myOdom;
//! HSV components
double hsvS = 1.0, hsvV = 1.0;
int hueMax = 200, hueMin = 0;

//! nose data
// TODO: change this for something better
lse_sensor_msgs::Nostril myNostril, myNostril_0, myNostril_1;


void noseCallback(const lse_sensor_msgs::NostrilConstPtr& msg)
{
	myNostril.sensor_model = msg->sensor_model;
	if(myNostril.sensor_model.compare("Figaro 2620"))
	{
		myNostril_0.sensor_model = msg->sensor_model;
		myNostril_0.reading = msg->reading;
	}
	else if(myNostril.sensor_model.compare("Figaro 2600"))
	{
		myNostril_1.sensor_model = msg->sensor_model;
		myNostril_1.reading = msg->reading;
	}

	//ROS_INFO("NoseCallback: <0>%s:%f   <1>%s:%f", myNostril_0.sensor_model.c_str(), myNostril_0.reading, myNostril_1.sensor_model.c_str(), myNostril_1.reading);

}


/*! \fn std::string char2string(char *in)
 *  \brief Convert a char to a string.
 *  
 *	\param in a pointer to char.
 *  \return a std::string.
 */
std::string char2string(char *in)
{
    std::string str(in);
    return str;
}



/*! \fn std_msgs::ColorRGBA hsv_2_rgb(float H, float S, float V)
 *  \brief Convertion function between the HSV and the RGB space colors.
 *  
 *	\param H the Hue component of HSV.
 *	\param S the Saturation component of HSV
 *	\param V the Value component of HSV
 *
 *  \return the converted values of R, G and B.
 */
std_msgs::ColorRGBA hsv_2_rgb(float H, float S, float V)
{
	float R, G, B;
	double hf;
	int    i;
	double f;
	double pv;
	double qv;
	double tv;
	
	std_msgs::ColorRGBA rgbColor;
	
	// h -> [0...360]
	// s and v -> [0...1]
	if( V == 0 )
	{
		R = 0;
		G = 0;
		B = 0;
	}
	else if( S == 0 )
	{
		R = V;
		G = V;
		B = V;
	}
	else
	{
		hf = H / 60.0;
		i  = (int) floor( hf );
		f  = hf - i;
		pv  = V * ( 1 - S );
		qv  = V * ( 1 - S * f );
		tv  = V * ( 1 - S * ( 1 - f ) );
		//ROS_INFO("i%d",i);
		switch( i )
		{
			// Red is the dominant color
			case 0:
				R = V;
				G = tv;
				B = pv;
				break;
			// Green is the dominant color
			
			case 1:
				R = qv;
				G = V;
				B = pv;
				break;
			case 2:
				R = pv;
				G = V;
				B = tv;
				break;
			// Blue is the dominant color
			case 3:
				R = pv;
				G = qv;
				B = V;
				break;
			case 4:
				R = tv;
				G = pv;
				B = V;
				break;
			// Red is the dominant color
			case 5:
			  R = V;
			  G = pv;
			  B = qv;
			  break;
			// just in case something goes wrong 
			case 6:
				R = V;
				G = tv;
				B = pv;
				break;
			case -1:
				R = V;
				G = pv;
				B = qv;
				break;
			default:
				// shouldn't get here
				ROS_ERROR("Value error in Pixel conversion...,");
				break;
		}
	}
	// RGB values from 0...1
	rgbColor.r = R;
	rgbColor.g = G;
	rgbColor.b = B;
	
	return rgbColor;
}


/*! \fn void makeColorBar(visualization_msgs::Marker *ptrMarker, int val)
 *  \brief Make the color bar with the markers according to the specified values.
 *  
 *	\param ptrMarker a pointer to the marker object.
 *	\param val the actual value to be inserted in the bar.
 *
 *  \return nothing.
 */
void makeColorBar(visualization_msgs::Marker *ptrMarker, int val)
{
	std_msgs::ColorRGBA col;
	int step;
	int N_STEPS = 10;
	float v;
	int idx = 0;
	int deltaHue;
	
	deltaHue = hueMax - hueMin;

	step = deltaHue / N_STEPS;
	//val = val * 6;
	val = (val <= 100) ? val : 100;
	//ROS_INFO("val:%d", val);
	v = (val/100.0) * deltaHue;
	
	// clear current values
	ptrMarker->points.clear();
	ptrMarker->colors.clear();
	
	// Fill in the colors
	for (int j = deltaHue; j > (deltaHue - v);j = j - step)
	{
		col = hsv_2_rgb(j, hsvS, hsvV);
		ptrMarker->colors.push_back(col);
		idx++;
	}
	// And now the objects
	for (int i = 0; i < idx;i++)
	{
		geometry_msgs::Point p;
		p.x = 0.0;
		p.y = -0.3;
		p.z = (0.2 + (i*0.15));
		
		// add point to vector
		ptrMarker->points.push_back(p);
	}
	//ROS_INFO("step:%d  v:%f", step, v);	
}


void configColorBar(ros::NodeHandle *pn, visualization_msgs::Marker *ptrMarker)
{
	// Read parameters
	// Scale
	pn->param("scale/x", ptrMarker->scale.x, 0.15);
	pn->param("scale/y", ptrMarker->scale.x, 0.15);
	pn->param("scale/z", ptrMarker->scale.x, 0.15);

	// HSV S and V components
	pn->param("hsv_s", hsvS, 1.0);
	pn->param("hsv_v", hsvV, 1.0);

	// HSV H interval
	pn->param("hue_max", hueMax, 200);
	pn->param("hue_min", hueMin, 0);
}


int main( int argc, char** argv )
{
	ros::init(argc, argv, "odor_bar");
	ros::NodeHandle n("~");
	ROS_INFO("odor_bar for ROS %.2f", NODE_VERSION);

	ros::Subscriber nose_sub = n.subscribe("nose", 2, noseCallback);

	ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("odor_bars", 10);

	float chem_val = 0;
	char currValue[5];
	

	ros::Rate r(1);

	while (ros::ok())
	{
		visualization_msgs::Marker points, text;
		points.header.frame_id = text.header.frame_id = "/base_link";
		points.header.stamp = text.header.stamp = ros::Time::now();
		points.ns = text.ns = "odor_bar";
		text.lifetime = ros::Duration();
		points.action = text.action = visualization_msgs::Marker::ADD;
		points.pose.orientation.w = text.pose.orientation.w = 1.0;


		points.id = 0;
		text.id = 1;


		points.type = visualization_msgs::Marker::CUBE_LIST;
		text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;


		// POINTS markers use x and y scale for width/height respectively
		points.scale.x = 0.15;
		points.scale.y = 0.15;
		points.scale.z = 0.15;

		// LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
		text.scale.z = 0.1;



		// Points are green
		points.color.g = 1.0f;
		points.color.a = 1.0;

		// Line list is red
		text.color.r = 1.0;
   	 	text.color.a = 1.0;


		geometry_msgs::Point p;
		p.x = 0;
		p.y = 0.1;
		p.z = 0.5;
		text.points.push_back(p);


		chem_val = myNostril_0.reading;
		//ROS_INFO("val:%f", chem_val);
		chem_val = (100.0/3000.0) * chem_val;
		//ROS_INFO("norm_val:%f", chem_val);
		makeColorBar(&points, (int)(chem_val));	

		sprintf(currValue,"%d%%", (int)chem_val);
		//ROS_INFO("currValue:%s", currValue);
		text.text.clear();
		text.text = currValue;
	 	marker_pub.publish(text);

		marker_pub.publish(points);

   
		ros::spinOnce();
		r.sleep();
	}

	ROS_INFO("That's enough!!!");
}





