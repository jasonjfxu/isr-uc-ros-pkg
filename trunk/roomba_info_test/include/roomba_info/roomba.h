/*
 * Copyright (c) 2009, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef ROOMBA_APP_ROOMBA_H
#define ROOMBA_APP_ROOMBA_H

#include <ros/ros.h>
#include <boost/shared_ptr.hpp>

#include <roomba_info/BatState.h>

#include <wx/wx.h>

#define PI 3.14159265


// Roomba battery in mV - between 0 and 65535
// Roomba charging state - between 0 and 5
//	0	Not Charging
//	1	Reconditioning Charging
//	2	Full Charging
//	3	Trickle Charging
//	4	Waiting
//	5	Charging Fault Condition


namespace roomba_info
{

struct Vector2
{
  Vector2()
  : x(0.0)
  , y(0.0)
  {}

  Vector2(float _x, float _y)
  : x(_x)
  , y(_y)
  {}

  bool operator==(const Vector2& rhs)
  {
    return x == rhs.x && y == rhs.y;
  }

  bool operator!=(const Vector2& rhs)
  {
    return x != rhs.x || y != rhs.y;
  }

  float x;
  float y;
};

class Roomba
{
public:
  Roomba(const ros::NodeHandle& nh);

  int bat_value_;
  int bat_state_;
  std::string bat_state_;
  int bat_remain_;  
  
private:
  //void velocityCallback(const VelocityConstPtr& vel);
 
  void batteryCallback(const BatStateConstPtr& bat);
  
  ros::NodeHandle nh_;



  ros::Subscriber battery_sub_;


  ros::WallTime last_command_time_;


  struct TeleportRequest
  {
    TeleportRequest(float x, float y, float _theta, float _linear, bool _relative)
    : pos(x, y)
    , theta(_theta)
    , linear(_linear)
    , relative(_relative)
    {}

    Vector2 pos;
    float theta;
    float linear;
    bool relative;
  };
  typedef std::vector<TeleportRequest> V_TeleportRequest;
  V_TeleportRequest teleport_requests_;
};
typedef boost::shared_ptr<Roomba> RoombaPtr;

}

#endif
