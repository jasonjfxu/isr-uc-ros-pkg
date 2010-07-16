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

#include <wx/wx.h>
#include <wx/event.h>
#include <wx/timer.h>

#include <wx/string.h>
#include <wx/bitmap.h>
#include <wx/image.h>
#include <wx/icon.h>
#include <wx/menu.h>
#include <wx/gdicmn.h>
#include <wx/font.h>
#include <wx/colour.h>
#include <wx/settings.h>
#include <wx/stattext.h>
#include <wx/gauge.h>
#include <wx/sizer.h>
#include <wx/frame.h>




#include <ros/ros.h>

#include <std_srvs/Empty.h>
#include <roomba_info/Spawn.h>
#include <roomba_info/Kill.h>
#include <map>

#include "roomba.h"

namespace roomba_info
{

class InfoFrame : public wxFrame
{
public:
  InfoFrame(wxWindow* parent);
  ~InfoFrame();


private:
  void onUpdate(wxTimerEvent& evt);

  void updateData();
  void clear();

  

  //bool spawnCallback(turtlesim::Spawn::Request&, turtlesim::Spawn::Response&);
  //bool killCallback(turtlesim::Kill::Request&, turtlesim::Kill::Response&);

  ros::NodeHandle nh_;
  wxTimer* update_timer_;
  
  wxMenuBar* menubar;
  wxMenu* file_menu;
  wxStaticText* bat_staticText;
  wxGauge* bat_gauge;
  wxStaticText* charge_staticText;
  wxStaticText* state_staticText;

  roomba_info::Roomba::Roomba* roomba_obj;
  ros::WallTime last_info_update_;


  ros::ServiceServer spawn_srv_;
  ros::ServiceServer kill_srv_;

  typedef std::map<std::string, RoombaPtr> M_Roomba;
  M_Roomba roomba_;
  uint32_t id_counter_;

  wxImage turtle_images_[3];

};

}
