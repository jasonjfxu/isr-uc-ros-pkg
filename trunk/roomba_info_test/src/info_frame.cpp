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

#include "roomba_info/info_frame.h"

#include <ros/package.h>
#include <cstdlib>
#include <ctime>


namespace roomba_info
{

InfoFrame::InfoFrame(wxWindow* parent)
: wxFrame(parent, wxID_ANY, wxT("RoombaInfo"), wxDefaultPosition, wxSize(330, 140), wxDEFAULT_FRAME_STYLE & ~wxRESIZE_BORDER)
{
	
	this->SetSizeHints( wxDefaultSize, wxDefaultSize );
	
	menubar = new wxMenuBar( 0 );
	file_menu = new wxMenu();
	wxMenuItem* exit_menuItem;
	exit_menuItem = new wxMenuItem( file_menu, wxID_ANY, wxString( wxT("Exit") ) , wxEmptyString, wxITEM_NORMAL );
	file_menu->Append( exit_menuItem );
	
	menubar->Append( file_menu, wxT("File") );
	
	this->SetMenuBar( menubar );
	
	wxGridSizer* gSizer2;
	gSizer2 = new wxGridSizer( 3, 2, 0, 0 );
	
	bat_staticText = new wxStaticText( this, wxID_ANY, wxT("Battery Level"), wxDefaultPosition, wxDefaultSize, 0 );
	bat_staticText->Wrap( -1 );
	gSizer2->Add( bat_staticText, 0, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5 );
	
	bat_gauge = new wxGauge( this, wxID_ANY, 100, wxDefaultPosition, wxDefaultSize, wxGA_HORIZONTAL );
	bat_gauge->SetValue( 25 ); 
	gSizer2->Add( bat_gauge, 0, wxALL|wxALIGN_CENTER_VERTICAL|wxALIGN_CENTER_HORIZONTAL, 5 );
	
	charge_staticText = new wxStaticText( this, wxID_ANY, wxT("Charging State"), wxDefaultPosition, wxDefaultSize, 0 );
	charge_staticText->Wrap( -1 );
	gSizer2->Add( charge_staticText, 0, wxALL|wxALIGN_CENTER_VERTICAL|wxALIGN_CENTER_HORIZONTAL, 5 );
	
	state_staticText = new wxStaticText( this, wxID_ANY, wxT("state"), wxDefaultPosition, wxDefaultSize, 0 );
	state_staticText->Wrap( -1 );
	gSizer2->Add( state_staticText, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	this->SetSizer( gSizer2 );
	this->Layout();
	
	
  srand(time(NULL));

  update_timer_ = new wxTimer(this);
  update_timer_->Start(16);

  Connect(update_timer_->GetId(), wxEVT_TIMER, wxTimerEventHandler(InfoFrame::onUpdate), NULL, this);

  
//  roomba_info::Roomba* roomba_obj = new roomba_info::Roomba(nh_);
  roomba_obj = new Roomba(nh_);
 
  //clear();

  //spawn_srv_ = nh_.advertiseService("spawn", &TurtleFrame::spawnCallback, this);
  //kill_srv_ = nh_.advertiseService("kill", &TurtleFrame::killCallback, this);

  ROS_INFO("Starting roomba_info_app with node name %s", ros::this_node::getName().c_str()) ;

 }

InfoFrame::~InfoFrame()
{
  delete update_timer_;
}

/*
bool InfoFrame::spawnCallback(turtlesim::Spawn::Request& req, turtlesim::Spawn::Response& res)
{
  std::string name = spawnTurtle(req.name, req.x, req.y, req.theta);
  if (name.empty())
  {
    ROS_ERROR("A turtled named [%s] already exists", req.name.c_str());
    return false;
  }

  res.name = name;

  return true;
}

bool InfoFrame::killCallback(turtlesim::Kill::Request& req, turtlesim::Kill::Response&)
{
  M_Turtle::iterator it = turtles_.find(req.name);
  if (it == turtles_.end())
  {
    ROS_ERROR("Tried to kill turtle [%s], which does not exist", req.name.c_str());
    return false;
  }

  turtles_.erase(it);

  return true;
}
*/

void InfoFrame::clear()
{
  // TODO
  // clear the gauge and the shown value
}

void InfoFrame::onUpdate(wxTimerEvent& evt)
{
  ros::spinOnce();

  updateData();

  if (!ros::ok())
  {
    Close();
  }
}



void InfoFrame::updateData()
{
  if (last_info_update_.isZero())
  {
    last_info_update_ = ros::WallTime::now();
    return;
  }
  
  int test_val = roomba_obj->bat_value_;
  int test_state = roomba_obj->bat_state_;
  std:: string stest_state;
  
  //strcpy(stest_state, roomba_info->bat_state_);
  
  //int test_remain = roomba_info->bat_remain_;
   
  
  ROS_INFO("This is just a test!%d  %d/n",test_val, test_state);
  
   
  if(test_val >= 0 && test_val <= 100)
	bat_gauge->SetValue( test_val ); 
  
  
  // string variable
  std::string test = "Not Charging";
    
  switch( test_state )
  {
	  case 0:
		//	0	Not Charging		
		//state_staticText->SetLabel(wxT("Not Charging"));		
		state_staticText->SetLabel(wxString(test.c_str(), wxConvUTF8));
		break;
		
	  case 1:
		//	1	Reconditioning Charging
		state_staticText->SetLabel(wxT("Reconditioning Charging"));
		break;
	  case 2:		
		//	2	Full Charging
		state_staticText->SetLabel(wxT("Full Charging"));
		break;
	  case 3:
		//	3	Trickle Charging
		state_staticText->SetLabel(wxT("Trickle Charging"));
		break;
	  case 4:
		//	4	Waiting
		state_staticText->SetLabel(wxT("Waiting"));
		break;
	  case 5:
		//	5	Charging Fault Condition
		state_staticText->SetLabel(wxT("Charging Fault Condition"));
		break;
	  default:
		// goes here
state_staticText->SetLabel(wxT("state"));
		break;
  }  
		
	  
  //TODO
  // update the values here

}

}
