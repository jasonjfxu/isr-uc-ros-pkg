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
* Author: Gonçalo Cabrita and Pedro Sousa on 03/01/2010
*********************************************************************/
#include <string.h>
#include <stdio.h>
#include <math.h>

#include "PSSources.h"

// *****************************************************************************
// Constructor.
plumesim::PSGaussian::PSGaussian()
{	
	this->changesOverTime = true;
	
	this->startPoint.px = 0.0;
	this->startPoint.py = 0.0;
	this->startPoint.pz = 0.0;
	
	this->arenaRect.startPoint.px = 0.0;
	this->arenaRect.startPoint.py = 0.0;
	this->arenaRect.endPoint.px = 0.0;
	this->arenaRect.endPoint.py = 0.0;
	
	this->cellSize = 0.1;
	this->diffX = 0.2;
	this->diffY = 0.005;
	this->radius = 0.1;
	this->maxPointsPerCell = 10;
}

// *****************************************************************************
// Destructor.
plumesim::PSGaussian::~PSGaussian()
{
	// Clean up...
}

// *****************************************************************************
// Setup routine.
int plumesim::PSGaussian::setup()
{
	int i, j;
	
	if(!plumePoints.empty()) plumePoints.clear();
	if(!frame.empty()) frame.clear();
	
	int numXpoints = (this->arenaRect.endPoint.px - this->arenaRect.startPoint.px)/this->cellSize;
	int numYpoints = (this->arenaRect.endPoint.py - this->arenaRect.startPoint.py)/this->cellSize;
	
	for(i=0 ; i<numXpoints ; i++)
	{
		for(j=0 ; j<numYpoints ; j++)
		{
			PSFramePoint frameCell;
			frameCell.point.px = startPoint.px + this->cellSize*i;
			frameCell.point.py = startPoint.py + this->cellSize*j;
			frameCell.point.pz = 0.0;
			
			frameCell.odor.chemical = 0.0;			// Start value
			frameCell.odor.windDirection = 0.0;		// rad
			frameCell.odor.windSpeed = 0.5;			// m/s
			
			this->frame.push_back(frameCell);
		}
	}
	t = 1;
	this->isPlaying = true;
	
	return 0;
}

// *****************************************************************************
// Cleanup routine.
int plumesim::PSGaussian::cleanup()
{
	plumePoints.clear();
	frame.clear();
	return 0;
}

// *****************************************************************************
// Generate points.
int plumesim::PSGaussian::generatePoints()
{
	PSFramePoint * frameCell;
	int pointsCount;
	int j;
	
	if(!plumePoints.empty()) plumePoints.clear();
	
	for(int i=0 ; i<this->frame.size() ; i++)
	{
		frameCell = &this->frame[i];
		
		frameCell->odor.chemical = 100.0/(4*M_PI*sqrt(this->diffX)*sqrt(this->diffY)*t)*exp(-(((frameCell->point.px - this->startPoint.px)*(frameCell->point.px - this->startPoint.px)/this->diffX)+((frameCell->point.py - this->startPoint.py)*(frameCell->point.py - this->startPoint.py)/this->diffY))/(4*t));

		pointsCount = round(maxPointsPerCell * frameCell->odor.chemical/100.0);
		for(j=0 ; j<pointsCount ; j++)
		{
			PSPoint3d point;
			point.px = frameCell->point.px + cellSize * randomNormal();
			point.py = frameCell->point.py + cellSize * randomNormal();
			// TODO: Complete for full 3D support!
			point.pz = 0.0;
			this->plumePoints.push_back(point);
		}
	}
	t++;
	if(t > 25) t = 20;
	//isPlaying = false; 
	
	return(0);
}

// *****************************************************************************
// Sends the chemical reading for a given position.
int plumesim::PSGaussian::getChemicalReading(PSPoint3d * point, PSOdorData * odor_data)
{	
	double distance;
	double minDistance = cellSize;
	
	odor_data->chemical = 0.0;
	odor_data->windSpeed = 0.0;
	odor_data->windDirection = 0.0;
	
	PSFramePoint * framePoint;
	for(int i=0 ; i<this->frame.size() ; i++)
	{
		framePoint = &this->frame[i];
		
		distance = sqrt((point->px - framePoint->point.px)*(point->px - framePoint->point.px) + (point->py - framePoint->point.py)*(point->py - framePoint->point.py) + (point->pz - framePoint->point.pz)*(point->pz - framePoint->point.pz));
		if(distance < minDistance)
		{
			odor_data->chemical = framePoint->odor.chemical;
			odor_data->windSpeed = framePoint->odor.windSpeed;
			odor_data->windDirection = framePoint->odor.windDirection;
			
			minDistance = distance;
		}
	}
	return(0);
}

// EOF

