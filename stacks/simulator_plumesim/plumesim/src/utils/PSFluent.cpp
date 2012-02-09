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
* Author: Gonçalo Cabrita and Pedro Sousa on 03/03/2010
*********************************************************************/
#include <string.h>
#include <stdio.h>
#include <math.h>

#include "PSSources.h"

// *****************************************************************************
// Constructor.
plumesim::PSFluent::PSFluent()
{
	this->maxPointsPerCell = 10;
	this->fileName = "fluentlog";
	this->numOfFrames = 1;
	this->cellSize = 0.1;
	this->maxOdorValue = 1.0;
}

// *****************************************************************************
// Destructor.
plumesim::PSFluent::~PSFluent()
{
	// Clean up...
}

// *****************************************************************************
// Setup routine.
int plumesim::PSFluent::setup()
{
	if(!plumePoints.empty()) plumePoints.clear();
	if(!frame.empty()) frame.clear();
	
	if(this->numOfFrames > 1) this->changesOverTime = true;
	else this->changesOverTime = false;
	
	this->isPlaying = true;
	
	this->currentFrame = 1;
	
	return 0;
}

// *****************************************************************************
// Cleanup routine.
int plumesim::PSFluent::cleanup()
{
	plumePoints.clear();
	frame.clear();
	return 0;
}

// *****************************************************************************
// Read points from the log file.
int plumesim::PSFluent::generatePoints()
{
	if(currentFrame > numOfFrames)
	{
		isPlaying = false;
		return(0);
	}
	
	int i, j;
	int pointsCount;
	
	if(!plumePoints.empty()) plumePoints.clear();
	if(!frame.empty()) frame.clear();
	
	char logFileName[128];
	char logFileNum[8];
	strcpy(logFileName, fileName.c_str());
	sprintf(logFileNum, "%03d", currentFrame);
	strncat(logFileName, logFileNum, strlen(logFileNum));
	// Open the file
	logFile = fopen(logFileName, "r");
	if(logFile == NULL) return(-1);
	// Ignore the first line
	char dummy[64];
	fgets(dummy, 64, logFile);
	
	while( !feof(logFile) )
	{
		PSFramePoint pointFromFrame;
		pointFromFrame.point.pz = 0.0;
		// index x y chemical
		fscanf(logFile, "%d,%lf,%lf,%lf\n", &i, &pointFromFrame.point.px, &pointFromFrame.point.py, &pointFromFrame.odor.chemical);
		pointFromFrame.odor.chemical = 100.0*pointFromFrame.odor.chemical/maxOdorValue;
		
		frame.push_back(pointFromFrame);
			
		pointsCount = round(maxPointsPerCell * pointFromFrame.odor.chemical/100.0);
		for(j=0 ; j<pointsCount ; j++)
		{
			PSPoint3d point;
			point.px = pointFromFrame.point.px + cellSize * randomNormal();
			point.py = pointFromFrame.point.py + cellSize * randomNormal();
			// TODO: Complete for full 3D support!
			point.pz = 0.0;
			plumePoints.push_back(point);
		}
	}
	currentFrame++;
	
	// Close the file
	fclose(logFile);
	
	return(0);
}

// *****************************************************************************
// Sends the chemical reading for a given position.
int plumesim::PSFluent::getChemicalReading(PSPoint3d * point, PSOdorData * odor_data)
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

