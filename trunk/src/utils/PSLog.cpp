/*
 *  PSLog.cpp
 *  
 *
 *  Created by Gonçalo Cabrita and Pedro Sousa on 02/23/2010.
 *  Copyright 2010 ISR. All rights reserved.
 *
 *	Comments:
 *	No comments.
 *
 */

/*
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

/*!
 *
 * \file PSLog.cpp
 *
 * \brief This file implements the class PSLog. This class contains the PlumeSim log file reading code.
 *
 * \author Gonçalo Cabrita
 * \author Pedro Sousa
 * \date 02/23/2010
 * \version 1.2
 *
 * \bug none discovered
 *
 * \note Updated by Gonçalo Cabrita 01/06/2010
 *
 */

#include <string.h>
#include <stdio.h>
#include <math.h>

#include "PSSources.h"

// *****************************************************************************
// Constructor.
PSLog::PSLog()
{
	this->maxPointsPerCell = 10;
	this->logFilePath = "log.txt";
}

// *****************************************************************************
// Destructor.
PSLog::~PSLog()
{
	// Clean up...
}

// *****************************************************************************
// Setup routine.
int PSLog::setup()
{
	if(!plumePoints.empty()) plumePoints.clear();
	if(!frame.empty()) frame.clear();
	
	// Open the file
	this->logFile = fopen(logFilePath.c_str(), "r");
	// Take out the settings...
	// period ppFrame nFrames cellSize maxOdor
	if( fscanf(this->logFile, "%d %d %lf\n", &this->period, &this->numOfFrames, &this->cellSize) == 0)
	{
		return(-1);
	}
	
	if(this->numOfFrames > 1) this->changesOverTime = true;
	else this->changesOverTime = false;
	this->isPlaying = true;
	
	return(0);
}

// *****************************************************************************
// Cleanup routine.
int PSLog::cleanup()
{
	// Close the file
	fclose(logFile);
	// Clean up...
	plumePoints.clear();
	frame.clear();
	return 0;
}

// *****************************************************************************
// Read points from the log file.
int PSLog::generatePoints()
{
	int i, j;
	int pointsCount;
	int pointsInFrame;
	
	if( !feof(logFile) )
	{
		if(!plumePoints.empty()) plumePoints.clear();
		if(!frame.empty()) frame.clear();

		// +t
		fscanf(logFile, "+%d %d\n", &currentFrame, &pointsInFrame);
		
		for(i=0 ; i<pointsInFrame ; i++)
		{
			PSFramePoint pointFromFrame;
			// x y z windSpeed windDirection chemical
			//fscanf(logFile, "%lf,%lf,%lf,%lf,%lf,%lf\n", &pointFromFrame.point.px, &pointFromFrame.point.py, &pointFromFrame.point.pz,  &pointFromFrame.odor.windSpeed, &pointFromFrame.odor.windDirection, &pointFromFrame.odor.chemical);
			fscanf(logFile, "%lf %lf %lf\n", &pointFromFrame.point.px, &pointFromFrame.point.py, &pointFromFrame.odor.chemical);
			
			this->frame.push_back(pointFromFrame);
			
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
	}
	else
	{
		isPlaying = false;
	}
	return(0);
}

// *****************************************************************************
// Sends the chemical reading for a given position.
int PSLog::getChemicalReading(PSPoint3d * point, PSOdorData * odor_data)
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

