/*
 *  PSGaussian.cpp
 *  
 *
 *  Created by Gonçalo Cabrita and Pedro Sousa on 03/01/2010.
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
 * \file PSGaussian.cpp
 *
 * \brief This file implements the class PSGaussian.  This class contains the gaussian plume generation algorithm.
 *
 * \author Gonçalo Cabrita
 * \author Pedro Sousa
 * \date 03/01/2010
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
PSGaussian::PSGaussian()
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
PSGaussian::~PSGaussian()
{
	// Clean up...
}

// *****************************************************************************
// Setup routine.
int PSGaussian::setup()
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
int PSGaussian::cleanup()
{
	plumePoints.clear();
	frame.clear();
	return 0;
}

// *****************************************************************************
// Generate points.
int PSGaussian::generatePoints()
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
int PSGaussian::getChemicalReading(PSPoint3d * point, PSOdorData * odor_data)
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

