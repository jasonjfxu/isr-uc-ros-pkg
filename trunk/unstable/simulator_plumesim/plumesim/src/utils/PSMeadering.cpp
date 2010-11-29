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
* Author: Gonçalo Cabrita and Pedro Sousa on 02/15/2010
*********************************************************************/
#include <string.h>
#include <stdio.h>
#include <math.h>

#include "PSSources.h"

// *****************************************************************************
// Constructor.
plumesim::PSMeadering::PSMeadering()
{
	this->changesOverTime = false;
	
	this->startPoint.px = 0.0;
	this->startPoint.py = 0.0;
	this->startPoint.pz = 0.0;
	
	this->arenaRect.startPoint.px = 0.0;
	this->arenaRect.startPoint.py = 0.0;
	this->arenaRect.endPoint.px = 0.0;
	this->arenaRect.endPoint.py = 0.0;
	
	this->numOfPlumePoints = 200;
	this->releaseRate = 10.0;
	this->dispersionCoeficient = 0.1;
	this->radius = 0.1;
}

// *****************************************************************************
// Destructor.
plumesim::PSMeadering::~PSMeadering()
{
	// Clean up...
}

// *****************************************************************************
// Setup routine.
int plumesim::PSMeadering::setup()
{
	if(!plumePoints.empty()) plumePoints.clear();
	this->isPlaying = true;
	return 0;
}

// *****************************************************************************
// Cleanup routine.
int plumesim::PSMeadering::cleanup()
{
	plumePoints.clear();
	return 0;
}

// *****************************************************************************
// Generate points.
int plumesim::PSMeadering::generatePoints()
{
	int i;
	
	if(!plumePoints.empty()) plumePoints.clear();

	PSPoint3d startupPoints[MIN_NUM_OF_POINTS];
	
	double step = (arenaRect.endPoint.px - startPoint.px)/MIN_NUM_OF_POINTS;
	// Fill the startup array
	for(i=0 ; i<MIN_NUM_OF_POINTS ; i++) startupPoints[i].px = startPoint.px + i * step;
	
	double sigma = 0.2*step;
	// Posicao do centro da pluma na origem
	startupPoints[0].py = startPoint.py;
	
	double ay[MIN_NUM_OF_POINTS];
	// Fill the array
	for(i=0 ; i<MIN_NUM_OF_POINTS ; i++) ay[i] = sigma*randomNormal();
	
	double vy[MIN_NUM_OF_POINTS];
	// Velocidade lateral da pluma na origem
	vy[0] = sigma*randomNormal();
	
	for(i=1 ; i<MIN_NUM_OF_POINTS ; i++)
	{
		// Perturbacao da velocidade lateral
		vy[i] = vy[i-1] + ay[i-1];
		// Actualizacao do centro da pluma
		startupPoints[i].py = startupPoints[i-1].py + vy[i-1];
	}
	
	linearInterpolation(startupPoints, MIN_NUM_OF_POINTS);
	
	PSPoint3d * plumePoint;
	for(i=0 ; i<plumePoints.size() ; i++)
	{
		plumePoint = &plumePoints[i];
		// TODO: test for other release points
		plumePoint->py = plumePoint->py + ( dispersionCoeficient * (plumePoint->px - startPoint.px) * (releaseRate * randomNormal()));
	}
	
	PSPoint3d center;
	center.px = startPoint.px + radius;
	center.py = startPoint.py;
	center.pz = 0.0;
	
	maxPoints = this->countPoints(&center, radius);

	this->isPlaying = false;
	
	return(0);
}

// *****************************************************************************
// Sends the chemical reading for a given position.
int plumesim::PSMeadering::getChemicalReading(PSPoint3d * point, PSOdorData * odor_data)
{	
	int pointCount = this->countPoints(point, radius);
	
	odor_data->chemical = fmin(100.0, 100.0*pointCount/maxPoints);
	// Constants
	// TODO: Maybe get this from the .cfg file
	odor_data->windDirection = 0.0;		// rad
	odor_data->windSpeed = 0.5;			// m/s
	
	return(0);
}

// *****************************************************************************
// Linear interpolation
void plumesim::PSMeadering::linearInterpolation(PSPoint3d * points, int numOfPoints)
{
	double b, m;
	int i, j;
	PSPoint3d point;

	int pointsOnThisSection;
	
	double x_step = (points[numOfPoints-1].px - points[0].px) / numOfPlumePoints;
	
	for(i=0 ; i<numOfPoints-1 ; i++)
	{
		point.px = points[i].px;
		point.py = points[i].py;
		point.pz = 0.0;
		plumePoints.push_back(point);
		
		m = (points[i+1].py - points[i].py)/(points[i+1].px - points[i].px);
		b = points[i].py - m * points[i].px;
		
		pointsOnThisSection = (points[i+1].px - points[i].px) / x_step;
		
		for(j=0 ; j<pointsOnThisSection ; j++)
		{
			point.px = points[i].px + (j+1)*x_step;
			point.py = m * point.px + b;
			point.pz = 0.0;
			plumePoints.push_back(point);
		}
	}
	point.px = points[numOfPoints-1].px;
	point.py = points[numOfPoints-1].py;
	point.pz = 0.0;
	plumePoints.push_back(point);
}

// EOF

