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
#include <cstdlib>

#include "PSSources.h"

// *****************************************************************************
// Constructor.
plumesim::PSSource::PSSource()
{
	// Nothing...
}

// *****************************************************************************
// Destructor.
plumesim::PSSource::~PSSource()
{
	// Clean up...
}

// *****************************************************************************
// Setup routine.
int plumesim::PSSource::setup()
{
	return 0;
}

// *****************************************************************************
// Cleanup routine.
int plumesim::PSSource::cleanup()
{
	return 0;
}

// *****************************************************************************
// Generate points.
int plumesim::PSSource::generatePoints()
{
	return 0;
}

// *****************************************************************************
// Sends the chemical reading for a given position.
int plumesim::PSSource::getChemicalReading(PSPoint3d * point, PSOdorData * odor_data)
{
	return 0;
}

// *****************************************************************************
// If the class is time invariant or not.
bool plumesim::PSSource::ChangesOverTime()
{
	return changesOverTime;
}

// *****************************************************************************
// If the class is still processing data.
bool plumesim::PSSource::IsPlaying()
{
	return isPlaying;
}

// *****************************************************************************
// Uniform distribution, [0 ... 1]
float plumesim::PSSource::drand()
{
	return ((rand()+1.0)/(RAND_MAX+1.0));
}

// *****************************************************************************
// Normal distribution, centered on 0, std dev 1
double plumesim::PSSource::randomNormal()
{
	return (sqrt(-2*log(drand()))*cos(2*M_PI*drand()));
}

// *****************************************************************************
// Count the points in a sphere
int plumesim::PSSource::countPoints(PSPoint3d * center, double radius)
{	
	int pointCount = 0;
	for(int i=0 ; i<plumePoints.size() ; i++)
	{
		if(sqrt((center->px - plumePoints[i].px)*(center->px - plumePoints[i].px) + (center->py - plumePoints[i].py)*(center->py - plumePoints[i].py) + (center->pz - plumePoints[i].pz)*(center->pz - plumePoints[i].pz)) <= radius) pointCount++;
	}
	return pointCount;
}

// EOF

