/*
 *  PSSource.cpp
 *  
 *
 *  Created by Gonçalo Cabrita and Pedro Sousa on 02/15/2010.
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
 * \file PSSource.cpp
 *
 * \brief This file contains the base class for a plume generation source class. All plume source classes extend PSSource.
 *
 * \author Gonçalo Cabrita
 * \author Pedro Sousa
 * \date 02/15/2010
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
#include <cstdlib>

#include "PSSources.h"

// *****************************************************************************
// Constructor.
PSSource::PSSource()
{
	// Nothing...
}

// *****************************************************************************
// Destructor.
PSSource::~PSSource()
{
	// Clean up...
}

// *****************************************************************************
// Setup routine.
int PSSource::setup()
{
	return 0;
}

// *****************************************************************************
// Cleanup routine.
int PSSource::cleanup()
{
	return 0;
}

// *****************************************************************************
// Generate points.
int PSSource::generatePoints()
{
	return 0;
}

// *****************************************************************************
// Sends the chemical reading for a given position.
int PSSource::getChemicalReading(PSPoint3d * point, PSOdorData * odor_data)
{
	return 0;
}

// *****************************************************************************
// If the class is time invariant or not.
bool PSSource::ChangesOverTime()
{
	return changesOverTime;
}

// *****************************************************************************
// If the class is still processing data.
bool PSSource::IsPlaying()
{
	return isPlaying;
}

// *****************************************************************************
// Uniform distribution, [0 ... 1]
float PSSource::drand()
{
	return ((rand()+1.0)/(RAND_MAX+1.0));
}

// *****************************************************************************
// Normal distribution, centered on 0, std dev 1
double PSSource::randomNormal()
{
	return (sqrt(-2*log(drand()))*cos(2*M_PI*drand()));
}

// *****************************************************************************
// Count the points in a sphere
int PSSource::countPoints(PSPoint3d * center, double radius)
{	
	int pointCount = 0;
	for(int i=0 ; i<plumePoints.size() ; i++)
	{
		if(sqrt((center->px - plumePoints[i].px)*(center->px - plumePoints[i].px) + (center->py - plumePoints[i].py)*(center->py - plumePoints[i].py) + (center->pz - plumePoints[i].pz)*(center->pz - plumePoints[i].pz)) <= radius) pointCount++;
	}
	return pointCount;
}

// EOF

