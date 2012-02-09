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
#include <vector>
#include <string>

#define MIN_NUM_OF_POINTS	20

namespace plumesim
{
	/*!
	 *
	 * \struct PSPoint2d
	 * It defines a 2D point.
	 * It already defines an object.
	 *
	 */
	typedef struct
	{
		double px;
		double py;
	
	} PSPoint2d;

	/*!
	 *
	 * \struct PSPoint3d
	 * It defines a 3D point.
	 * It already defines an object.
	 *
	 */
	typedef struct
	{
		double px;
		double py;
		double pz;

	} PSPoint3d;

	/*!
	 *
	 * \struct PSRect
	 * It defines a rectangle from a 2D starting point and a 2D ending point.
	 * It already defines an object.
	 *
	 */
	typedef struct
	{
		PSPoint2d startPoint;
		PSPoint2d endPoint;
	
	} PSRect;

	/*!
	 *
	 * \struct PSOdorData
	 * It includes a chemical value as well as the wind speed and direction.
	 * It already defines an object.
	 *
	 */
	typedef struct
	{
		double chemical;		// ??
		double windSpeed;		// m/s
		double windDirection;	// rad
	
	} PSOdorData;

	/*!
	 *
	 * \struct PSFramePoint
	 * Plume point containing the position and odor data.
	 * It already defines an object.
	 *
	 */
	typedef struct
	{
		PSPoint3d point;
		PSOdorData odor;
	
	} PSFramePoint;


	/*!
	 * 
	 * \class PSSource PSSource.h
	 * \brief Base class for all the plume source algorithms.
	 *
	 */
	class PSSource
	{
		public:
		
		//! PSSource constructor.
		/*!
		 * This is the class constructor.
		 */
		PSSource();
		//! PSSource destructor.
		/*!
		 * This is the class destructor.
		 */
		~PSSource();
	
		/*!
		 *
		 * \fn virtual int setup()
		 * \brief Virtual function for setting up the class.
		 * \param none.
		 * \return none.
		 *
		 */
		virtual int setup();
		/*!
		 *
		 * \fn virtual int cleanup()
		 * \brief Virtual function for cleaning up the class.
		 * \param none.
		 * \return none.
		 *
		 */
		virtual int cleanup();
	
		std::vector<PSPoint3d> plumePoints;		/**< Linked list of plume points to draw.*/
	
		/*!
		 *
		 * \fn virtual int generatePoints()
		 * \brief Virtual function for generating plume points.
		 * \param none.
		 * \return 0 - if successful
		 * \return -1 - otherwise
		 *
		 */
		virtual int generatePoints();
		/*!
		 *
		 * \fn virtual int getChemicalReading(PSPoint3d * point, PSOdorData * odor_data)
		 * \brief Virtual function for reading a single chemical point based on the robot current position.
		 * \param point A 3D point containning the current position of the robot.
		 * \param odor_data The odor data structure where the result is written.
		 * \return 0 - if successful
		 * \return -1 - otherwise
		 *
		 */
		virtual int getChemicalReading(PSPoint3d * point, PSOdorData * odor_data);
		/*!
		 *
		 * \fn bool ChangesOverTime()
		 * \brief If the class is time invariant or not.
		 * \param none.
		 * \return true - if the plume changes over time
		 * \return false - otherwise
		 *
		 */
		bool ChangesOverTime();
		/*!
		 *
		 * \fn bool IsPlaying()
		 * \brief If the class is still processing data.
		 * \param none.
		 * \return true - if the plume is still playing
		 * \return false - otherwise
		 *
		 */
		bool IsPlaying();
		
		protected:
	
		/*!
		 *
		 * \fn float drand()
		 * \brief Uniform distribution, [0 ... 1]
		 * \param none.
		 * \return Random generated value.
		 *
		 */
		float drand();
		/*!
		 *
		 * \fn double randomNormal()
		 * \brief Normal distribution, centered on 0, std dev 1
		 * \param none.
		 * \return Random generated value.
		 *
		 */
		double randomNormal();
		/*!
		 *
		 * \fn int countPoints(PSPoint3d * center, double radius)
		 * \brief Counts the plume points inside a sphere
		 * \param center The center of the sphere.
		 * \param radius The radius of the sphere.
		 * \return The number of points found.
		 *
		 */
		int countPoints(PSPoint3d * center, double radius);
		
		bool changesOverTime;		/**< Stores if the plume changes over time or not.*/
		bool isPlaying;				/**< Stores if the plume is playing ot not.*/
	};


	/*!
	 * 
	 * \class PSMeadering PSSource.h
	 * \brief Extends the PSSource class to provide plume generation based on the meandering algorithm.
	 *
	 */
	class PSMeadering:public PSSource
	{
		public:
		
		/*!
		 *
		 * \fn PSMeadering()
		 * \brief PSMeadering constructor. This is the class constructor.
		 *
		 */
		PSMeadering();
		//! PSMeadering destructor.
		/*!
		 * This is the class destructor.
		 */
		~PSMeadering();
	
		/*!
		 *
		 * \fn int setup()
		 * \brief Function for setting up the class.
		 * \param none.
		 * \return none.
		 *
		 */
		int setup();
		/*!
		 *
		 * \fn int cleanup()
		 * \brief Function for cleaning up the class.
		 * \param none.
		 * \return none.
		 *
		 */
		int cleanup();
	
		/*!
		 *
		 * \fn int generatePoints()
		 * \brief Function for generating plume points based on the meandering algorithm.
		 * \param none.
		 * \return 0 - if successful
		 * \return -1 - otherwise
		 *
		 */
		int generatePoints();
		/*!
		 *
		 * \fn int getChemicalReading(player_point_3d_t * point, odor_data_t * odor_data)
		 * \brief Function for reading a single chemical point based on the robot current position.
		 * \param point A 3D point containning the current position of the robot.
		 * \param odor_data The odor data structure where the result is written.
		 * \return 0 - if successful
		 * \return -1 - otherwise
		 *
		 */
		int getChemicalReading(PSPoint3d * point, PSOdorData * odor_data);
		
		// Preferences
		PSPoint3d startPoint;				/**< Odor release point.*/
		PSRect arenaRect;					/**< Rectangle containing the environment.*/
		double releaseRate;					/**< Release rate.*/
		double dispersionCoeficient;		/**< Dispersion coeficient.*/
		double radius;						/**< Radius of the odor reading sphere.*/
		int maxPoints;						/**< Maximum number of points to be drawn for each point.*/
		int numOfPlumePoints;				/**< Total number of plume points.*/
	
		private:
	
		/*!
		 *
		 * \fn void linearInterpolation(PSPoint3d * points, int numOfPoints)
		 * \brief Linear interpolation
		 * \param points Pointer to array of points to interpolate.
		 * \param numOfPoints Number of points in the array.
		 * \param newPoints Pointer to linked list of points to write the result to.
		 * \return none.
		 *
		 */
		void linearInterpolation(PSPoint3d * points, int numOfPoints);
	};


	/*!
	 * 
	 * \class PSGaussian PSSource.h
	 * \brief Extends the PSSource class to provide plume generation based on the gaussian algorithm.
	 *
	 */
	class PSGaussian:public PSSource
	{
		public:
	
		/*!
		 *
		 * \fn PSGaussian()
		 * \brief PSGaussian constructor. This is the class constructor.
		 *
		 */
		PSGaussian();
		//! PSGaussian destructor.
		/*!
		 * This is the class destructor.
		 */
		~PSGaussian();
	
		/*!
		 *
		 * \fn int setup()
		 * \brief Function for setting up the class.
		 * \param none.
		 * \return none.
		 *
		 */
		int setup();
		/*!
		 *
		 * \fn int cleanup()
		 * \brief Function for cleaning up the class.
		 * \param none.
		 * \return none.
		 *
		 */
		int cleanup();
	
		/*!
		 *
		 * \fn int generatePoints()
		 * \brief Function for generating plume points based on the gaussian algorithm.
		 * \param none.
		 * \return 0 - if successful
		 * \return -1 - otherwise
		 *
		 */
		int generatePoints();
		/*!
		 *
		 * \fn int getChemicalReading(PSPoint3d * point, PSOdorData * odor_data)
		 * \brief Function for reading a single chemical point based on the robot current position.
		 * \param point A 3D point containning the current position of the robot.
		 * \param odor_data The odor data structure where the result is written.
		 * \return 0 - if successful
		 * \return -1 - otherwise
		 *
		 */
		int getChemicalReading(PSPoint3d * point, PSOdorData * odor_data);
	
		// Preferences
		PSPoint3d startPoint;				/**< Odor release point.*/
		PSRect arenaRect;					/**< Rectangle containing the environment.*/
		double radius;						/**< Radius of the odor reading sphere.*/
		double cellSize;					/**< Cell size of the grid in meters.*/
		int maxPointsPerCell;				/**< Maximum number of points to be drawn for each cell.*/
	
		double diffX;						/**< Diffusivity constant in x.*/
		double diffY;						/**< Diffusivity constant in y.*/
	
		private:
	
		int t;								/**< Current time frame.*/
	
		std::vector<PSFramePoint> frame;	/**< Current frame.*/
	};


	/*!
	 * 
	 * \class PSLog PSSource.h
	 * \brief Extends the PSSource class to provide plume generation based on .pslog files.
	 *
	 */
	class PSLog:public PSSource
	{
		public:
		
		/*!
		 *
		 * \fn PSLog()
		 * \brief PSLog constructor. This is the class constructor.
		 *
		 */
		PSLog();
		//! PSLog destructor.
		/*!
		 * This is the class destructor.
		 */
		~PSLog();
	
		/*!
		 *
		 * \fn int setup()
		 * \brief Function for setting up the class.
		 * \param none.
		 * \return none.
		 *
		 */
		int setup();
		/*!
		 *
		 * \fn int cleanup()
		 * \brief Function for cleaning up the class.
		 * \param none.
		 * \return none.
		 *
		 */
		int cleanup();
	
		/*!
		 *
		 * \fn int generatePoints()
		 * \brief Function for generating plume points based on a .pslog file.
		 * \param none.
		 * \return 0 - if successful
		 * \return -1 - otherwise
		 *
		 */
		int generatePoints();
		/*!
		 *
		 * \fn int getChemicalReading(PSPoint3d * point, PSOdorData * odor_data)
		 * \brief Function for reading a single chemical point based on the robot current position.
		 * \param point A 3D point containning the current position of the robot.
		 * \param odor_data The odor data structure where the result is written.
		 * \return 0 - if successful
		 * \return -1 - otherwise
		 *
		 */
		int getChemicalReading(PSPoint3d * point, PSOdorData * odor_data);
		
		// Preferences
		FILE * logFile;				/**< Pointer to the log file.*/
		std::string logFilePath;		/**< Path to the log file.*/
	
		int period;				/**< Original period in ms.*/
		int numOfFrames;			/**< Number of frames.*/
		int currentFrame;			/**< Current index.*/
		double cellSize;			/**< Cell size of the grid in meters.*/
		int maxPointsPerCell;			/**< Maximum number of points to be drawn for each cell.*/
	
		private:
	
		std::vector<PSFramePoint> frame;	/**< Current frame.*/
	};


	/*!
	 * 
	 * \class PSFluent PSSource.h
	 * \brief Extends the PSSource class to provide plume generation based on Fluent log files.
	 *
	 */
	class PSFluent:public PSSource
	{
		public:
	
		/*!
		 *
		 * \fn PSFluent()
		 * \brief PSFluent constructor. This is the class constructor.
		 *
		 */
		PSFluent();
		//! PSFluent destructor.
		/*!
		 * This is the class destructor.
		 */
		~PSFluent();
	
		/*!
		 *
		 * \fn int setup()
		 * \brief Function for setting up the class.
		 * \param none.
		 * \return none.
		 *
		 */
		int setup();
		/*!
		 *
		 * \fn int cleanup()
		 * \brief Function for cleaning up the class.
		 * \param none.
		 * \return none.
		 *
		 */
		int cleanup();
	
		/*!
		 *
		 * \fn int generatePoints()
		 * \brief Function for generating plume points based on a set of Fluent log files.
		 * \param none.
		 * \return 0 - if successful
		 * \return -1 - otherwise
		 *
		 */
		int generatePoints();
		/*!
		 *
		 * \fn int getChemicalReading(PSPoint3d * point, PSOdorData * odor_data)
		 * \brief Function for reading a single chemical point based on the robot current position.
		 * \param point A 3D point containning the current position of the robot.
		 * \param odor_data The odor data structure where the result is written.
		 * \return 0 - if successful
		 * \return -1 - otherwise
		 *
		 */
		int getChemicalReading(PSPoint3d * point, PSOdorData * odor_data);
	
		// Preferences
		FILE * logFile;				/**< Pointer to the log file.*/
		std::string fileName;			/**< Path to the log file.*/
	
		int numOfFrames;			/**< Number of frames.*/
		int currentFrame;			/**< Current frame index.*/	
		double cellSize;			/**< Cell size of the grid in meters.*/
		int maxPointsPerCell;			/**< Maximum number of points to be drawn for each cell.*/
		double maxOdorValue;			/**< Maximum odor value.*/
	
		private:
	
		std::vector<PSFramePoint> frame;	/**< Current frame.*/
	};

} // namespace

// EOF
