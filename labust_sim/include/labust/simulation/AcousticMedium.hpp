/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, LABUST, UNIZG-FER
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
 *   * Neither the name of the LABUST nor the names of its
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
 *  Author: Dula Nad
 *  Created: 01.02.2013.
 *********************************************************************/
#ifndef ACOUSTICMEDIUM_HPP_
#define ACOUSTICMEDIUM_HPP_
#include <underwater_msgs/MediumTransmission.h>

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

#include <boost/tuple/tuple.hpp>

#include <map>
#include <string>

namespace labust
{
	namespace simulation
	{
		/**
		 *  This class implements the acoustic medium simulation layer.
		 */
		class AcousticMedium
		{
			struct TransponderInfo
			{
				nav_msgs::Odometry::Ptr odom;
			};

			typedef std::map<int, TransponderInfo> TransponderMap;
			typedef std::multimap<double,
					underwater_msgs::MediumTransmissionPtr> TransmissionMap;
			typedef boost::tuple<double, double, double> RBE;

			///RBE enumerator
			enum {range = 0, bearing, elevation};

		public:
			/**
			 * The generic constructor.
			 */
			AcousticMedium();

			/**
			 *  The method initializes the ROS node and configures
			 *  the model from the ROS parameters.
			 */
			void onInit();

		private:
			///New medium data handler
			void onMediumData(const underwater_msgs::MediumTransmission::ConstPtr& data);

			///Helper range and angle calculator
			RBE getRelativePosition(const TransponderInfo& source,
					const TransponderInfo& receiver);


			///Gateway to acoustic medium
			ros::Subscriber toMedium;
			///Incoming from acoustic medium
			ros::Publisher frMedium;

			///Sound speed
			double vs;

			///Acoustic transponders in medium
			TransponderMap transponders;
		};
	}
}

/* ACOUSTICMEDIUM_HPP_ */
#endif
