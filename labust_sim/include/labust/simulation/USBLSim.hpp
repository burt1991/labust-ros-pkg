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
#ifndef USBLSIM_HPP_
#define USBLSIM_HPP_
#include <labust/simulation/USBLGenHWSim.hpp>

#include <underwater_msgs/ModemTransmission.h>
#include <ros/ros.h>

#include <map>
#include <string>

namespace labust
{
	namespace simulation
	{
		/**
		 *  This class implements a single USBL sensor.
		 *
		 *  \todo Add passive localization
		 *  \todo Add master/slave/interleaved operation.
		 *  \todo Add buffered reply option.
		 *  \todo Use boot bimap ?
		 *  \todo Extend implementation to Action Server ?
		 */
		class USBLSim
		{
			typedef std::map<int, std::string> IDMap;
			typedef std::map<std::string, int> NameMap;
		public:
			/**
			 * The generic constructor.
			 */
			USBLSim();

			/**
			 *  The method initializes the ROS node and configures
			 *  the model from the ROS parameters.
			 */
			void onInit();

		private:
			/// Modem request handler
			void onModemOut(const underwater_msgs::ModemTransmission::ConstPtr& data);

			///USBL position reporter
			ros::Publisher odomOut;
			///Incoming messages payload publisher
			ros::Publisher modemIn;
			///Incoming nav estimate
			ros::Publisher navIn;
			///Modem outgoing request subscriber
			ros::Subscriber modemOut;

			///ID Name map
			IDMap id2name;
			///Name to ID map
			NameMap name2id;

			///Hardware interface
			USBLGenHWSim impl;
		};
	}
}

/* USBLSIM_HPP_ */
#endif
