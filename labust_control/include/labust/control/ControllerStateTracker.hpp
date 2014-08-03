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
 *  Created on: 26.06.2013.
 *  Author: Dula Nad
 *********************************************************************/
#ifndef CONTROLLERSTATETRACKER_HPP_
#define CONTROLLERSTATETRACKER_HPP_
#include <navcon_msgs/ControllerState.h>
#include <navcon_msgs/ControllerInfo.h>
#include <navcon_msgs/RegisterController_v3.h>
#include <ros/ros.h>

#include <boost/shared_ptr.hpp>
#include <boost/function.hpp>

namespace labust
{
	namespace control
	{
		class ControllerStateTracker
		{
			typedef navcon_msgs::ControllerState CStateT;
			typedef navcon_msgs::ControllerInfo InfoT;
			typedef boost::function< void(const std::string&, const InfoT&) > CallbackType;
		public:
			typedef boost::shared_ptr<ControllerStateTracker> Ptr;

			ControllerStateTracker(){};

			void init(ros::NodeHandle nh, ros::NodeHandle ph, CallbackType callback)
			{
				this->nh = nh;
				this->callback = callback;
				enableSub = nh.subscribe<navcon_msgs::ControllerState>("controller_state",1,
						&ControllerStateTracker::onControllerState, this);
			}

			void registerController(const navcon_msgs::RegisterController_v3Request& request)
			{
			  ros::ServiceClient client = nh.serviceClient<navcon_msgs::RegisterController_v3>("register_controller");

			  while (nh.ok() && !client.waitForExistence(ros::Duration(5.0)))
			  {
			  	ROS_WARN("Unable to register controller.");
			  }

			  navcon_msgs::RegisterController_v3 srv;
			  srv.request = request;
			  if (client.call(srv))
			  {
			    ROS_INFO("Registered controller %s.", request.name.c_str());
			  }
			  else
			  {
			    ROS_ERROR("Failed to register controller %s.", request.name.c_str());
			  }

			  //Add name to list
			  this->names.push_back(request.name);
			};

			void onControllerState(const navcon_msgs::ControllerState::ConstPtr& cstate)
			{
				ROS_DEBUG("New controller state.");
				for (int i=0; i<names.size(); ++i)
				{
					//Name lookup
					CStateT::_name_type::const_iterator it = std::find(cstate->name.begin(), cstate->name.end(), names[i]);

					if (it != cstate->name.end())
					{
						ROS_DEBUG("Found controller named:%s", names[i].c_str());
						//Calculate location in the info array.
						size_t idx = it - cstate->name.begin();
						if (callback != 0)
						{
							callback(names[i], cstate->info[idx]);
						}
						else
						{
							ROS_WARN("Empty callback for controller.");
						}
					}
				}
			}

		protected:
			/**
			 * The topic subscriber.
			 */
			ros::Subscriber enableSub;
			/**
			 * Controller names.
			 */
			std::vector<std::string> names;
			/**
			 * The on change event callback.
			 */
			CallbackType callback;
			/**
			 * Node handle
			 */
			ros::NodeHandle nh;
		};
	};
};

/* CONTROLLERSTATETRACKER_HPP_ */
#endif
