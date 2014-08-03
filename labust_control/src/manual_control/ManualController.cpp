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
#include <labust/control/ROSController.hpp>
#include <labust/control/EnablePolicy.hpp>
#include <labust/control/ManControl.hpp>
#include <labust/tools/MatrixLoader.hpp>
#include <labust/tools/conversions.hpp>

#include <auv_msgs/BodyVelocityReq.h>

#include <set>

namespace labust
{
	namespace control{

		class ControllerMultiStateTracker
		{
			typedef navcon_msgs::ControllerState CStateT;
			typedef navcon_msgs::ControllerInfo InfoT;
			typedef boost::function< void(InfoT&) > CallbackType;
		public:
			typedef boost::shared_ptr<ControllerStateTracker> Ptr;

			ControllerMultiStateTracker(ros::NodeHandle& nh, ros::NodeHandle& ph)
			{
				enableSub = nh.subscribe<navcon_msgs::ControllerState>("controller_state",1,
						&ControllerStateTracker::onControllerState, this);
			}

			void registerCallback(const CallbackType& callback){this->callback = callback;};

			void onControllerState(const navcon_msgs::ControllerState::ConstPtr& cstate)
			{
				bool newUpdate = false;

				for (std::set<std::string>::iterator it = names.begin(); it != names.end(); ++it)
				{
					CStateT::_name_type::const_iterator it = std::find(cstate->name.begin(), cstate->name.end(), *it);
					if (it != cstate->name.end())
					{
						size_t idx = it - cstate->name.begin();
						if ((newUpdate = changed(cstate->info[idx]), info[idx])) break;
					}
				}

				if (newUpdate)
				{
					size_t idx = it - cstate->name.begin();
					if (changed(cstate->info[idx], info))
					{
						ROS_INFO("New change.");
						info = cstate->info[idx];
						if (callback != 0)
						{
							callback(info);
						}
						else
						{
							ROS_WARN("Empty callback for controller state tracker.");
						}
					}
				}
			}

		protected:
			/**
			 * Comparison of two info object.
			 */
			bool changed(const InfoT& left, const InfoT& right)
			{
				return (left.reference_topic != right.reference_topic) ||
						(left.state_topic != right.state_topic) ||
						(left.tracking_topic != right.tracking_topic) ||
						(left.state != right.state);
			}

			/**
			 * The topic subscriber.
			 */
			ros::Subscriber enableSub;
			/**
			 * Controller name.
			 */
			std::vector<std::string> names;
			/**
			 * The controller info.
			 */
			std::vector<InfoT> info;
			/**
			 * The on change event callback.
			 */
			CallbackType callback;
		};


		struct ManualController
		{
			ManualController():
			nu_max(Eigen::Vector6d::Zero()){};

			void init(ros::NodeHandle& nh, ros::NodeHandle& ph)
			{
				labust::tools::getMatrixParam(nh,"manual_controller/maximum_speeds", nu_max);

				//Init subscriber
				joy = nh.subscribe<sensor_msgs::Joy>("joy", 1,
						&ManualController::onJoy,this);

				//Init publishers
				tauRef = nh.advertise<auv_msgs::BodyForceReq>("tauRef", 1);
				nuRef = nh.advertise<auv_msgs::BodyVelocityReq>("nuRef", 1);
				stateRef = nh.advertise<auv_msgs::NavSts>("stateRef", 1);
			}

			void onJoy(const sensor_msgs::Joy::ConstPtr& joyIn)
			{
				if (!Enable::enable) return;
				auv_msgs::BodyVelocityReq::Ptr nu(new auv_msgs::BodyVelocityReq());
				Eigen::Vector6d mapped;
				mapper.remap(*joyIn, mapped);

				nu->header.stamp = ros::Time::now();
				nu->header.frame_id = "base_link";
				nu->goal.requester = "nu_manual";
				nu->disable_axis = this->disable_axis;

				mapped = nu_max.cwiseProduct(mapped);
				labust::tools::vectorToPoint(mapped, nu->twist.linear);
				labust::tools::vectorToPoint(mapped, nu->twist.angular, 3);

				nuRef.publish(nu);
			}

		private:
			Eigen::Vector6d nu_max;
			ros::Subscriber joy;
			ros::Publisher tauRef,nuRef,stateRef;
			JoystickMapping mapper;
		};
	}
}

int main(int argc, char* argv[])
{
	ros::init(argc,argv,"nu_manual");

	labust::control::NuManual<labust::control::EnableServicePolicy> controller;
	ros::spin();

	return 0;
}



