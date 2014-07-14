/*********************************************************************
 * missionExecution.hpp
 *
 *  Created on: Apr 22, 2014
 *      Author: Filip Mandic
 *
 ********************************************************************/

/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, LABUST, UNIZG-FER
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
*********************************************************************/

#ifndef MISSIONEXECUTION_HPP_
#define MISSIONEXECUTION_HPP_


#include <labust_mission/labustMission.hpp>
#include <exprtk/exprtk.hpp>

#include <decision_making/SynchCout.h>
#include <decision_making/BT.h>
#include <decision_making/FSM.h>
#include <decision_making/ROSTask.h>
#include <decision_making/DecisionMaking.h>

extern decision_making::EventQueue* mainEventQueue;

namespace ser = ros::serialization;

/*********************************************************************
 ***  MissionExecution class definition
 ********************************************************************/

namespace labust {
	namespace mission {

		class MissionExecution{

		public:

			/*****************************************************************
			 ***  Class functions
			 ****************************************************************/

			MissionExecution(ros::NodeHandle& nh);

			/*****************************************************************
			 ***  ROS Subscriptions Callback
			 ****************************************************************/

			void onStateHat(const auv_msgs::NavSts::ConstPtr& data);

			void onDataEventsContainer(const misc_msgs::DataEventsContainer::ConstPtr& data);

			void onEventString(const std_msgs::String::ConstPtr& msg);

			void onReceivePrimitive(const misc_msgs::SendPrimitive::ConstPtr& data);

			/*********************************************************************
			 *** Helper functions
			 ********************************************************************/

			void requestPrimitive();

			void setTimeout(double timeout);

			void onTimeout(const ros::TimerEvent& timer);

			/*********************************************************************
			 ***  Class variables
			 ********************************************************************/

			ros::NodeHandle nh_;
			ros::Timer timer;
			ros::Publisher pubRequestPrimitive;
			ros::Subscriber subDataEventsContainer, subEventString, subReceivePrimitive;

			auv_msgs::NED oldPosition; /* Remember last primitive end point */
			misc_msgs::SendPrimitive receivedPrimitive;

			bool checkEventFlag;
			int nextPrimitive;

			vector<string> eventsActive;
		};

		/*****************************************************************
		 ***  Class functions
		 ****************************************************************/

		MissionExecution::MissionExecution(ros::NodeHandle& nh):checkEventFlag(false),nextPrimitive(1){

			/* Subscribers */
			subEventString = nh.subscribe<std_msgs::String>("eventString",1, &MissionExecution::onEventString, this);
			subReceivePrimitive = nh.subscribe<misc_msgs::SendPrimitive>("sendPrimitive",1, &MissionExecution::onReceivePrimitive, this);
			subDataEventsContainer = nh.subscribe<misc_msgs::DataEventsContainer>("dataEventsContainer",1, &MissionExecution::onDataEventsContainer, this);

			/* Publishers */
			pubRequestPrimitive = nh.advertise<std_msgs::UInt16>("requestPrimitive",1);
		}

		/*****************************************************************
		 ***  ROS Subscriptions Callback
		 ****************************************************************/

		void MissionExecution::onDataEventsContainer(const misc_msgs::DataEventsContainer::ConstPtr& data){

			/* Reset flag and counters */
			int flag = 0, i = 0;

			/* If primitve has active events */
			if(checkEventFlag){

				for(std::vector<uint8_t>::iterator it = receivedPrimitive.event.onEventNext.begin() ;
														it != receivedPrimitive.event.onEventNext.end(); ++it){

					/* For each primitive event check if it is true */
					if(data->eventsVar[receivedPrimitive.event.onEventNextActive[i++]-1] == 1){
						flag = 1;
						nextPrimitive = *it;
						ROS_ERROR("Aktivan event: %d", receivedPrimitive.event.onEventNextActive[i-1]);
						checkEventFlag = false;
						mainEventQueue->riseEvent("/PRIMITIVE_FINISHED");
					}

					/* First true event has priority */
					if (flag) break;
				}
			}
		}

		/* */
		void MissionExecution::onReceivePrimitive(const misc_msgs::SendPrimitive::ConstPtr& data){

			receivedPrimitive = *data;

			/* Check if received primitive has active events */
			if(receivedPrimitive.event.onEventNextActive.empty() == 0){
				checkEventFlag = true;
			}

			/* Call primitive */
			switch(data->primitiveID){

				case go2point_FA:

					mainEventQueue->riseEvent("/GO2POINT_FA");
					break;

				case go2point_UA:

					mainEventQueue->riseEvent("/GO2POINT_UA");
					break;

				case dynamic_positioning:

					mainEventQueue->riseEvent("/DYNAMIC_POSITIONING");
					break;

				case course_keeping_FA:

					mainEventQueue->riseEvent("/COURSE_KEEPING_FA");
					break;

				case course_keeping_UA:

					mainEventQueue->riseEvent("/COURSE_KEEPING_UA");
					break;

				case iso:

					mainEventQueue->riseEvent("/ISO");
					break;

				case none:

					ROS_ERROR("Mission ended.");
					mainEventQueue->riseEvent("/STOP");
			}
		}

		void MissionExecution::onEventString(const std_msgs::String::ConstPtr& msg){

			mainEventQueue->riseEvent(msg->data.c_str());
			ROS_INFO("EventString: %s",msg->data.c_str());
			if(strcmp(msg->data.c_str(),"/STOP") == 0){
				checkEventFlag = false;
				nextPrimitive = 1;
			}
		}

		/*********************************************************************
		 *** Helper functions
		 ********************************************************************/

		void MissionExecution::requestPrimitive(){
			std_msgs::UInt16 req;
			req.data = nextPrimitive++;
			pubRequestPrimitive.publish(req);
		}


		void MissionExecution::setTimeout(double timeout){

		   	if(timeout != 0){
		   		ROS_ERROR("Setting timeout: %f", timeout);
				timer = nh_.createTimer(ros::Duration(timeout), &MissionExecution::onTimeout, this, true);
		   	}
		}

		void MissionExecution::onTimeout(const ros::TimerEvent& timer){

			ROS_ERROR("Timeout");
			mainEventQueue->riseEvent("/PRIMITIVE_FINISHED");
			checkEventFlag = false;

		}
	}
}

#endif /* MISSIONEXECUTION_HPP_ */
