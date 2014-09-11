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

#include <boost/function.hpp>

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

			//template <typename primitiveType>
		    void evaluatePrimitive(string primitiveString){

				misc_msgs::EvaluateExpression evalExpr;
				primitiveStrContainer = labust::utilities::split(primitiveString, ':');

				for(vector<string>::iterator it = primitiveStrContainer.begin(); it != primitiveStrContainer.end(); it = it + 2){

					evalExpr.request.expression = (*(it+1)).c_str();
					primitiveMap[*it] =  (labust::utilities::callService(srvExprEval, evalExpr)).response.result;
				}
			}

			/*****************************************************************
			 ***  State machine primitive states
			 ****************************************************************/

		    void dynamic_postitioning_state(){

		    	if(!timeoutActive)
		    		setTimeout(receivedPrimitive.event.timeout);

				if(refreshRate>0){

					evaluatePrimitive(receivedPrimitive.primitiveString.data);
					CM.dynamic_positioning(true, primitiveMap["north"], primitiveMap["east"], primitiveMap["heading"]);
					oldPosition.north = primitiveMap["north"];
					oldPosition.east = primitiveMap["east"];

					ROS_ERROR("DP: %f. %f, %f",primitiveMap["north"], primitiveMap["east"], primitiveMap["heading"]);

					if(!refreshActive)
						setRefreshRate(refreshRate, boost::bind(&MissionExecution::dynamic_postitioning_state, this));

				} else {

					ROS_ERROR("Debug");
					misc_msgs::DynamicPositioning data = labust::utilities::deserializeMsg<misc_msgs::DynamicPositioning>(receivedPrimitive.primitiveData);
					CM.dynamic_positioning(true,data.point.north,data.point.east, data.heading);
					oldPosition = data.point;
				}
		    }

		    void go2point_FA_state(){

		    	if(!timeoutActive)
		    		setTimeout(receivedPrimitive.event.timeout);

				if(refreshRate>0){
					evaluatePrimitive(receivedPrimitive.primitiveString.data);
					CM.go2point_FA(true, oldPosition.north, oldPosition.east, primitiveMap["north"], primitiveMap["east"], primitiveMap["speed"], primitiveMap["heading"], primitiveMap["victory_radius"]);
					oldPosition.north = primitiveMap["north"];
					oldPosition.east = primitiveMap["east"];

					if(!refreshActive)
						setRefreshRate(refreshRate, boost::bind(&MissionExecution::go2point_FA_state, this));


				} else {

					misc_msgs::Go2PointFA data = labust::utilities::deserializeMsg<misc_msgs::Go2PointFA>(receivedPrimitive.primitiveData);
					CM.go2point_FA(true,oldPosition.north,oldPosition.east,data.point.north,data.point.east, data.speed, data.heading, data.victoryRadius);
					oldPosition = data.point;
				}
		    }

		    void go2point_UA_state(){

		    	if(!timeoutActive)
		    		setTimeout(receivedPrimitive.event.timeout);

		    	if(refreshRate>0){
		    						evaluatePrimitive(receivedPrimitive.primitiveString.data);
		    						CM.go2point_UA(true, oldPosition.north, oldPosition.east, primitiveMap["north"], primitiveMap["east"], primitiveMap["speed"], primitiveMap["victory_radius"]);
		    						oldPosition.north = primitiveMap["north"];
		    						oldPosition.east = primitiveMap["east"];

		    						if(!refreshActive)
		    							setRefreshRate(refreshRate, boost::bind(&MissionExecution::go2point_UA_state, this));


		    					} else {

					misc_msgs::Go2PointUA data = labust::utilities::deserializeMsg<misc_msgs::Go2PointUA>(receivedPrimitive.primitiveData);
					CM.go2point_UA(true,oldPosition.north,oldPosition.east,data.point.north,data.point.east, data.speed, data.victoryRadius);
					oldPosition = data.point;
				}
		    }

		    void course_keeping_FA_state(){

		    	if(!timeoutActive)
		    		setTimeout(receivedPrimitive.event.timeout);

				if(refreshRate>0){

					evaluatePrimitive(receivedPrimitive.primitiveString.data);
					CM.course_keeping_FA(true, primitiveMap["course"], primitiveMap["speed"], primitiveMap["heading"]);

					if(!refreshActive)
						setRefreshRate(refreshRate, boost::bind(&MissionExecution::course_keeping_FA_state, this));


				} else {

					misc_msgs::CourseKeepingFA data = labust::utilities::deserializeMsg<misc_msgs::CourseKeepingFA>(receivedPrimitive.primitiveData);
					CM.course_keeping_FA(true,data.course, data.speed, data.heading);
				}
		    }

		    void course_keeping_UA_state(){

		    	if(!timeoutActive)
		    		setTimeout(receivedPrimitive.event.timeout);

		    	if(refreshRate>0){

					evaluatePrimitive(receivedPrimitive.primitiveString.data);
					CM.course_keeping_UA(true, primitiveMap["course"], primitiveMap["speed"]);

					if(!refreshActive)
						setRefreshRate(refreshRate, boost::bind(&MissionExecution::course_keeping_UA_state, this));

				} else {
					misc_msgs::CourseKeepingUA data = labust::utilities::deserializeMsg<misc_msgs::CourseKeepingUA>(receivedPrimitive.primitiveData);
					CM.course_keeping_UA(true,data.course, data.speed);
				}
		    }

		    void iso_state(){

		    	if(!timeoutActive)
		    		setTimeout(receivedPrimitive.event.timeout);

				misc_msgs::ISO data = labust::utilities::deserializeMsg<misc_msgs::ISO>(receivedPrimitive.primitiveData);
				CM.ISOprimitive(true, data.dof, data.command, data.hysteresis, data.reference, data.sampling_rate);
		    }

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

			void setRefreshRate(double timeout, boost::function<void(void)> onRefreshCallback );

			void onTimeout(const ros::TimerEvent& timer);

			void onPrimitiveEndReset();

			/*********************************************************************
			 ***  Class variables
			 ********************************************************************/

			ros::NodeHandle nh_;
			ros::Timer timer, refreshRateTimer;
			ros::Publisher pubRequestPrimitive;
			ros::Subscriber subDataEventsContainer, subEventString, subReceivePrimitive;

			auv_msgs::NED oldPosition; /* Remember last primitive end point */
			misc_msgs::SendPrimitive receivedPrimitive;

			bool checkEventFlag, refreshActive, timeoutActive;
			int nextPrimitive;

			vector<string> eventsActive, primitiveStrContainer;

			map<string, double> primitiveMap;

			ros::ServiceClient srvExprEval;

			double refreshRate;

			labust::controller::ControllerManager CM;

		};

		/*****************************************************************
		 ***  Class functions
		 ****************************************************************/

		MissionExecution::MissionExecution(ros::NodeHandle& nh):checkEventFlag(false),
																	nextPrimitive(1),
																	refreshActive(false),
																	timeoutActive(false),
																	refreshRate(0.0),
																	CM(nh){

			/** Subscribers */
			subEventString = nh.subscribe<std_msgs::String>("eventString",1, &MissionExecution::onEventString, this);
			subReceivePrimitive = nh.subscribe<misc_msgs::SendPrimitive>("sendPrimitive",1, &MissionExecution::onReceivePrimitive, this);
			subDataEventsContainer = nh.subscribe<misc_msgs::DataEventsContainer>("dataEventsContainer",1, &MissionExecution::onDataEventsContainer, this);

			/** Publishers */
			pubRequestPrimitive = nh.advertise<std_msgs::UInt16>("requestPrimitive",1);

			/** Services */
			srvExprEval = nh.serviceClient<misc_msgs::EvaluateExpression>("evaluate_expression");

			/** Define primitive parameters  */
			primitiveMap.insert(std::pair<string, double>("north", 0.0));
			primitiveMap.insert(std::pair<string, double>("east", 0.0));
			primitiveMap.insert(std::pair<string, double>("heading", 0.0));
			primitiveMap.insert(std::pair<string, double>("course", 0.0));
			primitiveMap.insert(std::pair<string, double>("speed", 0.0));
			primitiveMap.insert(std::pair<string, double>("victory_radius", 0.0));
		}

		/*****************************************************************
		 ***  ROS Subscriptions Callback
		 ****************************************************************/

		void MissionExecution::onDataEventsContainer(const misc_msgs::DataEventsContainer::ConstPtr& data){

			/** If primitive has active events */
			if(checkEventFlag){

				/** Reset flag and counters */
				int flag = 0, i = 0;

				for(std::vector<uint8_t>::iterator it = receivedPrimitive.event.onEventNext.begin() ;
														it != receivedPrimitive.event.onEventNext.end(); ++it){

					/** For each primitive event check if it is true */
					if(data->eventsVar[receivedPrimitive.event.onEventNextActive[i++]-1] == 1){
						flag = 1;
						nextPrimitive = *it;
						ROS_ERROR("Event active:: %d", receivedPrimitive.event.onEventNextActive[i-1]);

						onPrimitiveEndReset();
						mainEventQueue->riseEvent("/PRIMITIVE_FINISHED");
					}

					/** First true event has priority */
					if (flag) break;
				}
			}
		}

		/* */
		void MissionExecution::onReceivePrimitive(const misc_msgs::SendPrimitive::ConstPtr& data){

			receivedPrimitive = *data;
			refreshRate = data->refreshRate;

			/** Check if received primitive has active events */
			if(receivedPrimitive.event.onEventNextActive.empty() == 0){
				checkEventFlag = true;
			}

			/** Call primitive */
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

				case placeholder:

					mainEventQueue->riseEvent("/PLACEHOLDER");
					break;

				case none:

					ROS_ERROR("Mission ended.");
					mainEventQueue->riseEvent("/STOP");
			}
		}


		/*********************************************************************
		 *** Helper functions
		 ********************************************************************/

		void MissionExecution::onEventString(const std_msgs::String::ConstPtr& msg){

			mainEventQueue->riseEvent(msg->data.c_str());
			ROS_INFO("EventString: %s",msg->data.c_str());
			if(strcmp(msg->data.c_str(),"/STOP") == 0){
				onPrimitiveEndReset();
				nextPrimitive = 1;
			}
		}

		void MissionExecution::requestPrimitive(){

			std_msgs::UInt16 req;
			req.data = nextPrimitive++;
			pubRequestPrimitive.publish(req);
		}

		void MissionExecution::setTimeout(double timeout){

		   	if(timeout != 0){
		   		ROS_ERROR("Setting timeout: %f", timeout);
				timer = nh_.createTimer(ros::Duration(timeout), &MissionExecution::onTimeout, this, true);
				timeoutActive = true;
		   	}
		}

		void MissionExecution::setRefreshRate(double timeout,  boost::function<void(void)> onRefreshCallback){

		   	if(timeout != 0){
		   		ROS_ERROR("Setting refresh rate: %f", timeout);
		   		refreshRateTimer = nh_.createTimer(ros::Duration(timeout), boost::bind(onRefreshCallback), false);
		   		refreshActive = true;
		   	}
		}

		void MissionExecution::onTimeout(const ros::TimerEvent& timer){

			ROS_ERROR("Timeout");
			onPrimitiveEndReset();
			mainEventQueue->riseEvent("/PRIMITIVE_FINISHED");
		}

		void MissionExecution::onPrimitiveEndReset(){

			/** Stop refresh rate timer */
			refreshRateTimer.stop();
			/** Stop timeout timer */
			timer.stop();
		    /** Reset execution flags */
			refreshActive = timeoutActive = checkEventFlag = false;
		}
	}
}

#endif /* MISSIONEXECUTION_HPP_ */
