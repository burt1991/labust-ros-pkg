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
#include <boost/algorithm/string.hpp>

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

		    void evaluatePrimitive(string primitiveString);

			/*****************************************************************
			 ***  State machine primitive states
			 ****************************************************************/

		    void dynamic_postitioning_state();

		    void go2point_FA_state();

		    void go2point_UA_state();

		    void course_keeping_FA_state();

		    void course_keeping_UA_state();

		    void iso_state();

		    void path_following_state();

		    void pointer_state();

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

			/** Controller manager class */
			labust::controller::ControllerManager CM;

			/** ROS Node handle */
			ros::NodeHandle nh_;

			/** Timers */
			ros::Timer timer, refreshRateTimer;

			/** Publishers */
			ros::Publisher pubRequestPrimitive;

			/** Subscribers */
			ros::Subscriber subDataEventsContainer, subEventString, subReceivePrimitive;

			/** Services */
			ros::ServiceClient srvExprEval;

			/** Remember last primitive end point */
			auv_msgs::NED oldPosition;

			/** Store last received primitive */
			misc_msgs::SendPrimitive receivedPrimitive;

			/** Vectors */
			vector<string> eventsActive, primitiveStrContainer;

			/** Map for storing last primitive floating point data */
			map<string, double> primitiveMap;

			/** Map for storing last primitive string data */
			map<string, string> primitiveStringMap;

			/** Execution flags */
			bool checkEventFlag, refreshActive, timeoutActive;

			/** Next primitive to request */
			int nextPrimitive;

			/** Primitive reference refresh rate */
			double refreshRate;
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
			///TODO Automate primitive map creation.
			primitiveMap.insert(std::pair<string, double>("north", 0.0));
			primitiveMap.insert(std::pair<string, double>("east", 0.0));
			primitiveMap.insert(std::pair<string, double>("depth", 0.0));
			primitiveMap.insert(std::pair<string, double>("heading", 0.0));
			primitiveMap.insert(std::pair<string, double>("course", 0.0));
			primitiveMap.insert(std::pair<string, double>("speed", 0.0));
			primitiveMap.insert(std::pair<string, double>("victory_radius", 0.0));
			primitiveMap.insert(std::pair<string, double>("dof", 0.0));
			primitiveMap.insert(std::pair<string, double>("command", 0.0));
			primitiveMap.insert(std::pair<string, double>("hysteresis", 0.0));
			primitiveMap.insert(std::pair<string, double>("reference", 0.0));
			primitiveMap.insert(std::pair<string, double>("sampling_rate", 0.0));
			primitiveMap.insert(std::pair<string, double>("victory_radius", 0.0));

			primitiveStringMap.insert(std::pair<string, string>("radius_topic", ""));
			primitiveStringMap.insert(std::pair<string, string>("center_topic", ""));
			primitiveStringMap.insert(std::pair<string, string>("target_topic", ""));
			primitiveStringMap.insert(std::pair<string, string>("point", ""));

		}

	    void MissionExecution::evaluatePrimitive(string primitiveString){

			misc_msgs::EvaluateExpression evalExpr;
			primitiveStrContainer = labust::utilities::split(primitiveString, ':');

			for(vector<string>::iterator it = primitiveStrContainer.begin(); it != primitiveStrContainer.end(); it = it + 2){

				size_t found = (*(it+1)).find_first_of('#');
				if(found != string::npos){
					primitiveStringMap[*it] =  (*(it+1)).erase(found,1);
					/// Debug
					ROS_ERROR("Evaluation %s: %s", (*it).c_str(),primitiveStringMap[*it].c_str());
					continue;
				}

				ROS_ERROR("Evaluation %s: %f", (*it).c_str(),primitiveMap[*it]);


				evalExpr.request.expression = (*(it+1)).c_str();
				primitiveMap[*it] =  (labust::utilities::callService(srvExprEval, evalExpr)).response.result;
			}
		}

		/*****************************************************************
		 ***  State machine primitive states
		 ****************************************************************/

	    void MissionExecution::dynamic_postitioning_state(){

	    	/** Activate primitive timeout */
	    	if(!timeoutActive && receivedPrimitive.event.timeout > 0)
	    		setTimeout(receivedPrimitive.event.timeout);

	    	/** Evaluate primitive data with current values */
			evaluatePrimitive(receivedPrimitive.primitiveString.data);
			/** Activate primitive */
			CM.dynamic_positioning(true, primitiveMap["north"], primitiveMap["east"], primitiveMap["heading"]);
			oldPosition.north = primitiveMap["north"];
			oldPosition.east = primitiveMap["east"];

			ROS_ERROR("DP: %f. %f, %f",primitiveMap["north"], primitiveMap["east"], primitiveMap["heading"]);

			/** Activate primitive reference refresh */
			if(!refreshActive && refreshRate > 0)
				setRefreshRate(refreshRate, boost::bind(&MissionExecution::dynamic_postitioning_state, this));
	    }

	    void MissionExecution::go2point_FA_state(){

	    	if(!timeoutActive && receivedPrimitive.event.timeout > 0)
	    		setTimeout(receivedPrimitive.event.timeout);

			evaluatePrimitive(receivedPrimitive.primitiveString.data);
			CM.go2point_FA(true, oldPosition.north, oldPosition.east, primitiveMap["north"], primitiveMap["east"], primitiveMap["speed"], primitiveMap["heading"], primitiveMap["victory_radius"]);
			oldPosition.north = primitiveMap["north"];
			oldPosition.east = primitiveMap["east"];

			if(!refreshActive && refreshRate > 0)
				setRefreshRate(refreshRate, boost::bind(&MissionExecution::go2point_FA_state, this));
	    }

	    void MissionExecution::go2point_UA_state(){

	    	if(!timeoutActive && receivedPrimitive.event.timeout > 0)
	    		setTimeout(receivedPrimitive.event.timeout);

			evaluatePrimitive(receivedPrimitive.primitiveString.data);
			CM.go2point_UA(true, oldPosition.north, oldPosition.east, primitiveMap["north"], primitiveMap["east"], primitiveMap["speed"], primitiveMap["victory_radius"]);
			oldPosition.north = primitiveMap["north"];
			oldPosition.east = primitiveMap["east"];

			if(!refreshActive && refreshRate > 0)
				setRefreshRate(refreshRate, boost::bind(&MissionExecution::go2point_UA_state, this));
	    }

	    void MissionExecution::course_keeping_FA_state(){

	    	if(!timeoutActive && receivedPrimitive.event.timeout > 0)
	    		setTimeout(receivedPrimitive.event.timeout);

			evaluatePrimitive(receivedPrimitive.primitiveString.data);
			CM.course_keeping_FA(true, primitiveMap["course"], primitiveMap["speed"], primitiveMap["heading"]);

			if(!refreshActive && refreshRate > 0)
				setRefreshRate(refreshRate, boost::bind(&MissionExecution::course_keeping_FA_state, this));
	    }

	    void MissionExecution::course_keeping_UA_state(){

	    	if(!timeoutActive && receivedPrimitive.event.timeout > 0)
	    		setTimeout(receivedPrimitive.event.timeout);

			evaluatePrimitive(receivedPrimitive.primitiveString.data);
			CM.course_keeping_UA(true, primitiveMap["course"], primitiveMap["speed"]);

			if(!refreshActive && refreshRate > 0)
				setRefreshRate(refreshRate, boost::bind(&MissionExecution::course_keeping_UA_state, this));
	    }

	    void MissionExecution::iso_state(){

	    	if(!timeoutActive && receivedPrimitive.event.timeout > 0)
	    		setTimeout(receivedPrimitive.event.timeout);

	    	ROS_ERROR("ISO: %f, %f", primitiveMap["dof"], primitiveMap["command"]);

	    	/** Evaluate primitive data with current values */
			evaluatePrimitive(receivedPrimitive.primitiveString.data);
			/** Activate primitive */
			CM.ISOprimitive(true, primitiveMap["dof"], primitiveMap["command"], primitiveMap["hysteresis"], primitiveMap["reference"], primitiveMap["sampling_rate"]);
	    }

	    void MissionExecution::path_following_state(){

//	    	if(!timeoutActive && receivedPrimitive.event.timeout > 0)
//	    		setTimeout(receivedPrimitive.event.timeout);
//
			evaluatePrimitive(receivedPrimitive.primitiveString.data);
//			CM.go2point_FA(true, oldPosition.north, oldPosition.east, primitiveMap["north"], primitiveMap["east"], primitiveMap["speed"], primitiveMap["heading"], primitiveMap["victory_radius"]);
//			oldPosition.north = primitiveMap["north"];
//			oldPosition.east = primitiveMap["east"];
//
//			if(!refreshActive && refreshRate > 0)
//				setRefreshRate(refreshRate, boost::bind(&MissionExecution::go2point_FA_state, this));
	    }

	    void MissionExecution::pointer_state(){

//	    	if(!timeoutActive && receivedPrimitive.event.timeout > 0)
//	    		setTimeout(receivedPrimitive.event.timeout);
//
			evaluatePrimitive(receivedPrimitive.primitiveString.data);
//			CM.go2point_FA(true, oldPosition.north, oldPosition.east, primitiveMap["north"], primitiveMap["east"], primitiveMap["speed"], primitiveMap["heading"], primitiveMap["victory_radius"]);
//			oldPosition.north = primitiveMap["north"];
//			oldPosition.east = primitiveMap["east"];
//
//			if(!refreshActive && refreshRate > 0)
//				setRefreshRate(refreshRate, boost::bind(&MissionExecution::go2point_FA_state, this));
	    }

		/*****************************************************************
		 ***  ROS Subscriptions Callback
		 ****************************************************************/

		/** DataEventsContainer callback  */
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

		/** ReceivePrimitive topic callback */
		void MissionExecution::onReceivePrimitive(const misc_msgs::SendPrimitive::ConstPtr& data){

			receivedPrimitive = *data;
			refreshRate = data->refreshRate;

			/** Check if received primitive has active events */
			if(receivedPrimitive.event.onEventNextActive.empty() == 0){
				checkEventFlag = true;
			}

			/** Call primitive */
			if(data->primitiveID != none){

				string id_string(PRIMITIVES[data->primitiveID]);
				id_string = "/" + boost::to_upper_copy(id_string);
				mainEventQueue->riseEvent(id_string.c_str());
			}else{
				ROS_ERROR("Mission ended.");
				mainEventQueue->riseEvent("/STOP");
			}
		}

		/*********************************************************************
		 *** Helper functions
		 ********************************************************************/

		/** EventString topic callback */
		void MissionExecution::onEventString(const std_msgs::String::ConstPtr& msg){

			mainEventQueue->riseEvent(msg->data.c_str());
			ROS_INFO("EventString: %s",msg->data.c_str());
			if(strcmp(msg->data.c_str(),"/STOP") == 0){
				onPrimitiveEndReset();
				nextPrimitive = 1;
			}
		}

		/** Request new primitive */
		void MissionExecution::requestPrimitive(){

			std_msgs::UInt16 req;
			req.data = nextPrimitive++;
			pubRequestPrimitive.publish(req);
		}

		/** Set primitive timeout */
		void MissionExecution::setTimeout(double timeout){

		   	if(timeout != 0){
		   		ROS_ERROR("Setting timeout: %f", timeout);
				timer = nh_.createTimer(ros::Duration(timeout), &MissionExecution::onTimeout, this, true);
				timeoutActive = true;
		   	}
		}

		/** Set primitive reference refresh rate */
		void MissionExecution::setRefreshRate(double timeout,  boost::function<void(void)> onRefreshCallback){

		   	if(timeout != 0){
		   		ROS_ERROR("Setting refresh rate: %f", timeout);
		   		refreshRateTimer = nh_.createTimer(ros::Duration(timeout), boost::bind(onRefreshCallback), false);
		   		refreshActive = true;
		   	}
		}

		/** On timeout finish primitive execution */
		void MissionExecution::onTimeout(const ros::TimerEvent& timer){

			ROS_ERROR("Timeout");
			onPrimitiveEndReset();
			mainEventQueue->riseEvent("/TIMEOUT");
		}

		/** Reset timers and flags */
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
