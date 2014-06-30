/*********************************************************************
 * data_event_manager.cpp
 *
 *  Created on: Jun 30, 2014
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
*  BUT NOT LIMITED TO, PROCUREME	void onMissionSetup(misc_msgs::){

	}NT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#include<labust_mission/labustMission.hpp>
#include<labust_mission/dataManager.hpp>
#include<labust_mission/eventEvaluation.hpp>

#include<misc_msgs/EvaluateExpression.h>check

class DataEventManager{

public:
	DataEventManager(){

		ros::NodeHandle nh;

		subStateHatAbsSlow = nh.subscribe<auv_msgs::NavSts>("stateHatAbsSlow",1,&DataEventManager::onStateHatAbsSlow,this);

		srvEvaluateExpression = nh.advertiseService("evaluate_expression", &DataEventManager::expressionEvaluationService,this);

	}

	/* Callback for evaluating states and event condition */
	void onStateHatAbsSlow(const auv_msgs::NavSts::ConstPtr& data){


		DM.updateData(data);
		EE.updateSymbolTable(DM.stateHatVar,DM.missionVar,DM.eventsVar);
		EE.evaluateEvents(eventsContainer, eventsValue);

		publishDataEvent(DM.mainData, EE.eventsData);
	}

	void publishDataEvent(){

	}

	/* Callback that initializes mission parameters and events */
	void onMissionSetup(const misc_msgs::MissionSetup::ConstPtr& data){

		parseMissionParam(data->missionParam.c_str());
		parseMissionEvents(data->missionParam.c_str());
	}


	/* Service that evaluates string expression */
	bool expressionEvaluationService(misc_msgs::EvaluateExpression::Request &req, misc_msgs::EvaluateExpression::Response &res){

		res.result = EE.evaluateStringExpression(req.expression);
		return true;
	}

	/*****************************************************************
	 *** Helper functions
	 ****************************************************************/

	void parseMissionParam(std::string missionParamString){

		std::vector<std::string> tmp;
		tmp = labust::utilities::split(missionParamString.c_str(), ':');
		int i = 0;
		for(std::vector<std::string>::iterator it = tmp.begin()+1; it != tmp.end(); it=it+2){
			DM.missionVar[i++] = atof((*it).c_str());
		}
	}

	void parseMissionEvents(std::string missionEventsString){

		eventsContainer = labust::utilities::split(missionEventsString.c_str(), ':');
	}

	labust::event::EventEvaluation EE;
	labust::data::DataManager DM;

	ros::Subscriber subStateHatAbsSlow;
	ros::Publisher pubDataEvent;

	ros::ServiceServer srvEvaluateExpression;

	std::vector<std::string> eventsContainer;
	std::vector<double> eventsValue;
	std::vector<bool> eventsStatus;

};


int main(int argc, char **argv){

	ros::init(argc, argv, "data_event_manager");
	ros::NodeHandle nh;
	DataEventManager DEM;
	ros::spin();
	return 0;
}


