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
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#include<labust_mission/labustMission.hpp>
#include<labust_mission/dataManager.hpp>
#include<labust_mission/eventEvaluation.hpp>

/*********************************************************************
 *** DataEventManager class definition
 ********************************************************************/

class DataEventManager{

public:
	DataEventManager():EE(DM.stateHatVar,DM.missionVar,DM.eventsVar){

		ros::NodeHandle nh;

		/* Subscriber */
		subStateHatAbs = nh.subscribe<auv_msgs::NavSts>("stateHatAbs",1,&DataEventManager::onStateHat,this);
		subMissionSetup = nh.subscribe<misc_msgs::MissionSetup>("missionSetup",1,&DataEventManager::onMissionSetup,this);

		/* Publisher */
		pubDataEventsContainer = nh.advertise<misc_msgs::DataEventsContainer>("dataEventsContainer",1);

		/* Service */
		srvEvaluateExpression = nh.advertiseService("evaluate_expression", &DataEventManager::expressionEvaluationService,this);
	}

	/* Callback for evaluating states and event condition */
	void onStateHat(const auv_msgs::NavSts::ConstPtr& data){

		/* Update all data in DataManager */
		//DM.updateData(data);

		/* Update EventEvaluation symbol table and evaluate events */
		//EE.updateSymbolTable(DM.stateHatVar,DM.missionVar,DM.eventsVar);
		//EE.evaluateEvents(eventsContainer, boost::ref(eventsValue));

		/* Publish events data and events states */
		//publishDataEvent(DM.mainData, eventsValue);
	}

	/* Publish data and events */
	void publishDataEvent(vector<double> mainVal, vector<double> eventsVal){

		misc_msgs::DataEventsContainer dataEventsContainer;
		dataEventsContainer.stateVar.data  = mainVal;
		dataEventsContainer.eventsVar.data = eventsVal;

		pubDataEventsContainer.publish(dataEventsContainer);
	}

	/* Callback that initializes mission parameters and events */
	void onMissionSetup(const misc_msgs::MissionSetup::ConstPtr& data){

		parseMissionParam(data->missionParams.c_str());
		parseMissionEvents(data->missionParams.c_str());
	}


	/* Service that evaluates string expression */
	bool expressionEvaluationService(misc_msgs::EvaluateExpression::Request &req, misc_msgs::EvaluateExpression::Response &res){

		//ROS_ERROR("Evaluating string expression: %s", req.expression.c_str());
		res.result = EE.evaluateStringExpression(req.expression);
		return true;
	}

	/*****************************************************************
	 *** Helper functions
	 ****************************************************************/

	/* Parse string with mission parameters sent from mission parser */
	void parseMissionParam(string missionParamString){

		if(missionParamString.empty() == 0){
			vector<string> tmp;
			tmp = labust::utilities::split(missionParamString.c_str(), ':');
			int i = 0;
			for(vector<string>::iterator it = tmp.begin()+1; it != tmp.end(); it=it+2){

				DM.missionVarNames[i] = (*(it-1)).c_str(); /* Assign mission variable name */
				DM.missionVar[i++] = atof((*it).c_str()); /* Assign mission variable value */
			}
		}
	}

	/* Parse string with mission events sent from mission parser */
	void parseMissionEvents(string missionEventsString){

		if(missionEventsString.empty() == 0){
			eventsContainer = labust::utilities::split(missionEventsString.c_str(), ':');
		}
	}


	/*****************************************************************
	 *** Class variables
	 ****************************************************************/

	labust::data::DataManager DM;
	labust::event::EventEvaluation EE;

	ros::Subscriber subStateHatAbs, subMissionSetup;
	ros::Publisher pubDataEvent, pubDataEventsContainer;

	ros::ServiceServer srvEvaluateExpression;

	vector<string> eventsContainer;
	vector<double> eventsValue;
	vector<bool> eventsStatus;

};

/*********************************************************************
 *** Main
 ********************************************************************/

int main(int argc, char **argv){

	ros::init(argc, argv, "data_event_manager");
	ros::NodeHandle nh;
	DataEventManager DEM;
	ros::spin();
	return 0;
}


