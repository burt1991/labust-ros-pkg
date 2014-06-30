//\todo omoguci overide external event descriptiona
//\todo uvedi razlicite tipove ekternigh varijabli konitnuirane, flag, ...
/*********************************************************************
 * eventEvaluation.hpp
 *
 *  Created on: May 8, 2014
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

#ifndef EVENTEVALUATION_HPP_
#define EVENTEVALUATION_HPP_

#include <labust_mission/labustMission.hpp>
#include <exprtk/exprtk.hpp>

/*********************************************************************
 ***  EventEvaluation class definition
 ********************************************************************/

namespace labust {
	namespace event {

		class EventEvaluation{

		public:

			typedef exprtk::symbol_table<double> symbol_table_t;
			typedef exprtk::expression<double> expression_t;
			typedef exprtk::parser<double> parser_t;
			typedef exprtk::parser_error::type error_t;

			/*****************************************************************
			 ***  Class functions
			 ****************************************************************/

			EventEvaluation();

			int checkEventState(auv_msgs::NavSts data, std::string expression_str);

			void evaluateEvents(std::vector<std::string> events, std::vector<std::string>& eventsValue);

			double evaluateStringExpression(std::string expression_str);

			void updateSymbolTable(std::vector<double> stateVar, std::vector<double> missionVar, std::vector<double> eventsVar);

			void initializeSymbolTable();

			//void onReceiveExternalEvent(const misc_msgs::ExternalEvent::ConstPtr& data);

			//void setExternalEvents();

			//void onStateHat(const auv_msgs::NavSts::ConstPtr& data);

			/*********************************************************************
			 ***  Class variables
			 ********************************************************************/

			//ros::Subscriber subExternalEvents;
			//ros::Subscriber subStateHatAbs;

			symbol_table_t symbol_table;

			//double x,y,z,psi;
			//std::vector<misc_msgs::ExternalEvent> externalEventContainer;



		};

		EventEvaluation::EventEvaluation(){

			//ros::NodeHandle nh;
			//subExternalEvents= nh.subscribe<misc_msgs::ExternalEvent>("externalEvent",5, &EventEvaluation::onReceiveExternalEvent, this); // Ovdej si uvao promjenu s 5 na 1
			//subStateHatAbs= nh.subscribe<auv_msgs::NavSts>("stateHatAbs",1, &EventEvaluation::onStateHat, this);


			//externalEventContainer.resize(5); // 5 eksternih evenata

			//setExternalEvents();

			/* Initialize symbol table */
			initializeSymbolTable();
		}

		void EventEvaluation::evaluateEvents(std::vector<std::string> events, std::vector<std::string>& eventsValue){
			int i = 0;
			for(std::vector<std::string>::iterator it = events.begin() ; it != events.end(); ++it){

				eventsValue[i] = evaluateStringExpression((*it).c_str());
				i++;
			}
		}


		double EventEvaluation::evaluateStringExpression(std::string expression_str){

			expression_t expression;
			expression.register_symbol_table(symbol_table);

			parser_t parser;

			if (!parser.compile(expression_str,expression))
			{
			  ROS_ERROR("Error: %s\tExpression: %s\n",parser.error().c_str(),expression_str.c_str());

			  for (std::size_t i = 0; i < parser.error_count(); ++i)
			  {
				 error_t error = parser.get_error(i);
				 ROS_ERROR("Error: %02d Position: %02d Type: [%s] Msg: %s Expr: %s\n",
						static_cast<int>(i),
						static_cast<int>(error.token.position),
						exprtk::parser_error::to_str(error.mode).c_str(),
						error.diagnostic.c_str(),
						expression_str.c_str());
			  }
			  return -1;
			}

			return expression.value();
		}

		void EventEvaluation::initializeSymbolTable(){

			symbol_table.add_constants();

			symbol_table.add_variable("u",0);
			symbol_table.add_variable("v",0);
			symbol_table.add_variable("w",0);
			symbol_table.add_variable("r",0);
			symbol_table.add_variable("x",0);
			symbol_table.add_variable("y",0);
			symbol_table.add_variable("z",0);
			symbol_table.add_variable("psi",0);
			symbol_table.add_variable("x_var",0);
			symbol_table.add_variable("y_var",0);
			symbol_table.add_variable("z_var",0);
			symbol_table.add_variable("psi_var",0);
			symbol_table.add_variable("alt",0);

			int i = 0;
			for(std::vector<double>::iterator it = eventsVar.begin() ; it != eventsVar.end(); ++it){

				i++;
				std::string eventName = "event";
				eventName.append(static_cast<ostringstream*>( &(ostringstream() << i) )->str());
				double value = 0;
				//std::string eventName = (*it).description.c_str();
				symbol_table.create_variable(eventName.c_str());
				symbol_table.get_variable(eventName.c_str())->ref() = double(value);
			}
		}

		void EventEvaluation::updateSymbolTable(std::vector<double> stateVar, std::vector<double> missionVar, std::vector<double> eventsVar){

				symbol_table.get_variable("u")->ref() = stateVar[u];
				symbol_table.get_variable("v")->ref() = stateVar[v];
				symbol_table.get_variable("w")->ref() = stateVar[w];
				symbol_table.get_variable("r")->ref() = stateVar[r];
				symbol_table.get_variable("x")->ref() = stateVar[x];
				symbol_table.get_variable("y")->ref() = stateVar[y];
				symbol_table.get_variable("z")->ref() = stateVar[z];
				symbol_table.get_variable("psi")->ref() = stateVar[psi];
				symbol_table.get_variable("x_var")->ref() = stateVar[x_var];
				symbol_table.get_variable("y_var")->ref() = stateVar[y_var];
				symbol_table.get_variable("z_var")->ref() = stateVar[z_var];
				symbol_table.get_variable("psi_var")->ref() = stateVar[psi_var];
				symbol_table.get_variable("alt")->ref() = stateVar[alt];

				int i = 0;
				for(std::vector<double>::iterator it = eventsVar.begin() ; it != eventsVar.end(); ++it){

					i++;
					std::string eventName = "event";
					eventName.append(static_cast<ostringstream*>( &(ostringstream() << i) )->str());
					double value = (*it);
					//std::string eventName = (*it).description.c_str();

					//symbol_table.create_variable(eventName.c_str());
					symbol_table.get_variable(eventName.c_str())->ref() = double(value);
				}
		}

//		void EventEvaluation::onReceiveExternalEvent(const misc_msgs::ExternalEvent::ConstPtr& data){
//
//			misc_msgs::ExternalEvent tmp;
//			tmp = externalEventContainer.at((data->id)-1);
//			tmp.id = data->id;
//			//tmp.description = data->description; // overwrites event name
//			tmp.value = data->value;
//			externalEventContainer.at((data->id)-1) = tmp;
//
//			//ROS_ERROR("EVENT: %d, VRIJEDNOST: %f", tmp.id, tmp.value);
//
//		}

//		void EventEvaluation::setExternalEvents(){
//
//			int i = 0;
//
//			for(std::vector<misc_msgs::ExternalEvent>::iterator it = externalEventContainer.begin() ; it != externalEventContainer.end(); ++it){
//				std::string externName = "event";
//				i++;
//				externName.append(static_cast<ostringstream*>( &(ostringstream() << i) )->str());
//
//				(*it).id = i;
//				(*it).description = externName.c_str();
//				(*it).value = 0;
//				ROS_ERROR("%d, %s",i,externName.c_str());
//			}
//		}
	}
}


#endif /* EVENTEVALUATION_HPP_ */
