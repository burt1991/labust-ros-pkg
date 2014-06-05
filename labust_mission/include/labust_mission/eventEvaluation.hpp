//\todo omoguci overide external event descriptiona
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

			void updateData(auv_msgs::NavSts data, symbol_table_t *symbol_table);

			void onReceiveExternalEvent(const misc_msgs::ExternalEvent::ConstPtr& data);

			void setExternalEvents();

			/*********************************************************************
			 ***  Class variables
			 ********************************************************************/

			ros::Subscriber subExternalEvents_;

			std::vector<misc_msgs::ExternalEvent> externalEventContainer;

		};

		EventEvaluation::EventEvaluation(){

			ros::NodeHandle nh;
			subExternalEvents_ = nh.subscribe<misc_msgs::ExternalEvent>("externalEvent",1, &EventEvaluation::onReceiveExternalEvent, this);

			externalEventContainer.resize(5); // 5 eksternih evenata

			setExternalEvents();
		}

		int EventEvaluation::checkEventState(auv_msgs::NavSts data, std::string expression_str){

			symbol_table_t symbol_table;
			//updateData(data, &symbol_table);

			/* Read estimated values */
			double u = data.body_velocity.x;
			double v = data.body_velocity.y;
			double w = data.body_velocity.z;
			double r = data.orientation_rate.yaw;
			double x = data.position.north;
			double y = data.position.east;
			double z = data.position.depth;
			double psi = data.orientation.yaw;

			double x_var = data.position_variance.north;
			double y_var = data.position_variance.east;
			double z_var = data.position_variance.depth;
			double psi_var = data.orientation_variance.yaw;

			double alt = data.altitude;

			symbol_table.add_constants();
			symbol_table.add_variable("u",u);
			symbol_table.add_variable("v",v);
			symbol_table.add_variable("w",w);
			symbol_table.add_variable("r",r);
			symbol_table.add_variable("x",x);
			symbol_table.add_variable("y",y);
			symbol_table.add_variable("z",z);
			symbol_table.add_variable("psi",psi);
			symbol_table.add_variable("x_var",x_var);
			symbol_table.add_variable("y_var",y_var);
			symbol_table.add_variable("z_var",z_var);
			symbol_table.add_variable("psi_var",psi_var);
			symbol_table.add_variable("alt",alt);

			for(std::vector<misc_msgs::ExternalEvent>::iterator it = externalEventContainer.begin() ; it != externalEventContainer.end(); ++it){

				double value = (*it).value;
				symbol_table.add_variable((*it).description.c_str(),value);
			}

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

			double result = expression.value();

			//ROS_ERROR("Result: %10.5f\n",result);
			if(result == 1)
				return 1;
			else
				return 0;
		}

		void EventEvaluation::updateData(auv_msgs::NavSts data, symbol_table_t *symbol_table){

//			/* Read estimated values */
//			double u = data.body_velocity.x;
//			double v = data.body_velocity.y;
//			double w = data.body_velocity.z;
//			double r = data.orientation_rate.yaw;
//			double x = data.position.north;
//			double y = data.position.east;
//			double z = data.position.depth;
//			double psi = data.orientation.yaw;
//
//			double x_var = data.position_variance.north;
//			double y_var = data.position_variance.east;
//			double z_var = data.position_variance.depth;
//			double psi_var = data.orientation_variance.yaw;
//
//			double alt = data.altitude;
//
//			ROS_ERROR("x: %f", x);
//
//			symbol_table->add_constants();
//			symbol_table->add_variable("u",u);
//			symbol_table->add_variable("v",v);
//			symbol_table->add_variable("w",w);
//			symbol_table->add_variable("r",r);
//			symbol_table->add_variable("x",x);
//			symbol_table->add_variable("y",y);
//			symbol_table->add_variable("z",z);
//			symbol_table->add_variable("psi",psi);
//			symbol_table->add_variable("x_var",x_var);
//			symbol_table->add_variable("y_var",y_var);
//			symbol_table->add_variable("z_var",z_var);
//			symbol_table->add_variable("psi_var",psi_var);
//			symbol_table->add_variable("alt",alt);


		}

		void EventEvaluation::onReceiveExternalEvent(const misc_msgs::ExternalEvent::ConstPtr& data){

			misc_msgs::ExternalEvent tmp;
			tmp = externalEventContainer.at((data->id)-1);
			tmp.id = data->id;
			tmp.description = data->description;
			tmp.value = data->value;
			externalEventContainer.at((data->id)-1) = tmp;

		}

		void EventEvaluation::setExternalEvents(){

			int i = 0;
			std::string externName = "event";

			for(std::vector<misc_msgs::ExternalEvent>::iterator it = externalEventContainer.begin() ; it != externalEventContainer.end(); ++it){

				i++;
				externName.append(static_cast<ostringstream*>( &(ostringstream() << i) )->str());

				(*it).id = i;
				(*it).description = externName.c_str();
				(*it).value = 0;
			}
		}
	}
}


#endif /* EVENTEVALUATION_HPP_ */
