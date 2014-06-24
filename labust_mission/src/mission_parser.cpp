//\todo napravi da parser pamti pokazivac na zadnji poslani node koko se ne bi svaki put trebalo parsati korz cijeli xml
//\todo napraviti missionParser (tj klasu koja direktno poziva primitive) klasu tako da extenda razlicite klase za parsanje (u buducnosti pojednostavljeno
//prebacivanje na razlicite mission planere)
//\todo Pogledja treba li poslati sve evente kao jednu poruku na poectku, kako bi se kasnije smanjila kolicina podataka koja se salje skupa s primitivom
//\TODO Dodati mogucnost odabira vise evenata na koje primitiv može reagirati. (Nema potrebe za svaki slučaj nanovo definirati event)

/*********************************************************************
 * mission_parser.cpp
 *
 *  Created on: Apr 18, 2014
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

#include <labust_mission/labustMission.hpp>
#include <labust_mission/eventEvaluation.hpp>

#include <misc_msgs/StartParser.h>
#include <tinyxml2.h>

namespace ser = ros::serialization;

using namespace std;
using namespace tinyxml2;

namespace labust {
	namespace mission {

		/*********************************************************************
		 ***  MissionParser class definition
		 ********************************************************************/

		class MissionParser{

		public:

			/*****************************************************************
			 ***  Class functions
			 ****************************************************************/

			MissionParser(ros::NodeHandle& nh);

			void sendPrimitve();

			void go2pointFA(double north, double east, double heading, double speed, double victoryRadius);

			void go2pointUA(double north, double east, double speed, double victoryRadius);

			void dynamicPositioning(double north, double east, double heading);

			void courseKeepingFA(double course, double heading, double speed);

			void courseKeepingUA(double course, double speed);

			int parseMission(int id, string xmlFile);

			int parseEvents(string xmlFile);

			int parseMissionParam(string xmlFile);

			void onRequestPrimitive(const std_msgs::UInt16::ConstPtr& req);

			void onEventString(const std_msgs::String::ConstPtr& msg);

			void onReceiveXmlPath(const misc_msgs::StartParser::ConstPtr& msg);

			/*****************************************************************
			 ***  Helper functions
			 ****************************************************************/

			template <typename primitiveType>
			void serializePrimitive(int id, primitiveType data);

			/*****************************************************************
			 ***  Class variables
			 ****************************************************************/

			int ID, lastID;
			double newXpos, newYpos, newVictoryRadius, newSpeed, newCourse, newHeading;
			//double oldXpos, oldYpos, oldVictoryRadius, oldSpeed, oldCourse, oldHeading;
			double newTimeout;

			bool eventsFlag, primitiveHasEvent;

			int eventID;

			string xmlFile;

			std::vector<std::string> eventsContainer;

			std::vector<uint8_t> eventsActive;
			//std::vector<std::string> eventsActiveContainer;
			std::vector<uint8_t> eventsGoToNext;

			ros::Publisher pubSendPrimitive, pubRiseEvent, pubMissionOffset;
			ros::Subscriber subRequestPrimitive, subEventString, subReceiveXmlPath;

			auv_msgs::NED offset;

			labust::event::EventEvaluation EE;

			int breakpoint;
		};


		/*****************************************************************
		 ***  Class functions
		 ****************************************************************/

		MissionParser::MissionParser(ros::NodeHandle& nh):ID(0), lastID(0), newXpos(0), newYpos(0), newVictoryRadius(0), newSpeed(0),
				newCourse(0), newHeading(0), newTimeout(0), eventsFlag(false), primitiveHasEvent(false), eventID(0), breakpoint(1){

			/* Subscribers */
//			subRequestPrimitive = nh.subscribe<std_msgs::Bool>("requestPrimitive",1,&MissionParser::onRequestPrimitive, this);
			subRequestPrimitive = nh.subscribe<std_msgs::UInt16>("requestPrimitive",1,&MissionParser::onRequestPrimitive, this);
			subEventString = nh.subscribe<std_msgs::String>("eventString",1,&MissionParser::onEventString, this);
			subReceiveXmlPath = nh.subscribe<misc_msgs::StartParser>("startParse",1,&MissionParser::onReceiveXmlPath, this);


			/* Publishers */
			pubSendPrimitive = nh.advertise<misc_msgs::SendPrimitive>("sendPrimitive",1);
			pubRiseEvent = nh.advertise<std_msgs::String>("eventString",1);
			pubMissionOffset = nh.advertise<auv_msgs::NED>("missionOffset",1);


			/* Parse file path */
			ros::NodeHandle ph("~");
			xmlFile = "mission.xml";
			ph.param("xml_save_path", xmlFile, xmlFile);
		}

		void MissionParser::sendPrimitve(){


			ROS_ERROR("%s",xmlFile.c_str());

			ROS_ERROR("%d",ID);
			int status = parseMission(ID, xmlFile);

			ROS_ERROR("%s", primitives[status]);

			switch(status){

				case go2point_FA:

					ROS_ERROR("T2 = %f,%f, Heading = %f, Speed = %f, Victory radius = %f", newXpos, newYpos, newHeading, newSpeed, newVictoryRadius);
					go2pointFA(newXpos, newYpos, newHeading, newSpeed, newVictoryRadius);
					break;

				case go2point_UA:

					ROS_ERROR("T2 = %f,%f, Speed = %f, Victory radius = %f", newXpos, newYpos, newSpeed, newVictoryRadius);
					go2pointUA(newXpos, newYpos, newSpeed, newVictoryRadius);
					break;

				case dynamic_positioning:

					ROS_ERROR("T2 = %f,%f, Heading = %f", newXpos, newYpos, newHeading);
					dynamicPositioning(newXpos, newYpos, newHeading);
					break;

				case course_keeping_FA:
					ROS_ERROR("Course = %f, Heading = %f, Speed = %f", newCourse, newHeading, newSpeed);
					courseKeepingFA(newCourse, newHeading, newSpeed);
					break;

				case course_keeping_UA:

					ROS_ERROR("Course = %f, Speed = %f", newCourse, newSpeed);
					courseKeepingUA(newCourse, newSpeed);
					break;

				case none:

					ROS_ERROR("Mission ended.");
					std_msgs::String tmp;
					tmp.data = "/STOP";
					pubRiseEvent.publish(tmp);

					/* Reset file path */
					ros::NodeHandle ph("~");
					xmlFile = "mission.xml";
					ph.param("xml_save_path", xmlFile, xmlFile);
			}
		}

		void MissionParser::go2pointFA(double north, double east, double heading, double speed, double victoryRadius){

			misc_msgs::Go2PointFA data;
			data.point.north = north-offset.north;
			data.point.east = east-offset.east;
			data.point.depth = 0;
			data.heading = heading;
			data.speed = speed;
			data.victoryRadius = victoryRadius;

			serializePrimitive<misc_msgs::Go2PointFA>(go2point_FA, data);
		}

		void MissionParser::go2pointUA(double north, double east, double speed, double victoryRadius){

			misc_msgs::Go2PointUA data;
			data.point.north = north-offset.north;
			data.point.east = east-offset.east;
			data.point.depth = 0;
			data.speed = speed;
			data.victoryRadius = victoryRadius;

			serializePrimitive<misc_msgs::Go2PointUA>(go2point_UA, data);
		}

		void MissionParser::dynamicPositioning(double north, double east, double heading){

			misc_msgs::DynamicPositioning data;
			data.point.north = north-offset.north;
			data.point.east = east-offset.east;
			data.point.depth = 0;
			data.heading = heading;

			serializePrimitive<misc_msgs::DynamicPositioning>(dynamic_positioning, data);

		}

		void MissionParser::courseKeepingFA(double course, double heading, double speed){

			misc_msgs::CourseKeepingFA data;
			data.course = course;
			data.heading = heading;
			data.speed = speed;

			serializePrimitive<misc_msgs::CourseKeepingFA>(course_keeping_FA, data);
		}

		void MissionParser::courseKeepingUA(double course, double speed){

			misc_msgs::CourseKeepingUA data;
			data.course = course;
			data.speed = speed;

			serializePrimitive<misc_msgs::CourseKeepingUA>(course_keeping_UA, data);
		}


		int MissionParser::parseMission(int id, string xmlFile){

		   XMLDocument xmlDoc;

		   XMLNode *mission;
		   XMLNode *primitive;
		   XMLNode *primitiveParam;

		   /* Open XML file */
		   if(xmlDoc.LoadFile(xmlFile.c_str()) == XML_SUCCESS) {

			   /* Find mission node */
			   mission = xmlDoc.FirstChildElement("mission");
			   if(mission){

				   /* Loop through primitive nodes */
				   primitive = mission->FirstChildElement("primitive");
				   do{

					   XMLElement *elem = primitive->ToElement();
					   string primitiveName = elem->Attribute("name");
					   ROS_INFO("%s", primitiveName.c_str());

					   primitiveParam = primitive->FirstChildElement("id");
					   XMLElement *elemID = primitiveParam->ToElement();

					   /* If ID is correct process primitive data */
					   string id_string = static_cast<ostringstream*>( &(ostringstream() << id) )->str();
					   string tmp = elemID->GetText();

					   if (tmp.compare(id_string) == 0){

						   /* Reset data */
							newTimeout = 0;
							eventsActive.clear();
							eventsGoToNext.clear();

						   /* Case: go2point_FA *****************************/
						   if(primitiveName.compare("go2point_FA") == 0){

							   primitiveParam = primitive->FirstChildElement("param");
							   do{

								   XMLElement *elem2 = primitiveParam->ToElement();
								   string primitiveParamName = elem2->Attribute("name");
								   //ROS_ERROR("%s", primitiveParamName.c_str());

								   if(primitiveParamName.compare("north") == 0){

									   newXpos = EE.evaluateStringExpression(elem2->GetText(), 0);

								   } else if(primitiveParamName.compare("east") == 0){

									   newYpos = EE.evaluateStringExpression(elem2->GetText(), 0);

								   } else if(primitiveParamName.compare("speed") == 0){

									   newSpeed = atof(elem2->GetText());

								   } else if(primitiveParamName.compare("victory_radius") == 0){

									   newVictoryRadius = atof(elem2->GetText());

								   } else if(primitiveParamName.compare("heading") == 0){

									   newHeading = atof(elem2->GetText());

								   } else if(primitiveParamName.compare("onEventStop") == 0){

									    std::string goToId = elem2->Attribute("goToId");
									    ROS_ERROR("gottoid aatribut %s",goToId.c_str());
									    if(goToId.empty()==0){
									   if(strcmp(goToId.c_str(),"breakpoint") == 0){
										   eventsGoToNext.push_back(breakpoint);
									   }else{
										   eventsGoToNext.push_back(atoi(goToId.c_str()));
									   }
								   } else {
									   eventsGoToNext.push_back(ID+1);
								   }


										eventsActive.push_back(atof(elem2->GetText()));
										//eventID = atof(elem2->GetText());
										primitiveHasEvent = true;
								   }
							   } while(primitiveParam = primitiveParam->NextSiblingElement("param"));

							   return go2point_FA;

							/* Case: go2point_UA ****************************/
							} else if (primitiveName.compare("go2point_UA") == 0){

							   primitiveParam = primitive->FirstChildElement("param");
							   do{

								   XMLElement *elem2 = primitiveParam->ToElement();
								   string primitiveParamName = elem2->Attribute("name");

								   if(primitiveParamName.compare("north") == 0){

									   newXpos = atof(elem2->GetText());

								   } else if(primitiveParamName.compare("east") == 0){

									   newYpos = atof(elem2->GetText());

								   } else if(primitiveParamName.compare("speed") == 0){

									   newSpeed = atof(elem2->GetText());

								   } else if(primitiveParamName.compare("victory_radius") == 0){

									   newVictoryRadius = atof(elem2->GetText());

								   } else if(primitiveParamName.compare("onEventStop") == 0){

									    std::string goToId = elem2->Attribute("goToId");
									    ROS_ERROR("gottoid aatribut %s",goToId.c_str());
										if(goToId.empty()==0){
										   eventsGoToNext.push_back(atoi(goToId.c_str()));
										} else {
										   eventsGoToNext.push_back(ID+1);
										}

										eventsActive.push_back(atof(elem2->GetText()));
										//eventID = atof(elem2->GetText());
										primitiveHasEvent = true;
								   }
							   } while(primitiveParam = primitiveParam->NextSiblingElement("param"));

							   return go2point_UA;

							/* Case: dynamic_positioning ********************/
							} else if (primitiveName.compare("dynamic_positioning") == 0){

							   primitiveParam = primitive->FirstChildElement("param");
							   do{

								   XMLElement *elem2 = primitiveParam->ToElement();
								   string primitiveParamName = elem2->Attribute("name");
								   //ROS_ERROR("%s", primitiveParamName.c_str());

								   if(primitiveParamName.compare("north") == 0){

									   newXpos = EE.evaluateStringExpression(elem2->GetText(), 0);

								   } else if(primitiveParamName.compare("east") == 0){

									   newYpos = EE.evaluateStringExpression(elem2->GetText(), 0);

								   } else if(primitiveParamName.compare("heading") == 0){

									   newHeading = atof(elem2->GetText());

								   } else if(primitiveParamName.compare("timeout") == 0){

									  newTimeout = atof(elem2->GetText());

								   } else if(primitiveParamName.compare("onEventStop") == 0){

									    std::string goToId = elem2->Attribute("goToId");
									    ROS_ERROR("gottoid aatribut %s",goToId.c_str());
										if(goToId.empty()==0){
											 if(strcmp(goToId.c_str(),"breakpoint") == 0){
												   eventsGoToNext.push_back(breakpoint);
											   }else{
												   eventsGoToNext.push_back(atoi(goToId.c_str()));
												// breakpoint = ID;
											   }

										} else {
										   eventsGoToNext.push_back(ID+1);
										}

										eventsActive.push_back(atof(elem2->GetText()));
										//eventID = atof(elem2->GetText());
										primitiveHasEvent = true;
								   }
							   } while(primitiveParam = primitiveParam->NextSiblingElement("param"));

								return dynamic_positioning;

							/* Case: course_keeping_FA **********************/
							}else if (primitiveName.compare("course_keeping_FA") == 0){

							   primitiveParam = primitive->FirstChildElement("param");
							   do{

								   XMLElement *elem2 = primitiveParam->ToElement();
								   string primitiveParamName = elem2->Attribute("name");
								   //ROS_ERROR("%s", primitiveParamName.c_str());

								   if(primitiveParamName.compare("course") == 0){

									  // newCourse = atof(elem2->GetText());
									   newCourse = EE.evaluateStringExpression(elem2->GetText(), newCourse);


								   } else if(primitiveParamName.compare("speed") == 0){

									   newSpeed = atof(elem2->GetText());

								   } else if(primitiveParamName.compare("heading") == 0){

									   newHeading = atof(elem2->GetText());

								   } else if(primitiveParamName.compare("timeout") == 0){

									  newTimeout = atof(elem2->GetText());

								   } else if(primitiveParamName.compare("onEventStop") == 0){

									   std::string goToId = elem2->Attribute("goToId");
									   if(goToId.empty()==0){
										   if(strcmp(goToId.c_str(),"breakpoint") == 0){
											   eventsGoToNext.push_back(breakpoint);
										   }else{
											   eventsGoToNext.push_back(atoi(goToId.c_str()));
										   }
									   } else {
										   eventsGoToNext.push_back(ID+1);
									   }

									   eventsActive.push_back(atof(elem2->GetText()));
									   //eventID = atof(elem2->GetText());
									   primitiveHasEvent = true;
								   }
							   } while(primitiveParam = primitiveParam->NextSiblingElement("param"));

							   return course_keeping_FA;

							/* Case: course_keeping_UA **********************/
							}else if (primitiveName.compare("course_keeping_UA") == 0){

							   primitiveParam = primitive->FirstChildElement("param");
							   do{

								   XMLElement *elem2 = primitiveParam->ToElement();
								   string primitiveParamName = elem2->Attribute("name");
								   //ROS_ERROR("%s", primitiveParamName.c_str());

								   if(primitiveParamName.compare("course") == 0){

									   //newCourse = atof(elem2->GetText());
									   newCourse = EE.evaluateStringExpression(elem2->GetText(), newCourse);

								   } else if(primitiveParamName.compare("speed") == 0){

									   newSpeed = atof(elem2->GetText());

								   } else if(primitiveParamName.compare("timeout") == 0){

									  newTimeout = atof(elem2->GetText());

								   } else if(primitiveParamName.compare("onEventStop") == 0){

									   std::string goToId = elem2->Attribute("goToId");
									   if(goToId.empty()==0){
										   if(strcmp(goToId.c_str(),"breakpoint") == 0){
											   eventsGoToNext.push_back(1);
										   }else{
											   eventsGoToNext.push_back(atoi(goToId.c_str()));
										   }
									   } else {
										   eventsGoToNext.push_back(ID+1);
									   }

									   eventsActive.push_back(atof(elem2->GetText()));
									   //eventID = atof(elem2->GetText());
									   primitiveHasEvent = true;
								   }
							   } while(primitiveParam = primitiveParam->NextSiblingElement("param"));

							   return course_keeping_UA;
						  }
					   }
				   } while(primitive = primitive->NextSiblingElement("primitive"));

				   return none;

			   } else {
				   ROS_ERROR("No mission defined");
				   return -1;
			   }
		   } else {
			   ROS_ERROR("Cannot open XML file!");
			   return -1;
		   }
		}

		int MissionParser::parseEvents(string xmlFile){

		   XMLDocument xmlDoc;

		   XMLNode *events;
		   XMLNode *event;
		   XMLNode *primitiveParam;

		   /* Open XML file */
		   if(xmlDoc.LoadFile(xmlFile.c_str()) == XML_SUCCESS) {

			   /* Find events node */
			   events = xmlDoc.FirstChildElement("events");
			   if(events){
				   for (event = events->FirstChildElement("event"); event != NULL; event = event->NextSiblingElement()){

					   eventsContainer.push_back(event->ToElement()->GetText());
				   }
				   eventsFlag = true;
			   } else {
				   ROS_ERROR("No events defined");
				   return -1;
			   }
		   } else {
			   ROS_ERROR("Cannot open XML file!");
			   return -1;
		   }

		   // debug PRINT

		   for(std::vector<std::string>::iterator it = eventsContainer.begin() ; it != eventsContainer.end(); ++it){

				std::string vTmp = *it;
				ROS_ERROR("Event string: %s", vTmp.c_str());
			}
		}


		int MissionParser::parseMissionParam(string xmlFile){

//		   XMLDocument xmlDoc;
//
//		   XMLNode *events;
//		   XMLNode *event;
//		   XMLNode *primitiveParam;
//
//		   /* Open XML file */
//		   if(xmlDoc.LoadFile(xmlFile.c_str()) == XML_SUCCESS) {
//
//			   /* Find events node */
//			   events = xmlDoc.FirstChildElement("events");
//			   if(events){
//				   for (event = events->FirstChildElement("event"); event != NULL; event = event->NextSiblingElement()){
//
//					   eventsContainer.push_back(event->ToElement()->GetText());
//				   }
//				   eventsFlag = true;
//			   } else {
//				   ROS_ERROR("No events defined");
//				   return -1;
//			   }
//		   } else {
//			   ROS_ERROR("Cannot open XML file!");
//			   return -1;
//		   }
//
//		   // debug PRINT
//
//		   for(std::vector<std::string>::iterator it = eventsContainer.begin() ; it != eventsContainer.end(); ++it){
//
//				std::string vTmp = *it;
//				ROS_ERROR("Event string: %s", vTmp.c_str());
//			}
		}

		void MissionParser::onRequestPrimitive(const std_msgs::UInt16::ConstPtr& req){

			if(req->data){
				lastID = ID;
				ID = req->data;
				if(abs(ID-lastID)>1){
					breakpoint = lastID;
					ROS_ERROR("DEBUG: New breakpoint %d", breakpoint);
				}

				sendPrimitve();
			}
		}

		void MissionParser::onEventString(const std_msgs::String::ConstPtr& msg){

			if(strcmp(msg->data.c_str(),"/STOP") == 0){
				ID = 0;
				eventsContainer.clear();
			}
		}

		void MissionParser::onReceiveXmlPath(const misc_msgs::StartParser::ConstPtr& msg){

			if(msg->fileName.empty() == 0){
				xmlFile.assign(msg->fileName.c_str());

				if(msg->relative){
					offset.north = -msg->startPosition.north;
					offset.east = -msg->startPosition.east;
				} else {
					offset.north = offset.east = 0;
				}

				pubMissionOffset.publish(offset);

				parseEvents(xmlFile.c_str());
			}
		}


		/*****************************************************************
		 ***  Helper functions
		 ****************************************************************/

		template <typename primitiveType>
		void MissionParser::serializePrimitive(int id, primitiveType data){

			uint32_t serial_size = ros::serialization::serializationLength(data);
			std::vector<uint8_t> buffer(serial_size);

			ser::OStream stream(&buffer.front(), serial_size);
			ser::serialize(stream, data);

			misc_msgs::SendPrimitive sendContainer;
			sendContainer.primitiveID = id;
			sendContainer.primitiveData = buffer;

			std::string EventsContainerTmp;

			for(std::vector<uint8_t>::iterator it = eventsActive.begin() ; it != eventsActive.end(); ++it){

				EventsContainerTmp.append(eventsContainer.at(*it-1));
				EventsContainerTmp.append(":");

				ROS_ERROR("%s", EventsContainerTmp.c_str());
			}

			sendContainer.event.timeout = newTimeout;

			ROS_ERROR("Evo koji eventi su aktivni: %s", EventsContainerTmp.c_str());

			if(eventsFlag && primitiveHasEvent){
				sendContainer.event.onEventStop = EventsContainerTmp.c_str();
			} else {
				sendContainer.event.onEventStop = "";
			}

			sendContainer.event.onEventNext = eventsGoToNext;

			//sendContainer.event.onEventStop = (eventsFlag && primitiveHasEvent) ? eventsContainer.at(eventID-1).c_str():"";

			primitiveHasEvent = false;

			pubSendPrimitive.publish(sendContainer);
		}
	}
}


/*********************************************************************
 ***  Main function
 ********************************************************************/

int main(int argc, char** argv){

	ros::init(argc, argv, "mission_parser");
	ros::NodeHandle nh;

	using namespace labust::mission;
	MissionParser MP(nh);

	ros::spin();
	return 0;
}






