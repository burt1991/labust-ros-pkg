/*********************************************************************
 * utils.hpp
 *
 *  Created on: Jun 16, 2014
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

#ifndef UTILS_HPP_
#define UTILS_HPP_

#include <ros/ros.h>

namespace labust{
	namespace utilities{


		template <typename custom_srv>
		custom_srv callService(ros::ServiceClient& client, custom_srv& request){

			if (client.call(request)){
				ROS_INFO("Call to service %s successful", client.getService().c_str());
				return request;
			} else {
				ROS_ERROR("* Call to service %s failed", client.getService().c_str());
			}
		}

		/*************************************************************
		 ***  Functions used for splitting string expressions
		 *************************************************************/
		std::vector<std::string> &split(const std::string &s, char delim, std::vector<std::string> &elems) {
			std::stringstream ss(s);
			std::string item;
			while (std::getline(ss, item, delim)) {
				if(!item.empty()){
					elems.push_back(item);
				}
			}
			return elems;
		}

		std::vector<std::string> split(const std::string &s, char delim) {
			std::vector<std::string> elems;
			split(s, delim, elems);
			return elems;
		}


//		template <typename primitiveType>
//				void MissionParser::serializePrimitive(int id, primitiveType data){
//
//					uint32_t serial_size = ros::serialization::serializationLength(data);
//					std::vector<uint8_t> buffer(serial_size);
//
//					ser::OStream stream(&buffer.front(), serial_size);
//					ser::serialize(stream, data);
//
//					misc_msgs::SendPrimitive sendContainer;
//					sendContainer.primitiveID = id;
//					sendContainer.primitiveData = buffer;
//
//					std::string EventsContainerTmp;
//
//					for(std::vector<uint8_t>::iterator it = eventsActive.begin() ; it != eventsActive.end(); ++it){
//
//						EventsContainerTmp.append(eventsContainer.at(*it-1));
//						EventsContainerTmp.append(":");
//
//						ROS_ERROR("%s", EventsContainerTmp.c_str());
//					}
//
//					sendContainer.event.timeout = newTimeout;
//
//					ROS_ERROR("Evo koji eventi su aktivni: %s", EventsContainerTmp.c_str());
//
//					if(eventsFlag && primitiveHasEvent){
//						sendContainer.event.onEventStop = EventsContainerTmp.c_str();
//					} else {
//						sendContainer.event.onEventStop = "";
//					}
//
//					sendContainer.event.onEventNext = eventsGoToNext;
//
//					//sendContainer.event.onEventStop = (eventsFlag && primitiveHasEvent) ? eventsContainer.at(eventID-1).c_str():"";
//
//					primitiveHasEvent = false;
//
//					pubSendPrimitive.publish(sendContainer);
//				}
//
//		template <typename primitiveType>
//		primitiveType MissionExecution::deserializePrimitive(std::vector<uint8_t> primitiveData){
//
//			std::vector<uint8_t> my_buffer;
//			primitiveType my_primitive;
//
//			my_buffer = primitiveData;
//
//			uint32_t serial_size = ros::serialization::serializationLength(my_primitive);
//
//			uint8_t *iter = &my_buffer.front();
//
//			ser::IStream stream(iter, serial_size);
//			ser::deserialize(stream, my_primitive);
//
//			return my_primitive;
//		}
	}
}

#endif /* UTILS_HPP_ */
