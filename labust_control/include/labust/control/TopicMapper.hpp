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
 *********************************************************************/
#ifndef TOPICMAPPER_HPP_
#define TOPICMAPPER_HPP_
#include <labust/tools/conversions.hpp>

#include <auv_msgs/NavSts.h>
#include <auv_msgs/BodyVelocityReq.h>
#include <auv_msgs/BodyForceReq.h>

namespace labust
{
	namespace control
	{
		/**
		 * The velocity controller topic types and the mapper from the ROS type
		 * to a six element velocity vector.
		 */
		struct VelConTypes
		{
			typedef auv_msgs::BodyForceReq OutputType;
			typedef auv_msgs::BodyForceReq WindupType;

			typedef auv_msgs::BodyVelocityReq ReferenceType;
			typedef auv_msgs::BodyVelocityReq FeedForwardType;

			typedef auv_msgs::NavSts StateType;

			template <class Vector>
			static void getOutput(const OutputType& msg, Vector& vec)
			{
				labust::tools::pointToVector(msg.wrench.force, vec);
				labust::tools::pointToVector(msg.wrench.torque, vec, 3);
			}

			template <class Vector>
			static void getReference(const ReferenceType& msg, Vector& vec)
			{
				labust::tools::pointToVector(msg.twist.linear, vec);
				labust::tools::pointToVector(msg.twist.angular, vec, 3);
			}

			template <class Vector>
			static void getState(const StateType& msg, Vector& vec)
			{
				labust::tools::pointToVector(msg.body_velocity, vec);
				labust::tools::rpyToVector(msg.orientation_rate, vec, 3);
			}

			template <class Vector>
			static void setOutput(const Vector& vec, OutputType& msg)
			{
				labust::tools::vectorToPoint(vec,msg.wrench.force);
				labust::tools::vectorToPoint(vec, msg.wrench.torque, 3);
			}
		};
	}
}

/* TOPICMAPPER_HPP_ */
#endif
