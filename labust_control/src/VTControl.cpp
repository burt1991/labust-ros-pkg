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
 *
 *  Author: Dula Nad
 *  Created: 01.02.2013.
 *********************************************************************/
#include <labust/control/HLControl.hpp>
#include <labust/control/EnablePolicy.hpp>
#include <labust/control/WindupPolicy.hpp>
#include <labust/control/PSatDController.h>
#include <labust/control/PIFFController.h>
#include <labust/tools/conversions.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Dense>
#include <auv_msgs/BodyVelocityReq.h>
#include <auv_msgs/BodyForceReq.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <ros/ros.h>

namespace labust
{
	namespace control{
		///The simple line following controller
		struct VTControl : DisableAxis
		{
			VTControl():
				k1(1),
				k2(1),
				kdelta(1),
				Ts(0.1),
				aAngle(M_PI/8),
				wh(0.2),
				underactuated(false),
				listener(buffer){};

			void init()
			{
				initialize_controller();
			}

			void reset(const auv_msgs::NavSts& ref, const auv_msgs::NavSts& state){};

  		void windup(const auv_msgs::BodyForceReq& tauAch)
			{
				//Copy into controller
				//con.windup = tauAch.disable_axis.yaw;
			};

			auv_msgs::BodyVelocityReqPtr step(const auv_msgs::NavSts& ref,
					const auv_msgs::NavSts& state)
			{
				auv_msgs::BodyVelocityReqPtr nu(new auv_msgs::BodyVelocityReq());

				//Get the position in the line frame
				try
				{
					geometry_msgs::TransformStamped sf;
					sf = buffer.lookupTransform("serret_frenet_frame", "base_link", ros::Time(0));
					double s1(sf.transform.translation.x),y1(sf.transform.translation.y);
					double r,p,beta;
					labust::tools::eulerZYXFromQuaternion(sf.transform.rotation, r, p, beta);

					//Calculate desired yaw-rate
					if (underactuated)
					{
						double delta = -aAngle*tanh(kdelta*y1);
						double yaw(ref.orientation.yaw);
						Eigen::Vector2d out, in;
						Eigen::Matrix2d R;
						in<<state.body_velocity.x,state.body_velocity.y;
						R<<cos(yaw),sin(yaw),-sin(yaw),cos(yaw);
						out = R*in;
						double U = sqrt(pow(out(0),2) + pow(out(1),2));
						double ds = U*cos(beta) + k2*s1;
						//Curvature
						double cc = 1/10;
						double dy1 = -cc*ds*s1 + U*sin(beta);

						nu->twist.angular.z = 1/cosh(kdelta*y1)*dy1 - k1*(beta - delta) + cc*ds;
						nu->twist.linear.x = ref.body_velocity.x;
					}
					else
					{	}
				}
				catch (tf2::TransformException& ex)
				{
					ROS_WARN("%s",ex.what());
				}

				nu->header.stamp = ros::Time::now();
				nu->goal.requester = (underactuated)?"uavt_controller":"favt_controller";
				labust::tools::vectorToDisableAxis(disable_axis, nu->disable_axis);

				return nu;
			}

			void initialize_controller()
			{
				ROS_INFO("Initializing VT controller...");
				ros::NodeHandle nh, ph("~");

				sTwist = nh.advertise<geometry_msgs::TwistStamped>("sf_twist",1);

				nh.param("vt_controller/k1",k1,k1);
				nh.param("vt_controller/k2",k2,k2);
				double surge(0.1);
				nh.param("vt_controller/default_surge",surge,surge);
  			nh.param("vt_controller/approach_angle",aAngle,aAngle);
				nh.param("vt_controller/sampling",Ts,Ts);
				ph.param("underactuated",underactuated,underactuated);

				PIDBase_init(&con);

				disable_axis[0] = 0;
				if (underactuated)
				{
					disable_axis[5] = 0;
				}
				else
				{
					disable_axis[1] = 0;
				}

				ROS_INFO("VT controller initialized.");
			}

		private:
			double k1, k2, kdelta;
			double Ts;
			double aAngle, wh;
			bool underactuated;
			tf2_ros::Buffer buffer;
			tf2_ros::TransformListener listener;
			tf2_ros::TransformBroadcaster broadcast;
			ros::Publisher sTwist;
		};
	}}

int main(int argc, char* argv[])
{
	ros::init(argc,argv,"vt_control");

	labust::control::HLControl<labust::control::VTControl,
	labust::control::EnableServicePolicy,
	labust::control::WindupPolicy<auv_msgs::BodyForceReq> > controller;
	ros::spin();

	return 0;
}



