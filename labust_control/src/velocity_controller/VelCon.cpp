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
#include <labust/control/PIFFController.h>
#include <labust/math/NumberManipulation.hpp>
#include <labust/tools/MatrixLoader.hpp>
#include <labust/simulation/DynamicsParams.hpp>
#include <labust/tools/DynamicsLoader.hpp>
#include <labust/tools/conversions.hpp>
#include <labust/control/ControlTopicHandler.hpp>
#include <labust/control/TopicMapper.hpp>
#include <navcon_msgs/RegisterController_v3.h>

#include <Eigen/Dense>
#include <auv_msgs/BodyForceReq.h>
#include <auv_msgs/BodyVelocityReq.h>
#include <auv_msgs/NavSts.h>
#include <ros/ros.h>

#include <boost/bind.hpp>
#include <boost/array.hpp>

namespace labust
{
	namespace control
	{
		class VelocityController
		{
			enum {max_dof=6};
			typedef boost::array<double, max_dof> vector;
		public:
			VelocityController():
				Ts(0.1)
		{
				this->onInit();
		}

			void onInit()
			{
				//Register the controllers from this node
				const char* names[]={"surge", "sway", "heave", "yaw_rate"};
				int dofidx[4]={0,1,2,5};

				ros::NodeHandle nh,ph("~");

				handler.init(nh, ph, boost::bind(&VelocityController::onTick, this,_1));
				cstateTracker.init(nh, ph, boost::bind(&VelocityController::onControllerState, this,_1,_2));

				for (int i=0; i<4; ++i)
				{
					dofs.push_back(dofidx[i]);
					this->names.push_back(names[i]);
					pids.push_back(PIDBase());

					//Register controller
					navcon_msgs::RegisterController_v3Request info;
					info.parent = ros::this_node::getName();
					info.name = names[i];
					info.used_tau[dofidx[i]] = 1;
					cstateTracker.registerController(info);
				}

				this->configure(nh);
			}

			VelConTypes::OutputType::Ptr onTick(const ControlTopicHandler<VelConTypes>::Topics& topics)
			{
				ROS_INFO("Tick");
				bool publish(false);
				vector out;
				out.assign(0);

				//Get the topic elements
				vector manual, tracking, state, ref, windup;
				VelConTypes::getOutput(*topics.cManual, manual);
				VelConTypes::getState(*topics.cState, state);
				VelConTypes::getOutput(*topics.cTracking, tracking);
				VelConTypes::getReference(*topics.cReference,ref);
				//feedforward
				//windup
				labust::tools::disableAxisToVector(topics.cTracking->disable_axis, windup);

				for (int i=0; i<dofs.size();++i)
				{
					//Check if some of the axis are timed-out ?

					int dof = dofs[i];

					switch (currentState[names[i]])
					{
					case navcon_msgs::ControllerInfo::DISABLED:
						ROS_INFO("Disabled.");
						break;
					case navcon_msgs::ControllerInfo::MANUAL:
						ROS_INFO("Manual output.");
						out[dof] = manual[dof];
						//The controller does tracking internally in manual
						pids[i].desired = ref[dof];
						pids[i].state = state[dof];
						PIFF_step(&pids[i], Ts);
						pids[i].internalState = manual[dof];
						pids[i].output = manual[dof];

						publish = true;
						break;
					case navcon_msgs::ControllerInfo::EXTERNAL:
						//Determine windup
						pids[i].desired = ref[dof];
						pids[i].state = state[dof];
						//pids[i].extWindup = windup[dof];
						PIFF_step(&pids[i], Ts);
						out[dof] = pids[i].output;
						ROS_INFO("Controller %d::%d : %f %f %f",	i, dof, ref[dof], state[dof], out[dof]);
						publish = true;
						break;
					case navcon_msgs::ControllerInfo::TRACKING:
						ROS_INFO("Tracking %d : %f %f %f",	i, ref[dof], state[dof], tracking[dof]);
						//pids[dof].tracking(ref[dof], state[dof], tracking[dof]);
						pids[i].desired = ref[dof];
						pids[i].state = state[dof];
						PIFF_step(&pids[i], Ts);
						pids[i].internalState = tracking[dof];
						pids[i].output = tracking[dof];
						break;
					default:
						break;
					}
				}

				VelConTypes::OutputType::Ptr retVal;
				if (publish)
				{
					//map array to output type
					retVal.reset(new VelConTypes::OutputType());
					VelConTypes::setOutput(out, *retVal);
				}

				return retVal;
			}

			void onControllerState(const std::string& name, const navcon_msgs::ControllerInfo& info)
			{
				//Update topic subscriptions
				handler.updateTopics(info);
				//Update the state
				currentState[name] = info.state;
			}

		private:

			void configure(ros::NodeHandle nh)
			{
				typedef Eigen::Matrix<double,6,1> Vector6d;
				using labust::simulation::vector;
				vector closedLoopFreq(vector::Ones());
				labust::tools::getMatrixParam(nh,"velocity_controller/closed_loop_freq", closedLoopFreq);
				vector autoTracking(vector::Zero());
				labust::tools::getMatrixParam(nh,"velocity_controller/auto_tracking", autoTracking);
				vector outputLimit(vector::Zero());
				labust::tools::getMatrixParam(nh,"velocity_controller/output_limits", outputLimit);
				nh.param("velocity_controller/Ts",Ts,Ts);

				labust::simulation::DynamicsParams model;
				labust::tools::loadDynamicsParams(nh,model);
				vector alphas(model.Ma.diagonal());
				vector alpha_mass;
				alpha_mass<<model.m,model.m,model.m,model.Io.diagonal();
				alphas += alpha_mass;

				for (int i=0; i<dofs.size(); ++i)
				{
					int dof = dofs[i];
					ROS_INFO("Process %d :: dof: %d, w=%f, at = %f",i, dof,closedLoopFreq(dof), autoTracking(dof));
					PT1Model pt1;
					pt1.alpha = alphas(dof);
					pt1.beta = model.Dlin(dof,dof);
					pt1.betaa = model.Dquad(dof,dof);

					PIDBase_init(&pids[i]);
					PIFF_modelTune(&pids[i], &pt1, float(closedLoopFreq(dof)));
					pids[i].autoWindup = autoTracking(dof);
					pids[i].outputLimit = outputLimit(dof);
					pids[i].useBackward = 1;
				}
			}

			ControlTopicHandler<VelConTypes> handler;
			ControllerStateTracker cstateTracker;
			std::map<std::string, int> currentState;
			std::vector<int> dofs;
			std::vector<std::string> names;
			std::vector<PIDBase> pids;
			double Ts;
		};
	}
}

int main(int argc, char* argv[])
{
	ros::init(argc,argv,"velcon");

	labust::control::VelocityController velcon;

	ros::spin();

	return 0;
}



