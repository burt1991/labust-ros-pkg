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
#include <labust/simulation/USBLSim.hpp>

using namespace labust::simulation;

USBLSim::USBLSim()
{
	this->onInit();
}

void USBLSim::onInit()
{
	ros::NodeHandle nh,ph("~");

	//Subscribers
	modemOut = nh.subscribe<underwater_msgs::ModemTransmission>("modem_out", 1,
			&USBLSim::onModemOut, this);

	/*
	//odometry
	tauInWrench = nh.subscribe<geometry_msgs::WrenchStamped>("tauInWrench", 1, &SimCore::onTau<geometry_msgs::WrenchStamped>, this);
	//general
	currentsSub = nh.subscribe<geometry_msgs::TwistStamped>("currents", 1, &SimCore::onCurrents, this);

	//Publishers
	//Auv_msgs
	meas = nh.advertise<auv_msgs::NavSts>("meas_ideal",1);
	measn = nh.advertise<auv_msgs::NavSts>("meas_noisy",1);
	tauAch = nh.advertise<auv_msgs::BodyForceReq>("tauAch",1);
	//odometry
	odom = nh.advertise<nav_msgs::Odometry>("meas_odom",1);
	odomn = nh.advertise<nav_msgs::Odometry>("meas_odom_noisy",1);
	tauAchWrench = nh.advertise<geometry_msgs::WrenchStamped>("tauAchWrench",1);
	//general
	acc = nh.advertise<geometry_msgs::Vector3>("nuacc_ideal",1);

	double fs(10);
	double maxThrust(1), minThrust(-1);
	ph.param("maxThrust",maxThrust,maxThrust);
	ph.param("minThrust",minThrust,minThrust);
	model.allocator.setThrusterMinMax(minThrust, maxThrust);
	ph.param("Rate",fs,fs);
	ph.param("ModelWrap", wrap,wrap);
	model.dT = 1/(fs*wrap);
	rate = ros::Rate(fs);
	ph.param("publish_world", enablePublishWorld, enablePublishWorld);
	ph.param("publish_sim_base", enablePublishSimBaseLink, enablePublishSimBaseLink);
	ph.param("originLat", originLat, originLat);
	ph.param("originLon", originLon, originLon);
	runner = boost::thread(boost::bind(&SimCore::start, this));
	*/
}

void USBLSim::onModemOut(const underwater_msgs::ModemTransmission::ConstPtr& data)
{
	ROS_INFO("Requested: %d", data->action);
}

int main(int argc, char* argv[])
{
	ros::init(argc,argv,"usbl_sim");
	ros::NodeHandle nh;

	labust::simulation::USBLSim simulator;

	ros::spin();
	return 0;
}
