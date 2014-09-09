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
#include <labust/simulation/AcousticMedium.hpp>

using namespace labust::simulation;

AcousticMedium::AcousticMedium():
		vs(1500)
{
	this->onInit();
}

void AcousticMedium::onInit()
{
	ros::NodeHandle nh,ph("~");

	frMedium = nh.advertise<underwater_msgs::MediumTransmission>("to_medium",1);
	toMedium = nh.subscribe<underwater_msgs::MediumTransmission>("from_medium", 10,
			&AcousticMedium::onMediumData, this);

	//Dummy transponders
	transponders[0].odom.reset(new nav_msgs::Odometry());
	transponders[1].odom.reset(new nav_msgs::Odometry());
	transponders[1].odom->pose.pose.position.x = 1000;
	transponders[2].odom.reset(new nav_msgs::Odometry());
  transponders[2].odom->pose.pose.position.x = 1000;
  transponders[2].odom->pose.pose.position.y = 1000;
	/*
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

void AcousticMedium::onMediumData(const underwater_msgs::MediumTransmission::ConstPtr& data)
{
	ROS_INFO("Sender %d added medium from:",data->sender);

	TransponderMap::const_iterator sit = transponders.find(data->sender);
	if (sit == transponders.end())
	{
		ROS_ERROR("Wrong transponder source %d", data->sender);
		return;
	}

	TransmissionMap transmissions;

	//For each receiver != sender
	for (TransponderMap::const_iterator it=transponders.begin();
			it != transponders.end();
			++it)
	{
		if (it->first == sit->first) continue;

		//Publisher
		underwater_msgs::MediumTransmission::Ptr outRep(new
				underwater_msgs::MediumTransmission());
		//Calculate distance, bearing and elevation to receiver
		RBE val = getRelativePosition(sit->second, it->second);
		outRep->message = data->message;
		outRep->range = val.get<range>();
		outRep->bearing= val.get<bearing>();
		outRep->elevation = val.get<elevation>();
		//Check spatial occlusion
	  //Check if there is a transmission conflict on the receiver side
 	  //Check the strength on the receiver side
	  //Roll a dice on success ratio
		//Calculate transmission time
		double ttime = data->duration + outRep->range/vs;

		outRep->sender = data->sender;
		outRep->duration = ttime;
		outRep->receiver = it->first;

		//Add to transmission queue
		transmissions.insert(std::make_pair(ttime, outRep));
	}

	///\todo Move this to a thread to avoid blocking
	double timeBase(0);
	for (TransmissionMap::const_iterator it=transmissions.begin();
			it != transmissions.end();
			++it)
	{
		double diffTime = it->first - timeBase;
		ROS_INFO("Current time base: %f", timeBase);
		ros::Duration(diffTime).sleep();
		frMedium.publish(it->second);
		timeBase += diffTime;
	};
}

AcousticMedium::RBE AcousticMedium::getRelativePosition(const TransponderInfo& source,
		const TransponderInfo& receiver)
{
	RBE retVal;

	double dx = source.odom->pose.pose.position.x -
			receiver.odom->pose.pose.position.x;
	double dy = source.odom->pose.pose.position.y -
			receiver.odom->pose.pose.position.y;
	double dz = source.odom->pose.pose.position.z -
			receiver.odom->pose.pose.position.z;
	retVal.get<range>() = sqrt(dx*dx+dy*dy+dz*dz);
	retVal.get<bearing>() = atan2(dy,dx);
	retVal.get<elevation>() = atan2(dz, sqrt(dx*dx + dy*dy));

	return retVal;
}

int main(int argc, char* argv[])
{
	ros::init(argc,argv,"acoustic_medium");
	ros::NodeHandle nh;

	labust::simulation::AcousticMedium simulator;

	ros::spin();
	return 0;
}
