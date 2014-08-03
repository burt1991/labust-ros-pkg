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
 *  Created on: 26.06.2013.
 *  Author: Dula Nad
 *********************************************************************/
#ifndef CONTROLTOPICHANDLER_HPP_
#define CONTROLTOPICHANDLER_HPP_
#include <labust/control/ControllerStateTracker.hpp>
#include <navcon_msgs/ControllerState.h>
#include <navcon_msgs/ControllerInfo.h>
#include <ros/ros.h>

#include <boost/shared_ptr.hpp>
#include <boost/function.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

namespace labust
{
	namespace control
	{
		template <class MsgInterface>
		class ControlTopicHandler
		{
		public:
			struct Topics
			{
				Topics():
					cState(new typename MsgInterface::StateType()),
					cReference(new typename MsgInterface::ReferenceType()),
					cTracking(new typename MsgInterface::OutputType()),
					cManual(new typename MsgInterface::OutputType()){};
				/**
				 * Current state.
				 */
				typename MsgInterface::StateType::ConstPtr cState;
				/**
				 * Current reference.
				 */
				typename MsgInterface::ReferenceType::ConstPtr cReference;
				/**
				 * Current tracking.
				 */
				typename MsgInterface::OutputType::ConstPtr cTracking;
				/**
				 * Current manual.
				 */
				typename MsgInterface::OutputType::ConstPtr cManual;
			};

			typedef boost::function<typename MsgInterface::OutputType::Ptr	(const Topics&)> Callback;

			/**
			 * Main constructor.
			 * Controller could be inherited instead of composited.
			 */
			ControlTopicHandler():
				refSync(false),
				stateSync(false),
				trackSync(false),
				enableTrackSync(false),
				manSync(false),
				policy(2),
				sync(policy){};

			/**
			 * The controller initialization and subscriptions.
			 */
			void init(ros::NodeHandle nh, ros::NodeHandle ph, Callback callback)
			{
				this->callback = callback;
				//Init handlers
				this->nh = nh;
				this->ph = ph;
				//Initialize subscribers
				state = nh.subscribe<typename MsgInterface::StateType>("state", 1,
						&ControlTopicHandler::onState,this);
				reference = nh.subscribe<typename MsgInterface::ReferenceType>("reference", 1,
						&ControlTopicHandler::onReference,this);
				tracking = nh.subscribe<typename MsgInterface::OutputType>("tracking", 1,
						&ControlTopicHandler::onTracking,this);
				manual = nh.subscribe<typename MsgInterface::OutputType>("manual", 1,
						&ControlTopicHandler::onTracking,this);
/*
				image1_sub.subscribe(nh, "state", 1);
			  image2_sub.subscribe(nh, "reference", 1);

			  policy.setMaxIntervalDuration(ros::Duration(0.1));


			  // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
			  sync.connectInput(image2_sub, image1_sub);
			  sync.registerCallback(boost::bind(&ControlTopicHandler::onFilter,this, _1, _2));

			  ROS_INFO("Topic sync.");
*/
				//Initialize publishers
				output = nh.advertise<typename MsgInterface::OutputType>("out", 1);
			}

			void updateTopics(const navcon_msgs::ControllerInfo& info)
			{
				if (!info.manual_topic.empty() && (state.getTopic() != info.manual_topic))
				{
					state = nh.subscribe<typename MsgInterface::OutputType>(info.manual_topic, 1,
							&ControlTopicHandler::onManual,this);
				}

				if (!info.state_topic.empty() && (state.getTopic() != info.state_topic))
				{
					state = nh.subscribe<typename MsgInterface::StateType>(info.state_topic, 1,
							&ControlTopicHandler::onState,this);
				}

				if (!info.external_topic.empty() && (reference.getTopic() != info.external_topic))
				{
					reference = nh.subscribe<typename MsgInterface::ReferenceType>(info.external_topic, 1,
							&ControlTopicHandler::onReference,this);
				}

				if (!info.tracking_topic.empty() && (tracking.getTopic() != info.tracking_topic))
				{
					tracking = nh.subscribe<typename MsgInterface::OutputType>(info.tracking_topic, 1,
							&ControlTopicHandler::onTracking,this);
				}
			}

		protected:
			//Testing the use of message filters.
		  typedef message_filters::sync_policies::ApproximateTime<typename MsgInterface::ReferenceType,
		  		typename MsgInterface::StateType> MySyncPolicy;
			 message_filters::Subscriber<typename MsgInterface::StateType> image1_sub;
			 message_filters::Subscriber<typename MsgInterface::ReferenceType> image2_sub;
			 MySyncPolicy policy;
			 message_filters::Synchronizer<MySyncPolicy> sync;
			 void onFilter(const typename MsgInterface::ReferenceType::ConstPtr& ref, const typename MsgInterface::StateType::ConstPtr& state)
			 {
				 ROS_WARN("Topic triggered.");
			 }

			/**
			 * The tick message.
			 */
			void onTick()
			{
				if (stateSync && refSync)
				{
					double dTref = (topics.cState->header.stamp - topics.cReference->header.stamp).toSec();
					double dTtracking = (topics.cState->header.stamp - topics.cTracking->header.stamp).toSec();
					ROS_INFO("Current time difference to state: ref=%f, tracking=%f", dTref, dTtracking);

					typename MsgInterface::OutputType::Ptr cOut = callback(topics);
					if (cOut != 0) output.publish(cOut);

					trackSync = stateSync = refSync = manSync = false;
				}
			}

			/**
			 * Handle manual messages.
			 */
			void onManual(const typename MsgInterface::OutputType::ConstPtr& man)
			{
				topics.cManual = man;
				manSync = true;

				this->onTick();
			}
			/**
			 * Handle state messages.
			 */
			void onState(const typename MsgInterface::StateType::ConstPtr& state)
			{
				topics.cState = state;
				stateSync = true;

				this->onTick();
			}
			/**
			 * Handle reference messages.
			 */
			void onReference(const typename MsgInterface::ReferenceType::ConstPtr& ref)
			{
				topics.cReference = ref;
				refSync = true;

				this->onTick();
			}
			/**
			 * Handle tracking messages.
			 */
			void onTracking(const typename MsgInterface::OutputType::ConstPtr& track)
			{
				topics.cTracking = track;
				trackSync = true;
				enableTrackSync= true;

				this->onTick();
			}

			/**
			 * The node handles.
			 */
			ros::NodeHandle nh, ph;

			/**
			 * The state topic subscriber.
			 */
			ros::Subscriber manual;
			/**
			 * The state topic subscriber.
			 */
			ros::Subscriber state;
			/**
			 * The reference topic subscriber.
			 */
			ros::Subscriber reference;
			/**
			 * The tracking topic subscriber.
			 */
			ros::Subscriber tracking;
			/**
			 * The output topic publisher.
			 */
			ros::Publisher output;
			/**
			 * Track variables for sync.
			 */
			bool manSync, refSync, stateSync, trackSync, enableTrackSync;

			/**
			 * Topics.
			 */
			Topics topics;

			Callback callback;
		};
	}
}

/* CONTROLTOPICHANDLER_HPP_ */
#endif
