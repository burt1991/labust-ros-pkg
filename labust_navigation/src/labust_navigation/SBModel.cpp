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
*  Created on: Feb 25, 2013
*  Author: Dula Nad
*
*  Modified by: Filip Mandic
*********************************************************************/
#include <labust/navigation/SBModel.hpp>
#include <vector>
#include <iostream>
#include <ros/ros.h>

using namespace labust::navigation;

SBModel::SBModel():
		dvlModel(0),
		xdot(0),
		ydot(0)
{
	this->initModel();
};

SBModel::~SBModel(){};

void SBModel::initModel()
{
  //std::cout<<"Init model."<<std::endl;
  x = vector::Zero(stateNum);
  xdot = 0;
  ydot = 0;
  //Setup the transition matrix
  derivativeAW();
  R0 = R;
  V0 = V;

  //std::cout<<"R:"<<R<<"\n"<<V<<std::endl;
}

void SBModel::calculateXYInovationVariance(const SBModel::matrix& P, double& xin,double &yin)
{
	xin = sqrt(P(xp,xp)) + sqrt(R0(xp,xp));
	yin = sqrt(P(yp,yp)) + sqrt(R0(yp,yp));
}

double SBModel::calculateAltInovationVariance(const SBModel::matrix& P)
{
	return 0; //sqrt(P(altitude,altitude)) + sqrt(R0(altitude,altitude));
}

void SBModel::calculateUVInovationVariance(const SBModel::matrix& P, double& uin,double &vin)
{
	uin = sqrt(P(u,u)) + sqrt(R0(v,v));// Greska???
	vin = sqrt(P(v,v)) + sqrt(R0(v,v));
}

void SBModel::step(const input_type& input)
{

  /*******************************************************************
   *** Single beacon navigation
   ******************************************************************/

  /* States --- u v w r x y z Psi vx vy buoyancy b ub vb rb xb yb Psib */

  x(u) += Ts*(-surge.Beta(x(u))/surge.alpha*x(u) + 1/surge.alpha * input(X));
  x(v) += Ts*(-sway.Beta(x(v))/sway.alpha*x(v) + 1/sway.alpha * input(Y));
  x(w) += Ts*(-heave.Beta(x(w))/heave.alpha*x(w) + 1/heave.alpha * (input(Z) + x(buoyancy)));
  x(r) += Ts*(-yaw.Beta(x(r))/yaw.alpha*x(r) + 1/yaw.alpha * input(N) + 0*x(b));

  xdot = x(u)*cos(x(psi)) - x(v)*sin(x(psi)) + x(xc);
  ydot = x(u)*sin(x(psi)) + x(v)*cos(x(psi)) + x(yc);

  x(xp) += Ts * xdot;
  x(yp) += Ts * ydot;
  x(zp) += Ts * x(w);
  x(psi) += Ts * x(r);

  x(xc) += 0;
  x(yc) += 0;
  x(buoyancy) += 0;
  x(b) += 0;

  x(ub) += 0;
  x(vb) += 0;
  x(rb) += 0;

  x(xb) += Ts*(cos(x(psib))*x(ub) - sin(x(psib))*x(vb));
  x(yb) += Ts*(sin(x(psib))*x(ub) + cos(x(psib))*x(vb));
  x(psib) += Ts * x(rb);

  xk_1 = x;

  derivativeAW();
};

void SBModel::derivativeAW()
{

	/*******************************************************************
	*** Single beacon navigation
	******************************************************************/

	/* States --- u v w r x y z Psi vx vy vz vpsi ub vb rb xb yb Psib */

	A = matrix::Identity(stateNum, stateNum);

	A(u,u) = 1-Ts*(surge.beta + 2*surge.betaa*fabs(x(u)))/surge.alpha;

	A(v,v) = 1-Ts*(sway.beta + 2*sway.betaa*fabs(x(v)))/sway.alpha;

	A(w,w) = 1-Ts*(heave.beta + 2*heave.betaa*fabs(x(w)))/heave.alpha;
	A(w,buoyancy) = Ts/heave.alpha;

	A(r,r) = 1-Ts*(yaw.beta + 2*yaw.betaa*fabs(x(r)))/yaw.alpha;
	A(r,b) = 0*Ts;

	A(xp,u) = Ts*cos(x(psi));
	A(xp,v) = -Ts*sin(x(psi));
	A(xp,psi) = Ts*(-x(u)*sin(x(psi)) - x(v)*cos(x(psi)));
	A(xp,xc) = Ts;

	A(yp,u) = Ts*sin(x(psi));
	A(yp,v) = Ts*cos(x(psi));
	A(yp,psi) = Ts*(x(u)*cos(x(psi)) - x(v)*sin(x(psi)));
	A(yp,yc) = Ts;

	A(zp,w) = Ts;

	A(psi,r) = Ts;

	A(xb,ub) = Ts*cos(x(psib));
	A(xb,vb) = -Ts*sin(x(psib));
	A(xb,psib) = -Ts*(x(vb)*cos(x(psib))-x(ub)*sin(x(psib)));

	A(yb,ub) = Ts*sin(x(psib));
	A(yb,vb) = Ts*cos(x(psib));
    A(yb,psib) = -Ts*(x(vb)*cos(x(psib))-x(ub)*sin(x(psib)));

    A(psib,rb) = Ts;
}

const SBModel::output_type& SBModel::update(vector& measurements, vector& newMeas)
{
	std::vector<size_t> arrived;
	std::vector<double> dataVec;

	//vector<size_t> arrived;
	//vector<double> dataVec;

	for (size_t i=0; i<newMeas.size(); ++i)
	{
		//ROS_ERROR("Redni broj mjerenja: %d", i);
		if (newMeas(i))
		{
			arrived.push_back(i);
			dataVec.push_back(measurements(i));
			newMeas(i) = 0;
		}
	}

	ROS_ERROR("broj mjerenja: %d", arrived.size());

	for( std::vector<size_t>::const_iterator it = arrived.begin(); it != arrived.end(); ++it)
		ROS_ERROR("%d", *it);


	//if (dvlModel != 0) derivativeH();

	derivativeH();

	measurement.resize(arrived.size());
	H = matrix::Zero(arrived.size(),stateNum);
	y = vector::Zero(arrived.size());
	R = matrix::Zero(arrived.size(),arrived.size());
	V = matrix::Zero(arrived.size(),arrived.size());

	for (size_t i=0; i<arrived.size();++i)
	{
		measurement(i) = dataVec[i];

		//if (dvlModel != 0)
		//{
			H.row(i)=Hnl.row(arrived[i]);
			y(i) = ynl(arrived[i]);
		//}
		//else
		//{
		//	H(i,arrived[i]) = 1;
		//	y(i) = x(arrived[i]);
		//}

		for (size_t j=0; j<arrived.size(); ++j)
		{
			R(i,j)=R0(arrived[i],arrived[j]);
			V(i,j)=V0(arrived[i],arrived[j]);
		}
	}

	//std::cout<<"Setup H:"<<H<<std::endl;
	//std::cout<<"Setup R:"<<R<<std::endl;
	//std::cout<<"Setup V:"<<V<<std::endl;

	return measurement;
}

void SBModel::estimate_y(output_type& y)
{
  y=this->y;
}

void SBModel::derivativeH()
{

	Hnl = matrix::Zero(measSize,stateNum);
	ynl = vector::Zero(measSize);

    //enum {um=0,vm,zm,psim,dm,ubm,vbm,rbm,xbm,ybm,psibm,measSize}; /* Measurement vector */

	ynl(um) = x(u)+x(xc)*cos(x(psi))+x(yc)*sin(x(psi));
	ynl(vm) = x(v)-x(xc)*sin(x(psi))+x(yc)*cos(x(psi));
	ynl(zm) = x(zp);
	ynl(psim) = x(psi);
	ynl(dm) = sqrt(pow((x(xp)-x(xb)),2)+pow((x(yp)-x(yb)),2)+pow(x(zp),2));
	ynl(ubm) = x(ub);
	ynl(vbm) = x(vb);
	ynl(rbm) = x(rb);
	ynl(xbm) = x(xb);
	ynl(ybm) = x(yb);
	ynl(psibm) = x(psib);

	Hnl(um,u) = 1;
	Hnl(um,xc) = cos(x(psi));
	Hnl(um,yc) = sin(x(psi));
	Hnl(um,psi) = -x(xc)*sin(x(psi)) + x(yc)*cos(x(psi));

	Hnl(vm,v) = 1;
	Hnl(vm,xc) = -sin(x(psi));
	Hnl(vm,yc) = cos(x(psi));
	Hnl(vm,psi) = -x(xc)*cos(x(psi)) - x(yc)*sin(x(psi));

	Hnl(zm,zp) = 1;

	Hnl(psim,psi) = 1;

	Hnl(dm,xp) = (x(xp)-x(xb))/sqrt(pow((x(xp)-x(xb)),2)+pow((x(yp)-x(yb)),2)+pow(x(zp),2));
	Hnl(dm,yp) = (x(yp)-x(yb))/sqrt(pow((x(xp)-x(xb)),2)+pow((x(yp)-x(yb)),2)+pow(x(zp),2));
	Hnl(dm,zp) = x(zp)/sqrt(pow((x(xp)-x(xb)),2)+pow((x(yp)-x(yb)),2)+pow(x(zp),2));
	Hnl(dm,xb) = -(x(xp)-x(xb))/sqrt(pow((x(xp)-x(xb)),2)+pow((x(yp)-x(yb)),2)+pow(x(zp),2));
	Hnl(dm,yb) = -(x(yp)-x(yb))/sqrt(pow((x(xp)-x(xb)),2)+pow((x(yp)-x(yb)),2)+pow(x(zp),2));

	Hnl(ubm,ub) = 1;
	Hnl(vbm,vb) = 1;
	Hnl(rbm,rb) = 1;
	Hnl(xbm,xb) = 1;
	Hnl(ybm,yb) = 1;
	Hnl(psibm,psib) = 1;
}

