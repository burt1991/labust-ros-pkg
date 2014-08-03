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
#ifndef PIDCONTROLLER_HPP_
#define PIDCONTROLLER_HPP_
/**
 * The helper function for output saturation.
 */
float sat(float u, float low, float high);
namespace labust
{
	namespace control
	{

		class PIDBase
		{
		public:
			/**
			 * The proportional, integral, derivative,
			 * filter and tracking gain.
			 */
			double Kp, Ki, Kd, Tf, Kt;

			bool autoWindup;

			bool extWindup;

			bool useBackward;

			bool useConditional;

			double conditionalFactor;

			bool _windup;

			double eik, epk, edk, ffk, flk;

			double rk;

			double a, b, c;

			double uk;

			double uk_1, ffk_1, flk_1;

			double eik_1, epk_1, edk_1;

			double Ik_1,Pk_1, FFk_1, FLk_1;

			double Ik, Pk, FFk, FLk;

			double track;

			double output;

			double umax;


		};

		double step(PIDBase* self, double Ts)
		{
			if (self->autoWindup != 0)
			{
				//Report windup and perform anti-windup step
				self->_windup = (self->output != self->uk) &&
						(self->output*self->eik > 0);
			}
			else
			{
				//If externally reported windup perform tracking
				self->_windup = (self->extWindup != 0) &&
						(self->output*self->eik > 0);
			}

			//Backward recalculation
			if ((self->Ik_1 != 0) && self->_windup && self->useBackward)
			{
				//Calculate the proportional influence
				double diff = self->track - self->uk_1 + self->Ik_1;
				//If the proportional part is already in windup remove the whole last integral
				//Otherwise recalculate the integral to be on the edge of windup
				self->uk_1 -= ((diff*self->track <= 0)?self->Ik_1:0*(self->Ik_1 - diff));
			}

			//Proportional term
			self->Pk = self->Kp*(self->epk-self->epk_1);

			//Integral term
			//This is the equivalent of full tracking anti-windup
			if (self->useConditional && (self->Kp != 0))
			{
				self->Ik = (self->eik <= self->conditionalFactor/self->Kp) ? ((!self->_windup)?self->Ki*Ts*self->eik:0):0;
			}
			else
			{
				self->Ik = (!self->_windup)?self->Ki*Ts*self->eik:0;
			}

			//Feed forward term
			self->FFk = self->ffk - self->ffk_1;

			//Feedback linearization term
			self->FLk = self->flk - self->flk_1;
			//self->internalState += self->lastF;
			//Set initial output
			self->uk = self->uk_1 + self->Pk + self->Ik + self->FFk + self->FLk;

			if (self->autoWindup != 0)	self->output = sat(self->uk,-self->umax, self->umax);

			self->Pk_1 = self->Pk;
			self->Ik_1 = self->Ik;
			self->FFk_1 = self->FFk;
			self->FLk_1 = self->FLk;

			self->eik_1 = self->eik;
			self->epk_1 = self->epk;
			self->edk_1 = self->edk;
			self->ffk_1 = self->ffk;
			self->flk_1 = self->flk;
			self->uk_1 = self->uk;
		}
	}
}

/**
 * Initialize the controller base. Set all the values to zero
 * and disable auto windup detection.
 */
void PIDBase_init(PIDBase* self);

/**
 * The helper function for output saturation.
 */
float sat(float u, float low, float high);
/* PIDBASE_H_ */
#endif
