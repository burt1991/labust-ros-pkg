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
*********************************************************************/
#ifndef LDTRAVMODEL_HPP_
#define LDTRAVMODEL_HPP_
#include <labust/navigation/SSModel.hpp>

namespace labust
{
  namespace navigation
  {
    /**
     * This class implements a EKF filter for the LDTravOcean ROV.
     *
     * \todo Extract the update method into a separate file for future use.
     */
    class LDTravModel : public SSModel<double>
    {
    	typedef SSModel<double> Base;
    public:
      typedef vector input_type;
      typedef vector output_type;

      struct ModelParams
      {
    	  ModelParams():
    		  alpha(1),
    		  beta(1),
    		  betaa(0){};

    	  ModelParams(double alpha, double beta, double betaa):
    		  alpha(alpha),
    		  beta(beta),
    		  betaa(betaa){}

    	  inline double Beta(double val)
    	  {
    		  return beta + betaa*fabs(val);
    	  }

    	  double alpha, beta, betaa;
      };

      enum {u=0,v,w,r,xp,yp,zp,psi,xc,yc,b,buoyancy};
      enum {stateNum = 12};
      enum {inputSize = 4};
      enum {measSize = 7};
      enum {X=0,Y,Z,N};
//      enum {u_m=0,v_m,w_m,x_m,y_m,z_m,psi_m};
//      enum {xpsi_m=0, ypsi_m, xypsipsi_m};
//      enum {upsi_m=0, vpsi_m, wpsi_m, psipsi_m};
//      enum {psiOnly_m = 0};
        
      /**
       * The default constructor.
       */
      LDTravModel();
      /**
       * Generic destructor.
       */
      ~LDTravModel();

      /**
       * Perform a prediction step based on the system input.
       *
       * \param u System input.
       */
      void step(const input_type& input);
      /**
       * Calculates the estimated output of the model.
       *
       * \param y Inserts the estimated output values here.
       */
      void estimate_y(output_type& y);
      /**
       * Initialize the model to default values
       */
      void initModel();

      /**
       * Setup the measurement matrix for available measurements.
       */
      const output_type& update(vector& measurements, vector& newMeas);

      /**
       * Set the model parameters.
       */
      void setParameters(const ModelParams& surge,
    		  const ModelParams& sway,
    		  const ModelParams& heave,
    		  const ModelParams& yaw)
      {
    	  this->surge = surge;
    	  this->sway = sway;
    	  this->sway = heave;
    	  this->yaw = yaw;
      }

      void calculateXYInovationVariance(const matrix& P, double& xin,double &yin);

      /**
       * Return the speeds in the local frame.
       */
      inline void getNEDSpeed(double& xdot, double& ydot)
      {
      	xdot = this->xdot;
      	ydot = this->ydot;
      }

    protected:
     /**
       * Calculate the Jacobian matrices.
       */
      void derivativeAW();
      /**
        * Calculate the Jacobian matrices.
        */
       void derivativeHV(int num);
      /**
       * The model parameters.
       */
      ModelParams surge,sway,heave,yaw;
      /**
       * The newest measurement.
       */
      output_type measurement;
      /**
       * The full update matrix.
       */
      matrix R0,V0;
      /**
       * The NED speeds.
       */
      double xdot,ydot;
    };
  }
}

/* LDTRAVMODEL_HPP_ */
#endif