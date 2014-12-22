/*
 * EscPerturbation.hpp
 *
 *  Created on: Dec 15, 2014
 *      Author: Filip Mandic
 */

#ifndef ESCPERTURBATION_HPP_
#define ESCPERTURBATION_HPP_

#include <Eigen/Dense>
#include <stdint.h>


namespace labust{
	namespace control{
		namespace esc{

		/*************************************************************
		 *** Abstract class definition
		 ************************************************************/
			template <typename precission = double>
			class EscPerturbationBase {

			public:

				typedef precission numericprecission;

				typedef Eigen::Matrix<precission, Eigen::Dynamic, Eigen::Dynamic> matrix;
				typedef Eigen::Matrix<precission, Eigen::Dynamic, 1> vector;

				EscPerturbationBase():Ts_(0),cycle_count_(0),controlNum(2){

					state_initialized_ = false;
					initialized_ = false;
					old_vals_initialized_ = false;
				}

			    virtual ~EscPerturbationBase(){}

				/*****************************************************
				 *** Class functions
				 ****************************************************/

				 virtual numericprecission preFiltering(numericprecission cost_signal){

					 return cost_signal;
				 }

				 virtual vector gradientEstimation(numericprecission cost_signal_filtered, vector additional_input = vector::Zero(2)) = 0;

				 virtual vector postFiltering(vector estimated_gradient){
					 return estimated_gradient;
				 }



				 virtual vector superimposePerturbation(vector control) = 0;

				 virtual vector step(numericprecission cost_signal){

					 numericprecission filtered_cost =  preFiltering(cost_signal);
					 pre_filter_input_old_ = cost_signal;
					 pre_filter_output_old_ = filtered_cost;

					 //vector estimated_gradient(controlNum);
					 vector estimated_gradient = gradientEstimation(filtered_cost);
					 estimated_gradient_old_ = estimated_gradient;

					 vector control = postFiltering(estimated_gradient);

					 vector controlInput =  superimposePerturbation(control);

					 old_vals_initialized_ = true;
					 cycle_count_++;

					 return controlInput;
				 }

				virtual void reset(){

					state_initialized_ = false;
					initialized_ = false;
					old_vals_initialized_ = false;
					cycle_count_ = 0;
				}

				/*****************************************************
				 *** General parameters
				 ****************************************************/

				/*** Sampling time */
				precission Ts_;

				/*** Cycle */
				uint32_t cycle_count_;

				/*** Status flags */
				bool state_initialized_, initialized_, old_vals_initialized_;

				/*** Number of control inputs (states) */
				int controlNum;

				/*** */
				numericprecission pre_filter_input_old_, pre_filter_output_old_;
				vector estimated_gradient_old_;

			};
		}
	}
}




#endif /* ESCPERTURBATION_HPP_ */
