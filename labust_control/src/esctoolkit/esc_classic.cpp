/*
 * esc_classic.cpp
 *
 *  Created on: Dec 15, 2014
 *      Author: Filip Mandic
 */

#include <labust/control/esc/EscPerturbation.hpp>
//#include <vector>

#include <ros/ros.h>

namespace labust{
	namespace control{
		namespace esc{


			class EscClassic : public EscPerturbationBase<double> {

			public:

				/*** Control inputs */
				//enum {u = 0, v, controlNum};

				EscClassic();

				~EscClassic();

				 void initController();

				 numericprecission preFiltering(numericprecission cost_signal);

				 vector gradientEstimation(numericprecission cost_signal_filtered, vector additional_input);

				 vector postFiltering(vector estimated_gradient);

				 vector superimposePerturbation(vector control);

				 //vector step(numericprecission cost_signal);
				 /***
				 * K - gain
				 * A0 - perturbation amplitude
				 */
				vector gain_, sin_amp_, sin_freq_, corr_gain_, low_pass_pole_, comp_pole_, comp_zero_;
				vector control_ref_, signal_demodulated_old_, lpf_out_old_,corr_signal_, phase_shift_,comp_old_;
				/*** Controlled state */
				vector state_;

				numericprecission high_pass_pole_, hpf_out_old_, obj_val_old_;
				numericprecission period_;
			};

			typedef EscClassic Base;

			EscClassic::EscClassic():state_(vector::Zero(controlNum)),period_(0){

			}

			EscClassic::~EscClassic(){

			}

			void EscClassic::initController(){

				lpf_out_old_.resize(controlNum);
				signal_demodulated_old_.resize(controlNum);
				comp_old_.resize(controlNum);
				corr_signal_.resize(controlNum);
				control_ref_.resize(controlNum);

				lpf_out_old_.setZero();
				control_ref_ = state_;
				lpf_out_old_.setZero();
				signal_demodulated_old_.setZero();
				comp_old_.setZero();
				corr_signal_.setZero();

				phase_shift_.resize(controlNum);
				phase_shift_[0] = 0;
				for (size_t i = 1; i<controlNum; i++){
					phase_shift_[i] = i*M_PI/((double)controlNum);
				}
				state_initialized_ = true;

			}

			Base::numericprecission EscClassic::preFiltering(numericprecission cost_signal){

				return (-(period_*high_pass_pole_-2)*pre_filter_output_old_+2*cost_signal-2*pre_filter_input_old_)/(2+high_pass_pole_*period_);
			}

			Base::vector EscClassic::gradientEstimation(numericprecission cost_signal_filtered, vector additional_input){

				vector signal_demodulated(controlNum);
				for (size_t i = 0; i<controlNum; i++){
					signal_demodulated[i]= cost_signal_filtered*sin_amp_[i]*std::sin(double(cycle_count_*period_*sin_freq_[i] + phase_shift_[i]));
				}
				return signal_demodulated;
			}

			Base::vector EscClassic::postFiltering(vector estimated_gradient){

				vector lpf_out(controlNum);
				vector comp_out(controlNum);

				for (size_t i = 0; i<controlNum; i++){

					lpf_out[i] = ((2.0-low_pass_pole_[i]*period_)*lpf_out_old_[i]+low_pass_pole_[i]*period_*estimated_gradient[i]+low_pass_pole_[i]*period_*estimated_gradient_old_[i])/(2.0+low_pass_pole_[i]*period_);
					comp_out[i]= ((2.0+period_*comp_zero_[i])*lpf_out[i]+(period_*comp_zero_[i]-2.0)*lpf_out_old_[i]-(period_*comp_pole_[i]-2.0)*comp_old_[i])/(2.0+period_*comp_pole_[i]);
					corr_signal_[i] = corr_signal_[i]+corr_gain_[i]*comp_out[i]*period_;

					lpf_out_old_[i] = lpf_out[i];
					comp_old_[i] = comp_out[i];
				}

				return corr_signal_;

			}

			Base::vector EscClassic::superimposePerturbation(Base::vector control){

				for (size_t i = 0; i<controlNum; i++){
					if(!old_vals_initialized_)
						control_ref_[i] = sin_amp_[i]*std::sin(double(cycle_count_*period_*sin_freq_[i] + phase_shift_[i]));
					else
						control_ref_[i] = control[i]+sin_amp_[i]*std::sin(double(cycle_count_*period_*sin_freq_[i] + phase_shift_[i]));
				}

				return control_ref_;
			}
		}
	}
}

using namespace labust::control::esc;

int main(int argc, char** argv){

	ros::init(argc, argv, "ESCclassic");
	ros::NodeHandle nh;

	EscClassic ESC;


	ros::spin();

	return 0;
}




