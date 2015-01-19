/*
 * esc_ekf_grad_model.cpp
 *
 *  Created on: Jan 13, 2015
 *      Author: Filip Mandic
 */

#include <labust/control/esc/EscPerturbation.hpp>

#include <ros/ros.h>

namespace labust{
	namespace control{
		namespace esc{


			class EscEkfGradModel : public EscPerturbationBase<double> {

			public:

				EscEkfGradModel(int ctrlNum, numericprecission Ts);

				~EscEkfGradModel();

				 void initController(double sin_amp, double sin_freq, double corr_gain, double high_pass_pole, double low_pass_pole, double comp_zero, double comp_pole, double period);

				 vector gradientEstimation(numericprecission cost_signal_filtered, vector additional_input);

				 vector controllerGain(vector postFiltered);

				 vector superimposePerturbation(vector control);

				 vector modelUpdate(vector state, vector input);

				 vector outputUpdate(vector state, vector input);

				 /***
				 * K - gain
				 * A0 - perturbation amplitude
				 */
				vector sin_amp_, sin_freq_, corr_gain_;
				vector control_ref_, signal_demodulated_old_, phase_shift_;
				/*** Controlled state */
				vector state_;

				/*** EKF */

				matrix A, L, H, M, Q, R;
				matrix Pk_plu, Pk_min, Kk;
				vector xk_plu, xk_min, hk;

				vector input, input_past;

				/*** Measurement vector */
				vector yk;

				Eigen::Vector3d n1, n2;
			};

			typedef EscEkfGradModel Base;

			Base::EscEkfGradModel(int ctrlNum, numericprecission Ts):EscPerturbationBase<double>(ctrlNum, Ts),
										state_(vector::Zero(controlNum)){

				signal_demodulated_old_.resize(controlNum);
				control_ref_.resize(controlNum);
				sin_amp_.resize(controlNum);
				sin_freq_.resize(controlNum);
				gain_.resize(controlNum);
				control_.resize(controlNum);

				control_ref_ = state_;
				signal_demodulated_old_.setZero();

				A = matrix::Identity(3,3);
				L = matrix::Identity(3,3);
				H = matrix::Zero(3,3);
				M = matrix::Identity(3,3);

				Pk_plu = 1.0e-3*matrix::Identity(3,3);
				xk_plu = vector::Zero(3);

				//Qk = 1e-0*diag([0.75 0.75 0.5]); % Process noise vector OVO JE DOBRO 2
				//Rk = 1e-0*diag([1 1 1]); % Measurement noise vector

				Eigen::Vector3d tmp;
				tmp << 2, 2, 5;
				Q = tmp.asDiagonal();
				tmp << 0.01, 0.01, 0.01;
				R = tmp.asDiagonal();

				n1 = vector::Zero(3);
				n2 = vector::Zero(3);
				yk = vector::Zero(3);
				input = vector::Zero(8);

				phase_shift_.resize(controlNum);
				phase_shift_[0] = 0;

				for (size_t i = 1; i<controlNum; i++){
					phase_shift_[i] = i*M_PI/((double)controlNum);
				}
				state_initialized_ = true;
			}

			Base::~EscEkfGradModel(){

			}

			void Base::initController(double sin_amp, double sin_freq, double corr_gain, double high_pass_pole, double low_pass_pole, double comp_zero, double comp_pole, double Ts){

				sin_amp_.setConstant(sin_amp);
				sin_freq_.setConstant(sin_freq);
				gain_.setConstant(corr_gain);
				Ts_ = Ts;
				cycle_count_ = 0;
				state_initialized_ = false;
				old_vals_initialized_ = false;
				initialized_ = true;
			}


			Base::vector Base::gradientEstimation(numericprecission cost_signal_filtered, vector additional_input){

				vector signal_demodulated(controlNum);

				input << additional_input(0), n1(0), n2(0), additional_input(1), n1(1), n2(1), additional_input(2), additional_input(3);
				yk << cost_signal_filtered, n1(2), n2(2);

				n2 = n1;
				n1 << additional_input.head(2), cost_signal_filtered;

				H << input(0), input(3), 1,
					 input(1), input(4), 1,
					 input(2), input(5), 1;

				Pk_min = A*Pk_plu*A.transpose() + L*Q*L.transpose();
				xk_min =  modelUpdate(xk_plu, input);

				hk = outputUpdate(xk_min, input);
				matrix tmp = H*Pk_min*H.transpose()+M*R*M.transpose();
				Kk = (Pk_min*H.transpose())*tmp.inverse();

				xk_plu = xk_min+Kk*(yk-hk);
				Pk_plu = (matrix::Identity(3,3)-Kk*H)*Pk_min;
				input_past = input;

				signal_demodulated << xk_plu(0), xk_plu(1);
				return signal_demodulated;

			}


			Base::vector Base::controllerGain(vector postFiltered){
				control_ = gain_.cwiseProduct(postFiltered);
				return control_;
			}

			Base::vector Base::superimposePerturbation(Base::vector control){

				for (size_t i = 0; i<controlNum; i++){
					if(!old_vals_initialized_)
						control_ref_[i] = sin_amp_[i]*std::sin(double(cycle_count_*Ts_*sin_freq_[i] + phase_shift_[i]+M_PI/2));
					else
						control_ref_[i] = control[i]+sin_amp_[i]*std::sin(double(cycle_count_*Ts_*sin_freq_[i] + phase_shift_[i]+M_PI/2));
				}

				return control_ref_;
			}

			Base::vector Base::modelUpdate(vector state, vector input){

				vector model(3);

				model << state(0)+2*input(6)*Ts_, state(1)+2*input(7)*Ts_, state(2);

				return model;
			}

			Base::vector Base::outputUpdate(vector state, vector input){
				// outputUpdate = [u1k*dF1+u2k*dF2+d u1k1*dF1+u2k1*dF2+d u1k2*dF1+u2k2*dF2+d].';

				vector output(3);
				output << state(0)*input(0)+state(1)*input(3)+state(2),
						  state(0)*input(1)+state(1)*input(4)+state(2),
						  state(0)*input(2)+state(1)*input(5)+state(2);
				return output;
			}
		}
	}
}










