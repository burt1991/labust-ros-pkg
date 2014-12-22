/*
 * perturb_esc_nd.cpp
 *
 *  Created on: Aug 1, 2012
 *      Author: Berk Calli
 *      Organization: Delft Biorobotics Lab., Delft University of Technology
 *  	Contact info: b.calli@tudelft.nl, web: www.dbl.tudelft.nl
 *
 * Class for perturbation based extremum seeking control
 *
 * * References:
 * - K. B. Ariyur and M. Krstic, "Real-Time Optimization by Extremum-Seeking Control", Wiley, 2003.
 * - B. Calli, W. Caarls, P. Jonker, M. Wisse, "Comparison of Extremum Seeking Control Algorithms for Robotic Applications", IROS 2012.
 *
 */

#include "esc_perturb/perturb_esc_nd.h"

PerturbESCND::PerturbESCND(){
	sin_amp_ = 0;
	sin_freq_ = 0;
	corr_gain_ = 0;
	high_pass_pole_ = 0;
	low_pass_pole_ = 0;
	comp_pole_ = 0;
	comp_zero_ = 0;
	period_ = 0;
	state_initialized_ = false;
	initialized_ = false;
	old_vals_initialized_ = false;
}
PerturbESCND::PerturbESCND(double sin_amp, double sin_freq, double corr_gain, double high_pass_pole, double low_pass_pole, double comp_zero, double comp_pole, double period){
	init(sin_amp, sin_freq, corr_gain, high_pass_pole, low_pass_pole, comp_zero, comp_pole, period);
}
void PerturbESCND::init(double sin_amp, double sin_freq, double corr_gain, double high_pass_pole, double low_pass_pole, double comp_zero, double comp_pole, double period){
	sin_amp_ = sin_amp;
	sin_freq_ = sin_freq;
	corr_gain_ = corr_gain;
	high_pass_pole_ = high_pass_pole;
	low_pass_pole_ = low_pass_pole;
	comp_pole_ = comp_pole;
	comp_zero_ = comp_zero;
	period_ = period;
	obj_val_old_ = 0;
	cycle_count_ = 0;
	hpf_out_old_ = 0;
	opt_dim_ = 0;
	state_initialized_ = false;
	old_vals_initialized_ = false;
	initialized_ = true;
}

std::vector<double>  PerturbESCND::step(std::vector<double> state, double obj_val){

	if(!initialized_){
		fprintf(stderr,"The perturbation based ESC (1D) is not initialized... It will not be executed. \n");
		return std::vector<double>();
	}

	if(!state_initialized_ && state.empty()){
		fprintf(stderr,"The state value of the perturbation based ESC (1D) cannot be initialized: State vector is empty. The algorithm will not be executed. \n");
		return std::vector<double>();
	}

	else if(!state_initialized_){

		opt_dim_ = (unsigned int)state.size();
		lpf_out_old_.resize(opt_dim_);
		signal_demodulated_old_.resize(opt_dim_);
		comp_old_.resize(opt_dim_);
		corr_signal_.resize(opt_dim_);
		pos_ref_.resize(opt_dim_);
		for (size_t i = 0; i<opt_dim_; i++){
			pos_ref_[i] = state[i];
			lpf_out_old_[i] = 0;
			signal_demodulated_old_[i] = 0;
			comp_old_[i] = 0;
			corr_signal_[i] = 0;
		}

		phase_shift_.resize(opt_dim_);
		phase_shift_[0] = 0;
		for (size_t i = 1; i<opt_dim_; i++){
			phase_shift_[i] = i*PI/((double)opt_dim_);
		}
		state_initialized_ = true;
	}

	double hpf_out = (-(period_*high_pass_pole_-2)*hpf_out_old_+2*obj_val-2*obj_val_old_)/(2+high_pass_pole_*period_);
	hpf_out_old_ = hpf_out;
	obj_val_old_ = obj_val;
	std::vector<double> signal_demodulated(opt_dim_);
	std::vector<double> lpf_out(opt_dim_);
	std::vector<double> comp_out(opt_dim_);
	std::vector<double> output;

	for (size_t i = 0; i<opt_dim_; i++){
		signal_demodulated[i]= hpf_out*sin_amp_*std::sin(cycle_count_*period_*sin_freq_ + phase_shift_[i]);
		lpf_out[i] = ((2.0-low_pass_pole_*period_)*lpf_out_old_[i]+low_pass_pole_*period_*signal_demodulated[i]+low_pass_pole_*period_*signal_demodulated_old_[i])/(2.0+low_pass_pole_*period_);
		comp_out[i]= ((2.0+period_*comp_zero_)*lpf_out[i]+(period_*comp_zero_-2.0)*lpf_out_old_[i]-(period_*comp_pole_-2.0)*comp_old_[i])/(2.0+period_*comp_pole_);
		corr_signal_[i] = corr_signal_[i]+corr_gain_*comp_out[i]*period_;
		if(!old_vals_initialized_)
			pos_ref_[i] = sin_amp_*std::sin(cycle_count_*period_*sin_freq_ + phase_shift_[i]);	//modulation
		else
			pos_ref_[i] = corr_signal_[i]+sin_amp_*std::sin(cycle_count_*period_*sin_freq_ + phase_shift_[i]);	//modulation
		output.push_back(pos_ref_[i]);
		signal_demodulated_old_[i] = signal_demodulated[i];
		lpf_out_old_[i] = lpf_out[i];
		comp_old_[i] = comp_out[i];
	}
	old_vals_initialized_ = true;
	cycle_count_++;

	return output;
}

ESC::inputType PerturbESCND::getInputType(){
	return ESC::inputStateValue;
}

ESC::outputType PerturbESCND::getOutputType(){
	return ESC::outputPosition;
}

std::vector<double> PerturbESCND::monitor(){
	std::vector<double> monitor_values;

	for(size_t i = 0; i<opt_dim_; i++)
		monitor_values.push_back(corr_signal_[i]);

	return monitor_values;
}
std::vector<std::string> PerturbESCND::monitorNames(){
	std::vector<std::string> monitor_names;
	std::string base = "correction signal ";
	char numstr[21];
	for(size_t i = 0; i<opt_dim_; i++){
		sprintf(numstr, "%zd", i+1);
		monitor_names.push_back(base + numstr);
	}

	return monitor_names;
}

void PerturbESCND::reset(){
	obj_val_old_ = 0;
	cycle_count_ = 0;
	hpf_out_old_ = 0;
	opt_dim_ = 0;
	state_initialized_ = false;
	old_vals_initialized_ = false;
}
