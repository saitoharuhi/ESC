/*
 * disturbance_observer.hpp
 *
 *  Created on: Jul 13, 2025
 *      Author: gomas
 */

#ifndef COMMONLIB_DISTURBANCE_OBSERVER_HPP_
#define COMMONLIB_DISTURBANCE_OBSERVER_HPP_

#include "filter.hpp"

#include <concepts>

//TODO:開発と動作確認
namespace CommonLib::Math{

//うおーテンプレートここで使いたくないーーー！！！！
//でもIFilterにキャストするとパラメータの設定周りが極めてめんどくさくなるんだよなー
template<class T>
concept TransferFunction = std::derived_from<T, IFilter<float>>;


//ローパスフィルタ無しのシンプルな外乱オブザーバー
template<TransferFunction T>
class DisturbanceObserverBase{
protected:
	float prev_controller_output = 0.0f;
	float prev_observed_disturbance = 0.0f;
public:
	T inverse_model; //制御対象の逆モデル

	DisturbanceObserverBase(T&& _inverse_model) //なんとなく右辺値参照にしたけどべつにいいかも
		 :inverse_model(std::move(_inverse_model)){
	}

	virtual float observe_disturbance(float feedback_val,float controller_output){
		float observed_disturbance = inverse_model(feedback_val) - (prev_controller_output - prev_observed_disturbance);

		prev_controller_output = controller_output;
		prev_observed_disturbance = observed_disturbance;
		return observed_disturbance;
	}

	virtual void reset(void){
		prev_controller_output = 0.0f;
		prev_observed_disturbance = 0.0f;
	}

	virtual ~DisturbanceObserverBase(){};
};

//ローパスフィルタ内臓してるやつ
template<TransferFunction T>
class DisturbanceObserver:public DisturbanceObserverBase<T>{
private:
	const float operation_freq;
	const float lpf_q_factor;
	float lpf_cutoff_freq;
	//CommonLib::Math::BiquadFilter<float> lpf;
	CommonLib::Math::LowpassFilterBD<float> lpf;
public:
	DisturbanceObserver(float _operation_freq,T&& _inverse_model,float _lpf_cutoff_freq,float _lpf_q_factor = 1.0f) //なんとなく右辺値参照にしたけどべつにいいかも
		:DisturbanceObserverBase<T>(std::move(_inverse_model)),
		 operation_freq(_operation_freq),
		 lpf_q_factor(_lpf_q_factor),
		 lpf_cutoff_freq(_lpf_cutoff_freq),
		 //lpf(operation_freq,lpf_cutoff_freq,lpf_q_factor),
		 lpf(operation_freq,lpf_cutoff_freq)
		 {
	}

	float observe_disturbance(float feedback_val,float controller_output)override{
		float observed_disturbance = lpf(DisturbanceObserverBase<T>::inverse_model(feedback_val) - (DisturbanceObserverBase<T>::prev_controller_output - DisturbanceObserverBase<T>::prev_observed_disturbance));

		DisturbanceObserverBase<T>::prev_controller_output = controller_output;
		DisturbanceObserverBase<T>::prev_observed_disturbance = observed_disturbance;
		return observed_disturbance;
	}

	void set_lpf_cutoff_freq(float f_cutoff){
		//lpf.set_param(operation_freq,f_cutoff,lpf_q_factor);
		lpf.set_param(operation_freq,f_cutoff);
		lpf_cutoff_freq = f_cutoff;
	}
	float get_lpf_cutoff_freq(void){
		return lpf_cutoff_freq;
	}

	void reset(void)override{
		DisturbanceObserverBase<T>::prev_controller_output = 0.0f;
		DisturbanceObserverBase<T>::prev_observed_disturbance = 0.0f;
		lpf.reset();
	}
};
}



#endif /* COMMONLIB_DISTURBANCE_OBSERVER_HPP_ */
