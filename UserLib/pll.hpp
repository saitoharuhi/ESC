/*
 * pll.hpp
 *
 *  Created on: Dec 17, 2025
 *      Author: gomas
 */

#ifndef PLL_HPP_
#define PLL_HPP_

#include "CommonLib/Math/pid.hpp"
namespace BoardLib{

//離散的なエンコーダ出力を滑らかにする用途
//実際の位置と推定される位置の差が0となるようにPI制御器を用いて速度を推定し，積分して位置を推定する
template<typename T>
class PLL{
	const float control_period;
	CommonLib::Math::TrackingPIController pi;
	float prev_pi_out;
	T sum;
	T true_val;
public:
	PLL(float f_control,float response_freq) //f_control:制御周期,f_sample:真値の更新周期（目安）
	:control_period(1.f/f_control),
	 pi(CommonLib::Math::TrackingPIController(f_control,2*M_PI*response_freq,(2*M_PI*response_freq)*(2*M_PI*response_freq)*0.2f)),
	 prev_pi_out(0.f),
	 sum(static_cast<T>(0)),
	 true_val(static_cast<T>(0)){

	}
	void update_sample(T v){
		true_val = v;
	}
	void update(void){
		T error = sum - true_val;
		prev_pi_out = pi(0,static_cast<float>(error));
		sum += prev_pi_out*control_period;
	}
	void override_pos(T v){
		sum = v;
		pi.reset();
	}
	T get_pos(void)const{
		return sum;
	}
	float get_spd(void)const{
		return prev_pi_out;
	}
	T get_sample(void)const{
		return true_val;
	}
};
}

#endif /* PLL_HPP_ */
