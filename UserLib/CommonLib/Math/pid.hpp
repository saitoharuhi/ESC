/*
 * pid.hpp
 *
 *  Created on: Jun 26, 2024
 *      Author: gomas
 */

#ifndef PID_HPP_
#define PID_HPP_

#include "filter.hpp"

#include <cfloat>
#include <algorithm>
#include <limits>
#include <utility>

namespace CommonLib::Math{

///////////////////////////////////////////////////////////////////////////////////////////////
//PI（後退差分）
///////////////////////////////////////////////////////////////////////////////////////////////
class PIController{
protected:
	const float pid_freq;
	float kp;
	float ki;
	float error_sum = 0.0f;

	//anti windup
	float k_anti_windup;
	float limit_min;
	float limit_max;
public:
	PIController(float _pid_freq,float _kp,float _ki,float _k_anti_windup,
			float _limit_min = std::numeric_limits<float>().lowest(),
			float _limit_max = std::numeric_limits<float>().max()):
		pid_freq(_pid_freq),
		kp(_kp),
		ki(_ki/pid_freq),
		k_anti_windup(_k_anti_windup),
		limit_min(_limit_min),
		limit_max(_limit_max){
	}
	virtual float operator()(float target,float feedback){
		float error = target - feedback;
		error_sum += error;

		float pid_result =
				error * kp
				+ error_sum * ki;

		float pid_result_clamped = std::clamp<float>(pid_result, limit_min, limit_max);

		error_sum -= (pid_result - pid_result_clamped)*k_anti_windup;

		return pid_result_clamped;
	}

	virtual void reset(void){
		error_sum = 0;
	}

	void set_p_gain(float _kp){	kp = _kp; }
	float get_p_gain(void)const{return kp;}

	void set_i_gain(float _ki){ ki = _ki/pid_freq; }
	float get_i_gain(void)const{ return ki*pid_freq; }

	void set_anti_windup_gain(float _kb) { k_anti_windup = _kb; }
	float get_anti_windup_gain(void)const{ return k_anti_windup; }

	void set_limit(float _limit_max){
		limit_min = -_limit_max;
		limit_max = _limit_max;
	}
	void set_limit(float _limit_min,float _limit_max){
		limit_min = _limit_min;
		limit_max = _limit_max;
	}
	std::pair<float,float> get_limit(void)const{
		return std::pair<float,float>{limit_min,limit_max};
	}

	virtual ~PIController(){}
};

///////////////////////////////////////////////////////////////////////////////////////////////
//PID（後退差分）
///////////////////////////////////////////////////////////////////////////////////////////////
class PIDController:public PIController{
protected:
	static constexpr float eps = 0.0001f; //kdによるゼロ除算回避
	float kd;
	float prev_error = 0.0f;

	const float lpf_eta;
	LowpassFilterBD<float> lpf;
public:
	PIDController(float _pid_freq,float _kp,float _ki,float _kd, float _k_anti_windup,
			float _limit_min = std::numeric_limits<float>().lowest(),
			float _limit_max = std::numeric_limits<float>().max(),
			float _lpf_eta = 0.1f):
		PIController(_pid_freq,_kp,_ki,_k_anti_windup,_limit_min,_limit_max),
		kd(_kd*pid_freq),
		lpf_eta(_lpf_eta),
		lpf(pid_freq,abs(kd)<= eps ? 1/(2*M_PI*_lpf_eta*0.1) : 1/(2*M_PI*_lpf_eta*_kd)){
	}

	float operator()(float target,float feedback) override{
		float error = target - feedback;
		error_sum += error;

		float pid_result =
				error * kp
				+ error_sum * ki
				+ lpf(error - prev_error) * kd;

		float pid_result_clamped = std::clamp<float>(pid_result, limit_min, limit_max);

		prev_error = error;
		error_sum -= (pid_result - pid_result_clamped)*k_anti_windup;

		return pid_result_clamped;
	}

	void set_d_gain(float _kd){
		kd = _kd*pid_freq;
		if(abs(kd)>= eps){
			lpf.set_param(pid_freq,lpf_eta*_kd);
		}
	}
	float get_d_gain(void)const{ return kd/pid_freq; }

	void reset(void)override{
		error_sum = 0;
		prev_error = 0;
	}
};

///////////////////////////////////////////////////////////////////////////////////////////////
//外部の値を参照して積算値を調整可能にしたPI制御器
///////////////////////////////////////////////////////////////////////////////////////////////
class TrackingPIController:public PIController{
public:
	TrackingPIController(float _pid_freq,float _kp,float _ki)
			:CommonLib::Math::PIController(
					_pid_freq,
					_kp,
					_ki,
					0.0f,//アンチワインドアップ処理しない
					std::numeric_limits<float>().lowest(),
					std::numeric_limits<float>().max()){
	}

	//通常のアンチワインドアップ処理を省略
	float operator()(float target,float feedback)override{
		float error = target - feedback;
		error_sum += error;
		return error*kp + error_sum*ki;
	}

	//エラー積算値を操作可能
	void subtraction_from_error_sum(float subtraction_value){
		error_sum -= subtraction_value;
	}
	float get_error_sum(void)const{return error_sum;}
};

///////////////////////////////////////////////////////////////////////////////
//builder classes
///////////////////////////////////////////////////////////////////////////////
class PIBuilder{
public:
	float freq = 1.0f;
	float kp = 0;
	float ki = 0;
	float k_anti_windup = 0;
	float limit_max = std::numeric_limits<float>().max();
	float limit_min = std::numeric_limits<float>().lowest();

	PIBuilder(float _freq = 1.0f):freq(_freq){}

	PIBuilder& set_gain(float _kp,float _ki){
		kp = _kp;
		ki = _ki;
		if(fabs(kp) > 0.001f){
			k_anti_windup = 1.0f/kp;
		}

		return *this;
	}
	PIBuilder& set_gain(float _kp,float _ki, float _k_anti_windup){
		kp = _kp;
		ki = _ki;
		k_anti_windup = _k_anti_windup;

		return *this;
	}
	PIBuilder& set_limit(float _limit_min,float _limit_max){
		limit_max = _limit_max;
		limit_min = _limit_min;
		return *this;
	}
	PIBuilder& set_limit(float _limit_max){
		limit_max =  _limit_max;
		limit_min = -_limit_max;
		return *this;
	}
	PIController build()const{
		return PIController{freq,kp,ki,k_anti_windup,limit_min,limit_max};
	}
};

class PIDBuilder{
public:
	float freq = 1.0f;
	float kp = 0;
	float ki = 0;
	float kd = 0;
	float k_anti_windup = 0;
	float limit_max = std::numeric_limits<float>().max();
	float limit_min = std::numeric_limits<float>().lowest();
	float lpf_gain = 0.15f;

	PIDBuilder(float _freq = 1.0f):freq(_freq){}

	PIDBuilder& set_gain(float _kp,float _ki,float _kd){
		kp = _kp;
		ki = _ki;
		kd = _kd;
		if(fabs(kp) > 0.001f){
			k_anti_windup = 1.0f/kp;
		}

		return *this;
	}
	PIDBuilder& set_gain(float _kp,float _ki,float _kd, float _k_anti_windup){
		kp = _kp;
		ki = _ki;
		kd = _kd;
		k_anti_windup = _k_anti_windup;

		return *this;
	}
	PIDBuilder& set_limit(float _limit_min,float _limit_max){
		limit_max = _limit_max;
		limit_min = _limit_min;
		return *this;
	}
	PIDBuilder& set_limit(float _limit_max){
		limit_max =  _limit_max;
		limit_min = -_limit_max;
		return *this;
	}
	PIDBuilder& set_lpf_gain(float _gain){
		lpf_gain = _gain;
		return *this;
	}
	PIDController build()const{
		return PIDController{freq,kp,ki,kd,k_anti_windup,limit_min,limit_max,lpf_gain};
	}
};

}//namespace CommonLib::Math



#endif /* PID_HPP_ */
