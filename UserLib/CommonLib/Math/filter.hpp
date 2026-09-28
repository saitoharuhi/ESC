/*
 * filter.hpp
 *
 *  Created on: Oct 24, 2024
 *      Author: gomas
 */

#ifndef COMMONLIB_FILTER_HPP_
#define COMMONLIB_FILTER_HPP_

#include <concepts>
#include <type_traits>
#include <cmath>

namespace CommonLib::Math{

	template <class T>
	concept Arithmetic = std::is_arithmetic_v<T>;

	template<Arithmetic T>
	class IFilter{
	public:
		virtual T operator() (T input) = 0;
		virtual void reset(void) = 0;
		virtual ~IFilter(){}
	};


	//後退差分による一次LPF
	template<Arithmetic T>
	class LowpassFilterBD:public IFilter<T>{
	private:
		T data = static_cast<T>(0);
		float k = 0.0f;
	public:
		LowpassFilterBD(float _k):k(_k){}
		LowpassFilterBD(float sampling_freq, float cutoff_freq):k(2*M_PI*cutoff_freq/(sampling_freq + 2*M_PI*cutoff_freq)){}

		T operator() (T input)override{
			data = static_cast<T>(static_cast<float>(input)*k + (1.0f-k)*static_cast<float>(data));
			return data;
		}
		T get(void)const{
			return data;
		}
		void overwrite(T val){
			data = val;
		}
		void set_param(float sampling_freq, float cutoff_freq){
			k = 2*M_PI*cutoff_freq/(sampling_freq + 2*M_PI*cutoff_freq);
		}
		void reset(void)override{
			data = static_cast<T>(0);
		}
	};

	//双一次変換による二次ローパスフィルタ
	template<Arithmetic T>
	class BiquadFilter:public IFilter<T>{
	private:
		T x1;
		T x2;
		T y1;
		T y2;
		float b0;
		float b1;
		float b2;
		//float a0;
		float a1;
		float a2;
	public:
		BiquadFilter(float sampling_freq, float cutoff_freq,float q_factor){
			reset();
		    set_param(sampling_freq,cutoff_freq,q_factor);
		}

		T operator() (T input)override{
		    float output = b0 * input + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2;

		    x2 = x1;
		    x1 = input;
		    y2 = y1;
		    y1 = output;

		    return output;
		}
		T get(void)const{
			return y1;
		}
		void set_param(float sampling_freq, float cutoff_freq,float q_factor){
			float omega = 2.0 * M_PI * cutoff_freq / sampling_freq;
			float alpha = sin(omega) / (2.0 * q_factor);
			float cos_omega = cos(omega);
			float a0_inv = 1.0 / (1.0 + alpha);

			b0 = (1.0 - cos_omega) / 2.0 * a0_inv;
			b1 = (1.0 - cos_omega) * a0_inv;
			b2 = (1.0 - cos_omega) / 2.0 * a0_inv;
			a1 = -2.0 * cos_omega * a0_inv;
			a2 = (1.0 - alpha) * a0_inv;
		}
		void reset(void)override{
			x1 = static_cast<T>(0);
			x2 = static_cast<T>(0);
			y1 = static_cast<T>(0);
			y2 = static_cast<T>(0);
		}
	};

	//後退差分による一次HPF
	template<Arithmetic T>
	class HighpassFilterBD:public IFilter<T>{
	private:
		T data = static_cast<T>(0);
		T prev_input = static_cast<T>(0);
		const float k = 0.0f;
	public:
		//G(s)=tau*s / (tau*s + 1)を後退差分で離散化
		HighpassFilterBD(float _k):k(_k){}
		HighpassFilterBD(float f_sample, float f_cutoff):k(2*M_PI*f_cutoff/(f_sample + 2*M_PI*f_cutoff)){}

		T operator() (T input)override{
			data = static_cast<T>(k*static_cast<float>(input - prev_input + data));
			prev_input = input;
			return data;
		}
		T get(void)const{
			return data;
		}
		void overwrite(T val){
			data = val;
		}
		void reset(void)override{
			data = static_cast<T>(0);
		}
	};
}


#endif /* COMMONLIB_FILTER_HPP_ */
