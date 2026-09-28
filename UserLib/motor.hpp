/*
 * motor.hpp
 *
 *  Created on: Oct 10, 2024
 *      Author: gomas
 */

#ifndef MOTOR_HPP_
#define MOTOR_HPP_

#include "main.h"

#include "CommonLib/pwm.hpp"
#include "CommonLib/Math/motor_math.hpp"
#include "CommonLib/Math/sin_table.hpp"
#include "CommonLib/Math/pid.hpp"
#include "CommonLib/encoder.hpp"
#include "CommonLib/Math/disturbance_observer.hpp"

#include <optional>
#include <utility>
#include <bit>
#include <cmath>

namespace BoardLib{
///////////////////////////////////////////////////////////////////////////////////////////////
//モーター制御用クラス
///////////////////////////////////////////////////////////////////////////////////////////////
class Motor{
	//SPMSMを想定(Ld=Lq)
	static constexpr std::pair<float,float> calc_pi_gain(float _motor_R, float _motor_L,float natural_frequency,float damping_ratio){
		float omega_n = natural_frequency * 2 * M_PI;
		//二次標準式ゲイン
//			float kp = 2.0f*damping_ratio*omega_n*_motor_L - _motor_R;//omega_n*_motor_L;
//			float ki = omega_n*omega_n*_motor_L;//omega_n*_motor_R;

		//極零相殺ゲイン
		float kp = omega_n*_motor_L;
		float ki = omega_n*_motor_R;

		return std::pair<float,float>{kp,ki};
	}

	static constexpr float duty_limit = 0.9f;//duty90%までしか出さない
	static constexpr float dead_time_comp_th = 0.1;//デッドタイム補償の不感帯
	const float operation_freq;
	const float dead_time_duty;
	const float delay_compensation_coef;
	float R;
	float L;
	float phi; //e_speed[rad/s]*phi = voltage
	const CommonLib::Math::SinTable<12>* table;

	float Kb;
public:
	//デバッグ用で外に出してるだけなやつら
	CommonLib::Math::TrackingPIController d_i_pi;
	CommonLib::Math::TrackingPIController q_i_pi;

	CommonLib::PWMHard u,v,w;

	CommonLib::Math::DQ dq_v;

	Motor(
			float _operation_freq,
			float motor_R,
			float motor_L,
			float motor_phi,
			const CommonLib::Math::SinTable<12>* _table,
			CommonLib::PWMHard _u,
			CommonLib::PWMHard _v,
			CommonLib::PWMHard _w,
			float _dead_time,
			float i_control_natual_freq,
			float damping_ratio)
		:operation_freq(_operation_freq),
		 dead_time_duty(_dead_time*0.5*_operation_freq),
		 delay_compensation_coef(1.5f/operation_freq),
		 R(motor_R),
		 L(motor_L),
		 phi(motor_phi),
		 table(_table),
		 Kb(0.0f),
		 d_i_pi(operation_freq,0.0f,0.0f),
		 q_i_pi(operation_freq,0.0f,0.0f),
		 u(_u),
		 v(_v),
		 w(_w){

		update_motor_param(R,L,phi,i_control_natual_freq,damping_ratio);
	}

	void start(void){
		u.start();
		v.start();
		w.start();
	}
	void stop(void){
		u.stop();
		v.stop();
		w.stop();
	}

	void soft_stop(void){
		u(0.0f);
		v(0.0f);
		w(0.0f);
	}

	void drive(CommonLib::Math::DQ dq_duty, q15_t angle){ //duty:-1~1
		CommonLib::Math::UVW tmp = dq_duty.to_uvw(table->sin_cos(angle));
		u(tmp.u*0.5 + 0.5f);
		v(tmp.v*0.5 + 0.5f);
		w(tmp.w*0.5 + 0.5f);
	}

	void drive(CommonLib::Math::UVW uvw_duty){ //duty:-1~1
		u(uvw_duty.u*0.5 + 0.5f);
		v(uvw_duty.v*0.5 + 0.5f);
		w(uvw_duty.w*0.5 + 0.5f);
	}

	void current_pi(float batt_v,CommonLib::Math::DQ dq_i,CommonLib::Math::DQ target_dq_i,q15_t e_angle,int32_t electrical_velocity){
		//電流PI ゲインがデカすぎるとdq_vがnanになるバグ？アリ
		dq_v = CommonLib::Math::DQ{
			.d = d_i_pi(target_dq_i.d,dq_i.d),
			.q = q_i_pi(target_dq_i.q,dq_i.q)
		};

		//FF成分の計算と加算
		//PI制御器でid=0一定にできると仮定→iqへの干渉項が消える→phiによる起電力へのFFのみでOK
		//電流のノイズが大きく下手にFFすると不安定になるのでこの設計とした
		constexpr float q_to_rad = 2.f*M_PI / static_cast<float>(1<<16);
		dq_v.q += phi*static_cast<float>(electrical_velocity)*q_to_rad;

		//d軸電圧を保持してq軸電圧をクランプ
		//std::sqrtfの最適化が必要なため-Ofast推奨
//		constexpr float clamp_voltage_coef_sq = (duty_limit*0.5f)*(duty_limit*0.5f);//SVMしない時用
		constexpr float clamp_voltage_coef_sq = (2.f/sqrtf(3.f)*duty_limit*0.5f)*(2.f/sqrtf(3.f)*duty_limit*0.5f);
		float v_inv = 1.f/batt_v;
		float vq_clamp_coef = sqrtf((clamp_voltage_coef_sq - dq_v.d*dq_v.d*v_inv*v_inv)/(dq_v.q*dq_v.q));
		auto dq_duty = CommonLib::Math::DQ{ //-0.5~0.5
			.d = dq_v.d*v_inv,
			.q = dq_v.q*std::min(v_inv,vq_clamp_coef)
		};
		//アンチワインドアップ(dutyの反映が最優先なので後回しでもよい)
		d_i_pi.subtraction_from_error_sum((dq_v.d - dq_duty.d*batt_v)*Kb);
		q_i_pi.subtraction_from_error_sum((dq_v.q - dq_duty.q*batt_v)*Kb);

		CommonLib::Math::SinCos sin_cos = table->sin_cos(e_angle);// + static_cast<float>(electrical_velocity)*delay_compensation_coef);
		auto uvw_duty = dq_duty.to_uvw(sin_cos).sv_modulation().sperimposition(0.5);//SVMしてから0.5重畳してで0~1のdutyに

		CommonLib::Math::UVW target_uvw_i = target_dq_i.to_uvw(sin_cos);

		//デッドタイム補償しつつdutyをセット
		u(uvw_duty.u + (target_uvw_i.u>dead_time_comp_th ? dead_time_duty :(target_uvw_i.u<-dead_time_comp_th ? -dead_time_duty:0.0f)));
		v(uvw_duty.v + (target_uvw_i.v>dead_time_comp_th ? dead_time_duty :(target_uvw_i.v<-dead_time_comp_th ? -dead_time_duty:0.0f)));
		w(uvw_duty.w + (target_uvw_i.w>dead_time_comp_th ? dead_time_duty :(target_uvw_i.v<-dead_time_comp_th ? -dead_time_duty:0.0f)));
//		u(uvw_duty.u);
//		v(uvw_duty.v);
//		w(uvw_duty.w);
	}

	void update_motor_param(float motor_R,float motor_L,float motor_phi,float _i_control_natural_freq, float damping_ratio){
		auto [kp,ki] = calc_pi_gain(motor_R,motor_L,_i_control_natural_freq,damping_ratio);

		d_i_pi.reset();
		d_i_pi.set_p_gain(kp);
		d_i_pi.set_i_gain(ki);

		q_i_pi.reset();
		q_i_pi.set_p_gain(kp);
		q_i_pi.set_i_gain(ki);
		Kb = sqrtf(ki/kp);
		R = motor_R;
		L = motor_L;
		phi = motor_phi;
	}
};
}//BoardLib




#endif /* MOTOR_HPP_ */
