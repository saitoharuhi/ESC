/*
 * calibration.hpp
 *
 *  Created on: Nov 25, 2025
 *      Author: gomas
 */

#ifndef CALIBRATION_HPP_
#define CALIBRATION_HPP_

#include "board_param.hpp"
#include "motor.hpp"
#include <utility>
#include <algorithm>
#include <cmath>

namespace BoardLib{
/////////////////////////////////////////////////////////////////////////////////////////
//モーターのキャリブレーションをいろいろするクラス
/////////////////////////////////////////////////////////////////////////////////////////

//電気系キャリブレーション
class MotorCalibrator{
public:
	enum class State{
		RESET,   //測定完了
		WAIT,
		I_SENS_CHECK,
		POS_INIT,
		MEASURE_R,
		MEASURE_L,

		ENC_AMP_TEST1,//ロボマスモータ用テスト
		ENC_AMP_TEST2,//ロボマスモータ用テスト

		POLE_N,//その他のモーター用テスト

		MOVE_TO_POS_ORIGIN,
		ANG_BIAS,
		FINISH,
		ERROR = -1 //測定失敗
	};
private:
	//時定数測定用クラス
	//いろいろいじってたらかなり汚い感じになってしまったので直したい
	class TimeConstantStopWatch{
		int32_t  cnt = 0;
		float trigger_val;
		bool finished = true;
		const MotorParams *mp;
		float i_sens_gain;
		CommonLib::Math::SinCos sc;
	public:
		void start(float _trigger_val,const MotorParams *_mp,float _i_sens_gain,CommonLib::Math::SinCos _sc){
			mp = _mp;
			i_sens_gain = _i_sens_gain;
			trigger_val = _trigger_val;
			sc = _sc;

			cnt = 0;
			finished = false;
		}
		void update(int32_t iv_adc,int32_t iw_adc){ //観測値が一定の値を下回ったらカウントストップ
			CommonLib::Math::UVW i_uvw;
			i_uvw.v = static_cast<float>(iv_adc - mp->iv_bias)*i_sens_gain;
			i_uvw.w = static_cast<float>(iw_adc - mp->iw_bias)*i_sens_gain;
			i_uvw.u = -i_uvw.v -i_uvw.w;
			CommonLib::Math::DQ i_dq = i_uvw.to_dq(sc);

			if(!finished){
				cnt ++;
				if(i_dq.d < trigger_val){
					finished = true;
				}
			}
		}
		bool is_finished(void)const{
			return finished;
		}
		int get_count(void)const{
			return cnt;
		}
	};


	const float operation_freq;
	const CommonLib::Math::SinTable<12>* table;
	const float pwm_freq;
	const float i_sens_gain;

	MotorParams measured_params;

	int cnt = 0;
	State state = State::RESET;

	float rad = 0.0f;

	float i_ave = 0.f;
	uint16_t enc_sin_max = 0;
	uint16_t enc_cos_max = 0;
	uint16_t enc_cos_min = 10000;
	uint16_t enc_sin_min = 10000;

	//キャリブレーションに使用する各値
	bool is_sincos_encoder = true;
	float meas_voltage = 0.5f; //各パラメータを測定する際にモータに印加する電圧
	//↑モータがちゃんと駆動される&&過電流にならない電圧にすること
	float meas_current = 5.f;//

public:
	TimeConstantStopWatch stop_watch;

	MotorCalibrator(
			float _operation_freq,
			const CommonLib::Math::SinTable<12>* _table,
			float _pwm_freq,
			float _i_sens_gain)
	:operation_freq(_operation_freq),
	 table(_table),
	 pwm_freq(_pwm_freq),
	 i_sens_gain(_i_sens_gain){
	}

	void reset_calibration(void){
		cnt = 0;
		rad = 0;
		state = State::RESET;
	}
	State get_state(void)const{
		return state;
	}

	void set_caliblation_param(bool _is_sincos_enc,float measurement_voltage,float measurement_current,int pole_n){
		is_sincos_encoder = _is_sincos_enc;
		meas_voltage = measurement_voltage;
		meas_current = measurement_current;
		measured_params.pole_n = abs(pole_n);
	}

	//電気角，テーブル，生電流，エンコーダ生値
	//戻り値のStateがResetになったらキャリブレーション完了
	std::pair<CommonLib::Math::UVW,State> caliblation(q15_t raw_angle,int32_t iv_adc,int32_t iw_adc,float batt_v,uint16_t enc_sin_raw,uint16_t enc_cos_raw){

		CommonLib::Math::UVW pwm;
		switch(state){
		case State::RESET:{
			state = State::WAIT;
			cnt = 0;
			rad = 0;
			i_ave = 0.f;
			enc_sin_max = 0;
			enc_cos_max = 0;
			enc_cos_min = 10000;
			enc_sin_min = 10000;
			pwm = CommonLib::Math::UVW{0,0,0};
		}
		break;
		case State::WAIT:{
			cnt ++;
			pwm = CommonLib::Math::UVW{0,0,0};
			if(cnt >= 1.0*operation_freq){
				cnt = 0;
				state = State::I_SENS_CHECK;
			}
		}break;
		case State::I_SENS_CHECK:{
			static int32_t iv_bias = 0;
			static int32_t iw_bias = 0;
			iv_bias += iv_adc;
			iw_bias += iw_adc;

			pwm = CommonLib::Math::UVW{0,0,0};
			cnt ++;

			if(cnt >= 256){
				measured_params.iv_bias = iv_bias/256;
				measured_params.iw_bias = iw_bias/256;
				iv_bias = 0;
				iw_bias = 0;
				cnt = 0;
				state = State::POS_INIT;
			}
		}
		break;
		case State::POS_INIT:{
			cnt ++;
			pwm = CommonLib::Math::DQ{(static_cast<int>(cnt/(1.0f*operation_freq)))*meas_voltage/batt_v,0.f}.to_uvw(table->sin_cos(0.f));

			if(cnt > static_cast<int>(1.0f*operation_freq)){
				state = State::MEASURE_R;
				cnt = 0;
				i_ave = 0;
			}
		}
		break;
		case State::MEASURE_R:{
			static int m_cnt = 0;
			cnt ++;
			pwm = CommonLib::Math::DQ{meas_voltage/batt_v,0.f}.to_uvw(table->sin_cos(0.f));
			if(cnt > static_cast<int>(0.3f*operation_freq)){
				CommonLib::Math::UVW i_uvw;
				i_uvw.v = static_cast<float>(iv_adc-measured_params.iv_bias)*i_sens_gain;
				i_uvw.w = static_cast<float>(iw_adc-measured_params.iw_bias)*i_sens_gain;
				i_uvw.u = -i_uvw.v-i_uvw.w;
				CommonLib::Math::DQ i_dq = i_uvw.to_dq(table->sin_cos(0.f));
				i_ave += i_dq.d;
				m_cnt ++;
			}
			if(cnt > static_cast<int>(1.0f*operation_freq)){
				i_ave /= static_cast<float>(m_cnt);
				measured_params.R = meas_voltage/i_ave;

				state = State::MEASURE_L;
				cnt = 0;
				m_cnt = 0;

				stop_watch.start(i_ave/M_E,&measured_params,i_sens_gain,table->sin_cos(0.f));
				pwm = CommonLib::Math::DQ{0,0}.to_uvw(table->sin_cos(0.f));
			}
		}
		break;
		case State::MEASURE_L:{
			pwm = CommonLib::Math::DQ{0,0}.to_uvw(table->sin_cos(0.f));

			if(stop_watch.is_finished()){
				measured_params.L = stop_watch.get_count()/pwm_freq*measured_params.R;
				if(is_sincos_encoder){
					state = State::ENC_AMP_TEST1;
				}else{
					state = State::POLE_N;
//					state = State::ANG_BIAS;
				}
				cnt = 0;
			}
		}
		break;
		//TODO:回転方向検出，極数判別
		case State::ENC_AMP_TEST1:{
			enc_sin_max = std::max(enc_sin_max,enc_sin_raw);
			enc_cos_max = std::max(enc_cos_max,enc_cos_raw);
			enc_sin_min = std::min(enc_sin_min,enc_sin_raw);
			enc_cos_min = std::min(enc_cos_min,enc_cos_raw);

			cnt ++;
			rad = 2.0*M_PI*static_cast<float>(cnt)/operation_freq; //毎秒一回転の速さで回転
			pwm = CommonLib::Math::DQ{meas_voltage/batt_v,0}.to_uvw(table->sin_cos(rad));

			if(cnt > static_cast<int>(3.0f*operation_freq)){
				state = State::ENC_AMP_TEST2;
				cnt = 0;
			}
		}
		break;
		case State::ENC_AMP_TEST2:{
			enc_sin_max = std::max(enc_sin_max,enc_sin_raw);
			enc_cos_max = std::max(enc_cos_max,enc_cos_raw);
			enc_sin_min = std::min(enc_sin_min,enc_sin_raw);
			enc_cos_min = std::min(enc_cos_min,enc_cos_raw);


			cnt ++;
			rad = 2.0*M_PI - (2.0*M_PI*static_cast<float>(cnt)/operation_freq); //毎秒一回転の速さで回転
			pwm = CommonLib::Math::DQ{meas_voltage/batt_v,0}.to_uvw(table->sin_cos(rad));

			if(cnt > static_cast<int>(3.0f*operation_freq)){
				measured_params.sinenc_bias = (enc_sin_max+enc_sin_min)/2;
				measured_params.cosenc_bias = (enc_cos_max+enc_cos_min)/2;
				uint16_t sin_diff = enc_sin_max-enc_sin_min;
				uint16_t cos_diff = enc_cos_max-enc_cos_min;
				if(sin_diff > cos_diff){
					measured_params.cosenc_gain = 16;
					measured_params.sinenc_gain = (16*cos_diff)/sin_diff;
				}else{
					measured_params.sinenc_gain = 16;
					measured_params.cosenc_gain = (16*sin_diff)/cos_diff;
				}

				state = State::MOVE_TO_POS_ORIGIN;
				cnt = 0;
			}
		}
		break;
		case State::POLE_N:{
			static q15_t init_angle = 0;
			if(cnt == 0){
				init_angle = raw_angle;
			}
			cnt ++;
			rad = 2.0*M_PI*static_cast<float>(cnt)/operation_freq; //毎秒一回転の速さで回転
//			pwm = CommonLib::Math::DQ{meas_voltage/batt_v,0}.to_uvw(table->sin_cos(rad));
			pwm = CommonLib::Math::DQ{(meas_current*measured_params.R)/batt_v,0}.to_uvw(table->sin_cos(rad));

			if(cnt > static_cast<int>(1.0f*operation_freq)){
				float diff = static_cast<float>(static_cast<q15_t>(raw_angle - init_angle));//いったんq15_tで計算してからfloatにしましょうね
				if(diff < 0){
					measured_params.pole_n *= -1;
				}
				//本当は極数も自動で測りたいけど結構精度が悪く原点探索に支障が出るのでいったん廃止
//				if(fabs(diff) < 100.f){
//					if(diff>0.f){
//						measured_params.pole_n = 1;
//					}else{
//						measured_params.pole_n = -1;
//					}
//				}else{
//					measured_params.pole_n = static_cast<int>(round(static_cast<float>(1<<16)/diff));//丸めがうまくいかなくてバグるかも
//				}

				state = State::MOVE_TO_POS_ORIGIN;
				cnt = 0;
			}
		}
		break;
		case State::MOVE_TO_POS_ORIGIN:{
			cnt ++;
			rad = 0;
			pwm = CommonLib::Math::DQ{(meas_current*measured_params.R)/batt_v,0}.to_uvw(table->sin_cos(rad));

			if(cnt > static_cast<int>(1.0f*operation_freq)){
				state = State::ANG_BIAS;
				cnt = 0;
			}
		}
		break;
		case State::ANG_BIAS:{
			static float angle_bias = 0.f;
			static int m_cnt = 0;
			cnt ++;
			rad = 0;
//			pwm = CommonLib::Math::DQ{meas_voltage/batt_v,0}.to_uvw(table->sin_cos(rad));
			pwm = CommonLib::Math::DQ{(meas_current*measured_params.R)/batt_v,0}.to_uvw(table->sin_cos(rad));

			if(cnt > static_cast<int>(1.0f*operation_freq)){
				m_cnt ++;
				if(is_sincos_encoder){//ロボマス式エンコーダなら極数は無関係
					angle_bias += static_cast<float>(raw_angle);
				}else{
					angle_bias += static_cast<float>(raw_angle*measured_params.pole_n);
				}
			}

			if(cnt > static_cast<int>(1.0f*operation_freq)){
				angle_bias /= static_cast<float>(m_cnt);
				measured_params.e_angle_bias = static_cast<q15_t>(angle_bias);
				angle_bias = 0.f;

				state = State::FINISH;
				cnt = 0;
				m_cnt = 0;
			}
		}
		break;
		case State::FINISH:{
			cnt = 0;
			rad = 0;
			pwm = CommonLib::Math::UVW{0,0,0};
			state = State::RESET;
		}
		break;
		default:
			//nop;
			break;
		}

		return std::pair(pwm,state);
	}

	const MotorParams& get_motor_params(void)const{
		return measured_params;
	}
	void set_measurement_voltage(float v){
		meas_voltage = v;
	}
};

//TODO:動作確認
//機械系キャリブレーション
class MechanicalCalibrator{
public:
	enum class State{
		RESET,
		ACCELERATION,
		DECELERATION,
		OFF,
		ERROR = -1 //測定失敗
	};
private:
	const int measurement_n;
	const float update_period;
	float steady_spd;

	const int start_up_time;
	const int on_hold_time;
	const int off_time;

	State state = State::ACCELERATION;
	int loop_cnt = 0;
	int cnt = 0;
	float max_spd = 0.0f;

	float J = 0.0f;
	float D = 0.0f;
	float J_ave = 0.0f;
	float D_ave = 0.0f;

	bool error = false;

	CommonLib::Math::PIController pi;
	CommonLib::Math::LowpassFilterBD<float> lpf;

public:
	//測定回数，測定周波数，印加トルク，収束するまでの目安時間
	MechanicalCalibrator(int _measurement_n = 4,float update_freq = 1000.0f,float _steady_spd = 20.0f,float settling_time = 5.0f)
	:measurement_n(_measurement_n),
	 update_period(1.0f/update_freq),
	 steady_spd(_steady_spd),
	 start_up_time(static_cast<int>(settling_time*update_freq*0.02f)),
	 on_hold_time(static_cast<int>(settling_time*update_freq*1.0f)),
	 off_time(static_cast<int>(settling_time*update_freq*1.0f)),
	 pi(CommonLib::Math::PIBuilder(update_freq).set_gain(0.5, 0.1).set_limit(1.0f).build()),
	 lpf(update_freq,update_freq*0.05){
	}

	//[モーターに印加するトルク，キャリブレーション処理を継続するか]
	std::pair<float,State> calibration(float spd){;
		float command_trq = 0.f;

		switch(state){
		case State::RESET:
			reset();
			command_trq = 0.f;
			break;
		case State::ACCELERATION:
			error = false;
			cnt ++;
			if(cnt > on_hold_time){
				D = lpf.get()/spd;
				if(D < 0.0f){
					error = true;
				}
				max_spd = spd;
				cnt = 0;

				state = State::DECELERATION;
				command_trq = 0.f;
			}else{
				command_trq = pi(steady_spd,spd);
				lpf(command_trq);
			}
			break;
		case State::DECELERATION:
			cnt ++;
			command_trq = 0.f;
			if(abs(spd) < abs(max_spd*(1.0/M_E))){
				J = (static_cast<float>(cnt)*update_period)*D;
				cnt = 0;
				state = State::OFF;
			}
			break;
		case State::OFF:
			cnt ++;
			if(cnt > off_time){//十分減速するまで待機
				cnt = 0;
				J_ave += J;
				D_ave += D;

				loop_cnt ++;
				if(loop_cnt >= measurement_n){ //規定回数測定
					J_ave /= measurement_n;
					D_ave /= measurement_n;

					state = State::RESET;
					command_trq = 0.f;
				}else if(error){
					state = State::ERROR;
					command_trq = 0.f;
				}else{ //測定継続
					steady_spd *= -1.0f;
					pi.reset();

					state = State::ACCELERATION;
					command_trq = 0.f;
				}
			}else{
				command_trq = 0.f;
			}
			break;
		case State::ERROR:
			reset();
			break;
		}
		return std::pair<float,State>{command_trq,state};
	}

	float get_inertia(void)const{
		return J_ave;
	}
	float get_friction_coef(void)const{
		return D_ave;
	}
	bool is_failed(void)const{
		return error;
	}
	void reset(void){
		state = State::ACCELERATION;
		loop_cnt = 0;
		cnt = 0;
		max_spd = 0.0f;

		J = 0.0f;
		D = 0.0f;
		J_ave = 0.0f;
		D_ave = 0.0f;
	}
};

}


#endif /* CALIBRATION_HPP_ */
