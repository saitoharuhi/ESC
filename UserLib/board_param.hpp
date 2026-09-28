/*
 * board_param.hpp
 *
 *  Created on: Nov 30, 2025
 *      Author: gomas
 */

#ifndef BOARD_PARAM_HPP_
#define BOARD_PARAM_HPP_

#include "CommonLib/Protocol/id_defines.hpp"
#include "CommonLib/flash_management.hpp"
#include "CommonLib/Math/motor_math.hpp"

namespace BoardLib{

struct MotorParams{
	int pole_n = 7;
	float R = 461.0e-3;
	float L = 64.22e-6;
	float phi = 1.0f/(32.96f/60.0f*2.0f*M_PI*7);
	float i_control_natural_freq = 250.f;
	float damping_ratio = 1.0f;

	int32_t iv_bias = 2048;
	int32_t iw_bias = 2048;

	q15_t e_angle_bias = 0;

	//ロボマスエンコーダ系パラメータ
	q15_t sinenc_bias = 1200;
	q15_t sinenc_gain = 16;

	q15_t cosenc_bias = 1200;
	q15_t cosenc_gain = 16;
};

inline constexpr auto default_motor_param = MotorParams{
	.pole_n = 7,
	.R = 461.0e-3,
	.L = 64.22e-6,
	.phi = 1.0f/(32.96f/60.0f*2.0f*M_PI*7),
	.i_control_natural_freq = 250.f,
	.damping_ratio = 1.0f,

	.iv_bias = 2048,
	.iw_bias = 2048,

	.e_angle_bias = 0,

	.sinenc_bias = 1200,
	.sinenc_gain = 16,

	.cosenc_bias = 1200,
	.cosenc_gain = 16,
};

struct BoardParams{
	CommonLib::FlashState f_state = CommonLib::FlashState::RESET;
	size_t id = 0;
	MReg::RobomasMD feedback_format = MReg::RobomasMD::C610;
	sMDUReg::EncType enc_type = sMDUReg::EncType::Robomas;
	bool is_dc_motor = false;

	float spd_kp = 0.12f;
	float spd_ki = 0.7f;
	float spd_kd = 0.0f;
	float crr_limit = 5.f;

	float pos_kp = 0.4;
	float pos_ki = 0.0f;
	float pos_kd = 0.0f;
	float spd_limit = 500.f;

	uint8_t robomas_feedback_enable = 0b1;
	uint32_t usb_feedback_period = 0;

	MotorParams m_params = default_motor_param;
};

inline constexpr auto default_board_param = BoardParams{
	.f_state = CommonLib::FlashState::RESET,
	.id = 0,
	.feedback_format = MReg::RobomasMD::C610,
	.enc_type = sMDUReg::EncType::Robomas,
	.is_dc_motor = false,

	.spd_kp = 0.12f,
	.spd_ki = 0.07f,
	.spd_kd = 0.0f,
	.crr_limit = 5.f,

	.pos_kp = 0.4f,
	.pos_ki = 0.0f,
	.pos_kd = 0.0f,
	.spd_limit = 500.f,

	.robomas_feedback_enable = 0b1,
	.usb_feedback_period = 0,

	.m_params = default_motor_param,
};

}//BoardLib



#endif /* BOARD_PARAM_HPP_ */
