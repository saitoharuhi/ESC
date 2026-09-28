/*
 * motor_param.hpp
 *
 *  Created on: Jul 19, 2025
 *      Author: gomas
 */

#ifndef ROBOMAS_PARAM_HPP_
#define ROBOMAS_PARAM_HPP_

#include "CommonLib/Protocol/id_defines.hpp"
#include <cstdint>
#include <algorithm>

namespace BoardLib{
namespace RobomasMotorParam{
	//ロボマスモーターのモーターパラメータ
	inline constexpr float torque_coef[2] = {0.18f, 0.3f}; //トルク定数[Nm/A]
	inline constexpr float torque_coef_inv[2] = {1.0f/0.18f, 1.0f/0.3f}; //トルク定数の逆数[A/(N/m)]
	inline constexpr float gear_ratio[2] = {36.0f,3591.0f/187.0f};
	inline constexpr float current_limit[2] = {10.0f,20.0f};
	inline constexpr float current_to_robomas_param_coef[2] = {1000.0f,16384.0f/20.0f};
	inline constexpr float robomas_param_to_current_coef[2] = {1.0f/1000.0f,20.0f/16384.0f};

	inline constexpr float get_gear_ratio(MReg::RobomasMD m_type){
		return gear_ratio[static_cast<size_t>(m_type)];
	}
	inline constexpr int16_t torque_to_robomas_value(MReg::RobomasMD m_type, float torque){
		int index = static_cast<size_t>(m_type);
		float current = std::clamp(torque*torque_coef_inv[index], -current_limit[index], current_limit[index]);
		return static_cast<int16_t>(current * current_to_robomas_param_coef[index]);
	}

	inline constexpr float robomas_value_to_torque(MReg::RobomasMD m_type, int16_t value){
		int index = static_cast<size_t>(m_type);
		return static_cast<float>(value) * (robomas_param_to_current_coef[index]*torque_coef_inv[index]);
	}

	inline constexpr float robomas_value_to_current(MReg::RobomasMD m_type, int16_t value){
		int index = static_cast<size_t>(m_type);
		return static_cast<float>(value) * robomas_param_to_current_coef[index];
	}
	inline constexpr int16_t current_to_robomas_value(MReg::RobomasMD m_type, float current){
		int index = static_cast<size_t>(m_type);
		return static_cast<float>(current) * current_to_robomas_param_coef[index];
	}
};

}//BoardLib

#endif /* ROBOMAS_PARAM_HPP_ */
