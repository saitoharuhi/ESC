/*
 * LED_pattern.hpp
 *
 *  Created on: Jul 11, 2025
 *      Author: gomas
 */

#ifndef LED_PATTERN_HPP_
#define LED_PATTERN_HPP_


#include "CommonLib/sequencer.hpp"

namespace BoardLib::LEDPattern{

	inline constexpr CommonLib::Note ok[] = {
		{1.0f,100},
		{0.0f,100},
		CommonLib::Note::end_of_sequence<float>(0.0f)
	};
	inline constexpr CommonLib::Note running[] = {
		{0.2f,100},
		{0.0f,900},
		CommonLib::Note::end_of_sequence<float>(0.0f)
	};
	inline constexpr CommonLib::Note error[]={
		{1.0f,100},
		{0.0f,100},
		{1.0f,700},
		{0.0f,100},
		CommonLib::Note::end_of_sequence<float>(0.0f)
	};

	inline constexpr CommonLib::Note sos[]={
		{1.0f,100},
		{0.0f,100},
		{1.0f,100},
		{0.0f,100},
		{1.0f,100},
		{0.0f,100},

		{1.0f,500},
		{0.0f,100},
		{1.0f,500},
		{0.0f,100},
		{1.0f,500},
		{0.0f,100},

		{1.0f,100},
		{0.0f,100},
		{1.0f,100},
		{0.0f,100},
		{1.0f,100},
		{0.0f,1000},
		CommonLib::Note::end_of_sequence<float>(0.0f)
	};

	inline const CommonLib::Note calibrating[] = {
		{1.0f,100},
		{0.0f,100},
		CommonLib::Note::end_of_sequence<float>(0.0f)
	};

	inline const CommonLib::Note pwm_mode[] = {
		{0.0f,  1},
		{1.0f,100},
		{0.0f,1899},
		CommonLib::Note::end_of_sequence<float>(0.0f)
	};
	inline const CommonLib::Note speed_mode[] = {
		{0.0f,  1},
		{1.0f,100},
		{0.0f,200},
		{1.0f,100},
		{0.0f,1599},
		CommonLib::Note::end_of_sequence<float>(0.0f)
	};
	inline const CommonLib::Note position_mode[] = {
		{0.0f,1},
		{1.0f,100},
		{0.0f,200},
		{1.0f,100},
		{0.0f,200},
		{1.0f,100},
		{0.0f,1299},

		CommonLib::Note::end_of_sequence<float>(0.0f)
	};

	inline const CommonLib::Note abs_pwm_mode[] = {
		{0.0f,1},
		{1.0f,500},
		{0.0f,200},
		{1.0f,100},
		{0.0f,1299},
		CommonLib::Note::end_of_sequence<float>(0.0f)
	};
	inline const CommonLib::Note abs_speed_mode[] = {
		{0.0f,  1},
		{1.0f,500},
		{0.0f,200},
		{1.0f,100},
		{0.0f,200},
		{1.0f,100},
		{0.0f,899},

		CommonLib::Note::end_of_sequence<float>(0.0f)
	};
	inline const CommonLib::Note abs_position_mode[] = {
		{0.0f,  1},
		{1.0f,500},
		{0.0f,200},
		{1.0f,100},
		{0.0f,200},
		{1.0f,100},
		{0.0f,200},
		{1.0f,100},
		{0.0f,599},

		CommonLib::Note::end_of_sequence<float>(0.0f)
	};

	inline const CommonLib::Note* led_mode_indicate[2][3]={
		{BoardLib::LEDPattern::pwm_mode,BoardLib::LEDPattern::speed_mode,BoardLib::LEDPattern::position_mode},
		{BoardLib::LEDPattern::abs_pwm_mode,BoardLib::LEDPattern::abs_speed_mode,BoardLib::LEDPattern::abs_position_mode}
	};

	inline const CommonLib::Note vesc_only[] = {
		{0.0f,  1},
		{1.0f,1000},
		{0.0f,999},

		CommonLib::Note::end_of_sequence<float>(0.0f)
	};

	inline constexpr CommonLib::Note running0[] = {
		{0.2f,100},
		{0.0f,1400},
		CommonLib::Note::end_of_sequence<float>(0.0f)
	};
	inline constexpr CommonLib::Note running1[] = {
		{0.2f,100},
		{0.0f,200},
		{0.2f,100},
		{0.0f,1100},
		CommonLib::Note::end_of_sequence<float>(0.0f)
	};
	inline constexpr CommonLib::Note running2[] = {
		{0.2f,100},
		{0.0f,200},
		{0.2f,100},
		{0.0f,200},
		{0.2f,100},
		{0.0f,800},
		CommonLib::Note::end_of_sequence<float>(0.0f)
	};
	inline constexpr CommonLib::Note running3[] = {
		{0.2f,100},
		{0.0f,200},
		{0.2f,100},
		{0.0f,200},
		{0.2f,100},
		{0.0f,200},
		{0.2f,100},
		{0.0f,500},
		CommonLib::Note::end_of_sequence<float>(0.0f)
	};

	inline const CommonLib::Note* indicate_id[4]={
		running0,
		running1,
		running2,
		running3
	};
}


#endif /* LED_PATTERN_HPP_ */
