/*
 * id_defines.hpp
 *
 *  Created on: Jul 16, 2025
 *      Author: gomas
 */

#ifndef COMMONLIB_PROTOCOL_ID_DEFINES_HPP_
#define COMMONLIB_PROTOCOL_ID_DEFINES_HPP_

#include <cstddef>
namespace CommonLib::Protocol{
enum class DataType:size_t{
	COMMON_ID,
	PCU_ID,
	MDC_ID,
	GPIO_ID,
	MDC2_ID,
	LED_ID,
	sMDU_ID,
	COMMON_ID_ENFORCE = 0xF
};

namespace CommonRegister{
	inline constexpr size_t NOP             = 0x0000;
	inline constexpr size_t ID_RQ           = 0x0001;
	inline constexpr size_t SAVE_PARAM      = 0x0002;
	inline constexpr size_t RESET_PARAM     = 0x0003;
	inline constexpr size_t EMS             = 0x000E;
	inline constexpr size_t RESET_EMS       = 0x000F;
}

namespace PCURegister{
	inline constexpr size_t NOP             = 0x0000;
	inline constexpr size_t PCU_STATE       = 0x0001;
	inline constexpr size_t CELL_N          = 0x0002;
	inline constexpr size_t EX_EMS_TRG      = 0x0003;
	inline constexpr size_t EMS_RQ          = 0x0004;
	inline constexpr size_t COMMON_EMS_EN   = 0x0005;

	inline constexpr size_t BATT_V          = 0x0010;
	inline constexpr size_t V_LIMIT_HIGH    = 0x0011;
	inline constexpr size_t V_LIMIT_LOW     = 0x0012;

	inline constexpr size_t BATT_I          = 0x0020;
	inline constexpr size_t I_LIMIT         = 0x0021;

	inline constexpr size_t MONITOR_PERIOD  = 0x00F0;
	inline constexpr size_t MONITOR_REG     = 0x00F1;

	namespace PCUStateBitPos{
		inline constexpr size_t EMS = 0;
		inline constexpr size_t SOFT_EMS = 1;
		inline constexpr size_t OVA = 2;
		inline constexpr size_t UVA = 3;
		inline constexpr size_t OIA = 4;
	}
}

namespace MDCRegister{
	inline constexpr size_t NOP             = 0x0000;
	inline constexpr size_t MOTOR_STATE     = 0x0001;
	inline constexpr size_t CONTROL         = 0x0002;

	inline constexpr size_t ABS_GEAR_RATIO  = 0x0005;
	inline constexpr size_t CAL_RQ          = 0x0006;
	inline constexpr size_t LOAD_J          = 0x0007;
	inline constexpr size_t LOAD_D          = 0x0008;
	inline constexpr size_t DOB_CF          = 0x0009;
	inline constexpr size_t OMEGA_N         = 0x000A;

	inline constexpr size_t CAN_TIMEOUT     = 0x000F;

	inline constexpr size_t TRQ             = 0x0010;
	inline constexpr size_t TRQ_TARGET      = 0x0011;

	inline constexpr size_t SPD             = 0x0020;
	inline constexpr size_t SPD_TARGET      = 0x0021;
	inline constexpr size_t TRQ_LIM         = 0x0022;
	inline constexpr size_t SPD_GAIN_P      = 0x0023;
	inline constexpr size_t SPD_GAIN_I      = 0x0024;
	inline constexpr size_t SPD_GAIN_D      = 0x0025;

	inline constexpr size_t POS             = 0x0030;
	inline constexpr size_t POS_TARGET      = 0x0031;
	inline constexpr size_t SPD_LIM         = 0x0032;
	inline constexpr size_t POS_GAIN_P      = 0x0033;
	inline constexpr size_t POS_GAIN_I      = 0x0034;
	inline constexpr size_t POS_GAIN_D      = 0x0035;

	inline constexpr size_t ABS_POS         = 0x003A;
	inline constexpr size_t ABS_SPD         = 0x003B;
	inline constexpr size_t ABS_TURN_CNT    = 0x003C;

	inline constexpr size_t VESC_MODE       = 0x0040;
	inline constexpr size_t VESC_TARGET     = 0x0041;
	inline constexpr size_t VESC_VOLTAGE    = 0x0042;
	inline constexpr size_t VESC_CURRENT    = 0x0043;
	inline constexpr size_t VESC_ERPM       = 0x0044;

	inline constexpr size_t MONITOR_PERIOD  = 0x00F0;
	inline constexpr size_t MONITOR_REG1    = 0x00F1;
	inline constexpr size_t MONITOR_REG2    = 0x00F2;

	namespace  ControlBitPos{
		inline constexpr size_t MODE = 0;
		inline constexpr size_t MOTOR = 2;
		inline constexpr size_t DOB_EN = 4;
		inline constexpr size_t ABS_EN = 5;
		inline constexpr size_t MD_GUESS_EN = 6;
		inline constexpr size_t KEEP_MODE = 7;
	}

	enum class ControlMode:size_t{
		OPEN_LOOP,
		SPEED,
		POSITION,
	};
	enum class VescMode:size_t{
		NOP,
		PWM,
		CURRENT,
		SPEED,
		POSITION
	};

	enum class RobomasMD{
		C610,
		C620
	};

	inline constexpr size_t monitable_list[] = {
		NOP,
		MOTOR_STATE,
		CONTROL,
		CAL_RQ,
		TRQ,
		TRQ_TARGET,
		SPD,
		SPD_TARGET,
		POS,
		POS_TARGET,
		ABS_POS,
		ABS_SPD,
		ABS_TURN_CNT,
		VESC_VOLTAGE,
		VESC_CURRENT,
		VESC_ERPM,
	};
}

namespace GPIORegister{
	inline constexpr size_t NOP             = 0x0000;
	inline constexpr size_t PORT_MODE       = 0x0001;
	inline constexpr size_t PORT_READ       = 0x0002;
	inline constexpr size_t PORT_WRITE      = 0x0003;
	inline constexpr size_t PORT_INT_EN     = 0x0004;
	inline constexpr size_t ESC_MODE_EN     = 0x0005;

	inline constexpr size_t PWM_PERIOD      = 0x0010;
	inline constexpr size_t PWM_DUTY        = 0x0020;

	inline constexpr size_t MONITOR_PERIOD  = 0x00F0;
	inline constexpr size_t MONITOR_REG     = 0x00F1;
}

namespace LEDRegister{
	inline constexpr size_t NOP             = 0x0000;

	inline constexpr size_t LED_MODE        = 0x0040;
	inline constexpr size_t ENABLE_AUTO_TRANSITION = 0x0041;
	inline constexpr size_t EMG_BLINK_PERIOD = 0x0042;
	inline constexpr size_t EMG_COLOR       = 0x0050;
	inline constexpr size_t LED_REF         = 0x0060;

	inline constexpr size_t MONITOR_PERIOD  = 0x00F0;
	inline constexpr size_t MONITOR_REG     = 0x00F1;
}
namespace sMDURegister{
	inline constexpr size_t NOP             = 0x0000;
	inline constexpr size_t STATE           = 0x0001;
	inline constexpr size_t ERROR_FLAG      = 0x0002;
	inline constexpr size_t MOTOR_TYPE      = 0x0003;
	inline constexpr size_t R               = 0x0004;
	inline constexpr size_t L               = 0x0005;
	inline constexpr size_t PHI             = 0x0006;
	inline constexpr size_t POLE_N          = 0x0007;
	inline constexpr size_t CTRL_NATURAL_FREQ = 0x0008;
	inline constexpr size_t CTRL_DAMPING_RATIO = 0x0009;
	inline constexpr size_t CAL_RQ          = 0x000A;
	inline constexpr size_t ROBOMAS_FB_EN   = 0x000B;
	inline constexpr size_t CONTROL_MODE    = 0x000C;
	inline constexpr size_t PARAMS_APPLY    = 0x000F;

	inline constexpr size_t CURRENT         = 0x0010;
	inline constexpr size_t CURRENT_TARGET  = 0x0011;

	inline constexpr size_t SPD             = 0x0020;
	inline constexpr size_t SPD_TARGET      = 0x0021;
	inline constexpr size_t CURRENT_LIM     = 0x0022;
	inline constexpr size_t SPD_GAIN_P      = 0x0023;
	inline constexpr size_t SPD_GAIN_I      = 0x0024;
	inline constexpr size_t SPD_GAIN_D      = 0x0025;
	inline constexpr size_t SPD_GAIN_ANTIWINDUP = 0x0026;

	inline constexpr size_t POS             = 0x0030;
	inline constexpr size_t POS_TARGET      = 0x0031;
	inline constexpr size_t SPD_LIM         = 0x0032;
	inline constexpr size_t POS_GAIN_P      = 0x0033;
	inline constexpr size_t POS_GAIN_I      = 0x0034;
	inline constexpr size_t POS_GAIN_D      = 0x0035;
	inline constexpr size_t POS_GAIN_ANTIWINDUP = 0x0036;

	inline constexpr size_t BOARD_TEMP      = 0x0040;

	inline constexpr size_t USB_FB_PERIOD  = 0x00F0;

	enum class EncType{
		Robomas = 0x0,
		AS5048  = 0x1,
		AS5600 = 0x2,
		AMT21x = 0x3,
	};

	enum class OpMode{
		EMS,
		NORMAL,
		CALIBRATION
	};

	namespace ErrorFlagBitPos{
		inline constexpr size_t CURRENT = 0u;
		inline constexpr size_t VOLTAGE = 1u;
		inline constexpr size_t ENCODER = 2u;
		inline constexpr size_t MOTOR_TYPE = 3u;
		inline constexpr size_t BOARD_TEMP = 4u;

		inline constexpr size_t EXTERNAL_EMS = 6u;
		inline constexpr size_t PARAM_UPDATE = 7u;
	}

	enum class ControlMode:size_t{
		CURRENT,
		SPEED,
		POSITION,
	};
}


} //namespace CommonLib::Protocol

namespace CReg = CommonLib::Protocol::CommonRegister;
namespace PReg = CommonLib::Protocol::PCURegister;
namespace MReg = CommonLib::Protocol::MDCRegister;
namespace GReg = CommonLib::Protocol::GPIORegister;
namespace LEDReg = CommonLib::Protocol::LEDRegister;
namespace sMDUReg = CommonLib::Protocol::sMDURegister;




#endif /* COMMONLIB_PROTOCOL_ID_DEFINES_HPP_ */
