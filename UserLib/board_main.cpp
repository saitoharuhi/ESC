/*
 * board_main.cpp
 *
 *  Created on: Oct 10, 2024
 *      Author: gomas
 */


#include "main.h"
#include <stdio.h>

#include "usbd_cdc_if.h"
#include "usb_device.h"

#include "cordic.hpp"
#include "LED_pattern.hpp"
#include "motor.hpp"
#include "calibration.hpp"
#include "board_param.hpp"
#include "robomas_param.hpp"
#include "abs_encoder.hpp"
#include "pll.hpp"

#include "CommonLib/button.hpp"
#include "CommonLib/Math/sin_table.hpp"
#include "CommonLib/Math/pid.hpp"
#include "CommonLib/Math/filter.hpp"
#include "CommonLib/fdcan_control.hpp"
#include "CommonLib/gpio.hpp"
#include "CommonLib/slcan.hpp"
#include "CommonLib/serial_if.hpp"
#include "CommonLib/timer_interruption_control.hpp"
#include "CommonLib/usb_cdc.hpp"
#include "CommonLib/sequencer.hpp"
#include "CommonLib/pwm.hpp"
#include "CommonLib/id_map_control.hpp"
#include "CommonLib/flash_management.hpp"

#include <array>
#include <bit>
#include <stdio.h>
#include <optional>

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;

extern CORDIC_HandleTypeDef hcordic;
extern SPI_HandleTypeDef hspi3;
extern I2C_HandleTypeDef hi2c1;

extern FDCAN_HandleTypeDef hfdcan1;

extern OPAMP_HandleTypeDef hopamp2;
extern OPAMP_HandleTypeDef hopamp3;

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim17;
extern USBD_HandleTypeDef hUsbDeviceFS;
extern UART_HandleTypeDef huart2;

namespace Blib = BoardLib;
namespace Clib = CommonLib;

///////////////////////////////////////////////////////////////////////////
//cube mxの値を人力設定
///////////////////////////////////////////////////////////////////////////
constexpr float PWM_FRQ = 34.0e3;
constexpr float DEAD_TIME = 200e-9;
constexpr float MAX_CURR = 28.f;

namespace ADCDet{
	volatile uint32_t &ENC_A = ADC1->JDR1;//sin
	volatile uint32_t &ENC_B = ADC2->JDR3;//cos

	volatile uint32_t &IV = ADC2->JDR1;
	volatile uint32_t &IW = ADC2->JDR2;

	volatile uint32_t &BATT_V = ADC1->JDR2;

	//変換関数
	constexpr float to_current(uint16_t adc_val,uint16_t adc_bias){
		constexpr float amp_gain_inv = 1.0f/15.0f; //cube mxで設定するオペアンプのゲインの逆数
		constexpr float shant_r_inv = 1.0f/0.003f; //シャント抵抗値の逆数

		float v = (adc_val-adc_bias) * 3.3f/static_cast<float>(1<<12);

		return v*amp_gain_inv*shant_r_inv;
	}

	constexpr float to_voltage(uint16_t adc_val){
		constexpr float coef = 3.3f/static_cast<float>(0xFFF);
		return adc_val * coef * 11.0f;
	}

	volatile uint16_t regular_conv_buff[2] = {0};
	volatile uint16_t &MOTOR_TEMP = regular_conv_buff[0];
	volatile uint16_t &BOARD_TEMP = regular_conv_buff[1];
}

namespace Task{
	void dynamic_mem_alloc(void);
	void gpio_init(sMDUReg::EncType e_type,bool estimate_enc_type = false);
	void pwm_io_disable(void);
	void pwm_io_enable(void);
	void can_communication(void);
	void usb_communication(void);

	std::optional<Clib::Protocol::DataPacket> sMDU_data_operation(const Clib::Protocol::DataPacket& dp);
	std::optional<Clib::Protocol::DataPacket> common_data_operation(const Clib::Protocol::DataPacket& dp);

	void robomas_operation(const Clib::CanFrame& cf);
	Clib::CanFrame robomas_feedback(void);

	void error_check(void);
}

namespace BoardElement{
	///////////////////////////////////////////////////////////////////////////
	//モーターの状態変数
	///////////////////////////////////////////////////////////////////////////
	Clib::Math::UVW uvw_i = {.u=0.0f, .v=0.0f, .w=0.0f};
	Clib::Math::AB ab_i = {.a = 0.0f, .b = 0.0f};
	Clib::Math::DQ dq_i = {.d = 0.0f, .q = 0.0f};

	Clib::Math::DQ target_i = {.d = 0.0f, .q =0.0f};

	q15_t e_angle = 0;
	q15_t raw_angle = 0;
	///////////////////////////////////////////////////////////////////////////
	//制御用インスタンスたち
	///////////////////////////////////////////////////////////////////////////
	sMDUReg::OpMode op_mode = sMDUReg::OpMode::EMS;
	uint8_t error_flags = 0;

	constexpr auto table = Clib::Math::SinTable<12>{};
	auto cordic = Blib::FastMathCordic{CORDIC};

	auto e_angle_enc = Clib::ContinuableEncoder{16,PWM_FRQ,1.0f,PWM_FRQ/(500.f)};//ギア比として極対数を設定しろ

	auto motor = Blib::Motor{
		PWM_FRQ,
		461.0e-3,
		64.22e-6,
		1.0f/(32.96f/60.0f*2.0f*M_PI*7),
		&table,
		Clib::PWMHard{&htim1,TIM_CHANNEL_3},
		Clib::PWMHard{&htim1,TIM_CHANNEL_1},
		Clib::PWMHard{&htim1,TIM_CHANNEL_2},
		DEAD_TIME,
		250.0f,
		1.0f
	};

	auto circuit_calibrator = Blib::MotorCalibrator{
		1000.0f,
		&table,
		PWM_FRQ,
		3.3f/(15.f*0.003f*static_cast<float>(1<<12))
	};

//	bool running_mechanical_calibration = false;//いったん廃止
	auto mechanical_calibrator = Blib::MechanicalCalibrator{
		4,
		1000.f,
		20.f,
		5.f,
	};

	Blib::BoardParams b_params;

	auto vbus_voltage_lpf = Clib::Math::LowpassFilterBD<float>{PWM_FRQ,10.f};
	auto motor_temp_lpf = Clib::Math::LowpassFilterBD<float>{1000.f,10.f};
	auto board_temp_lpf = Clib::Math::LowpassFilterBD<float>{1000.f,10.f};

	auto as5048 = Blib::AS5048AEnc{&hspi3,Clib::GPIO{MTEMPSS_ADC2IN3_GPIO_Port,MTEMPSS_ADC2IN3_Pin}};
	auto as5600 = Blib::AS5600Enc{&hi2c1};
	auto amt21x = Blib::AMT21xEnc{&huart2,0x54};
	auto pll = Blib::PLL<q15_t>{PWM_FRQ,100.f};

	auto tim_1khz  = Clib::InterruptionTimerHard{&htim17};

	sMDUReg::ControlMode control_mode = sMDUReg::ControlMode::CURRENT;

	float target_spd = 0.f;
	float target_pos = 0.f;
	auto spd_pid = Clib::Math::PIDBuilder(1000.f).build();
	auto pos_pid = Clib::Math::PIDBuilder(1000.f).build();

	///////////////////////////////////////////////////////////////////////////
	//インターフェイス類
	///////////////////////////////////////////////////////////////////////////
	//配置newするためのメモリープール
	namespace TmpMemoryPool{
		uint8_t can_tx_buff[sizeof(Clib::RingBuffer<Clib::CanFrame,5>)];
		uint8_t can_rx_buff[sizeof(Clib::RingBuffer<Clib::CanFrame,5>)];
		uint8_t robomas_fb_tx_buff[sizeof(Clib::RingBuffer<Clib::CanFrame,2>)];
		uint8_t robomas_fb_rx_buff[sizeof(Clib::RingBuffer<Clib::CanFrame,2>)];
		uint8_t usb_rx_buff[sizeof(Clib::RingBuffer<Clib::StrPack,5>)];
		uint8_t usb_tx_buff[sizeof(Clib::RingBuffer<Clib::StrPack,5>)];
	}

	auto can = Clib::FdCanComm{
		&hfdcan1,
		std::unique_ptr<Clib::RingBuffer<Clib::CanFrame,5>>(
				new(TmpMemoryPool::can_tx_buff) Clib::RingBuffer<Clib::CanFrame,5>{}),
		std::unique_ptr<Clib::RingBuffer<Clib::CanFrame,5>>(
				new(TmpMemoryPool::can_rx_buff) Clib::RingBuffer<Clib::CanFrame,5>{}),
		0
	};
	auto robomas_fb_can = Clib::FdCanComm{//ロボマスフォーマットでの通信に使う受信専用インスタンス
		&hfdcan1,
		std::unique_ptr<Clib::RingBuffer<Clib::CanFrame,2>>(
				new(TmpMemoryPool::robomas_fb_tx_buff) Clib::RingBuffer<Clib::CanFrame,2>{}),
		std::unique_ptr<Clib::RingBuffer<Clib::CanFrame,2>>(
				new(TmpMemoryPool::robomas_fb_rx_buff) Clib::RingBuffer<Clib::CanFrame,2>{}),
		1
	};

	auto usb_cdc = Clib::UsbCdcComm{&hUsbDeviceFS,
		std::unique_ptr<Clib::RingBuffer<Clib::StrPack,5>>(
				new(TmpMemoryPool::usb_rx_buff) Clib::RingBuffer<Clib::StrPack,5>{}),
		std::unique_ptr<Clib::RingBuffer<Clib::StrPack,5>>(
				new(TmpMemoryPool::usb_tx_buff) Clib::RingBuffer<Clib::StrPack,5>{}),
	};

	bool apply_params_rq = false;
	bool remote_calibration_request = false;

	namespace IO{
		auto pwm_uh = Clib::GPIO{PWM_UH_GPIO_Port,PWM_UH_Pin};
		auto pwm_ul = Clib::GPIO{PWM_UL_GPIO_Port,PWM_UL_Pin};
		auto pwm_vh = Clib::GPIO{PWM_VH_GPIO_Port,PWM_VH_Pin};
		auto pwm_vl = Clib::GPIO{PWM_VL_GPIO_Port,PWM_VL_Pin};
		auto pwm_wh = Clib::GPIO{PWM_WH_GPIO_Port,PWM_WH_Pin};
		auto pwm_wl = Clib::GPIO{PWM_WL_GPIO_Port,PWM_WL_Pin};

		auto uart_rx = Clib::GPIO{UART_RX_GPIO_Port,UART_RX_Pin};
		auto uart_tx = Clib::GPIO{UART_TX_GPIO_Port,UART_TX_Pin};
		auto spi_sck = Clib::GPIO{SPI_SCK_GPIO_Port,SPI_SCK_Pin};
		auto spi_miso = Clib::GPIO{SPI_MISO_GPIO_Port,SPI_MISO_Pin};
		auto spi_mosi = Clib::GPIO{SPI_MOSI_GPIO_Port,SPI_MOSI_Pin};
		auto i2c_scl = Clib::GPIO{I2C_SCL_GPIO_Port,I2C_SCL_Pin};
		auto i2c_sda = Clib::GPIO{I2C_SDA_GPIO_Port,I2C_SDA_Pin};

		auto hall_a = Clib::GPIO{HALL_A_ADC1IN4_GPIO_Port,HALL_A_ADC1IN4_Pin};
		auto hall_b = Clib::GPIO{HALL_B_ADC2IN17_GPIO_Port,HALL_B_ADC2IN17_Pin};

		auto sw = Clib::GPIO{SW_GPIO_Port,SW_Pin};

		auto led_r = Clib::GPIO{LED_R_GPIO_Port,LED_R_Pin};
		auto led_g = Clib::GPIO{LED_G_GPIO_Port,LED_G_Pin};
		auto led_b = Clib::GPIO{LED_B_GPIO_Port,LED_B_Pin};
		auto board_temp = Clib::GPIO{TEMP_ADC2IN5_GPIO_Port,TEMP_ADC2IN5_Pin};
	}

	auto flash = Clib::G4FlashRW(FLASH_BANK_1,63,0x801F800);

	//↓動的メモリ確保せざるをえないやつら Task::dynamic_mem_allocで再度初期化
	auto id_map = CommonLib::LinerIDMap<0xFF>();
	auto led_r_sequencer = Clib::Sequencer{};
	auto led_g_sequencer = Clib::Sequencer{};
	auto led_b_sequencer = Clib::Sequencer{};
	auto button = Clib::Button{nullptr,1000.0f};

	CommonLib::Note error_indicate[] = {
		{0.2f,100},//0
		{0.0f,300},
		{0.2f,100},//1
		{0.0f,300},
		{0.2f,100},//2
		{0.0f,300},
		{0.2f,100},//3
		{0.0f,300},
		{0.2f,100},//4
		{0.0f,300},
		{0.2f,100},//5
		{0.0f,300},
		{0.2f,100},//6
		{0.0f,300},
		{0.2f,100},//7
		{0.0f,1000},
		CommonLib::Note::end_of_sequence<float>(0.0f)
	};
}

namespace be = BoardElement;

namespace Debug{//制御には不要だが宣言してある変数

}

extern "C" void cppmain(void){
	Task::dynamic_mem_alloc();

	HAL_Delay(100);

	//アナログ系初期化
	HAL_ADCEx_Calibration_Start(&hadc1,ADC_SINGLE_ENDED);
	HAL_ADCEx_Calibration_Start(&hadc2,ADC_SINGLE_ENDED);

	HAL_OPAMP_Start(&hopamp2);
	HAL_OPAMP_Start(&hopamp3);

	//↓保存パラメータリセット用
//	be::b_params = Blib::default_board_param;
//	be::flash.write(reinterpret_cast<uint8_t*>(&be::b_params), sizeof(Blib::BoardParams));
//	HAL_Delay(100);
	//↑保存パラメータリセット用

	be::flash.read(reinterpret_cast<uint8_t*>(&be::b_params), sizeof(Blib::BoardParams));

	if(be::b_params.f_state == Clib::FlashState::RESET){
		be::b_params = Blib::default_board_param;
		be::led_r_sequencer.play(Blib::LEDPattern::error);
		be::led_g_sequencer.play(Blib::LEDPattern::error);
		be::led_b_sequencer.play(Blib::LEDPattern::error);
	}else{
		be::led_r_sequencer.play(Blib::LEDPattern::running);
		be::led_g_sequencer.play(Blib::LEDPattern::running);
		be::led_b_sequencer.play(Blib::LEDPattern::running);
	}

	be::error_flags = 0xFF;
	be::op_mode = sMDUReg::OpMode::EMS;

	//エンコーダ設定
	Task::gpio_init(be::b_params.enc_type);
	be::e_angle_enc.set_gear_ratio(static_cast<float>(abs(be::b_params.m_params.pole_n)));

	//モータ制御設定
	be::motor.start();
	be::motor.update_motor_param(
			be::b_params.m_params.R,
			be::b_params.m_params.L,
			be::b_params.m_params.phi,
			be::b_params.m_params.i_control_natural_freq,
			be::b_params.m_params.damping_ratio);

	//速度・位置制御設定
	be::spd_pid.set_p_gain(be::b_params.spd_kp);
	be::spd_pid.set_i_gain(be::b_params.spd_ki);
	be::spd_pid.set_d_gain(be::b_params.spd_kd);
	be::spd_pid.set_anti_windup_gain(fabs(be::b_params.spd_kp)>0.0001f ? 1.f/be::b_params.spd_kp : 0.f);
	be::spd_pid.set_limit(be::b_params.crr_limit);

	be::pos_pid.set_p_gain(be::b_params.pos_kp);
	be::pos_pid.set_i_gain(be::b_params.pos_ki);
	be::pos_pid.set_d_gain(be::b_params.pos_kd);
	be::pos_pid.set_anti_windup_gain(fabs(be::b_params.pos_kp)>0.0001f ? 1.f/be::b_params.pos_kp : 0.f);
	be::pos_pid.set_limit(be::b_params.spd_limit);

	be::tim_1khz.start_timer(0.001f);

	//ADC設定
	HAL_ADC_Start(&hadc1);
	HAL_ADCEx_InjectedStart_IT(&hadc1);
	HAL_ADC_Start(&hadc2);
	HAL_ADCEx_InjectedStart_IT(&hadc2);
	HAL_Delay(10);
	be::vbus_voltage_lpf.overwrite(ADCDet::to_voltage(ADCDet::BATT_V));
	be::motor_temp_lpf.overwrite(ADCDet::MOTOR_TEMP);
	be::board_temp_lpf.overwrite(ADCDet::BOARD_TEMP);

	//CAN初期化
	be::can.set_filter_mask_mode(
			0,
			(static_cast<size_t>(Clib::Protocol::DataType::sMDU_ID)<<20) | (be::b_params.id<<16),
			0x00FF'0000,
			Clib::CanFilterMode::ONLY_EXT);
	be::can.set_filter_mask_mode(
			1,
			(static_cast<size_t>(Clib::Protocol::DataType::COMMON_ID)<<20) | (be::b_params.id<<16),
			0x00FF'0000,
			Clib::CanFilterMode::ONLY_EXT);
	be::can.set_filter_mask_mode(
			2,
			(static_cast<size_t>(Clib::Protocol::DataType::COMMON_ID_ENFORCE)<<20),
			0x00F0'0000,
			Clib::CanFilterMode::ONLY_EXT);
	be::robomas_fb_can.set_filter_mask_mode(
			0,
			0x200,
			0xFFFF,
			Clib::CanFilterMode::ONLY_STD);
	be::can.start();


	be::IO::led_r.write(true);
	be::IO::led_g.write(true);
	be::IO::led_b.write(true);

	be::error_flags = 0;
	if(be::b_params.f_state == Clib::FlashState::RESET){
		be::op_mode = sMDUReg::OpMode::EMS;
		be::error_flags |= 1u<<sMDUReg::ErrorFlagBitPos::PARAM_UPDATE;
	}else{
		be::op_mode = sMDUReg::OpMode::NORMAL;
		Task::pwm_io_enable();
	}

	be::led_r_sequencer.play(Blib::LEDPattern::ok);
	be::led_g_sequencer.play(Blib::LEDPattern::ok);
	be::led_b_sequencer.play(Blib::LEDPattern::ok);

	bool setting_mode = false;
	int new_id = -1;
	while(1){
		//LED処理
		switch(be::op_mode){
		case sMDUReg::OpMode::NORMAL:
			if(be::b_params.feedback_format == MReg::RobomasMD::C610){
				be::led_g_sequencer.play(Blib::LEDPattern::indicate_id[be::b_params.id],false);
			}else{
				be::led_b_sequencer.play(Blib::LEDPattern::indicate_id[be::b_params.id],false);
			}
			break;
		case sMDUReg::OpMode::EMS:
			if(not setting_mode){
				be::led_r_sequencer.play(be::error_indicate,false);
			}
			break;
		case sMDUReg::OpMode::CALIBRATION:
			be::led_b_sequencer.play(Blib::LEDPattern::calibrating,false);
			be::led_g_sequencer.play(Blib::LEDPattern::calibrating,false);
			break;
		default:
			break;
			//nop
		}

		//ボタン処理
		if((be::button.is_long_pressed(true) || be::remote_calibration_request) && not setting_mode){
			//キャリブレーション開始
			be::button.reset();
			be::remote_calibration_request = false;
			be::circuit_calibrator.set_caliblation_param(be::b_params.enc_type == sMDUReg::EncType::Robomas,0.5f,5.f,be::b_params.m_params.pole_n);

			be::op_mode = sMDUReg::OpMode::CALIBRATION;
			be::error_flags = 0b0;
			be::motor.start();
			Task::pwm_io_enable();
			be::led_b_sequencer.play(Blib::LEDPattern::calibrating,true);
			be::led_g_sequencer.play(Blib::LEDPattern::calibrating,true);
		}

		if(setting_mode){
			//ボタンを押して2秒以上放置されたらリセット
			if((be::button.get_leaving_time() > 2.0f) || (new_id >= 3)){
				be::motor.stop();
				if((3 >= new_id) && (new_id >= 0)){
					be::b_params.id = new_id;
				}else if(new_id == -1){
					be::target_i = Clib::Math::DQ{.d = 0.0f, .q =0.0f};
					HAL_NVIC_SystemReset();
				}

				setting_mode = false;
				new_id = -1;

				be::apply_params_rq = true;//ID保存
			}else{
				if(be::button.is_pressed(true)){//ID設定
					new_id ++;
					be::button.reset();
					be::led_g_sequencer.play(Blib::LEDPattern::ok,true);
				}else if(be::button.is_long_pressed(true)){//保存パラメータリセット
					be::button.reset();
					be::led_b_sequencer.play(Blib::LEDPattern::ok,true);

					be::op_mode = sMDUReg::OpMode::EMS;
					be::b_params = Blib::default_board_param;
					be::flash.write(reinterpret_cast<uint8_t*>(&be::b_params), sizeof(Blib::BoardParams));
					HAL_Delay(500);
					HAL_NVIC_SystemReset();
				}
				be::led_r_sequencer.play(Blib::LEDPattern::ok,false);
			}
		}else{
			if(be::button.is_pressed(true)){
				be::button.reset();
				setting_mode = true;
				be::led_r_sequencer.play(Blib::LEDPattern::ok,true);
			}
		}

		//保存データ更新など
		if(be::apply_params_rq){
			be::motor.stop();
			Task::pwm_io_disable();

			be::b_params.spd_kp = be::spd_pid.get_p_gain();
			be::b_params.spd_ki = be::spd_pid.get_i_gain();
			be::b_params.spd_kd = be::spd_pid.get_d_gain();
			be::b_params.crr_limit = be::spd_pid.get_limit().second;
			be::b_params.pos_kp = be::pos_pid.get_p_gain();
			be::b_params.pos_ki = be::pos_pid.get_i_gain();
			be::b_params.pos_kd = be::pos_pid.get_d_gain();
			be::b_params.spd_limit = be::pos_pid.get_limit().second;

			//書き込み
			be::b_params.f_state = Clib::FlashState::WRITED;
			be::flash.write(reinterpret_cast<uint8_t*>(&be::b_params), sizeof(Blib::BoardParams));

			//再起動
			Task::gpio_init(be::b_params.enc_type);
			be::amt21x.reset_state();
			be::as5048.reset_state();
			be::as5600.reset_state();
			be::e_angle_enc.set_gear_ratio(static_cast<float>(abs(be::b_params.m_params.pole_n)));
			be::motor.update_motor_param(
						be::b_params.m_params.R,
						be::b_params.m_params.L,
						be::b_params.m_params.phi,
						be::b_params.m_params.i_control_natural_freq,
						be::b_params.m_params.damping_ratio);

			be::can.stop();
			be::can.set_filter_mask_mode(
					0,
					(static_cast<size_t>(Clib::Protocol::DataType::sMDU_ID)<<20) | (be::b_params.id<<16),
					0x00FF'0000,
					Clib::CanFilterMode::ONLY_EXT);
			be::can.set_filter_mask_mode(
					1,
					(static_cast<size_t>(Clib::Protocol::DataType::COMMON_ID)<<20) | (be::b_params.id<<16),
					0x00FF'0000,
					Clib::CanFilterMode::ONLY_EXT);
			be::can.set_filter_mask_mode(
					2,
					(static_cast<size_t>(Clib::Protocol::DataType::COMMON_ID_ENFORCE)<<20),
					0x00F0'0000,
					Clib::CanFilterMode::ONLY_EXT);
			be::can.start();

			be::motor.start();
			Task::pwm_io_enable();
			be::apply_params_rq = false;
			be::error_flags = 0;
			be::op_mode = sMDUReg::OpMode::NORMAL;
			be::target_i = Clib::Math::DQ{.d = 0.0f, .q =0.0f};
		}

		Task::can_communication();
		Task::usb_communication();

//		uint8_t rx[2] = {0};
//		HAL_I2C_Mem_Read(be::as5600.get_handler(), 0x36<<1, 0x0C, I2C_MEMADD_SIZE_8BIT, rx, 2,200);
//		printf("%d,%d\r\n",ADCDet::regular_conv_buff[0],ADCDet::regular_conv_buff[1]);
//		printf("%x,%x\r\n",rx[0],rx[1]);
//		printf("%d,%d,%d\r\n",ADCDet::ENC_A,ADCDet::ENC_B,be::as5048.get_angle());
//		HAL_Delay(10);
	}
}

///////////////////////////////////////////////////////////////////////////
//PWM同期ループ
///////////////////////////////////////////////////////////////////////////
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc){
	//-OfastのSVM,デッドタイム補正アリで8.6usぐらいらしい
//	be::IO::led_r(false);
	static int adc_flag = 0;
	if(hadc == &hadc1){
		adc_flag |= 0b01;
	}else if(hadc == &hadc2){
		adc_flag |= 0b10;
	}

	if(adc_flag == 0b11){
		q15_t qsin = (static_cast<q15_t>(ADCDet::ENC_A) - be::b_params.m_params.sinenc_bias)*be::b_params.m_params.sinenc_gain;
		q15_t qcos = -(static_cast<q15_t>(ADCDet::ENC_B) - be::b_params.m_params.cosenc_bias)*be::b_params.m_params.cosenc_gain;
		be::cordic.start_atan2(qcos,qsin,8);

		be::uvw_i.v = ADCDet::to_current(ADCDet::IV,be::b_params.m_params.iv_bias);
		be::uvw_i.w = ADCDet::to_current(ADCDet::IW,be::b_params.m_params.iw_bias);
		be::uvw_i.u = -be::uvw_i.v - be::uvw_i.w;
		be::ab_i = be::uvw_i.to_ab();

		be::vbus_voltage_lpf(ADCDet::to_voltage(ADCDet::BATT_V));

		//Cordicの読み込みとuvw->dq変換
		if(be::b_params.enc_type == sMDUReg::EncType::Robomas){
			while(not be::cordic.handler.is_available()){}
			be::raw_angle = be::cordic.handler.read_ans();
			be::e_angle = be::raw_angle - be::b_params.m_params.e_angle_bias;
		}else{
			be::pll.update();
			be::raw_angle = be::pll.get_pos();
			be::e_angle = be::raw_angle*be::b_params.m_params.pole_n - be::b_params.m_params.e_angle_bias;
		}
		be::e_angle_enc.update(be::e_angle);

		be::dq_i = be::ab_i.to_dq(be::table.sin_cos(be::e_angle));


		//電流異検出はなんぼあってもいいですからね
		//電流センサのバイアスが大幅にずれることはないだろうの想定
		if((fabs(be::uvw_i.u) > MAX_CURR) || (fabs(be::uvw_i.v) > MAX_CURR) || (fabs(be::uvw_i.w) > MAX_CURR)){
			be::op_mode = sMDUReg::OpMode::EMS;
			be::error_flags |= 1u<<sMDUReg::ErrorFlagBitPos::CURRENT;

		}

		//電流制御
		switch(be::op_mode){
		case sMDUReg::OpMode::NORMAL:
			if(be::b_params.is_dc_motor){
				float duty = std::clamp(be::target_i.q,-0.9f,0.9f);
				be::motor.drive({
							.u = -duty,
							.v = duty,
							.w = 0,
						});
			}else{
				be::motor.current_pi(
						be::vbus_voltage_lpf.get()+0.01f,//電圧に0.01足しているのは0除算回避
						be::dq_i,
						be::target_i,
						be::e_angle,
						be::e_angle_enc.get_speed());
			}
			break;
		case sMDUReg::OpMode::CALIBRATION:
			be::circuit_calibrator.stop_watch.update(ADCDet::IV,ADCDet::IW);
			break;
		case sMDUReg::OpMode::EMS:
//			be::motor.drive({0,0,0});
			be::motor.soft_stop();
			Task::pwm_io_disable();
			break;
		default:
			//nop
			break;
		}

		adc_flag = 0;
	}
//	be::IO::led_r(true);
}

///////////////////////////////////////////////////////////////////////////
//1kHzループ
///////////////////////////////////////////////////////////////////////////
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim == be::tim_1khz.get_handler()){
		Task::error_check();

		//モータ制御関係
		switch(be::op_mode){
		case sMDUReg::OpMode::NORMAL:{
			switch(be::control_mode){
			case sMDUReg::ControlMode::POSITION:
				be::target_spd = be::pos_pid(be::target_pos,be::e_angle_enc.get_rad());
			case sMDUReg::ControlMode::SPEED:
				be::target_i.q = be::spd_pid(be::target_spd,be::e_angle_enc.get_rad_speed());
			case sMDUReg::ControlMode::CURRENT:
				break;
			}
		}
		break;
		case sMDUReg::OpMode::CALIBRATION:{
			auto [uvw,state] = be::circuit_calibrator.caliblation(
					be::raw_angle,
					ADCDet::IV,
					ADCDet::IW,
					be::vbus_voltage_lpf.get(),
					ADCDet::ENC_A,
					ADCDet::ENC_B);
			be::motor.drive(uvw);

			if(state == Blib::MotorCalibrator::State::RESET){
				be::op_mode = sMDUReg::OpMode::EMS;
				be::led_r_sequencer.play(Blib::LEDPattern::ok,true);
				be::b_params.m_params = be::circuit_calibrator.get_motor_params();
				be::apply_params_rq = true;
				if(be::b_params.enc_type == sMDUReg::EncType::Robomas){
					if(be::motor_temp_lpf.get() > 3000){ //M2006
						be::b_params.feedback_format = MReg::RobomasMD::C610;
						be::b_params.m_params.pole_n = 7;
						be::b_params.m_params.R = 461.0e-3;
						be::b_params.m_params.L = 64.22e-6;
						be::b_params.m_params.phi = 2.0f/(3.f*7.f*36.f)*0.18f;
					}else{
						be::b_params.feedback_format = MReg::RobomasMD::C620;
						be::b_params.m_params.pole_n = 7;
						be::b_params.m_params.R = 194.0e-3;
						be::b_params.m_params.L = 97.0e-6;
						be::b_params.m_params.phi = 2.0f/(3.f*7.f*3591.f/187.f)*0.3f;
					}
				}else{
					//仮の値
					be::b_params.m_params.phi = 0;
				}
			}
		}
		break;
		case sMDUReg::OpMode::EMS:{
//			be::circuit_calibrator.reset_calibration();
		}
		break;
		default:
			//nop
			break;
		}

		//各種更新処理
		be::led_r_sequencer.update();
		be::led_g_sequencer.update();
		be::led_b_sequencer.update();
		be::button.update();
		switch(be::b_params.enc_type){
		case sMDUReg::EncType::AS5048:
			be::as5048.read_start();
			break;
		case sMDUReg::EncType::AS5600:
			be::as5600.read_start();
			break;
		case sMDUReg::EncType::AMT21x:
			be::amt21x.read_start();
			break;
		default:
			break;
		}

		auto tx_frame = Task::robomas_feedback();
		if((be::op_mode == sMDUReg::OpMode::NORMAL) && be::b_params.robomas_feedback_enable){
			be::can.tx(tx_frame);
		}
		static size_t usb_fb_cnt = 0;
		if(be::b_params.usb_feedback_period != 0){
			if(usb_fb_cnt > be::b_params.usb_feedback_period){
				be::usb_cdc.tx(Clib::SLCAN::can_to_slcan_packed(tx_frame));
				usb_fb_cnt = 0;
			}
			usb_fb_cnt ++;
		}

		//モーター温度，基板温度のADC
		be::motor_temp_lpf(ADCDet::MOTOR_TEMP);
		be::board_temp_lpf(ADCDet::BOARD_TEMP);
		HAL_ADC_Start_DMA(&hadc2, (uint32_t *)ADCDet::regular_conv_buff, 2);
	}
}

///////////////////////////////////////////////////////////////////////////
//各種保護処理
///////////////////////////////////////////////////////////////////////////
void Task::error_check(void){
	//op_modeはerror_flagの状態に追従する形で遷移する(op_modeはread_only，ユーザーはerror_flagsを編集することでしかmodeをいじれない)
	//ただし，EMS状態に移行する場合については素早さが要求されるため，直接書き換える
	//基本的には，この関数の一番最後のところでerror_flagの状態を観測してNormalとEMSを遷移させる

	//電流異常
	//電流センサのバイアスが大幅にずれることはないだろうの想定
	if((fabs(be::uvw_i.u) > MAX_CURR) || (fabs(be::uvw_i.v) > MAX_CURR) || (fabs(be::uvw_i.w) > MAX_CURR)){
		be::op_mode = sMDUReg::OpMode::EMS;
		be::error_flags |= 1u<<sMDUReg::ErrorFlagBitPos::CURRENT;
	}

	//エンコーダ異常
	switch(be::b_params.enc_type){
	case sMDUReg::EncType::Robomas:
		if((ADCDet::ENC_A<150) || (ADCDet::ENC_B<150)){
			be::op_mode = sMDUReg::OpMode::EMS;
			be::error_flags |= 1u<<sMDUReg::ErrorFlagBitPos::ENCODER;
		}
		break;
	case sMDUReg::EncType::AS5048:
		//通信して無いのにMISOがLだったらスレーブデバイスが未接続
//		if(be::as5048.is_dead() || (be::as5048.is_ready() && ADCDet::ENC_A < 500)){
		//と思ったけど断線検知は無理そうだった無理そうだった
		if(be::as5048.is_dead()){
			be::op_mode = sMDUReg::OpMode::EMS;
			be::error_flags |= 1u<<sMDUReg::ErrorFlagBitPos::ENCODER;
		}
		break;
	case sMDUReg::EncType::AS5600:
		if(be::as5600.is_dead()){
			be::op_mode = sMDUReg::OpMode::EMS;
			be::error_flags |= 1u<<sMDUReg::ErrorFlagBitPos::ENCODER;
		}
		break;
	case sMDUReg::EncType::AMT21x:
		if(be::amt21x.is_dead()){
			be::op_mode = sMDUReg::OpMode::EMS;
			be::error_flags |= 1u<<sMDUReg::ErrorFlagBitPos::ENCODER;
		}
		break;
	}

	//接続モータがあってるか
	if(be::op_mode != sMDUReg::OpMode::CALIBRATION){
		if(be::b_params.enc_type == sMDUReg::EncType::Robomas){
			if((be::b_params.feedback_format == MReg::RobomasMD::C610) && (be::motor_temp_lpf.get() < 2800)){
				be::op_mode = sMDUReg::OpMode::EMS;
				be::error_flags |= 1u<<sMDUReg::ErrorFlagBitPos::MOTOR_TYPE;
			}else if((be::b_params.feedback_format == MReg::RobomasMD::C620) && (be::motor_temp_lpf.get() > 2800)){
				be::op_mode = sMDUReg::OpMode::EMS;
				be::error_flags |= 1u<<sMDUReg::ErrorFlagBitPos::MOTOR_TYPE;
			}
		}
	}

	//基板温度確認
	if(be::board_temp_lpf.get() < 1.f){
		be::op_mode = sMDUReg::OpMode::EMS;
		be::error_flags |= 1u<<sMDUReg::ErrorFlagBitPos::BOARD_TEMP;
	}


	//電源電圧異常
	if(be::vbus_voltage_lpf.get() < 9.f){
		be::op_mode = sMDUReg::OpMode::EMS;
		be::error_flags |= 1u<<sMDUReg::ErrorFlagBitPos::VOLTAGE;
	}else{
		//電源電圧異常以外のエラーが無ければエラーフラグを解除し起動
		if((be::error_flags & ~(1u<<sMDUReg::ErrorFlagBitPos::VOLTAGE)) == 0){
			be::error_flags &= ~(1u<<sMDUReg::ErrorFlagBitPos::VOLTAGE);
		}
	}

	//error_flagsを元にLEDパターンを変更
	for(int i = 0; i < 8; i++){
		if((be::error_flags >> i) & 0b1){
			be::error_indicate[2*i].interval =  500;
		}else{
			be::error_indicate[2*i].interval =  100;
		}
	}

	if(be::op_mode != sMDUReg::OpMode::CALIBRATION){
		if(be::error_flags == 0){//非常停止解除
			be::op_mode = sMDUReg::OpMode::NORMAL;
		}else{
			be::op_mode = sMDUReg::OpMode::EMS;
		}
	}

}

///////////////////////////////////////////////////////////////////////////
//動的メモリ確保してる可能性のあるやつらを隔離
///////////////////////////////////////////////////////////////////////////
void Task::dynamic_mem_alloc(void){
	be::id_map
		.add(sMDUReg::STATE,          CommonLib::DataAccessor::generate<uint8_t>(
				[]()->uint8_t{return static_cast<uint8_t>(be::op_mode);}))
		.add(sMDUReg::ERROR_FLAG,     CommonLib::DataAccessor::generate<uint8_t>(&be::error_flags))
		.add(sMDUReg::MOTOR_TYPE,     CommonLib::DataAccessor::generate<uint8_t>(
				[](uint8_t t){
					be::b_params.feedback_format = static_cast<MReg::RobomasMD>(t & 0b1);
					be::b_params.is_dc_motor = (t&0b100) != 0;
					be::b_params.enc_type = static_cast<sMDUReg::EncType>((t>>4) & 0b1111);
					be::op_mode = sMDUReg::OpMode::EMS;
					be::error_flags |= 1u<<sMDUReg::ErrorFlagBitPos::PARAM_UPDATE;
					Task::gpio_init(be::b_params.enc_type);},
				[]()->uint8_t{
						return static_cast<uint8_t>(
								static_cast<int>(be::b_params.feedback_format)
								| (be::b_params.is_dc_motor ? 0b100 : 0b000)
								| (static_cast<int>(be::b_params.enc_type)<<4));
					}))
		.add(sMDUReg::R,              CommonLib::DataAccessor::generate<float>(
				[](float r){
					be::b_params.m_params.R = r;
					be::error_flags |= 1u<<sMDUReg::ErrorFlagBitPos::PARAM_UPDATE;
					be::op_mode = sMDUReg::OpMode::EMS;},
				[]()->float{return be::b_params.m_params.R;}))
		.add(sMDUReg::L,              CommonLib::DataAccessor::generate<float>(
				[](float l){
					be::b_params.m_params.L = l;
					be::op_mode = sMDUReg::OpMode::EMS;
					be::error_flags |= 1u<<sMDUReg::ErrorFlagBitPos::PARAM_UPDATE;},
				[]()->float{return be::b_params.m_params.L;}))
		.add(sMDUReg::PHI,            CommonLib::DataAccessor::generate<float>(
				[](float phi){
					be::b_params.m_params.phi = phi;
					be::op_mode = sMDUReg::OpMode::EMS;
					be::error_flags |= 1u<<sMDUReg::ErrorFlagBitPos::PARAM_UPDATE;},
				[]()->float{return be::b_params.m_params.phi;}))
		.add(sMDUReg::POLE_N,         CommonLib::DataAccessor::generate<int8_t>(
				[](int8_t n){
					be::b_params.m_params.pole_n = n;
					be::op_mode = sMDUReg::OpMode::EMS;
					be::error_flags |= 1u<<sMDUReg::ErrorFlagBitPos::PARAM_UPDATE;},
				[]()->int8_t{return be::b_params.m_params.pole_n;}))
		.add(sMDUReg::CTRL_NATURAL_FREQ,CommonLib::DataAccessor::generate<float>(
				[](float f){
					be::b_params.m_params.i_control_natural_freq = f;
					be::op_mode = sMDUReg::OpMode::EMS;
					be::error_flags |= 1u<<sMDUReg::ErrorFlagBitPos::PARAM_UPDATE;},
				[]()->float{return be::b_params.m_params.i_control_natural_freq;}))
		.add(sMDUReg::CTRL_DAMPING_RATIO,CommonLib::DataAccessor::generate<float>(
				[](float zeta){
					be::b_params.m_params.damping_ratio = zeta;
					be::op_mode = sMDUReg::OpMode::EMS;
					be::error_flags |= 1u<<sMDUReg::ErrorFlagBitPos::PARAM_UPDATE;},
				[]()->float{return be::b_params.m_params.damping_ratio;}))

		.add(sMDUReg::CAL_RQ,         CommonLib::DataAccessor::generate<bool>(&be::remote_calibration_request))
		.add(sMDUReg::CONTROL_MODE,   CommonLib::DataAccessor::generate<uint8_t>(
				[](uint8_t m){
					be::control_mode = static_cast<sMDUReg::ControlMode>(m);
					be::target_i.q = 0.f;
					be::target_spd = 0.f;
					be::target_pos = be::e_angle_enc.get_rad();},
				[]()->uint8_t{return static_cast<uint8_t>(be::control_mode);}))
		.add(sMDUReg::ROBOMAS_FB_EN,  CommonLib::DataAccessor::generate<uint8_t>(&be::b_params.robomas_feedback_enable))
		.add(sMDUReg::PARAMS_APPLY,   CommonLib::DataAccessor::generate<bool>(&be::apply_params_rq))


		.add(sMDUReg::CURRENT,        CommonLib::DataAccessor::generate<float>(&be::dq_i.q))
		.add(sMDUReg::CURRENT_TARGET, CommonLib::DataAccessor::generate<float>(&be::target_i.q))

		.add(sMDUReg::SPD,            CommonLib::DataAccessor::generate<float>(
				[]()->float{return be::e_angle_enc.get_rad_speed();}))
		.add(sMDUReg::SPD_TARGET,     CommonLib::DataAccessor::generate<float>(&be::target_spd))
		.add(sMDUReg::CURRENT_LIM,    CommonLib::DataAccessor::generate<float>(
				[](float l){be::spd_pid.set_limit(l);},
				[]()->float{auto [ll,lh] = be::spd_pid.get_limit(); return lh;}))
		.add(sMDUReg::SPD_GAIN_P,     CommonLib::DataAccessor::generate<float>(
				[](float p){be::spd_pid.set_p_gain(p);},
				[]()->float{return  be::spd_pid.get_p_gain();}))
		.add(sMDUReg::SPD_GAIN_I,     CommonLib::DataAccessor::generate<float>(
				[](float i){be::spd_pid.set_i_gain(i);},
				[]()->float{return be::spd_pid.get_i_gain();}))
		.add(sMDUReg::SPD_GAIN_D,     CommonLib::DataAccessor::generate<float>(
				[](float d){be::spd_pid.set_d_gain(d);},
				[]()->float{return be::spd_pid.get_d_gain();}))
		.add(sMDUReg::SPD_GAIN_ANTIWINDUP,     CommonLib::DataAccessor::generate<float>(
						[](float b){be::spd_pid.set_anti_windup_gain(b);},
						[]()->float{return be::spd_pid.get_anti_windup_gain();}))

		.add(sMDUReg::POS,            CommonLib::DataAccessor::generate<float>(
				[]()->float{return be::e_angle_enc.get_rad();}))
		.add(sMDUReg::POS_TARGET,     CommonLib::DataAccessor::generate<float>(&be::target_pos))
		.add(sMDUReg::SPD_LIM,        CommonLib::DataAccessor::generate<float>(
				[](float l){be::pos_pid.set_limit(l);},
				[]()->float{auto [ll,lh] = be::pos_pid.get_limit(); return lh;}))
		.add(sMDUReg::POS_GAIN_P,     CommonLib::DataAccessor::generate<float>(
				[](float p){be::pos_pid.set_p_gain(p);},
				[]()->float{return  be::pos_pid.get_p_gain();}))
		.add(sMDUReg::POS_GAIN_I,     CommonLib::DataAccessor::generate<float>(
				[](float i){be::pos_pid.set_i_gain(i);},
				[]()->float{return be::pos_pid.get_i_gain();}))
		.add(sMDUReg::POS_GAIN_D,     CommonLib::DataAccessor::generate<float>(
				[](float d){be::pos_pid.set_d_gain(d);},
				[]()->float{return be::pos_pid.get_d_gain();}))
		.add(sMDUReg::POS_GAIN_ANTIWINDUP,     CommonLib::DataAccessor::generate<float>(
				[](float b){be::pos_pid.set_anti_windup_gain(b);},
				[]()->float{return be::pos_pid.get_anti_windup_gain();}))

		.add(sMDUReg::BOARD_TEMP,   CommonLib::DataAccessor::generate<uint16_t>(const_cast<uint16_t *>(&ADCDet::BOARD_TEMP)))

		.add(sMDUReg::USB_FB_PERIOD,  CommonLib::DataAccessor::generate<uint32_t>(&be::b_params.usb_feedback_period));

	be::led_r_sequencer.set_output_function(std::function<void(float)>([](float v){be::IO::led_r(v<0.01);}));
	be::led_g_sequencer.set_output_function(std::function<void(float)>([](float v){be::IO::led_g(v<0.01);}));
	be::led_b_sequencer.set_output_function(std::function<void(float)>([](float v){be::IO::led_b(v<0.01);}));

	be::button.set_getter_function([](void)->bool{return !be::IO::sw.read();});
}

///////////////////////////////////////////////////////////////////////////
//エンコーダ用IO設定
///////////////////////////////////////////////////////////////////////////
void Task::gpio_init(sMDUReg::EncType e_type,bool estimate_enc_type){
	be::IO::board_temp.set_io_pull_mode(Clib::GPIO::Pull::UP);

	if(estimate_enc_type){//エンコーダタイプを自動判別したかった（内臓プルアップ抵抗だと雑魚すぎて多分無理）
		be::IO::spi_miso.set_io_mode(Clib::GPIO::Mode::ANALOG);
		be::IO::spi_mosi.set_io_mode(Clib::GPIO::Mode::ANALOG);
		be::IO::spi_sck.set_io_mode(Clib::GPIO::Mode::ANALOG);
		be::as5048.ss.set_io_mode(Clib::GPIO::Mode::ANALOG);

		be::IO::i2c_scl.set_io_mode(Clib::GPIO::Mode::ANALOG);
		be::IO::i2c_sda.set_io_mode(Clib::GPIO::Mode::ANALOG);

		be::IO::uart_rx.set_io_mode(Clib::GPIO::Mode::ANALOG);
		be::IO::uart_tx.set_io_mode(Clib::GPIO::Mode::ANALOG);

		be::IO::hall_a.set_io_pull_mode(Clib::GPIO::Pull::UP);
		be::IO::hall_b.set_io_pull_mode(Clib::GPIO::Pull::UP);
		HAL_Delay(50);
		uint16_t a_state_pullup = ADCDet::ENC_A;
		uint16_t b_state_pullup = ADCDet::ENC_B;

		be::IO::hall_a.set_io_pull_mode(Clib::GPIO::Pull::DOWN);
		be::IO::hall_b.set_io_pull_mode(Clib::GPIO::Pull::DOWN);
		HAL_Delay(50);
		uint16_t a_state_pulldown = ADCDet::ENC_A;
		uint16_t b_state_pulldown = ADCDet::ENC_B;

		if(a_state_pullup > 3500 && a_state_pulldown < 100 && b_state_pullup > 3500 && b_state_pulldown > 3500){
			e_type = sMDUReg::EncType::AMT21x;
		}else if(a_state_pullup > 3500 && a_state_pulldown > 3500 && b_state_pullup > 3500 && b_state_pulldown < 100){
			e_type = sMDUReg::EncType::AS5048;
		}else if(a_state_pullup > 3500 && a_state_pulldown > 3500){
			e_type = sMDUReg::EncType::AS5600;
		}else{
			e_type = sMDUReg::EncType::Robomas;
		}
	}

	switch(e_type){
	case sMDUReg::EncType::Robomas:
		be::IO::hall_a.set_io_pull_mode(Clib::GPIO::Pull::DOWN);
		be::IO::hall_b.set_io_pull_mode(Clib::GPIO::Pull::DOWN);

		be::IO::spi_miso.set_io_mode(Clib::GPIO::Mode::ANALOG);
		be::IO::spi_mosi.set_io_mode(Clib::GPIO::Mode::ANALOG);
		be::IO::spi_sck.set_io_mode(Clib::GPIO::Mode::ANALOG);
		be::as5048.ss.set_io_mode(Clib::GPIO::Mode::ANALOG);
		be::IO::spi_miso.set_io_pull_mode(Clib::GPIO::Pull::NO);

		be::IO::i2c_scl.set_io_mode(Clib::GPIO::Mode::ANALOG);
		be::IO::i2c_sda.set_io_mode(Clib::GPIO::Mode::ANALOG);
		be::IO::i2c_scl.set_io_pull_mode(Clib::GPIO::Pull::NO);
		be::IO::i2c_sda.set_io_pull_mode(Clib::GPIO::Pull::NO);


		be::IO::uart_rx.set_io_mode(Clib::GPIO::Mode::ANALOG);
		be::IO::uart_tx.set_io_mode(Clib::GPIO::Mode::ANALOG);
		break;
	case sMDUReg::EncType::AS5048:
		be::IO::hall_a.set_io_pull_mode(Clib::GPIO::Pull::NO);
		be::IO::hall_b.set_io_pull_mode(Clib::GPIO::Pull::NO);

		be::as5048.ss.set_io_mode(Clib::GPIO::Mode::OUTPUT);
		be::IO::spi_miso.set_io_mode(Clib::GPIO::Mode::FUNCTION);
		be::IO::spi_mosi.set_io_mode(Clib::GPIO::Mode::FUNCTION);
		be::IO::spi_sck.set_io_mode(Clib::GPIO::Mode::FUNCTION);
		be::IO::spi_miso.set_io_pull_mode(Clib::GPIO::Pull::DOWN);

		be::IO::i2c_scl.set_io_mode(Clib::GPIO::Mode::ANALOG);
		be::IO::i2c_sda.set_io_mode(Clib::GPIO::Mode::ANALOG);
		be::IO::i2c_scl.set_io_pull_mode(Clib::GPIO::Pull::NO);
		be::IO::i2c_sda.set_io_pull_mode(Clib::GPIO::Pull::NO);

		be::IO::uart_rx.set_io_mode(Clib::GPIO::Mode::ANALOG);
		be::IO::uart_tx.set_io_mode(Clib::GPIO::Mode::ANALOG);
		break;
	case sMDUReg::EncType::AS5600:
		be::IO::hall_a.set_io_pull_mode(Clib::GPIO::Pull::NO);
		be::IO::hall_b.set_io_pull_mode(Clib::GPIO::Pull::NO);

		be::IO::spi_miso.set_io_mode(Clib::GPIO::Mode::ANALOG);
		be::IO::spi_mosi.set_io_mode(Clib::GPIO::Mode::ANALOG);
		be::IO::spi_sck.set_io_mode(Clib::GPIO::Mode::ANALOG);
		be::as5048.ss.set_io_mode(Clib::GPIO::Mode::ANALOG);
		be::IO::spi_miso.set_io_pull_mode(Clib::GPIO::Pull::NO);

		be::IO::i2c_scl.set_io_mode(Clib::GPIO::Mode::FUNCTION);
		be::IO::i2c_sda.set_io_mode(Clib::GPIO::Mode::FUNCTION);
		be::IO::i2c_scl.set_io_pull_mode(Clib::GPIO::Pull::UP);
		be::IO::i2c_sda.set_io_pull_mode(Clib::GPIO::Pull::UP);

		be::IO::uart_rx.set_io_mode(Clib::GPIO::Mode::ANALOG);
		be::IO::uart_tx.set_io_mode(Clib::GPIO::Mode::ANALOG);
		break;
	case sMDUReg::EncType::AMT21x:
		be::IO::hall_a.set_io_pull_mode(Clib::GPIO::Pull::NO);
		be::IO::hall_b.set_io_pull_mode(Clib::GPIO::Pull::NO);

		be::IO::spi_miso.set_io_mode(Clib::GPIO::Mode::ANALOG);
		be::IO::spi_mosi.set_io_mode(Clib::GPIO::Mode::ANALOG);
		be::IO::spi_sck.set_io_mode(Clib::GPIO::Mode::ANALOG);
		be::as5048.ss.set_io_mode(Clib::GPIO::Mode::ANALOG);
		be::IO::spi_miso.set_io_pull_mode(Clib::GPIO::Pull::NO);

		be::IO::i2c_scl.set_io_mode(Clib::GPIO::Mode::ANALOG);
		be::IO::i2c_sda.set_io_mode(Clib::GPIO::Mode::ANALOG);
		be::IO::i2c_scl.set_io_pull_mode(Clib::GPIO::Pull::NO);
		be::IO::i2c_sda.set_io_pull_mode(Clib::GPIO::Pull::NO);

		be::IO::uart_rx.set_io_mode(Clib::GPIO::Mode::FUNCTION);
		be::IO::uart_tx.set_io_mode(Clib::GPIO::Mode::FUNCTION);
		break;
	default:
		be::IO::hall_a.set_io_pull_mode(Clib::GPIO::Pull::DOWN);
		be::IO::hall_b.set_io_pull_mode(Clib::GPIO::Pull::DOWN);

		be::IO::spi_miso.set_io_mode(Clib::GPIO::Mode::ANALOG);
		be::IO::spi_mosi.set_io_mode(Clib::GPIO::Mode::ANALOG);
		be::IO::spi_sck.set_io_mode(Clib::GPIO::Mode::ANALOG);
		be::as5048.ss.set_io_mode(Clib::GPIO::Mode::ANALOG);
		be::IO::spi_miso.set_io_pull_mode(Clib::GPIO::Pull::NO);

		be::IO::i2c_scl.set_io_mode(Clib::GPIO::Mode::ANALOG);
		be::IO::i2c_sda.set_io_mode(Clib::GPIO::Mode::ANALOG);
		be::IO::i2c_scl.set_io_pull_mode(Clib::GPIO::Pull::NO);
		be::IO::i2c_sda.set_io_pull_mode(Clib::GPIO::Pull::NO);

		be::IO::uart_rx.set_io_mode(Clib::GPIO::Mode::ANALOG);
		be::IO::uart_tx.set_io_mode(Clib::GPIO::Mode::ANALOG);
		break;
	}
}
void Task::pwm_io_disable(void){
	be::IO::pwm_uh.write(false);
	be::IO::pwm_vh.write(false);
	be::IO::pwm_wh.write(false);
	be::IO::pwm_ul.write(false);
	be::IO::pwm_vl.write(false);
	be::IO::pwm_wl.write(false);

	be::IO::pwm_uh.set_io_mode(Clib::GPIO::Mode::OUTPUT);
	be::IO::pwm_ul.set_io_mode(Clib::GPIO::Mode::OUTPUT);
	be::IO::pwm_vh.set_io_mode(Clib::GPIO::Mode::OUTPUT);
	be::IO::pwm_vl.set_io_mode(Clib::GPIO::Mode::OUTPUT);
	be::IO::pwm_wh.set_io_mode(Clib::GPIO::Mode::OUTPUT);
	be::IO::pwm_wl.set_io_mode(Clib::GPIO::Mode::OUTPUT);
}
void Task::pwm_io_enable(void){
	be::IO::pwm_uh.set_io_mode(Clib::GPIO::Mode::FUNCTION);
	be::IO::pwm_ul.set_io_mode(Clib::GPIO::Mode::FUNCTION);
	be::IO::pwm_vh.set_io_mode(Clib::GPIO::Mode::FUNCTION);
	be::IO::pwm_vl.set_io_mode(Clib::GPIO::Mode::FUNCTION);
	be::IO::pwm_wh.set_io_mode(Clib::GPIO::Mode::FUNCTION);
	be::IO::pwm_wl.set_io_mode(Clib::GPIO::Mode::FUNCTION);
}

///////////////////////////////////////////////////////////////////////////
//CAN/USBとの送信
///////////////////////////////////////////////////////////////////////////
void Task::can_communication(void){
	auto rx_frame = be::can.rx();
	if(not rx_frame.has_value()) return;
	auto dp = rx_frame.value().encode_common_data_packet();
	if(not dp.has_value()) return;

	be::led_b_sequencer.play(Blib::LEDPattern::ok);

	std::optional<Clib::Protocol::DataPacket> return_pack;
	switch(dp.value().data_type){
	case Clib::Protocol::DataType::sMDU_ID:
		if(dp.value().board_ID == be::b_params.id){
			return_pack = sMDU_data_operation(dp.value());
		}
		break;
	case Clib::Protocol::DataType::COMMON_ID:
		if(dp.value().board_ID == be::b_params.id){
			return_pack = common_data_operation(dp.value());
		}
		break;
	case Clib::Protocol::DataType::COMMON_ID_ENFORCE:
		return_pack = common_data_operation(dp.value());
		break;
	default:
		return_pack = std::nullopt;
	}

	if(return_pack.has_value()){
		Clib::CanFrame tx_frame;
		tx_frame.decode_common_data_packet(return_pack.value());
		be::can.tx(tx_frame);
	}
}

void Task::usb_communication(){
	auto rx_str = be::usb_cdc.rx();
	if(not rx_str.has_value()) return;

	Clib::CanFrame rx_frame = Clib::SLCAN::slcan_packed_to_can(rx_str.value());
	auto dp = rx_frame.encode_common_data_packet();
	if(not dp.has_value()) return;

	std::optional<Clib::Protocol::DataPacket> return_pack;
	switch(dp.value().data_type){
	case Clib::Protocol::DataType::sMDU_ID:
		if(dp.value().board_ID == be::b_params.id){
			be::led_b_sequencer.play(Blib::LEDPattern::ok);
			return_pack = sMDU_data_operation(dp.value());
		}
		break;
	case Clib::Protocol::DataType::COMMON_ID:
		if(dp.value().board_ID == be::b_params.id){
			be::led_b_sequencer.play(Blib::LEDPattern::ok);
			return_pack = common_data_operation(dp.value());
		}
		break;
	case Clib::Protocol::DataType::COMMON_ID_ENFORCE:
		be::led_b_sequencer.play(Blib::LEDPattern::ok);
		return_pack = common_data_operation(dp.value());
		break;
	default:
		return_pack = std::nullopt;
	}

	if(return_pack.has_value()){
		Clib::CanFrame tx_frame;
		tx_frame.decode_common_data_packet(return_pack.value());
		Clib::StrPack tx_str = Clib::SLCAN::can_to_slcan_packed(tx_frame);

		be::usb_cdc.tx(tx_str);
	}
}

///////////////////////////////////////////////////////////////////////////
//データの反映
///////////////////////////////////////////////////////////////////////////
std::optional<Clib::Protocol::DataPacket> Task::sMDU_data_operation(const Clib::Protocol::DataPacket& dp){
	if(dp.is_request){
		Clib::Protocol::DataPacket return_packet = dp;
		return_packet.data_length = 0;
		return_packet.is_request = false;
		auto w = return_packet.writer();
		if(be::id_map.get(dp.register_ID & 0x00FF, w)){
			return return_packet;
		}else{
			return std::nullopt;
		}
	}else{
		auto r = dp.reader();
		be::id_map.set(dp.register_ID & 0x00FF, r);

		return std::nullopt;
	}
}
std::optional<Clib::Protocol::DataPacket> Task::common_data_operation(const Clib::Protocol::DataPacket& dp){
	Clib::Protocol::DataPacket return_packet = dp;

	switch(dp.register_ID){
	case CReg::ID_RQ:
		if(dp.is_request){
			return_packet.board_ID = be::b_params.id;
			return_packet.data_type = Clib::Protocol::DataType::COMMON_ID;
			return_packet.register_ID = CReg::ID_RQ;
			return_packet.is_request = false;
			return_packet.writer().write<uint8_t>(static_cast<uint8_t>(Clib::Protocol::DataType::sMDU_ID));
			return return_packet;
		}else{
			return std::nullopt;
		}
	case CReg::SAVE_PARAM:
		be::apply_params_rq = true;
		return std::nullopt;
	case CReg::RESET_PARAM:
		be::op_mode = sMDUReg::OpMode::EMS;
		be::b_params = Blib::default_board_param;
		be::apply_params_rq = true;

		return std::nullopt;
	case CReg::EMS:
		be::op_mode = sMDUReg::OpMode::EMS;
		be::error_flags |= 1u << sMDUReg::ErrorFlagBitPos::EXTERNAL_EMS;
		return std::nullopt;
	case CReg::RESET_EMS:
		be::op_mode = sMDUReg::OpMode::NORMAL;
		return std::nullopt;
	default:
		return std::nullopt;
	}
	return std::nullopt;
}

///////////////////////////////////////////////////////////////////////////
//ロボマス互換
///////////////////////////////////////////////////////////////////////////
//ロボマスモータが接続されている場合は互換
//ロボマスモータ以外のモータが接続されている場合はC620フォーマット(最大20A)で動作
void Task::robomas_operation(const Clib::CanFrame& cf){
	if(cf.id != 0x200){
		return;
	}
	auto r = cf.reader();
	int16_t v[4] = {0};
	for(int i = 0; i < 4; i++){
		v[i] = r.read<int16_t>(false).value();
	}
	be::target_i.d = 0.0f;
	MReg::RobomasMD md_type = be::b_params.feedback_format;
	be::target_i.q = std::clamp(Blib::RobomasMotorParam::robomas_value_to_current(md_type, v[be::b_params.id]),-MAX_CURR*0.9f,MAX_CURR*0.9f);
}

Clib::CanFrame Task::robomas_feedback(void){
	CommonLib::CanFrame cf;
	cf.id = 0x201 + be::b_params.id;
	auto writer = cf.writer();

	int16_t angle;
	int16_t spd;
	if(be::b_params.enc_type == sMDUReg::EncType::Robomas){
		angle = static_cast<int16_t>(be::e_angle_enc.get_rad()*static_cast<float>(1<<13)/(2.0*M_PI));
		spd = be::e_angle_enc.get_rad_speed()*60.0f/(2.0*M_PI);
	}else{
		//ロボマスモータ互換を実現したいので，エンコーダの値にM3508のギア比を掛けて出力
		float gear_ratio = (be::b_params.robomas_feedback_enable & 0b1) ? Blib::RobomasMotorParam::get_gear_ratio(be::b_params.feedback_format) : 1.f;
		angle = static_cast<int16_t>(be::e_angle_enc.get_rad()*static_cast<float>(1<<13)*gear_ratio/(2.0*M_PI));
		spd = static_cast<int16_t>(be::e_angle_enc.get_rad_speed()*60.0f*gear_ratio/(2.0*M_PI));
	}

	writer.write<int16_t>(angle,false);
	writer.write<int16_t>(spd,false);

	MReg::RobomasMD md_type = be::b_params.feedback_format;
	writer.write<int16_t>(Blib::RobomasMotorParam::current_to_robomas_value(md_type, be::dq_i.q),false);
	writer.write<int8_t>((md_type == MReg::RobomasMD::C610) ? 0 : 23);
	cf.data_length = 8;
	return cf;
}
//ロボマスフォーマットで送られてきた指令値の処理
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs){
	be::robomas_fb_can.rx_interrupt_task();
	auto robomas_frame = be::robomas_fb_can.rx();
	if(robomas_frame.has_value() && robomas_frame.value().id == 0x200){
		be::control_mode = sMDUReg::ControlMode::CURRENT;
		Task::robomas_operation(robomas_frame.value());
	}
}

///////////////////////////////////////////////////////////////////////////
//こまごました割り込み処理とか
///////////////////////////////////////////////////////////////////////////
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs){
	be::can.rx_interrupt_task();
}

void HAL_FDCAN_TxBufferCompleteCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t BufferIndexes){
	be::can.tx_interrupt_task();
}


void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi){
	be::as5048.read_finish_task();
	be::pll.update_sample(be::as5048.get_angle());
}
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c){
	be::as5600.read_finish_task();
	be::pll.update_sample(be::as5600.get_angle());
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	be::amt21x.read_finish_task();
	be::pll.update_sample(be::amt21x.get_angle());

}

extern "C"
void usb_cdc_rx_callback(const uint8_t *input,size_t size){
	be::usb_cdc.rx_interrupt_task(input, size);
}

extern "C" int _write(int file, char *ptr, int len) {
	//HAL_UART_Transmit(&huart2, (uint8_t*) ptr, len,100);
	CDC_Transmit_FS((uint8_t*)ptr, len);
	return len;
}

