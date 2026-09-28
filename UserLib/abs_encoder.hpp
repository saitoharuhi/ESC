/*
 * AMT212.hpp
 *
 *  Created on: Jul 10, 2025
 *      Author: gomas
 */

#ifndef ABS_ENCODER_HPP_
#define ABS_ENCODER_HPP_

#include "main.h"
#include "CommonLib/Math/motor_math.hpp"
#include "CommonLib/gpio.hpp"

namespace BoardLib{

class IEncoderReader{
public:
	enum class Context{
		SUCCESS,
		FAILURE
	};
	virtual bool is_ready(void)const = 0;
	virtual bool is_dead(void)const = 0;
	virtual void read_start(void) = 0;
	virtual void read_finish_task(Context c = Context::SUCCESS) = 0;
	virtual void reset_state(void) = 0;

	virtual q15_t get_angle(void)const = 0;
	virtual ~IEncoderReader(){}
};

#ifdef HAL_UART_MODULE_ENABLED
class AMT21xEnc:public IEncoderReader{
private:
	static constexpr size_t enc_resolution = 14;
	UART_HandleTypeDef* const uart;

	const uint8_t enc_id;

	uint8_t enc_val[2] = {0};

	bool new_data_available;
	bool no_responce;

	q15_t angle;

public:
	AMT21xEnc(UART_HandleTypeDef* _uart,uint8_t _enc_id = 0x54)
		:uart(_uart),
		 enc_id(_enc_id),
		 new_data_available(false),
		 no_responce(false),
		 angle(0){
	}

	bool is_ready(void)const override{
		return new_data_available;
	}

	bool is_dead(void)const override{
		return no_responce;
	}

	void reset_state(void)override{
		new_data_available = false;
		no_responce = false;
	}


	//エンコーダとの通信に使用する関数群
	//read_start
	//↓
	//read_finish_task(受信完了割り込み内）
	void read_start(void)override{
		if(uart == nullptr){
			return;
		}
		HAL_UART_Transmit_IT(uart, const_cast<uint8_t*>(&enc_id),1);
		HAL_UART_Receive_IT(uart, enc_val, 2);

		if(not new_data_available){
			no_responce = true;
			return;
		}
		new_data_available = false;
	}

	q15_t get_angle(void)const override{
		return angle;
	}

	//HAL_UART_RxCpltCallback内におくこと
	void read_finish_task(Context c = Context::SUCCESS)override{
		angle = (enc_val[1]<<8 | enc_val[0])<<(16-enc_resolution);
		no_responce = false;
		new_data_available = true;
	}

	UART_HandleTypeDef* get_handler(void){
		return uart;
	}
};

#endif //HAL_UART_MODULE_ENABLED


#ifdef HAL_I2C_MODULE_ENABLED
class AS5600Enc:public IEncoderReader{
private:
	static constexpr uint16_t as5600_id = 0x36;
	static constexpr size_t as5600_resolution = 12;

	I2C_HandleTypeDef* const i2c;

	uint8_t enc_val[2] = {0};

	bool new_data_available;
	bool no_responce;

	q15_t angle;
public:
	AS5600Enc(I2C_HandleTypeDef* _i2c)
		:i2c(_i2c),
		 new_data_available(false),
		 no_responce(false),
		 angle(0){
	}

	bool is_ready(void)const override{
		return new_data_available;
	}

	bool is_dead(void)const override{
		return no_responce;
	}

	void reset_state(void)override{
		new_data_available = false;
		no_responce = false;
	}

	void read_start(void) override{
		HAL_I2C_Mem_Read_IT(i2c, as5600_id<<1, 0x0c, I2C_MEMADD_SIZE_8BIT, enc_val, 2);
		if(not new_data_available){
			no_responce = true;
			return;
		}
		new_data_available = false;
	}

	//HAL_I2C_MemRxCpltCallbackで呼び出すこと(context = Context::SUCCESS)
	//HAL_I2C_ErrorCallbackの場合context = Context::FAILURE
	void read_finish_task(Context c = Context::SUCCESS)override{//通常の
		if(c == Context::SUCCESS){
			angle = (enc_val[0]<<8 | enc_val[1])<<(16-as5600_resolution);
			new_data_available = true;
			no_responce = false;
		}else{
			new_data_available = false;
			no_responce = true;
		}
	}

	q15_t get_angle(void)const override{
		return angle;
	}

	I2C_HandleTypeDef* get_handler(void){
		return i2c;
	}
};
#endif //HAL_I2C_MODULE_ENABLED

#ifdef HAL_SPI_MODULE_ENABLED
class AS5048AEnc:public IEncoderReader{
	static constexpr size_t as5048a_resolution = 14;

	SPI_HandleTypeDef* const spi;
	bool new_data_available;
	bool no_responce;

	volatile uint8_t enc_val[2] = {0};
	uint8_t tx[2] = {0xFF,0xFF};

	q15_t angle;

	bool command_sending = false;

public:
	CommonLib::GPIO ss;
	AS5048AEnc(SPI_HandleTypeDef* _spi,CommonLib::GPIO _ss)
		:spi(_spi),
		 new_data_available(false),
		 no_responce(false),
		 angle(0),
		 ss(_ss){
	}

	bool is_ready(void)const override{
		return new_data_available;
	}

	bool is_dead(void)const override{
		return no_responce;
	}

	void reset_state(void)override{
		new_data_available = false;
		no_responce = false;
	}

	void read_start(void)override{
		command_sending = true;//一旦読み取り用のコマンドを送信
		ss.write(false);
		HAL_SPI_TransmitReceive_IT(spi,tx,const_cast<uint8_t*>(enc_val), 2);
		if(not new_data_available){
			no_responce = true;
			return;
		}
		new_data_available = false;
	}

	void read_finish_task(Context c = Context::SUCCESS)override{
		ss.write(true);

//		if(c == Context::SUCCESS){
		if(c == Context::SUCCESS && !((enc_val[0]>>6) & 0b1)){
			if(command_sending){//一回目のデータは捨てる
				ss.write(false);
				HAL_SPI_TransmitReceive_IT(spi,tx,const_cast<uint8_t*>(enc_val), 2);
				command_sending = false;
				return;
			}

			angle = (enc_val[0]<<8 | enc_val[1])<<(16-as5048a_resolution);
			new_data_available = true;
			no_responce = false;
		}else{
			new_data_available = false;
			no_responce = true;
		}
	}

	q15_t get_angle(void)const override{
		return angle;
	}

	SPI_HandleTypeDef* get_handler(void){
		return spi;
	}
};
#endif //HAL_SPI_MODULE_ENABLED

}// namespace BoardLib

#endif /* ABS_ENCODER_HPP_ */
