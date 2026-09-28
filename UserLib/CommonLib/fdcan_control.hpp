/*
 * fdcan_control.hpp
 *
 *  Created on: Sep 26, 2024
 *      Author: gomas
 */

#ifndef FDCAN_CONTROL_HPP_
#define FDCAN_CONTROL_HPP_

#include "main.h"

#ifdef HAL_FDCAN_MODULE_ENABLED

#include "can_if.hpp"
#include "auto_callback.hpp"

#include <memory>
#include <cassert>
#include <optional>
#include <iterator>
#include <unordered_map>

namespace CommonLib{

//FDCanモジュールで普通のCANの通信をするクラス
template<bool AUTO_SETUP_CALLBACK = false>
class FdCanComm:public ICan{
public:
	//staticなメンバたち
	//FIFOの設定定数
	struct RxFifoParams{
		uint32_t fifo;
		uint32_t filter;
		uint32_t it;
		uint32_t it_flag;
	};
	static constexpr RxFifoParams fifo_params[] = {
		RxFifoParams{
			FDCAN_RX_FIFO0,
			FDCAN_FILTER_TO_RXFIFO0,
			FDCAN_IT_RX_FIFO0_NEW_MESSAGE,
			FDCAN_FLAG_RX_FIFO0_NEW_MESSAGE
		},
		RxFifoParams{
			FDCAN_RX_FIFO1,
			FDCAN_FILTER_TO_RXFIFO1,
			FDCAN_IT_RX_FIFO1_NEW_MESSAGE,
			FDCAN_FLAG_RX_FIFO1_NEW_MESSAGE
		}
	};

private:
	//自動コールバック関係
	static constexpr bool auto_setup_callback = AUTO_SETUP_CALLBACK && (USE_HAL_FDCAN_REGISTER_CALLBACKS == 1);
	using callback_t = std::conditional_t<auto_setup_callback, AutoSetupCallback<FDCAN_HandleTypeDef*>, Empty>;
	inline static callback_t rxfifo0_callback_manager;
	static void rxfifo0_callback(FDCAN_HandleTypeDef* fdcan,uint32_t RxFifo0ITs){ if constexpr (auto_setup_callback) rxfifo0_callback_manager.callback_f(fdcan); }
	inline static callback_t rxfifo1_callback_manager;
	static void rxfifo1_callback(FDCAN_HandleTypeDef* fdcan,uint32_t RxFifo1ITs){ if constexpr (auto_setup_callback) rxfifo1_callback_manager.callback_f(fdcan); }
	inline static callback_t tx_callback_manager;
	static void tx_callback(FDCAN_HandleTypeDef* fdcan,uint32_t BufferIndexes){ if constexpr (auto_setup_callback) tx_callback_manager.callback_f(fdcan); }


	//クラス本体
	FDCAN_HandleTypeDef* const fdcan;

	std::unique_ptr<IRingBuffer<CanFrame> > rx_buff;
	std::unique_ptr<IRingBuffer<CanFrame> > tx_buff;

	const size_t rx_fifo;

public:
	FdCanComm(FDCAN_HandleTypeDef *_fdcan,std::unique_ptr<IRingBuffer<CanFrame>> _rx_buff,std::unique_ptr<IRingBuffer<CanFrame>> &&_tx_buff,size_t _rx_fifo = 0)
		:fdcan(_fdcan),
		 rx_buff(std::move(_rx_buff)),
		 tx_buff(std::move(_tx_buff)),
		 rx_fifo(_rx_fifo){
		static_assert(not(AUTO_SETUP_CALLBACK && (USE_HAL_FDCAN_REGISTER_CALLBACKS != 1)),"When using AUTO_SETUP_CALLBACK, please enable Register callback using cubeMX.");
		assert(rx_fifo <= std::size(fifo_params));

		if(HAL_FDCAN_GetState(fdcan) == HAL_FDCAN_STATE_READY){
			setup_callback();
			start();
		}
	}

	void start(void);

	void stop(void);


	//自動コールバック設定を利用するときは呼び出し
	void setup_callback(void);



	bool tx(const CanFrame &tx_frame)override;

	std::optional<CanFrame> rx(void)override;


	//FDCanCommでは標準フレームと拡張フレームどちらも受信できるようにはできないので注意
	//すべてのフレームを受信
	bool set_filter_free(uint32_t filter_index,CanFilterMode fmode = CanFilterMode::ONLY_STD);
	//フィルタを設定
	bool set_filter_mask_mode(uint32_t filter_index,uint32_t id,uint32_t mask,CanFilterMode fmode);


	//HAL_FDCAN_TxBufferCompleteCallbackで呼び出し
	void tx_interrupt_task(void);

	//HAL_FDCAN_RxFifo0Callback or HAL_FDCAN_RxFifo1Callback で呼び出し
	void rx_interrupt_task(void);



	uint32_t tx_available(void)const override{
		return tx_buff->get_free_level();

	}
	uint32_t rx_available(void)const override{
		return rx_buff->get_busy_level();
	}


	FDCAN_HandleTypeDef *get_handler(void)const{
		return fdcan;
	}
};//class FdCanComm


///////////////////////////////////////////////////////////////////////////////////////////////////
//FdCanComm
///////////////////////////////////////////////////////////////////////////////////////////////////
template<bool AUTO_SETUP_CALLBACK>
inline void FdCanComm<AUTO_SETUP_CALLBACK>::start(void){
	HAL_FDCAN_Start(fdcan);
	HAL_FDCAN_ActivateNotification(fdcan, fifo_params[rx_fifo].it, fifo_params[rx_fifo].it_flag);
	HAL_FDCAN_ActivateNotification(fdcan, FDCAN_IT_TX_COMPLETE, FDCAN_TX_BUFFER0 | FDCAN_TX_BUFFER1 | FDCAN_TX_BUFFER2);
}

template<bool AUTO_SETUP_CALLBACK>
inline void FdCanComm<AUTO_SETUP_CALLBACK>::stop(void){
	HAL_FDCAN_Stop(fdcan);
}

template<bool AUTO_SETUP_CALLBACK>
inline void FdCanComm<AUTO_SETUP_CALLBACK>::setup_callback(void){
#if (USE_HAL_FDCAN_REGISTER_CALLBACKS == 1)
	if constexpr(auto_setup_callback){
		if(rx_fifo == 0){
			rxfifo0_callback_manager.callbacks.try_emplace(fdcan,[&](){this->rx_interrupt_task();});
			HAL_FDCAN_RegisterRxFifo0Callback(fdcan,rxfifo0_callback);
		}else{
			rxfifo1_callback_manager.callbacks.try_emplace(fdcan,[&](){this->rx_interrupt_task();});
			HAL_FDCAN_RegisterRxFifo1Callback(fdcan,rxfifo1_callback);
		}
		tx_callback_manager.callbacks.try_emplace(fdcan,[&](){this->tx_interrupt_task();});
		HAL_FDCAN_RegisterTxBufferCompleteCallback(fdcan,tx_callback);
	}
#endif
}

template<bool AUTO_SETUP_CALLBACK>
inline bool FdCanComm<AUTO_SETUP_CALLBACK>::tx(const CanFrame &tx_frame){
	if(HAL_FDCAN_GetTxFifoFreeLevel(fdcan) > 0){
		FDCAN_TxHeaderTypeDef tx_header;
		tx_header.Identifier = tx_frame.id;
		tx_header.IdType = tx_frame.is_ext_id ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID;
		tx_header.TxFrameType = tx_frame.is_remote ? FDCAN_REMOTE_FRAME : FDCAN_DATA_FRAME;
		tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
		tx_header.BitRateSwitch = FDCAN_BRS_OFF;
		tx_header.FDFormat = FDCAN_CLASSIC_CAN;
		tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
		tx_header.MessageMarker = 0;

#ifdef STM32G4xx_HAL_H
		tx_header.DataLength = tx_frame.data_length;
#elif STM32H7xx_HAL_H
		tx_header.DataLength = tx_frame.data_length<<16;
#endif

		HAL_FDCAN_AddMessageToTxFifoQ(fdcan, &tx_header, const_cast<uint8_t*>(tx_frame.data));
	}else{
		if(!tx_buff->push(tx_frame)){
			return false;
		}
	}
	return true;
}

template<bool AUTO_SETUP_CALLBACK>
inline std::optional<CanFrame> FdCanComm<AUTO_SETUP_CALLBACK>::rx(void){
	return rx_buff->pop();
}

//HAL_FDCAN_TxBufferCompleteCallbackで呼び出し
template<bool AUTO_SETUP_CALLBACK>
inline void FdCanComm<AUTO_SETUP_CALLBACK>::tx_interrupt_task(void){
	while(HAL_FDCAN_GetTxFifoFreeLevel(fdcan) && tx_buff->get_busy_level()){
		std::optional<CanFrame> tx_frame = tx_buff->pop();
		if(not tx_frame.has_value()){
			break;
		}

		FDCAN_TxHeaderTypeDef tx_header;
		tx_header.Identifier = tx_frame.value().id;
		tx_header.IdType = tx_frame.value().is_ext_id ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID;
		tx_header.TxFrameType = tx_frame.value().is_remote ? FDCAN_REMOTE_FRAME : FDCAN_DATA_FRAME;
		tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
		tx_header.BitRateSwitch = FDCAN_BRS_OFF;
		tx_header.FDFormat = FDCAN_CLASSIC_CAN;
		tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
		tx_header.MessageMarker = 0;

#ifdef STM32G4xx_HAL_H
		tx_header.DataLength = tx_frame.value().data_length;
#elif STM32H7xx_HAL_H
		tx_header.DataLength = tx_frame.data_length<<16;
#endif
		HAL_FDCAN_AddMessageToTxFifoQ(fdcan, &tx_header, const_cast<uint8_t*>(tx_frame.value().data));
	}
}

template<bool AUTO_SETUP_CALLBACK>
inline void FdCanComm<AUTO_SETUP_CALLBACK>::rx_interrupt_task(void){
	FDCAN_RxHeaderTypeDef rx_header;
	CanFrame rx_frame;

	HAL_FDCAN_GetRxMessage(fdcan, fifo_params[rx_fifo].fifo, &rx_header, rx_frame.data);

	rx_frame.data_length = rx_header.DataLength;
	rx_frame.is_remote = rx_header.RxFrameType == FDCAN_REMOTE_FRAME ? true : false;
	rx_frame.is_ext_id = rx_header.IdType == FDCAN_EXTENDED_ID ? true : false;
	rx_frame.id = rx_header.Identifier;

#ifdef STM32G4xx_HAL_H
	rx_frame.data_length = rx_header.DataLength;
#elif STM32H7xx_HAL_H
	rx_frame.data_length = rx_header.DataLength>>16;
#endif

	rx_buff->push(rx_frame);
}

template<bool AUTO_SETUP_CALLBACK>
inline bool FdCanComm<AUTO_SETUP_CALLBACK>::set_filter_free(uint32_t filter_index,CanFilterMode fmode){
	assert(fmode != CanFilterMode::STD_AND_EXT);
	stop();

	FDCAN_FilterTypeDef  filter;
	filter.IdType = fmode == CanFilterMode::ONLY_EXT ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID;
	filter.FilterIndex = filter_index;
	filter.FilterType = FDCAN_FILTER_MASK;
	filter.FilterConfig = fifo_params[rx_fifo].filter;
	filter.FilterID1 = 0x000;
	filter.FilterID2 = 0x000;

	if(HAL_FDCAN_ConfigFilter(fdcan, &filter)!=HAL_OK){
		return false;
	}
	if(HAL_FDCAN_ConfigGlobalFilter(fdcan,FDCAN_REJECT,FDCAN_REJECT,FDCAN_FILTER_REMOTE,FDCAN_FILTER_REMOTE) != HAL_OK){
		return false;
	}
	start();
	return false;
}

template<bool AUTO_SETUP_CALLBACK>
inline bool FdCanComm<AUTO_SETUP_CALLBACK>::set_filter_mask_mode(uint32_t filter_index,uint32_t id,uint32_t mask,CanFilterMode fmode){
	assert(fmode != CanFilterMode::STD_AND_EXT);
	stop();

	FDCAN_FilterTypeDef  filter;
	filter.IdType = (fmode == CanFilterMode::ONLY_EXT) ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID;
	filter.FilterIndex = filter_index;
	filter.FilterType = FDCAN_FILTER_MASK;
	filter.FilterConfig = fifo_params[rx_fifo].filter;
	filter.FilterID1 = id;
	filter.FilterID2 = mask;

	if(HAL_FDCAN_ConfigFilter(fdcan, &filter) != HAL_OK){
		return false;
	}
	if(HAL_FDCAN_ConfigGlobalFilter(fdcan,FDCAN_REJECT,FDCAN_REJECT,FDCAN_FILTER_REMOTE,FDCAN_FILTER_REMOTE) != HAL_OK){
		return false;
	}
	start();
	return true;
}

}//namespace CommonLib

#endif //HAL_FDCAN_MODULE_ENABLED


#endif /* FDCAN_HPP_ */
