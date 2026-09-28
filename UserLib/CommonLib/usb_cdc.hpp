/*
 * usb_cdc.hpp
 *
 *  Created on: Jul 13, 2025
 *      Author: gomas
 */

#ifndef USB_CDC_HPP_
#define USB_CDC_HPP_

//"usbd_cdc_if.h" "usb_device.h"をincludeしてからこのファイルをincludeすること
//ゆるして

#ifdef __USBD_CDC_IF_H__
#ifdef __USB_DEVICE__H__

///////////////////////////////////////////////////////////////////////////////
//USB CDC通信
///////////////////////////////////////////////////////////////////////////////
#include "serial_if.hpp"

namespace CommonLib{

//UsbCdcCommクラスは使い方が特殊なので注意
//適当なインスタンスを生成し（例:usb_cdc），次のような関数を定義する。
//extern "C" void usb_cdc_rx_callback(const uint8_t *input,size_t size){
//	usb_cdc.rx_interrupt_task(input, size);
//}
//この関数をUSB_Device/App/usb_cdc_if.cのCDC_Receive_FS関数内で呼び出す
//usb_cdc.tx_buff_management関数はタイマー割込みなどを用いて定期的に実行すること

class UsbCdcComm : public ISerial{
private:
	USBD_HandleTypeDef *usb;
	std::unique_ptr<IRingBuffer<StrPack>> rx_buff;
	std::unique_ptr<IRingBuffer<StrPack>> tx_buff;

	StrPack tmp_buff;
public:
	UsbCdcComm(USBD_HandleTypeDef *_usb,
			std::unique_ptr<IRingBuffer<StrPack>> _rx_buff,
			std::unique_ptr<IRingBuffer<StrPack>> _tx_buff)
	:usb(_usb),
	 rx_buff(std::move(_rx_buff)),
	 tx_buff(std::move(_tx_buff)){
	}

	USBD_HandleTypeDef *get_usb_handle(void)const{
		return usb;
	}

	////////////////////////////////////////////////////////////////////////////////////////////
	//tx functions
	////////////////////////////////////////////////////////////////////////////////////////////
	bool tx(const StrPack &data) override{
		USBD_CDC_HandleTypeDef *cdc = (USBD_CDC_HandleTypeDef*)usb->pClassData;

		if (cdc->TxState != 0){
			tx_buff->push(data);
			return true;
		}

		USBD_CDC_SetTxBuffer(usb, reinterpret_cast<uint8_t*>(const_cast<char*>(data.data)), data.size);
		if(USBD_CDC_TransmitPacket(usb) != USBD_OK){
			return false;
		}
		return true;
	}

	size_t tx_available(void)const override{
		return tx_buff->get_free_level();
	}
	void tx_buff_management(void){//送信しきれなかったデータについて管理する
		USBD_CDC_HandleTypeDef *cdc = (USBD_CDC_HandleTypeDef*)usb->pClassData;
		if (cdc->TxState != 0){
			std::optional<StrPack> tx_tmp = tx_buff->pop();
			if(tx_tmp.has_value()){
				tx(tx_tmp.value());
			}
		}
	}

	////////////////////////////////////////////////////////////////////////////////////////////
	//rx functions
	////////////////////////////////////////////////////////////////////////////////////////////
	std::optional<StrPack> rx(void) override{
		return rx_buff->pop();
	}
	size_t rx_available(void) const override{
		return rx_buff->get_busy_level();
	}

	void rx_interrupt_task(const uint8_t *input,size_t size){
		for(size_t i = 0; i < size; i++){
			if((input[i]=='\r') || (input[i]=='\n') || (input[i]=='\0') || (tmp_buff.size >= tmp_buff.max_size-1)){
				tmp_buff.data[tmp_buff.size] = input[i];
				tmp_buff.size ++;

				rx_buff->push(tmp_buff);

				tmp_buff.size = 0;
				memset(tmp_buff.data,0,tmp_buff.max_size);
			}else{
				tmp_buff.data[tmp_buff.size] = input[i];
				tmp_buff.size ++;
			}
		}
	}

};
}




#endif //__USBD_CDC_IF_H__
#endif //__USB_DEVICE__H__


#endif /* USB_CDC_HPP_ */
