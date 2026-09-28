/*
 * timer_helper.hpp
 *
 *  Created on: Jul 11, 2025
 *      Author: gomas
 */

#ifndef COMMONLIB_TIMER_HELPER_HPP_
#define COMMONLIB_TIMER_HELPER_HPP_

#include "main.h"

namespace CommonLib::TimerHelper{

//TODO:動作確認
inline uint32_t get_timer_clock_freq(TIM_TypeDef* tim_instance){
	uint32_t tim_clock = 0;

	//タイマーのクロックソースを特定
	//STM32F3,G4,H7を参考に決定
	switch(reinterpret_cast<uint32_t>(tim_instance)){
#ifdef TIM1
	case reinterpret_cast<uint32_t>(TIM1_BASE):
		tim_clock = SystemCoreClock >> (APBPrescTable[READ_BIT(RCC->CFGR, RCC_CFGR_PPRE2) >> RCC_CFGR_PPRE2_Pos] & 0x1FU);
		break;
#endif //TIM1
#ifdef TIM2
	case reinterpret_cast<uint32_t>(TIM2_BASE):
		tim_clock = SystemCoreClock >> (APBPrescTable[READ_BIT(RCC->CFGR, RCC_CFGR_PPRE1) >> RCC_CFGR_PPRE1_Pos] & 0x1FU);
		break;
#endif //TIM2

#ifdef TIM3
	case reinterpret_cast<uint32_t>(TIM3_BASE):
		tim_clock = SystemCoreClock >> (APBPrescTable[READ_BIT(RCC->CFGR, RCC_CFGR_PPRE1) >> RCC_CFGR_PPRE1_Pos] & 0x1FU);
		break;
#endif //TIM3

#ifdef TIM4
	case reinterpret_cast<uint32_t>(TIM4_BASE):
		tim_clock = SystemCoreClock >> (APBPrescTable[READ_BIT(RCC->CFGR, RCC_CFGR_PPRE1) >> RCC_CFGR_PPRE1_Pos] & 0x1FU);
		break;
#endif //TIM4

#ifdef TIM5
	case reinterpret_cast<uint32_t>(TIM5_BASE):
		tim_clock = SystemCoreClock >> (APBPrescTable[READ_BIT(RCC->CFGR, RCC_CFGR_PPRE1) >> RCC_CFGR_PPRE1_Pos] & 0x1FU);
		break;
#endif //TIM5

#ifdef TIM6
	case reinterpret_cast<uint32_t>(TIM6_BASE):
		tim_clock = SystemCoreClock >> (APBPrescTable[READ_BIT(RCC->CFGR, RCC_CFGR_PPRE1) >> RCC_CFGR_PPRE1_Pos] & 0x1FU);
		break;
#endif //TIM6

#ifdef TIM7
	case reinterpret_cast<uint32_t>(TIM7_BASE):
		tim_clock = SystemCoreClock >> (APBPrescTable[READ_BIT(RCC->CFGR, RCC_CFGR_PPRE1) >> RCC_CFGR_PPRE1_Pos] & 0x1FU);
		break;
#endif //TIM7

#ifdef TIM8
	case reinterpret_cast<uint32_t>(TIM8_BASE):
		tim_clock = SystemCoreClock >> (APBPrescTable[READ_BIT(RCC->CFGR, RCC_CFGR_PPRE2) >> RCC_CFGR_PPRE2_Pos] & 0x1FU);
		break;
#endif //TIM8

#ifdef TIM12
	case reinterpret_cast<uint32_t>(TIM12_BASE):
		tim_clock = SystemCoreClock >> (APBPrescTable[READ_BIT(RCC->CFGR, RCC_CFGR_PPRE1) >> RCC_CFGR_PPRE1_Pos] & 0x1FU);
		break;
#endif //TIM12

#ifdef TIM13
	case reinterpret_cast<uint32_t>(TIM13_BASE):
		tim_clock = SystemCoreClock >> (APBPrescTable[READ_BIT(RCC->CFGR, RCC_CFGR_PPRE1) >> RCC_CFGR_PPRE1_Pos] & 0x1FU);
		break;
#endif //TIM13

#ifdef TIM14
	case reinterpret_cast<uint32_t>(TIM14_BASE):
		tim_clock = SystemCoreClock >> (APBPrescTable[READ_BIT(RCC->CFGR, RCC_CFGR_PPRE1) >> RCC_CFGR_PPRE1_Pos] & 0x1FU);
		break;
#endif //TIM14

#ifdef TIM15
	case reinterpret_cast<uint32_t>(TIM15_BASE):
		tim_clock = SystemCoreClock >> (APBPrescTable[READ_BIT(RCC->CFGR, RCC_CFGR_PPRE2) >> RCC_CFGR_PPRE2_Pos] & 0x1FU);
		break;
#endif //TIM15

#ifdef TIM16
	case reinterpret_cast<uint32_t>(TIM16_BASE):
		tim_clock = SystemCoreClock >> (APBPrescTable[READ_BIT(RCC->CFGR, RCC_CFGR_PPRE2) >> RCC_CFGR_PPRE2_Pos] & 0x1FU);
		break;
#endif //TIM16

#ifdef TIM17
	case reinterpret_cast<uint32_t>(TIM17_BASE):
		tim_clock = SystemCoreClock >> (APBPrescTable[READ_BIT(RCC->CFGR, RCC_CFGR_PPRE2) >> RCC_CFGR_PPRE2_Pos] & 0x1FU);
		break;
#endif //TIM17
#ifdef TIM20
	case reinterpret_cast<uint32_t>(TIM20_BASE):
		tim_clock = SystemCoreClock >> (APBPrescTable[READ_BIT(RCC->CFGR, RCC_CFGR_PPRE2) >> RCC_CFGR_PPRE2_Pos] & 0x1FU);
		break;
#endif //TIM20
	default:
		return 0;
	}

	if(SystemCoreClock == tim_clock){
		//nop
	}else{
		tim_clock *=2;
	}

	return tim_clock/(tim_instance->PSC+1);//何故かHALにPSCを読む関数がない
}

} //namespace TimerHelper


#endif /* COMMONLIB_TIMER_HELPER_HPP_ */
