/*
 * button.hpp
 *
 *  Created on: Nov 27, 2025
 *      Author: gomas
 */

#ifndef BUTTON_HPP_
#define BUTTON_HPP_

#include <functional>

namespace CommonLib{
class Button{
	std::function<bool(void)> get_button_state;
	const float update_t;
	const size_t press_th;
	const size_t long_press_th;

	size_t cnt = 0;
	size_t prev_state_cnt = 0;
	bool is_pressing = false;
public:
	Button(std::function<bool(void)> button_state_getter,float update_freq,float press_time = 0.05f,float long_press_time = 2.0f)
	:get_button_state(button_state_getter),
	 update_t(1.0/update_freq),
	 press_th(press_time*update_freq),
	 long_press_th(long_press_time*update_freq){
	}

	bool is_long_pressed(bool peep = false){
		//既にボタンが離されて，前回押されていた時の時間が設定より長ければtrue
		if((prev_state_cnt > long_press_th) && not is_pressing){
			if(peep){
				//nop
			}else{
				prev_state_cnt = 0;
			}
			return true;
		}
		return false;
	}

	bool is_pressed(bool peep = false){
		if((long_press_th > prev_state_cnt) && (prev_state_cnt  > press_th) && not is_pressing){
			if(peep){
				//nop
			}else{
				prev_state_cnt = 0;
			}
			return true;
		}
		return false;
	}

	float get_leaving_time(void){
		if(not is_pressing){
			return cnt*update_t;
		}
		return 0.0f;
	}

	void reset(void){
		cnt = 0;
		prev_state_cnt = 0;
	}

	void set_getter_function(std::function<bool(void)> button_state_getter){
		get_button_state = button_state_getter;
	}

	//タイマー割り込みなどで定期的に呼び出す
	void update(void){
		if(get_button_state == nullptr){
			return;
		}

		//ボタンに変化があったタイミングで処理を行う
		if(is_pressing != get_button_state()){
			prev_state_cnt = cnt;
			cnt = 0;
		}else{
			cnt ++;
		}
		is_pressing = get_button_state();
	}
};
}


#endif /* BUTTON_HPP_ */
