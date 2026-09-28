/*
 * slcan.hpp
 *
 *  Created on: Jul 11, 2025
 *      Author: gomas
 */

#ifndef COMMONLIB_SLCAN_HPP_
#define COMMONLIB_SLCAN_HPP_

#include "can_if.hpp"
#include "serial_if.hpp"

namespace CommonLib::SLCAN{
constexpr size_t SLCAN_STR_MAX_SIZE = 28;

inline size_t can_to_slcan(const CanFrame &frame,char *str,const size_t str_max_size){
	if(str_max_size < SLCAN_STR_MAX_SIZE){
		return 0;
	}
	size_t head = 0;
	//frame type(0)
	if(frame.is_remote){
		if(frame.is_ext_id){
			str[head] = 'R';
		}else{
			str[head] = 'r';
		}
	}else{
		if(frame.is_ext_id){
			str[head] = 'T';
		}else{
			str[head] = 't';
		}
	}

	//ID
	if(frame.is_ext_id){
		for(head = 1; head < 9; head++){
			str[head] = (frame.id >> 4*(8-head)) & 0xF;
			if(str[head] < 10){
				str[head] += '0';
			}else{
				str[head] += 'A'-10;
			}
		}
	}else{
		for(head = 1; head < 4; head++){
			str[head] = (frame.id >> 4*(3-head)) & 0xF;
			if(str[head] < 10){
				str[head] += '0';
			}else{
				str[head] += 'A'-10;
			}
		}
	}

	//DLC
	str[head++] = frame.data_length + '0';

	if(frame.is_remote){
		str[++head] = '\r';
	}else{
		for(size_t i = 0; i < frame.data_length; i++){
			str[i*2 + head] = (frame.data[i] >> 4)&0xF;

			if(str[i*2 + head] < 10){
				str[i*2 + head] += '0';
			}else{
				str[i*2 + head] += 'A'-10;
			}

			str[i*2 + head+1] = frame.data[i] & 0xF;
			if(str[i*2 + head+1] < 10){
				str[i*2 + head+1] += '0';
			}else{
				str[i*2 + head+1] += 'A'-10;
			}
		}
		head += frame.data_length*2;

		str[head] = '\r';
	}

	return ++head;
}


inline bool slcan_to_can(const char *str, CanFrame &frame){
	int head = 0;
	switch(str[head]){
	case 't':
		frame.is_ext_id = false;
		frame.is_remote = false;
		break;
	case 'T':
		frame.is_ext_id = true;
		frame.is_remote = false;
		break;
	case 'R':
		frame.is_remote = true;
		frame.is_ext_id = true;
		break;
	case 'r':
		frame.is_remote = true;
		frame.is_ext_id= false;
		break;
	default:
		return false;
		break;
	}
	//ID
	if(frame.is_ext_id){
		for(head = 1; head < 9; head++){
			int tmp = 0;
			if(str[head] >= 'A'){
				tmp = (str[head] - 'A'+10) & 0xF;
			}else{
				tmp = (str[head] - '0') & 0xF;
			}

			frame.id |= tmp << 4*(8-head);
		}
	}else{
		for(head = 1; head < 4; head++){
			int tmp = 0;
			if(str[head] >= 'A'){
				tmp = (str[head] - 'A'+10) & 0xF;
			}else{
				tmp = (str[head] - '0') & 0xF;
			}

			frame.id |= tmp << 4*(3-head);
		}
	}

	//DLC
	frame.data_length = str[head++]&0xF;
	if(frame.data_length > 8){
		return false;
	}

	//data
	if(frame.is_remote){

	}else{
		for(size_t i = 0; i < frame.data_length; i ++){
			int tmp1 = str[head + 2*i];
			int tmp2 = str[head + 1 + 2*i];
			if(tmp1 >= 'A'){
				tmp1 = (tmp1 - 'A'+10) & 0xF;
			}else{
				tmp1 = (tmp1 - '0') & 0xF;
			}

			if(tmp2 >= 'A'){
				tmp2 = (tmp2 - 'A'+10) & 0xF;
			}else{
				tmp2 = (tmp2 - '0') & 0xF;
			}

			frame.data[i] = (tmp1 << 4) | tmp2;
		}
	}
	return true;
}


inline StrPack can_to_slcan_packed(const CanFrame &frame){
	StrPack pack;
	pack.size = can_to_slcan(frame,pack.data,pack.max_size);
	return pack;
}

inline CanFrame slcan_packed_to_can(const StrPack &pack){
	CanFrame frame;
	slcan_to_can(pack.data,frame);
	return frame;
}
}

#endif /* COMMONLIB_SLCAN_HPP_ */
