/*
 * auto_callback.hpp
 *
 *  Created on: Aug 17, 2025
 *      Author: gomas
 */

#ifndef COMMONLIB_AUTO_CALLBACK_HPP_
#define COMMONLIB_AUTO_CALLBACK_HPP_

#include <functional>
#include <unordered_map>

namespace CommonLib{

template<class T>
class AutoSetupCallback{
public:
	std::unordered_map<T,std::function<void(void)>> callbacks{};

	void callback_f(T key){
		auto iter = callbacks.find(key);
		if(iter != callbacks.end() && (iter->second != nullptr)){
			iter->second();
		}
	}
};

class Empty{

};

}//namespace CommonLib



#endif /* COMMONLIB_AUTO_CALLBACK_HPP_ */
