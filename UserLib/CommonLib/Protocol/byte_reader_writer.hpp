/*
 * byte_reader_writer.hpp
 *
 *  Created on: Sep 26, 2024
 *      Author: gomas
 */

#ifndef BYTE_READER_WRITER_HPP_
#define BYTE_READER_WRITER_HPP_

#include <optional>
#include <cstring>
#include <cstdint>

namespace CommonLib::Protocol{

class ByteWriter{
	uint8_t *const begin;
	uint8_t *const end;
	size_t &written_size;
	public:
	ByteWriter(uint8_t* _begin, uint8_t* _end,size_t &_written_size): begin(_begin),end(_end),written_size(_written_size){
		written_size = 0;
	}
	ByteWriter(uint8_t* _bytes, size_t sz,size_t &_written_size): begin(_bytes),end(_bytes+sz),written_size(_written_size){
		written_size = 0;
	}

	template <class T>
	bool write(const T& value,bool little_endian = true){ //リトルエンディアン
		if (written_size+begin+sizeof(T) > end){
			return false;
		}
		const uint8_t* src_bytes = reinterpret_cast<const uint8_t*>(&value);
		if(little_endian){
			memcpy(begin+written_size, src_bytes,sizeof(T));
		}else{
			for (size_t i = 0; i < sizeof(T); i++) {
				begin[written_size + i] = src_bytes[sizeof(T) - 1 - i];
			}
		}
		written_size += sizeof(T);
		return true;
	}
};

class ByteReader{
	const uint8_t *const begin;
	const uint8_t *const end;
	const uint8_t* iter;
public:
	ByteReader(const uint8_t* _begin, const uint8_t* _end): begin(_begin),end(_end),iter(_begin){}
	ByteReader(const uint8_t* _bytes, size_t sz): begin(_bytes),end(_bytes+sz),iter(_bytes){}

	template <class T>
	std::optional<T> read(bool little_endian = true){
		T value;
		if (iter+sizeof(T) > end){
			return std::nullopt;
		}
		if(little_endian){
			memcpy(&value,iter, sizeof(T));
			iter += sizeof(T);
		}else{
			for(int i = sizeof(T)-1; i >= 0; i--){
				memcpy(reinterpret_cast<uint8_t*>(&value)+ i,iter, 1);
				iter += sizeof(uint8_t);
			}
		}
		return value;
	}
};

}



#endif /* BYTE_READER_WRITER_HPP_ */
