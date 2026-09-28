/*
 * programmable_pwm.hpp
 *
 *  Created on: Jun 27, 2024
 *      Author: gomas
 */

#ifndef SEQUENCABLE_IO_HPP_
#define SEQUENCABLE_IO_HPP_


#include <functional>
#include <variant>
#include <bit>
#include <cstdint>

namespace CommonLib{
	using NoteData = std::variant<uint32_t,int32_t,float>;
	struct Note{
		NoteData value;
		uint32_t interval;

		bool is_end_note(void)const{
			return interval == 0;
		}

		//シーケンス終端ノートの生成
		template<class T>
		static inline constexpr Note end_of_sequence(T value = static_cast<T>(0)){
			return Note{value,0};
		}
	};

	//Noteの配列をシーケンスとして実行する
	//Note::interval == 0なNoteを読み込んだ際にシーケンスの終了
	class Sequencer{
	private:
		const Note *playing_pattern_origin = nullptr;
		const Note *playing_pattern = nullptr;
		uint32_t pattern_count = 0;
		uint32_t interval_count = 0;

		std::function<void(NoteData d)> output;
	public:

		template<class T>
		Sequencer(std::function<void(T)> output_func):
			output([=](NoteData d){output_func(std::get<T>(d));}){
		}

		Sequencer(void){
			output = nullptr;
		}

		template<class T>
		void set_output_function(std::function<void(T)> _f){
			output = [=](NoteData d){_f(std::get<T>(d));};
		}

		//pattern:実行するシーケンス配列のポインタ，force:すでに別のシーケンスが走っている場合に上書きして実行するか
		bool play(const Note *pattern,bool force = false){
			if(is_playing() && (not force)){
				return false;
			}
			playing_pattern_origin = pattern;
			playing_pattern = pattern;
			pattern_count = 0;
			interval_count = 0;

			interval_count = playing_pattern[pattern_count].interval;

			output(playing_pattern[pattern_count].value);
			return true;
		}

		bool is_playing(void){
			return playing_pattern!=nullptr;
		}

		const Note * get_playing_pattern(void){
			return playing_pattern_origin;
		}



		void update(void){
			if((playing_pattern == nullptr) || (output == nullptr)){
				return;
			}
			interval_count  --;
			if(interval_count <= 0){
				++pattern_count;

				if(playing_pattern[pattern_count].is_end_note()){
					output(playing_pattern[pattern_count].value);
					playing_pattern = nullptr;
					playing_pattern_origin = nullptr;
					return;
				}
				interval_count = playing_pattern[pattern_count].interval;
				output(playing_pattern[pattern_count].value);
			}
		}

		template<class T>
		void reset(T reset_val = 0){
			playing_pattern = nullptr;
			playing_pattern_origin = nullptr;
			pattern_count = 0;
			interval_count = 0;
			if(output != nullptr){
				output(reset_val);
			}
		}
	};
}





#endif /* SEQUENCABLE_IO_HPP_ */
