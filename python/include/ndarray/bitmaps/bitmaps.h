// Copyright 2021 Innoviz Technologies
//
// Licensed under the Innoviz Open Dataset License Agreement (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://github.com/InnovizTechnologies/InnovizAPI/blob/master/LICENSE.md
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef __NDARRAY_BITMAPS_H__
#define __NDARRAY_BITMAPS_H__

#include <cstdint>
#include <memory>

namespace ndarray {

	namespace bitmap_helpers {

		class IBitFieldGetter {

		public:

			virtual uint32_t get(int index) = 0;

			virtual ~IBitFieldGetter() {}
		};


		class BitFieldGetter : public IBitFieldGetter {

		private:

			static constexpr int key_offset_factor = 1000;

			static inline int key(int offset, int width);

			static constexpr int ckey(int offset, int width);

			std::unique_ptr<IBitFieldGetter> getter;

		public:
			BitFieldGetter();

			BitFieldGetter(void* data, size_t* strides, size_t ndim, int offset, int width);

			uint32_t get(int index);
		};

	}
}

#endif // __NDARRAY_BITMAPS_H__