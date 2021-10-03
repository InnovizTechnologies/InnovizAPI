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

/*
* Requires C++14 and above
* heavily inspired from https://docs.scipy.org/doc/numpy-1.15.1/reference/generated/numpy.dtype.html
*/

#ifndef __NDARRAY_H__
#define __NDARRAY_H__

// define int to string converter
#define NDARRAY_IND_STR(x) #x
#define NDARRAY_STR(x) NDARRAY_IND_STR(x)

#define NDARRAY_VERSION_MAJOR 0
#define NDARRAY_VERSION_MINOR 1
#define NDARRAY_VERSION_PATCH 0

#define NDARRAY_VERSION NDARRAY_STR(NDARRAY_VERSION_MAJOR)  ## \
	"." ## NDARRAY_STR(NDARRAY_VERSION_MINOR) ## \
	"." ## NDARRAY_STR(NDARRAY_VERSION_PATCH)

#define NDARRAY_DTYPE_DEFAULT_NAME "void96"
#define CS_IGNORE // used by cpp sharp as an ignore flag

#ifdef NDARRAY_EXPORT
#if defined(_WIN32)
#define NDARRAY_API __declspec(dllexport)
#elif __GNUC__ >= 4
#define NDARRAY_API __attribute__ ((visibility("default")))
#define __stdcall 
#else
#error Unsupported platform
#endif
#else
#ifdef NDARRAY_IMPORT
#if defined(_WIN32)
#define NDARRAY_API _declspec(dllimport)
#elif __GNUC__ >= 4
#define NDARRAY_API 
#define __stdcall 
#else
#error Unsupported platform
#endif
#else
#define NDARRAY_API
#endif
#endif

#include <vector>
#include <memory>
#include <sstream>
#include <string>

#ifdef NDARRAY_NLOHMANN_INCLUDE
#include <nlohmann/json.hpp>
#endif

#include "common.h"
#include "ndarray/bitmaps/bitmaps.h"

/**
 * change functions to camel case.
*/
namespace ndarray {

	enum EFormat {
		EFORMAT_NONE,
		EFORMAT_U1,
		EFORMAT_S1,
		EFORMAT_U2,
		EFORMAT_S2,
		EFORMAT_U4,
		EFORMAT_S4,
		EFORMAT_U8,
		EFORMAT_S8,
		EFORMAT_F4,
		EFORMAT_F8,
		EFORMAT_STRUCT,
		EFORMAT_BM1,
		EFORMAT_BM2,
		EFORMAT_BM4,
		EFORMAT_BM8,
		EFORMAT_BIT_FIELD,
		EFORMAT_SBIT_FIELD,
	};

	enum EMEMManagement {
		E_MEM_MNG_ALLOCATE,
		// E_MEM_MNG_MOVE, // FUTURE
		E_MEM_MNG_VIEW,
	};

	enum EFilterType {
        E_FILTER_TYPE_BASE = 0,
		E_FILTER_TYPE_NOISE = 1,
		E_FILTER_TYPE_LESS_THAN_THR = 2,
		E_FILTER_TYPE_CLASS = 3,
		E_FILTER_TYPE_EQUAL_TO = 3,
	};

    typedef uint32_t nf_label_t;

    enum class nf_pixel_state : uint8_t {
        rejected,
        unchanged,
        added,
    };

    static constexpr int nf_rangescount = 4;

	struct NDARRAY_API FilterAttr {
    public:
		std::string selector = "";
		EFilterType type = E_FILTER_TYPE_BASE;

        explicit FilterAttr(std::string selector, EFilterType type) : selector{selector}, type{type} {}
        explicit FilterAttr(const FilterAttr& other) : selector{other.selector}, type{other.type} {}

        virtual FilterAttr* copy() const { return nullptr; }
        virtual ~FilterAttr() {}

    protected:
        FilterAttr& operator=(const FilterAttr& other) {
            selector = other.selector;
            type = other.type;
            return *this;
        }
	};

    struct NDARRAY_API LessThanFilterAttr : public FilterAttr {
    public:
        double threshold = 0.;

        explicit LessThanFilterAttr(std::string selector, double _threshold = 0.) :
            FilterAttr{selector, E_FILTER_TYPE_LESS_THAN_THR}, threshold{_threshold} {}

        virtual FilterAttr* copy() const {
            return new LessThanFilterAttr(*reinterpret_cast<const LessThanFilterAttr*>(this));
        }

        virtual ~LessThanFilterAttr() {}
    };

    struct NDARRAY_API NoiseFilterAttr : public FilterAttr {
    public:
        double data_ranges[nf_rangescount - 1] = {500., 1400., 3350.,};
        double neighbor_threshold[nf_rangescount] = {30., 64., 137., 188.};
        uint16_t segment_size_threshold[nf_rangescount] = {10, 8, 5, 4};
        
        nf_pixel_state* state_img = nullptr;
        nf_label_t* label_img = nullptr;

        explicit NoiseFilterAttr(const double *_data_ranges = nullptr,
                                 const double *_neighbor_threshold = nullptr,
                                 const uint16_t *_segment_size_threshold = nullptr) :
            FilterAttr{"distance", E_FILTER_TYPE_NOISE} {
            copy_parameters(_data_ranges, _neighbor_threshold, _segment_size_threshold);
        }

		NoiseFilterAttr(const NoiseFilterAttr& other) :
            FilterAttr{other} {
            copy_parameters(other.data_ranges, other.neighbor_threshold, other.segment_size_threshold);
        }

        NoiseFilterAttr& operator=(const NoiseFilterAttr& other) {
            FilterAttr::operator=(other);
            copy_parameters(other.data_ranges, other.neighbor_threshold, other.segment_size_threshold);
            return *this;
        }

        virtual FilterAttr* copy() const {
            return new NoiseFilterAttr(*reinterpret_cast<const NoiseFilterAttr*>(this));
        }

        virtual ~NoiseFilterAttr() {}

    protected:
        void copy_parameters(const double *_data_ranges = nullptr,
                             const double *_neighbor_threshold = nullptr,
                             const uint16_t *_segment_size_threshold = nullptr) {
            if (_data_ranges)
                memcpy(data_ranges, _data_ranges, sizeof(data_ranges));
            if (_neighbor_threshold)
                memcpy(neighbor_threshold, _neighbor_threshold, sizeof(neighbor_threshold));
            if (_segment_size_threshold)
                memcpy(segment_size_threshold, _segment_size_threshold, sizeof(segment_size_threshold));
        }
    };

	class NDARRAY_API DType {

	public:

		/* static functions */

		template< typename T > static std::string tformat();

		static std::string format(EFormat type);

	private:
		size_t _itemsize;
		EFormat _eformat;
		size_t _ndim;
		std::unique_ptr<size_t[]> _shape;
		std::string _name;
		std::string _description;

		/* fields meta data */
		size_t _nfields;
		std::unique_ptr<DType[]> _fields_dtypes;
		std::unique_ptr<size_t[]> _fields_offsets;  /** offsets in bytes of each field. */

		/* bit resolution members - relevant for bitfield only */
		int _bit_offset;
		int _bit_width;

		/* private methods */
		void copy(const DType& src);
#ifdef NDARRAY_NLOHMANN_INCLUDE
		/* private constructors and constructors helpers*/
		CS_IGNORE void parse_struct_array_fields(nlohmann::json struct_, size_t &itemsize, size_t &nfields, std::unique_ptr<DType[]> &fields_dtypes, std::unique_ptr<size_t[]> &fields_offsets);
		
		CS_IGNORE void parse_struct_array_bitfields(nlohmann::json bitmap, size_t itemsize, size_t &nfields, std::unique_ptr<DType[]> &fields_dtypes, std::unique_ptr<size_t[]> &fields_offsets);

		CS_IGNORE void parse_struct_dtype(std::string name, std::string description, nlohmann::json struct_, std::vector<size_t> shape);

		CS_IGNORE void parse_basic_or_bitmap_dtype(std::string name, std::string description, nlohmann::json struct_, nlohmann::json bitmap, std::vector<size_t> shape);

		/* constructing getting handle to json_obj to avoid json class explicit usage in interface */
		CS_IGNORE DType(nlohmann::json dtype);
#endif

	public:
		DType();

		CS_IGNORE DType(const char dtype_s[]); // ignore by cpp sharp to avoid duplcation of string constructor

		DType(std::string dtype_s);

		/* copy constructor */
		DType(const DType& rhs);

		/* operators */
		DType& operator=(const DType& rhs);

		bool operator==(const DType& rhs) const;

		bool operator!=(const DType& rhs) const;

		std::string str();

		std::string pybind11_format();

		friend NDARRAY_API std::ostream& operator<<(std::ostream& os, const DType& rhs);

		/* getters */
		const size_t itemsize();
		const EFormat eformat();
		const std::string numpy_typestr();
		const std::string name();
		const std::string description();
		const size_t ndim();
		const size_t* shape();
		
		/* fields meta data */
		const size_t nfields();
		const DType* fields_dtypes();
		const size_t* fields_offsets();
		
		/* bit resolution members */
		const int bit_offset();
		const int bit_width();		

		/* memcpy \ assignment getters */
		void get_shape(size_t* shape, size_t ndim);
		
		void get_fields_dtypes(DType* fields_dtypes, size_t nfields);
		void get_field_dtype(DType &fields_dtype, size_t index);
		void get_fields_offsets(size_t* fields_offsets, size_t nfields);
		void get_fields_names(std::string* names, size_t nfields);		
		void get_field_name(std::string &name, size_t index);

		/* vectors getters (safe) */
		const std::vector<size_t> shape_s();

		const std::vector<DType> fields_dtypes_s();
		const std::vector<size_t> fields_offsets_s();
		const std::vector<std::string> fields_names_s();

		/* base */
		const DType base();

	};

	class NDARRAY_API NDArray {
	public:
		static NDArray* create(DType dtype, std::vector<size_t> shape, void* data = nullptr, EMEMManagement memmng=E_MEM_MNG_ALLOCATE);

		static NDArray* create(DType dtype, size_t shape[], size_t ndims, void* data = nullptr, EMEMManagement memmng = E_MEM_MNG_ALLOCATE);

		static NDArray* share_ptr(NDArray* ndarray);

		static void release(NDArray* ndarray);

		/* helpers */
		static void dtype_to_strides(DType dtype, const size_t ndim, const size_t shape[], size_t strides[], size_t &size);

	private:
		/* members */
		size_t _size;
		size_t _ndim;
		std::unique_ptr<size_t[]> _shape;
		std::unique_ptr <size_t[]> _strides;		/*!< number of byte for next item in current dimenstion */
		std::unique_ptr<char[]> _buffer;	/*!< buffer which holds data, only managed in base */
		char* _data;						/*!< address where data starts, might be with offset relative to _buffer*/
		DType _dtype;
		NDArray* _base;	/*!< pointer to base NDArray from which current NDArray was derived. */
		std::unique_ptr<bitmap_helpers::IBitFieldGetter> _bf_getter;
		
		
		/* constructors */
        NDArray();
		NDArray(DType dtype, size_t* shape, size_t ndim, void* data, EMEMManagement memmng);

		// create NDArray equivalent in structure to self
		NDArray(NDArray* ndarray);

		/*
		* @desc creates an NDArray for required field selector
		* base will be assign to source NDArray, strides will be the same as base NDArray.
		*/
		NDArray(std::string selector, NDArray* base);

		/*
		* @desc creates an NDArray for required index
		* base will be assign to source NDArray, strides will be the same as base NDArray
		*/
		NDArray(size_t index, NDArray* base);

		~NDArray();

		std::unique_ptr<bitmap_helpers::IBitFieldGetter> get_bitfield_getter();

		void get_field_details(const char* selector, size_t pos, DType dtype, size_t &f_offset, DType &f_dtype);

	public:
		/* getter templates */
		template <typename T> inline double getd(size_t index) {
			return static_cast<double>(get<T>(index));
		}

		template <typename T> inline T get(size_t index) {
			return *(T*)(_data + index * _strides[_ndim-1]);
		}

		template <typename T> inline T get_s(size_t index, EFormat edtype) {
			if (_ndim != 1) {
				std::stringstream msg;
				msg << "access by single index only supported for 1d array.";
				throw std::runtime_error(msg.str());
			}

			if (_dtype.eformat() != edtype) {
				std::stringstream msg;
				msg << "requested type of format '" << DType::format(edtype) << "', for field in format '" << DType::format(_dtype.eformat()) << "'.";
				throw std::runtime_error(msg.str());
			}

			if (index > _size) {
				std::stringstream msg;
				msg << "index '" << index << "' is out of range '" << _size << "'.";
				throw std::runtime_error(msg.str());
			}

			return get<T>(index);
		}

		template <typename T> inline T get(size_t r, size_t c) {
			return (((T*)(&_data[r * _strides[0] + c * _strides[1]]))[0]);
		}

		template <typename T> inline T get_s(size_t r, size_t c, EFormat edtype) {
			if (_ndim != 2) {
				std::stringstream msg;
				msg << "access by row, column only supported for 2d array.";
				throw std::runtime_error(msg.str());
			}

			if (_dtype.eformat() != edtype) {
				std::stringstream msg;
				msg << "incompatible format type requested, requested format '" << DType::format(edtype) <<
					"', for field of format '" << DType::format(_dtype.eformat()) << "'.";
				throw std::runtime_error(msg.str());
			}

			if (r > _shape[0]) {
				std::stringstream msg;
				msg << "row '" << r << "' is out of rows size " << _shape[0] << "'.";
				throw std::runtime_error(msg.str());
			}

			if (c > _shape[1]) {
				std::stringstream msg;
				msg << "row '" << c << "' is out of rows size " << _shape[1] << "'.";
				throw std::runtime_error(msg.str());
			}

			return get<T>(r, c);
		}

        template <typename T> inline T get(size_t x, size_t y, size_t z) {
            return *reinterpret_cast<T*>(&_data[x * _strides[0] + y * _strides[1] + z * _strides[2]]);
        }

        int64_t get_as_s8(size_t x, size_t y, size_t z) {
            switch (_dtype.eformat()) {
            case EFORMAT_U1:
                return static_cast<int64_t>(get<uint8_t>(x, y, z));
            case EFORMAT_S1:
                return static_cast<int64_t>(get<int8_t>(x, y, z));
            case EFORMAT_U2:
                return static_cast<int64_t>(get<uint16_t>(x, y, z));
            case EFORMAT_S2:
                return static_cast<int64_t>(get<int16_t>(x, y, z));
            case EFORMAT_U4:
                return static_cast<int64_t>(get<uint32_t>(x, y, z));
            case EFORMAT_S4:
                return static_cast<int64_t>(get<int32_t>(x, y, z));
            case EFORMAT_S8:
                return static_cast<int64_t>(get<int64_t>(x, y, z));
            case EFORMAT_F4:
                return static_cast<int64_t>(get<float>(x, y, z));
            default:
                std::stringstream msg;
                msg << "Unsupported dtype to get as int: '" << _dtype << "'.";
                throw std::runtime_error(msg.str());
            }
        }

		/* setters templates */
		template <typename T> inline void set(size_t index, T value) {
			((T*)(&_data[index * _strides[0]]))[0] = value;
		}

		template <typename T> inline void set_s(size_t index, T value, EFormat edtype) {
			if (_ndim != 1) {
				std::stringstream msg;
				msg << "access by single index only supported for 1d array.";
				throw std::runtime_error(msg.str());
			}

			if (_dtype.eformat() != edtype) {
				std::stringstream msg;
				msg << "requested type of format '" << DType::format(edtype) << "', for field in format '" << DType::format(_dtype.eformat()) << "'.";
				throw std::runtime_error(msg.str());
			}

			if (index > _size) {
				std::stringstream msg;
				msg << "index '" << index << "' is out of range '" << _size << "'.";
				throw std::runtime_error(msg.str());
			}

			set<T>(index, value);
		}

        inline void reset(size_t index) {
            memset(_data + index * _strides[0], 0, itemsize());
        }

		template <typename T> inline void set(size_t r, size_t c, T value) {
			((T*)((&_data[r * _strides[0] + c * _strides[1]])))[0] = value;
		}
        
		template <typename T> inline void set_s(size_t r, size_t c, T value, EFormat edtype) {
			if (_ndim != 2) {
				std::stringstream msg;
				msg << "access by row, column only supported for 2d array.";
				throw std::runtime_error(msg.str());
			}

			if (_dtype.eformat() != edtype) {
				std::stringstream msg;
				msg << "incompatible format type requested, requested format '" << DType::format(edtype) <<
					"', for field of format '" << DType::format(_dtype.eformat()) << "'.";
				throw std::runtime_error(msg.str());
			}

			if (r > _shape[0]) {
				std::stringstream msg;
				msg << "row '" << r << "' is out of rows size " << _shape[0] << "'.";
				throw std::runtime_error(msg.str());
			}

			if (c > _shape[1]) {
				std::stringstream msg;
				msg << "row '" << c << "' is out of rows size " << _shape[1] << "'.";
				throw std::runtime_error(msg.str());
			}

			set<T>(r, c, value);
		}

        inline void reset(size_t r, size_t c) {
            memset(_data + r * _strides[0] + c * _strides[1], 0, itemsize());
        }

        template <typename T> inline void set(size_t x, size_t y, size_t z, T value) {
            *reinterpret_cast<T*>(_data + x * _strides[0] + y * _strides[1] + z * _strides[2]) = value;
        }

        void set_as_s8(size_t x, size_t y, size_t z, int64_t value) {
            switch (_dtype.eformat()) {
            case EFORMAT_U1:
                set(x, y, z, static_cast<uint8_t>(value));
            case EFORMAT_S1:
                set(x, y, z, static_cast<int8_t>(value));
            case EFORMAT_U2:
                set(x, y, z, static_cast<uint16_t>(value));
            case EFORMAT_S2:
                set(x, y, z, static_cast<int16_t>(value));
            case EFORMAT_U4:
                set(x, y, z, static_cast<uint32_t>(value));
            case EFORMAT_S4:
                set(x, y, z, static_cast<int32_t>(value));
            case EFORMAT_S8:
                set(x, y, z, value);
            default:
                std::stringstream msg;
                msg << "Unsupported dtype to set as int: '" << DType::format(_dtype.eformat()) << "'.";
                throw std::runtime_error(msg.str());
            }
        }

        inline void reset(size_t x, size_t y, size_t z) {
            memset(_data + x * _strides[0] + y * _strides[1] + z * _strides[2], 0, itemsize());
        }

		/* getters */
		const DType dtype();
		const size_t itemsize();
		const size_t size();
		const size_t nbytes();
		const size_t ndim();
		const size_t* shape();
		const size_t* strides();
		const size_t offset();
		const EMEMManagement memmng();
		const char* data();
		const NDArray* base();
		const bool is_base();
		const bool is_mem_owner();
		const size_t refcount();

		/* memcpy getters */
		void get_shape(size_t* shape, size_t ndim);

		void get_strides(size_t* strides, size_t ndim);

		/* safe getters */
		const std::vector<size_t> shape_s();
		const std::vector<size_t> strides_s();

		NDArray* at(std::string selector);

		NDArray* at(size_t index);

		// {{templater:getters.ejs.cpp}} autogenerated code segment
		/* u1 getters */
		uint8_t get_u1(size_t index);
		
		uint8_t get_u1_s(size_t index);
		
		uint8_t get_u1(size_t r, size_t c);
		
		uint8_t get_u1_s(size_t r, size_t c);
		
		/* u2 getters */
		uint16_t get_u2(size_t index);
		
		uint16_t get_u2_s(size_t index);
		
		uint16_t get_u2(size_t r, size_t c);
		
		uint16_t get_u2_s(size_t r, size_t c);
		
		/* u4 getters */
		uint32_t get_u4(size_t index);
		
		uint32_t get_u4_s(size_t index);
		
		uint32_t get_u4(size_t r, size_t c);
		
		uint32_t get_u4_s(size_t r, size_t c);
		
		/* u8 getters */
		uint64_t get_u8(size_t index);
		
		uint64_t get_u8_s(size_t index);
		
		uint64_t get_u8(size_t r, size_t c);
		
		uint64_t get_u8_s(size_t r, size_t c);
		
		/* s1 getters */
		int8_t get_s1(size_t index);
		
		int8_t get_s1_s(size_t index);
		
		int8_t get_s1(size_t r, size_t c);
		
		int8_t get_s1_s(size_t r, size_t c);
		
		/* s2 getters */
		int16_t get_s2(size_t index);
		
		int16_t get_s2_s(size_t index);
		
		int16_t get_s2(size_t r, size_t c);
		
		int16_t get_s2_s(size_t r, size_t c);
		
		/* s4 getters */
		int32_t get_s4(size_t index);
		
		int32_t get_s4_s(size_t index);
		
		int32_t get_s4(size_t r, size_t c);
		
		int32_t get_s4_s(size_t r, size_t c);
		
		/* s8 getters */
		int64_t get_s8(size_t index);
		
		int64_t get_s8_s(size_t index);
		
		int64_t get_s8(size_t r, size_t c);
		
		int64_t get_s8_s(size_t r, size_t c);
		
		/* f4 getters */
		float get_f4(size_t index);
		
		float get_f4_s(size_t index);
		
		float get_f4(size_t r, size_t c);
		
		float get_f4_s(size_t r, size_t c);
		
		/* f8 getters */
		double get_f8(size_t index);
		
		double get_f8_s(size_t index);
		
		double get_f8(size_t r, size_t c);
		
		double get_f8_s(size_t r, size_t c);		
		 
		// {{templater:end}}

		/* bitfield getter*/
		uint32_t bf_get(int index);
		
		// {{templater:setters.ejs.cpp}} autogenerated code segment
		/* u1 setters */
		void set_u1(size_t index, uint8_t value);
		
		void set_u1_s(size_t index, uint8_t value);
		
		void set_u1(size_t r, size_t c, uint8_t value);
		
		void set_u1_s(size_t r, size_t c, uint8_t value);
		
		/* u2 setters */
		void set_u2(size_t index, uint16_t value);
		
		void set_u2_s(size_t index, uint16_t value);
		
		void set_u2(size_t r, size_t c, uint16_t value);
		
		void set_u2_s(size_t r, size_t c, uint16_t value);
		
		/* u4 setters */
		void set_u4(size_t index, uint32_t value);
		
		void set_u4_s(size_t index, uint32_t value);
		
		void set_u4(size_t r, size_t c, uint32_t value);
		
		void set_u4_s(size_t r, size_t c, uint32_t value);
		
		/* u8 setters */
		void set_u8(size_t index, uint64_t value);
		
		void set_u8_s(size_t index, uint64_t value);
		
		void set_u8(size_t r, size_t c, uint64_t value);
		
		void set_u8_s(size_t r, size_t c, uint64_t value);
		
		/* s1 setters */
		void set_s1(size_t index, int8_t value);
		
		void set_s1_s(size_t index, int8_t value);
		
		void set_s1(size_t r, size_t c, int8_t value);
		
		void set_s1_s(size_t r, size_t c, int8_t value);
		
		/* s2 setters */
		void set_s2(size_t index, int16_t value);
		
		void set_s2_s(size_t index, int16_t value);
		
		void set_s2(size_t r, size_t c, int16_t value);
		
		void set_s2_s(size_t r, size_t c, int16_t value);
		
		/* s4 setters */
		void set_s4(size_t index, int32_t value);
		
		void set_s4_s(size_t index, int32_t value);
		
		void set_s4(size_t r, size_t c, int32_t value);
		
		void set_s4_s(size_t r, size_t c, int32_t value);
		
		/* s8 setters */
		void set_s8(size_t index, int64_t value);
		
		void set_s8_s(size_t index, int64_t value);
		
		void set_s8(size_t r, size_t c, int64_t value);
		
		void set_s8_s(size_t r, size_t c, int64_t value);
		
		/* f4 setters */
		void set_f4(size_t index, float value);
		
		void set_f4_s(size_t index, float value);
		
		void set_f4(size_t r, size_t c, float value);
		
		void set_f4_s(size_t r, size_t c, float value);
		
		/* f8 setters */
		void set_f8(size_t index, double value);
		
		void set_f8_s(size_t index, double value);
		
		void set_f8(size_t r, size_t c, double value);
		
		void set_f8_s(size_t r, size_t c, double value);
		
		 
		// {{templater:end}}

		/* as double getter */
		double to_d(size_t index);

		/* filters */
		
		/*
		* @description: apply filter in filter attr on arr
		* @param fattr filter attributes
		*/
		void ApplyFilter(const FilterAttr* fattr);

	};

}

template class INVZ_API std::unique_ptr<ndarray::DType[]>;
template class INVZ_API std::unique_ptr<ndarray::bitmap_helpers::IBitFieldGetter>;

#endif // __NDARRAY_H__