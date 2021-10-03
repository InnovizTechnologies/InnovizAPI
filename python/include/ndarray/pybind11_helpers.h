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
/****************************/
/* NDArray pybdin11 helpers */
/****************************/

#ifndef __NDARRAY_PYBIND11_HELPERS_H__
#define __NDARRAY_PYBIND11_HELPERS_H__

#include <pybind11/numpy.h>

#include "ndarray.h"

pybind11::array py_array_from_dtype(std::vector<size_t> shape, ndarray::DType dtype, void* data = nullptr);


#endif // __NDARRAY_PYBIND11_HELPERS_H__
