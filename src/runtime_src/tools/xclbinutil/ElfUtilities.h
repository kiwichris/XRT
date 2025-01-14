/**
 * Copyright (C) 2021-2022 Xilinx, Inc
 *
 * Licensed under the Apache License, Version 2.0 (the "License"). You may
 * not use this file except in compliance with the License. A copy of the
 * License is located at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations
 * under the License.
 */

#ifndef __ElfUtilities_h_

// Include files

// Please keep this list to a minimum
#include <boost/property_tree/ptree.hpp>
#include <string>
#include <vector>

namespace XclBinUtilities {

void dataMineExportedFunctionsDWARF(const std::string& elfLibrary, boost::property_tree::ptree &ptFunctions);

};

#endif
