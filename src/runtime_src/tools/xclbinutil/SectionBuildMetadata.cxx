/**
 * Copyright (C) 2018, 2020, 2022 Xilinx, Inc
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

#include "SectionBuildMetadata.h"

#include "version.h" // Globally included from main
#include "XclBinUtilities.h"
#include <boost/functional/factory.hpp>
#include <boost/property_tree/json_parser.hpp>

namespace XUtil = XclBinUtilities;

// Static Variables / Classes
SectionBuildMetadata::init SectionBuildMetadata::initializer;

SectionBuildMetadata::init::init() 
{ 
  auto sectionInfo = std::make_unique<SectionInfo>(BUILD_METADATA, "BUILD_METADATA", boost::factory<SectionBuildMetadata*>()); 
  sectionInfo->nodeName = "build_metadata";

  addSectionType(std::move(sectionInfo));
}

void 
SectionBuildMetadata::marshalToJSON(char* _pDataSection, 
                                    unsigned int _sectionSize, 
                                    boost::property_tree::ptree& _ptree) const
{
    XUtil::TRACE("");
    XUtil::TRACE("Extracting: BUILD_METADATA");

    std::unique_ptr<unsigned char> memBuffer(new unsigned char[_sectionSize + 1]);
    memcpy((char *) memBuffer.get(), _pDataSection, _sectionSize);
    memBuffer.get()[_sectionSize] = '\0';

    std::stringstream ss((char*) memBuffer.get());

    XUtil::TRACE_BUF("BUILD_METADATA", (const char *) memBuffer.get(), _sectionSize+1);
    try {
      boost::property_tree::ptree pt;
      boost::property_tree::read_json(ss, pt);
      boost::property_tree::ptree &buildMetaData = pt.get_child("build_metadata");
      _ptree.add_child("build_metadata", buildMetaData);
    } catch (const std::exception & e) {
      std::string msg("ERROR: Bad JSON format detected while marshaling build metadata (");
      msg += e.what();
      msg += ").";
      throw std::runtime_error(msg);
    }
}

void 
SectionBuildMetadata::marshalFromJSON(const boost::property_tree::ptree& _ptSection, 
                                      std::ostringstream& _buf) const
{
   XUtil::TRACE("BUILD_METADATA");
   boost::property_tree::ptree ptWritable = _ptSection;
   ptWritable.put("build_metadata.xclbin.packaged_by.name", "xclbinutil");
   ptWritable.put("build_metadata.xclbin.packaged_by.version", xrt_build_version); 
   ptWritable.put("build_metadata.xclbin.packaged_by.hash", xrt_build_version_hash);
   ptWritable.put("build_metadata.xclbin.packaged_by.time_stamp", xrt_build_version_date_rfc);
   boost::property_tree::write_json(_buf, ptWritable, false );
}

bool 
SectionBuildMetadata::doesSupportAddFormatType(FormatType _eFormatType) const
{
  if ((_eFormatType == FormatType::JSON) ||
      (_eFormatType == FormatType::RAW)){
    return true;
  }
  return false;
}

bool 
SectionBuildMetadata::doesSupportDumpFormatType(FormatType _eFormatType) const
{
    if ((_eFormatType == FormatType::JSON) ||
        (_eFormatType == FormatType::HTML))
    {
      return true;
    }

    return false;
}

