/**
 * Copyright (C) 2018 - 2022 Xilinx, Inc
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

#include "SectionPartitionMetadata.h"

#include "DTC.h"
#include "XclBinUtilities.h"
#include <algorithm>
#include <boost/algorithm/string/replace.hpp>
#include <boost/format.hpp>
#include <boost/functional/factory.hpp>
#include <boost/property_tree/json_parser.hpp>

namespace XUtil = XclBinUtilities;

// Static Variables / Classes
SectionPartitionMetadata::init SectionPartitionMetadata::initializer;

SectionPartitionMetadata::init::init() 
{ 
  auto sectionInfo = std::make_unique<SectionInfo>(PARTITION_METADATA, "PARTITION_METADATA", boost::factory<SectionPartitionMetadata *>()); 
  sectionInfo->nodeName = "partition_metadata";

  addSectionType(std::move(sectionInfo));
}


template <typename T>
std::vector<T> as_vector(boost::property_tree::ptree const& pt, 
                         boost::property_tree::ptree::key_type const& key)
{
    std::vector<T> r;
    for (auto& item : pt.get_child(key))
        r.push_back(item.second.get_value<T>());
    return r;
}


// Variable name to data size mapping table
const FDTProperty::PropertyNameFormat SectionPartitionMetadata::m_propertyNameFormat = {
  { "__INFO", FDTProperty::DF_sz },
  { "alias_name", FDTProperty::DF_sz },
  { "compatible", FDTProperty::DF_asz },
  { "firmware_branch_name", FDTProperty::DF_sz },
  { "firmware_product_name", FDTProperty::DF_sz },
  { "firmware_version_major", FDTProperty::DF_u32 },
  { "firmware_version_minor", FDTProperty::DF_u32 },
  { "firmware_version_revision", FDTProperty::DF_u32 },
  { "interface_uuid", FDTProperty::DF_sz },
  { "interrupt_alias", FDTProperty::DF_asz },
  { "interrupts", FDTProperty::DF_au32 },
  { "logic_uuid", FDTProperty::DF_sz },
  { "major", FDTProperty::DF_u32 },
  { "minor", FDTProperty::DF_u32 },
  { "pcie_bar_mapping", FDTProperty::DF_u32},
  { "pcie_physical_function", FDTProperty::DF_u32},
  { "range", FDTProperty::DF_u64 },
  { "reg", FDTProperty::DF_au64 },
  { "resolves_interface_uuid", FDTProperty::DF_sz },
};

// -- Helper transformation helper functions ---------------------------------

/**
 * Transformation function 
 *  
 * @param _ptOriginal The original property tree item 
 * @param _ptTransormed The resulting transformed property tree 
 */
typedef void (SubNodeFunc)(const boost::property_tree::ptree& _ptOriginal, boost::property_tree::ptree & _ptTransformed);


/** 
 * The subNode (child graph) to search for. 
 *  
 * @param _nodeName  The node name to search for 
 * @param _bRequired Indicates if this value is required.  If 
 *                   not found an error exception will be thrown
 * @param _nodeFunction The function to be called if the node is 
 *                      found.
 * @param _ptOriginal The property to performed the search in 
 * @param _ptTransformed The property to that was transformed
 */  
void 
SchemaTransform_subNode( const std::string & _nodeName,
                         bool _bRequired,
                         SubNodeFunc & _nodeFunction,
                         const boost::property_tree::ptree & _ptOriginal,
                         boost::property_tree::ptree & _ptTransformed )
{
  // Find the node
  boost::property_tree::ptree::const_assoc_iterator iter = _ptOriginal.find(_nodeName);

  if (iter == _ptOriginal.not_found()) {
    if (_bRequired == true) {
      std::string errMsg = "Error: Missing expected partition_metadata node entry: '" + _nodeName + "'";
      throw std::runtime_error(errMsg);
    }
    return;
  }

  // Get the nodes child property tree and call the helper function.
  boost::property_tree::ptree ptChildNode;
  _nodeFunction(iter->second, ptChildNode);
  _ptTransformed.add_child(_nodeName, ptChildNode);
}

/**
 * The property name to find and transform. 
 *  
 * @param _valueName The property name to search for
 * @param _bRequired Indicates if this property is must to have
 * @param _ptOriginal The property tree to submit
 * @param _ptTransformed The transformed property tree
 */
void 
SchemaTransform_nameValue( const std::string & _valueName,
                           const std::string & _transformedName,
                           bool _bRequired,
                           const boost::property_tree::ptree & _ptOriginal,
                           boost::property_tree::ptree & _ptTransformed )
{
  // Look for the property 
  boost::property_tree::ptree::const_assoc_iterator iter = _ptOriginal.find(_valueName);
  if (iter == _ptOriginal.not_found()) {
    if (_bRequired == true) {
      std::string errMsg = "Error: Missing expected partition_metadata entry: '" + _valueName + "'";
      throw std::runtime_error(errMsg);
    }
    return;
  }

  // Copy the name and value to the transform property tree
  const std::string & sValue = _ptOriginal.get<std::string>(_valueName.c_str());

  if (_transformedName.empty()) {
    _ptTransformed.put(_valueName.c_str(), sValue.c_str());
  } else {
    _ptTransformed.put(_transformedName.c_str(), sValue.c_str());
  }

}

// -- Universal Schema Functions ----------------------------------------------


/**
 * Helper method to transform the firmware section
 * 
 * @param _ptOriginal The original firmware property tree
 * @param _ptTransformed The transform firmware property tree
 */
void 
SchemaTransformUniversal_firmware( const boost::property_tree::ptree& _ptOriginal,
                                   boost::property_tree::ptree & _ptTransformed)
{
  SchemaTransform_nameValue("firmware_product_name", "", true  /*required*/, _ptOriginal, _ptTransformed);
  SchemaTransform_nameValue("firmware_branch_name", "", true  /*required*/, _ptOriginal, _ptTransformed);
  SchemaTransform_nameValue("firmware_version_major", "", true  /*required*/, _ptOriginal, _ptTransformed);
  SchemaTransform_nameValue("firmware_version_minor", "", false  /*required*/, _ptOriginal, _ptTransformed);
  SchemaTransform_nameValue("firmware_version_revision", "", false  /*required*/, _ptOriginal, _ptTransformed);
}

/**
 * Transform the schema version
 *
 * @param _ptOriginal The original version property tree
 * @param _ptTransformed The transformed version property tree
 */
void 
SchemaTransformUniversal_schema_version( const boost::property_tree::ptree& _ptOriginal,
                                         boost::property_tree::ptree& _ptTransformed)
{
  // DRC check for known values
  if (_ptOriginal.find("major") == _ptOriginal.not_found()) {
    throw std::runtime_error("Error: schema_version.major key not found.");
  }

  if (_ptOriginal.find("minor") == _ptOriginal.not_found()) {
    throw std::runtime_error("Error: schema_version.minor key not found.");
  }

  std::string sMajor = _ptOriginal.get<std::string>("major");
  int major = sMajor.empty() ? -1 : std::stoi(sMajor.c_str(), 0, 16);

  std::string sMinor = _ptOriginal.get<std::string>("minor");
  int minor = sMinor.empty() ? -1 : std::stoi(sMinor.c_str(), 0 , 16);

  if ((major != 1) || (minor != 0)) {
    if (sMajor.empty())
      sMajor = "<undefined>";

    if (sMinor.empty())
      sMinor = "<undefined>";

    std::string errMsg = "Error: Unsupported schema_version: Major: '" + sMajor + "', Minor: '" + sMinor + "'";
    throw std::runtime_error(errMsg);
  }

  // All is clear, pass the schema on
  _ptTransformed.put("major", sMajor.c_str());
  _ptTransformed.put("minor", sMinor.c_str());
}

// -- Transform to DTC schema functions ---------------------------------------

/**
 * Helper method to transform a generic node
 * 
 * @param _ptOriginal The original firmware property tree
 * @param _ptTransformed The transform firmware property tree
 */
void 
SchemaTransformToDTC_pcie( const boost::property_tree::ptree& _ptOriginal,
                           boost::property_tree::ptree & _ptTransformed)
{
  boost::property_tree::ptree ptEmpty;

  // --- Bars ---
  boost::property_tree::ptree ptBarOriginal;
  ptBarOriginal = _ptOriginal.get_child("bars", ptEmpty);

  if (ptBarOriginal.empty()) 
    throw std::runtime_error("Error: pcie.bars[] list not found.");

  unsigned int count = 0;
  boost::property_tree::ptree ptBarArray;
  for (auto barEntryOrig : ptBarOriginal) {
    boost::property_tree::ptree ptBarEntry;
    boost::property_tree::ptree _ptBarOriginalEntry = barEntryOrig.second;

    SchemaTransform_nameValue("pcie_base_address_register", "pcie_bar_mapping",  true  /*required*/, _ptBarOriginalEntry, ptBarEntry);
    SchemaTransform_nameValue("pcie_physical_function", "", true  /*required*/, _ptBarOriginalEntry , ptBarEntry);

    // -- Transform 'offset' and 'range' to 'reg'
    if ((_ptBarOriginalEntry.find("offset") != _ptBarOriginalEntry.not_found()) &&
      (_ptBarOriginalEntry.find("range") != _ptBarOriginalEntry.not_found())) {
      boost::property_tree::ptree ptRegOffset;
      ptRegOffset.put("", _ptBarOriginalEntry.get<std::string>("offset").c_str());
      boost::property_tree::ptree ptRegRange;

      ptRegRange.put("", _ptBarOriginalEntry.get<std::string>("range").c_str());
      boost::property_tree::ptree ptReg;

      ptReg.push_back(std::make_pair("", ptRegOffset));
      ptReg.push_back(std::make_pair("", ptRegRange));
      ptBarEntry.add_child("reg", ptReg);
    }

    ptBarArray.add_child(std::to_string(count++), ptBarEntry);
  }

  _ptTransformed.add_child("bars", ptBarArray);
}

void 
SchemaTransformToDTC_interrupt_endpoint( const std::string _sEndPointName,
                                         const boost::property_tree::ptree& _ptOriginal,
                                         boost::property_tree::ptree & _ptTransformed)
{
  // -- Transform array to 'interrupts' array
  // Validate the count id is correct
  if (_ptOriginal.size() % 2 != 0) {
    auto errMsg =  boost::format("Error: The interrupt count (%d) for the interrupt '%s' needs to be a paired (e.g. even) set.") % _ptOriginal.size() % _sEndPointName;
    throw std::runtime_error(errMsg.str());
  }

  // -- Move the array down to an interrupt array
  _ptTransformed.put("alias_name", _sEndPointName.c_str());
  _ptTransformed.add_child("interrupts", _ptOriginal);
}

void 
SchemaTransformToDTC_interrupt_mapping( const boost::property_tree::ptree& _ptOriginal,
                                        boost::property_tree::ptree & _ptTransformed)
{ 
  // Look at each entry
  unsigned int indexCount = 0;
  for (auto endpoint : _ptOriginal) {
    boost::property_tree::ptree ptEndpoint;

    SchemaTransformToDTC_interrupt_endpoint(endpoint.first, endpoint.second, ptEndpoint);
    std::string entry = std::string("@") + std::to_string(indexCount++);
    _ptTransformed.add_child(entry.c_str(), ptEndpoint);
  }
}

/**
 * Helper function to transform the interfaces
 * 
 * @param _ptOriginal The original set of interfaces
 * @param _ptTransformed The transform set of interfaces
 */
void 
SchemaTransformToDTC_interfaces( const boost::property_tree::ptree& _ptOriginal,
                                     boost::property_tree::ptree & _ptTransformed)
{
  unsigned int index = 0;
  for (auto interface : _ptOriginal) {
    boost::property_tree::ptree ptInterface;

    SchemaTransform_nameValue("interface_uuid", "", true  /*required*/, interface.second, ptInterface);

    std::string sIndex = (boost::format("@%d") % index++).str();
    _ptTransformed.add_child(sIndex.c_str(), ptInterface);
  }
}


/**
 * Transform the addressable endpoint to the DTC schema format
 * 
 * @param _sEndPointName The name of the endpoint
 * @param _ptOriginal The original endpoint node
 * @param _ptTransformed The transformed endpoint node
 */
void 
SchemaTransformToDTC_addressable_endpoint( const std::string _sEndPointName,
                                           const boost::property_tree::ptree& _ptOriginal,
                                           boost::property_tree::ptree & _ptTransformed)
{
  // -- Transform 'offset' and 'range' to 'reg'
  if ((_ptOriginal.find("offset") != _ptOriginal.not_found()) &&
      (_ptOriginal.find("range") != _ptOriginal.not_found())) {
    boost::property_tree::ptree ptRegOffset;
    ptRegOffset.put("", _ptOriginal.get<std::string>("offset").c_str());

    boost::property_tree::ptree ptRegRange;
    ptRegRange.put("", _ptOriginal.get<std::string>("range").c_str());

    boost::property_tree::ptree ptReg;
    ptReg.push_back(std::make_pair("", ptRegOffset));
    ptReg.push_back(std::make_pair("", ptRegRange));

    _ptTransformed.add_child("reg", ptReg);
  }

  // -- Get 'pcie_physical_function'
  SchemaTransform_nameValue("pcie_physical_function", "", false  /* required*/, _ptOriginal, _ptTransformed);

  // -- Transform 'register_abstraction_name' to 'compatible'
  if (_ptOriginal.find("register_abstraction_name") != _ptOriginal.not_found()) {
    std::string sAbstractName = _ptOriginal.get<std::string>("register_abstraction_name");

    const std::string delimiters = ":";      // Our delimiter

    // Working variables
    std::string::size_type pos = 0;
    std::string::size_type lastPos = 0;
    std::vector<std::string> tokens;

    // Parse the string until the entire string has been parsed
    while( lastPos < (sAbstractName.length() + 1))
    {
      pos = sAbstractName.find_first_of(delimiters, lastPos);

      if ( pos == std::string::npos ) {
        pos = sAbstractName.length();
      }

      std::string token = sAbstractName.substr(lastPos, pos-lastPos);
      tokens.push_back(token);
      lastPos = pos + 1;
    }

    if (tokens.size() != 4) {
      throw std::runtime_error("Error: 'addressable_endpoints." + _sEndPointName + ".register_abstraction_name' is invalid: '" + sAbstractName + "'");
    }

    // Example Format: "xilinx.com,reg_abs-xdma_msix-1.0"
    std::string sCompLine1 = tokens[0] + "," + tokens[1] + "-" + tokens[2] + "-" + tokens[3];
    boost::property_tree::ptree ptCompLine1;
    ptCompLine1.put("", sCompLine1.c_str());

    boost::property_tree::ptree ptCompLine2;
    ptCompLine2.put("", tokens[2].c_str());

    boost::property_tree::ptree ptCompatible;
    ptCompatible.push_back(std::make_pair("", ptCompLine1));
    ptCompatible.push_back(std::make_pair("", ptCompLine2));

    _ptTransformed.add_child("compatible", ptCompatible);
  }

  // -- Transform 'msix_interrupt_start_index' and 'msix_interrupt_end_index' to 'interrupts'
  {
    // DRC checks
    if ((_ptOriginal.find("msix_interrupt_start_index") == _ptOriginal.not_found()) &&
        (_ptOriginal.find("msix_interrupt_end_index") != _ptOriginal.not_found())) {
      throw std::runtime_error("Error: 'addressable_endpoints." + _sEndPointName + ".msix_interrupt_start_index' key not found.");
    }

    if ((_ptOriginal.find("msix_interrupt_start_index") != _ptOriginal.not_found()) &&
        (_ptOriginal.find("msix_interrupt_end_index") == _ptOriginal.not_found()) ){
      throw std::runtime_error("Error: 'addressable_endpoints." + _sEndPointName + ".msix_interrupt_end_index' key not found.");
    }

    // Get the data
    if ((_ptOriginal.find("msix_interrupt_start_index") != _ptOriginal.not_found()) &&
        (_ptOriginal.find("msix_interrupt_end_index") != _ptOriginal.not_found())) {

      boost::property_tree::ptree ptStartRange;
      std::string sInterruptStart = _ptOriginal.get<std::string>("msix_interrupt_start_index");
      ptStartRange.put("", sInterruptStart.c_str());

      boost::property_tree::ptree ptEndRange;
      std::string sInterruptEnd = _ptOriginal.get<std::string>("msix_interrupt_end_index");
      ptEndRange.put("", sInterruptEnd.c_str());

      boost::property_tree::ptree ptInterrupts;
      ptInterrupts.push_back(std::make_pair("", ptStartRange));
      ptInterrupts.push_back(std::make_pair("", ptEndRange));


      _ptTransformed.add_child("interrupts", ptInterrupts);
    }
  }

  // -- Get the 'firmware" information
  SchemaTransform_subNode( "firmware", false /*required*/, 
                           SchemaTransformUniversal_firmware,
                           _ptOriginal, _ptTransformed);

  // -- Get the 'pcie_base_address_register' (optional)
  SchemaTransform_nameValue("pcie_base_address_register", "pcie_bar_mapping", false  /*required*/, _ptOriginal, _ptTransformed);

  // -- Get the 'interrupt_alias' (optional)
  if (_ptOriginal.find("interrupt_alias") != _ptOriginal.not_found()) 
      _ptTransformed.add_child("interrupt_alias", _ptOriginal.get_child("interrupt_alias"));

  // -- Get the "interrupt_mapping" (optional)
  SchemaTransform_subNode( "interrupt_mapping", false /*required*/, 
                           SchemaTransformToDTC_interrupt_mapping,
                           _ptOriginal, _ptTransformed);

}

/**
 * Transforms all of the addressable_endpoints array elements.
 * 
 * @param _ptOriginal The original addressable_endpoints 
 *                    property tree
 * @param _ptTransformed The transformed addressable_endpoints 
 *                       property tree.
 */
void 
SchemaTransformToDTC_addressable_endpoints( const boost::property_tree::ptree& _ptOriginal,
                                            boost::property_tree::ptree & _ptTransformed)
{ 
  // Look at each entry
  for (auto endpoint : _ptOriginal) {
    boost::property_tree::ptree ptEndpoint;

    SchemaTransformToDTC_addressable_endpoint(endpoint.first, endpoint.second, ptEndpoint);
    _ptTransformed.add_child(endpoint.first, ptEndpoint);
  }
}

void
SchemaTransformToDTC_partition_info ( const boost::property_tree::ptree& _ptOriginal,
                                           boost::property_tree::ptree & _ptTransformed)
{
  // Convert the entire JSON tree under the __INFO node to be a DTB string entry.
  std::ostringstream buf;

  boost::property_tree::write_json(buf, _ptOriginal, false);
  _ptTransformed.put("__INFO", buf.str());
}

/**
 * Transform the original partition_metadata format to the DTC 
 * format. 
 * 
 * @param _ptOriginal The original property tree
 * @param _ptTransformed The transformed property tree
 */
void 
SchemaTransformToDTC_root( const boost::property_tree::ptree & _ptOriginal, 
                                boost::property_tree::ptree & _ptTransformed)
{
  SchemaTransform_subNode( "schema_version", true /*required*/, 
                           SchemaTransformUniversal_schema_version,
                           _ptOriginal, _ptTransformed);

  SchemaTransform_nameValue("logic_uuid", "", false  /*required*/, _ptOriginal, _ptTransformed);

  SchemaTransform_subNode( "interfaces", true /*required*/, 
                           SchemaTransformToDTC_interfaces,
                           _ptOriginal, _ptTransformed);

  SchemaTransform_subNode( "pcie", false /*required*/, 
                           SchemaTransformToDTC_pcie,
                           _ptOriginal, _ptTransformed);

  SchemaTransform_subNode( "addressable_endpoints", false /*required*/, 
                           SchemaTransformToDTC_addressable_endpoints,
                           _ptOriginal, _ptTransformed);

  SchemaTransform_subNode( "partition_info", false /*required*/,
                           SchemaTransformToDTC_partition_info,
                           _ptOriginal, _ptTransformed);
}


// -- Transform to partition metadata schema functions -----------------------

/**
 * Helper method to transform a generic node
 * 
 * @param _ptOriginal The original firmware property tree
 * @param _ptTransformed The transform firmware property tree
 */
void 
SchemaTransformToPM_pcie( const boost::property_tree::ptree& _ptOriginal,
                          boost::property_tree::ptree & _ptTransformed)
{
  boost::property_tree::ptree ptEmpty;

  // --- Bars ---
  boost::property_tree::ptree ptBarOriginal;
  ptBarOriginal = _ptOriginal.get_child("bars", ptEmpty);

  if (ptBarOriginal.empty()) 
    throw std::runtime_error("Error: pcie.bars[] list not found.");

  boost::property_tree::ptree ptBarArray;
  for (auto barEntryOrig : ptBarOriginal) {
    boost::property_tree::ptree ptBarEntry;
    boost::property_tree::ptree _ptBarOriginalEntry = barEntryOrig.second;

    SchemaTransform_nameValue("pcie_bar_mapping", "pcie_base_address_register", true  /*required*/, _ptBarOriginalEntry, ptBarEntry);
    SchemaTransform_nameValue("pcie_physical_function", "", true  /*required*/, _ptBarOriginalEntry, ptBarEntry);

    // -- Transform 'reg' to 'offset' and 'range'
    if (_ptBarOriginalEntry.find("reg") != _ptBarOriginalEntry.not_found()) {
      auto regVector = as_vector<std::string>(_ptBarOriginalEntry, "reg");

      if (regVector.size() == 2) {
        ptBarEntry.put("offset", regVector[0].c_str());
        ptBarEntry.put("range", regVector[1].c_str());
      }
    }

    ptBarArray.push_back(std::make_pair("", ptBarEntry));
  }

  _ptTransformed.add_child("bars", ptBarArray);
}


void 
SchemaTransformToPM_interrupt_endpoint( const boost::property_tree::ptree& _ptOriginal,
                                        std::string & _sEndPointName,
                                        boost::property_tree::ptree & _ptTransformed)
{
  _sEndPointName = _ptOriginal.get<std::string>("alias_name", "");
  if (_sEndPointName.empty()) 
    throw std::runtime_error("Error: interrupt addressable_endpoints property 'alias_name' is either not defined or empty.");

  // -- Transform 'interrupts" to the addressable_endpoint array format
  if (_ptOriginal.find("interrupts") != _ptOriginal.not_found()) {
    auto interruptVector = as_vector<std::string>(_ptOriginal, "interrupts");

    if ((interruptVector.size() % 2) != 0) 
      throw std::runtime_error("Error: 'addressable_endpoints." + _sEndPointName + ".interrupts' doesn't have and even set of items.");
    
    // -- Push the array back to interrupt key name
    for (const std::string & interruptValue : interruptVector) {
      boost::property_tree::ptree ptValue;
      ptValue.put("", interruptValue.c_str());
      _ptTransformed.push_back(std::make_pair("", ptValue));
    }
  }
}


void 
SchemaTransformToPM_interrupt_mapping( const boost::property_tree::ptree& _ptOriginal,
                                       boost::property_tree::ptree & _ptTransformed)
{ 
  // Look at each entry
  for (auto endpoint : _ptOriginal) {
    boost::property_tree::ptree ptEndpoint;
    std::string endpointName;

    SchemaTransformToPM_interrupt_endpoint(endpoint.second, endpointName, ptEndpoint);
    _ptTransformed.add_child(endpointName, ptEndpoint);
  }
}

/**
 * Helper function to transform the interfaces
 * 
 * @param _ptOriginal The original set of interfaces
 * @param _ptTransformed The transform set of interfaces
 */
void 
SchemaTransformToPM_interfaces( const boost::property_tree::ptree& _ptOriginal,
                                boost::property_tree::ptree & _ptTransformed)
{
  for (auto interface : _ptOriginal) {
    boost::property_tree::ptree ptInterface;

    SchemaTransform_nameValue("interface_uuid", "", true  /*required*/, interface.second, ptInterface);
    _ptTransformed.push_back(std::make_pair("", ptInterface));
  }
}

/**
 * Transform the addressable endpoint to the DTB schema format
 * 
 * @param _sEndPointName The name of the endpoint
 * @param _ptOriginal The original endpoint node
 * @param _ptTransformed The transformed endpoint node
 */
void 
SchemaTransformToPM_addressable_endpoint( const std::string _sEndPointName,
                                          const boost::property_tree::ptree& _ptOriginal,
                                          boost::property_tree::ptree & _ptTransformed)
{
  // -- Transform 'reg' to 'offset' and 'range'
  if (_ptOriginal.find("reg") != _ptOriginal.not_found()) {
    auto regVector = as_vector<std::string>(_ptOriginal, "reg");
    
    if (regVector.size() != 2) {
      throw std::runtime_error("Error: 'addressable_endpoints." + _sEndPointName + ".reg' doesn't have 2 items.");
    }
   
    _ptTransformed.put("offset", regVector[0].c_str());
    _ptTransformed.put("range", regVector[1].c_str());
  }

  // -- Get 'pcie_physical_function'
  SchemaTransform_nameValue("pcie_physical_function", "", false  /*required*/, _ptOriginal, _ptTransformed);

  // -- Transform 'compatible' to 'register_abstraction_name'
  // Example: (new)  xilinx.com,reg_abs-xdma_msix-1.0 xdma_msix
  //          (old)  xilinx.com,reg_abs-1.0           xdma_msix
  if (_ptOriginal.find("compatible") != _ptOriginal.not_found()) {
    auto compatableVector = as_vector<std::string>(_ptOriginal, "compatible");
    if (compatableVector.size() != 2) {
      throw std::runtime_error("Error: 'addressable_endpoints." + _sEndPointName + ".compatible' doesn't have 2 items.");
    }

    std::string registerAbstraction = compatableVector[0];
    std::string searchStr = ",";
    registerAbstraction.replace(registerAbstraction.find(searchStr),searchStr.length(),":");
    // Example: (new)  xilinx.com:reg_abs-xdma_msix-1.0 xdma_msix
    //          (old)  xilinx.com:reg_abs-1.0           xdma_msix

    searchStr = "-";
    std::string replaceStr = ":";                 // Assume new syntax

    // Determine if an older format was used 
    size_t dashCount = std::count(registerAbstraction.begin(), registerAbstraction.end(), '-');
    if (dashCount == 1) 
      replaceStr = ":" + compatableVector[1] + ":";

    boost::replace_all(registerAbstraction,searchStr,replaceStr);
    // Example:  xilinx.com:reg_abs:xdma_msix:1.0 xdma_msix

    _ptTransformed.put("register_abstraction_name", registerAbstraction.c_str());
  }

  // -- Transform 'interrupts" to 'msix_interrupt_start_index' and 'msix_interrupt_end_index'
  if (_ptOriginal.find("interrupts") != _ptOriginal.not_found()) {
    auto interruptVector = as_vector<std::string>(_ptOriginal, "interrupts");

    if (interruptVector.size() != 2) {
      throw std::runtime_error("Error: 'addressable_endpoints." + _sEndPointName + ".interrupts' doesn't have 2 items.");
    }

    _ptTransformed.put("msix_interrupt_start_index", interruptVector[0].c_str());
    _ptTransformed.put("msix_interrupt_end_index", interruptVector[1].c_str());
  }

  // -- Get the 'firmware" information
  SchemaTransform_subNode( "firmware", false /*required*/, 
                           SchemaTransformUniversal_firmware,
                           _ptOriginal, _ptTransformed);

  // -- Get the 'pcie_base_address_register' (optional)
  SchemaTransform_nameValue("pcie_bar_mapping", "pcie_base_address_register", false  /*required*/, _ptOriginal, _ptTransformed);

  // -- Get the 'interrupt_alias' (optional)
  if (_ptOriginal.find("interrupt_alias") != _ptOriginal.not_found()) 
      _ptTransformed.add_child("interrupt_alias", _ptOriginal.get_child("interrupt_alias"));

  SchemaTransform_subNode( "interrupt_mapping", false /*required*/, 
                           SchemaTransformToPM_interrupt_mapping,
                            _ptOriginal, _ptTransformed);
}

/**
 * Transforms all of the addressable_endpoints array elements.
 * 
 * @param _ptOriginal The original addressable_endpoints 
 *                    property tree
 * @param _ptTransformed The transformed addressable_endpoints 
 *                       property tree.
 */
void 
SchemaTransformToPM_addressable_endpoints( const boost::property_tree::ptree& _ptOriginal,
                                           boost::property_tree::ptree & _ptTransformed)
{ 
  // Look at each entry
  for (auto endpoint : _ptOriginal) {
    boost::property_tree::ptree ptEndpoint;

    SchemaTransformToPM_addressable_endpoint(endpoint.first, endpoint.second, ptEndpoint);
    _ptTransformed.add_child(endpoint.first, ptEndpoint);
  }
}


/**
 * Transforms all of the partition_info array elements.
 * 
 * @param _ptOriginal The original addressable_endpoints 
 *                    property tree
 * @param _ptTransformed The transformed addressable_endpoints 
 *                       property tree.
 */
void 
SchemaTransformToPM_partition_info( const boost::property_tree::ptree& _ptOriginal,
                                          boost::property_tree::ptree & _ptTransformed)
{ 
  // DRC check for known values
  if (_ptOriginal.find("__INFO") == _ptOriginal.not_found()) {
    throw std::runtime_error("Error: 'partition_info '__INFO' key not found.");
  }

  // Convert the JSON string to a boost property tree
  std::string sInfo = _ptOriginal.get<std::string>("__INFO");

  // Convert the JSON file to a boost property tree
  std::istringstream is(sInfo);
  boost::property_tree::read_json(is, _ptTransformed);
}

/**
 * Transform the DTC schema to the partition_metadata format
 * 
 * @param _ptOriginal The original property tree
 * @param _ptTransformed The transformed property tree
 */
void 
SchemaTransformToPM_root( const boost::property_tree::ptree & _ptOriginal, 
                                boost::property_tree::ptree & _ptTransformed)
{
  SchemaTransform_subNode( "schema_version", true /*required*/, 
                           SchemaTransformUniversal_schema_version,
                           _ptOriginal, _ptTransformed);

  SchemaTransform_nameValue("logic_uuid", "", false  /*required*/, _ptOriginal, _ptTransformed);

  SchemaTransform_subNode( "interfaces", true /*required*/, 
                           SchemaTransformToPM_interfaces,
                           _ptOriginal, _ptTransformed);

  SchemaTransform_subNode( "pcie", false /*required*/, 
                           SchemaTransformToPM_pcie,
                           _ptOriginal, _ptTransformed);

  SchemaTransform_subNode( "addressable_endpoints", false /*required*/, 
                           SchemaTransformToPM_addressable_endpoints,
                           _ptOriginal, _ptTransformed);

  SchemaTransform_subNode( "partition_info", false /*required*/, 
                           SchemaTransformToPM_partition_info,
                           _ptOriginal, _ptTransformed);
}

bool 
SectionPartitionMetadata::doesSupportAddFormatType(FormatType _eFormatType) const
{
  if (( _eFormatType == FormatType::JSON ) ||
      ( _eFormatType == FormatType::RAW )) {
    return true;
  }
  return false;
}

bool 
SectionPartitionMetadata::doesSupportDumpFormatType(FormatType _eFormatType) const
{
    if ((_eFormatType == FormatType::JSON) ||
        (_eFormatType == FormatType::HTML) ||
        (_eFormatType == FormatType::RAW))
    {
      return true;
    }

    return false;
}

void
SectionPartitionMetadata::marshalToJSON(char* _pDataSection,
                          unsigned int _sectionSize,
                          boost::property_tree::ptree& _ptree) const 
{
  XUtil::TRACE("");
  XUtil::TRACE("Extracting: DTC Image");

  boost::property_tree::ptree ptTransformed;

  // Parse the DTC buffer
  if (_pDataSection != nullptr) {
    boost::property_tree::ptree dtcTree;
    class DTC dtc(_pDataSection, _sectionSize, m_propertyNameFormat);
    dtc.marshalToJSON(dtcTree, m_propertyNameFormat);

    SchemaTransformToPM_root(dtcTree, ptTransformed);
  }
  // Create the JSON file
  _ptree.add_child("partition_metadata", ptTransformed);
  XUtil::TRACE_PrintTree("Ptree", _ptree);
}

void
SectionPartitionMetadata::marshalFromJSON(const boost::property_tree::ptree& _ptSection,
                            std::ostringstream& _buf) const 
{
  boost::property_tree::ptree ptOriginal = _ptSection.get_child("partition_metadata");

  if (ptOriginal.empty()) {
    throw std::runtime_error("Error: Missing 'partition_metadata' root node.");
  }

  boost::property_tree::ptree ptEmpty;
  boost::property_tree::ptree ptPartitionInfo = _ptSection.get_child("partition_info", ptEmpty);

  if (!ptPartitionInfo.empty()) {
    if (!ptOriginal.get_child("partition_info", ptEmpty).empty()) {
      throw std::runtime_error("Error: 'partition_info' is already a child node in partition_metadata.");
    }

    ptOriginal.add_child("partition_info", ptPartitionInfo);
  }

  boost::property_tree::ptree ptTransformed;
  SchemaTransformToDTC_root(ptOriginal, ptTransformed);

  XUtil::TRACE_PrintTree("Transformed JSON", ptTransformed);

  // Parse the DTC JSON file
  class DTC dtc(ptTransformed, m_propertyNameFormat);

  dtc.marshalToDTC(_buf);

  // Dump debug data
  std::string sBuf = _buf.str();
  XUtil::TRACE_BUF("DTC Buffer", sBuf.c_str(), sBuf.size());
}



void 
SectionPartitionMetadata::appendToSectionMetadata(const boost::property_tree::ptree& _ptAppendData,
                                         boost::property_tree::ptree& _ptToAppendTo)
{
  XUtil::TRACE_PrintTree("To Append To", _ptToAppendTo);
  XUtil::TRACE_PrintTree("Append data", _ptAppendData);

  boost::property_tree::ptree &ipShellTree = _ptToAppendTo.get_child("partition_metadata");
  for (auto childTree : _ptAppendData) {
    ipShellTree.add_child(childTree.first, childTree.second);
  }

  XUtil::TRACE_PrintTree("To Append To Done", _ptToAppendTo);
}

