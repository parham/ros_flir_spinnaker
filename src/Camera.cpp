

#include <string>
#include "ros_flir_spinnaker/Camera.hpp"

phm::Camera::Camera(Spinnaker::GenApi::INodeMap *nodeMap)
    : pNodeMap(std::unique_ptr<Spinnaker::GenApi::INodeMap>(nodeMap)) {
    // Empty body
}

phm::Camera::~Camera() {
    // Empty body
}

std::string phm::Camera::getModelName() {
    return strModelName;
}

void phm::Camera::initialize() {
    Spinnaker::GenApi::CIntegerPtr heightMaxNode =
        static_cast<Spinnaker::GenApi::CIntegerPtr>(getProperty("HeightMax"));
    iMaxHeight = heightMaxNode->GetValue();
    Spinnaker::GenApi::CIntegerPtr widthMaxNode =
        static_cast<Spinnaker::GenApi::CIntegerPtr>(getProperty("WidthMax"));
    iMaxWidth = widthMaxNode->GetValue();
    // Set Throughput to maximum
    // phm::setMaxInt(pNodeMap.get(), "DeviceLinkThroughputLimit");
}

void phm::Camera::configure(const ros_flir_spinnaker::phmSpinnakerConfig &, uint32_t) {
    ROS_INFO("Configure ...");
}

bool phm::Camera::executeCommand(const std::string & cmd) {
    Spinnaker::GenApi::CCommandPtr cmdPtr = pNodeMap->GetNode(cmd.c_str());
    if (!Spinnaker::GenApi::IsImplemented(cmdPtr)) {
        ROS_ERROR_STREAM("[SpinnakerCamera]: ("
            << static_cast<Spinnaker::GenApi::CStringPtr>(pNodeMap->GetNode("DeviceID"))->GetValue()
            << ") Feature name " << cmd << " not implemented.");
        return false;
    }
    if (Spinnaker::GenApi::IsAvailable(cmdPtr)) {
        if (Spinnaker::GenApi::IsWritable(cmdPtr)) {
            // Execute the command.
            cmdPtr->Execute();
            ROS_INFO_STREAM("Command " << cmd << " is executed!");
            return true;
        } else {
            ROS_WARN_STREAM("[SpinnakerCamera]: ("
                << static_cast<Spinnaker::GenApi::CStringPtr>(pNodeMap->GetNode("DeviceID"))->GetValue()
                << ") Feature " << cmd << " not writable.");
        }
    } else {
        ROS_WARN_STREAM("[SpinnakerCamera]: ("
            << static_cast<Spinnaker::GenApi::CStringPtr>(pNodeMap->GetNode("DeviceID"))->GetValue()
            << ") Feature " << cmd << " not available.");
    }

    return false;
}

Spinnaker::GenApi::CNodePtr phm::Camera::getProperty(const Spinnaker::GenICam::gcstring prop) {
    Spinnaker::GenApi::CNodePtr ptr = pNodeMap->GetNode(prop);
    if (!Spinnaker::GenApi::IsAvailable(ptr) || !Spinnaker::GenApi::IsReadable(ptr)) {
        throw std::runtime_error("Unable to get parmeter " + prop);
    }
    return ptr;
}

bool phm::Camera::setProperty(const std::string & prop, const std::string & entryName) {
    // *** NOTES ***
    // Enumeration nodes are slightly more complicated to set than other
    // nodes. This is because setting an enumeration node requires working
    // with two nodes instead of the usual one.
    //
    // As such, there are a number of steps to setting an enumeration node:
    // retrieve the enumeration node from the nodemap, retrieve the desired
    // entry node from the enumeration node, retrieve the integer value from
    // the entry node, and set the new value of the enumeration node with
    // the integer value from the entry node.
    Spinnaker::GenApi::CEnumerationPtr enumerationPtr = pNodeMap->GetNode(prop.c_str());

    if (!Spinnaker::GenApi::IsImplemented(enumerationPtr)) {
        ROS_ERROR_STREAM("[SpinnakerCamera]: ("
            << static_cast<Spinnaker::GenApi::CStringPtr>(pNodeMap->GetNode("DeviceID"))->GetValue()
            << ") Enumeration name " << prop << " not implemented.");
        return false;
    }

    if (Spinnaker::GenApi::IsAvailable(enumerationPtr)) {
        if (Spinnaker::GenApi::IsWritable(enumerationPtr)) {
            Spinnaker::GenApi::CEnumEntryPtr enumEmtryPtr = enumerationPtr->GetEntryByName(entryName.c_str());
            if (Spinnaker::GenApi::IsAvailable(enumEmtryPtr)) {
                if (Spinnaker::GenApi::IsReadable(enumEmtryPtr)) {
                    enumerationPtr->SetIntValue(enumEmtryPtr->GetValue());
                    ROS_INFO_STREAM("[SpinnakerCamera]: ("
                        << static_cast<Spinnaker::GenApi::CStringPtr>(pNodeMap->GetNode("DeviceID"))->GetValue()
                        << ") " << prop << " set to " << enumerationPtr->GetCurrentEntry()->GetSymbolic()
                        << ".");

                    return true;
                } else {
                    ROS_WARN_STREAM("[SpinnakerCamera]: ("
                        << static_cast<Spinnaker::GenApi::CStringPtr>(pNodeMap->GetNode("DeviceID"))->GetValue()
                        << ") Entry name " << entryName << " not writable.");
                }
            } else {
                ROS_WARN_STREAM("[SpinnakerCamera]: ("
                    << static_cast<Spinnaker::GenApi::CStringPtr>(pNodeMap->GetNode("DeviceID"))->GetValue()
                    << ") Entry name " << entryName << " not available.");

                ROS_WARN("Available:");
                Spinnaker::GenApi::NodeList_t entries;
                enumerationPtr->GetEntries(entries);
                for (auto &entry : entries) {
                    auto enumEntry = dynamic_cast<Spinnaker::GenApi::IEnumEntry *>(entry);
                    if (enumEntry && Spinnaker::GenApi::IsAvailable(entry))
                        ROS_WARN_STREAM(" - " << entry->GetName() << " (display " << entry->GetDisplayName() << ", symbolic "
                            << enumEntry->GetSymbolic() << ")");
                }
            }
        } else {
            ROS_WARN_STREAM("[SpinnakerCamera]: ("
                << static_cast<Spinnaker::GenApi::CStringPtr>(pNodeMap->GetNode("DeviceID"))->GetValue()
                << ") Enumeration " << prop << " not writable.");
        }
    } else {
        ROS_WARN_STREAM("[SpinnakerCamera]: ("
            << static_cast<Spinnaker::GenApi::CStringPtr>(pNodeMap->GetNode("DeviceID"))->GetValue()
            << ") Enumeration " << prop << " not available.");
    }
    return false;
}

bool phm::Camera::setProperty(const std::string & prop, const double & value) {
    Spinnaker::GenApi::CFloatPtr floatPtr = pNodeMap->GetNode(prop.c_str());
    if (!Spinnaker::GenApi::IsImplemented(floatPtr)) {
        ROS_ERROR_STREAM("[SpinnakerCamera]: ("
            << static_cast<Spinnaker::GenApi::CStringPtr>(pNodeMap->GetNode("DeviceID"))->GetValue()
            << ") Feature name " << prop << " not implemented.");
        return false;
    }
    if (Spinnaker::GenApi::IsAvailable(floatPtr)) {
        if (Spinnaker::GenApi::IsWritable(floatPtr)) {
            float temp_value = value;
            if (temp_value > floatPtr->GetMax())
                temp_value = floatPtr->GetMax();
            else if (temp_value < floatPtr->GetMin())
                temp_value = floatPtr->GetMin();
            floatPtr->SetValue(temp_value);
            ROS_INFO_STREAM("[SpinnakerCamera]: ("
                << static_cast<Spinnaker::GenApi::CStringPtr>(pNodeMap->GetNode("DeviceID"))->GetValue() << ") "
                << prop << " set to " << floatPtr->GetValue() << ".");
            return true;
        } else {
            ROS_WARN_STREAM("[SpinnakerCamera]: ("
                << static_cast<Spinnaker::GenApi::CStringPtr>(pNodeMap->GetNode("DeviceID"))->GetValue()
                << ") Feature " << prop << " not writable.");
        }
    } else {
        ROS_WARN_STREAM("[SpinnakerCamera]: ("
            << static_cast<Spinnaker::GenApi::CStringPtr>(pNodeMap->GetNode("DeviceID"))->GetValue()
            << ") Feature " << prop << " not available.");
    }
    return false;
}

bool phm::Camera::setProperty(const std::string & prop, const bool & value) {
    Spinnaker::GenApi::CBooleanPtr boolPtr = pNodeMap->GetNode(prop.c_str());
    if (!Spinnaker::GenApi::IsImplemented(boolPtr)) {
        ROS_ERROR_STREAM("[SpinnakerCamera]: ("
            << static_cast<Spinnaker::GenApi::CStringPtr>(pNodeMap->GetNode("DeviceID"))->GetValue()
            << ") Feature name " << prop << " not implemented.");
        return false;
    }
    if (Spinnaker::GenApi::IsAvailable(boolPtr)) {
        if (Spinnaker::GenApi::IsWritable(boolPtr)) {
            boolPtr->SetValue(value);
            ROS_INFO_STREAM("[SpinnakerCamera]: ("
                << static_cast<Spinnaker::GenApi::CStringPtr>(pNodeMap->GetNode("DeviceID"))->GetValue() << ") "
                << prop << " set to " << boolPtr->GetValue() << ".");
            return true;
        } else {
            ROS_WARN_STREAM("[SpinnakerCamera]: ("
                << static_cast<Spinnaker::GenApi::CStringPtr>(pNodeMap->GetNode("DeviceID"))->GetValue()
                << ") Feature " << prop << " not writable.");
        }
    } else {
        ROS_WARN_STREAM("[SpinnakerCamera]: ("
            << static_cast<Spinnaker::GenApi::CStringPtr>(pNodeMap->GetNode("DeviceID"))->GetValue()
            << ") Feature " << prop << " not available.");
    }
    return false;
}

bool phm::Camera::setProperty(const std::string & prop, const int & value) {
    Spinnaker::GenApi::CIntegerPtr intPtr = pNodeMap->GetNode(prop.c_str());
    if (!Spinnaker::GenApi::IsImplemented(intPtr)) {
        ROS_ERROR_STREAM("[SpinnakerCamera]: ("
            << static_cast<Spinnaker::GenApi::CStringPtr>(pNodeMap->GetNode("DeviceID"))->GetValue()
            << ") Feature name " << prop << " not implemented.");
        return false;
    }
    if (Spinnaker::GenApi::IsAvailable(intPtr)) {
        if (Spinnaker::GenApi::IsWritable(intPtr)) {
            int temp_value = value;
            if (temp_value > intPtr->GetMax())
                temp_value = intPtr->GetMax();
            else if (temp_value < intPtr->GetMin())
                temp_value = intPtr->GetMin();
            intPtr->SetValue(temp_value);
            ROS_INFO_STREAM("[SpinnakerCamera]: ("
                << static_cast<Spinnaker::GenApi::CStringPtr>(pNodeMap->GetNode("DeviceID"))->GetValue() << ") "
                << prop << " set to " << intPtr->GetValue() << ".");
            return true;
        } else {
            ROS_WARN_STREAM("[SpinnakerCamera]: ("
                << static_cast<Spinnaker::GenApi::CStringPtr>(pNodeMap->GetNode("DeviceID"))->GetValue()
                << ") Feature " << prop << " not writable.");
        }
    } else {
        ROS_WARN_STREAM("[SpinnakerCamera]: ("
            << static_cast<Spinnaker::GenApi::CStringPtr>(pNodeMap->GetNode("DeviceID"))->GetValue()
            << ") Feature " << prop << " not available.");
    }
    return false;
}

bool phm::Camera::setMaxInt(const std::string & prop) {
    Spinnaker::GenApi::CIntegerPtr intPtr = pNodeMap->GetNode(prop.c_str());
    if (Spinnaker::GenApi::IsAvailable(intPtr)) {
        if (Spinnaker::GenApi::IsWritable(intPtr)) {
            intPtr->SetValue(intPtr->GetMax());
            ROS_INFO_STREAM("[SpinnakerCamera]: ("
                << static_cast<Spinnaker::GenApi::CStringPtr>(pNodeMap->GetNode("DeviceID"))->GetValue() << ") "
                << prop << " set to " << intPtr->GetValue() << ".");
            return true;
        } else {
            ROS_WARN_STREAM("[SpinnakerCamera]: ("
                << static_cast<Spinnaker::GenApi::CStringPtr>(pNodeMap->GetNode("DeviceID"))->GetValue()
                << ") Feature " << prop << " not writable.");
        }
    } else {
        ROS_WARN_STREAM("[SpinnakerCamera]: ("
            << static_cast<Spinnaker::GenApi::CStringPtr>(pNodeMap->GetNode("DeviceID"))->GetValue()
            << ") Feature " << prop << " not available.");
    }
    return false;
}