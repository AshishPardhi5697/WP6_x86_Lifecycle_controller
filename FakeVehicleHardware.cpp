/*
 * Copyright (C) 2021 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#define LOG_TAG "FakeVehicleHardware"
#define ATRACE_TAG ATRACE_TAG_HAL
#define FAKE_VEHICLEHARDWARE_DEBUG false  // STOPSHIP if true.

#include "FakeVehicleHardware.h"

#include <FakeObd2Frame.h>
#include <JsonFakeValueGenerator.h>
#include <LinearFakeValueGenerator.h>
#include <PropertyUtils.h>
#include <VehicleHalTypes.h>
#include <VehicleUtils.h>

#include <android-base/file.h>
#include <android-base/parsedouble.h>
#include <android-base/properties.h>
#include <android-base/strings.h>
#include <android/hardware/automotive/vehicle/TestVendorProperty.h>
#include <utils/Log.h>
#include <utils/SystemClock.h>
#include <utils/Trace.h>

#include <dirent.h>
#include <inttypes.h>
#include <sys/types.h>
#include <regex>
#include <unordered_set>
#include <vector>

// Header for HIE
#include "HidlIfDataComm.h"
#include "EmulatorConfig.h"
#include "FcpComm.h"
#include <EmuLog.h>
#include <android-base/logging.h>
#include <aidl/android/hardware/automotive/vehicle/VehicleApPowerStateShutdownParam.h>
#include <aidl/com/honda/hardware/automotive/vehicle/VehicleHondaPowerStateReq.h>
#include <aidl/com/honda/hardware/automotive/vehicle/VehicleHondaPowerStateReqParam.h>
#include <sys/stat.h>

#define FILE_DIR_VEHICLE "/data/vendor/persist/emulator/vehicle/"
#define FILE_NAME_NON_VOLATILE FILE_DIR_VEHICLE "nonvolatile.json"
#define FILE_NAME_PARTS_CONFIGURATION FILE_DIR_VEHICLE "partsconfiguration.json"
#define FILE_NAME_CUSTOMIZE_CUSTOMCHECK FILE_DIR_VEHICLE "HONDA_CUSTOMIZE_CUSTOMCHECK.json"
#define FILE_NAME_CUSTOMIZE_CUSTOM_GET_RESPONSE FILE_DIR_VEHICLE "CUSTOMIZE_CUSTOM_GET_RESPONSE.json"
#define FILE_NAME_INDIRECT_COMMUNICATION FILE_DIR_VEHICLE "indirect_initvalue.json"
#define HONDA_CUSTOMIZE_CUSTOMCHECK_DEFAULT_VALUE_KEY 0xFFFF
#define HONDA_CUSTOMIZE_CUSTOMCHECK_FAILURE_VALUE_KEY 0x0000
#define METER_RESET 0x50

namespace android {
namespace hardware {
namespace automotive {
namespace vehicle {
namespace fake {

namespace {

#define PROP_ID_TO_CSTR(A) (propIdToString(A).c_str())

using ::aidl::android::hardware::automotive::vehicle::CruiseControlCommand;
using ::aidl::android::hardware::automotive::vehicle::CruiseControlType;
using ::aidl::android::hardware::automotive::vehicle::DriverDistractionState;
using ::aidl::android::hardware::automotive::vehicle::DriverDistractionWarning;
using ::aidl::android::hardware::automotive::vehicle::DriverDrowsinessAttentionState;
using ::aidl::android::hardware::automotive::vehicle::DriverDrowsinessAttentionWarning;
using ::aidl::android::hardware::automotive::vehicle::ErrorState;
using ::aidl::android::hardware::automotive::vehicle::GetValueRequest;
using ::aidl::android::hardware::automotive::vehicle::GetValueResult;
using ::aidl::android::hardware::automotive::vehicle::RawPropValues;
using ::aidl::android::hardware::automotive::vehicle::SetValueRequest;
using ::aidl::android::hardware::automotive::vehicle::SetValueResult;
using ::aidl::android::hardware::automotive::vehicle::StatusCode;
using ::aidl::android::hardware::automotive::vehicle::SubscribeOptions;
using ::aidl::android::hardware::automotive::vehicle::toString;
using ::aidl::android::hardware::automotive::vehicle::VehicleApPowerStateReport;
using ::aidl::android::hardware::automotive::vehicle::VehicleApPowerStateReq;
using ::aidl::android::hardware::automotive::vehicle::VehicleApPowerStateShutdownParam;
using ::aidl::android::hardware::automotive::vehicle::VehicleArea;
using ::aidl::android::hardware::automotive::vehicle::VehicleAreaConfig;
using ::aidl::android::hardware::automotive::vehicle::VehicleHwKeyInputAction;
using ::aidl::android::hardware::automotive::vehicle::VehiclePropConfig;
using ::aidl::android::hardware::automotive::vehicle::VehicleProperty;
using ::aidl::android::hardware::automotive::vehicle::VehiclePropertyAccess;
using ::aidl::android::hardware::automotive::vehicle::VehiclePropertyChangeMode;
using ::aidl::android::hardware::automotive::vehicle::VehiclePropertyGroup;
using ::aidl::android::hardware::automotive::vehicle::VehiclePropertyStatus;
using ::aidl::android::hardware::automotive::vehicle::VehiclePropertyType;
using ::aidl::android::hardware::automotive::vehicle::VehiclePropValue;
using ::aidl::android::hardware::automotive::vehicle::VehicleHwKeyInputAction;
using ::aidl::android::hardware::automotive::vehicle::VehicleGear;
using ::aidl::android::hardware::automotive::vehicle::VehicleUnit;

using ::android::hardware::automotive::remoteaccess::GetApPowerBootupReasonRequest;
using ::android::hardware::automotive::remoteaccess::GetApPowerBootupReasonResponse;
using ::android::hardware::automotive::remoteaccess::IsVehicleInUseRequest;
using ::android::hardware::automotive::remoteaccess::IsVehicleInUseResponse;
using ::android::hardware::automotive::remoteaccess::PowerController;

using ::aidl::com::honda::hardware::automotive::vehicle::VehicleHondaPowerStateReq;
using ::aidl::com::honda::hardware::automotive::vehicle::VehicleHondaPowerStateReqParam;

using ::android::base::EqualsIgnoreCase;
using ::android::base::Error;
using ::android::base::GetIntProperty;
using ::android::base::ParseFloat;
using ::android::base::Result;
using ::android::base::ScopedLockAssertion;
using ::android::base::StartsWith;
using ::android::base::StringPrintf;

// In order to test large number of vehicle property configs, we might generate additional fake
// property config start from this ID. These fake properties are for getPropertyList,
//  getPropertiesAsync, and setPropertiesAsync.
// HONDA_ECL_CANCELDISP_REQ/HONDA_EAT_TRANS_SPEED/HONDA_SEND_CANNM_OTA are within this range.
// 0x21403000
constexpr int32_t STARTING_VENDOR_CODE_PROPERTIES_FOR_TEST =
        0x3000 | toInt(VehiclePropertyGroup::VENDOR) | toInt(VehicleArea::GLOBAL) |
        toInt(VehiclePropertyType::INT32);
// 0x21405000
constexpr int32_t ENDING_VENDOR_CODE_PROPERTIES_FOR_TEST =
        0x5000 | toInt(VehiclePropertyGroup::VENDOR) | toInt(VehicleArea::GLOBAL) |
        toInt(VehiclePropertyType::INT32);
// The directory for default property configuration file.
// For config file format, see impl/default_config/config/README.md.
constexpr char DEFAULT_CONFIG_DIR[] = "/vendor/etc/automotive/honda/vhalconfig/";
// The directory for property configuration file that overrides the default configuration file.
// For config file format, see impl/default_config/config/README.md.
constexpr char OVERRIDE_CONFIG_DIR[] = "/vendor/etc/automotive/honda/vhaloverride/";
// The optional config file for power controller grpc service that provides vehicleInUse and
// ApPowerBootupReason property.
constexpr char GRPC_SERVICE_CONFIG_FILE[] = "/vendor/etc/automotive/powercontroller/serverconfig";
// If OVERRIDE_PROPERTY is set, we will use the configuration files from OVERRIDE_CONFIG_DIR to
// overwrite the default configs.
constexpr char OVERRIDE_PROPERTY[] = "persist.vendor.vhal_init_value_override";
constexpr char POWER_STATE_REQ_CONFIG_PROPERTY[] = "ro.vendor.fake_vhal.ap_power_state_req.config";
// The value to be returned if VENDOR_PROPERTY_FOR_ERROR_CODE_TESTING is set as the property
constexpr int VENDOR_ERROR_CODE = 0x00ab0005;
// A list of supported options for "--set" command.
const std::unordered_set<std::string> SET_PROP_OPTIONS = {
        // integer.
        "-i",
        // 64bit integer.
        "-i64",
        // float.
        "-f",
        // string.
        "-s",
        // bytes in hex format, e.g. 0xDEADBEEF.
        "-b",
        // Area id in integer.
        "-a",
        // Timestamp in int64.
        "-t"};

// ADAS _ENABLED property to list of ADAS state properties using ErrorState enum.
const std::unordered_map<int32_t, std::vector<int32_t>> mAdasEnabledPropToAdasPropWithErrorState = {
        // AEB
        {
                toInt(VehicleProperty::AUTOMATIC_EMERGENCY_BRAKING_ENABLED),
                {
                        toInt(VehicleProperty::AUTOMATIC_EMERGENCY_BRAKING_STATE),
                },
        },
        // FCW
        {
                toInt(VehicleProperty::FORWARD_COLLISION_WARNING_ENABLED),
                {
                        toInt(VehicleProperty::FORWARD_COLLISION_WARNING_STATE),
                },
        },
        // BSW
        {
                toInt(VehicleProperty::BLIND_SPOT_WARNING_ENABLED),
                {
                        toInt(VehicleProperty::BLIND_SPOT_WARNING_STATE),
                },
        },
        // LDW
        {
                toInt(VehicleProperty::LANE_DEPARTURE_WARNING_ENABLED),
                {
                        toInt(VehicleProperty::LANE_DEPARTURE_WARNING_STATE),
                },
        },
        // LKA
        {
                toInt(VehicleProperty::LANE_KEEP_ASSIST_ENABLED),
                {
                        toInt(VehicleProperty::LANE_KEEP_ASSIST_STATE),
                },
        },
        // LCA
        {
                toInt(VehicleProperty::LANE_CENTERING_ASSIST_ENABLED),
                {
                        toInt(VehicleProperty::LANE_CENTERING_ASSIST_STATE),
                },
        },
        // ELKA
        {
                toInt(VehicleProperty::EMERGENCY_LANE_KEEP_ASSIST_ENABLED),
                {
                        toInt(VehicleProperty::EMERGENCY_LANE_KEEP_ASSIST_STATE),
                },
        },
        // CC
        {
                toInt(VehicleProperty::CRUISE_CONTROL_ENABLED),
                {
                        toInt(VehicleProperty::CRUISE_CONTROL_TYPE),
                        toInt(VehicleProperty::CRUISE_CONTROL_STATE),
                },
        },
        // HOD
        {
                toInt(VehicleProperty::HANDS_ON_DETECTION_ENABLED),
                {
                        toInt(VehicleProperty::HANDS_ON_DETECTION_DRIVER_STATE),
                        toInt(VehicleProperty::HANDS_ON_DETECTION_WARNING),
                },
        },
        // Driver Drowsiness and Attention
        {
                toInt(VehicleProperty::DRIVER_DROWSINESS_ATTENTION_SYSTEM_ENABLED),
                {
                        toInt(VehicleProperty::DRIVER_DROWSINESS_ATTENTION_STATE),
                },
        },
        // Driver Drowsiness and Attention Warning
        {
                toInt(VehicleProperty::DRIVER_DROWSINESS_ATTENTION_WARNING_ENABLED),
                {
                        toInt(VehicleProperty::DRIVER_DROWSINESS_ATTENTION_WARNING),
                },
        },
        // Driver Distraction
        {
                toInt(VehicleProperty::DRIVER_DISTRACTION_SYSTEM_ENABLED),
                {
                        toInt(VehicleProperty::DRIVER_DISTRACTION_STATE),
                        toInt(VehicleProperty::DRIVER_DISTRACTION_WARNING),
                },
        },
        // Driver Distraction Warning
        {
                toInt(VehicleProperty::DRIVER_DISTRACTION_WARNING_ENABLED),
                {
                        toInt(VehicleProperty::DRIVER_DISTRACTION_WARNING),
                },
        },
        // LSCW
        {
                toInt(VehicleProperty::LOW_SPEED_COLLISION_WARNING_ENABLED),
                {
                        toInt(VehicleProperty::LOW_SPEED_COLLISION_WARNING_STATE),
                },
        },
        // ESC
        {
                toInt(VehicleProperty::ELECTRONIC_STABILITY_CONTROL_ENABLED),
                {
                        toInt(VehicleProperty::ELECTRONIC_STABILITY_CONTROL_STATE),
                },
        },
        // CTMW
        {
                toInt(VehicleProperty::CROSS_TRAFFIC_MONITORING_ENABLED),
                {
                        toInt(VehicleProperty::CROSS_TRAFFIC_MONITORING_WARNING_STATE),
                },
        },
        // LSAEB
        {
                toInt(VehicleProperty::LOW_SPEED_AUTOMATIC_EMERGENCY_BRAKING_ENABLED),
                {
                        toInt(VehicleProperty::LOW_SPEED_AUTOMATIC_EMERGENCY_BRAKING_STATE),
                },
        },
};

// The list of VHAL properties that might be handled by an external power controller.
const std::unordered_set<int32_t> mPowerPropIds = {toInt(VehicleProperty::VEHICLE_IN_USE),
                                                   toInt(VehicleProperty::AP_POWER_BOOTUP_REASON)};

void maybeGetGrpcServiceInfo(std::string* address) {
    std::ifstream ifs(GRPC_SERVICE_CONFIG_FILE);
    if (!ifs) {
        ALOGI("Cannot open grpc service config file at: %s, assume no service is available",
              GRPC_SERVICE_CONFIG_FILE);
        return;
    }
    ifs >> *address;
    ifs.close();
}

inline std::string vecToStringOfHexValues(const std::vector<int32_t>& vec) {
    std::stringstream ss;
    ss << "[";
    for (size_t i = 0; i < vec.size(); i++) {
        if (i != 0) {
            ss << ",";
        }
        ss << std::showbase << std::hex << vec[i];
    }
    ss << "]";
    return ss.str();
}

}  // namespace

std::mutex gMutex;
std::condition_variable gCond;
std::map<uint16_t, VehiclePropValue> gCustomCheckVPValueSettings;
std::map<int32_t, VehiclePropValue> gIndirectVPValueSettings;
std::unique_ptr<HidlIfDataComm> hidlIfDataComm_;

void FakeVehicleHardware::storePropInitialValue(const ConfigDeclaration& config) {
    const VehiclePropConfig& vehiclePropConfig = config.config;
    int propId = vehiclePropConfig.prop;

    // A global property will have only a single area
    bool globalProp = isGlobalProp(propId);
    size_t numAreas = globalProp ? 1 : vehiclePropConfig.areaConfigs.size();

    if (propId == toInt(VehicleProperty::HVAC_POWER_ON)) {
        const auto& configArray = vehiclePropConfig.configArray;
        hvacPowerDependentProps.insert(configArray.begin(), configArray.end());
    }

    bool isAreaIdInitializing = false;
    if (numAreas == 0) {
        numAreas = 1;
        isAreaIdInitializing = true;
    }

    for (size_t i = 0; i < numAreas; i++) {
        int32_t curArea = 0;
        if (globalProp) {
            curArea = 0;
        } else if (isAreaIdInitializing) {
            curArea = 0;
        } else {
            curArea = vehiclePropConfig.areaConfigs[i].areaId;
        }
        // Create a separate instance for each individual zone
        VehiclePropValue prop = {
                .timestamp = elapsedRealtimeNano(),
                .areaId = curArea,
                .prop = propId,
                .value = {},
        };

        if (config.initialAreaValues.empty()) {
            if (config.initialValue == RawPropValues{}) {
                ALOGD("init default value for prop 0x%x area 0x%x", propId, curArea);
                // [modify] Donot Skip empty initial values.
                // continue;
            }
            prop.value = config.initialValue;
        } else if (auto valueForAreaIt = config.initialAreaValues.find(curArea);
                   valueForAreaIt != config.initialAreaValues.end()) {
            prop.value = valueForAreaIt->second;
        } else {
            ALOGW("failed to get default value for prop 0x%x area 0x%x", propId, curArea);
            continue;
        }

        auto result =
                mServerSidePropStore->writeValue(mValuePool->obtain(prop), /*updateStatus=*/true);
        if (!result.ok()) {
            ALOGE("failed to write default config value, error: %s, status: %d",
                  getErrorMsg(result).c_str(), getIntErrorCode(result));
        }
    }
}

FakeVehicleHardware::FakeVehicleHardware()
    : FakeVehicleHardware(DEFAULT_CONFIG_DIR, OVERRIDE_CONFIG_DIR, false) {}

FakeVehicleHardware::FakeVehicleHardware(std::string defaultConfigDir,
                                         std::string overrideConfigDir, bool forceOverride)
    : mValuePool(std::make_unique<VehiclePropValuePool>()),
      mServerSidePropStore(new VehiclePropertyStore(mValuePool)),
      mDefaultConfigDir(defaultConfigDir),
      mOverrideConfigDir(overrideConfigDir),
      mFakeObd2Frame(new obd2frame::FakeObd2Frame(mServerSidePropStore)),
      mFakeUserHal(new FakeUserHal(mValuePool)),
      mRecurrentTimer(new RecurrentTimer()),
      mGeneratorHub(new GeneratorHub(
              [this](const VehiclePropValue& value) { eventFromVehicleBus(value); })),
      mPendingGetValueRequests(this),
      mPendingSetValueRequests(this),
      mForceOverride(forceOverride),
      mAddExtraTestVendorConfigs(false),
      mPowerRecurrentTimer(new RecurrentTimer()),
      mEmulatedCustomizeThread(std::bind(&FakeVehicleHardware::processHondaCustomizeEvent, this, std::placeholders::_1)) {
    init();
}

FakeVehicleHardware::~FakeVehicleHardware() {
    mPendingGetValueRequests.stop();
    mPendingSetValueRequests.stop();
    mGeneratorHub.reset();
    mEmulatedCustomizeThread.terminate();
}

bool FakeVehicleHardware::UseOverrideConfigDir() {
    return mForceOverride ||
           android::base::GetBoolProperty(OVERRIDE_PROPERTY, /*default_value=*/false);
}

std::unordered_map<int32_t, ConfigDeclaration> FakeVehicleHardware::loadConfigDeclarations() {
    std::unordered_map<int32_t, ConfigDeclaration> configsByPropId;
    bool defaultConfigLoaded = loadPropConfigsFromDir(mDefaultConfigDir, &configsByPropId);
    if (!defaultConfigLoaded) {
        // This cannot work without a valid default config.
        ALOGE("Failed to load default config, exiting");
        exit(1);
    }
    if (UseOverrideConfigDir()) {
        loadPropConfigsFromDir(mOverrideConfigDir, &configsByPropId);
    }
    return configsByPropId;
}

void FakeVehicleHardware::init() {
    FcpComm::getInstance()->init();
    hidlIfDataComm_ = std::make_unique<HidlIfDataComm>();
    hidlIfDataComm_->init();

    maybeGetGrpcServiceInfo(&mPowerControllerServiceAddress);

    for (auto& [_, configDeclaration] : loadConfigDeclarations()) {
        VehiclePropConfig cfg = configDeclaration.config;
        VehiclePropertyStore::TokenFunction tokenFunction = nullptr;
        //Ashish changes start
        if (cfg.prop == toInt(VehicleProperty::AP_POWER_STATE_REQ)) {
    // DO NOT clobber the JSON configArray[0] (it contains VehicleApPowerStateConfigFlag bits).
    // Use JSON as default, override only if ro.vendor.fake_vhal.ap_power_state_req.config is set.
    int defaultConfig = 0;
    if (!cfg.configArray.empty()) {
        defaultConfig = cfg.configArray[0];
    }
 
    const int config = android::base::GetIntProperty(
            POWER_STATE_REQ_CONFIG_PROPERTY, /*default_value=*/defaultConfig);
 
    if (cfg.configArray.empty()) {
        cfg.configArray.resize(1);
    }
    cfg.configArray[0] = config;
 
    ALOGI("AP_POWER_STATE_REQ configArray[0]=0x%x (default=0x%x, prop=%s)",
        cfg.configArray[0], defaultConfig, POWER_STATE_REQ_CONFIG_PROPERTY);
} else if (cfg.prop == OBD2_FREEZE_FRAME) {
    tokenFunction = [](const VehiclePropValue& propValue) { return propValue.timestamp; };
} //ashish changes finish

 //      if (cfg.prop == toInt(VehicleProperty::AP_POWER_STATE_REQ)) {
 //          int config = GetIntProperty(POWER_STATE_REQ_CONFIG_PROPERTY, /*default_value=*/0);
 //           cfg.configArray[0] = config;
 //       } else if (cfg.prop == OBD2_FREEZE_FRAME) {
 //           tokenFunction = [](const VehiclePropValue& propValue) { return propValue.timestamp; };
  //      }

        mServerSidePropStore->registerProperty(cfg, tokenFunction);
        if (obd2frame::FakeObd2Frame::isDiagnosticProperty(cfg)) {
            // Ignore storing default value for diagnostic property. They have special get/set
            // logic.
            continue;
        }
        storePropInitialValue(configDeclaration);
        defaultconfig::appendDefaultConfig(configDeclaration);
    }

    // OBD2_LIVE_FRAME and OBD2_FREEZE_FRAME must be configured in default configs.
    auto maybeObd2LiveFrame = mServerSidePropStore->getPropConfig(OBD2_LIVE_FRAME);
    if (maybeObd2LiveFrame.has_value()) {
        mFakeObd2Frame->initObd2LiveFrame(maybeObd2LiveFrame.value());
    }
    auto maybeObd2FreezeFrame = mServerSidePropStore->getPropConfig(OBD2_FREEZE_FRAME);
    if (maybeObd2FreezeFrame.has_value()) {
        mFakeObd2Frame->initObd2FreezeFrame(maybeObd2FreezeFrame.value());
    }

    mServerSidePropStore->setOnValuesChangeCallback([this](std::vector<VehiclePropValue> values) {
        return onValuesChangeCallback(std::move(values));
    });

    // Initial value setting
    std::unique_lock<std::mutex> lock(gMutex);
    if (!FcpComm::getInstance()->isGetFcpWord()) {
        std::chrono::steady_clock::time_point time =
            std::chrono::steady_clock::now() + std::chrono::milliseconds(100 /* Time to wait for data */);
        std::cv_status result = gCond.wait_until(lock, time);
        if (result == std::cv_status::timeout) {
            LOG(WARNING) << __func__ << " Fcp data acquisition timed out";
        }
    }
    initPropertyForColdStart();
}

std::vector<VehiclePropConfig> FakeVehicleHardware::getAllPropertyConfigs() const {
    std::vector<VehiclePropConfig> allConfigs = mServerSidePropStore->getAllConfigs();
    if (mAddExtraTestVendorConfigs) {
        generateVendorConfigs(/* outAllConfigs= */ allConfigs);
    }
    return allConfigs;
}

VehiclePropValuePool::RecyclableType FakeVehicleHardware::createApPowerStateReq(
        VehicleApPowerStateReq state) {
    auto req = mValuePool->obtain(VehiclePropertyType::INT32_VEC, 2);
    req->prop = toInt(VehicleProperty::AP_POWER_STATE_REQ);
    req->areaId = 0;
    req->timestamp = elapsedRealtimeNano();
    req->status = VehiclePropertyStatus::AVAILABLE;
    req->value.int32Values[0] = toInt(state);
    // Param = 0.
    req->value.int32Values[1] = 0;
    return req;
}

VehiclePropValuePool::RecyclableType FakeVehicleHardware::createAdasStateReq(int32_t propertyId,
                                                                             int32_t areaId,
                                                                             int32_t state) {
    auto req = mValuePool->obtain(VehiclePropertyType::INT32);
    req->prop = propertyId;
    req->areaId = areaId;
    req->timestamp = elapsedRealtimeNano();
    req->status = VehiclePropertyStatus::AVAILABLE;
    req->value.int32Values[0] = state;
    return req;
}

VhalResult<void> FakeVehicleHardware::setApPowerStateReqShutdown(const VehiclePropValue& value) {
    if (value.value.int32Values.size() != 1) {
        return StatusError(StatusCode::INVALID_ARG)
               << "Failed to set SHUTDOWN_REQUEST, expect 1 int value: "
               << "VehicleApPowerStateShutdownParam";
    }
    int powerStateShutdownParam = value.value.int32Values[0];
    auto prop = createApPowerStateReq(VehicleApPowerStateReq::SHUTDOWN_PREPARE);
    prop->value.int32Values[1] = powerStateShutdownParam;
    if (auto writeResult = mServerSidePropStore->writeValue(
                std::move(prop), /*updateStatus=*/true, VehiclePropertyStore::EventMode::ALWAYS);
        !writeResult.ok()) {
        return StatusError(getErrorCode(writeResult))
               << "failed to write AP_POWER_STATE_REQ into property store, error: "
               << getErrorMsg(writeResult);
    }
    return {};
}

VhalResult<void> FakeVehicleHardware::setApPowerStateReport(const VehiclePropValue& value, bool* isSpecialValue) {
    *isSpecialValue = true;
    auto updatedValue = mValuePool->obtain(value);
    updatedValue->timestamp = elapsedRealtimeNano();

    if (auto writeResult = mServerSidePropStore->writeValue(std::move(updatedValue));
        !writeResult.ok()) {
        return StatusError(getErrorCode(writeResult))
               << "failed to write value into property store, error: " << getErrorMsg(writeResult);
    }

    LOG(INFO) << "AP_POWER_STATE_REPORT reached! value:" << value.value.int32Values[0];
    mHwKeyInput.receivePowerState(value);

    VehiclePropValuePool::RecyclableType prop;
    int32_t state = value.value.int32Values[0];
    switch (state) {
        case toInt(VehicleApPowerStateReport::DEEP_SLEEP_EXIT):
            setSuspendExitEvent(false);
            writeValueAndNotifyChange(*createApPowerStateReq(VehicleApPowerStateReq::ON), true /* updateStatus */);
            writeValueAndNotifyChange(createHondaIcbNotification(toInt(AccState::ACC_ON),
                                                                toInt(KeyofftimerStartUpState::OFF),
                                                               toInt(HalInitializeState::COMPLETED)),
                                    true);
            *isSpecialValue = false;
            break;
        case toInt(VehicleApPowerStateReport::SHUTDOWN_CANCELLED):
            processPowerStateEvent(value);
            *isSpecialValue = false;
            break;
        case toInt(VehicleApPowerStateReport::WAIT_FOR_VHAL):
            // CPMS is in WAIT_FOR_VHAL state, simply move to ON and send back to HAL.
            // Must erase existing state because in the case when Car Service crashes, the power
            // state would already be ON when we receive WAIT_FOR_VHAL and thus new property change
            // event would be generated. However, Car Service always expect a property change event
            // even though there is not actual state change.

            processPowerStateEvent(value); 
            *isSpecialValue = false;
            break;
        case toInt(VehicleApPowerStateReport::HIBERNATION_EXIT):
            mServerSidePropStore->removeValuesForProperty(
                   toInt(VehicleProperty::AP_POWER_STATE_REQ));
            prop = createApPowerStateReq(VehicleApPowerStateReq::ON);

            // ALWAYS update status for generated property value
            if (auto writeResult =
                        mServerSidePropStore->writeValue(std::move(prop), /*updateStatus=*/true);
                !writeResult.ok()) {
                return StatusError(getErrorCode(writeResult))
                       << "failed to write AP_POWER_STATE_REQ into property store, error: "
                       << getErrorMsg(writeResult);
            }
         	//LOG(INFO) << "ashish: enter into HIBERNATION_EXIT, stay gated";
   		 //*isSpecialValue = false;
            break;
           
        case toInt(VehicleApPowerStateReport::DEEP_SLEEP_ENTRY):
            writeValueAndNotifyChange(createHondaIcbNotification(toInt(AccState::ACC_OFF),
                                                                toInt(KeyofftimerStartUpState::OFF),
                                                                toInt(HalInitializeState::IMCOMPLETE)),
                                    true);
            writeValueAndNotifyChange(*createApPowerStateReq(VehicleApPowerStateReq::FINISHED),
                                    true /* updateStatus */);
            processPowerStateEvent(value);
            *isSpecialValue = false;
            break;
        case toInt(VehicleApPowerStateReport::HIBERNATION_ENTRY):
            // CPMS is in WAIT_FOR_FINISH state, send the FINISHED command
            // Send back to HAL
            // ALWAYS update status for generated property value
            prop = createApPowerStateReq(VehicleApPowerStateReq::FINISHED);
            if (auto writeResult =
                        mServerSidePropStore->writeValue(std::move(prop), /*updateStatus=*/true);
                !writeResult.ok()) {
                return StatusError(getErrorCode(writeResult))
                       << "failed to write AP_POWER_STATE_REQ into property store, error: "
                       << getErrorMsg(writeResult);
            }
            break;
        case toInt(VehicleApPowerStateReport::SHUTDOWN_START):
            writeValueAndNotifyChange(createHondaIcbNotification(toInt(AccState::ACC_OFF),
                                                                toInt(KeyofftimerStartUpState::OFF),
                                                                toInt(HalInitializeState::IMCOMPLETE)),
                                    true);
            *isSpecialValue = false;
            break;
        case toInt(VehicleApPowerStateReport::ON):
            setSuspendExitEvent(false);
            *isSpecialValue = false;
            break;
        default:
            ALOGE("Unknown VehicleApPowerStateReport: %d", state);
            break;
    }
    return {};
}

int FakeVehicleHardware::getHvacTempNumIncrements(int requestedTemp, int minTemp, int maxTemp,
                                                  int increment) {
    requestedTemp = std::max(requestedTemp, minTemp);
    requestedTemp = std::min(requestedTemp, maxTemp);
    int numIncrements = std::round((requestedTemp - minTemp) / static_cast<float>(increment));
    return numIncrements;
}

void FakeVehicleHardware::updateHvacTemperatureValueSuggestionInput(
        const std::vector<int>& hvacTemperatureSetConfigArray,
        std::vector<float>* hvacTemperatureValueSuggestionInput) {
    int minTempInCelsius = hvacTemperatureSetConfigArray[0];
    int maxTempInCelsius = hvacTemperatureSetConfigArray[1];
    int incrementInCelsius = hvacTemperatureSetConfigArray[2];

    int minTempInFahrenheit = hvacTemperatureSetConfigArray[3];
    int maxTempInFahrenheit = hvacTemperatureSetConfigArray[4];
    int incrementInFahrenheit = hvacTemperatureSetConfigArray[5];

    // The HVAC_TEMPERATURE_SET config array values are temperature values that have been multiplied
    // by 10 and converted to integers. Therefore, requestedTemp must also be multiplied by 10 and
    // converted to an integer in order for them to be the same units.
    int requestedTemp = static_cast<int>((*hvacTemperatureValueSuggestionInput)[0] * 10.0f);
    int numIncrements =
            (*hvacTemperatureValueSuggestionInput)[1] == toInt(VehicleUnit::CELSIUS)
                    ? getHvacTempNumIncrements(requestedTemp, minTempInCelsius, maxTempInCelsius,
                                               incrementInCelsius)
                    : getHvacTempNumIncrements(requestedTemp, minTempInFahrenheit,
                                               maxTempInFahrenheit, incrementInFahrenheit);

    int suggestedTempInCelsius = minTempInCelsius + incrementInCelsius * numIncrements;
    int suggestedTempInFahrenheit = minTempInFahrenheit + incrementInFahrenheit * numIncrements;
    // HVAC_TEMPERATURE_VALUE_SUGGESTION specifies the temperature values to be in the original
    // floating point form so we divide by 10 and convert to float.
    (*hvacTemperatureValueSuggestionInput)[2] = static_cast<float>(suggestedTempInCelsius) / 10.0f;
    (*hvacTemperatureValueSuggestionInput)[3] =
            static_cast<float>(suggestedTempInFahrenheit) / 10.0f;
}

VhalResult<void> FakeVehicleHardware::setHvacTemperatureValueSuggestion(
        const VehiclePropValue& hvacTemperatureValueSuggestion) {
    auto hvacTemperatureSetConfigResult =
            mServerSidePropStore->getPropConfig(toInt(VehicleProperty::HVAC_TEMPERATURE_SET));

    if (!hvacTemperatureSetConfigResult.ok()) {
        return StatusError(getErrorCode(hvacTemperatureSetConfigResult)) << StringPrintf(
                       "Failed to set HVAC_TEMPERATURE_VALUE_SUGGESTION because"
                       " HVAC_TEMPERATURE_SET could not be retrieved. Error: %s",
                       getErrorMsg(hvacTemperatureSetConfigResult).c_str());
    }

    const auto& originalInput = hvacTemperatureValueSuggestion.value.floatValues;
    if (originalInput.size() != 4) {
        return StatusError(StatusCode::INVALID_ARG) << StringPrintf(
                       "Failed to set HVAC_TEMPERATURE_VALUE_SUGGESTION because float"
                       " array value is not size 4.");
    }

    bool isTemperatureUnitSpecified = originalInput[1] == toInt(VehicleUnit::CELSIUS) ||
                                      originalInput[1] == toInt(VehicleUnit::FAHRENHEIT);
    if (!isTemperatureUnitSpecified) {
        return StatusError(StatusCode::INVALID_ARG) << StringPrintf(
                       "Failed to set HVAC_TEMPERATURE_VALUE_SUGGESTION because float"
                       " value at index 1 is not any of %d or %d, which corresponds to"
                       " VehicleUnit#CELSIUS and VehicleUnit#FAHRENHEIT respectively.",
                       toInt(VehicleUnit::CELSIUS), toInt(VehicleUnit::FAHRENHEIT));
    }

    auto updatedValue = mValuePool->obtain(hvacTemperatureValueSuggestion);
    const auto& hvacTemperatureSetConfigArray = hvacTemperatureSetConfigResult.value().configArray;
    auto& hvacTemperatureValueSuggestionInput = updatedValue->value.floatValues;

    updateHvacTemperatureValueSuggestionInput(hvacTemperatureSetConfigArray,
                                              &hvacTemperatureValueSuggestionInput);

    updatedValue->timestamp = elapsedRealtimeNano();
    auto writeResult = mServerSidePropStore->writeValue(std::move(updatedValue),
                                                        /* updateStatus = */ true,
                                                        VehiclePropertyStore::EventMode::ALWAYS);
    if (!writeResult.ok()) {
        return StatusError(getErrorCode(writeResult))
               << StringPrintf("failed to write value into property store, error: %s",
                               getErrorMsg(writeResult).c_str());
    }

    return {};
}

VhalResult<void> FakeVehicleHardware::isAdasPropertyAvailable(int32_t adasStatePropertyId) const {
    auto adasStateResult = mServerSidePropStore->readValue(adasStatePropertyId);
    if (!adasStateResult.ok()) {
        ALOGW("Failed to get ADAS ENABLED property 0x%x, error: %s", adasStatePropertyId,
              getErrorMsg(adasStateResult).c_str());
        return {};
    }

    if (adasStateResult.value()->value.int32Values.size() == 1 &&
        adasStateResult.value()->value.int32Values[0] < 0) {
        auto errorState = adasStateResult.value()->value.int32Values[0];
        switch (errorState) {
            case toInt(ErrorState::NOT_AVAILABLE_DISABLED):
                return StatusError(StatusCode::NOT_AVAILABLE_DISABLED)
                       << "ADAS feature is disabled.";
            case toInt(ErrorState::NOT_AVAILABLE_SPEED_LOW):
                return StatusError(StatusCode::NOT_AVAILABLE_SPEED_LOW)
                       << "ADAS feature is disabled because the vehicle speed is too low.";
            case toInt(ErrorState::NOT_AVAILABLE_SPEED_HIGH):
                return StatusError(StatusCode::NOT_AVAILABLE_SPEED_HIGH)
                       << "ADAS feature is disabled because the vehicle speed is too high.";
            case toInt(ErrorState::NOT_AVAILABLE_POOR_VISIBILITY):
                return StatusError(StatusCode::NOT_AVAILABLE_POOR_VISIBILITY)
                       << "ADAS feature is disabled because the visibility is too poor.";
            case toInt(ErrorState::NOT_AVAILABLE_SAFETY):
                return StatusError(StatusCode::NOT_AVAILABLE_SAFETY)
                       << "ADAS feature is disabled because of safety reasons.";
            default:
                return StatusError(StatusCode::NOT_AVAILABLE) << "ADAS feature is not available.";
        }
    }

    return {};
}

VhalResult<void> FakeVehicleHardware::setUserHalProp(const VehiclePropValue& value) {
    auto result = mFakeUserHal->onSetProperty(value);
    if (!result.ok()) {
        return StatusError(getErrorCode(result))
               << "onSetProperty(): HAL returned error: " << getErrorMsg(result);
    }
    auto& updatedValue = result.value();
    if (updatedValue != nullptr) {
        ALOGI("onSetProperty(): updating property returned by HAL: %s",
              updatedValue->toString().c_str());
        // Update timestamp otherwise writeValue might fail because the timestamp is outdated.
        updatedValue->timestamp = elapsedRealtimeNano();
        if (auto writeResult = mServerSidePropStore->writeValue(
                    std::move(result.value()),
                    /*updateStatus=*/true, VehiclePropertyStore::EventMode::ALWAYS);
            !writeResult.ok()) {
            return StatusError(getErrorCode(writeResult))
                   << "failed to write value into property store, error: "
                   << getErrorMsg(writeResult);
        }
    }
    return {};
}

VhalResult<void> FakeVehicleHardware::synchronizeHvacTemp(int32_t hvacDualOnAreaId,
                                                          std::optional<float> newTempC) const {
    auto hvacTemperatureSetResults = mServerSidePropStore->readValuesForProperty(
            toInt(VehicleProperty::HVAC_TEMPERATURE_SET));
    if (!hvacTemperatureSetResults.ok()) {
        return StatusError(StatusCode::NOT_AVAILABLE)
               << "Failed to get HVAC_TEMPERATURE_SET, error: "
               << getErrorMsg(hvacTemperatureSetResults);
    }
    auto& hvacTemperatureSetValues = hvacTemperatureSetResults.value();
    std::optional<float> tempCToSynchronize = newTempC;
    for (size_t i = 0; i < hvacTemperatureSetValues.size(); i++) {
        int32_t areaId = hvacTemperatureSetValues[i]->areaId;
        if ((hvacDualOnAreaId & areaId) != areaId) {
            continue;
        }
        if (hvacTemperatureSetValues[i]->status != VehiclePropertyStatus::AVAILABLE) {
            continue;
        }
        // When HVAC_DUAL_ON is initially enabled, synchronize all area IDs
        // to the temperature of the first area ID, which is the driver's.
        if (!tempCToSynchronize.has_value()) {
            tempCToSynchronize = hvacTemperatureSetValues[i]->value.floatValues[0];
            continue;
        }
        auto updatedValue = std::move(hvacTemperatureSetValues[i]);
        updatedValue->value.floatValues[0] = tempCToSynchronize.value();
        updatedValue->timestamp = elapsedRealtimeNano();
        // This will trigger a property change event for the current hvac property value.
        auto writeResult =
                mServerSidePropStore->writeValue(std::move(updatedValue), /*updateStatus=*/true,
                                                 VehiclePropertyStore::EventMode::ALWAYS);
        if (!writeResult.ok()) {
            return StatusError(getErrorCode(writeResult))
                   << "Failed to write value into property store, error: "
                   << getErrorMsg(writeResult);
        }
    }
    return {};
}

std::optional<int32_t> FakeVehicleHardware::getSyncedAreaIdIfHvacDualOn(
        int32_t hvacTemperatureSetAreaId) const {
    auto hvacDualOnResults =
            mServerSidePropStore->readValuesForProperty(toInt(VehicleProperty::HVAC_DUAL_ON));
    if (!hvacDualOnResults.ok()) {
        return std::nullopt;
    }
    auto& hvacDualOnValues = hvacDualOnResults.value();
    for (size_t i = 0; i < hvacDualOnValues.size(); i++) {
        if ((hvacDualOnValues[i]->areaId & hvacTemperatureSetAreaId) == hvacTemperatureSetAreaId &&
            hvacDualOnValues[i]->value.int32Values.size() == 1 &&
            hvacDualOnValues[i]->value.int32Values[0] == 1) {
            return hvacDualOnValues[i]->areaId;
        }
    }
    return std::nullopt;
}

FakeVehicleHardware::ValueResultType FakeVehicleHardware::getUserHalProp(
        const VehiclePropValue& value) const {
    auto propId = value.prop;
    ALOGI("get(): getting value for prop %s from User HAL", PROP_ID_TO_CSTR(propId));

    auto result = mFakeUserHal->onGetProperty(value);
    if (!result.ok()) {
        return StatusError(getErrorCode(result))
               << "get(): User HAL returned error: " << getErrorMsg(result);
    } else {
        auto& gotValue = result.value();
        if (gotValue != nullptr) {
            ALOGI("get(): User HAL returned value: %s", gotValue->toString().c_str());
            gotValue->timestamp = elapsedRealtimeNano();
            return result;
        } else {
            return StatusError(StatusCode::INTERNAL_ERROR) << "get(): User HAL returned null value";
        }
    }
}

VhalResult<bool> FakeVehicleHardware::isCruiseControlTypeStandard() const {
    auto isCruiseControlTypeAvailableResult =
            isAdasPropertyAvailable(toInt(VehicleProperty::CRUISE_CONTROL_TYPE));
    if (!isCruiseControlTypeAvailableResult.ok()) {
        return isCruiseControlTypeAvailableResult.error();
    }
    auto cruiseControlTypeValue =
            mServerSidePropStore->readValue(toInt(VehicleProperty::CRUISE_CONTROL_TYPE));
    return cruiseControlTypeValue.value()->value.int32Values[0] ==
           toInt(CruiseControlType::STANDARD);
}

FakeVehicleHardware::ValueResultType FakeVehicleHardware::maybeGetSpecialValue(
        const VehiclePropValue& value, bool* isSpecialValue) const {
    *isSpecialValue = false;
    int32_t propId = value.prop;
    ValueResultType result;

    if (mPowerControllerServiceAddress != "") {
        if (mPowerPropIds.find(propId) != mPowerPropIds.end()) {
            *isSpecialValue = true;
            return getPowerPropFromExternalService(propId);
        }
    }

    if (mAddExtraTestVendorConfigs && propId >= STARTING_VENDOR_CODE_PROPERTIES_FOR_TEST &&
        propId < ENDING_VENDOR_CODE_PROPERTIES_FOR_TEST) {
        *isSpecialValue = true;
        result = mValuePool->obtainInt32(/* value= */ 5);

        result.value()->prop = propId;
        result.value()->areaId = 0;
        result.value()->timestamp = elapsedRealtimeNano();
        return result;
    }

    if (mFakeUserHal->isSupported(propId)) {
        *isSpecialValue = true;
        return getUserHalProp(value);
    }

    VhalResult<void> isAdasPropertyAvailableResult;

    switch (propId) {
        case OBD2_FREEZE_FRAME:
            *isSpecialValue = true;
            result = mFakeObd2Frame->getObd2FreezeFrame(value);
            if (result.ok()) {
                result.value()->timestamp = elapsedRealtimeNano();
            }
            return result;
        case OBD2_FREEZE_FRAME_INFO:
            *isSpecialValue = true;
            result = mFakeObd2Frame->getObd2DtcInfo();
            if (result.ok()) {
                result.value()->timestamp = elapsedRealtimeNano();
            }
            return result;
        case toInt(TestVendorProperty::ECHO_REVERSE_BYTES):
            *isSpecialValue = true;
            return getEchoReverseBytes(value);
        case toInt(TestVendorProperty::VENDOR_PROPERTY_FOR_ERROR_CODE_TESTING):
            *isSpecialValue = true;
            return StatusError((StatusCode)VENDOR_ERROR_CODE);
        case toInt(VehicleProperty::CRUISE_CONTROL_TARGET_SPEED):
            isAdasPropertyAvailableResult =
                    isAdasPropertyAvailable(toInt(VehicleProperty::CRUISE_CONTROL_STATE));
            if (!isAdasPropertyAvailableResult.ok()) {
                *isSpecialValue = true;
                return isAdasPropertyAvailableResult.error();
            }
            return nullptr;
        case toInt(VehicleProperty::ADAPTIVE_CRUISE_CONTROL_TARGET_TIME_GAP):
            [[fallthrough]];
        case toInt(VehicleProperty::ADAPTIVE_CRUISE_CONTROL_LEAD_VEHICLE_MEASURED_DISTANCE): {
            isAdasPropertyAvailableResult =
                    isAdasPropertyAvailable(toInt(VehicleProperty::CRUISE_CONTROL_STATE));
            if (!isAdasPropertyAvailableResult.ok()) {
                *isSpecialValue = true;
                return isAdasPropertyAvailableResult.error();
            }
            auto isCruiseControlTypeStandardResult = isCruiseControlTypeStandard();
            if (!isCruiseControlTypeStandardResult.ok()) {
                *isSpecialValue = true;
                return isCruiseControlTypeStandardResult.error();
            }
            if (isCruiseControlTypeStandardResult.value()) {
                *isSpecialValue = true;
                return StatusError(StatusCode::NOT_AVAILABLE_DISABLED)
                       << "tried to get target time gap or lead vehicle measured distance value "
                       << "while on a standard CC setting";
            }
            return nullptr;
        }
        default:
            // Do nothing.
            break;
    }

    return nullptr;
}

FakeVehicleHardware::ValueResultType FakeVehicleHardware::getPowerPropFromExternalService(
        int32_t propId) const {
    auto channel =
            grpc::CreateChannel(mPowerControllerServiceAddress, grpc::InsecureChannelCredentials());
    auto clientStub = PowerController::NewStub(channel);
    switch (propId) {
        case toInt(VehicleProperty::VEHICLE_IN_USE):
            return getVehicleInUse(clientStub.get());
        case toInt(VehicleProperty::AP_POWER_BOOTUP_REASON):
            return getApPowerBootupReason(clientStub.get());
        default:
            return StatusError(StatusCode::INTERNAL_ERROR)
                   << "Unsupported power property ID: " << propId;
    }
}

FakeVehicleHardware::ValueResultType FakeVehicleHardware::getVehicleInUse(
        PowerController::Stub* clientStub) const {
    IsVehicleInUseRequest request = {};
    IsVehicleInUseResponse response = {};
    grpc::ClientContext context;
    auto status = clientStub->IsVehicleInUse(&context, request, &response);
    if (!status.ok()) {
        return StatusError(StatusCode::TRY_AGAIN) << "Cannot connect to GRPC service "
                                                  << ", error: " << status.error_message();
    }
    auto result = mValuePool->obtainBoolean(response.isvehicleinuse());
    result->prop = toInt(VehicleProperty::VEHICLE_IN_USE);
    result->areaId = 0;
    result->status = VehiclePropertyStatus::AVAILABLE;
    result->timestamp = elapsedRealtimeNano();
    return result;
}

FakeVehicleHardware::ValueResultType FakeVehicleHardware::getApPowerBootupReason(
        PowerController::Stub* clientStub) const {
    GetApPowerBootupReasonRequest request = {};
    GetApPowerBootupReasonResponse response = {};
    grpc::ClientContext context;
    auto status = clientStub->GetApPowerBootupReason(&context, request, &response);
    if (!status.ok()) {
        return StatusError(StatusCode::TRY_AGAIN) << "Cannot connect to GRPC service "
                                                  << ", error: " << status.error_message();
    }
    auto result = mValuePool->obtainInt32(response.bootupreason());
    result->prop = toInt(VehicleProperty::AP_POWER_BOOTUP_REASON);
    result->areaId = 0;
    result->status = VehiclePropertyStatus::AVAILABLE;
    result->timestamp = elapsedRealtimeNano();
    return result;
}

FakeVehicleHardware::ValueResultType FakeVehicleHardware::getEchoReverseBytes(
        const VehiclePropValue& value) const {
    auto readResult = mServerSidePropStore->readValue(value);
    if (!readResult.ok()) {
        return readResult;
    }
    auto& gotValue = readResult.value();
    gotValue->timestamp = elapsedRealtimeNano();
    std::vector<uint8_t> byteValues = gotValue->value.byteValues;
    size_t byteSize = byteValues.size();
    for (size_t i = 0; i < byteSize; i++) {
        gotValue->value.byteValues[i] = byteValues[byteSize - 1 - i];
    }
    return std::move(gotValue);
}

void FakeVehicleHardware::sendHvacPropertiesCurrentValues(int32_t areaId, int32_t hvacPowerOnVal) {
    for (auto& powerPropId : hvacPowerDependentProps) {
        auto powerPropResults = mServerSidePropStore->readValuesForProperty(powerPropId);
        if (!powerPropResults.ok()) {
            ALOGW("failed to get power prop 0x%x, error: %s", powerPropId,
                  getErrorMsg(powerPropResults).c_str());
            continue;
        }
        auto& powerPropValues = powerPropResults.value();
        for (size_t j = 0; j < powerPropValues.size(); j++) {
            auto powerPropValue = std::move(powerPropValues[j]);
            if ((powerPropValue->areaId & areaId) == powerPropValue->areaId) {
                powerPropValue->status = hvacPowerOnVal ? VehiclePropertyStatus::AVAILABLE
                                                        : VehiclePropertyStatus::UNAVAILABLE;
                powerPropValue->timestamp = elapsedRealtimeNano();
                // This will trigger a property change event for the current hvac property value.
                mServerSidePropStore->writeValue(std::move(powerPropValue), /*updateStatus=*/true,
                                                 VehiclePropertyStore::EventMode::ALWAYS);
            }
        }
    }
}

void FakeVehicleHardware::sendAdasPropertiesState(int32_t propertyId, int32_t state) {
    auto& adasDependentPropIds = mAdasEnabledPropToAdasPropWithErrorState.find(propertyId)->second;
    for (auto dependentPropId : adasDependentPropIds) {
        auto dependentPropConfigResult = mServerSidePropStore->getPropConfig(dependentPropId);
        if (!dependentPropConfigResult.ok()) {
            ALOGW("Failed to get config for ADAS property 0x%x, error: %s", dependentPropId,
                  getErrorMsg(dependentPropConfigResult).c_str());
            continue;
        }
        auto& dependentPropConfig = dependentPropConfigResult.value();
        for (auto& areaConfig : dependentPropConfig.areaConfigs) {
            int32_t hardcoded_state = state;
            // TODO: restore old/initial values here instead of hardcoded value (b/295542701)
            if (state == 1 && dependentPropId == toInt(VehicleProperty::CRUISE_CONTROL_TYPE)) {
                hardcoded_state = toInt(CruiseControlType::ADAPTIVE);
            }
            auto propValue =
                    createAdasStateReq(dependentPropId, areaConfig.areaId, hardcoded_state);
            // This will trigger a property change event for the current ADAS property value.
            mServerSidePropStore->writeValue(std::move(propValue), /*updateStatus=*/true,
                                             VehiclePropertyStore::EventMode::ALWAYS);
        }
    }
}

VhalResult<void> FakeVehicleHardware::maybeSetSpecialValue(const VehiclePropValue& value,
                                                           bool* isSpecialValue) {
    *isSpecialValue = false;
    VehiclePropValuePool::RecyclableType updatedValue;
    int32_t propId = value.prop;

    if (mAddExtraTestVendorConfigs && propId >= STARTING_VENDOR_CODE_PROPERTIES_FOR_TEST &&
        propId < ENDING_VENDOR_CODE_PROPERTIES_FOR_TEST) {
        *isSpecialValue = true;
        return {};
    }

    if (mFakeUserHal->isSupported(propId)) {
        *isSpecialValue = true;
        return setUserHalProp(value);
    }

    if (mAdasEnabledPropToAdasPropWithErrorState.count(propId) &&
        value.value.int32Values.size() == 1) {
        if (value.value.int32Values[0] == 1) {
            // Set default state to 1 when ADAS feature is enabled.
            sendAdasPropertiesState(propId, /* state = */ 1);
        } else {
            sendAdasPropertiesState(propId, toInt(ErrorState::NOT_AVAILABLE_DISABLED));
        }
    }

    VhalResult<void> isAdasPropertyAvailableResult;
    VhalResult<bool> isCruiseControlTypeStandardResult;
    switch (propId) {
        case toInt(VehicleProperty::AP_POWER_STATE_REPORT):
            *isSpecialValue = true;
            return setApPowerStateReport(value, isSpecialValue);
        case toInt(VehicleProperty::SHUTDOWN_REQUEST):
            // If we receive SHUTDOWN_REQUEST, we should send this to an external component which
            // should shutdown Android system via sending an AP_POWER_STATE_REQ event. Here we have
            // no external components to notify, so we just send the event.
            *isSpecialValue = true;
            return setApPowerStateReqShutdown(value);
        case toInt(VehicleProperty::VEHICLE_MAP_SERVICE):
            // Placeholder for future implementation of VMS property in the default hal. For
            // now, just returns OK; otherwise, hal clients crash with property not supported.
            *isSpecialValue = true;
            return {};
        case OBD2_FREEZE_FRAME_CLEAR:
            *isSpecialValue = true;
            return mFakeObd2Frame->clearObd2FreezeFrames(value);
        case toInt(TestVendorProperty::VENDOR_PROPERTY_FOR_ERROR_CODE_TESTING):
            *isSpecialValue = true;
            return StatusError((StatusCode)VENDOR_ERROR_CODE);
        case toInt(VehicleProperty::HVAC_POWER_ON):
            if (value.value.int32Values.size() != 1) {
                *isSpecialValue = true;
                return StatusError(StatusCode::INVALID_ARG)
                       << "HVAC_POWER_ON requires only one int32 value";
            }
            // When changing HVAC power state, send current hvac property values
            // through on change event.
            sendHvacPropertiesCurrentValues(value.areaId, value.value.int32Values[0]);
            return {};
        case toInt(VehicleProperty::HVAC_TEMPERATURE_VALUE_SUGGESTION):
            *isSpecialValue = true;
            return setHvacTemperatureValueSuggestion(value);
        case toInt(VehicleProperty::HVAC_TEMPERATURE_SET):
            if (value.value.floatValues.size() != 1) {
                *isSpecialValue = true;
                return StatusError(StatusCode::INVALID_ARG)
                       << "HVAC_DUAL_ON requires only one float value";
            }
            if (auto hvacDualOnAreaId = getSyncedAreaIdIfHvacDualOn(value.areaId);
                hvacDualOnAreaId.has_value()) {
                *isSpecialValue = true;
                return synchronizeHvacTemp(hvacDualOnAreaId.value(), value.value.floatValues[0]);
            }
            return {};
        case toInt(VehicleProperty::HVAC_DUAL_ON):
            if (value.value.int32Values.size() != 1) {
                *isSpecialValue = true;
                return StatusError(StatusCode::INVALID_ARG)
                       << "HVAC_DUAL_ON requires only one int32 value";
            }
            if (value.value.int32Values[0] == 1) {
                synchronizeHvacTemp(value.areaId, std::nullopt);
            }
            return {};
        case toInt(VehicleProperty::LANE_CENTERING_ASSIST_COMMAND): {
            isAdasPropertyAvailableResult =
                    isAdasPropertyAvailable(toInt(VehicleProperty::LANE_CENTERING_ASSIST_STATE));
            if (!isAdasPropertyAvailableResult.ok()) {
                *isSpecialValue = true;
            }
            return isAdasPropertyAvailableResult;
        }
        case toInt(VehicleProperty::CRUISE_CONTROL_COMMAND):
            isAdasPropertyAvailableResult =
                    isAdasPropertyAvailable(toInt(VehicleProperty::CRUISE_CONTROL_STATE));
            if (!isAdasPropertyAvailableResult.ok()) {
                *isSpecialValue = true;
                return isAdasPropertyAvailableResult;
            }
            isCruiseControlTypeStandardResult = isCruiseControlTypeStandard();
            if (!isCruiseControlTypeStandardResult.ok()) {
                *isSpecialValue = true;
                return isCruiseControlTypeStandardResult.error();
            }
            if (isCruiseControlTypeStandardResult.value() &&
                (value.value.int32Values[0] ==
                         toInt(CruiseControlCommand::INCREASE_TARGET_TIME_GAP) ||
                 value.value.int32Values[0] ==
                         toInt(CruiseControlCommand::DECREASE_TARGET_TIME_GAP))) {
                *isSpecialValue = true;
                return StatusError(StatusCode::NOT_AVAILABLE_DISABLED)
                       << "tried to use a change target time gap command while on a standard CC "
                       << "setting";
            }
            return {};
        case toInt(VehicleProperty::ADAPTIVE_CRUISE_CONTROL_TARGET_TIME_GAP): {
            isAdasPropertyAvailableResult =
                    isAdasPropertyAvailable(toInt(VehicleProperty::CRUISE_CONTROL_STATE));
            if (!isAdasPropertyAvailableResult.ok()) {
                *isSpecialValue = true;
                return isAdasPropertyAvailableResult;
            }
            isCruiseControlTypeStandardResult = isCruiseControlTypeStandard();
            if (!isCruiseControlTypeStandardResult.ok()) {
                *isSpecialValue = true;
                return isCruiseControlTypeStandardResult.error();
            }
            if (isCruiseControlTypeStandardResult.value()) {
                *isSpecialValue = true;
                return StatusError(StatusCode::NOT_AVAILABLE_DISABLED)
                       << "tried to set target time gap or lead vehicle measured distance value "
                       << "while on a standard CC setting";
            }
            return {};
        }

#ifdef ENABLE_VEHICLE_HAL_TEST_PROPERTIES
        case toInt(VehicleProperty::CLUSTER_REPORT_STATE):
            [[fallthrough]];
        case toInt(VehicleProperty::CLUSTER_REQUEST_DISPLAY):
            [[fallthrough]];
        case toInt(VehicleProperty::CLUSTER_NAVIGATION_STATE):
            [[fallthrough]];
        case toInt(TestVendorProperty::VENDOR_CLUSTER_SWITCH_UI):
            [[fallthrough]];
        case toInt(TestVendorProperty::VENDOR_CLUSTER_DISPLAY_STATE):
            *isSpecialValue = true;
            updatedValue = mValuePool->obtain(getPropType(value.prop));
            updatedValue->prop = value.prop & ~toInt(VehiclePropertyGroup::MASK);
            if (getPropGroup(value.prop) == VehiclePropertyGroup::SYSTEM) {
                updatedValue->prop |= toInt(VehiclePropertyGroup::VENDOR);
            } else {
                updatedValue->prop |= toInt(VehiclePropertyGroup::SYSTEM);
            }
            updatedValue->value = value.value;
            updatedValue->timestamp = elapsedRealtimeNano();
            updatedValue->areaId = value.areaId;
            // if (auto writeResult = mServerSidePropStore->writeValue(std::move(updatedValue));
            if (auto writeResult = writeValueAndNotifyChange(*updatedValue, false);
                !writeResult.ok()) {
                return StatusError(getErrorCode(writeResult))
                       << "failed to write value into property store, error: "
                       << getErrorMsg(writeResult);
            }
            return {};
#endif  // ENABLE_VEHICLE_HAL_TEST_PROPERTIES
        case HONDA_CUSTOMIZE_IN_CUSTOM_MODE:
            EMULOG_I("HONDA_CUSTOMIZE_IN_CUSTOM_MODE [value:%d]", value.value.byteValues[0]);
            *isSpecialValue = true;
            return {};
        case HONDA_CUSTOMIZE_REQUEST:
        case HONDA_CUSTOMIZE_CUSTOM_GET_REQUEST:
        case HONDA_CUSTOMIZE_CUSTOM_SET_REQUEST:
            mEmulatedCustomizeThread.push_task(value);
            *isSpecialValue = true;
            return {};
        case HONDA_CUSTOMIZE_ACC_BUZZER_SET:
        case HONDA_CUSTOMIZE_DWS_INIT_REQUEST:
        case HONDA_CUSTOMIZE_LKAS_BUZZER_SET:
        case HONDA_CUSTOMIZE_CMBS_ALART_SET:
        case HONDA_CUSTOMIZE_SIF_SET:
        case HONDA_CUSTOMIZE_CDC_INTELLIGENT_LED_CST_STT:
        case HONDA_CUSTOMIZE_RDMS_SET:
        case HONDA_CUSTOMIZE_DAAS_SET:
        case HONDA_CUSTOMIZE_BSI_SCM_CUST_SET:
        case HONDA_CUSTOMIZE_ALC_CUST_SET:
        case HONDA_CUSTOMIZE_HSS_ADB_SET:
        case HONDA_CUSTOMIZE_TSR_DISP_ONOFF_SET:
        case HONDA_CUSTOMIZE_TSR_WARN_STATUS_SET:
        case HONDA_CUSTOMIZE_TSR_WARN_KPH_SET:
        case HONDA_CUSTOMIZE_TSR_WARN_MPH_SET:
        case HONDA_CUSTOMIZE_EAS_LC_SET:
        case HONDA_CUSTOMIZE_EAS_PH_SET:
        case HONDA_CUSTOMIZE_FCTW_ATT_SET:
        case HONDA_CUSTOMIZE_ACC_CSA_SET:
        case HONDA_CUSTOMIZE_PKS_RR_SET:
        case HONDA_CUSTOMIZE_IACC_SETUP_SET:
        case HONDA_CUSTOMIZE_REV_MATCH_SET:
        case HONDA_CUSTOMIZE_PEAEMG_SET:
        case HONDA_CUSTOMIZE_TSR_SRBUZ_SET:
        case HONDA_CUSTOMIZE_IDS_ENGINE_SET:
        case HONDA_CUSTOMIZE_IDS_SUSPENSION_SET:
        case HONDA_CUSTOMIZE_IDS_STEERING_SET:
        case HONDA_CUSTOMIZE_IDS_GAUGE_SET:
        case HONDA_CUSTOMIZE_IDS_IDLESTOP_SET:
        case HONDA_CUSTOMIZE_IDS_REVMATCH_SET:
        case HONDA_CUSTOMIZE_IDS_PTSOUND_SET:
        case HONDA_CUSTOMIZE_IDS_ACC_SET:
        case HONDA_CUSTOMIZE_IDS_LIGHTING_SET:
        case HONDA_CUSTOMIZE_IDS_ALL_SET:
        case HONDA_EVS_ONOFF_REQ:
        case HONDA_EVS_IDS_COLLABO_REQ:
        case HONDA_EVS_SOUND_TYPE_REQ:
        case HONDA_EVS_VOLUME_REQ:
        case HONDA_EVS_BALANCE_TYPE_ALL_REQ:
        case HONDA_EVS_BALANCE_TYPE_DR_REQ:
        case HONDA_EVS_BALANCE_TYPE_FR_REQ:
        case HONDA_EVS_BALANCE_TYPE_RR_REQ:
        case HONDA_EVS_SOUND_TYPE_PRESET_REQ:
        case HONDA_CUSTOMIZE_APS_AUTO_EPB_SET:
        case HONDA_CUSTOMIZE_APS_DETECT_SOUND_SET:
        case HONDA_CUSTOMIZE_APS_MEMORY_DELE_SET:
        case HONDA_ROFMOD_THEME_COLOR_SET:
        case HONDA_ROFMOD_ROOMLIGHT_SET:
        case HONDA_ROFMOD_BRI_SET:
        case HONDA_LID_DOORLOCK_SW_SET:
        case HONDA_CUSTOMIZE_ELM_RRFOG_CHANGE_MODE_SET:
        case HONDA_CUSTOMIZE_ELM_LIGHT_CHANGE_MODE_SET:
        case HONDA_CUSTOMIZE_ELM_FRFOG_CHANGE_MODE_SET:
        case HONDA_CUSTOMIZE_DPMS_DR_MASG_MODE_SET:
        case HONDA_CUSTOMIZE_DPMS_AS_MASG_MODE_SET:
        case HONDA_CUSTOMIZE_DPMS_MIRCOUNT1_SET:
        case HONDA_CUSTOMIZE_DPMS_MIR_SELECT_SET:
        case HONDA_CUSTOMIZE_DPMS_MIR_REVERSE_SET:
        case HONDA_ROFMOD_SETTING_FB_SET:
        case HONDA_METER_LANGUAGE_SEND_REQUEST:
        case HONDA_CUSTOMIZE_GAUGE_PATTERN_SET:
        case HONDA_CUSTOMIZE_WARNING_MSG_SET:
        case HONDA_CUSTOMIZE_EXTEMP_DISP_C_SET:
        case HONDA_CUSTOMIZE_EXTEMP_DISP_F_SET:
        case HONDA_CUSTOMIZE_MTGEAR_DISP_SET:
        case HONDA_CUSTOMIZE_TRIPA_RESET_GAS_SET:
        case HONDA_CUSTOMIZE_TRIPA_RESET_PHEV_SET:
        case HONDA_CUSTOMIZE_TRIPA_RESET_BEV_SET:
        case HONDA_CUSTOMIZE_TRIPA_RESET_FCV_SET:
        case HONDA_CUSTOMIZE_TRIPB_RESET_GAS_SET:
        case HONDA_CUSTOMIZE_TRIPB_RESET_PHEV_SET:
        case HONDA_CUSTOMIZE_TRIPB_RESET_BEV_SET:
        case HONDA_CUSTOMIZE_TRIPB_RESET_FCV_SET:
        case HONDA_CUSTOMIZE_ALARM_VOL_SET:
        case HONDA_CUSTOMIZE_REVERSE_ALARM_SET:
        case HONDA_CUSTOMIZE_REV_INDICATOR_SET:
        case HONDA_CUSTOMIZE_AMBIENT_METER_SET:
        case HONDA_CUSTOMIZE_IDLESTOP_GUIDANCE_SET:
        case HONDA_CUSTOMIZE_TBT_DISPLAY_SET:
        case HONDA_CUSTOMIZE_REARSEAT_REMINDER_SET:
        case HONDA_CUSTOMIZE_DRIVERANGE_NOTIFY_SET:
        case HONDA_CUSTOMIZE_TIREANGLE_MONITOR_SET:
        case HONDA_CUSTOMIZE_DRIVE_UNIT_SET:
        case HONDA_CUSTOMIZE_CONSUMPTION_UNIT_KM_SET:
        case HONDA_CUSTOMIZE_SHIFTUP_TIMING_ALARM_SET:
        case HONDA_CUSTOMIZE_LANGUAGE_EU_SET:
        case HONDA_CUSTOMIZE_LANGUAGE_US_SET:
        case HONDA_CUSTOMIZE_LANGUAGE_CN_SET:
        case HONDA_CUSTOMIZE_LANGUAGE_PT_SET:
        case HONDA_CUSTOMIZE_LANGUAGE_KR_SET:
        case HONDA_CUSTOMIZE_GAUGE_PATTERN_PF3_SET:
        case HONDA_CUSTOMIZE_TBT_DISPLAY_HUD_SET:
        case HONDA_CUSTOMIZE_ADAS_DISPLAY_HUD_SET:
        case HONDA_CUSTOMIZE_SPEEDTSR_DISPLAY_HUD_SET:
        case HONDA_CUSTOMIZE_SPEED_DISPLAY_HUD_SET:
        case HONDA_CUSTOMIZE_DETAIL_DISPLAY_HUD_SET:
        case HONDA_CUSTOMIZE_TRAIL_GAUGE_SET:
        case HONDA_CUSTOMIZE_PS_MET_MID:
        case HONDA_CUSTOMIZE_SET_DID_SETTING:
        case HONDA_CUSTOMIZE_SW_TURN_MODE_SET:
        case HONDA_CUSTOMIZE_EAS_PARK_MODE_SET:
        case HONDA_CUSTOMIZE_EAS_WELCOME_MODE_SET:
        case HONDA_CUSTOMIZE_IDS_RESET2:
        case HONDA_CUSTOMIZE_IDS_ENGINE2_SET:
        case HONDA_CUSTOMIZE_IDS_STEERING2_SET:
        case HONDA_CUSTOMIZE_IDS_SUSPENSION2_SET:
        case HONDA_CUSTOMIZE_IDS_AWD2_SET:
        case HONDA_CUSTOMIZE_IDS_PTSOUND2_SET:
        case HONDA_CUSTOMIZE_ADAS_NOTIF_LED_CST_STT_SET:
        case HONDA_CUSTOMIZE_TLI_RED_FUN_SET:
        case HONDA_CUSTOMIZE_TLI_GREEN_FUN_SET:
        case HONDA_METER_HUD_ONOFF_SET:
        case HONDA_NAVI_DIMMING_GLASS_REQ:
        case HONDA_NAVI_DIMMING_GLASS_REQ_PS:
        case HONDA_CUSTOMIZE_HVAC_LEARN_AUTO_FUNC_SET:
        case HONDA_CUSTOMIZE_MVC_INTERLOCK_SET:
        case HONDA_CUSTOMIZE_REFERLINE_SET:
        case HONDA_CUSTOMIZE_HUD_CONTENTS_SET:
        case HONDA_CUSTOMIZE_DID_CONTENTS_SET:
        case HONDA_CUSTOMIZE_TBT_DISPLAY_DID_SET:
        case HONDA_CUSTOMIZE_AMN_DISPLAY_DID_SET:
        case HONDA_CUSTOMIZE_AUDIO_DISPLAY_DID_SET:
        case HONDA_CUSTOMIZE_HFL_DISPLAY_DID_SET:
        case HONDA_CUSTOMIZE_SR_DISPLAY_DID_SET:
        case HONDA_CUSTOMIZE_SMS_DISPLAY_DID_SET:
        case HONDA_CUSTOMIZE_COMPANION_DISPLAY_DID_SET:
        case HONDA_ENTRY_NAVI_CHILDCHECK_REQ:
        case HONDA_LID_OPCL_SW_SET:
        case HONDA_LID_DR_OPCL_SW_SET:
        case HONDA_LID_AS_OPCL_SW_SET:
        case HONDA_LID_RD_OPCL_SW_SET:
        case HONDA_LID_RA_OPCL_SW_SET:
        case HONDA_CUSTOMIZE_CDC_AD_MAIN_STT_SET:
        case HONDA_CUSTOMIZE_CDC_ACC_CC_MODE_STT_SET:
        case HONDA_CUSTOMIZE_CDC_ACC_LIM_MODE_STT_SET:
        case HONDA_CUSTOMIZE_CDC_LKAS_ENABLE_STT_SET:
        case HONDA_CUSTOMIZE_AHD_CUST_ONOFF_SET:
        case HONDA_CUSTOMIZE_CDC_ACC_DIST_STT_SET:
        case HONDA_CUSTOMIZE_CDC_ACC_SLA_START_SPEED_STT_SET:
        case HONDA_CUSTOMIZE_CDC_IACC_ONOFF_STT_SET:
        case HONDA_CUSTOMIZE_CDC_ACC_SLA_MODE_WO_AUTO_STT_SET:
        case HONDA_CUSTOMIZE_CDC_ACC_SLA_MODE_W_AUTO_STT_SET:
        case HONDA_CUSTOMIZE_CDC_ACC_SLA_OFFSET_KPH_STT_SET:
        case HONDA_CUSTOMIZE_CDC_ACC_SLA_OFFSET_MPH_STT_SET:
        case HONDA_CUSTOMIZE_CDC_ACC_BLINK_AXEL_STT_SET:
        case HONDA_CUSTOMIZE_CDC_ACC_RESUME_ONOFF_STT_SET:
        case HONDA_CUSTOMIZE_CDC_ACC_AXELMODE_STT_SET:
        case HONDA_CUSTOMIZE_CDC_ADA_ONOFF_STT_SET:
        case HONDA_CUSTOMIZE_CDC_ADA_FUNC_STT_SET:
        case HONDA_CUSTOMIZE_CDC_ADA_LONINTENSITY_STT_SET:
        case HONDA_CUSTOMIZE_CDC_ADA_LONCURVE_STT_SET:
        case HONDA_CUSTOMIZE_CDC_ADA_LATINTENSITY_STT_SET:
        case HONDA_CUSTOMIZE_CDC_AILD_HANDS_OFF_ALARM_STT_SET:
        case HONDA_CUSTOMIZE_CDC_ALCA_GAP_SEARCH_CUST_STT_SET:
        case HONDA_CUSTOMIZE_CDC_ALCA_TURN_LEVER_CUST_STT_SET:
        case HONDA_CUSTOMIZE_ALCA_ALC_CUST_SET:
        case HONDA_CUSTOMIZE_CDC_ALCR_ALC_ONOFF_CUST_STT_SET:
        case HONDA_CUSTOMIZE_ALCR_PAS_CUST_RESULT_SET:
        case HONDA_CUSTOMIZE_CDC_ALC_NAVI_CUST_STT_SET:
        case HONDA_CUSTOMIZE_ALCR_NAV_CUST_RESULT_SET:
        case HONDA_CUSTOMIZE_CDC_AMN_ONOFF_STT_SET:
        case HONDA_CUSTOMIZE_CDC_AMN_NAV_TIMING_STT_SET:
        case HONDA_CUSTOMIZE_CDC_AMN_HOV_ENABLE_STT_SET:
        case HONDA_CUSTOMIZE_CDC_AMN_EXPRESS_ENABLE_STT_SET:
        case HONDA_CUSTOMIZE_CDC_AMN_INTERRUPT_DISP_STT_SET:
        case HONDA_CUSTOMIZE_ADAS_NOTIF_VIB_CST_STT_SET:
        case HONDA_CUSTOM_MENU_BIOENT_CONTROLLER:
        case HONDA_CUSTOMIZE_WELCOME_SOUND_SET:
        case HONDA_CUSTOM_ADAS_E_BSI_ONOFF_SET:
        case HONDA_CUSTOM_ADAS_E_BSI_RANGE_SET:
        case HONDA_CUSTOM_ADAS_E_BSI_2R_SET:
        case HONDA_CUSTOMIZE_C_LID_AUTO_SET:
        case HONDA_CUSTOMIZE_EC_LID_CLOSE_TIMER_SET:
        case HONDA_CUSTOMIZE_EC_LID_AUTO_CLOSE_SET:
        case HONDA_CUSTOM_ADAS_STATUS_CMBS_SET:
        case HONDA_CUSTOM_ADAS_FCTW_SET:
        case HONDA_CUSTOM_DAM_ALLOFF_SW_SET:
        case HONDA_CUSTOM_ADAS_AILD_ENABLE_SET:
        case HONDA_CUSTOM_ADAS_ALC_ONOFF_CUST_SET:
        case HONDA_CUSTOM_ADAS_E_BSI_CUST_SET:
        case HONDA_CUSTOM_ADAS_SAFETY_SW_STATUS_PKS_SET:
        case HONDA_CUSTOM_ADAS_SAFETY_SW_STATUS_LSAEB_SET:
        case HONDA_CUSTOM_ADAS_PKS_RR_STATUS_SET:
        case HONDA_CORE_CUSTOM_SNP_CRP_SET:
        case HONDA_CUSTOMIZE_RDMS_ONE_SET:
        case HONDA_CUSTOMIZE_RDMS_TWO_SET:
        case HONDA_CUSTOMIZE_EW_CUST_RESULT_SET:
        case HONDA_CUSTOMIZE_DMC_DROWSY_SETTING_SET:
        case HONDA_CUSTOMIZE_DMC_DIST_SETTING_SET:
        case HONDA_CUSTOMIZE_CDC_AES_ONOFF_STT_SET:
        case HONDA_CUSTOMIZE_CDC_DESS_CUST_STT_SET:
        case HONDA_CUSTOMIZE_CDC_EW_FUNC_ONOFF_STT_SET:
        case HONDA_CUSTOMIZE_CDC_EW_LV1_ONOFF_STT_SET:
        case HONDA_CUSTOMIZE_CDC_EW_ELATCH_ONOFF_STT_SET:
        case HONDA_CUSTOMIZE_CDC_EW_ELATCH_SENSE_STT_SET:
        case HONDA_CUSTOMIZE_BSINFO_VCHANGE_BCF_SET:
        case HONDA_CUSTOMIZE_ALARM_TYPE_SET:
        case HONDA_CUSTOMIZE_CDC_BSI_BUZZER_VOLUME_STT_SET:
        case HONDA_CUSTOMIZE_CDC_BSI_RANGE_STT_SET:
        case HONDA_CUSTOMIZE_CDC_BSI_2R_STT_SET:
        case HONDA_CUSTOMIZE_CDC_BSI_TRAILER_STT_SET:
        case HONDA_CUSTOMIZE_CDC_BSI_TRAILER_TYPE_STT_SET:
        case HONDA_CUSTOMIZE_CDC_BSI_TRAILER_WIDTH_STT_SET:
        case HONDA_CUSTOMIZE_CDC_BSI_TRAILER_LENGTH_STT_SET:
        case HONDA_CUSTOMIZE_CDC_PCDW_ONOFF_STT_SET:
        case HONDA_CUSTOMIZE_ODSF_ONOFF_SET:
        case HONDA_CUSTOMIZE_METER_TSR_SRBUZ_ONOFF_SET:
        case HONDA_CUSTOMIZE_DAM_EYECLOSED_STT_SET:
        case HONDA_METER_HUD_ILLUMI_SET:
        case HONDA_CUSTOMIZE_VOL_DISPLAY_DID_SET:
        case HONDA_CUSTOMIZE_HEADLIGHT_TIMER_SET:
        case HONDA_CUSTOMIZE_WELCOM_LIGHT_1_SET:
        case HONDA_CUSTOMIZE_WELCOM_LIGHT_2_SET:
        case HONDA_CUSTOMIZE_WELCOM_LIGHT_1_2_SET:
        case HONDA_EVS_BALANCE_STAT_SET:
            mEmulatedCustomizeThread.push_task(value);
            *isSpecialValue = true;
            return {};
        case HONDA_CUSTOMIZE_ACC_BUZZER:
        case HONDA_CUSTOMIZE_DWS_INIT_REQUEST_RECEIVED:
        case HONDA_CUSTOMIZE_LKAS_BUZZER:
        case HONDA_CUSTOMIZE_CMBS_ALART:
        case HONDA_CUSTOMIZE_SIF:
        case HONDA_CUSTOMIZE_INTELLIGENT_LED_CST_STT:
        case HONDA_CUSTOMIZE_RDMS:
        case HONDA_CUSTOMIZE_DAAS:
        case HONDA_CUSTOMIZE_BSI_SCM_CUST:
        case HONDA_CUSTOMIZE_ALC_CUST:
        case HONDA_CUSTOMIZE_HSS:
        case HONDA_CUSTOMIZE_ADB:
        case HONDA_CUSTOMIZE_EAS_LC:
        case HONDA_CUSTOMIZE_EAS_PH:
        case HONDA_CUSTOMIZE_FCTW_ATT:
        case HONDA_CUSTOMIZE_ACC_CSA:
        case HONDA_CUSTOMIZE_PKS_RR:
        case HONDA_CUSTOMIZE_IACC_SETUP:
        case HONDA_CUSTOMIZE_REV_MATCH:
        case HONDA_CUSTOMIZE_PEAEMG:
        case HONDA_CUSTOMIZE_TSR_SRBUZ:
        case HONDA_CUSTOMIZE_IDS_ENGINE:
        case HONDA_CUSTOMIZE_IDS_SUSPENSION:
        case HONDA_CUSTOMIZE_IDS_STEERING:
        case HONDA_CUSTOMIZE_IDS_GAUGE:
        case HONDA_CUSTOMIZE_IDS_IDLESTOP:
        case HONDA_CUSTOMIZE_IDS_REVMATCH:
        case HONDA_CUSTOMIZE_IDS_PTSOUND:
        case HONDA_CUSTOMIZE_IDS_ACC:
        case HONDA_CUSTOMIZE_IDS_LIGHTING:
        case HONDA_CUSTOMIZE_TSR_DISP_ONOFF:
        case HONDA_CUSTOMIZE_TSR_WARN_STATUS:
        case HONDA_CUSTOMIZE_TSR_WARN_KPH:
        case HONDA_CUSTOMIZE_TSR_WARN_MPH:
        case HONDA_EVS_ONOFF_STATUS:
        case HONDA_EVS_IDS_COLLABO_STATUS:
        case HONDA_EVS_SOUND_TYPE_STATUS:
        case HONDA_EVS_VOLUME_STATUS:
        case HONDA_EVS_BALANCE_TYPE_ALL_STATUS:
        case HONDA_EVS_BALANCE_TYPE_DR_STATUS:
        case HONDA_EVS_BALANCE_TYPE_FR_STATUS:
        case HONDA_EVS_BALANCE_TYPE_RR_STATUS:
        case HONDA_EVS_SOUND_TYPE_PRESET_STATUS:
        case HONDA_CUSTOMIZE_APS_AUTO_EPB:
        case HONDA_CUSTOMIZE_APS_DETECT_SOUND:
        case HONDA_CUSTOMIZE_APS_MEMORY_DELE:
        case HONDA_ROFMOD_THEME_COLOR:
        case HONDA_ROFMOD_ROOMLIGHT:
        case HONDA_ROFMOD_BRI:
        case HONDA_LID_DOORLOCK_SW:
        case HONDA_CUSTOMIZE_ELM_RRFOG_CHANGE_MODE:
        case HONDA_CUSTOMIZE_ELM_LIGHT_CHANGE_MODE:
        case HONDA_CUSTOMIZE_ELM_FRFOG_CHANGE_MODE:
        case HONDA_CUSTOMIZE_DPMS_DR_MASG_MODE:
        case HONDA_CUSTOMIZE_DPMS_AS_MASG_MODE:
        case HONDA_CUSTOMIZE_DPMS_MIRCOUNT1:
        case HONDA_CUSTOMIZE_DPMS_MIR_SELECT:
        case HONDA_CUSTOMIZE_DPMS_MIR_REVERSE:
        case HONDA_ROFMOD_SETTING_FB:
        case HONDA_METER_LANGUAGE_RECEIVE_RESPONSE:
        case HONDA_CUSTOMIZE_GAUGE_PATTERN:
        case HONDA_CUSTOMIZE_WARNING_MSG:
        case HONDA_CUSTOMIZE_EXTEMP_DISP_C:
        case HONDA_CUSTOMIZE_EXTEMP_DISP_F:
        case HONDA_CUSTOMIZE_MTGEAR_DISP:
        case HONDA_CUSTOMIZE_TRIPA_RESET_GAS:
        case HONDA_CUSTOMIZE_TRIPA_RESET_PHEV:
        case HONDA_CUSTOMIZE_TRIPA_RESET_BEV:
        case HONDA_CUSTOMIZE_TRIPA_RESET_FCV:
        case HONDA_CUSTOMIZE_TRIPB_RESET_GAS:
        case HONDA_CUSTOMIZE_TRIPB_RESET_PHEV:
        case HONDA_CUSTOMIZE_TRIPB_RESET_BEV:
        case HONDA_CUSTOMIZE_TRIPB_RESET_FCV:
        case HONDA_CUSTOMIZE_ALARM_VOL:
        case HONDA_CUSTOMIZE_REVERSE_ALARM:
        case HONDA_CUSTOMIZE_REV_INDICATOR:
        case HONDA_CUSTOMIZE_AMBIENT_METER:
        case HONDA_CUSTOMIZE_IDLESTOP_GUIDANCE:
        case HONDA_CUSTOMIZE_TBT_DISPLAY:
        case HONDA_CUSTOMIZE_REARSEAT_REMINDER:
        case HONDA_CUSTOMIZE_DRIVERANGE_NOTIFY:
        case HONDA_CUSTOMIZE_TIREANGLE_MONITOR:
        case HONDA_CUSTOMIZE_DRIVE_UNIT:
        case HONDA_CUSTOMIZE_CONSUMPTION_UNIT_KM:
        case HONDA_CUSTOMIZE_SHIFTUP_TIMING_ALARM:
        case HONDA_CUSTOMIZE_LANGUAGE_EU:
        case HONDA_CUSTOMIZE_LANGUAGE_US:
        case HONDA_CUSTOMIZE_LANGUAGE_CN:
        case HONDA_CUSTOMIZE_LANGUAGE_PT:
        case HONDA_CUSTOMIZE_LANGUAGE_KR:
        case HONDA_CUSTOMIZE_GAUGE_PATTERN_PF3:
        case HONDA_CUSTOMIZE_TBT_DISPLAY_HUD:
        case HONDA_CUSTOMIZE_ADAS_DISPLAY_HUD:
        case HONDA_CUSTOMIZE_SPEEDTSR_DISPLAY_HUD:
        case HONDA_CUSTOMIZE_SPEED_DISPLAY_HUD:
        case HONDA_CUSTOMIZE_DETAIL_DISPLAY_HUD:
        case HONDA_CUSTOMIZE_TRAIL_GAUGE:
        case HONDA_CUSTOMIZE_MET_PS_MID:
        case HONDA_CUSTOMIZE_GET_DID_SETTING:
        case HONDA_CUSTOMIZE_SW_TURN_MODE:
        case HONDA_CUSTOMIZE_EAS_PARK_MODE:
        case HONDA_CUSTOMIZE_EAS_WELCOME_MODE:
        case HONDA_CUSTOMIZE_IDS_RESET_STATUS2:
        case HONDA_CUSTOMIZE_IDS_ENGINE2:
        case HONDA_CUSTOMIZE_IDS_STEERING2:
        case HONDA_CUSTOMIZE_IDS_SUSPENSION2:
        case HONDA_CUSTOMIZE_IDS_AWD2:
        case HONDA_CUSTOMIZE_PW_SW_LOCK:
        case HONDA_CUSTOMIZE_SUNLOOF_KEYLESS:
        case HONDA_CUSTOMIZE_WIP_MAINTENANCE_MODE:
        case HONDA_CUSTOMIZE_SEAT_DIRECTION:
        case HONDA_CUSTOMIZE_SEATCOLUMN_DIRECTION:
        case HONDA_CUSTOMIZE_PW_SW_LOCK_SET:
        case HONDA_CUSTOMIZE_SUNLOOF_KEYLESS_SET:
        case HONDA_CUSTOMIZE_WIP_MAINTENANCE_MODE_SET:
        case HONDA_CUSTOMIZE_SEAT_DIRECTION_SET:
        case HONDA_CUSTOMIZE_SEATCOLUMN_DIRECTION_SET:
        case HONDA_CUSTOMIZE_IDS_PTSOUND2:
        case HONDA_CUSTOMIZE_DPMS_DR_MASG_STATUS:
        case HONDA_CUSTOMIZE_DPMS_AS_MASG_STATUS:
        case HONDA_HVAC_TEMPVARIATION:
        case HONDA_HVAC_RR_FUNC_FEATURE:
        case HONDA_CUSTOMIZE_ADAS_NOTIF_LED_CST_STT:
        case HONDA_CUSTOMIZE_TLI_RED_FUN:
        case HONDA_CUSTOMIZE_TLI_GREEN_FUN:
        case HONDA_METER_HUD_ONOFF:
        case HONDA_DIMMING_GLASS_STATE:
        case HONDA_CUSTOMIZE_HVAC_LEARN_AUTO_FUNC:
        case HONDA_CUSTOMIZE_MVC_INTERLOCK:
        case HONDA_CUSTOMIZE_REFERLINE:
        case HONDA_CUSTOMIZE_HUD_CONTENTS:
        case HONDA_CUSTOMIZE_DID_CONTENTS:
        case HONDA_CUSTOMIZE_TBT_DISPLAY_DID:
        case HONDA_CUSTOMIZE_AMN_DISPLAY_DID:
        case HONDA_CUSTOMIZE_AUDIO_DISPLAY_DID:
        case HONDA_CUSTOMIZE_HFL_DISPLAY_DID:
        case HONDA_CUSTOMIZE_SR_DISPLAY_DID:
        case HONDA_CUSTOMIZE_SMS_DISPLAY_DID:
        case HONDA_CUSTOMIZE_COMPANION_DISPLAY_DID:
        case HONDA_ENTRY_CPD_VEHICLE_CONFIG:
        case HONDA_LID_DR_OPCL_SW:
        case HONDA_LID_AS_OPCL_SW:
        case HONDA_LID_RD_OPCL_SW:
        case HONDA_LID_RA_OPCL_SW:
        case HONDA_CUSTOMIZE_AD_MAIN_STT:
        case HONDA_CUSTOMIZE_ACC_CC_MODE_STT:
        case HONDA_CUSTOMIZE_ACC_LIM_MODE_STT:
        case HONDA_CUSTOMIZE_LKAS_ENABLE_STT:
        case HONDA_CUSTOMIZE_AHD_CUST_ONOFF:
        case HONDA_CUSTOMIZE_ACC_DIST_STT:
        case HONDA_CUSTOMIZE_ACC_SLA_START_SPEED_STT:
        case HONDA_CUSTOMIZE_IACC_ONOFF_STT:
        case HONDA_CUSTOMIZE_ACC_SLA_MODE_WO_AUTO_STT:
        case HONDA_CUSTOMIZE_ACC_SLA_MODE_W_AUTO_STT:
        case HONDA_CUSTOMIZE_ACC_SLA_OFFSET_KPH_STT:
        case HONDA_CUSTOMIZE_ACC_SLA_OFFSET_MPH_STT:
        case HONDA_CUSTOMIZE_ACC_BLINK_AXEL_STT:
        case HONDA_CUSTOMIZE_ACC_RESUME_ONOFF_STT:
        case HONDA_CUSTOMIZE_ACC_AXELMODE_STT:
        case HONDA_CUSTOMIZE_ADA_ONOFF_STT:
        case HONDA_CUSTOMIZE_ADA_FUNC_STT:
        case HONDA_CUSTOMIZE_ADA_LONINTENSITY_STT:
        case HONDA_CUSTOMIZE_ADA_LONCURVE_STT:
        case HONDA_CUSTOMIZE_ADA_LATINTENSITY_STT:
        case HONDA_CUSTOMIZE_AILD_HANDS_OFF_ALARM_STT:
        case HONDA_CUSTOMIZE_ALCA_GAP_SEARCH_CUST_STT:
        case HONDA_CUSTOMIZE_ALCA_TURN_LEVER_CUST_STT:
        case HONDA_CUSTOMIZE_ALCA_ALC_CUST:
        case HONDA_CUSTOMIZE_ALCR_ALC_ONOFF_CUST_STT:
        case HONDA_CUSTOMIZE_ALCR_PAS_CUST_RESULT:
        case HONDA_CUSTOMIZE_ALC_NAVI_CUST_STT:
        case HONDA_CUSTOMIZE_ALCR_NAV_CUST_RESULT:
        case HONDA_CUSTOMIZE_AMN_ONOFF_STT:
        case HONDA_CUSTOMIZE_AMN_NAV_TIMING_STT:
        case HONDA_CUSTOMIZE_AMN_HOV_ENABLE_STT:
        case HONDA_CUSTOMIZE_AMN_EXPRESS_ENABLE_STT:
        case HONDA_CUSTOMIZE_AMN_INTERRUPT_DISP_STT:
        case HONDA_CUSTOMIZE_ADAS_NOTIF_VIB_CST_STT:
        case HONDA_BIOENT_CUSTOM_MENU_BIOENT_INTERFACE:
        case HONDA_CUSTOMIZE_WELCOME_SOUND:
        case HONDA_CUSTOM_ADAS_E_BSI_ONOFF:
        case HONDA_CUSTOM_ADAS_E_BSI_RANGE:
        case HONDA_CUSTOM_ADAS_E_BSI_2R:
        case HONDA_CUSTOMIZE_C_LID_AUTO:
        case HONDA_CUSTOMIZE_EC_LID_CLOSE_TIMER:
        case HONDA_CUSTOMIZE_EC_LID_AUTO_CLOSE:
        case HONDA_CUSTOM_ADAS_STATUS_CMBS:
        case HONDA_CUSTOM_ADAS_FCTW:
        case HONDA_CUSTOM_DAM_ALLOFF_SW:
        case HONDA_CUSTOM_ADAS_AILD_ENABLE:
        case HONDA_CUSTOM_ADAS_ALC_ONOFF_CUST:
        case HONDA_CUSTOM_ADAS_E_BSI_CUST:
        case HONDA_CUSTOM_ADAS_SAFETY_SW_STATUS_PKS:
        case HONDA_CUSTOM_ADAS_SAFETY_SW_STATUS_LSAEB:
        case HONDA_CUSTOM_ADAS_PKS_RR_STATUS:
        case HONDA_CORE_CUSTOM_SNP_CRP:
        case HONDA_CUSTOMIZE_RDMS_ONE:
        case HONDA_CUSTOMIZE_RDMS_TWO:
        case HONDA_CUSTOMIZE_EW_CUST_RESULT:
        case HONDA_CUSTOMIZE_DMC_DROWSY_SETTING:
        case HONDA_CUSTOMIZE_DMC_DIST_SETTING:
        case HONDA_CUSTOMIZE_AES_ONOFF_STT:
        case HONDA_CUSTOMIZE_DESS_CUST_STT:
        case HONDA_CUSTOMIZE_EW_FUNC_ONOFF_STT:
        case HONDA_CUSTOMIZE_EW_LV1_ONOFF_STT:
        case HONDA_CUSTOMIZE_EW_ELATCH_ONOFF_STT:
        case HONDA_CUSTOMIZE_EW_ELATCH_SENSE_STT:
        case HONDA_CUSTOMIZE_BSINFO_VCHANGE_BCF:
        case HONDA_CUSTOMIZE_ALARM_TYPE:
        case HONDA_CUSTOMIZE_BSI_BUZZER_VOLUME_STT:
        case HONDA_CUSTOMIZE_BSI_RANGE_STT:
        case HONDA_CUSTOMIZE_BSI_2R_STT:
        case HONDA_CUSTOMIZE_BSI_TRAILER_STT:
        case HONDA_CUSTOMIZE_BSI_TRAILER_TYPE_STT:
        case HONDA_CUSTOMIZE_BSI_TRAILER_WIDTH_STT:
        case HONDA_CUSTOMIZE_BSI_TRAILER_LENGTH_STT:
        case HONDA_CUSTOMIZE_PCDW_ONOFF_STT:
        case HONDA_CUSTOMIZE_ODSF_ONOFF:
        case HONDA_CUSTOMIZE_FCM_INFO_TSR_SRBUZ_ONOFF:
        case HONDA_CUSTOMIZE_DAM_EYECLOSED_STT:
        case HONDA_METER_HUD_ILLUMI:
        case HONDA_CUSTOMIZE_VOL_DISPLAY_DID:
        case HONDA_CUSTOMIZE_HEADLIGHT_TIMER:
        case HONDA_CUSTOMIZE_WELCOM_LIGHT_1:
        case HONDA_CUSTOMIZE_WELCOM_LIGHT_2:
        case HONDA_CUSTOMIZE_WELCOM_LIGHT_1_2:
        case HONDA_EVS_BALANCE_STAT:
            mEmulatedCustomizeThread.push_task(value);
            *isSpecialValue = true;
            return {};
        case HONDA_MAINT_RESET_REQ:
            mEmulatedCustomizeThread.push_task(value);
            *isSpecialValue = true;
            return {};
        case HONDA_IDS_REQ_ANSBACK:
        case HONDA_CUSTOMIZE_IDS_RESET:
        case HONDA_CUSTOMIZE_IDS_RESET_STATUS:
            mEmulatedCustomizeThread.push_task(value);
            *isSpecialValue = true;
            return {};
        case HONDA_IDS_CUSTOMIZE_SET: {
            VehiclePropValue propValue;
            propValue.value.byteValues.resize(1);
            if (value.value.int32Values[0] == 1) {
                propValue.value.byteValues[0] = 1;
            } else if (value.value.int32Values[0] == 0) {
                propValue.value.byteValues[0] = 0;
            } else {
                EMULOG_I("Argument Error Wrong Data: %d", value.value.int32Values[0]);
                *isSpecialValue = true;
                return StatusError(StatusCode::INVALID_ARG) << "Argument Error Wrong Data";
            }
            propValue.prop = (int32_t)HONDA_IDS_REQ_DISP_CUSTOMIZE;
            propValue.timestamp = elapsedRealtimeNano();
            writeValueAndNotifyChange(propValue, true /* updateStatus */);
            *isSpecialValue = true;
            return {};
        }
        case HONDA_POWER_STATE_REQ:
        {
            LOG(INFO) << "HONDA_POWER_STATE_REQ reached! value:" << value.value.int32Values[0];
            switch (value.value.int32Values[0]) {
            case toInt(VehicleHondaPowerStateReq::NORMAL_RUNNING):
                //writeValueAndNotifyChange(*createApPowerStateReq(VehicleApPowerStateReq::ON), true /* updateStatus */);
               LOG(INFO) << "Cold boot test: ignore NORMAL_RUNNING, keep WAIT_FOR_VHAL";
                processPowerStateEvent(value);
                break;
            case toInt(VehicleHondaPowerStateReq::PREPARE_SUSPEND):
                writeValueAndNotifyChange(*createApPowerStateReq(VehicleApPowerStateReq::SHUTDOWN_PREPARE),
                                       true /* updateStatus */);
                writeValueAndNotifyChange(createHondaIcbNotification(toInt(AccState::ACC_OFF),
                                                                  toInt(KeyofftimerStartUpState::OFF),
                                                                  toInt(HalInitializeState::IMCOMPLETE)),
                                       true);
                if (value.value.int32Values[3] > 0) {
                    startGarageModeTimerThread(value.value.int32Values[3]);
                } else {
                    // The SUSPEND_IMMEDIATELY described in the HAPPI uses the SLEEP_IMMEDIATELY value defined in
                    // types.hal.
                    writeValueAndNotifyChange(
                        *createApPowerStateReq(VehicleApPowerStateReq::SHUTDOWN_PREPARE),
                        true /* updateStatus */);
                }
                break;
            case toInt(VehicleHondaPowerStateReq::PREPARE_OFF):
                LOG(INFO) << "HONDA_POWER_STATE_REQ PREPARE_OFF value:" << value.value.int32Values[1];
                switch (value.value.int32Values[1]) {
                case toInt(VehicleHondaPowerStateReqParam::SHUTDOWN):
                    writeValueAndNotifyChange(
                        *createApPowerStateReq(VehicleApPowerStateReq::SHUTDOWN_PREPARE),
                        true /* updateStatus */);
                    break;
                case toInt(VehicleHondaPowerStateReqParam::REBOOT):
                    writeValueAndNotifyChange(
                        *createApPowerStateReq(VehicleApPowerStateReq::SHUTDOWN_PREPARE),
                        true /* updateStatus */);
                    break;
                default:
                    break;
                }
                processPowerStateEvent(value);
                break;
            default:
                break;
            }
            break;
        }
        case IGNITION_STATE:
            processPowerStateEvent(value);
            break;
        case HW_KEY_INPUT:
        case HW_ROTARY_INPUT:
        case HW_CUSTOM_INPUT:
        case HONDA_DISABLE_KEYEVENT:
            processHwKeyEvent(value, true /* updateStatus */);
            *isSpecialValue = true;
            return {};
        default:
            break;
    }
    return {};
}

StatusCode FakeVehicleHardware::setValues(std::shared_ptr<const SetValuesCallback> callback,
                                          const std::vector<SetValueRequest>& requests) {
    for (auto& request : requests) {
        if (FAKE_VEHICLEHARDWARE_DEBUG) {
            ALOGD("Set value for property ID: %s", PROP_ID_TO_CSTR(request.value.prop));
        }

        // In a real VHAL implementation, you could either send the setValue request to vehicle bus
        // here in the binder thread, or you could send the request in setValue which runs in
        // the handler thread. If you decide to send the setValue request here, you should not
        // wait for the response here and the handler thread should handle the setValue response.
        mPendingSetValueRequests.addRequest(request, callback);
    }

    return StatusCode::OK;
}

VhalResult<void> FakeVehicleHardware::setValue(const VehiclePropValue& value) {
    // Synchronize between vehicle propery.
    if (isLinkedProp(value.prop)) {
        syncLinkedPropValue(value);
    }
    // In a real VHAL implementation, this will send the request to vehicle bus if not already
    // sent in setValues, and wait for the response from vehicle bus.
    // Here we are just updating mValuePool.
    bool isSpecialValue = false;
    auto setSpecialValueResult = maybeSetSpecialValue(value, &isSpecialValue);
    if (isSpecialValue) {
        if (!setSpecialValueResult.ok()) {
            return StatusError(getErrorCode(setSpecialValueResult)) << StringPrintf(
                           "failed to set special value for property ID: %s, error: %s",
                           PROP_ID_TO_CSTR(value.prop), getErrorMsg(setSpecialValueResult).c_str());
        }
        return {};
    }

    // auto updatedValue = mValuePool->obtain(value);

    // auto writeResult = mServerSidePropStore->writeValue(
    //         std::move(updatedValue),
    //         /*updateStatus=*/false, /*mode=*/VehiclePropertyStore::EventMode::ON_VALUE_CHANGE,
    //         /*useCurrentTimestamp=*/true);
    // if (!writeResult.ok()) {
    //     return StatusError(getErrorCode(writeResult))
    //            << StringPrintf("failed to write value into property store, error: %s",
    //                            getErrorMsg(writeResult).c_str());
    // }

    return writeValueAndNotifyChange(value, false);
}

SetValueResult FakeVehicleHardware::handleSetValueRequest(const SetValueRequest& request) {
    SetValueResult setValueResult;
    setValueResult.requestId = request.requestId;

    if (auto result = setValue(request.value); !result.ok()) {
        ALOGE("failed to set value, error: %s, code: %d", getErrorMsg(result).c_str(),
              getIntErrorCode(result));
        setValueResult.status = getErrorCode(result);
    } else {
        setValueResult.status = StatusCode::OK;
    }

    return setValueResult;
}

StatusCode FakeVehicleHardware::getValues(std::shared_ptr<const GetValuesCallback> callback,
                                          const std::vector<GetValueRequest>& requests) const {
    for (auto& request : requests) {
        if (FAKE_VEHICLEHARDWARE_DEBUG) {
            ALOGD("getValues(%s)", PROP_ID_TO_CSTR(request.prop.prop));
        }

        // In a real VHAL implementation, you could either send the getValue request to vehicle bus
        // here in the binder thread, or you could send the request in getValue which runs in
        // the handler thread. If you decide to send the getValue request here, you should not
        // wait for the response here and the handler thread should handle the getValue response.
        mPendingGetValueRequests.addRequest(request, callback);
    }

    return StatusCode::OK;
}

GetValueResult FakeVehicleHardware::handleGetValueRequest(const GetValueRequest& request) {
    GetValueResult getValueResult;
    getValueResult.requestId = request.requestId;

    auto result = getValue(request.prop);
    if (!result.ok()) {
        ALOGE("failed to get value, error: %s, code: %d", getErrorMsg(result).c_str(),
              getIntErrorCode(result));
        getValueResult.status = getErrorCode(result);
    } else {
        getValueResult.status = StatusCode::OK;
        getValueResult.prop = *result.value();
    }
    return getValueResult;
}

FakeVehicleHardware::ValueResultType FakeVehicleHardware::getValue(
        const VehiclePropValue& value) const {
    // In a real VHAL implementation, this will send the request to vehicle bus if not already
    // sent in getValues, and wait for the response from vehicle bus.
    // Here we are just reading value from mValuePool.
    bool isSpecialValue = false;
    auto result = maybeGetSpecialValue(value, &isSpecialValue);
    if (isSpecialValue) {
        if (!result.ok()) {
            return StatusError(getErrorCode(result))
                   << StringPrintf("failed to get special value: %s, error: %s",
                                   PROP_ID_TO_CSTR(value.prop), getErrorMsg(result).c_str());
        } else {
            return result;
        }
    }

    auto readResult = mServerSidePropStore->readValue(value);
    if (!readResult.ok()) {
        StatusCode errorCode = getErrorCode(readResult);
        if (errorCode == StatusCode::NOT_AVAILABLE) {
            return StatusError(errorCode) << "value has not been set yet";
        } else {
            return StatusError(errorCode)
                   << "failed to get value, error: " << getErrorMsg(readResult);
        }
    }

    return readResult;
}

DumpResult FakeVehicleHardware::dump(const std::vector<std::string>& options) {
    DumpResult result;
    result.callerShouldDumpState = false;
    if (options.size() == 0) {
        // We only want caller to dump default state when there is no options.
        result.callerShouldDumpState = true;
        result.buffer = dumpAllProperties();
        return result;
    }
    std::string option = options[0];
    if (EqualsIgnoreCase(option, "--help")) {
        result.buffer = dumpHelp();
        return result;
    } else if (EqualsIgnoreCase(option, "--list")) {
        result.buffer = dumpListProperties();
    } else if (EqualsIgnoreCase(option, "--get")) {
        result.buffer = dumpSpecificProperty(options);
    } else if (EqualsIgnoreCase(option, "--getWithArg")) {
        result.buffer = dumpGetPropertyWithArg(options);
    } else if (EqualsIgnoreCase(option, "--set")) {
        result.buffer = dumpSetProperties(options);
    } else if (EqualsIgnoreCase(option, "--save-prop")) {
        result.buffer = dumpSaveProperty(options);
    } else if (EqualsIgnoreCase(option, "--restore-prop")) {
        result.buffer = dumpRestoreProperty(options);
    } else if (EqualsIgnoreCase(option, "--inject-event")) {
        result.buffer = dumpInjectEvent(options);
    } else if (EqualsIgnoreCase(option, kUserHalDumpOption)) {
        result.buffer = mFakeUserHal->dump();
    } else if (EqualsIgnoreCase(option, "--genfakedata")) {
        result.buffer = genFakeDataCommand(options);
    } else if (EqualsIgnoreCase(option, "--genTestVendorConfigs")) {
        mAddExtraTestVendorConfigs = true;
        result.refreshPropertyConfigs = true;
        result.buffer = "successfully generated vendor configs";
    } else if (EqualsIgnoreCase(option, "--restoreVendorConfigs")) {
        mAddExtraTestVendorConfigs = false;
        result.refreshPropertyConfigs = true;
        result.buffer = "successfully restored vendor configs";
    } else if (EqualsIgnoreCase(option, "--dumpSub")) {
        result.buffer = dumpSubscriptions();
    } else {
        result.buffer = StringPrintf("Invalid option: %s\n", option.c_str());
    }
    return result;
}

std::string FakeVehicleHardware::genFakeDataHelp() {
    return R"(
Generate Fake Data Usage:
--genfakedata --startlinear [propID] [mValue] [cValue] [dispersion] [increment] [interval]: "
Start a linear generator that generates event with floatValue within range:
[mValue - disperson, mValue + dispersion].
propID(int32): ID for the property to generate event for.
mValue(float): The middle of the possible values for the property.
cValue(float): The start value for the property, must be within the range.
dispersion(float): The range the value can change.
increment(float): The step the value would increase by for each generated event,
if exceed the range, the value would loop back.
interval(int64): The interval in nanoseconds the event would generate by.

--genfakedata --stoplinear [propID(int32)]: Stop a linear generator

--genfakedata --startjson --path [jsonFilePath] [repetition]:
Start a JSON generator that would generate events according to a JSON file.
jsonFilePath(string): The path to a JSON file. The JSON content must be in the format of
[{
    "timestamp": 1000000,
    "areaId": 0,
    "value": 8,
    "prop": 289408000
}, {...}]
Each event in the JSON file would be generated by the same interval their timestamp is relative to
the first event's timestamp.
repetition(int32, optional): how many iterations the events would be generated. If it is not
provided, it would iterate indefinitely.

--genfakedata --startjson --content [jsonContent]: Start a JSON generator using the content.

--genfakedata --stopjson [generatorID(string)]: Stop a JSON generator.

--genfakedata --keypress [keyCode(int32)] [display[int32]]: Generate key press.

--genfakedata --keyinputv2 [area(int32)] [display(int32)] [keyCode[int32]] [action[int32]]
  [repeatCount(int32)]

--genfakedata --motioninput [area(int32)] [display(int32)] [inputType[int32]] [action[int32]]
  [buttonState(int32)] --pointer [pointerId(int32)] [toolType(int32)] [xData(float)] [yData(float)]
  [pressure(float)] [size(float)]
  Generate a motion input event. --pointer option can be specified multiple times.

--genTestVendorConfigs: Generates fake VehiclePropConfig ranging from 0x5000 to 0x8000 all with
  vendor property group, global vehicle area, and int32 vehicle property type. This is mainly used
  for testing

--restoreVendorConfigs: Restores to to the default state if genTestVendorConfigs was used.
  Otherwise this will do nothing.

)";
}

std::string FakeVehicleHardware::parseErrMsg(std::string fieldName, std::string value,
                                             std::string type) {
    return StringPrintf("failed to parse %s as %s: \"%s\"\n%s", fieldName.c_str(), type.c_str(),
                        value.c_str(), genFakeDataHelp().c_str());
}

void FakeVehicleHardware::generateVendorConfigs(
        std::vector<VehiclePropConfig>& outAllConfigs) const {
    for (int i = STARTING_VENDOR_CODE_PROPERTIES_FOR_TEST;
         i < ENDING_VENDOR_CODE_PROPERTIES_FOR_TEST; i++) {
        VehiclePropConfig config;
        config.prop = i;
        config.access = VehiclePropertyAccess::READ_WRITE;
        outAllConfigs.push_back(config);
    }
}

std::string FakeVehicleHardware::genFakeDataCommand(const std::vector<std::string>& options) {
    if (options.size() < 2) {
        return "No subcommand specified for genfakedata\n" + genFakeDataHelp();
    }

    std::string command = options[1];
    if (command == "--startlinear") {
        // --genfakedata --startlinear [propID(int32)] [middleValue(float)]
        // [currentValue(float)] [dispersion(float)] [increment(float)] [interval(int64)]
        if (options.size() != 8) {
            return "incorrect argument count, need 8 arguments for --genfakedata --startlinear\n" +
                   genFakeDataHelp();
        }
        int32_t propId;
        float middleValue;
        float currentValue;
        float dispersion;
        float increment;
        int64_t interval;
        if (!android::base::ParseInt(options[2], &propId)) {
            return parseErrMsg("propId", options[2], "int");
        }
        if (!android::base::ParseFloat(options[3], &middleValue)) {
            return parseErrMsg("middleValue", options[3], "float");
        }
        if (!android::base::ParseFloat(options[4], &currentValue)) {
            return parseErrMsg("currentValue", options[4], "float");
        }
        if (!android::base::ParseFloat(options[5], &dispersion)) {
            return parseErrMsg("dispersion", options[5], "float");
        }
        if (!android::base::ParseFloat(options[6], &increment)) {
            return parseErrMsg("increment", options[6], "float");
        }
        if (!android::base::ParseInt(options[7], &interval)) {
            return parseErrMsg("interval", options[7], "int");
        }
        auto generator = std::make_unique<LinearFakeValueGenerator>(
                propId, middleValue, currentValue, dispersion, increment, interval);
        mGeneratorHub->registerGenerator(propId, std::move(generator));
        return "Linear event generator started successfully";
    } else if (command == "--stoplinear") {
        // --genfakedata --stoplinear [propID(int32)]
        if (options.size() != 3) {
            return "incorrect argument count, need 3 arguments for --genfakedata --stoplinear\n" +
                   genFakeDataHelp();
        }
        int32_t propId;
        if (!android::base::ParseInt(options[2], &propId)) {
            return parseErrMsg("propId", options[2], "int");
        }
        if (mGeneratorHub->unregisterGenerator(propId)) {
            return "Linear event generator stopped successfully";
        }
        return StringPrintf("No linear event generator found for property: %s",
                            PROP_ID_TO_CSTR(propId));
    } else if (command == "--startjson") {
        // --genfakedata --startjson --path path repetition
        // or
        // --genfakedata --startjson --content content repetition.
        if (options.size() != 4 && options.size() != 5) {
            return "incorrect argument count, need 4 or 5 arguments for --genfakedata "
                   "--startjson\n";
        }
        // Iterate infinitely if repetition number is not provided
        int32_t repetition = -1;
        if (options.size() == 5) {
            if (!android::base::ParseInt(options[4], &repetition)) {
                return parseErrMsg("repetition", options[4], "int");
            }
        }
        std::unique_ptr<JsonFakeValueGenerator> generator;
        if (options[2] == "--path") {
            const std::string& fileName = options[3];
            generator = std::make_unique<JsonFakeValueGenerator>(fileName, repetition);
            if (!generator->hasNext()) {
                return "invalid JSON file, no events";
            }
        } else if (options[2] == "--content") {
            const std::string& content = options[3];
            generator =
                    std::make_unique<JsonFakeValueGenerator>(/*unused=*/true, content, repetition);
            if (!generator->hasNext()) {
                return "invalid JSON content, no events";
            }
        }
        int32_t cookie = std::hash<std::string>()(options[3]);
        mGeneratorHub->registerGenerator(cookie, std::move(generator));
        return StringPrintf("JSON event generator started successfully, ID: %" PRId32, cookie);
    } else if (command == "--stopjson") {
        // --genfakedata --stopjson [generatorID(string)]
        if (options.size() != 3) {
            return "incorrect argument count, need 3 arguments for --genfakedata --stopjson\n";
        }
        int32_t cookie;
        if (!android::base::ParseInt(options[2], &cookie)) {
            return parseErrMsg("cookie", options[2], "int");
        }
        if (mGeneratorHub->unregisterGenerator(cookie)) {
            return "JSON event generator stopped successfully";
        } else {
            return StringPrintf("No JSON event generator found for ID: %s", options[2].c_str());
        }
    } else if (command == "--keypress") {
        int32_t keyCode;
        int32_t display;
        // --genfakedata --keypress [keyCode(int32)] [display[int32]]
        if (options.size() != 4) {
            return "incorrect argument count, need 4 arguments for --genfakedata --keypress\n";
        }
        if (!android::base::ParseInt(options[2], &keyCode)) {
            return parseErrMsg("keyCode", options[2], "int");
        }
        if (!android::base::ParseInt(options[3], &display)) {
            return parseErrMsg("display", options[3], "int");
        }
        // Send back to HAL
        onValueChangeCallback(
                createHwInputKeyProp(VehicleHwKeyInputAction::ACTION_DOWN, keyCode, display));
        onValueChangeCallback(
                createHwInputKeyProp(VehicleHwKeyInputAction::ACTION_UP, keyCode, display));
        return "keypress event generated successfully";
    } else if (command == "--keyinputv2") {
        int32_t area;
        int32_t display;
        int32_t keyCode;
        int32_t action;
        int32_t repeatCount;
        // --genfakedata --keyinputv2 [area(int32)] [display(int32)] [keyCode[int32]]
        // [action[int32]] [repeatCount(int32)]
        if (options.size() != 7) {
            return "incorrect argument count, need 7 arguments for --genfakedata --keyinputv2\n";
        }
        if (!android::base::ParseInt(options[2], &area)) {
            return parseErrMsg("area", options[2], "int");
        }
        if (!android::base::ParseInt(options[3], &display)) {
            return parseErrMsg("display", options[3], "int");
        }
        if (!android::base::ParseInt(options[4], &keyCode)) {
            return parseErrMsg("keyCode", options[4], "int");
        }
        if (!android::base::ParseInt(options[5], &action)) {
            return parseErrMsg("action", options[5], "int");
        }
        if (!android::base::ParseInt(options[6], &repeatCount)) {
            return parseErrMsg("repeatCount", options[6], "int");
        }
        // Send back to HAL
        onValueChangeCallback(createHwKeyInputV2Prop(area, display, keyCode, action, repeatCount));
        return StringPrintf(
                "keyinputv2 event generated successfully with area:%d, display:%d,"
                " keyCode:%d, action:%d, repeatCount:%d",
                area, display, keyCode, action, repeatCount);

    } else if (command == "--motioninput") {
        int32_t area;
        int32_t display;
        int32_t inputType;
        int32_t action;
        int32_t buttonState;
        int32_t pointerCount;

        // --genfakedata --motioninput [area(int32)] [display(int32)] [inputType[int32]]
        // [action[int32]] [buttonState(int32)] [pointerCount(int32)]
        // --pointer [pointerId(int32)] [toolType(int32)] [xData(float)] [yData(float)]
        // [pressure(float)] [size(float)]
        int optionsSize = (int)options.size();
        if (optionsSize / 7 < 2) {
            return "incorrect argument count, need at least 14 arguments for --genfakedata "
                   "--motioninput including at least 1 --pointer\n";
        }

        if (optionsSize % 7 != 0) {
            return "incorrect argument count, need 6 arguments for every --pointer\n";
        }
        pointerCount = (int)optionsSize / 7 - 1;

        if (!android::base::ParseInt(options[2], &area)) {
            return parseErrMsg("area", options[2], "int");
        }
        if (!android::base::ParseInt(options[3], &display)) {
            return parseErrMsg("display", options[3], "int");
        }
        if (!android::base::ParseInt(options[4], &inputType)) {
            return parseErrMsg("inputType", options[4], "int");
        }
        if (!android::base::ParseInt(options[5], &action)) {
            return parseErrMsg("action", options[5], "int");
        }
        if (!android::base::ParseInt(options[6], &buttonState)) {
            return parseErrMsg("buttonState", options[6], "int");
        }

        int32_t pointerId[pointerCount];
        int32_t toolType[pointerCount];
        float xData[pointerCount];
        float yData[pointerCount];
        float pressure[pointerCount];
        float size[pointerCount];

        for (int i = 7, pc = 0; i < optionsSize; i += 7, pc += 1) {
            int offset = i;
            if (options[offset] != "--pointer") {
                return "--pointer is needed for the motion input\n";
            }
            offset += 1;
            if (!android::base::ParseInt(options[offset], &pointerId[pc])) {
                return parseErrMsg("pointerId", options[offset], "int");
            }
            offset += 1;
            if (!android::base::ParseInt(options[offset], &toolType[pc])) {
                return parseErrMsg("toolType", options[offset], "int");
            }
            offset += 1;
            if (!android::base::ParseFloat(options[offset], &xData[pc])) {
                return parseErrMsg("xData", options[offset], "float");
            }
            offset += 1;
            if (!android::base::ParseFloat(options[offset], &yData[pc])) {
                return parseErrMsg("yData", options[offset], "float");
            }
            offset += 1;
            if (!android::base::ParseFloat(options[offset], &pressure[pc])) {
                return parseErrMsg("pressure", options[offset], "float");
            }
            offset += 1;
            if (!android::base::ParseFloat(options[offset], &size[pc])) {
                return parseErrMsg("size", options[offset], "float");
            }
        }

        // Send back to HAL
        onValueChangeCallback(createHwMotionInputProp(area, display, inputType, action, buttonState,
                                                      pointerCount, pointerId, toolType, xData,
                                                      yData, pressure, size));

        std::string successMessage = StringPrintf(
                "motion event generated successfully with area:%d, display:%d,"
                " inputType:%d, action:%d, buttonState:%d, pointerCount:%d\n",
                area, display, inputType, action, buttonState, pointerCount);
        for (int i = 0; i < pointerCount; i++) {
            successMessage += StringPrintf(
                    "Pointer #%d {\n"
                    " id:%d , tooltype:%d \n"
                    " x:%f , y:%f\n"
                    " pressure: %f, data: %f\n"
                    "}\n",
                    i, pointerId[i], toolType[i], xData[i], yData[i], pressure[i], size[i]);
        }
        return successMessage;
    }

    return StringPrintf("Unknown command: \"%s\"\n%s", command.c_str(), genFakeDataHelp().c_str());
}

VehiclePropValue FakeVehicleHardware::createHwInputKeyProp(VehicleHwKeyInputAction action,
                                                           int32_t keyCode, int32_t targetDisplay) {
    VehiclePropValue value = {
            .timestamp = elapsedRealtimeNano(),
            .areaId = 0,
            .prop = toInt(VehicleProperty::HW_KEY_INPUT),
            .status = VehiclePropertyStatus::AVAILABLE,
            .value.int32Values = {toInt(action), keyCode, targetDisplay},
    };
    return value;
}

VehiclePropValue FakeVehicleHardware::createHwKeyInputV2Prop(int32_t area, int32_t targetDisplay,
                                                             int32_t keyCode, int32_t action,
                                                             int32_t repeatCount) {
    VehiclePropValue value = {.timestamp = elapsedRealtimeNano(),
                              .areaId = area,
                              .prop = toInt(VehicleProperty::HW_KEY_INPUT_V2),
                              .status = VehiclePropertyStatus::AVAILABLE,
                              .value.int32Values = {targetDisplay, keyCode, action, repeatCount},
                              .value.int64Values = {elapsedRealtimeNano()}};
    return value;
}

VehiclePropValue FakeVehicleHardware::createHwMotionInputProp(
        int32_t area, int32_t display, int32_t inputType, int32_t action, int32_t buttonState,
        int32_t pointerCount, int32_t pointerId[], int32_t toolType[], float xData[], float yData[],
        float pressure[], float size[]) {
    std::vector<int> intValues;
    intValues.push_back(display);
    intValues.push_back(inputType);
    intValues.push_back(action);
    intValues.push_back(buttonState);
    intValues.push_back(pointerCount);
    for (int i = 0; i < pointerCount; i++) {
        intValues.push_back(pointerId[i]);
    }
    for (int i = 0; i < pointerCount; i++) {
        intValues.push_back(toolType[i]);
    }

    std::vector<float> floatValues;
    for (int i = 0; i < pointerCount; i++) {
        floatValues.push_back(xData[i]);
    }
    for (int i = 0; i < pointerCount; i++) {
        floatValues.push_back(yData[i]);
    }
    for (int i = 0; i < pointerCount; i++) {
        floatValues.push_back(pressure[i]);
    }
    for (int i = 0; i < pointerCount; i++) {
        floatValues.push_back(size[i]);
    }

    VehiclePropValue value = {.timestamp = elapsedRealtimeNano(),
                              .areaId = area,
                              .prop = toInt(VehicleProperty::HW_MOTION_INPUT),
                              .status = VehiclePropertyStatus::AVAILABLE,
                              .value.int32Values = intValues,
                              .value.floatValues = floatValues,
                              .value.int64Values = {elapsedRealtimeNano()}};
    return value;
}

void FakeVehicleHardware::eventFromVehicleBus(const VehiclePropValue& value) {
    mServerSidePropStore->writeValue(mValuePool->obtain(value));
}

std::string FakeVehicleHardware::dumpSubscriptions() {
    std::scoped_lock<std::mutex> lockGuard(mLock);
    std::string result = "Subscriptions: \n";
    for (const auto& [interval, actionForInterval] : mActionByIntervalInNanos) {
        for (const auto& propIdAreaId : actionForInterval.propIdAreaIdsToRefresh) {
            const auto& refreshInfo = mRefreshInfoByPropIdAreaId[propIdAreaId];
            bool vur = (refreshInfo.eventMode == VehiclePropertyStore::EventMode::ON_VALUE_CHANGE);
            float sampleRateHz = 1'000'000'000. / refreshInfo.intervalInNanos;
            result += StringPrintf("Continuous{property: %s, areaId: %d, rate: %lf hz, vur: %b}\n",
                                   PROP_ID_TO_CSTR(propIdAreaId.propId), propIdAreaId.areaId,
                                   sampleRateHz, vur);
        }
    }
    for (const auto& propIdAreaId : mSubOnChangePropIdAreaIds) {
        result += StringPrintf("OnChange{property: %s, areaId: %d}\n",
                               PROP_ID_TO_CSTR(propIdAreaId.propId), propIdAreaId.areaId);
    }
    return result;
}

std::string FakeVehicleHardware::dumpHelp() {
    return "Usage: \n\n"
           "[no args]: dumps (id and value) all supported properties \n"
           "--help: shows this help\n"
           "--list: lists the property IDs and their supported area IDs for all supported "
           "properties\n"
           "--get <PROP_ID_1> [PROP_ID_2] [PROP_ID_N]: dumps the value of specific properties. \n"
           "--getWithArg <PROP_ID> [ValueArguments]: gets the value for a specific property. "
           "The value arguments constructs a VehiclePropValue used in the getValue request. \n"
           "--set <PROP_ID> [ValueArguments]: sets the value of property PROP_ID, the value "
           "arguments constructs a VehiclePropValue used in the setValue request. \n"
           "--save-prop <PROP_ID> [-a AREA_ID]: saves the current value for PROP_ID, integration "
           "tests that modify prop value must call this before test and restore-prop after test. \n"
           "--restore-prop <PROP_ID> [-a AREA_ID]: restores a previously saved property value. \n"
           "--inject-event <PROP_ID> [ValueArguments]: inject a property update event from car\n\n"
           "ValueArguments are in the format of [-a OPTIONAL_AREA_ID] "
           "[-i INT_VALUE_1 [INT_VALUE_2 ...]] "
           "[-i64 INT64_VALUE_1 [INT64_VALUE_2 ...]] "
           "[-f FLOAT_VALUE_1 [FLOAT_VALUE_2 ...]] "
           "[-s STR_VALUE] "
           "[-b BYTES_VALUE].\n"
           "For example: to set property ID 0x1234, areaId 0x1 to int32 values: [1, 2, 3], "
           "use \"--set 0x1234 -a 0x1 -i 1 2 3\"\n"
           "Note that the string, bytes and area value can be set just once, while the other can"
           " have multiple values (so they're used in the respective array), "
           "BYTES_VALUE is in the form of 0xXXXX, e.g. 0xdeadbeef.\n" +
           genFakeDataHelp() + "Fake user HAL usage: \n" + mFakeUserHal->showDumpHelp();
}

std::string FakeVehicleHardware::dumpAllProperties() {
    auto configs = mServerSidePropStore->getAllConfigs();
    if (configs.size() == 0) {
        return "no properties to dump\n";
    }
    std::string msg = StringPrintf("dumping %zu properties\n", configs.size());
    int rowNumber = 1;
    for (const VehiclePropConfig& config : configs) {
        msg += dumpOnePropertyByConfig(rowNumber++, config);
    }
    return msg;
}

std::string FakeVehicleHardware::dumpOnePropertyByConfig(int rowNumber,
                                                         const VehiclePropConfig& config) {
    size_t numberAreas = config.areaConfigs.size();
    std::string msg = "";
    if (numberAreas == 0) {
        msg += StringPrintf("%d: ", rowNumber);
        msg += dumpOnePropertyById(config.prop, /* areaId= */ 0);
        return msg;
    }
    for (size_t j = 0; j < numberAreas; ++j) {
        if (numberAreas > 1) {
            msg += StringPrintf("%d-%zu: ", rowNumber, j);
        } else {
            msg += StringPrintf("%d: ", rowNumber);
        }
        msg += dumpOnePropertyById(config.prop, config.areaConfigs[j].areaId);
    }
    return msg;
}

std::string FakeVehicleHardware::dumpOnePropertyById(int32_t propId, int32_t areaId) {
    VehiclePropValue value = {
            .areaId = areaId,
            .prop = propId,
            .value = {},
    };
    bool isSpecialValue = false;
    auto result = maybeGetSpecialValue(value, &isSpecialValue);
    if (!isSpecialValue) {
        result = mServerSidePropStore->readValue(value);
    }
    if (!result.ok()) {
        return StringPrintf("failed to read property value: %s, error: %s, code: %d\n",
                            PROP_ID_TO_CSTR(propId), getErrorMsg(result).c_str(),
                            getIntErrorCode(result));

    } else {
        return result.value()->toString() + "\n";
    }
}

std::string FakeVehicleHardware::dumpListProperties() {
    auto configs = mServerSidePropStore->getAllConfigs();
    if (configs.size() == 0) {
        return "no properties to list\n";
    }
    int rowNumber = 1;
    std::stringstream ss;
    ss << "listing " << configs.size() << " properties" << std::endl;
    for (const auto& config : configs) {
        std::vector<int32_t> areaIds;
        for (const auto& areaConfig : config.areaConfigs) {
            areaIds.push_back(areaConfig.areaId);
        }
        ss << rowNumber++ << ": " << PROP_ID_TO_CSTR(config.prop) << ", propID: " << std::showbase
           << std::hex << config.prop << std::noshowbase << std::dec
           << ", areaIDs: " << vecToStringOfHexValues(areaIds) << std::endl;
    }
    return ss.str();
}

Result<void> FakeVehicleHardware::checkArgumentsSize(const std::vector<std::string>& options,
                                                     size_t minSize) {
    size_t size = options.size();
    if (size >= minSize) {
        return {};
    }
    return Error() << StringPrintf("Invalid number of arguments: required at least %zu, got %zu\n",
                                   minSize, size);
}

Result<int32_t> FakeVehicleHardware::parsePropId(const std::vector<std::string>& options,
                                                 size_t index) {
    const std::string& propIdStr = options[index];
    auto result = stringToPropId(propIdStr);
    if (result.ok()) {
        return result;
    }
    return safelyParseInt<int32_t>(index, propIdStr);
}

std::string FakeVehicleHardware::dumpSpecificProperty(const std::vector<std::string>& options) {
    if (auto result = checkArgumentsSize(options, /*minSize=*/2); !result.ok()) {
        return getErrorMsg(result);
    }

    // options[0] is the command itself...
    int rowNumber = 1;
    size_t size = options.size();
    std::string msg = "";
    for (size_t i = 1; i < size; ++i) {
        auto propResult = parsePropId(options, i);
        if (!propResult.ok()) {
            msg += getErrorMsg(propResult);
            continue;
        }
        int32_t prop = propResult.value();
        auto result = mServerSidePropStore->getPropConfig(prop);
        if (!result.ok()) {
            msg += StringPrintf("No property %s\n", PROP_ID_TO_CSTR(prop));
            continue;
        }
        msg += dumpOnePropertyByConfig(rowNumber++, result.value());
    }
    return msg;
}

std::vector<std::string> FakeVehicleHardware::getOptionValues(
        const std::vector<std::string>& options, size_t* index) {
    std::vector<std::string> values;
    while (*index < options.size()) {
        std::string option = options[*index];
        if (SET_PROP_OPTIONS.find(option) != SET_PROP_OPTIONS.end()) {
            return values;
        }
        values.push_back(option);
        (*index)++;
    }
    return values;
}

Result<VehiclePropValue> FakeVehicleHardware::parsePropOptions(
        const std::vector<std::string>& options) {
    // Options format:
    // --set/get/inject-event PROP [-f f1 f2...] [-i i1 i2...] [-i64 i1 i2...] [-s s1 s2...]
    // [-b b1 b2...] [-a a] [-t timestamp]
    size_t optionIndex = 1;
    auto result = parsePropId(options, optionIndex);
    if (!result.ok()) {
        return Error() << StringPrintf("Property ID/Name: \"%s\" is not valid: %s\n",
                                       options[optionIndex].c_str(), getErrorMsg(result).c_str());
    }
    VehiclePropValue prop = {};
    prop.prop = result.value();
    prop.status = VehiclePropertyStatus::AVAILABLE;
    optionIndex++;
    std::unordered_set<std::string> parsedOptions;

    while (optionIndex < options.size()) {
        std::string argType = options[optionIndex];
        optionIndex++;

        size_t currentIndex = optionIndex;
        std::vector<std::string> argValues = getOptionValues(options, &optionIndex);
        if (parsedOptions.find(argType) != parsedOptions.end()) {
            return Error() << StringPrintf("Duplicate \"%s\" options\n", argType.c_str());
        }
        parsedOptions.insert(argType);
        size_t argValuesSize = argValues.size();
        if (EqualsIgnoreCase(argType, "-i")) {
            if (argValuesSize == 0) {
                return Error() << "No values specified when using \"-i\"\n";
            }
            prop.value.int32Values.resize(argValuesSize);
            for (size_t i = 0; i < argValuesSize; i++) {
                auto int32Result = safelyParseInt<int32_t>(currentIndex + i, argValues[i]);
                if (!int32Result.ok()) {
                    return Error()
                           << StringPrintf("Value: \"%s\" is not a valid int: %s\n",
                                           argValues[i].c_str(), getErrorMsg(int32Result).c_str());
                }
                prop.value.int32Values[i] = int32Result.value();
            }
        } else if (EqualsIgnoreCase(argType, "-i64")) {
            if (argValuesSize == 0) {
                return Error() << "No values specified when using \"-i64\"\n";
            }
            prop.value.int64Values.resize(argValuesSize);
            for (size_t i = 0; i < argValuesSize; i++) {
                auto int64Result = safelyParseInt<int64_t>(currentIndex + i, argValues[i]);
                if (!int64Result.ok()) {
                    return Error()
                           << StringPrintf("Value: \"%s\" is not a valid int64: %s\n",
                                           argValues[i].c_str(), getErrorMsg(int64Result).c_str());
                }
                prop.value.int64Values[i] = int64Result.value();
            }
        } else if (EqualsIgnoreCase(argType, "-f")) {
            if (argValuesSize == 0) {
                return Error() << "No values specified when using \"-f\"\n";
            }
            prop.value.floatValues.resize(argValuesSize);
            for (size_t i = 0; i < argValuesSize; i++) {
                auto floatResult = safelyParseFloat(currentIndex + i, argValues[i]);
                if (!floatResult.ok()) {
                    return Error()
                           << StringPrintf("Value: \"%s\" is not a valid float: %s\n",
                                           argValues[i].c_str(), getErrorMsg(floatResult).c_str());
                }
                prop.value.floatValues[i] = floatResult.value();
            }
        } else if (EqualsIgnoreCase(argType, "-s")) {
            if (argValuesSize != 1) {
                return Error() << "Expect exact one value when using \"-s\"\n";
            }
            prop.value.stringValue = argValues[0];
        } else if (EqualsIgnoreCase(argType, "-b")) {
            if (argValuesSize != 1) {
                return Error() << "Expect exact one value when using \"-b\"\n";
            }
            auto bytesResult = parseHexString(argValues[0]);
            if (!bytesResult.ok()) {
                return Error() << StringPrintf("value: \"%s\" is not a valid hex string: %s\n",
                                               argValues[0].c_str(),
                                               getErrorMsg(bytesResult).c_str());
            }
            prop.value.byteValues = std::move(bytesResult.value());
        } else if (EqualsIgnoreCase(argType, "-a")) {
            if (argValuesSize != 1) {
                return Error() << "Expect exact one value when using \"-a\"\n";
            }
            auto int32Result = safelyParseInt<int32_t>(currentIndex, argValues[0]);
            if (!int32Result.ok()) {
                return Error() << StringPrintf("Area ID: \"%s\" is not a valid int: %s\n",
                                               argValues[0].c_str(),
                                               getErrorMsg(int32Result).c_str());
            }
            prop.areaId = int32Result.value();
        } else if (EqualsIgnoreCase(argType, "-t")) {
            if (argValuesSize != 1) {
                return Error() << "Expect exact one value when using \"-t\"\n";
            }
            auto int64Result = safelyParseInt<int64_t>(currentIndex, argValues[0]);
            if (!int64Result.ok()) {
                return Error() << StringPrintf("Timestamp: \"%s\" is not a valid int64: %s\n",
                                               argValues[0].c_str(),
                                               getErrorMsg(int64Result).c_str());
            }
            prop.timestamp = int64Result.value();
        } else {
            return Error() << StringPrintf("Unknown option: %s\n", argType.c_str());
        }
    }

    return prop;
}

std::string FakeVehicleHardware::dumpSetProperties(const std::vector<std::string>& options) {
    if (auto result = checkArgumentsSize(options, 3); !result.ok()) {
        return getErrorMsg(result);
    }

    auto parseResult = parsePropOptions(options);
    if (!parseResult.ok()) {
        return getErrorMsg(parseResult);
    }
    VehiclePropValue prop = std::move(parseResult.value());
    ALOGD("Dump: Setting property: %s", prop.toString().c_str());

    bool isSpecialValue = false;
    auto setResult = maybeSetSpecialValue(prop, &isSpecialValue);

    if (!isSpecialValue) {
        auto updatedValue = mValuePool->obtain(prop);
        updatedValue->timestamp = elapsedRealtimeNano();
        setResult = mServerSidePropStore->writeValue(std::move(updatedValue));
    }

    if (setResult.ok()) {
        return StringPrintf("Set property: %s\n", prop.toString().c_str());
    }
    return StringPrintf("failed to set property: %s, error: %s\n", prop.toString().c_str(),
                        getErrorMsg(setResult).c_str());
}

std::string FakeVehicleHardware::dumpGetPropertyWithArg(const std::vector<std::string>& options) {
    if (auto result = checkArgumentsSize(options, 3); !result.ok()) {
        return getErrorMsg(result);
    }

    auto parseResult = parsePropOptions(options);
    if (!parseResult.ok()) {
        return getErrorMsg(parseResult);
    }
    VehiclePropValue prop = std::move(parseResult.value());
    ALOGD("Dump: Getting property: %s", prop.toString().c_str());

    bool isSpecialValue = false;
    auto result = maybeGetSpecialValue(prop, &isSpecialValue);

    if (!isSpecialValue) {
        result = mServerSidePropStore->readValue(prop);
    }

    if (!result.ok()) {
        return StringPrintf("failed to read property value: %s, error: %s, code: %d\n",
                            PROP_ID_TO_CSTR(prop.prop), getErrorMsg(result).c_str(),
                            getIntErrorCode(result));
    }
    return StringPrintf("Get property result: %s\n", result.value()->toString().c_str());
}

std::string FakeVehicleHardware::dumpSaveProperty(const std::vector<std::string>& options) {
    // Format: --save-prop PROP [-a areaID]
    if (auto result = checkArgumentsSize(options, 2); !result.ok()) {
        return getErrorMsg(result);
    }

    auto parseResult = parsePropOptions(options);
    if (!parseResult.ok()) {
        return getErrorMsg(parseResult);
    }
    // We are only using the prop and areaId option.
    VehiclePropValue value = std::move(parseResult.value());
    int32_t propId = value.prop;
    int32_t areaId = value.areaId;

    auto readResult = mServerSidePropStore->readValue(value);
    if (!readResult.ok()) {
        return StringPrintf("Failed to save current property value, error: %s",
                            getErrorMsg(readResult).c_str());
    }

    std::scoped_lock<std::mutex> lockGuard(mLock);
    mSavedProps[PropIdAreaId{
            .propId = propId,
            .areaId = areaId,
    }] = std::move(readResult.value());

    return StringPrintf("Property: %" PRId32 ", areaID: %" PRId32 " saved", propId, areaId);
}

std::string FakeVehicleHardware::dumpRestoreProperty(const std::vector<std::string>& options) {
    // Format: --restore-prop PROP [-a areaID]
    if (auto result = checkArgumentsSize(options, 2); !result.ok()) {
        return getErrorMsg(result);
    }

    auto parseResult = parsePropOptions(options);
    if (!parseResult.ok()) {
        return getErrorMsg(parseResult);
    }
    // We are only using the prop and areaId option.
    VehiclePropValue value = std::move(parseResult.value());
    int32_t propId = value.prop;
    int32_t areaId = value.areaId;
    VehiclePropValuePool::RecyclableType savedValue;

    {
        std::scoped_lock<std::mutex> lockGuard(mLock);
        auto it = mSavedProps.find(PropIdAreaId{
                .propId = propId,
                .areaId = areaId,
        });
        if (it == mSavedProps.end()) {
            return StringPrintf("No saved property for property: %" PRId32 ", areaID: %" PRId32,
                                propId, areaId);
        }

        savedValue = std::move(it->second);
        // Remove the saved property after restoring it.
        mSavedProps.erase(it);
    }

    // Update timestamp.
    savedValue->timestamp = elapsedRealtimeNano();

    auto writeResult = mServerSidePropStore->writeValue(std::move(savedValue));
    if (!writeResult.ok()) {
        return StringPrintf("Failed to restore property value, error: %s",
                            getErrorMsg(writeResult).c_str());
    }

    return StringPrintf("Property: %" PRId32 ", areaID: %" PRId32 " restored", propId, areaId);
}

std::string FakeVehicleHardware::dumpInjectEvent(const std::vector<std::string>& options) {
    if (auto result = checkArgumentsSize(options, 3); !result.ok()) {
        return getErrorMsg(result);
    }

    auto parseResult = parsePropOptions(options);
    if (!parseResult.ok()) {
        return getErrorMsg(parseResult);
    }
    VehiclePropValue prop = std::move(parseResult.value());
    ALOGD("Dump: Injecting event from vehicle bus: %s", prop.toString().c_str());

    eventFromVehicleBus(prop);

    return StringPrintf("Event for property: %s injected", PROP_ID_TO_CSTR(prop.prop));
}

StatusCode FakeVehicleHardware::checkHealth() {
    // Always return OK for checkHealth.
    return StatusCode::OK;
}

void FakeVehicleHardware::registerOnPropertyChangeEvent(
        std::unique_ptr<const PropertyChangeCallback> callback) {
    if (mOnPropertyChangeCallback != nullptr) {
        ALOGE("registerOnPropertyChangeEvent must only be called once");
        return;
    }
    mOnPropertyChangeCallback = std::move(callback);
}

void FakeVehicleHardware::registerOnPropertySetErrorEvent(
        std::unique_ptr<const PropertySetErrorCallback> callback) {
    // In FakeVehicleHardware, we will never use mOnPropertySetErrorCallback.
    if (mOnPropertySetErrorCallback != nullptr) {
        ALOGE("registerOnPropertySetErrorEvent must only be called once");
        return;
    }
    mOnPropertySetErrorCallback = std::move(callback);
}

StatusCode FakeVehicleHardware::subscribe(SubscribeOptions options) {
    int32_t propId = options.propId;

    auto configResult = mServerSidePropStore->getPropConfig(propId);
    if (!configResult.ok()) {
        ALOGE("subscribe: property: %" PRId32 " is not supported", propId);
        return StatusCode::INVALID_ARG;
    }

    std::scoped_lock<std::mutex> lockGuard(mLock);
    for (int areaId : options.areaIds) {
        if (StatusCode status = subscribePropIdAreaIdLocked(propId, areaId, options.sampleRate,
                                                            options.enableVariableUpdateRate,
                                                            configResult.value());
            status != StatusCode::OK) {
            return status;
        }
    }
    return StatusCode::OK;
}

bool FakeVehicleHardware::isVariableUpdateRateSupported(const VehiclePropConfig& vehiclePropConfig,
                                                        int32_t areaId) {
    for (size_t i = 0; i < vehiclePropConfig.areaConfigs.size(); i++) {
        const auto& areaConfig = vehiclePropConfig.areaConfigs[i];
        if (areaConfig.areaId != areaId) {
            continue;
        }
        if (areaConfig.supportVariableUpdateRate) {
            return true;
        }
        break;
    }
    return false;
}

void FakeVehicleHardware::refreshTimestampForInterval(int64_t intervalInNanos) {
    std::unordered_map<PropIdAreaId, VehiclePropertyStore::EventMode, PropIdAreaIdHash>
            eventModeByPropIdAreaId;

    {
        std::scoped_lock<std::mutex> lockGuard(mLock);

        if (mActionByIntervalInNanos.find(intervalInNanos) == mActionByIntervalInNanos.end()) {
            ALOGE("No actions scheduled for the interval: %" PRId64 ", ignore the refresh request",
                  intervalInNanos);
            return;
        }

        ActionForInterval actionForInterval = mActionByIntervalInNanos[intervalInNanos];

        // Make a copy so that we don't hold the lock while trying to refresh the timestamp.
        // Refreshing the timestamp will inovke onValueChangeCallback which also requires lock, so
        // we must not hold lock.
        for (const PropIdAreaId& propIdAreaId : actionForInterval.propIdAreaIdsToRefresh) {
            const RefreshInfo& refreshInfo = mRefreshInfoByPropIdAreaId[propIdAreaId];
            eventModeByPropIdAreaId[propIdAreaId] = refreshInfo.eventMode;
        }
    }

    mServerSidePropStore->refreshTimestamps(eventModeByPropIdAreaId);
}

void FakeVehicleHardware::registerRefreshLocked(PropIdAreaId propIdAreaId,
                                                VehiclePropertyStore::EventMode eventMode,
                                                float sampleRateHz) {
    if (mRefreshInfoByPropIdAreaId.find(propIdAreaId) != mRefreshInfoByPropIdAreaId.end()) {
        unregisterRefreshLocked(propIdAreaId);
    }

    int64_t intervalInNanos = static_cast<int64_t>(1'000'000'000. / sampleRateHz);
    RefreshInfo refreshInfo = {
            .eventMode = eventMode,
            .intervalInNanos = intervalInNanos,
    };
    mRefreshInfoByPropIdAreaId[propIdAreaId] = refreshInfo;

    if (mActionByIntervalInNanos.find(intervalInNanos) != mActionByIntervalInNanos.end()) {
        // If we have already registered for this interval, then add the action info to the
        // actions list.
        mActionByIntervalInNanos[intervalInNanos].propIdAreaIdsToRefresh.insert(propIdAreaId);
        return;
    }

    // This is the first action for the interval, register a timer callback for that interval.
    auto action = std::make_shared<RecurrentTimer::Callback>(
            [this, intervalInNanos] { refreshTimestampForInterval(intervalInNanos); });
    mActionByIntervalInNanos[intervalInNanos] = ActionForInterval{
            .propIdAreaIdsToRefresh = {propIdAreaId},
            .recurrentAction = action,
    };
    mRecurrentTimer->registerTimerCallback(intervalInNanos, action);
}

void FakeVehicleHardware::unregisterRefreshLocked(PropIdAreaId propIdAreaId) {
    if (mRefreshInfoByPropIdAreaId.find(propIdAreaId) == mRefreshInfoByPropIdAreaId.end()) {
        ALOGW("PropId: %" PRId32 ", areaId: %" PRId32 " was not registered for refresh, ignore",
              propIdAreaId.propId, propIdAreaId.areaId);
        return;
    }

    int64_t intervalInNanos = mRefreshInfoByPropIdAreaId[propIdAreaId].intervalInNanos;
    auto& actionForInterval = mActionByIntervalInNanos[intervalInNanos];
    actionForInterval.propIdAreaIdsToRefresh.erase(propIdAreaId);
    if (actionForInterval.propIdAreaIdsToRefresh.empty()) {
        mRecurrentTimer->unregisterTimerCallback(actionForInterval.recurrentAction);
        mActionByIntervalInNanos.erase(intervalInNanos);
    }
    mRefreshInfoByPropIdAreaId.erase(propIdAreaId);
}

StatusCode FakeVehicleHardware::subscribePropIdAreaIdLocked(
        int32_t propId, int32_t areaId, float sampleRateHz, bool enableVariableUpdateRate,
        const VehiclePropConfig& vehiclePropConfig) {
    PropIdAreaId propIdAreaId{
            .propId = propId,
            .areaId = areaId,
    };
    switch (vehiclePropConfig.changeMode) {
        case VehiclePropertyChangeMode::STATIC:
            ALOGW("subscribe to a static property, do nothing.");
            return StatusCode::OK;
        case VehiclePropertyChangeMode::ON_CHANGE:
            mSubOnChangePropIdAreaIds.insert(std::move(propIdAreaId));
            return StatusCode::OK;
        case VehiclePropertyChangeMode::CONTINUOUS:
            if (sampleRateHz == 0.f) {
                ALOGE("Must not use sample rate 0 for a continuous property");
                return StatusCode::INVALID_ARG;
            }
            // For continuous properties, we must generate a new onPropertyChange event
            // periodically according to the sample rate.
            auto eventMode = VehiclePropertyStore::EventMode::ALWAYS;
            if (isVariableUpdateRateSupported(vehiclePropConfig, areaId) &&
                enableVariableUpdateRate) {
                eventMode = VehiclePropertyStore::EventMode::ON_VALUE_CHANGE;
            }

            registerRefreshLocked(propIdAreaId, eventMode, sampleRateHz);
            return StatusCode::OK;
    }
}

StatusCode FakeVehicleHardware::unsubscribe(int32_t propId, int32_t areaId) {
    std::scoped_lock<std::mutex> lockGuard(mLock);
    PropIdAreaId propIdAreaId{
            .propId = propId,
            .areaId = areaId,
    };
    if (mRefreshInfoByPropIdAreaId.find(propIdAreaId) != mRefreshInfoByPropIdAreaId.end()) {
        unregisterRefreshLocked(propIdAreaId);
    }
    mSubOnChangePropIdAreaIds.erase(propIdAreaId);
    return StatusCode::OK;
}

void FakeVehicleHardware::onPropertyValueFromCar(const VehiclePropValue& value, const bool byRead) {
    ALOGI("property: %s is update by read:%s.", value.toString().c_str(), byRead ? "Y" : "N");
    if (byRead) {
        onValueChangeCallback(value);
    }
}

void FakeVehicleHardware::triggerSendAllValues() {
    // Do nothing, implement by Derived class.
}

void FakeVehicleHardware::onNotifyFcpData() {
    LOG(DEBUG) << __func__ << " start";
    std::unique_lock<std::mutex> lock(gMutex);
    gCond.notify_all();
}

void FakeVehicleHardware::onValueChangeCallback(const VehiclePropValue& value) {
    ATRACE_CALL();
    onValuesChangeCallback({value});
}

void FakeVehicleHardware::onValuesChangeCallback(std::vector<VehiclePropValue> values) {
    ATRACE_CALL();
    std::vector<VehiclePropValue> subscribedUpdatedValues;

    {
        std::scoped_lock<std::mutex> lockGuard(mLock);
        if (mOnPropertyChangeCallback == nullptr) {
            return;
        }

        for (const auto& value : values) {
            PropIdAreaId propIdAreaId{
                    .propId = value.prop,
                    .areaId = value.areaId,
            };
            if (mRefreshInfoByPropIdAreaId.find(propIdAreaId) == mRefreshInfoByPropIdAreaId.end() &&
                mSubOnChangePropIdAreaIds.find(propIdAreaId) == mSubOnChangePropIdAreaIds.end()) {
                if (FAKE_VEHICLEHARDWARE_DEBUG) {
                    ALOGD("The updated property value: %s is not subscribed, ignore",
                          value.toString().c_str());
                }
                continue;
            }

            subscribedUpdatedValues.push_back(value);
        }
    }

    (*mOnPropertyChangeCallback)(std::move(subscribedUpdatedValues));
}

bool FakeVehicleHardware::loadPropConfigsFromDir(
        const std::string& dirPath,
        std::unordered_map<int32_t, ConfigDeclaration>* configsByPropId) {
    ALOGD("loading properties from %s", dirPath.c_str());
    auto dir = opendir(dirPath.c_str());
    if (dir == nullptr) {
        ALOGE("Failed to open config directory: %s", dirPath.c_str());
        return false;
    }

    std::regex regJson(".*[.]json", std::regex::icase);
    while (auto f = readdir(dir)) {
        if (!std::regex_match(f->d_name, regJson)) {
            continue;
        }
        std::string filePath = dirPath + "/" + std::string(f->d_name);
        ALOGI("loading properties from %s", filePath.c_str());
        auto result = mLoader.loadPropConfig(filePath);
        if (!result.ok()) {
            ALOGE("failed to load config file: %s, error: %s", filePath.c_str(),
                  result.error().message().c_str());
            continue;
        }
        for (auto& [propId, configDeclaration] : result.value()) {
            (*configsByPropId)[propId] = std::move(configDeclaration);
        }
    }
    closedir(dir);
    return true;
}

Result<float> FakeVehicleHardware::safelyParseFloat(int index, const std::string& s) {
    float out;
    if (!ParseFloat(s, &out)) {
        return Error() << StringPrintf("non-float argument at index %d: %s\n", index, s.c_str());
    }
    return out;
}

Result<std::vector<uint8_t>> FakeVehicleHardware::parseHexString(const std::string& s) {
    std::vector<uint8_t> bytes;
    if (s.size() % 2 != 0) {
        return Error() << StringPrintf("invalid hex string: %s, should have even size\n",
                                       s.c_str());
    }
    if (!StartsWith(s, "0x")) {
        return Error() << StringPrintf("hex string should start with \"0x\", got %s\n", s.c_str());
    }
    std::string subs = s.substr(2);
    std::transform(subs.begin(), subs.end(), subs.begin(),
                   [](unsigned char c) { return std::tolower(c); });

    bool highDigit = true;
    for (size_t i = 0; i < subs.size(); i++) {
        char c = subs[i];
        uint8_t v;
        if (c >= '0' && c <= '9') {
            v = c - '0';
        } else if (c >= 'a' && c <= 'f') {
            v = c - 'a' + 10;
        } else {
            return Error() << StringPrintf("invalid character %c in hex string %s\n", c,
                                           subs.c_str());
        }
        if (highDigit) {
            bytes.push_back(v * 16);
        } else {
            bytes[bytes.size() - 1] += v;
        }
        highDigit = !highDigit;
    }
    return bytes;
}

template <class CallbackType, class RequestType>
FakeVehicleHardware::PendingRequestHandler<CallbackType, RequestType>::PendingRequestHandler(
        FakeVehicleHardware* hardware)
    : mHardware(hardware) {
    // Don't initialize mThread in initialization list because mThread depends on mRequests and we
    // want mRequests to be initialized first.
    mThread = std::thread([this] {
        while (mRequests.waitForItems()) {
            handleRequestsOnce();
        }
    });
}

template <class CallbackType, class RequestType>
void FakeVehicleHardware::PendingRequestHandler<CallbackType, RequestType>::addRequest(
        RequestType request, std::shared_ptr<const CallbackType> callback) {
    mRequests.push({
            request,
            callback,
    });
}

template <class CallbackType, class RequestType>
void FakeVehicleHardware::PendingRequestHandler<CallbackType, RequestType>::stop() {
    mRequests.deactivate();
    if (mThread.joinable()) {
        mThread.join();
    }
}

template <>
void FakeVehicleHardware::PendingRequestHandler<FakeVehicleHardware::GetValuesCallback,
                                                GetValueRequest>::handleRequestsOnce() {
    std::unordered_map<std::shared_ptr<const GetValuesCallback>, std::vector<GetValueResult>>
            callbackToResults;
    for (const auto& rwc : mRequests.flush()) {
        ATRACE_BEGIN("FakeVehicleHardware:handleGetValueRequest");
        auto result = mHardware->handleGetValueRequest(rwc.request);
        ATRACE_END();
        callbackToResults[rwc.callback].push_back(std::move(result));
    }
    for (const auto& [callback, results] : callbackToResults) {
        ATRACE_BEGIN("FakeVehicleHardware:call get value result callback");
        (*callback)(std::move(results));
        ATRACE_END();
    }
}

template <>
void FakeVehicleHardware::PendingRequestHandler<FakeVehicleHardware::SetValuesCallback,
                                                SetValueRequest>::handleRequestsOnce() {
    std::unordered_map<std::shared_ptr<const SetValuesCallback>, std::vector<SetValueResult>>
            callbackToResults;
    for (const auto& rwc : mRequests.flush()) {
        ATRACE_BEGIN("FakeVehicleHardware:handleSetValueRequest");
        auto result = mHardware->handleSetValueRequest(rwc.request);
        ATRACE_END();
        callbackToResults[rwc.callback].push_back(std::move(result));
    }
    for (const auto& [callback, results] : callbackToResults) {
        ATRACE_BEGIN("FakeVehicleHardware:call set value result callback");
        (*callback)(std::move(results));
        ATRACE_END();
    }
}

VhalResult<void> FakeVehicleHardware::writeValueAndNotifyChange(const VehiclePropValue& value, bool updateStatus) {
    LOG(DEBUG) << __func__ << ": prop:" << value.prop;
    auto updatedValue = mValuePool->obtain(value);

    auto writeResult = mServerSidePropStore->writeValue(
            std::move(updatedValue),
            /*updateStatus=*/false, /*mode=*/VehiclePropertyStore::EventMode::ON_VALUE_CHANGE,
            /*useCurrentTimestamp=*/true);
    if (!writeResult.ok()) {
        return StatusError(getErrorCode(writeResult))
               << StringPrintf("failed to write value into property store, error: %s",
                               getErrorMsg(writeResult).c_str());
    }
    if (checkNonVolatileVP(value.prop)) {
        updateNonVolatileVP(value.prop, value.areaId);
    }

    onPropertyValueFromCar(value);
    return {};
}

void FakeVehicleHardware::setHwCustomInputProp(int32_t keyEvent, int32_t targetDisplay, int32_t repeatCounter, bool updateStatus) {
    auto hwCustomInputProp = mValuePool->obtain(VehiclePropertyType::INT32_VEC, 3);
    hwCustomInputProp->prop = HW_CUSTOM_INPUT;
    hwCustomInputProp->areaId = 0;
    hwCustomInputProp->timestamp = elapsedRealtimeNano();
    hwCustomInputProp->status = VehiclePropertyStatus::AVAILABLE;
    hwCustomInputProp->value.int32Values[0] = keyEvent;
    hwCustomInputProp->value.int32Values[1] = targetDisplay;
    hwCustomInputProp->value.int32Values[2] = repeatCounter;

    LOG(INFO) << __func__ << " HW_CUSTOM_INPUT:" << hwCustomInputProp->toString();
    writeValueAndNotifyChange(*hwCustomInputProp, updateStatus);
}

void FakeVehicleHardware::processHwKeyEvent(const VehiclePropValue &value, bool updateStatus) {
    if (value.prop == HW_CUSTOM_INPUT) {
        auto setFunc = [this, updateStatus](int32_t keyEvent, int32_t targetDisplay, int32_t repeatCounter) {
            setHwCustomInputProp(keyEvent, targetDisplay, repeatCounter, updateStatus);
        };
        mHwKeyInput.registerSetKeyPropertyValueFunc(setFunc);
    }

    for (const auto &keyProp : mHwKeyInput.processHwKeyProperty(value)) {
        switch (keyProp.prop) {
        case HW_KEY_INPUT: {
            LOG(INFO) << __func__ << " HW_KEY_INPUT:" << keyProp.toString();
            auto updatedPropValue = mValuePool->obtain(keyProp);
            updatedPropValue->timestamp = elapsedRealtimeNano();
            writeValueAndNotifyChange(*updatedPropValue, updateStatus);
            break;
        }
        case HW_ROTARY_INPUT:
            LOG(INFO) << __func__ << " HW_ROTARY_INPUT(disable)" << keyProp.toString();
            break;
        case HW_CUSTOM_INPUT: {
            LOG(INFO) << __func__ << " HW_CUSTOM_INPUT:" << keyProp.toString();
            break;
        }
        case HONDA_DISABLE_KEYEVENT: {
            LOG(INFO) << __func__ << " HONDA_HW_ROTARY_ENCODER/HONDA_DISABLE_KEYEVENT:" << keyProp.toString();
            auto updatedPropValue = mValuePool->obtain(keyProp);
            updatedPropValue->timestamp = elapsedRealtimeNano();
            writeValueAndNotifyChange(*updatedPropValue, updateStatus);
            break;
        }
        default:
            LOG(ERROR) << __func__ << " Unknown prop";
            break;
        }
    }
}

VehiclePropValue FakeVehicleHardware::createHondaIcbNotification(int32_t accState, int32_t timerState,
                                                              int32_t halInitializeState) {
    LOG(DEBUG) << __func__ << " start";
    int32_t tempPropId = toInt(HondaVehicleProperty::HONDA_ICB_NOTIFICATION);
    auto internalPropValue = mServerSidePropStore->readValue(tempPropId, 0, 0);
    if (internalPropValue.ok()) {
        internalPropValue.value()->timestamp = elapsedRealtimeNano();
        internalPropValue.value()->value.int32Values[3] = accState;
        internalPropValue.value()->value.int32Values[6] = timerState;
        internalPropValue.value()->value.int32Values[9] = halInitializeState;
        LOG(INFO) << __func__ << " HONDA_ICB_NOTIFICATION:" << internalPropValue.value()->toString();
    }
    return *internalPropValue.value();
}

void FakeVehicleHardware::initPropertyForColdStart() {
    LOG(DEBUG) << __func__ << " start";

    // Read parts configuration settings.
    Json::Value jsonSignalList;

    if (!readJsonFile(FILE_NAME_PARTS_CONFIGURATION, jsonSignalList)) {
        writePartsConfigurationSignalDefaultValue();
        readJsonFile(FILE_NAME_PARTS_CONFIGURATION, jsonSignalList);
    }
    std::list<VehiclePropConfig> partsConfigurationConfigSettings =
        readPartsConfigurationConfigSettings(jsonSignalList);
    std::list<VehiclePropConfig> partsConfigurationRemoveConfigs = readPartsConfigurationRemoveConfigs(jsonSignalList);
    std::list<VehiclePropValue> partsConfigurationVPValueSettings = readPartsConfigurationValueSettings(jsonSignalList);

    // Read non-volatile settings.
    std::list<VehiclePropValue> nonVolatileVPValueSettings = readNonVolatileVPValueSettings();
    // Read HONDA_CUSTOMIZE_CUSTOMCHECK
    gCustomCheckVPValueSettings = readCostomizeCustomCheckVPValueSettings();
    // Read HONDA_CUSTOMIZE_CUSTOM_GET_REQUEST,HONDA_CUSTOMIZE_CUSTOM_SET_REQUEST
    readCostomizeCustomGetResponseVPValueSettings();
    // Read indirect communication value.
    gIndirectVPValueSettings = readIndirectCommunicationVPValueSettings();

    for (size_t i = 0; i < defaultconfig::getDefaultConfigs().size(); i++) {
        int32_t tempPropId = defaultconfig::getDefaultConfigs()[i].config.prop;
        int32_t isVendorGroup = tempPropId & toInt(VehiclePropertyGroup::VENDOR);
        int32_t isSystemGroup = tempPropId & toInt(VehiclePropertyGroup::SYSTEM);
        // Honda VP extraction.
        if (isVendorGroup == toInt(VehiclePropertyGroup::VENDOR)) {
            auto internalPropConfig = mServerSidePropStore->getConfig(tempPropId);
            if (internalPropConfig.ok()) {
                // Remove config.
                auto accessNoneIte = std::find(kAccessNoneConfig.begin(), kAccessNoneConfig.end(), tempPropId);
                if (accessNoneIte != kAccessNoneConfig.end()) {
                    VehiclePropConfig config = {.prop = tempPropId};
                    mServerSidePropStore->removeConfig(config);
                }
            }

            size_t areaConfigSize = defaultconfig::getDefaultConfigs()[i].config.areaConfigs.size();
            for (size_t j = 0; j <= areaConfigSize; j++) {
                int tempAreaId = 0;
                if (areaConfigSize != 0) {
                    if (j == areaConfigSize) {
                        continue;
                    }
                    tempAreaId = defaultconfig::getDefaultConfigs()[i].config.areaConfigs[j].areaId;
                }
                auto internalPropValue = mServerSidePropStore->readValue(tempPropId, tempAreaId, 0);
                if (internalPropValue.ok()) {
                    // Update values different from DefaultConfig.
                    for (size_t i = 0; i < arraysize(kDifferenceInitialValue); i++) {
                        if (tempPropId == kDifferenceInitialValue[i].config.prop) {
                            internalPropValue.value()->value = kDifferenceInitialValue[i].initialValue;
                            break;
                        }
                    }

                    // Updating configs by Parts Configuration.
                    for (const VehiclePropConfig &updateConfig : partsConfigurationConfigSettings) {
                        if (tempPropId == updateConfig.prop) {
                            mServerSidePropStore->writeConfig(updateConfig);
                            break;
                        }
                    }
                    // Remove configs by Parts Configuration.
                    for (const VehiclePropConfig &removeProp : partsConfigurationRemoveConfigs) {
                        if (tempPropId == removeProp.prop) {
                            mServerSidePropStore->removeConfig(removeProp);
                            break;
                        }
                    }
                    // Updating values by Parts Configration.
                    for (const VehiclePropValue &propValue : partsConfigurationVPValueSettings) {
                        if (tempPropId == propValue.prop) {
                            internalPropValue.value()->value = propValue.value;
                            break;
                        }
                    }
                    // Updating values by non-volatile storage.
                    for (const VehiclePropValue &propValue : nonVolatileVPValueSettings) {
                        if (tempPropId == propValue.prop && tempAreaId == propValue.areaId) {
                            internalPropValue.value()->value = propValue.value;
                            break;
                        }
                    }
                    // Updating default value by HONDA_CUSTOMIZE_CUSTOMCHECK.
                    if (gCustomCheckVPValueSettings.find(HONDA_CUSTOMIZE_CUSTOMCHECK_DEFAULT_VALUE_KEY) != gCustomCheckVPValueSettings.end()) {
                        const VehiclePropValue& propValue = gCustomCheckVPValueSettings[HONDA_CUSTOMIZE_CUSTOMCHECK_DEFAULT_VALUE_KEY];
                        if (tempPropId == propValue.prop && tempAreaId == propValue.areaId) {
                            internalPropValue.value()->value = propValue.value;
                        }
                    }
                    // Updating default value by indirect communication value.
                    if (gIndirectVPValueSettings.find(tempPropId) != gIndirectVPValueSettings.end()) {
                        const VehiclePropValue& propValue = gIndirectVPValueSettings[tempPropId];
                        if (tempPropId == propValue.prop) {
                            internalPropValue.value()->value = propValue.value;
                        }
                    }
                    internalPropValue.value()->status = VehiclePropertyStatus::AVAILABLE;
                    internalPropValue.value()->timestamp = elapsedRealtimeNano();
                    mServerSidePropStore->writeValue(mValuePool->obtain(*internalPropValue.value()), true);
                }
            }
        }
        // AOSP VP extraction.
        if (isSystemGroup == toInt(VehiclePropertyGroup::SYSTEM)) {
            auto internalPropConfig = mServerSidePropStore->getConfig(tempPropId);
            if (internalPropConfig.ok()) {
                // Remove config.
                auto ite = std::find(kAccessNoneConfig.begin(), kAccessNoneConfig.end(), tempPropId);
                if (ite != kAccessNoneConfig.end()) {
                    VehiclePropConfig config = {.prop = tempPropId};
                    mServerSidePropStore->removeConfig(config);
                }
                // Update supportedEnumValues by FCP.
                if (FcpComm::getInstance()->isPropIdApplyFcp(tempPropId)) {
                    switch (tempPropId) {
                    case toInt(VehicleProperty::INFO_DRIVER_SEAT):
                    {
                        int32_t infoDriverSeatValue = FcpComm::getInstance()->getInfoDriverSeatValue();
                        if (infoDriverSeatValue != FCP_VALUE_ERROR) {
                            VehiclePropConfig updatePropConfig = copyVPConfig(internalPropConfig.value());
                            for (VehicleAreaConfig& areaConfig : updatePropConfig.areaConfigs) {
                                if (infoDriverSeatValue == ROW_1_LEFT) {
                                    areaConfig.supportedEnumValues = {ROW_1_LEFT};
                                } else if (infoDriverSeatValue == ROW_1_RIGHT) {
                                    areaConfig.supportedEnumValues = {ROW_1_RIGHT};
                                }
                            }
                            mServerSidePropStore->writeConfig(updatePropConfig);
                        }
                        break;
                    }
                    case toInt(VehicleProperty::INFO_EV_CONNECTOR_TYPE):
                    {
                        int32_t infoEvConnectorType = FcpComm::getInstance()->getInfoEvConnectorType();
                        if (infoEvConnectorType != FCP_VALUE_ERROR) {
                            VehiclePropConfig updatePropConfig = copyVPConfig(internalPropConfig.value());
                            for (VehicleAreaConfig& areaConfig : updatePropConfig.areaConfigs) {
                                areaConfig.supportedEnumValues = {infoEvConnectorType};
                            }
                            mServerSidePropStore->writeConfig(updatePropConfig);
                        }
                        break;
                    }
                    case toInt(VehicleProperty::INFO_EV_PORT_LOCATION):
                    {
                        int32_t infoEvPortLocation = FcpComm::getInstance()->getInfoEvPortLocation();
                        if (infoEvPortLocation != FCP_VALUE_ERROR) {
                            VehiclePropConfig updatePropConfig = copyVPConfig(internalPropConfig.value());
                            if (infoEvPortLocation == FCP_VALUE_ACCESS_NONE) {
                                infoEvPortLocation = UNKNOWN;
                            }
                            for (VehicleAreaConfig& areaConfig : updatePropConfig.areaConfigs) {
                                areaConfig.supportedEnumValues = {infoEvPortLocation};
                            }
                            mServerSidePropStore->writeConfig(updatePropConfig);
                        }
                        break;
                    }
                    case toInt(VehicleProperty::INFO_FUEL_DOOR_LOCATION):
                    {
                        int32_t infoFuelDoorLocation = FcpComm::getInstance()->getInfoFuelDoorLocation();
                        if (infoFuelDoorLocation != UNKNOWN) {
                            VehiclePropConfig updatePropConfig = copyVPConfig(internalPropConfig.value());
                            for (VehicleAreaConfig& areaConfig : updatePropConfig.areaConfigs) {
                                areaConfig.supportedEnumValues = {infoFuelDoorLocation};
                            }
                            mServerSidePropStore->writeConfig(updatePropConfig);
                        }
                        break;
                    }
                    case toInt(VehicleProperty::INFO_FUEL_TYPE):
                    {
                        int32_t infoFuelType = FcpComm::getInstance()->getInfoFuelType();
                        if (infoFuelType != FCP_VALUE_ERROR) {
                            VehiclePropConfig updatePropConfig = copyVPConfig(internalPropConfig.value());
                            for (VehicleAreaConfig& areaConfig : updatePropConfig.areaConfigs) {
                                areaConfig.supportedEnumValues = {infoFuelType};
                            }
                            mServerSidePropStore->writeConfig(updatePropConfig);
                        }
                        break;
                    }
                    case toInt(VehicleProperty::ELECTRONIC_TOLL_COLLECTION_CARD_TYPE):
                    {
                        int32_t cardType = FcpComm::getInstance()->getCardType();
                        if (cardType != FCP_VALUE_ERROR) {
                            VehiclePropConfig updatePropConfig = copyVPConfig(internalPropConfig.value());
                            for (VehicleAreaConfig& areaConfig : updatePropConfig.areaConfigs) {
                                areaConfig.supportedEnumValues = {UNKNOWN, cardType};
                            }
                            mServerSidePropStore->writeConfig(updatePropConfig);
                        }
                        break;
                    }
                    case toInt(VehicleProperty::HVAC_TEMPERATURE_DISPLAY_UNITS):
                    {
                        VehiclePropConfig updatePropConfig = copyVPConfig(internalPropConfig.value());
                        int C_TEMPVARIATION_AC = jsonSignalList["C_TEMPVARIATION_AC"].asInt() ;
                        if (TEMPVARIATION_AC_CHINA <= C_TEMPVARIATION_AC <= TEMPVARIATION_AC_KC_KE) {
                            VehiclePropConfig updatePropConfig = copyVPConfig(internalPropConfig.value());
                            for (VehicleAreaConfig& areaConfig : updatePropConfig.areaConfigs) {
                                areaConfig.supportedEnumValues = {toInt(VehicleUnit::CELSIUS)};
                            }
                            mServerSidePropStore->writeConfig(updatePropConfig);
                        } else if (C_TEMPVARIATION_AC == TEMPVARIATION_AC_KA) {
                            VehiclePropConfig updatePropConfig = copyVPConfig(internalPropConfig.value());
                            for (VehicleAreaConfig& areaConfig : updatePropConfig.areaConfigs) {
                                areaConfig.supportedEnumValues = {toInt(VehicleUnit::FAHRENHEIT)};
                            }
                            mServerSidePropStore->writeConfig(updatePropConfig);
                        }
                        break;
                    }
                    //Update supportedArea
                    case toInt(VehicleProperty::HVAC_STEERING_WHEEL_HEAT):
                    {
                        VehiclePropConfig updatePropConfig = copyVPConfig(internalPropConfig.value());
                        int C_FEATURE_HSW = jsonSignalList["C_FEATURE_HSW"].asInt();
                        if (C_FEATURE_HSW == FEATURE_HSW_EQUIPPED) {
                            for (VehicleAreaConfig& areaConfig : updatePropConfig.areaConfigs) {
                                areaConfig.minInt32Value = 0;
                                areaConfig.maxInt32Value = 1;
                            }
                            mServerSidePropStore->writeConfig(updatePropConfig);
                        } else if (C_FEATURE_HSW == FEATURE_HSW_HIGH_EQUIPPED) {
                            for (VehicleAreaConfig& areaConfig : updatePropConfig.areaConfigs) {
                                areaConfig.minInt32Value = 0;
                                areaConfig.maxInt32Value = 3;
                            }
                            mServerSidePropStore->writeConfig(updatePropConfig);
                        }
                        break;
                    }
                    default:
                        break;
                    }
                }
            }

            size_t areaConfigSize = defaultconfig::getDefaultConfigs()[i].config.areaConfigs.size();
            for (size_t j = 0; j <= areaConfigSize; j++) {
                int tempAreaId = 0;
                if (areaConfigSize != 0) {
                    if (j == areaConfigSize) {
                        continue;
                    }
                    tempAreaId = defaultconfig::getDefaultConfigs()[i].config.areaConfigs[j].areaId;
                }
                auto internalPropValue = mServerSidePropStore->readValue(tempPropId, tempAreaId, 0);
                if (internalPropValue.ok()) {
                    // Update valuea different from DefaultConfig.
                    for (size_t i = 0; i < arraysize(kDifferenceInitialValue); i++) {
                        if (tempPropId == kDifferenceInitialValue[i].config.prop) {
                            internalPropValue.value()->value = kDifferenceInitialValue[i].initialValue;
                            internalPropValue.value()->status = VehiclePropertyStatus::AVAILABLE;
                            internalPropValue.value()->timestamp = elapsedRealtimeNano();
                            // AOSP will be notified individually.
                            // This is because if you access an uninitialized VP, the app will stop.
                            mServerSidePropStore->writeValue(mValuePool->obtain(*internalPropValue.value()), true);
                            break;
                        }
                    }
                    // Updating configs by Parts Configuration.
                    for (const VehiclePropConfig &updateConfig : partsConfigurationConfigSettings) {
                        if (tempPropId == updateConfig.prop) {
                            mServerSidePropStore->writeConfig(updateConfig);
                            internalPropValue.value()->status = VehiclePropertyStatus::AVAILABLE;
                            internalPropValue.value()->timestamp = elapsedRealtimeNano();
                            mServerSidePropStore->writeValue(mValuePool->obtain(*internalPropValue.value()), true);
                            break;
                        }
                    }
                    // Remove configs by Parts Configuration.
                    for (const VehiclePropConfig &removeProp : partsConfigurationRemoveConfigs) {
                        if (tempPropId == removeProp.prop) {
                            mServerSidePropStore->removeConfig(removeProp);
                            break;
                        }
                    }
                    // Updating values by Parts Configration.
                    for (const VehiclePropValue &propValue : partsConfigurationVPValueSettings) {
                        if (tempPropId == propValue.prop) {
                            internalPropValue.value()->value = propValue.value;
                            internalPropValue.value()->status = VehiclePropertyStatus::AVAILABLE;
                            internalPropValue.value()->timestamp = elapsedRealtimeNano();
                            mServerSidePropStore->writeValue(mValuePool->obtain(*internalPropValue.value()), true);
                            break;
                        }
                    }
                    // Updating values by non-volatile storage.
                    for (const VehiclePropValue &propValue : nonVolatileVPValueSettings) {
                        if (tempPropId == propValue.prop && tempAreaId == propValue.areaId) {
                            internalPropValue.value()->value = propValue.value;
                            internalPropValue.value()->status = VehiclePropertyStatus::AVAILABLE;
                            internalPropValue.value()->timestamp = elapsedRealtimeNano();
                            mServerSidePropStore->writeValue(mValuePool->obtain(*internalPropValue.value()), true);
                            break;
                        }
                    }
                    // Update value by FCP.
                    if (FcpComm::getInstance()->isPropIdApplyFcp(tempPropId)) {
                        int32_t infoDriverSeatValue;
                        float infoEvBatteryCapacity;
                        int32_t infoEvConnectorType;
                        int32_t infoEvPortLocation;
                        float infoFuelCapacity;
                        int32_t infoFuelDoorLocation;
                        int32_t infoFuelType;
                        std::string infoModel;
                        int32_t infoModelYear;
                        float evBatteryInstantaneousChargeRate;
                        float evBatteryLevel;
                        int32_t evBatteryDisplayUnits;
                        int32_t evChargeCurrentDrawLimit;
                        int32_t evChargePercentLimit;
                        int32_t evChargePortInfo;
                        switch (tempPropId) {
                        case toInt(VehicleProperty::INFO_DRIVER_SEAT):
                            infoDriverSeatValue = FcpComm::getInstance()->getInfoDriverSeatValue();
                            LOG(DEBUG) << __func__ << " InfoDriverSeatValue:" << infoDriverSeatValue;
                            if (infoDriverSeatValue != FCP_VALUE_ERROR) {
                                internalPropValue.value()->value.int32Values[0] = infoDriverSeatValue;
                                internalPropValue.value()->status = VehiclePropertyStatus::AVAILABLE;
                            } else {
                                internalPropValue.value()->status = VehiclePropertyStatus::ERROR;
                            }
                            internalPropValue.value()->timestamp = elapsedRealtimeNano();
                            mServerSidePropStore->writeValue(mValuePool->obtain(*internalPropValue.value()), true);
                            break;
                        case toInt(VehicleProperty::INFO_EV_BATTERY_CAPACITY):
                            infoEvBatteryCapacity = FcpComm::getInstance()->getInfoEvBatteryCapacity();
                            LOG(DEBUG) << __func__ << " InfoEvBatteryCapacity:" << infoEvBatteryCapacity;
                            if (infoEvBatteryCapacity == FCP_VALUE_ERROR) {
                                internalPropValue.value()->status = VehiclePropertyStatus::ERROR;
                                internalPropValue.value()->timestamp = elapsedRealtimeNano();
                                mServerSidePropStore->writeValue(mValuePool->obtain(*internalPropValue.value()), true);
                            } else if (infoEvBatteryCapacity == FCP_VALUE_ACCESS_NONE) {
                                VehiclePropConfig config = {.prop = tempPropId};
                                mServerSidePropStore->removeConfig(config);
                            } else {
                                internalPropValue.value()->value.floatValues[0] = infoEvBatteryCapacity;
                                internalPropValue.value()->status = VehiclePropertyStatus::AVAILABLE;
                                internalPropValue.value()->timestamp = elapsedRealtimeNano();
                                mServerSidePropStore->writeValue(mValuePool->obtain(*internalPropValue.value()), true);
                            }
                            break;
                        case toInt(VehicleProperty::INFO_EV_CONNECTOR_TYPE):
                            infoEvConnectorType = FcpComm::getInstance()->getInfoEvConnectorType();
                            LOG(DEBUG) << __func__ << " InfoEvConnectorType:" << infoEvConnectorType;
                            if (infoEvConnectorType == FCP_VALUE_ERROR) {
                                internalPropValue.value()->status = VehiclePropertyStatus::ERROR;
                                internalPropValue.value()->timestamp = elapsedRealtimeNano();
                                mServerSidePropStore->writeValue(mValuePool->obtain(*internalPropValue.value()), true);
                            } else if (infoEvConnectorType == FCP_VALUE_ACCESS_NONE) {
                                VehiclePropConfig config = {.prop = tempPropId};
                                mServerSidePropStore->removeConfig(config);
                            } else {
                                internalPropValue.value()->value.int32Values[0] = infoEvConnectorType;
                                internalPropValue.value()->status = VehiclePropertyStatus::AVAILABLE;
                                internalPropValue.value()->timestamp = elapsedRealtimeNano();
                                mServerSidePropStore->writeValue(mValuePool->obtain(*internalPropValue.value()), true);
                            }
                            break;
                        case toInt(VehicleProperty::INFO_EV_PORT_LOCATION):
                            infoEvPortLocation = FcpComm::getInstance()->getInfoEvPortLocation();
                            LOG(DEBUG) << __func__ << " InfoEvPortLocation:" << infoEvPortLocation;
                            if (infoEvPortLocation == FCP_VALUE_ERROR) {
                                internalPropValue.value()->status = VehiclePropertyStatus::ERROR;
                                internalPropValue.value()->timestamp = elapsedRealtimeNano();
                                mServerSidePropStore->writeValue(mValuePool->obtain(*internalPropValue.value()), true);
                            } else if (infoEvPortLocation == FCP_VALUE_ACCESS_NONE) {
                                VehiclePropConfig config = {.prop = tempPropId};
                                mServerSidePropStore->removeConfig(config);
                            } else {
                                internalPropValue.value()->value.int32Values[0] = infoEvPortLocation;
                                internalPropValue.value()->status = VehiclePropertyStatus::AVAILABLE;
                                internalPropValue.value()->timestamp = elapsedRealtimeNano();
                                mServerSidePropStore->writeValue(mValuePool->obtain(*internalPropValue.value()), true);
                            }
                            break;
                        case toInt(VehicleProperty::INFO_FUEL_CAPACITY):
                            infoFuelCapacity = FcpComm::getInstance()->getInfoFuelCapacity();
                            LOG(DEBUG) << __func__ << " InfoFuelCapacity:" << infoFuelCapacity;
                            if (infoFuelCapacity == FCP_VALUE_ERROR) {
                                internalPropValue.value()->status = VehiclePropertyStatus::ERROR;
                                internalPropValue.value()->timestamp = elapsedRealtimeNano();
                                mServerSidePropStore->writeValue(mValuePool->obtain(*internalPropValue.value()), true);
                            } else if (infoFuelCapacity == FCP_VALUE_ACCESS_NONE) {
                                VehiclePropConfig config = {.prop = tempPropId};
                                mServerSidePropStore->removeConfig(config);
                            } else {
                                internalPropValue.value()->value.floatValues[0] = infoFuelCapacity;
                                internalPropValue.value()->status = VehiclePropertyStatus::AVAILABLE;
                                internalPropValue.value()->timestamp = elapsedRealtimeNano();
                                mServerSidePropStore->writeValue(mValuePool->obtain(*internalPropValue.value()), true);
                            }
                            break;
                        case toInt(VehicleProperty::INFO_FUEL_DOOR_LOCATION):
                            infoFuelDoorLocation = FcpComm::getInstance()->getInfoFuelDoorLocation();
                            LOG(DEBUG) << __func__ << " InfoFuelDoorLocation:" << infoFuelDoorLocation;
                            internalPropValue.value()->value.int32Values[0] = infoFuelDoorLocation;
                            internalPropValue.value()->status = VehiclePropertyStatus::AVAILABLE;
                            internalPropValue.value()->timestamp = elapsedRealtimeNano();
                            mServerSidePropStore->writeValue(mValuePool->obtain(*internalPropValue.value()), true);
                            if (infoFuelDoorLocation == UNKNOWN) {
                                VehiclePropConfig config = {.prop = tempPropId};
                                mServerSidePropStore->removeConfig(config);
                            }
                            break;
                        case toInt(VehicleProperty::INFO_FUEL_TYPE):
                            infoFuelType = FcpComm::getInstance()->getInfoFuelType();
                            LOG(DEBUG) << __func__ << " InfoFuelType:" << infoFuelType;
                            if (infoFuelType != FCP_VALUE_ERROR) {
                                internalPropValue.value()->value.int32Values[0] = infoFuelType;
                                internalPropValue.value()->status = VehiclePropertyStatus::AVAILABLE;
                            } else {
                                internalPropValue.value()->status = VehiclePropertyStatus::ERROR;
                            }
                            internalPropValue.value()->timestamp = elapsedRealtimeNano();
                            mServerSidePropStore->writeValue(mValuePool->obtain(*internalPropValue.value()), true);
                            break;
                        case toInt(VehicleProperty::INFO_MODEL):
                            infoModel = FcpComm::getInstance()->getInfoModel();
                            LOG(DEBUG) << __func__ << " InfoModel:" << infoModel;
                            if (infoModel.compare(FCP_STRING_VALUE_ERROR)) {
                                internalPropValue.value()->value.stringValue = infoModel;
                                internalPropValue.value()->status = VehiclePropertyStatus::AVAILABLE;
                            } else {
                                internalPropValue.value()->status = VehiclePropertyStatus::ERROR;
                            }
                            internalPropValue.value()->timestamp = elapsedRealtimeNano();
                            mServerSidePropStore->writeValue(mValuePool->obtain(*internalPropValue.value()), true);
                            break;
                        case toInt(VehicleProperty::INFO_MODEL_YEAR):
                            infoModelYear = FcpComm::getInstance()->getInfoModelYear();
                            LOG(DEBUG) << __func__ << " InfoModelYear:" << infoModelYear;
                            if (infoModelYear != FCP_VALUE_ERROR) {
                                internalPropValue.value()->value.int32Values[0] = infoModelYear;
                                internalPropValue.value()->status = VehiclePropertyStatus::AVAILABLE;
                            } else {
                                internalPropValue.value()->status = VehiclePropertyStatus::ERROR;
                            }
                            internalPropValue.value()->timestamp = elapsedRealtimeNano();
                            mServerSidePropStore->writeValue(mValuePool->obtain(*internalPropValue.value()), true);
                            break;
                        case toInt(VehicleProperty::EV_BATTERY_INSTANTANEOUS_CHARGE_RATE):
                            evBatteryInstantaneousChargeRate = FcpComm::getInstance()->getEvBatteryInstantaneousChargeRate();
                            LOG(DEBUG) << __func__ << " EvBatteryInstantaneousChargeRate:" << evBatteryInstantaneousChargeRate;
                            if (evBatteryInstantaneousChargeRate == FCP_VALUE_ERROR) {
                                internalPropValue.value()->status = VehiclePropertyStatus::ERROR;
                                internalPropValue.value()->timestamp = elapsedRealtimeNano();
                                mServerSidePropStore->writeValue(mValuePool->obtain(*internalPropValue.value()), true);
                            } else if (evBatteryInstantaneousChargeRate == FCP_VALUE_ACCESS_NONE) {
                                VehiclePropConfig config = {.prop = tempPropId};
                                mServerSidePropStore->removeConfig(config);
                            } else {
                                internalPropValue.value()->status = VehiclePropertyStatus::AVAILABLE;
                                internalPropValue.value()->timestamp = elapsedRealtimeNano();
                                mServerSidePropStore->writeValue(mValuePool->obtain(*internalPropValue.value()), true);
                            }
                            break;
                        case toInt(VehicleProperty::EV_BATTERY_LEVEL):
                            evBatteryLevel = FcpComm::getInstance()->getEvBatteryLevel();
                            LOG(DEBUG) << __func__ << " EvBatteryLevel:" << evBatteryLevel;
                            if (evBatteryLevel == FCP_VALUE_ERROR) {
                                internalPropValue.value()->status = VehiclePropertyStatus::ERROR;
                                internalPropValue.value()->timestamp = elapsedRealtimeNano();
                                mServerSidePropStore->writeValue(mValuePool->obtain(*internalPropValue.value()), true);
                            } else if (evBatteryLevel == FCP_VALUE_ACCESS_NONE) {
                                VehiclePropConfig config = {.prop = tempPropId};
                                mServerSidePropStore->removeConfig(config);
                            } else {
                                internalPropValue.value()->status = VehiclePropertyStatus::AVAILABLE;
                                internalPropValue.value()->timestamp = elapsedRealtimeNano();
                                mServerSidePropStore->writeValue(mValuePool->obtain(*internalPropValue.value()), true);
                            }
                            break;
                        case toInt(VehicleProperty::EV_BATTERY_DISPLAY_UNITS):
                            evBatteryDisplayUnits = FcpComm::getInstance()->getEvBatteryDisplayUnits();
                            if (evBatteryDisplayUnits == FCP_VALUE_ERROR) {
                                internalPropValue.value()->status = VehiclePropertyStatus::ERROR;
                                internalPropValue.value()->timestamp = elapsedRealtimeNano();
                                mServerSidePropStore->writeValue(mValuePool->obtain(*internalPropValue.value()), true);
                            } else if (evBatteryDisplayUnits == FCP_VALUE_ACCESS_NONE) {
                                VehiclePropConfig config = { .prop = tempPropId };
                                mServerSidePropStore->removeConfig(config);
                            } else {
                                internalPropValue.value()->status = VehiclePropertyStatus::AVAILABLE;
                                internalPropValue.value()->timestamp = elapsedRealtimeNano();
                                mServerSidePropStore->writeValue(mValuePool->obtain(*internalPropValue.value()), true);
                            }
                            break;
                        case toInt(VehicleProperty::EV_CHARGE_CURRENT_DRAW_LIMIT):
                            evChargeCurrentDrawLimit = FcpComm::getInstance()->getEvChargeCurrentDrawLimit();
                            if (evChargeCurrentDrawLimit == FCP_VALUE_ERROR) {
                                internalPropValue.value()->status = VehiclePropertyStatus::ERROR;
                                internalPropValue.value()->timestamp = elapsedRealtimeNano();
                                mServerSidePropStore->writeValue(mValuePool->obtain(*internalPropValue.value()), true);
                            } else if (evChargeCurrentDrawLimit == FCP_VALUE_ACCESS_NONE) {
                                VehiclePropConfig config = { .prop = tempPropId };
                                mServerSidePropStore->removeConfig(config);
                            } else {
                                internalPropValue.value()->status = VehiclePropertyStatus::AVAILABLE;
                                internalPropValue.value()->timestamp = elapsedRealtimeNano();
                                mServerSidePropStore->writeValue(mValuePool->obtain(*internalPropValue.value()), true);
                            }
                            break;
                        case toInt(VehicleProperty::EV_CHARGE_PERCENT_LIMIT):
                            evChargePercentLimit = FcpComm::getInstance()->getEvChargePercentLimit();
                            if (evChargePercentLimit == FCP_VALUE_ERROR) {
                                internalPropValue.value()->status = VehiclePropertyStatus::ERROR;
                                internalPropValue.value()->timestamp = elapsedRealtimeNano();
                                mServerSidePropStore->writeValue(mValuePool->obtain(*internalPropValue.value()), true);
                            } else if (evChargePercentLimit == FCP_VALUE_ACCESS_NONE) {
                                VehiclePropConfig config = { .prop = tempPropId };
                                mServerSidePropStore->removeConfig(config);
                            } else {
                                internalPropValue.value()->status = VehiclePropertyStatus::AVAILABLE;
                                internalPropValue.value()->timestamp = elapsedRealtimeNano();
                                mServerSidePropStore->writeValue(mValuePool->obtain(*internalPropValue.value()), true);
                            }
                            break;
                        case toInt(VehicleProperty::EV_CHARGE_PORT_CONNECTED):
                        case toInt(VehicleProperty::EV_CHARGE_PORT_OPEN):
                            evChargePortInfo = FcpComm::getInstance()->getEvChargePortInfo();
                            if (evChargePortInfo == FCP_VALUE_ERROR) {
                                internalPropValue.value()->status = VehiclePropertyStatus::ERROR;
                                internalPropValue.value()->timestamp = elapsedRealtimeNano();
                                mServerSidePropStore->writeValue(mValuePool->obtain(*internalPropValue.value()), true);
                            } else if (evChargePortInfo == FCP_VALUE_ACCESS_NONE) {
                                VehiclePropConfig config = { .prop = tempPropId };
                                mServerSidePropStore->removeConfig(config);
                            } else {
                                internalPropValue.value()->status = VehiclePropertyStatus::AVAILABLE;
                                internalPropValue.value()->timestamp = elapsedRealtimeNano();
                                mServerSidePropStore->writeValue(mValuePool->obtain(*internalPropValue.value()), true);
                            }
                            break;
                        default:
                            break;
                        }
                    }
                }
            }
        }
    }
}

void FakeVehicleHardware::initPropertyForSuspend() {
    LOG(INFO) << __func__ << " start";
    for (size_t i = 0; i < defaultconfig::getDefaultConfigs().size(); i++) {
        int32_t tempPropId = defaultconfig::getDefaultConfigs()[i].config.prop;
        int32_t result = tempPropId & toInt(VehiclePropertyGroup::VENDOR);
        // Honda VP extraction.
        if (result == toInt(VehiclePropertyGroup::VENDOR)) {
            size_t areaConfigSize = defaultconfig::getDefaultConfigs()[i].config.areaConfigs.size();
            for (size_t j = 0; j <= areaConfigSize; j++) {
                int tempAreaId = 0;
                if (areaConfigSize != 0) {
                    if (j == areaConfigSize) {
                        continue;
                    }
                    tempAreaId = defaultconfig::getDefaultConfigs()[i].config.areaConfigs[j].areaId;
                }
                auto internalPropValue = mServerSidePropStore->readValue(tempPropId, tempAreaId, 0);
                if (internalPropValue.ok()) {
                    internalPropValue.value()->status = VehiclePropertyStatus::UNAVAILABLE;
                    internalPropValue.value()->timestamp = elapsedRealtimeNano();
                    writeValueAndNotifyChange(*internalPropValue.value(), true);
                }
            }
        }
    }
}

void FakeVehicleHardware::initPropertyForResume() {
    LOG(INFO) << __func__ << " start";
    for (size_t i = 0; i < defaultconfig::getDefaultConfigs().size(); i++) {
        int32_t tempPropId = defaultconfig::getDefaultConfigs()[i].config.prop;
        int32_t result = tempPropId & toInt(VehiclePropertyGroup::VENDOR);
        // Honda VP extraction.
        if (result == toInt(VehiclePropertyGroup::VENDOR)) {
            size_t areaConfigSize = defaultconfig::getDefaultConfigs()[i].config.areaConfigs.size();
            for (size_t j = 0; j <= areaConfigSize; j++) {
                int tempAreaId = 0;
                if (areaConfigSize != 0) {
                    if (j == areaConfigSize) {
                        continue;
                    }
                    tempAreaId = defaultconfig::getDefaultConfigs()[i].config.areaConfigs[j].areaId;
                }
                auto internalPropValue = mServerSidePropStore->readValue(tempPropId, tempAreaId, 0);
                if (internalPropValue.ok()) {
                    // Set the initial value for the VP to be initialized by resume.
                    auto ite = std::find(kInitializeInResume.begin(), kInitializeInResume.end(), tempPropId);
                    if (ite != kInitializeInResume.end()) {
                        internalPropValue.value()->value = defaultconfig::getDefaultConfigs()[i].initialValue;
                    }
                    internalPropValue.value()->status = VehiclePropertyStatus::AVAILABLE;
                    internalPropValue.value()->timestamp = elapsedRealtimeNano();
                    writeValueAndNotifyChange(*internalPropValue.value(), true);
                }
            }
        }

        result = tempPropId & toInt(VehiclePropertyGroup::SYSTEM);
        // AOSP VP extraction.
        if (result == toInt(VehiclePropertyGroup::SYSTEM)) {
            // Set the initial value for the VP to be initialized by resume.
            auto ite = std::find(kInitializeInResume.begin(), kInitializeInResume.end(), tempPropId);
            if (ite != kInitializeInResume.end()) {
                size_t areaConfigSize = defaultconfig::getDefaultConfigs()[i].config.areaConfigs.size();
                for (size_t j = 0; j <= areaConfigSize; j++) {
                    int tempAreaId = 0;
                    if (areaConfigSize != 0) {
                        if (j == areaConfigSize) {
                            continue;
                        }
                        tempAreaId = defaultconfig::getDefaultConfigs()[i].config.areaConfigs[j].areaId;
                    }
                    auto internalPropValue = mServerSidePropStore->readValue(tempPropId, tempAreaId, 0);
                    if (internalPropValue.ok()) {
                        internalPropValue.value()->value = defaultconfig::getDefaultConfigs()[i].initialValue;
                        internalPropValue.value()->status = VehiclePropertyStatus::AVAILABLE;
                        internalPropValue.value()->timestamp = elapsedRealtimeNano();
                        writeValueAndNotifyChange(*internalPropValue.value(), true);
                    }
                }
            }
        }
    }
    setSuspendExitEvent(true);
    writeValueAndNotifyChange(createHondaIcbNotification(toInt(AccState::ACC_ON), toInt(KeyofftimerStartUpState::OFF),
                                                      toInt(HalInitializeState::COMPLETED)),
                           true);
}

void FakeVehicleHardware::initPropertyForShutdown() {
    LOG(INFO) << __func__ << " start";
    for (size_t i = 0; i < defaultconfig::getDefaultConfigs().size(); i++) {
        int32_t tempPropId = defaultconfig::getDefaultConfigs()[i].config.prop;
        int32_t result = tempPropId & toInt(VehiclePropertyGroup::VENDOR);
        // Honda VP extraction.
        if (result == toInt(VehiclePropertyGroup::VENDOR)) {
            size_t areaConfigSize = defaultconfig::getDefaultConfigs()[i].config.areaConfigs.size();
            for (size_t j = 0; j <= areaConfigSize; j++) {
                int tempAreaId = 0;
                if (areaConfigSize != 0) {
                    if (j == areaConfigSize) {
                        continue;
                    }
                    tempAreaId = defaultconfig::getDefaultConfigs()[i].config.areaConfigs[j].areaId;
                }
                auto internalPropValue = mServerSidePropStore->readValue(tempPropId, tempAreaId, 0);
                if (internalPropValue.ok()) {
                    internalPropValue.value()->status = VehiclePropertyStatus::UNAVAILABLE;
                    internalPropValue.value()->timestamp = elapsedRealtimeNano();
                    writeValueAndNotifyChange(*internalPropValue.value(), true);
                }
            }
        }
    }
    writeValueAndNotifyChange(createHondaIcbNotification(toInt(AccState::ACC_OFF), toInt(KeyofftimerStartUpState::OFF),
                                                      toInt(HalInitializeState::IMCOMPLETE)),
                           true);
    writeNonVolatileVP();
}

void FakeVehicleHardware::initPropertyThread() {
    LOG(INFO) << __func__ << " PowerState:" << mPowerState;
    if (mPowerState == WAIT_FOR_VHAL) {
        if (!mIsColdStartFinish) {
            mIsColdStartFinish = true;
            writeValueAndNotifyChange(createHondaIcbNotification(toInt(AccState::ACC_ON),
                                                              toInt(KeyofftimerStartUpState::OFF),
                                                              toInt(HalInitializeState::COMPLETED)),
                                   true);
        } else if (mIsResumeStart) {
            initPropertyForResume();
            mIsResumeStart = false;
        } else {
            writeValueAndNotifyChange(createHondaIcbNotification(toInt(AccState::ACC_ON),
                                                              toInt(KeyofftimerStartUpState::OFF),
                                                              toInt(HalInitializeState::COMPLETED)),
                                   true);
        }
    } else if (mPowerState == SUSPEND_TO_RAM) {
        initPropertyForSuspend();
    } else {
        initPropertyForShutdown();
    }
}

void FakeVehicleHardware::processPowerStateEvent(const VehiclePropValue &value) {
    LOG(DEBUG) << __func__ << " start";
    switch (value.prop) {
    case HONDA_POWER_STATE_REQ:
        switch (value.value.int32Values[0]) {
        case toInt(VehicleHondaPowerStateReq::PREPARE_OFF):
            if (mPowerState == ON) {
                mPowerState = SHUTDOWN_PREPERE;
                LOG(INFO) << __func__ << " State transitions from ON to SHUTDOWN_PREPERE";
                startPropertyInitializingThread();
            } else {
                LOG(WARNING) << __func__ << " Illegal state :" << mPowerState;
            }
            break;
        case toInt(VehicleHondaPowerStateReq::NORMAL_RUNNING):
            if (mPowerState == WAIT_FOR_VHAL) {
               // mPowerState = ON;
                LOG(INFO) << __func__ << " Cold boot gating active, stay in WAIT_FOR_VHAL";
            } else {
                LOG(WARNING) << __func__ << " Illegal state :" << mPowerState;
            }
            break;
        default:
            break;
        }
        break;
    case AP_POWER_STATE_REPORT:
        switch (value.value.int32Values[0]) {
        case toInt(VehicleApPowerStateReport::WAIT_FOR_VHAL):
            mPowerState = WAIT_FOR_VHAL;
            mIsResumeStart = false;
            mIsColdStartFinish = false;
            startPropertyInitializingThread();
            break;
        case toInt(VehicleApPowerStateReport::DEEP_SLEEP_ENTRY):
            stopGarageModeTimer();
            mPowerState = SUSPEND_TO_RAM;
            LOG(INFO) << __func__ << " State transitions to SUSPEND_TO_RAM";
            startPropertyInitializingThread();
            break;
        case toInt(VehicleApPowerStateReport::SHUTDOWN_CANCELLED):
            mPowerState = WAIT_FOR_VHAL;
            mIsResumeStart = true;
            startPropertyInitializingThread();
            break;
        default:
            break;
        }
        break;
    case IGNITION_STATE:
        LOG(INFO) << __func__ << " IGNITION_STATE value:" << value.value.int32Values[0];
        switch (value.value.int32Values[0]) {
        case IGNITION_STATE_ACC:
            if (mPowerState == SUSPEND_TO_RAM) {
                mPowerState = WAIT_FOR_VHAL;
                LOG(INFO) << __func__ << " State transitions from SUSPEND_TO_RAM to WAIT_FOR_VHAL";
                stopHondaPowerTimer();
                startPropertyInitializingThread();
                mIsResumeStart = true;
            } else {
                stopGarageModeTimer();
                writeValueAndNotifyChange(createHondaIcbNotification(toInt(AccState::ACC_ON),
                                                                  toInt(KeyofftimerStartUpState::OFF),
                                                                  toInt(HalInitializeState::IMCOMPLETE)),
                                       true);
                writeValueAndNotifyChange(*createApPowerStateReq(VehicleApPowerStateReq::CANCEL_SHUTDOWN),
                                       true /* updateStatus */);
            }
            break;
        case IGNITION_STATE_OFF:
            startHondaPowerTimer();
            break;
        default:
            break;
        }
        break;
    default:
        break;
    }
}

void FakeVehicleHardware::processHondaCustomizeEvent(const VehiclePropValue& value) {
    switch (value.prop) {
    case HONDA_CUSTOMIZE_REQUEST:
    case HONDA_CUSTOMIZE_CUSTOM_GET_REQUEST:
    case HONDA_CUSTOMIZE_CUSTOM_SET_REQUEST:
        processDirectCommunicationEvent(value);
        break;
    case HONDA_CUSTOMIZE_ACC_BUZZER:
    case HONDA_CUSTOMIZE_DWS_INIT_REQUEST_RECEIVED:
    case HONDA_CUSTOMIZE_LKAS_BUZZER:
    case HONDA_CUSTOMIZE_CMBS_ALART:
    case HONDA_CUSTOMIZE_SIF:
    case HONDA_CUSTOMIZE_INTELLIGENT_LED_CST_STT:
    case HONDA_CUSTOMIZE_DAAS:
    case HONDA_CUSTOMIZE_BSI_SCM_CUST:
    case HONDA_CUSTOMIZE_ALC_CUST:
    case HONDA_CUSTOMIZE_EAS_LC:
    case HONDA_CUSTOMIZE_EAS_PH:
    case HONDA_CUSTOMIZE_FCTW_ATT:
    case HONDA_CUSTOMIZE_ACC_CSA:
    case HONDA_CUSTOMIZE_PKS_RR:
    case HONDA_CUSTOMIZE_IACC_SETUP:
    case HONDA_CUSTOMIZE_REV_MATCH:
    case HONDA_CUSTOMIZE_PEAEMG:
    case HONDA_CUSTOMIZE_TSR_SRBUZ:
    case HONDA_CUSTOMIZE_IDS_ENGINE:
    case HONDA_CUSTOMIZE_IDS_SUSPENSION:
    case HONDA_CUSTOMIZE_IDS_STEERING:
    case HONDA_CUSTOMIZE_IDS_GAUGE:
    case HONDA_CUSTOMIZE_IDS_IDLESTOP:
    case HONDA_CUSTOMIZE_IDS_REVMATCH:
    case HONDA_CUSTOMIZE_IDS_PTSOUND:
    case HONDA_CUSTOMIZE_IDS_ACC:
    case HONDA_CUSTOMIZE_IDS_LIGHTING:
    case HONDA_EVS_ONOFF_STATUS:
    case HONDA_EVS_IDS_COLLABO_STATUS:
    case HONDA_EVS_SOUND_TYPE_STATUS:
    case HONDA_EVS_VOLUME_STATUS:
    case HONDA_EVS_BALANCE_TYPE_ALL_STATUS:
    case HONDA_EVS_BALANCE_TYPE_DR_STATUS:
    case HONDA_EVS_BALANCE_TYPE_FR_STATUS:
    case HONDA_EVS_BALANCE_TYPE_RR_STATUS:
    case HONDA_EVS_SOUND_TYPE_PRESET_STATUS:
    case HONDA_CUSTOMIZE_APS_AUTO_EPB:
    case HONDA_CUSTOMIZE_APS_DETECT_SOUND:
    case HONDA_CUSTOMIZE_APS_MEMORY_DELE:
    case HONDA_ROFMOD_THEME_COLOR:
    case HONDA_ROFMOD_ROOMLIGHT:
    case HONDA_ROFMOD_BRI:
    case HONDA_LID_DOORLOCK_SW:
    case HONDA_CUSTOMIZE_ELM_RRFOG_CHANGE_MODE:
    case HONDA_CUSTOMIZE_ELM_LIGHT_CHANGE_MODE:
    case HONDA_CUSTOMIZE_ELM_FRFOG_CHANGE_MODE:
    case HONDA_CUSTOMIZE_DPMS_DR_MASG_MODE:
    case HONDA_CUSTOMIZE_DPMS_AS_MASG_MODE:
    case HONDA_CUSTOMIZE_DPMS_MIRCOUNT1:
    case HONDA_CUSTOMIZE_DPMS_MIR_SELECT:
    case HONDA_CUSTOMIZE_DPMS_MIR_REVERSE:
    case HONDA_ROFMOD_SETTING_FB: 
    case HONDA_CUSTOMIZE_GAUGE_PATTERN:
    case HONDA_CUSTOMIZE_WARNING_MSG:
    case HONDA_CUSTOMIZE_EXTEMP_DISP_C:
    case HONDA_CUSTOMIZE_EXTEMP_DISP_F:
    case HONDA_CUSTOMIZE_MTGEAR_DISP:
    case HONDA_CUSTOMIZE_TRIPA_RESET_GAS:
    case HONDA_CUSTOMIZE_TRIPA_RESET_PHEV:
    case HONDA_CUSTOMIZE_TRIPA_RESET_BEV:
    case HONDA_CUSTOMIZE_TRIPA_RESET_FCV:
    case HONDA_CUSTOMIZE_TRIPB_RESET_GAS:
    case HONDA_CUSTOMIZE_TRIPB_RESET_PHEV:
    case HONDA_CUSTOMIZE_TRIPB_RESET_BEV:
    case HONDA_CUSTOMIZE_TRIPB_RESET_FCV:
    case HONDA_CUSTOMIZE_ALARM_VOL:
    case HONDA_CUSTOMIZE_REVERSE_ALARM:
    case HONDA_CUSTOMIZE_REV_INDICATOR:
    case HONDA_CUSTOMIZE_AMBIENT_METER:
    case HONDA_CUSTOMIZE_IDLESTOP_GUIDANCE:
    case HONDA_CUSTOMIZE_TBT_DISPLAY:
    case HONDA_CUSTOMIZE_REARSEAT_REMINDER:
    case HONDA_CUSTOMIZE_DRIVERANGE_NOTIFY:
    case HONDA_CUSTOMIZE_TIREANGLE_MONITOR:
    case HONDA_CUSTOMIZE_DRIVE_UNIT:
    case HONDA_CUSTOMIZE_CONSUMPTION_UNIT_KM:
    case HONDA_CUSTOMIZE_SHIFTUP_TIMING_ALARM:
    case HONDA_CUSTOMIZE_LANGUAGE_EU:
    case HONDA_CUSTOMIZE_LANGUAGE_US:
    case HONDA_CUSTOMIZE_LANGUAGE_CN:
    case HONDA_CUSTOMIZE_LANGUAGE_PT:
    case HONDA_CUSTOMIZE_LANGUAGE_KR:
    case HONDA_CUSTOMIZE_GAUGE_PATTERN_PF3:
    case HONDA_CUSTOMIZE_TBT_DISPLAY_HUD:
    case HONDA_CUSTOMIZE_ADAS_DISPLAY_HUD:
    case HONDA_CUSTOMIZE_SPEEDTSR_DISPLAY_HUD:
    case HONDA_CUSTOMIZE_SPEED_DISPLAY_HUD:
    case HONDA_CUSTOMIZE_DETAIL_DISPLAY_HUD:
    case HONDA_CUSTOMIZE_TRAIL_GAUGE:
    case HONDA_CUSTOMIZE_MET_PS_MID:
    case HONDA_CUSTOMIZE_GET_DID_SETTING:
    case HONDA_CUSTOMIZE_SW_TURN_MODE:
    case HONDA_CUSTOMIZE_EAS_PARK_MODE:
    case HONDA_CUSTOMIZE_EAS_WELCOME_MODE:
    case HONDA_CUSTOMIZE_IDS_RESET_STATUS2:
    case HONDA_CUSTOMIZE_IDS_ENGINE2:
    case HONDA_CUSTOMIZE_IDS_STEERING2:
    case HONDA_CUSTOMIZE_IDS_SUSPENSION2:
    case HONDA_CUSTOMIZE_IDS_AWD2:
    case HONDA_CUSTOMIZE_PW_SW_LOCK:
    case HONDA_CUSTOMIZE_SUNLOOF_KEYLESS:
    case HONDA_CUSTOMIZE_WIP_MAINTENANCE_MODE:
    case HONDA_CUSTOMIZE_SEAT_DIRECTION:
    case HONDA_CUSTOMIZE_SEATCOLUMN_DIRECTION:
    case HONDA_CUSTOMIZE_IDS_PTSOUND2:
    case HONDA_CUSTOMIZE_ADAS_NOTIF_LED_CST_STT:
    case HONDA_CUSTOMIZE_TLI_RED_FUN:
    case HONDA_CUSTOMIZE_TLI_GREEN_FUN:
    case HONDA_METER_HUD_ONOFF:
    case HONDA_DIMMING_GLASS_STATE:
    case HONDA_CUSTOMIZE_MVC_INTERLOCK:
    case HONDA_CUSTOMIZE_REFERLINE:
    case HONDA_CUSTOMIZE_HUD_CONTENTS:
    case HONDA_CUSTOMIZE_DID_CONTENTS:
    case HONDA_CUSTOMIZE_TBT_DISPLAY_DID:
    case HONDA_CUSTOMIZE_AMN_DISPLAY_DID:
    case HONDA_CUSTOMIZE_AUDIO_DISPLAY_DID:
    case HONDA_CUSTOMIZE_HFL_DISPLAY_DID:
    case HONDA_CUSTOMIZE_SR_DISPLAY_DID:
    case HONDA_CUSTOMIZE_SMS_DISPLAY_DID:
    case HONDA_CUSTOMIZE_COMPANION_DISPLAY_DID:
    case HONDA_CUSTOMIZE_DPMS_DR_MASG_STATUS:
    case HONDA_CUSTOMIZE_DPMS_AS_MASG_STATUS:
    case HONDA_HVAC_TEMPVARIATION:
    case HONDA_HVAC_RR_FUNC_FEATURE:
    case HONDA_CUSTOMIZE_HVAC_LEARN_AUTO_FUNC:
    case HONDA_METER_LANGUAGE_RECEIVE_RESPONSE:
    case HONDA_ENTRY_CPD_VEHICLE_CONFIG:
    case HONDA_LID_DR_OPCL_SW:
    case HONDA_LID_AS_OPCL_SW:
    case HONDA_LID_RD_OPCL_SW:
    case HONDA_LID_RA_OPCL_SW:
    case HONDA_CUSTOMIZE_AD_MAIN_STT:
    case HONDA_CUSTOMIZE_ACC_CC_MODE_STT:
    case HONDA_CUSTOMIZE_ACC_LIM_MODE_STT:
    case HONDA_CUSTOMIZE_LKAS_ENABLE_STT:
    case HONDA_CUSTOMIZE_AHD_CUST_ONOFF:
    case HONDA_CUSTOMIZE_ACC_DIST_STT:
    case HONDA_CUSTOMIZE_ACC_SLA_START_SPEED_STT:
    case HONDA_CUSTOMIZE_IACC_ONOFF_STT:
    case HONDA_CUSTOMIZE_ACC_SLA_MODE_WO_AUTO_STT:
    case HONDA_CUSTOMIZE_ACC_SLA_MODE_W_AUTO_STT:
    case HONDA_CUSTOMIZE_ACC_SLA_OFFSET_KPH_STT:
    case HONDA_CUSTOMIZE_ACC_SLA_OFFSET_MPH_STT:
    case HONDA_CUSTOMIZE_ACC_BLINK_AXEL_STT:
    case HONDA_CUSTOMIZE_ACC_RESUME_ONOFF_STT:
    case HONDA_CUSTOMIZE_ACC_AXELMODE_STT:
    case HONDA_CUSTOMIZE_ADA_ONOFF_STT:
    case HONDA_CUSTOMIZE_ADA_FUNC_STT:
    case HONDA_CUSTOMIZE_ADA_LONINTENSITY_STT:
    case HONDA_CUSTOMIZE_ADA_LONCURVE_STT:
    case HONDA_CUSTOMIZE_ADA_LATINTENSITY_STT:
    case HONDA_CUSTOMIZE_AILD_HANDS_OFF_ALARM_STT:
    case HONDA_CUSTOMIZE_ALCA_TURN_LEVER_CUST_STT:
    case HONDA_CUSTOMIZE_ALCA_ALC_CUST:
    case HONDA_CUSTOMIZE_ALCR_ALC_ONOFF_CUST_STT:
    case HONDA_CUSTOMIZE_ALCR_PAS_CUST_RESULT:
    case HONDA_CUSTOMIZE_ALC_NAVI_CUST_STT:
    case HONDA_CUSTOMIZE_ALCR_NAV_CUST_RESULT:
    case HONDA_CUSTOMIZE_AMN_ONOFF_STT:
    case HONDA_CUSTOMIZE_AMN_NAV_TIMING_STT:
    case HONDA_CUSTOMIZE_AMN_HOV_ENABLE_STT:
    case HONDA_CUSTOMIZE_AMN_EXPRESS_ENABLE_STT:
    case HONDA_CUSTOMIZE_AMN_INTERRUPT_DISP_STT:
    case HONDA_CUSTOMIZE_ADAS_NOTIF_VIB_CST_STT:
    case HONDA_BIOENT_CUSTOM_MENU_BIOENT_INTERFACE:
    case HONDA_CUSTOMIZE_WELCOME_SOUND:
    case HONDA_CUSTOM_ADAS_E_BSI_ONOFF:
    case HONDA_CUSTOM_ADAS_E_BSI_RANGE:
    case HONDA_CUSTOM_ADAS_E_BSI_2R:
    case HONDA_CUSTOMIZE_C_LID_AUTO:
    case HONDA_CUSTOMIZE_EC_LID_CLOSE_TIMER:
    case HONDA_CUSTOMIZE_EC_LID_AUTO_CLOSE:
    case HONDA_CUSTOM_ADAS_STATUS_CMBS:
    case HONDA_CUSTOM_ADAS_FCTW:
    case HONDA_CUSTOM_DAM_ALLOFF_SW:
    case HONDA_CUSTOM_ADAS_AILD_ENABLE:
    case HONDA_CUSTOM_ADAS_ALC_ONOFF_CUST:
    case HONDA_CUSTOM_ADAS_E_BSI_CUST:
    case HONDA_CUSTOM_ADAS_SAFETY_SW_STATUS_PKS:
    case HONDA_CUSTOM_ADAS_SAFETY_SW_STATUS_LSAEB:
    case HONDA_CUSTOM_ADAS_PKS_RR_STATUS:
    case HONDA_CORE_CUSTOM_SNP_CRP:
    case HONDA_CUSTOMIZE_RDMS_ONE:
    case HONDA_CUSTOMIZE_RDMS_TWO:
    case HONDA_CUSTOMIZE_EW_CUST_RESULT:
    case HONDA_CUSTOMIZE_DMC_DROWSY_SETTING:
    case HONDA_CUSTOMIZE_DMC_DIST_SETTING:
    case HONDA_CUSTOMIZE_AES_ONOFF_STT:
    case HONDA_CUSTOMIZE_DESS_CUST_STT:
    case HONDA_CUSTOMIZE_EW_FUNC_ONOFF_STT:
    case HONDA_CUSTOMIZE_EW_LV1_ONOFF_STT:
    case HONDA_CUSTOMIZE_EW_ELATCH_ONOFF_STT:
    case HONDA_CUSTOMIZE_EW_ELATCH_SENSE_STT:
    case HONDA_CUSTOMIZE_BSINFO_VCHANGE_BCF:
    case HONDA_CUSTOMIZE_ALARM_TYPE:
    case HONDA_CUSTOMIZE_BSI_BUZZER_VOLUME_STT:
    case HONDA_CUSTOMIZE_BSI_RANGE_STT:
    case HONDA_CUSTOMIZE_BSI_2R_STT:
    case HONDA_CUSTOMIZE_BSI_TRAILER_STT:
    case HONDA_CUSTOMIZE_BSI_TRAILER_TYPE_STT:
    case HONDA_CUSTOMIZE_BSI_TRAILER_WIDTH_STT:
    case HONDA_CUSTOMIZE_BSI_TRAILER_LENGTH_STT:
    case HONDA_CUSTOMIZE_PCDW_ONOFF_STT:
    case HONDA_CUSTOMIZE_ODSF_ONOFF:
    case HONDA_CUSTOMIZE_FCM_INFO_TSR_SRBUZ_ONOFF:
    case HONDA_CUSTOMIZE_DAM_EYECLOSED_STT:
    case HONDA_METER_HUD_ILLUMI:
    case HONDA_CUSTOMIZE_VOL_DISPLAY_DID:
    case HONDA_CUSTOMIZE_HEADLIGHT_TIMER:
    case HONDA_CUSTOMIZE_WELCOM_LIGHT_1:
    case HONDA_CUSTOMIZE_WELCOM_LIGHT_2:
    case HONDA_CUSTOMIZE_WELCOM_LIGHT_1_2:
    case HONDA_EVS_BALANCE_STAT: {
        // ExtからのRead値設定
        EMULOG_I("set Read value : property=0x%x", value.prop);
        auto updatedPropValue = value;
        updatedPropValue.status = VehiclePropertyStatus::AVAILABLE;
        updatedPropValue.timestamp = elapsedRealtimeNano();
        writeValueAndNotifyChange(updatedPropValue, true);
        break;
    }
    case HONDA_CUSTOMIZE_HSS:
    case HONDA_CUSTOMIZE_ADB: {
        // ExtからのRead値設定
        EMULOG_I("set Read value : property=0x%x", value.prop);
        auto updatedPropValue = value;
        updatedPropValue.status = VehiclePropertyStatus::AVAILABLE;
        updatedPropValue.timestamp = elapsedRealtimeNano();
        if (hidlIfDataComm_->getHssAdbAvailVal() == value.prop) {
            writeValueAndNotifyChange(updatedPropValue, true);
        } else {
            updatedPropValue.value.byteValues[0] = 0xFF;
            writeValueAndNotifyChange(updatedPropValue, true);
        }
        break;
    }
    case HONDA_CUSTOMIZE_RDMS: {
        // ExtからのRead値設定
        EMULOG_I("set Read value : property=0x%x", value.prop);
        auto propValue = value;
        int32_t avail_val = hidlIfDataComm_->getRdmsAvailVal();
        for (int i = 1; i < 4; i++) {
            if (avail_val != i) {
                propValue.value.byteValues[i] = 0;
            }
        }
        propValue.status = VehiclePropertyStatus::AVAILABLE;
        propValue.timestamp = elapsedRealtimeNano();
        writeValueAndNotifyChange(propValue, true);
        break;
    }
    case HONDA_CUSTOMIZE_TSR_DISP_ONOFF:
    case HONDA_CUSTOMIZE_TSR_WARN_STATUS:
    case HONDA_CUSTOMIZE_TSR_WARN_KPH:
    case HONDA_CUSTOMIZE_TSR_WARN_MPH:
        // ExtからのRead値設定
        EMULOG_I("set Read value : property=0x%x", value.prop);
        updateTsrValue(value);
        break;
    case HONDA_MAINT_RESET_REQ: {
        auto updatedPropValue = value;
        updatedPropValue.prop = (int32_t)HONDA_MAINT_RESULT;
        updatedPropValue.status = VehiclePropertyStatus::AVAILABLE;
        updatedPropValue.timestamp = elapsedRealtimeNano();
        updatedPropValue.value.byteValues.resize(1);
        if (!mEmulatedCustomizeThread.isCanceled()) {
            int32_t result = hidlIfDataComm_->getMaintResult();
            if (result == CommResult::SUCCESS) {
                updatedPropValue.value.byteValues[0] = 0x01;
            } else if (result == CommResult::FAIL) {
                updatedPropValue.value.byteValues[0] = 0x02;
            } else {
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                updatedPropValue.value.byteValues[0] = 0xFF;
            }
            writeValueAndNotifyChange(updatedPropValue, true);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        updatedPropValue.timestamp = elapsedRealtimeNano();
        updatedPropValue.value.byteValues[0] = 0x00;
        writeValueAndNotifyChange(updatedPropValue, true);
        break;
    }
    case HONDA_IDS_REQ_ANSBACK:
    case HONDA_CUSTOMIZE_IDS_RESET:
    case HONDA_CUSTOMIZE_IDS_RESET_STATUS:
        processIdsEvent(value);
        break;
    default:
        processIndirectCommunicationEvent(value);
        break;
    }
}

void FakeVehicleHardware::processDirectCommunicationEvent(const VehiclePropValue& value) {
    uint16_t pfEcuId = 0x0000;
    // 閾値からキュー追加から処理開始までの時間を引いておく
    int64_t sleep_time = (int64_t)hidlIfDataComm_->getCustomizeTime(value.prop) - (elapsedRealtime() - value.timestamp);
    VehiclePropValue customizePropValue;
    switch (value.prop) {
    case HONDA_CUSTOMIZE_REQUEST:
        if (hidlIfDataComm_->getCustomizeResult(value.prop)) {
            pfEcuId = (uint16_t)value.value.byteValues[0];
            pfEcuId = (pfEcuId << 8) | value.value.byteValues[1];
            EMULOG_D("pfEcuId=0x%04x", pfEcuId);
            if (gCustomCheckVPValueSettings.find(pfEcuId) == gCustomCheckVPValueSettings.end()) {
                // 外部ファイルに設定されていない場合は初期値を設定する
                pfEcuId = HONDA_CUSTOMIZE_CUSTOMCHECK_DEFAULT_VALUE_KEY;
            }
        } else {
            // ExtCtrlでエラー状態を設定した場合は、エラー処理を行う
            pfEcuId = HONDA_CUSTOMIZE_CUSTOMCHECK_FAILURE_VALUE_KEY;
        }
        if (gCustomCheckVPValueSettings.find(pfEcuId) != gCustomCheckVPValueSettings.end()) {
            auto& propValue = gCustomCheckVPValueSettings[pfEcuId];
            propValue.status = VehiclePropertyStatus::AVAILABLE;
            propValue.timestamp = elapsedRealtimeNano();
            // 指定時間経過後に応答を返す
            if (sleep_time > 0) {
                std::this_thread::sleep_for(std::chrono::milliseconds(sleep_time));
            }
            // ここでwriteして、後から(VehicleHalManager側で)readしたデータをコールバック通知する
            writeValueAndNotifyChange(propValue, true);
        } else {
            // デフォルト値を取得して、コールバック通知する
            auto internalPropValue = mServerSidePropStore->readValue(HONDA_CUSTOMIZE_CUSTOMCHECK, 0, 0);
            // 指定時間経過後に応答を返す
            if (sleep_time > 0) {
                std::this_thread::sleep_for(std::chrono::milliseconds(sleep_time));
            }
            onPropertyValueFromCar(*internalPropValue.value(), true);
        }
        break;
    case HONDA_CUSTOMIZE_CUSTOM_GET_REQUEST:
        customizePropValue = mCustomizeCustomStore.getCustomSettingValue(hidlIfDataComm_->getCustomizeResult(value.prop), value.value.byteValues[0], value.value.byteValues[1]);
        customizePropValue.prop = (int32_t)HONDA_CUSTOMIZE_CUSTOM_GET_RESPONSE;
        customizePropValue.areaId = 0;
        customizePropValue.status = VehiclePropertyStatus::AVAILABLE;
        customizePropValue.timestamp = elapsedRealtimeNano();
        // 処理時間も考慮し、指定時間内に応答を返すようタイマーを設定する
        sleep_time = sleep_time - 70;
        if (sleep_time > 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(sleep_time));
        }
        writeValueAndNotifyChange(customizePropValue, true);
        break;
    case HONDA_CUSTOMIZE_CUSTOM_SET_REQUEST: {
        bool result = hidlIfDataComm_->getCustomizeResult(value.prop);
        customizePropValue = mCustomizeCustomStore.setCustomSettingValue(result, value);
        // 間接通信初期化
        if (result && value.value.byteValues[0] == METER_RESET) {
            resetMeterReadPropVal(value);
        }
        customizePropValue.prop = (int32_t)HONDA_CUSTOMIZE_CUSTOM_SET_RESPONSE;
        customizePropValue.areaId = 0;
        customizePropValue.status = VehiclePropertyStatus::AVAILABLE;
        customizePropValue.timestamp = elapsedRealtimeNano();
        // 処理時間も考慮し、指定時間内に応答を返すようタイマーを設定する
        sleep_time = sleep_time - 70;
        if (sleep_time > 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(sleep_time));
        }
        writeValueAndNotifyChange(customizePropValue, true);
        break;
    }
    default:
        break;
    }
}

void FakeVehicleHardware::resetMeterReadPropVal(const VehiclePropValue& value) {
    VehiclePropValue propValue;
    bool is_reset = true;
    for (int i = 1; i < 8; i++) {
        if (value.value.byteValues[i] != 0) {
            is_reset = false;
            break;
        }
    }
    if (is_reset) {
        // リセット開始
        EMULOG_I("meter reset");
        if (gIndirectVPValueSettings.size() == 0) {
            gIndirectVPValueSettings = readDefaultIndirectCommunicationVPValueSettings();
        }
        for (auto itr = gIndirectVPValueSettings.begin(); itr != gIndirectVPValueSettings.end(); ++itr) {
            propValue = itr->second;
            propValue.status = VehiclePropertyStatus::AVAILABLE;
            propValue.timestamp = elapsedRealtimeNano();
            mServerSidePropStore->writeValue(mValuePool->obtain(propValue), true);
            if (checkNonVolatileVP(value.prop)) {
                updateNonVolatileVP(value.prop, value.areaId);
            }
        }
    }
}

void FakeVehicleHardware::processIndirectCommunicationEvent(const VehiclePropValue& value) {
    VehiclePropValue propValue;
    int64_t sleep_time = 3000 - (elapsedRealtime() - value.timestamp);
    if (hidlIfDataComm_->getIndirectResult() != CommResult::TIMEOUT) {
        sleep_time = sleep_time - 100;
    }
    if (sleep_time > 0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(sleep_time));
    }
    switch (value.prop) {
    case HONDA_CUSTOMIZE_DWS_INIT_REQUEST:
        propValue = value;
        if (hidlIfDataComm_->getIndirectResult() == CommResult::SUCCESS) {
            propValue.value.byteValues[0] = 0x02;
            propValue.prop = getReadProp(value.prop);
            propValue.status = VehiclePropertyStatus::AVAILABLE;
            propValue.timestamp = elapsedRealtimeNano();
            writeValueAndNotifyChange(propValue, true);
        } else {
            // 失敗もしくはタイムアウトの場合は元の値を反映する
            auto internalPropValue = mServerSidePropStore->readValue(getReadProp(value.prop), 0, 0);
            onPropertyValueFromCar(*internalPropValue.value(), true);
        }
        break;
    case HONDA_CUSTOMIZE_TSR_DISP_ONOFF_SET:
    case HONDA_CUSTOMIZE_TSR_WARN_STATUS_SET:
    case HONDA_CUSTOMIZE_TSR_WARN_KPH_SET:
    case HONDA_CUSTOMIZE_TSR_WARN_MPH_SET:
        if (hidlIfDataComm_->getIndirectResult() == CommResult::SUCCESS) {
            // TSR特殊動作
            updateTsrValue(value);
        } else {
            // 失敗もしくはタイムアウトの場合は元の値を反映する
            auto internalPropValue = mServerSidePropStore->readValue(getReadProp(value.prop), 0, 0);
            onPropertyValueFromCar(*internalPropValue.value(), true);
        }
        break;
    case HONDA_CUSTOMIZE_ACC_BUZZER_SET:
    case HONDA_CUSTOMIZE_LKAS_BUZZER_SET:
    case HONDA_CUSTOMIZE_CMBS_ALART_SET:
    case HONDA_CUSTOMIZE_SIF_SET:
    case HONDA_CUSTOMIZE_CDC_INTELLIGENT_LED_CST_STT:
    case HONDA_CUSTOMIZE_DAAS_SET:
    case HONDA_CUSTOMIZE_BSI_SCM_CUST_SET:
    case HONDA_CUSTOMIZE_ALC_CUST_SET:
    case HONDA_CUSTOMIZE_EAS_LC_SET:
    case HONDA_CUSTOMIZE_EAS_PH_SET:
    case HONDA_CUSTOMIZE_FCTW_ATT_SET:
    case HONDA_CUSTOMIZE_ACC_CSA_SET:
    case HONDA_CUSTOMIZE_PKS_RR_SET:
    case HONDA_CUSTOMIZE_IACC_SETUP_SET:
    case HONDA_CUSTOMIZE_REV_MATCH_SET:
    case HONDA_CUSTOMIZE_PEAEMG_SET:
    case HONDA_CUSTOMIZE_TSR_SRBUZ_SET:
    case HONDA_CUSTOMIZE_HSS_ADB_SET:
    case HONDA_EVS_ONOFF_REQ:
    case HONDA_EVS_IDS_COLLABO_REQ:
    case HONDA_EVS_SOUND_TYPE_REQ:
    case HONDA_EVS_VOLUME_REQ:
    case HONDA_EVS_BALANCE_TYPE_ALL_REQ:
    case HONDA_EVS_BALANCE_TYPE_DR_REQ:
    case HONDA_EVS_BALANCE_TYPE_FR_REQ:
    case HONDA_EVS_BALANCE_TYPE_RR_REQ:
    case HONDA_EVS_SOUND_TYPE_PRESET_REQ:
    case HONDA_CUSTOMIZE_APS_AUTO_EPB_SET:
    case HONDA_CUSTOMIZE_APS_DETECT_SOUND_SET:
    case HONDA_CUSTOMIZE_APS_MEMORY_DELE_SET:
    case HONDA_ROFMOD_THEME_COLOR_SET:
    case HONDA_ROFMOD_ROOMLIGHT_SET:
    case HONDA_ROFMOD_BRI_SET:
    case HONDA_LID_DOORLOCK_SW_SET:
    case HONDA_CUSTOMIZE_ELM_RRFOG_CHANGE_MODE_SET:
    case HONDA_CUSTOMIZE_ELM_FRFOG_CHANGE_MODE_SET:
    case HONDA_CUSTOMIZE_DPMS_DR_MASG_MODE_SET:
    case HONDA_CUSTOMIZE_DPMS_AS_MASG_MODE_SET:
    case HONDA_CUSTOMIZE_DPMS_MIRCOUNT1_SET:
    case HONDA_CUSTOMIZE_DPMS_MIR_SELECT_SET:
    case HONDA_CUSTOMIZE_DPMS_MIR_REVERSE_SET:
    case HONDA_ROFMOD_SETTING_FB_SET:
    case HONDA_CUSTOMIZE_GAUGE_PATTERN_SET:
    case HONDA_CUSTOMIZE_WARNING_MSG_SET:
    case HONDA_CUSTOMIZE_EXTEMP_DISP_C_SET:
    case HONDA_CUSTOMIZE_EXTEMP_DISP_F_SET:
    case HONDA_CUSTOMIZE_MTGEAR_DISP_SET:
    case HONDA_CUSTOMIZE_TRIPA_RESET_GAS_SET:
    case HONDA_CUSTOMIZE_TRIPA_RESET_PHEV_SET:
    case HONDA_CUSTOMIZE_TRIPA_RESET_BEV_SET:
    case HONDA_CUSTOMIZE_TRIPA_RESET_FCV_SET:
    case HONDA_CUSTOMIZE_TRIPB_RESET_GAS_SET:
    case HONDA_CUSTOMIZE_TRIPB_RESET_PHEV_SET:
    case HONDA_CUSTOMIZE_TRIPB_RESET_BEV_SET:
    case HONDA_CUSTOMIZE_TRIPB_RESET_FCV_SET:
    case HONDA_CUSTOMIZE_ALARM_VOL_SET:
    case HONDA_CUSTOMIZE_REVERSE_ALARM_SET:
    case HONDA_CUSTOMIZE_REV_INDICATOR_SET:
    case HONDA_CUSTOMIZE_AMBIENT_METER_SET:
    case HONDA_CUSTOMIZE_IDLESTOP_GUIDANCE_SET:
    case HONDA_CUSTOMIZE_TBT_DISPLAY_SET:
    case HONDA_CUSTOMIZE_REARSEAT_REMINDER_SET:
    case HONDA_CUSTOMIZE_DRIVERANGE_NOTIFY_SET:
    case HONDA_CUSTOMIZE_TIREANGLE_MONITOR_SET:
    case HONDA_CUSTOMIZE_DRIVE_UNIT_SET:
    case HONDA_CUSTOMIZE_CONSUMPTION_UNIT_KM_SET:
    case HONDA_CUSTOMIZE_SHIFTUP_TIMING_ALARM_SET:
    case HONDA_CUSTOMIZE_LANGUAGE_EU_SET:
    case HONDA_CUSTOMIZE_LANGUAGE_US_SET:
    case HONDA_CUSTOMIZE_LANGUAGE_CN_SET:
    case HONDA_CUSTOMIZE_LANGUAGE_PT_SET:
    case HONDA_CUSTOMIZE_LANGUAGE_KR_SET:
    case HONDA_CUSTOMIZE_GAUGE_PATTERN_PF3_SET:
    case HONDA_CUSTOMIZE_TBT_DISPLAY_HUD_SET:
    case HONDA_CUSTOMIZE_ADAS_DISPLAY_HUD_SET:
    case HONDA_CUSTOMIZE_SPEEDTSR_DISPLAY_HUD_SET:
    case HONDA_CUSTOMIZE_SPEED_DISPLAY_HUD_SET:
    case HONDA_CUSTOMIZE_DETAIL_DISPLAY_HUD_SET:
    case HONDA_CUSTOMIZE_TRAIL_GAUGE_SET:
    case HONDA_CUSTOMIZE_PS_MET_MID:
    case HONDA_CUSTOMIZE_SET_DID_SETTING:
    case HONDA_CUSTOMIZE_SW_TURN_MODE_SET:
    case HONDA_CUSTOMIZE_EAS_PARK_MODE_SET:
    case HONDA_CUSTOMIZE_EAS_WELCOME_MODE_SET:
    case HONDA_CUSTOMIZE_IDS_RESET2:
    case HONDA_CUSTOMIZE_IDS_ENGINE2_SET:
    case HONDA_CUSTOMIZE_IDS_STEERING2_SET:
    case HONDA_CUSTOMIZE_IDS_SUSPENSION2_SET:
    case HONDA_CUSTOMIZE_IDS_AWD2_SET:
    case HONDA_CUSTOMIZE_PW_SW_LOCK_SET:
    case HONDA_CUSTOMIZE_SUNLOOF_KEYLESS_SET:
    case HONDA_CUSTOMIZE_WIP_MAINTENANCE_MODE_SET:
    case HONDA_CUSTOMIZE_SEAT_DIRECTION_SET:
    case HONDA_CUSTOMIZE_SEATCOLUMN_DIRECTION_SET:
    case HONDA_CUSTOMIZE_IDS_PTSOUND2_SET:
    case HONDA_CUSTOMIZE_ADAS_NOTIF_LED_CST_STT_SET:
    case HONDA_CUSTOMIZE_TLI_RED_FUN_SET:
    case HONDA_CUSTOMIZE_TLI_GREEN_FUN_SET:
    case HONDA_METER_HUD_ONOFF_SET:
    case HONDA_NAVI_DIMMING_GLASS_REQ:
    case HONDA_NAVI_DIMMING_GLASS_REQ_PS:
    case HONDA_CUSTOMIZE_HVAC_LEARN_AUTO_FUNC_SET:
    case HONDA_CUSTOMIZE_MVC_INTERLOCK_SET:
    case HONDA_CUSTOMIZE_REFERLINE_SET:
    case HONDA_CUSTOMIZE_HUD_CONTENTS_SET:
    case HONDA_CUSTOMIZE_DID_CONTENTS_SET:
    case HONDA_CUSTOMIZE_TBT_DISPLAY_DID_SET:
    case HONDA_CUSTOMIZE_AMN_DISPLAY_DID_SET:
    case HONDA_CUSTOMIZE_AUDIO_DISPLAY_DID_SET:
    case HONDA_CUSTOMIZE_HFL_DISPLAY_DID_SET:
    case HONDA_CUSTOMIZE_SR_DISPLAY_DID_SET:
    case HONDA_CUSTOMIZE_SMS_DISPLAY_DID_SET:
    case HONDA_CUSTOMIZE_COMPANION_DISPLAY_DID_SET:
    case HONDA_LID_DR_OPCL_SW_SET:
    case HONDA_LID_AS_OPCL_SW_SET:
    case HONDA_LID_RD_OPCL_SW_SET:
    case HONDA_LID_RA_OPCL_SW_SET:
    case HONDA_CUSTOMIZE_CDC_AD_MAIN_STT_SET:
    case HONDA_CUSTOMIZE_CDC_ACC_CC_MODE_STT_SET:
    case HONDA_CUSTOMIZE_CDC_ACC_LIM_MODE_STT_SET:
    case HONDA_CUSTOMIZE_CDC_LKAS_ENABLE_STT_SET:
    case HONDA_CUSTOMIZE_AHD_CUST_ONOFF_SET:
    case HONDA_CUSTOMIZE_CDC_ACC_DIST_STT_SET:
    case HONDA_CUSTOMIZE_CDC_ACC_SLA_START_SPEED_STT_SET:
    case HONDA_CUSTOMIZE_CDC_IACC_ONOFF_STT_SET:
    case HONDA_CUSTOMIZE_CDC_ACC_SLA_MODE_WO_AUTO_STT_SET:
    case HONDA_CUSTOMIZE_CDC_ACC_SLA_MODE_W_AUTO_STT_SET:
    case HONDA_CUSTOMIZE_CDC_ACC_SLA_OFFSET_KPH_STT_SET:
    case HONDA_CUSTOMIZE_CDC_ACC_SLA_OFFSET_MPH_STT_SET:
    case HONDA_CUSTOMIZE_CDC_ACC_BLINK_AXEL_STT_SET:
    case HONDA_CUSTOMIZE_CDC_ACC_RESUME_ONOFF_STT_SET:
    case HONDA_CUSTOMIZE_CDC_ACC_AXELMODE_STT_SET:
    case HONDA_CUSTOMIZE_CDC_ADA_ONOFF_STT_SET:
    case HONDA_CUSTOMIZE_CDC_ADA_FUNC_STT_SET:
    case HONDA_CUSTOMIZE_CDC_ADA_LONINTENSITY_STT_SET:
    case HONDA_CUSTOMIZE_CDC_ADA_LONCURVE_STT_SET:
    case HONDA_CUSTOMIZE_CDC_ADA_LATINTENSITY_STT_SET:
    case HONDA_CUSTOMIZE_CDC_AILD_HANDS_OFF_ALARM_STT_SET:
    case HONDA_CUSTOMIZE_CDC_ALCA_GAP_SEARCH_CUST_STT_SET:
    case HONDA_CUSTOMIZE_CDC_ALCA_TURN_LEVER_CUST_STT_SET:
    case HONDA_CUSTOMIZE_ALCA_ALC_CUST_SET:
    case HONDA_CUSTOMIZE_CDC_ALCR_ALC_ONOFF_CUST_STT_SET:
    case HONDA_CUSTOMIZE_ALCR_PAS_CUST_RESULT_SET:
    case HONDA_CUSTOMIZE_CDC_ALC_NAVI_CUST_STT_SET:
    case HONDA_CUSTOMIZE_ALCR_NAV_CUST_RESULT_SET:
    case HONDA_CUSTOMIZE_CDC_AMN_ONOFF_STT_SET:
    case HONDA_CUSTOMIZE_CDC_AMN_NAV_TIMING_STT_SET:
    case HONDA_CUSTOMIZE_CDC_AMN_HOV_ENABLE_STT_SET:
    case HONDA_CUSTOMIZE_CDC_AMN_EXPRESS_ENABLE_STT_SET:
    case HONDA_CUSTOMIZE_CDC_AMN_INTERRUPT_DISP_STT_SET:
    case HONDA_CUSTOMIZE_ADAS_NOTIF_VIB_CST_STT_SET:
    case HONDA_CUSTOM_MENU_BIOENT_CONTROLLER:
    case HONDA_CUSTOMIZE_WELCOME_SOUND_SET:
    case HONDA_CUSTOM_ADAS_E_BSI_ONOFF_SET:
    case HONDA_CUSTOM_ADAS_E_BSI_RANGE_SET:
    case HONDA_CUSTOM_ADAS_E_BSI_2R_SET:
    case HONDA_CUSTOMIZE_C_LID_AUTO_SET:
    case HONDA_CUSTOMIZE_EC_LID_CLOSE_TIMER_SET:
    case HONDA_CUSTOMIZE_EC_LID_AUTO_CLOSE_SET:
    case HONDA_CUSTOM_ADAS_STATUS_CMBS_SET:
    case HONDA_CUSTOM_ADAS_FCTW_SET:
    case HONDA_CUSTOM_DAM_ALLOFF_SW_SET:
    case HONDA_CUSTOM_ADAS_AILD_ENABLE_SET:
    case HONDA_CUSTOM_ADAS_ALC_ONOFF_CUST_SET:
    case HONDA_CUSTOM_ADAS_E_BSI_CUST_SET:
    case HONDA_CUSTOM_ADAS_SAFETY_SW_STATUS_PKS_SET:
    case HONDA_CUSTOM_ADAS_SAFETY_SW_STATUS_LSAEB_SET:
    case HONDA_CUSTOM_ADAS_PKS_RR_STATUS_SET:
    case HONDA_CORE_CUSTOM_SNP_CRP_SET:
    case HONDA_CUSTOMIZE_RDMS_ONE_SET:
    case HONDA_CUSTOMIZE_RDMS_TWO_SET:
    case HONDA_CUSTOMIZE_EW_CUST_RESULT_SET:
    case HONDA_CUSTOMIZE_DMC_DROWSY_SETTING_SET:
    case HONDA_CUSTOMIZE_DMC_DIST_SETTING_SET:
    case HONDA_CUSTOMIZE_CDC_AES_ONOFF_STT_SET:
    case HONDA_CUSTOMIZE_CDC_DESS_CUST_STT_SET:
    case HONDA_CUSTOMIZE_CDC_EW_FUNC_ONOFF_STT_SET:
    case HONDA_CUSTOMIZE_CDC_EW_LV1_ONOFF_STT_SET:
    case HONDA_CUSTOMIZE_CDC_EW_ELATCH_ONOFF_STT_SET:
    case HONDA_CUSTOMIZE_CDC_EW_ELATCH_SENSE_STT_SET:
    case HONDA_CUSTOMIZE_BSINFO_VCHANGE_BCF_SET:
    case HONDA_CUSTOMIZE_ALARM_TYPE_SET:
    case HONDA_CUSTOMIZE_CDC_BSI_BUZZER_VOLUME_STT_SET:
    case HONDA_CUSTOMIZE_CDC_BSI_RANGE_STT_SET:
    case HONDA_CUSTOMIZE_CDC_BSI_2R_STT_SET:
    case HONDA_CUSTOMIZE_CDC_BSI_TRAILER_STT_SET:
    case HONDA_CUSTOMIZE_CDC_BSI_TRAILER_TYPE_STT_SET:
    case HONDA_CUSTOMIZE_CDC_BSI_TRAILER_WIDTH_STT_SET:
    case HONDA_CUSTOMIZE_CDC_BSI_TRAILER_LENGTH_STT_SET:
    case HONDA_CUSTOMIZE_CDC_PCDW_ONOFF_STT_SET:
    case HONDA_CUSTOMIZE_ODSF_ONOFF_SET:
    case HONDA_CUSTOMIZE_METER_TSR_SRBUZ_ONOFF_SET:
    case HONDA_CUSTOMIZE_DAM_EYECLOSED_STT_SET:
    case HONDA_METER_HUD_ILLUMI_SET:
    case HONDA_CUSTOMIZE_VOL_DISPLAY_DID_SET:
    case HONDA_CUSTOMIZE_HEADLIGHT_TIMER_SET:
    case HONDA_CUSTOMIZE_WELCOM_LIGHT_1_SET:
    case HONDA_CUSTOMIZE_WELCOM_LIGHT_2_SET:
    case HONDA_CUSTOMIZE_WELCOM_LIGHT_1_2_SET:
    case HONDA_EVS_BALANCE_STAT_SET:
        // 通常の動作 & HDS/ADBの特殊動作
        if (hidlIfDataComm_->getIndirectResult() == CommResult::SUCCESS) {
            // 成功の場合は今回値を反映する
            propValue = value;
            propValue.prop = getReadProp(value.prop);
            propValue.status = VehiclePropertyStatus::AVAILABLE;
            propValue.timestamp = elapsedRealtimeNano();
            writeValueAndNotifyChange(propValue, true);
        } else {
            // 失敗もしくはタイムアウトの場合は元の値を反映する
            auto internalPropValue = mServerSidePropStore->readValue(getReadProp(value.prop), 0, 0);
            onPropertyValueFromCar(*internalPropValue.value(), true);
        }
        break;
    case HONDA_LID_OPCL_SW_SET: {
        std::vector<int> vehiclePropertys = { HONDA_LID_DR_OPCL_SW, HONDA_LID_AS_OPCL_SW, HONDA_LID_RD_OPCL_SW, HONDA_LID_RA_OPCL_SW };
        if (hidlIfDataComm_->getIndirectResult() == CommResult::SUCCESS && value.value.int32Values.size() == vehiclePropertys.size()) {
            for (size_t i = 0; i < value.value.int32Values.size(); i++) {
                auto idsPropValue = mServerSidePropStore->readValue(vehiclePropertys[i], 0, 0);
                idsPropValue.value()->value.int32Values[0] = value.value.int32Values[i];
                idsPropValue.value()->status = VehiclePropertyStatus::AVAILABLE;
                idsPropValue.value()->timestamp = elapsedRealtimeNano();
                writeValueAndNotifyChange(*idsPropValue.value(), true);
            }
        } else {
            for (size_t i = 0; i < vehiclePropertys.size(); i++) {
                auto propValue = mServerSidePropStore->readValue(vehiclePropertys[i], 0, 0);
                onPropertyValueFromCar(*propValue.value(), true);
            }
        }
        break;
    }
    case HONDA_ENTRY_NAVI_CHILDCHECK_REQ:
        if (hidlIfDataComm_->getIndirectResult() == CommResult::SUCCESS) {
            auto idsPropValue = mServerSidePropStore->readValue(getReadProp(value.prop), 0, 0);
            idsPropValue.value()->value.int32Values[0] = value.value.byteValues[0];
            idsPropValue.value()->value.int32Values[1] = 1;
            idsPropValue.value()->status = VehiclePropertyStatus::AVAILABLE;
            idsPropValue.value()->timestamp = elapsedRealtimeNano();
            writeValueAndNotifyChange(*idsPropValue.value(), true);
        } else {
            auto internalPropValue = mServerSidePropStore->readValue(getReadProp(value.prop), 0, 0);
            onPropertyValueFromCar(*internalPropValue.value(), true);
        }
        break;
    case HONDA_METER_LANGUAGE_SEND_REQUEST:
        if (hidlIfDataComm_->getIndirectResult() == CommResult::SUCCESS) {
            // 成功の場合は今回値を反映する
            propValue = value;
            propValue.prop = value.prop;
            propValue.status = VehiclePropertyStatus::AVAILABLE;
            propValue.timestamp = elapsedRealtimeNano();
            writeValueAndNotifyChange(propValue, true);
            auto internalPropValue = mServerSidePropStore->readValue(value.prop, 0, 0);
            auto readPropvalue = mServerSidePropStore->readValue(getReadProp(value.prop), 0, 0);
            if (internalPropValue.value()->value.stringValue == propValue.value.stringValue) {
                readPropvalue.value()->value.int32Values[0] = 1;
            } else {
                readPropvalue.value()->value.int32Values[0] = 0;
            }
            writeValueAndNotifyChange(*readPropvalue.value(), true);
        } else {
            // 失敗もしくはタイムアウトの場合は元の値を反映する
            auto internalPropValue = mServerSidePropStore->readValue(getReadProp(value.prop), 0, 0);
            onPropertyValueFromCar(*internalPropValue.value(), true);
        }
        break;
    case HONDA_CUSTOMIZE_ELM_LIGHT_CHANGE_MODE_SET:
        if (hidlIfDataComm_->getIndirectResult() == CommResult::SUCCESS) {
            // 成功の場合は今回値を反映する
            propValue.value.int32Values.resize(5);
            propValue.prop = getReadProp(value.prop);
            propValue.status = VehiclePropertyStatus::AVAILABLE;
            propValue.timestamp = elapsedRealtimeNano();
            auto internalPropValue = mServerSidePropStore->readValue(getReadProp(value.prop), 0, 0);
            if (internalPropValue.ok()) {
                propValue.value.int32Values[0] = value.value.int32Values[0];
                propValue.value.int32Values[1] = internalPropValue.value()->value.int32Values[1];
                propValue.value.int32Values[2] = internalPropValue.value()->value.int32Values[2];
                propValue.value.int32Values[3] = internalPropValue.value()->value.int32Values[3];
                propValue.value.int32Values[4] = internalPropValue.value()->value.int32Values[4];
            }
            writeValueAndNotifyChange(propValue, true);
        } else {
            // 失敗もしくはタイムアウトの場合は元の値を反映する
            auto internalPropValue = mServerSidePropStore->readValue(getReadProp(value.prop), 0, 0);
            onPropertyValueFromCar(*internalPropValue.value(), true);
        }
        break;
    case HONDA_CUSTOMIZE_IDS_ENGINE_SET:
    case HONDA_CUSTOMIZE_IDS_SUSPENSION_SET:
    case HONDA_CUSTOMIZE_IDS_STEERING_SET:
    case HONDA_CUSTOMIZE_IDS_GAUGE_SET:
    case HONDA_CUSTOMIZE_IDS_IDLESTOP_SET:
    case HONDA_CUSTOMIZE_IDS_REVMATCH_SET:
    case HONDA_CUSTOMIZE_IDS_PTSOUND_SET:
    case HONDA_CUSTOMIZE_IDS_ACC_SET:
    case HONDA_CUSTOMIZE_IDS_LIGHTING_SET: {
        int8_t ids_status = 1;
        if (hidlIfDataComm_->getIndirectResult() == CommResult::SUCCESS) {
            // 成功の場合は今回値を反映する
            propValue = value;
            propValue.prop = getReadProp(value.prop);
            propValue.status = VehiclePropertyStatus::AVAILABLE;
            propValue.timestamp = elapsedRealtimeNano();
            writeValueAndNotifyChange(propValue, true);
            ids_status = 1;
        } else {
            // 失敗もしくはタイムアウトの場合は元の値を反映する
            auto internalPropValue = mServerSidePropStore->readValue(getReadProp(value.prop), 0, 0);
            onPropertyValueFromCar(*internalPropValue.value(), true);
            if (hidlIfDataComm_->getIndirectResult() == CommResult::FAIL) {
                ids_status = 2;
            } else if (hidlIfDataComm_->getIndirectResult() == CommResult::TIMEOUT) {
                ids_status = 0;
            }
        }
        // HONDA_CUSTOMIZE_IDS_SETTING_STATUS通知
        auto idsPropValue = mServerSidePropStore->readValue(HONDA_CUSTOMIZE_IDS_SETTING_STATUS, 0, 0);
        idsPropValue.value()->value.byteValues[0] = ids_status;
        idsPropValue.value()->status = VehiclePropertyStatus::AVAILABLE;
        idsPropValue.value()->timestamp = elapsedRealtimeNano();
        writeValueAndNotifyChange(*idsPropValue.value(), true);
        break;
    }
    case HONDA_CUSTOMIZE_IDS_ALL_SET:
        if (hidlIfDataComm_->getIndirectResult() == CommResult::SUCCESS) {
            // HONDA_CUSTOMIZE_IDS_ENGINE_SET
            propValue.value.byteValues.resize(1);
            propValue.prop = (int32_t)HONDA_CUSTOMIZE_IDS_ENGINE;
            propValue.status = VehiclePropertyStatus::AVAILABLE;
            propValue.timestamp = elapsedRealtimeNano();
            propValue.value.byteValues[0] = value.value.byteValues[0];
            writeValueAndNotifyChange(propValue, true);
            // HONDA_CUSTOMIZE_IDS_SUSPENSION_SET
            propValue.prop = (int32_t)HONDA_CUSTOMIZE_IDS_SUSPENSION;
            propValue.value.byteValues[0] = value.value.byteValues[1];
            writeValueAndNotifyChange(propValue, true);
            // HONDA_CUSTOMIZE_IDS_STEERING_SET
            propValue.prop = (int32_t)HONDA_CUSTOMIZE_IDS_STEERING;
            propValue.value.byteValues[0] = value.value.byteValues[2];
            writeValueAndNotifyChange(propValue, true);
            // HONDA_CUSTOMIZE_IDS_GAUGE_SET
            propValue.prop = (int32_t)HONDA_CUSTOMIZE_IDS_GAUGE;
            propValue.value.byteValues[0] = value.value.byteValues[3];
            writeValueAndNotifyChange(propValue, true);
            // HONDA_CUSTOMIZE_IDS_IDLESTOP_SET
            propValue.prop = (int32_t)HONDA_CUSTOMIZE_IDS_IDLESTOP;
            propValue.value.byteValues[0] = value.value.byteValues[4];
            writeValueAndNotifyChange(propValue, true);
            // HONDA_CUSTOMIZE_IDS_REVMATCH_SET
            propValue.prop = (int32_t)HONDA_CUSTOMIZE_IDS_REVMATCH;
            propValue.value.byteValues[0] = value.value.byteValues[5];
            writeValueAndNotifyChange(propValue, true);
            // HONDA_CUSTOMIZE_IDS_PTSOUND_SET
            propValue.prop = (int32_t)HONDA_CUSTOMIZE_IDS_PTSOUND;
            propValue.value.byteValues[0] = value.value.byteValues[6];
            writeValueAndNotifyChange(propValue, true);
            // HONDA_CUSTOMIZE_IDS_ACC_SET
            propValue.prop = (int32_t)HONDA_CUSTOMIZE_IDS_ACC;
            propValue.value.byteValues[0] = value.value.byteValues[7];
            writeValueAndNotifyChange(propValue, true);
            // HONDA_CUSTOMIZE_IDS_LIGHTING_SET
            propValue.prop = (int32_t)HONDA_CUSTOMIZE_IDS_LIGHTING;
            propValue.value.byteValues[0] = value.value.byteValues[8];
            writeValueAndNotifyChange(propValue, true);
        } else {
            // 失敗・タイムアウト時は前回値を返す
            // HONDA_CUSTOMIZE_IDS_ENGINE_SET
            auto propValue = mServerSidePropStore->readValue(HONDA_CUSTOMIZE_IDS_ENGINE, 0, 0);
            onPropertyValueFromCar(*propValue.value(), true);
            // HONDA_CUSTOMIZE_IDS_SUSPENSION_SET
            propValue = mServerSidePropStore->readValue(HONDA_CUSTOMIZE_IDS_SUSPENSION, 0, 0);
            onPropertyValueFromCar(*propValue.value(), true);
            // HONDA_CUSTOMIZE_IDS_STEERING_SET
            propValue = mServerSidePropStore->readValue(HONDA_CUSTOMIZE_IDS_STEERING, 0, 0);
            onPropertyValueFromCar(*propValue.value(), true);
            // HONDA_CUSTOMIZE_IDS_GAUGE_SET
            propValue = mServerSidePropStore->readValue(HONDA_CUSTOMIZE_IDS_GAUGE, 0, 0);
            onPropertyValueFromCar(*propValue.value(), true);
            // HONDA_CUSTOMIZE_IDS_IDLESTOP_SET
            propValue = mServerSidePropStore->readValue(HONDA_CUSTOMIZE_IDS_IDLESTOP, 0, 0);
            onPropertyValueFromCar(*propValue.value(), true);
            // HONDA_CUSTOMIZE_IDS_REVMATCH_SET
            propValue = mServerSidePropStore->readValue(HONDA_CUSTOMIZE_IDS_REVMATCH, 0, 0);
            onPropertyValueFromCar(*propValue.value(), true);
            // HONDA_CUSTOMIZE_IDS_PTSOUND_SET
            propValue = mServerSidePropStore->readValue(HONDA_CUSTOMIZE_IDS_PTSOUND, 0, 0);
            onPropertyValueFromCar(*propValue.value(), true);
            // HONDA_CUSTOMIZE_IDS_ACC_SET
            propValue = mServerSidePropStore->readValue(HONDA_CUSTOMIZE_IDS_ACC, 0, 0);
            onPropertyValueFromCar(*propValue.value(), true);
            // HONDA_CUSTOMIZE_IDS_LIGHTING_SET
            propValue = mServerSidePropStore->readValue(HONDA_CUSTOMIZE_IDS_LIGHTING, 0, 0);
            onPropertyValueFromCar(*propValue.value(), true);
        }
        break;
    case HONDA_CUSTOMIZE_RDMS_SET:
        // RDMSの特殊動作
        if (hidlIfDataComm_->getIndirectResult() == CommResult::SUCCESS) {
            propValue.value.byteValues.resize(4);
            for (int i = 0; i < 4; i++) {
                propValue.value.byteValues[i] = 0;
            }
            propValue.value.byteValues[hidlIfDataComm_->getRdmsAvailVal()] = value.value.byteValues[0];
            propValue.prop = getReadProp(value.prop);
            propValue.status = VehiclePropertyStatus::AVAILABLE;
            propValue.timestamp = elapsedRealtimeNano();
            writeValueAndNotifyChange(propValue, true);
        } else {
            auto internalPropValue = mServerSidePropStore->readValue(getReadProp(value.prop), 0, 0);
            onPropertyValueFromCar(*internalPropValue.value(), true);
        }
        break;
    default:
        break;
    }
}

int32_t FakeVehicleHardware::getReadProp(int32_t write_prop) {
    int32_t read_prop = 0;
    switch (write_prop) {
    case HONDA_CUSTOMIZE_ACC_BUZZER_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_ACC_BUZZER;
        break;
    case HONDA_CUSTOMIZE_DWS_INIT_REQUEST:
        read_prop = (int32_t)HONDA_CUSTOMIZE_DWS_INIT_REQUEST_RECEIVED;
        break;
    case HONDA_CUSTOMIZE_LKAS_BUZZER_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_LKAS_BUZZER;
        break;
    case HONDA_CUSTOMIZE_CMBS_ALART_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_CMBS_ALART;
        break;
    case HONDA_CUSTOMIZE_SIF_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_SIF;
        break;
    case HONDA_CUSTOMIZE_CDC_INTELLIGENT_LED_CST_STT:
        read_prop = (int32_t)HONDA_CUSTOMIZE_INTELLIGENT_LED_CST_STT;
        break;
    case HONDA_CUSTOMIZE_DAAS_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_DAAS;
        break;
    case HONDA_CUSTOMIZE_BSI_SCM_CUST_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_BSI_SCM_CUST;
        break;
    case HONDA_CUSTOMIZE_ALC_CUST_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_ALC_CUST;
        break;
    case HONDA_CUSTOMIZE_TSR_DISP_ONOFF_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_TSR_DISP_ONOFF;
        break;
    case HONDA_CUSTOMIZE_TSR_WARN_STATUS_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_TSR_WARN_STATUS;
        break;
    case HONDA_CUSTOMIZE_TSR_WARN_KPH_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_TSR_WARN_KPH;
        break;
    case HONDA_CUSTOMIZE_TSR_WARN_MPH_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_TSR_WARN_MPH;
        break;
    case HONDA_CUSTOMIZE_EAS_LC_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_EAS_LC;
        break;
    case HONDA_CUSTOMIZE_EAS_PH_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_EAS_PH;
        break;
    case HONDA_CUSTOMIZE_FCTW_ATT_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_FCTW_ATT;
        break;
    case HONDA_CUSTOMIZE_ACC_CSA_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_ACC_CSA;
        break;
    case HONDA_CUSTOMIZE_PKS_RR_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_PKS_RR;
        break;
    case HONDA_CUSTOMIZE_IACC_SETUP_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_IACC_SETUP;
        break;
    case HONDA_CUSTOMIZE_REV_MATCH_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_REV_MATCH;
        break;
    case HONDA_CUSTOMIZE_PEAEMG_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_PEAEMG;
        break;
    case HONDA_CUSTOMIZE_TSR_SRBUZ_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_TSR_SRBUZ;
        break;
    case HONDA_CUSTOMIZE_IDS_ENGINE_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_IDS_ENGINE;
        break;
    case HONDA_CUSTOMIZE_IDS_SUSPENSION_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_IDS_SUSPENSION;
        break;
    case HONDA_CUSTOMIZE_IDS_STEERING_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_IDS_STEERING;
        break;
    case HONDA_CUSTOMIZE_IDS_GAUGE_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_IDS_GAUGE;
        break;
    case HONDA_CUSTOMIZE_IDS_IDLESTOP_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_IDS_IDLESTOP;
        break;
    case HONDA_CUSTOMIZE_IDS_REVMATCH_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_IDS_REVMATCH;
        break;
    case HONDA_CUSTOMIZE_IDS_PTSOUND_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_IDS_PTSOUND;
        break;
    case HONDA_CUSTOMIZE_IDS_ACC_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_IDS_ACC;
        break;
    case HONDA_CUSTOMIZE_IDS_LIGHTING_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_IDS_LIGHTING;
        break;
    case HONDA_CUSTOMIZE_RDMS_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_RDMS;
        break;
    case HONDA_CUSTOMIZE_HSS_ADB_SET:
        read_prop = hidlIfDataComm_->getHssAdbAvailVal();
        break;
    case HONDA_EVS_ONOFF_REQ:
        read_prop = (int32_t)HONDA_EVS_ONOFF_STATUS;
        break;
    case HONDA_EVS_IDS_COLLABO_REQ:
        read_prop = (int32_t)HONDA_EVS_IDS_COLLABO_STATUS;
        break;
    case HONDA_EVS_SOUND_TYPE_REQ:
        read_prop = (int32_t)HONDA_EVS_SOUND_TYPE_STATUS;
        break;
    case HONDA_EVS_VOLUME_REQ:
        read_prop = (int32_t)HONDA_EVS_VOLUME_STATUS;
        break;
    case HONDA_EVS_BALANCE_TYPE_ALL_REQ:
        read_prop = (int32_t)HONDA_EVS_BALANCE_TYPE_ALL_STATUS;
        break;
    case HONDA_EVS_BALANCE_TYPE_DR_REQ:
        read_prop = (int32_t)HONDA_EVS_BALANCE_TYPE_DR_STATUS;
        break;
    case HONDA_EVS_BALANCE_TYPE_FR_REQ:
        read_prop = (int32_t)HONDA_EVS_BALANCE_TYPE_FR_STATUS;
        break;
    case HONDA_EVS_BALANCE_TYPE_RR_REQ:
        read_prop = (int32_t)HONDA_EVS_BALANCE_TYPE_RR_STATUS;
        break;
    case HONDA_EVS_SOUND_TYPE_PRESET_REQ:
        read_prop = (int32_t)HONDA_EVS_SOUND_TYPE_PRESET_STATUS;
        break;
    case HONDA_CUSTOMIZE_APS_AUTO_EPB_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_APS_AUTO_EPB;
        break;
    case HONDA_CUSTOMIZE_APS_DETECT_SOUND_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_APS_DETECT_SOUND;
        break;
    case HONDA_CUSTOMIZE_APS_MEMORY_DELE_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_APS_MEMORY_DELE;
        break;
    case HONDA_ROFMOD_THEME_COLOR_SET:
        read_prop = (int32_t)HONDA_ROFMOD_THEME_COLOR;
        break;
    case HONDA_ROFMOD_ROOMLIGHT_SET:
        read_prop = (int32_t)HONDA_ROFMOD_ROOMLIGHT;
        break;
    case HONDA_ROFMOD_BRI_SET:
        read_prop = (int32_t)HONDA_ROFMOD_BRI;
        break;
    case HONDA_LID_DOORLOCK_SW_SET:
        read_prop = (int32_t)HONDA_LID_DOORLOCK_SW;
        break;
    case HONDA_CUSTOMIZE_ELM_RRFOG_CHANGE_MODE_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_ELM_RRFOG_CHANGE_MODE;
        break;
    case HONDA_CUSTOMIZE_ELM_LIGHT_CHANGE_MODE_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_ELM_LIGHT_CHANGE_MODE;
        break;
    case HONDA_CUSTOMIZE_ELM_FRFOG_CHANGE_MODE_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_ELM_FRFOG_CHANGE_MODE;
        break;
    case HONDA_CUSTOMIZE_DPMS_DR_MASG_MODE_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_DPMS_DR_MASG_MODE;
        break;
    case HONDA_CUSTOMIZE_DPMS_AS_MASG_MODE_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_DPMS_AS_MASG_MODE;
        break;
    case HONDA_CUSTOMIZE_DPMS_MIRCOUNT1_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_DPMS_MIRCOUNT1;
        break;
    case HONDA_CUSTOMIZE_DPMS_MIR_SELECT_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_DPMS_MIR_SELECT;
        break;
    case HONDA_CUSTOMIZE_DPMS_MIR_REVERSE_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_DPMS_MIR_REVERSE;
        break;
    case HONDA_ROFMOD_SETTING_FB_SET:
        read_prop = (int32_t)HONDA_ROFMOD_SETTING_FB;
        break;
    case HONDA_METER_LANGUAGE_SEND_REQUEST:
        read_prop = (int32_t)HONDA_METER_LANGUAGE_RECEIVE_RESPONSE;
        break;
    case HONDA_CUSTOMIZE_GAUGE_PATTERN_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_GAUGE_PATTERN;
        break;
    case HONDA_CUSTOMIZE_WARNING_MSG_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_WARNING_MSG;
        break;
    case HONDA_CUSTOMIZE_EXTEMP_DISP_C_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_EXTEMP_DISP_C;
        break;
    case HONDA_CUSTOMIZE_EXTEMP_DISP_F_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_EXTEMP_DISP_F;
        break;
    case HONDA_CUSTOMIZE_MTGEAR_DISP_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_MTGEAR_DISP;
        break;
    case HONDA_CUSTOMIZE_TRIPA_RESET_GAS_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_TRIPA_RESET_GAS;
        break;
    case HONDA_CUSTOMIZE_TRIPA_RESET_PHEV_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_TRIPA_RESET_PHEV;
        break;
    case HONDA_CUSTOMIZE_TRIPA_RESET_BEV_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_TRIPA_RESET_BEV;
        break;
    case HONDA_CUSTOMIZE_TRIPA_RESET_FCV_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_TRIPA_RESET_FCV;
        break;
    case HONDA_CUSTOMIZE_TRIPB_RESET_GAS_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_TRIPB_RESET_GAS;
        break;
    case HONDA_CUSTOMIZE_TRIPB_RESET_PHEV_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_TRIPB_RESET_PHEV;
        break;
    case HONDA_CUSTOMIZE_TRIPB_RESET_BEV_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_TRIPB_RESET_BEV;
        break;
    case HONDA_CUSTOMIZE_TRIPB_RESET_FCV_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_TRIPB_RESET_FCV;
        break;
    case HONDA_CUSTOMIZE_ALARM_VOL_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_ALARM_VOL;
        break;
    case HONDA_CUSTOMIZE_REVERSE_ALARM_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_REVERSE_ALARM;
        break;
    case HONDA_CUSTOMIZE_REV_INDICATOR_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_REV_INDICATOR;
        break;
    case HONDA_CUSTOMIZE_AMBIENT_METER_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_AMBIENT_METER;
        break;
    case HONDA_CUSTOMIZE_IDLESTOP_GUIDANCE_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_IDLESTOP_GUIDANCE;
        break;
    case HONDA_CUSTOMIZE_TBT_DISPLAY_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_TBT_DISPLAY;
        break;
    case HONDA_CUSTOMIZE_REARSEAT_REMINDER_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_REARSEAT_REMINDER;
        break;
    case HONDA_CUSTOMIZE_DRIVERANGE_NOTIFY_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_DRIVERANGE_NOTIFY;
        break;
    case HONDA_CUSTOMIZE_TIREANGLE_MONITOR_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_TIREANGLE_MONITOR;
        break;
    case HONDA_CUSTOMIZE_DRIVE_UNIT_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_DRIVE_UNIT;
        break;
    case HONDA_CUSTOMIZE_CONSUMPTION_UNIT_KM_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_CONSUMPTION_UNIT_KM;
        break;
    case HONDA_CUSTOMIZE_SHIFTUP_TIMING_ALARM_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_SHIFTUP_TIMING_ALARM;
        break;
    case HONDA_CUSTOMIZE_LANGUAGE_EU_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_LANGUAGE_EU;
        break;
    case HONDA_CUSTOMIZE_LANGUAGE_US_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_LANGUAGE_US;
        break;
    case HONDA_CUSTOMIZE_LANGUAGE_CN_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_LANGUAGE_CN;
        break;
    case HONDA_CUSTOMIZE_LANGUAGE_PT_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_LANGUAGE_PT;
        break;
    case HONDA_CUSTOMIZE_LANGUAGE_KR_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_LANGUAGE_KR;
        break;
    case HONDA_CUSTOMIZE_GAUGE_PATTERN_PF3_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_GAUGE_PATTERN_PF3;
        break;
    case HONDA_CUSTOMIZE_TBT_DISPLAY_HUD_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_TBT_DISPLAY_HUD;
        break;
    case HONDA_CUSTOMIZE_ADAS_DISPLAY_HUD_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_ADAS_DISPLAY_HUD;
        break;
    case HONDA_CUSTOMIZE_SPEEDTSR_DISPLAY_HUD_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_SPEEDTSR_DISPLAY_HUD;
        break;
    case HONDA_CUSTOMIZE_SPEED_DISPLAY_HUD_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_SPEED_DISPLAY_HUD;
        break;
    case HONDA_CUSTOMIZE_DETAIL_DISPLAY_HUD_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_DETAIL_DISPLAY_HUD;
        break;
    case HONDA_CUSTOMIZE_TRAIL_GAUGE_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_TRAIL_GAUGE;
        break;
    case HONDA_CUSTOMIZE_PS_MET_MID:
        read_prop = (int32_t)HONDA_CUSTOMIZE_MET_PS_MID;
        break;
    case HONDA_CUSTOMIZE_SET_DID_SETTING:
        read_prop = (int32_t)HONDA_CUSTOMIZE_GET_DID_SETTING;
        break;
    case HONDA_CUSTOMIZE_SW_TURN_MODE_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_SW_TURN_MODE;
        break;
    case HONDA_CUSTOMIZE_EAS_PARK_MODE_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_EAS_PARK_MODE;
        break;
    case HONDA_CUSTOMIZE_EAS_WELCOME_MODE_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_EAS_WELCOME_MODE;
        break;
    case HONDA_CUSTOMIZE_IDS_RESET2:
        read_prop = (int32_t)HONDA_CUSTOMIZE_IDS_RESET_STATUS2;
        break;
    case HONDA_CUSTOMIZE_IDS_ENGINE2_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_IDS_ENGINE2;
        break;
    case HONDA_CUSTOMIZE_IDS_STEERING2_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_IDS_STEERING2;
        break;
    case HONDA_CUSTOMIZE_IDS_SUSPENSION2_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_IDS_SUSPENSION2;
        break;
    case HONDA_CUSTOMIZE_IDS_AWD2_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_IDS_AWD2;
        break;
    case HONDA_CUSTOMIZE_PW_SW_LOCK_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_PW_SW_LOCK;
        break;
    case HONDA_CUSTOMIZE_SUNLOOF_KEYLESS_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_SUNLOOF_KEYLESS;
        break;
    case HONDA_CUSTOMIZE_WIP_MAINTENANCE_MODE_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_WIP_MAINTENANCE_MODE;
        break;
    case HONDA_CUSTOMIZE_SEAT_DIRECTION_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_SEAT_DIRECTION;
        break;
    case HONDA_CUSTOMIZE_SEATCOLUMN_DIRECTION_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_SEATCOLUMN_DIRECTION;
        break;
    case HONDA_CUSTOMIZE_IDS_PTSOUND2_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_IDS_PTSOUND2;
        break;
    case HONDA_CUSTOMIZE_ADAS_NOTIF_LED_CST_STT_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_ADAS_NOTIF_LED_CST_STT;
        break;
    case HONDA_CUSTOMIZE_TLI_RED_FUN_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_TLI_RED_FUN;
        break;
    case HONDA_CUSTOMIZE_TLI_GREEN_FUN_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_TLI_GREEN_FUN;
        break;
    case HONDA_METER_HUD_ONOFF_SET:
        read_prop = (int32_t)HONDA_METER_HUD_ONOFF;
        break;
    case HONDA_NAVI_DIMMING_GLASS_REQ_PS:
        read_prop = (int32_t)HONDA_DIMMING_GLASS_STATE;
        break;
    case HONDA_NAVI_DIMMING_GLASS_REQ:
        read_prop = (int32_t)HONDA_DIMMING_GLASS_STATE;
        break;
    case HONDA_CUSTOMIZE_HVAC_LEARN_AUTO_FUNC_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_HVAC_LEARN_AUTO_FUNC;
        break;
    case HONDA_CUSTOMIZE_MVC_INTERLOCK_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_MVC_INTERLOCK;
        break;
    case HONDA_CUSTOMIZE_REFERLINE_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_REFERLINE;
        break;
    case HONDA_CUSTOMIZE_HUD_CONTENTS_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_HUD_CONTENTS;
        break;
    case HONDA_CUSTOMIZE_DID_CONTENTS_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_DID_CONTENTS;
        break;
    case HONDA_CUSTOMIZE_TBT_DISPLAY_DID_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_TBT_DISPLAY_DID;
        break;
    case HONDA_CUSTOMIZE_AMN_DISPLAY_DID_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_AMN_DISPLAY_DID;
        break;
    case HONDA_CUSTOMIZE_AUDIO_DISPLAY_DID_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_AUDIO_DISPLAY_DID;
        break;
    case HONDA_CUSTOMIZE_HFL_DISPLAY_DID_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_HFL_DISPLAY_DID;
        break;
    case HONDA_CUSTOMIZE_SR_DISPLAY_DID_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_SR_DISPLAY_DID;
        break;
    case HONDA_CUSTOMIZE_SMS_DISPLAY_DID_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_SMS_DISPLAY_DID;
        break;
    case HONDA_CUSTOMIZE_COMPANION_DISPLAY_DID_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_COMPANION_DISPLAY_DID;
        break;
    case HONDA_ENTRY_NAVI_CHILDCHECK_REQ:
        read_prop = (int32_t)HONDA_ENTRY_CPD_VEHICLE_CONFIG;
        break;
    case HONDA_LID_DR_OPCL_SW_SET:
        read_prop = (int32_t)HONDA_LID_DR_OPCL_SW;
        break;
    case HONDA_LID_AS_OPCL_SW_SET:
        read_prop = (int32_t)HONDA_LID_AS_OPCL_SW;
        break;
    case HONDA_LID_RD_OPCL_SW_SET:
        read_prop = (int32_t)HONDA_LID_RD_OPCL_SW;
        break;
    case HONDA_LID_RA_OPCL_SW_SET:
        read_prop = (int32_t)HONDA_LID_RA_OPCL_SW;
        break;
    case HONDA_CUSTOMIZE_CDC_AD_MAIN_STT_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_AD_MAIN_STT;
        break;
    case HONDA_CUSTOMIZE_CDC_ACC_CC_MODE_STT_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_ACC_CC_MODE_STT;
        break;
    case HONDA_CUSTOMIZE_CDC_ACC_LIM_MODE_STT_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_ACC_LIM_MODE_STT;
        break;
    case HONDA_CUSTOMIZE_CDC_LKAS_ENABLE_STT_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_LKAS_ENABLE_STT;
        break;
    case HONDA_CUSTOMIZE_AHD_CUST_ONOFF_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_AHD_CUST_ONOFF;
        break;
    case HONDA_CUSTOMIZE_CDC_ACC_DIST_STT_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_ACC_DIST_STT;
        break;
    case HONDA_CUSTOMIZE_CDC_ACC_SLA_START_SPEED_STT_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_ACC_SLA_START_SPEED_STT;
        break;
    case HONDA_CUSTOMIZE_CDC_IACC_ONOFF_STT_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_IACC_ONOFF_STT;
        break;
    case HONDA_CUSTOMIZE_CDC_ACC_SLA_MODE_WO_AUTO_STT_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_ACC_SLA_MODE_WO_AUTO_STT;
        break;
    case HONDA_CUSTOMIZE_CDC_ACC_SLA_MODE_W_AUTO_STT_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_ACC_SLA_MODE_W_AUTO_STT;
        break;
    case HONDA_CUSTOMIZE_CDC_ACC_SLA_OFFSET_KPH_STT_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_ACC_SLA_OFFSET_KPH_STT;
        break;
    case HONDA_CUSTOMIZE_CDC_ACC_SLA_OFFSET_MPH_STT_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_ACC_SLA_OFFSET_MPH_STT;
        break;
    case HONDA_CUSTOMIZE_CDC_ACC_BLINK_AXEL_STT_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_ACC_BLINK_AXEL_STT;
        break;
    case HONDA_CUSTOMIZE_CDC_ACC_RESUME_ONOFF_STT_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_ACC_RESUME_ONOFF_STT;
        break;
    case HONDA_CUSTOMIZE_CDC_ACC_AXELMODE_STT_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_ACC_AXELMODE_STT;
        break;
    case HONDA_CUSTOMIZE_CDC_ADA_ONOFF_STT_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_ADA_ONOFF_STT;
        break;
    case HONDA_CUSTOMIZE_CDC_ADA_FUNC_STT_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_ADA_FUNC_STT;
        break;
    case HONDA_CUSTOMIZE_CDC_ADA_LONINTENSITY_STT_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_ADA_LONINTENSITY_STT;
        break;
    case HONDA_CUSTOMIZE_CDC_ADA_LONCURVE_STT_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_ADA_LONCURVE_STT;
        break;
    case HONDA_CUSTOMIZE_CDC_ADA_LATINTENSITY_STT_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_ADA_LATINTENSITY_STT;
        break;
    case HONDA_CUSTOMIZE_CDC_AILD_HANDS_OFF_ALARM_STT_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_AILD_HANDS_OFF_ALARM_STT;
        break;
    case HONDA_CUSTOMIZE_CDC_ALCA_GAP_SEARCH_CUST_STT_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_ALCA_GAP_SEARCH_CUST_STT;
        break;
    case HONDA_CUSTOMIZE_CDC_ALCA_TURN_LEVER_CUST_STT_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_ALCA_TURN_LEVER_CUST_STT;
        break;
    case HONDA_CUSTOMIZE_ALCA_ALC_CUST_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_ALCA_ALC_CUST;
        break;
    case HONDA_CUSTOMIZE_CDC_ALCR_ALC_ONOFF_CUST_STT_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_ALCR_ALC_ONOFF_CUST_STT;
        break;
    case HONDA_CUSTOMIZE_ALCR_PAS_CUST_RESULT_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_ALCR_PAS_CUST_RESULT;
        break;
    case HONDA_CUSTOMIZE_CDC_ALC_NAVI_CUST_STT_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_ALC_NAVI_CUST_STT;
        break;
    case HONDA_CUSTOMIZE_ALCR_NAV_CUST_RESULT_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_ALCR_NAV_CUST_RESULT;
        break;
    case HONDA_CUSTOMIZE_CDC_AMN_ONOFF_STT_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_AMN_ONOFF_STT;
        break;
    case HONDA_CUSTOMIZE_CDC_AMN_NAV_TIMING_STT_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_AMN_NAV_TIMING_STT;
        break;
    case HONDA_CUSTOMIZE_CDC_AMN_HOV_ENABLE_STT_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_AMN_HOV_ENABLE_STT;
        break;
    case HONDA_CUSTOMIZE_CDC_AMN_EXPRESS_ENABLE_STT_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_AMN_EXPRESS_ENABLE_STT;
        break;
    case HONDA_CUSTOMIZE_CDC_AMN_INTERRUPT_DISP_STT_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_AMN_INTERRUPT_DISP_STT;
        break;
    case HONDA_CUSTOMIZE_ADAS_NOTIF_VIB_CST_STT_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_ADAS_NOTIF_VIB_CST_STT;
        break;
    case HONDA_CUSTOM_MENU_BIOENT_CONTROLLER:
        read_prop = (int32_t)HONDA_BIOENT_CUSTOM_MENU_BIOENT_INTERFACE;
        break;
    case HONDA_CUSTOMIZE_WELCOME_SOUND_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_WELCOME_SOUND;
        break;
    case HONDA_CUSTOM_ADAS_E_BSI_ONOFF_SET:
        read_prop = (int32_t)HONDA_CUSTOM_ADAS_E_BSI_ONOFF;
        break;
    case HONDA_CUSTOM_ADAS_E_BSI_RANGE_SET:
        read_prop = (int32_t)HONDA_CUSTOM_ADAS_E_BSI_RANGE;
        break;
    case HONDA_CUSTOM_ADAS_E_BSI_2R_SET:
        read_prop = (int32_t)HONDA_CUSTOM_ADAS_E_BSI_2R;
        break;
    case HONDA_CUSTOMIZE_C_LID_AUTO_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_C_LID_AUTO;
        break;
    case HONDA_CUSTOMIZE_EC_LID_CLOSE_TIMER_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_EC_LID_CLOSE_TIMER;
        break;
    case HONDA_CUSTOMIZE_EC_LID_AUTO_CLOSE_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_EC_LID_AUTO_CLOSE;
        break;
    case HONDA_CUSTOM_ADAS_STATUS_CMBS_SET:
        read_prop = (int32_t)HONDA_CUSTOM_ADAS_STATUS_CMBS;
        break;
    case HONDA_CUSTOM_ADAS_FCTW_SET:
        read_prop = (int32_t)HONDA_CUSTOM_ADAS_FCTW;
        break;
    case HONDA_CUSTOM_DAM_ALLOFF_SW_SET:
        read_prop = (int32_t)HONDA_CUSTOM_DAM_ALLOFF_SW;
        break;
    case HONDA_CUSTOM_ADAS_AILD_ENABLE_SET:
        read_prop = (int32_t)HONDA_CUSTOM_ADAS_AILD_ENABLE;
        break;
    case HONDA_CUSTOM_ADAS_ALC_ONOFF_CUST_SET:
        read_prop = (int32_t)HONDA_CUSTOM_ADAS_ALC_ONOFF_CUST;
        break;
    case HONDA_CUSTOM_ADAS_E_BSI_CUST_SET:
        read_prop = (int32_t)HONDA_CUSTOM_ADAS_E_BSI_CUST;
        break;
    case HONDA_CUSTOM_ADAS_SAFETY_SW_STATUS_PKS_SET:
        read_prop = (int32_t)HONDA_CUSTOM_ADAS_SAFETY_SW_STATUS_PKS;
        break;
    case HONDA_CUSTOM_ADAS_SAFETY_SW_STATUS_LSAEB_SET:
        read_prop = (int32_t)HONDA_CUSTOM_ADAS_SAFETY_SW_STATUS_LSAEB;
        break;
    case HONDA_CUSTOM_ADAS_PKS_RR_STATUS_SET:
        read_prop = (int32_t)HONDA_CUSTOM_ADAS_PKS_RR_STATUS;
        break;
    case HONDA_CORE_CUSTOM_SNP_CRP_SET:
        read_prop = (int32_t)HONDA_CORE_CUSTOM_SNP_CRP;
        break;
    case HONDA_CUSTOMIZE_RDMS_ONE_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_RDMS_ONE;
        break;
    case HONDA_CUSTOMIZE_RDMS_TWO_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_RDMS_TWO;
        break;
    case HONDA_CUSTOMIZE_EW_CUST_RESULT_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_EW_CUST_RESULT;
        break;
    case HONDA_CUSTOMIZE_DMC_DROWSY_SETTING_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_DMC_DROWSY_SETTING;
        break;
    case HONDA_CUSTOMIZE_DMC_DIST_SETTING_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_DMC_DIST_SETTING;
        break;
    case HONDA_CUSTOMIZE_CDC_AES_ONOFF_STT_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_AES_ONOFF_STT;
        break;
    case HONDA_CUSTOMIZE_CDC_DESS_CUST_STT_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_DESS_CUST_STT;
        break;
    case HONDA_CUSTOMIZE_CDC_EW_FUNC_ONOFF_STT_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_EW_FUNC_ONOFF_STT;
        break;
    case HONDA_CUSTOMIZE_CDC_EW_LV1_ONOFF_STT_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_EW_LV1_ONOFF_STT;
        break;
    case HONDA_CUSTOMIZE_CDC_EW_ELATCH_ONOFF_STT_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_EW_ELATCH_ONOFF_STT;
        break;
    case HONDA_CUSTOMIZE_CDC_EW_ELATCH_SENSE_STT_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_EW_ELATCH_SENSE_STT;
        break;
    case HONDA_CUSTOMIZE_BSINFO_VCHANGE_BCF_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_BSINFO_VCHANGE_BCF;
        break;
    case HONDA_CUSTOMIZE_ALARM_TYPE_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_ALARM_TYPE;
        break;
    case HONDA_CUSTOMIZE_CDC_BSI_BUZZER_VOLUME_STT_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_BSI_BUZZER_VOLUME_STT;
        break;
    case HONDA_CUSTOMIZE_CDC_BSI_RANGE_STT_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_BSI_RANGE_STT;
        break;
    case HONDA_CUSTOMIZE_CDC_BSI_2R_STT_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_BSI_2R_STT;
        break;
    case HONDA_CUSTOMIZE_CDC_BSI_TRAILER_STT_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_BSI_TRAILER_STT;
        break;
    case HONDA_CUSTOMIZE_CDC_BSI_TRAILER_TYPE_STT_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_BSI_TRAILER_TYPE_STT;
        break;
    case HONDA_CUSTOMIZE_CDC_BSI_TRAILER_WIDTH_STT_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_BSI_TRAILER_WIDTH_STT;
        break;
    case HONDA_CUSTOMIZE_CDC_BSI_TRAILER_LENGTH_STT_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_BSI_TRAILER_LENGTH_STT;
        break;
    case HONDA_CUSTOMIZE_CDC_PCDW_ONOFF_STT_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_PCDW_ONOFF_STT;
        break;
    case HONDA_CUSTOMIZE_ODSF_ONOFF_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_ODSF_ONOFF;
        break;
    case HONDA_CUSTOMIZE_METER_TSR_SRBUZ_ONOFF_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_FCM_INFO_TSR_SRBUZ_ONOFF;
        break;
    case HONDA_CUSTOMIZE_DAM_EYECLOSED_STT_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_DAM_EYECLOSED_STT;
        break;
    case HONDA_METER_HUD_ILLUMI_SET:
        read_prop = (int32_t)HONDA_METER_HUD_ILLUMI;
        break;
    case HONDA_CUSTOMIZE_VOL_DISPLAY_DID_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_VOL_DISPLAY_DID;
        break;
    case HONDA_CUSTOMIZE_HEADLIGHT_TIMER_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_HEADLIGHT_TIMER;
        break;
    case HONDA_CUSTOMIZE_WELCOM_LIGHT_1_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_WELCOM_LIGHT_1;
        break;
    case HONDA_CUSTOMIZE_WELCOM_LIGHT_2_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_WELCOM_LIGHT_2;
        break;
    case HONDA_CUSTOMIZE_WELCOM_LIGHT_1_2_SET:
        read_prop = (int32_t)HONDA_CUSTOMIZE_WELCOM_LIGHT_1_2;
    case HONDA_EVS_BALANCE_STAT_SET:
        read_prop = (int32_t)HONDA_EVS_BALANCE_STAT;
        break;
    default:
        break;
    }
    return read_prop;
}

void FakeVehicleHardware::updateTsrValue(const VehiclePropValue& value) {
    VehiclePropValue propValue;
    propValue = value;
    propValue.status = VehiclePropertyStatus::AVAILABLE;
    propValue.timestamp = elapsedRealtimeNano();
    switch (value.prop) {
    case HONDA_CUSTOMIZE_TSR_DISP_ONOFF_SET:
    case HONDA_CUSTOMIZE_TSR_DISP_ONOFF:
        propValue.prop = (int32_t)HONDA_CUSTOMIZE_TSR_DISP_ONOFF;
        writeValueAndNotifyChange(propValue, true);
        if (value.value.byteValues[0] == 0x02) {
            auto subValue = mServerSidePropStore->readValue(HONDA_CUSTOMIZE_TSR_WARN_STATUS, 0, 0);
            subValue.value()->value.byteValues[0] = 0x00;
            writeValueAndNotifyChange(*subValue.value(), true);
            subValue = mServerSidePropStore->readValue(HONDA_CUSTOMIZE_TSR_WARN_KPH, 0, 0);
            subValue.value()->value.byteValues[0] = 0x00;
            writeValueAndNotifyChange(*subValue.value(), true);
            subValue = mServerSidePropStore->readValue(HONDA_CUSTOMIZE_TSR_WARN_MPH, 0, 0);
            subValue.value()->value.byteValues[0] = 0x00;
            writeValueAndNotifyChange(*subValue.value(), true);
        }
        break;
    case HONDA_CUSTOMIZE_TSR_WARN_STATUS_SET:
    case HONDA_CUSTOMIZE_TSR_WARN_STATUS: {
        propValue.prop = (int32_t)HONDA_CUSTOMIZE_TSR_WARN_STATUS;
        auto dispOnOffPropValue = mServerSidePropStore->readValue(HONDA_CUSTOMIZE_TSR_DISP_ONOFF, 0, 0);
        if (dispOnOffPropValue.value()->value.byteValues[0] == 0x02) {
            auto subValue = mServerSidePropStore->readValue(HONDA_CUSTOMIZE_TSR_WARN_STATUS, 0, 0);
            subValue.value()->value.byteValues[0] = 0x00;
            writeValueAndNotifyChange(*subValue.value(), true);
        } else if (value.value.byteValues[0] == 0x02) {
            propValue.prop = (int32_t)HONDA_CUSTOMIZE_TSR_WARN_STATUS;
            writeValueAndNotifyChange(propValue, true);
            auto subValue = mServerSidePropStore->readValue(HONDA_CUSTOMIZE_TSR_WARN_KPH, 0, 0);
            subValue.value()->value.byteValues[0] = 0x00;
            writeValueAndNotifyChange(*subValue.value(), true);
            subValue = mServerSidePropStore->readValue(HONDA_CUSTOMIZE_TSR_WARN_MPH, 0, 0);
            subValue.value()->value.byteValues[0] = 0x00;
            writeValueAndNotifyChange(*subValue.value(), true);
        } else {
            propValue.prop = (int32_t)HONDA_CUSTOMIZE_TSR_WARN_STATUS;
            writeValueAndNotifyChange(propValue, true);
        }
        break;
    }
    case HONDA_CUSTOMIZE_TSR_WARN_KPH_SET:
    case HONDA_CUSTOMIZE_TSR_WARN_KPH: {
        propValue.prop = (int32_t)HONDA_CUSTOMIZE_TSR_WARN_KPH;
        auto dispOnOffPropValue = mServerSidePropStore->readValue(HONDA_CUSTOMIZE_TSR_DISP_ONOFF, 0, 0);
        auto statusPropValue = mServerSidePropStore->readValue(HONDA_CUSTOMIZE_TSR_WARN_STATUS, 0, 0);
        if (dispOnOffPropValue.value()->value.byteValues[0] != 0x02 && statusPropValue.value()->value.byteValues[0] != 0x02) {
            writeValueAndNotifyChange(propValue, true);
        } else {
            auto kpt_val = mServerSidePropStore->readValue(HONDA_CUSTOMIZE_TSR_WARN_KPH, 0, 0);
            kpt_val.value()->value.byteValues[0] = 0x00;
            writeValueAndNotifyChange(*kpt_val.value(), true);
        }
        break;
    }
    case HONDA_CUSTOMIZE_TSR_WARN_MPH_SET:
    case HONDA_CUSTOMIZE_TSR_WARN_MPH: {
        propValue.prop = (int32_t)HONDA_CUSTOMIZE_TSR_WARN_MPH;
        auto dispOnOffPropValue = mServerSidePropStore->readValue(HONDA_CUSTOMIZE_TSR_DISP_ONOFF, 0, 0);
        auto statusPropValue = mServerSidePropStore->readValue(HONDA_CUSTOMIZE_TSR_WARN_STATUS, 0, 0);
        if (dispOnOffPropValue.value()->value.byteValues[0] != 0x02 && statusPropValue.value()->value.byteValues[0] != 0x02) {
            writeValueAndNotifyChange(propValue, true);
        } else {
            auto mpt_val = mServerSidePropStore->readValue(HONDA_CUSTOMIZE_TSR_WARN_MPH, 0, 0);
            mpt_val.value()->value.byteValues[0] = 0x00;
            writeValueAndNotifyChange(*mpt_val.value(), true);
        }
        break;
    }
    default:
        break;
    }
}

void FakeVehicleHardware::processIdsEvent(const VehiclePropValue& value) {
    switch (value.prop) {
    case HONDA_IDS_REQ_ANSBACK: {
        int32_t answer_back = hidlIfDataComm_->getIdsAnswerBackType();
        if (answer_back == AnswerBack::TYPE_30AA) {
            // 30AAタイプ 応答１回目
            auto propValue = value;
            propValue.value.byteValues[2] = 0xFF;
            propValue.status = VehiclePropertyStatus::AVAILABLE;
            propValue.timestamp = elapsedRealtimeNano();
            writeValueAndNotifyChange(propValue, true);
            // 応答２回目
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            if (propValue.value.byteValues[1] != 0xFF) {
                propValue.value.byteValues[1] = 0;
            }
            propValue.timestamp = elapsedRealtimeNano();
            writeValueAndNotifyChange(propValue, true);
        } else if (answer_back == AnswerBack::TYPE_TYAW) {
            // TYAWタイプ 応答１回目
            auto propValue = mServerSidePropStore->readValue(HONDA_IDS_REQ_ANSBACK, 0, 0);
            propValue.value()->value.byteValues[1] = 0;
            propValue.value()->value.byteValues[2] = value.value.byteValues[2];
            propValue.value()->status = VehiclePropertyStatus::AVAILABLE;
            propValue.value()->timestamp = elapsedRealtimeNano();
            writeValueAndNotifyChange(*propValue.value(), true);
            // 応答２回目
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            propValue.value()->value.byteValues[0] = value.value.byteValues[0];
            propValue.value()->timestamp = elapsedRealtimeNano();
            writeValueAndNotifyChange(*propValue.value(), true);
            // 応答３回目
            std::this_thread::sleep_for(std::chrono::milliseconds(150));
            propValue.value()->value.byteValues[2] = 0;
            propValue.value()->timestamp = elapsedRealtimeNano();
            writeValueAndNotifyChange(*propValue.value(), true);
        }
        break;
    }
    case HONDA_CUSTOMIZE_IDS_RESET: {
        VehiclePropValue propValue;
        propValue.prop = (int32_t)HONDA_CUSTOMIZE_IDS_RESET_STATUS;
        propValue.status = VehiclePropertyStatus::AVAILABLE;
        propValue.timestamp = elapsedRealtimeNano();
        propValue.value.byteValues.resize(1);
        int32_t sleep_time = 0;
        int32_t result = hidlIfDataComm_->getIdsResult();
        if (value.value.int32Values[0] == 1) {
            sleep_time = 900;
            if (result == CommResult::SUCCESS) {
                propValue.value.byteValues[0] = 1;
            } else if (result == CommResult::FAIL) {
                propValue.value.byteValues[0] = 2;
            } else {
                propValue.value.byteValues[0] = 0;
                sleep_time = 1000;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(sleep_time));
            writeValueAndNotifyChange(propValue, true);
        } else if (value.value.int32Values[0] == 0) {
            sleep_time = 200;
            if (result != CommResult::TIMEOUT) {
                // タイムアウト以外は応答する
                propValue.value.byteValues[0] = 0;
                propValue.timestamp = elapsedRealtimeNano();
                std::this_thread::sleep_for(std::chrono::milliseconds(sleep_time));
                writeValueAndNotifyChange(propValue, true);
            }
        } else {
            EMULOG_I("Argument Error Wrong Data: %d", value.value.byteValues[0]);
        }
        break;
    }
    case HONDA_CUSTOMIZE_IDS_RESET_STATUS: {
        // ExtからのRead値設定
        EMULOG_I("set Read value : property=0x%x", value.prop);
        auto updatedPropValue = value;
        updatedPropValue.status = VehiclePropertyStatus::AVAILABLE;
        updatedPropValue.timestamp = elapsedRealtimeNano();
        writeValueAndNotifyChange(updatedPropValue, true);
        break;
    }
    default:
        break;
    }
}

void FakeVehicleHardware::setSuspendExitEvent(bool wakeup) {
    LOG(INFO) << __func__ << " SetProperty wakeup:" << std::to_string(wakeup);
    android::base::SetProperty("vendor.hie.suspend.wakeup", std::to_string(wakeup));
}

void FakeVehicleHardware::setHondaPowerTimerEvent(int32_t keyOffTimer, int32_t prepareRunning, int32_t refresh,
                                               int32_t sleep) {
    auto propValue = mValuePool->obtain(VehiclePropertyType::INT32_VEC, 4);
    propValue->prop = toInt(HondaVehicleProperty::HONDA_POWER_TIMER_EVENT);
    propValue->areaId = 0;
    propValue->timestamp = elapsedRealtimeNano();
    propValue->status = VehiclePropertyStatus::AVAILABLE;
    propValue->value.int32Values[0] = keyOffTimer;
    propValue->value.int32Values[1] = prepareRunning;
    propValue->value.int32Values[2] = refresh;
    propValue->value.int32Values[3] = sleep;
    LOG(INFO) << __func__ << " HONDA_POWER_TIMER_EVENT:" << propValue->toString();
    writeValueAndNotifyChange(*propValue, true);
}

void FakeVehicleHardware::hondaPowerTimerExpired() {
    LOG(INFO) << __func__ << " start";
    setHondaPowerTimerEvent(toInt(KeyOffTimer::TIME_OUT), toInt(PrepareRunning::OFF), toInt(Refresh::OFF),
                            toInt(Sleep::OFF));
    int32_t tempPropId = toInt(HondaVehicleProperty::HONDA_ICB_NOTIFICATION);
    auto internalPropValue = mServerSidePropStore->readValue(tempPropId, 0, 0);
    if (internalPropValue.ok()) {
        internalPropValue.value()->value.int32Values[0] = 0x00000001;
        internalPropValue.value()->value.int32Values[1] = toInt(InlineDiagState::COMPLETED);
        internalPropValue.value()->value.int32Values[2] = toInt(AntiTheftState::UNLOCK);
        internalPropValue.value()->value.int32Values[3] = toInt(AccState::ACC_OFF);
        internalPropValue.value()->value.int32Values[5] = toInt(WelcomeState::STATE_7);
        internalPropValue.value()->value.int32Values[6] = toInt(KeyofftimerStartUpState::TIME_OUT);
        internalPropValue.value()->value.int32Values[8] = toInt(ClankingState::OFF);
        internalPropValue.value()->value.int32Values[9] = toInt(HalInitializeState::COMPLETED);
        internalPropValue.value()->timestamp = elapsedRealtimeNano();
        LOG(INFO) << __func__ << " HONDA_ICB_NOTIFICATION:" << internalPropValue.value()->toString();
        writeValueAndNotifyChange(*internalPropValue.value(), true);
    }
}

void FakeVehicleHardware::onHondaPowerTimer() {
    LOG(INFO) << __func__ << " start";
    hondaPowerTimerExpired();
    stopHondaPowerTimer();
}

void FakeVehicleHardware::startHondaPowerTimer() {
    LOG(INFO) << __func__ << " start";
    mPowerCallback = std::make_shared<RecurrentTimer::Callback>(std::bind(&FakeVehicleHardware::onHondaPowerTimer, this));
    mPowerRecurrentTimer->registerTimerCallback(TIMER_HONDA_POWER_STATE_NS, mPowerCallback);
}

void FakeVehicleHardware::stopHondaPowerTimer() {
    LOG(INFO) << __func__ << " start";
    mPowerRecurrentTimer->unregisterTimerCallback(mPowerCallback);
}

void FakeVehicleHardware::monitorGarageModeTimer(int32_t garageModeTime) {
    LOG(INFO) << __func__ << " garageModeTime:" << garageModeTime;
    // The SUSPEND_IMMEDIATELY described in the HAPPI uses the SLEEP_IMMEDIATELY value defined in types.hal.
    // static const int32_t SUSPEND_IMMEDIATELY = toInt(VehicleApPowerStateShutdownParam::SLEEP_IMMEDIATELY);
    std::unique_lock<std::mutex> lock(gMutex);
    std::chrono::steady_clock::time_point time =
        std::chrono::steady_clock::now() + std::chrono::seconds(garageModeTime);
    std::cv_status result = gCond.wait_until(lock, time);
    if (result == std::cv_status::timeout) {
        LOG(INFO) << __func__ << " garage mode timer is timeout";
        writeValueAndNotifyChange(*createApPowerStateReq(VehicleApPowerStateReq::SHUTDOWN_PREPARE),
                                  true /* updateStatus */);
    } else {
        LOG(INFO) << __func__ << " garage mode timer is cancel";
    }
}

void FakeVehicleHardware::stopGarageModeTimer() {
    LOG(INFO) << __func__ << " start";
    std::unique_lock<std::mutex> lock(gMutex);
    gCond.notify_all();
}

bool FakeVehicleHardware::isLinkedProp(int32_t prop) {
    bool isLinkedProp = false;
    switch (prop) {
    case toInt(VehicleProperty::PERF_VEHICLE_SPEED):
    case toInt(VehicleProperty::GEAR_SELECTION):
    case toInt(VehicleProperty::PARKING_BRAKE_ON):
    case toInt(VehicleProperty::IGNITION_STATE):
    case toInt(HondaVehicleProperty::HONDA_VSPNE):
    case toInt(HondaVehicleProperty::HONDA_EAT_TRANS_SPEED):
    case toInt(HondaVehicleProperty::HONDA_AT):
    case toInt(HondaVehicleProperty::HONDA_PARKING_BRAKE):
    case toInt(HondaVehicleProperty::HONDA_IGNITION_STATE):
        LOG(INFO) << __func__ << " prop:" << prop;
        isLinkedProp = true;
        break;
    default:
        break;
    }
    return isLinkedProp;
}

void FakeVehicleHardware::syncLinkedPropValue(VehiclePropValue value) {
    LOG(DEBUG) << __func__ << " start";
    int32_t tempPropId;
    auto internalPropValue = mServerSidePropStore->readValue(0, 0, 0);
    LOG(INFO) << __func__ << " SYNC VALUE:" << value.toString();
    switch (value.prop) {
    case toInt(VehicleProperty::PERF_VEHICLE_SPEED):
        // sync HONDA_VSPNE value.
        tempPropId = toInt(HondaVehicleProperty::HONDA_VSPNE);
        internalPropValue = mServerSidePropStore->readValue(tempPropId, 0, 0);
        if (internalPropValue.ok()) {
            int kmh = std::max(std::min(static_cast<int>(value.value.floatValues[0] * 3.6f), 0xFA), 0x00);    // m/s to km/h, min 0km/h, max 250km/h
            int damh = std::max(std::min(static_cast<int>(value.value.floatValues[0] * 3.6f * 100.0f), 0x0FFFF), 0x00000);  // m/s to decameter per hour, min 0.00km/h, max 655.35km/h
            internalPropValue.value()->value.int32Values[0] = 0x0FA0;  // 4000rpm
            internalPropValue.value()->value.int32Values[1] = kmh;
            internalPropValue.value()->value.int32Values[2] = kmh;
            internalPropValue.value()->value.int32Values[3] = kmh;
            internalPropValue.value()->value.int32Values[4] = damh;
            internalPropValue.value()->timestamp = elapsedRealtimeNano();
            LOG(INFO) << __func__ << " HONDA_VSPNE:" << internalPropValue.value()->toString();
            writeValueAndNotifyChange(*internalPropValue.value(), true);
        }
        // sync HONDA_EAT_TRANS_SPEED value.
        tempPropId = toInt(HondaVehicleProperty::HONDA_EAT_TRANS_SPEED);
        internalPropValue = mServerSidePropStore->readValue(tempPropId, 0, 0);
        if (internalPropValue.ok()) {
            int damh =  std::max(std::min(static_cast<int>(value.value.floatValues[0] * 3.6f * 100.0f), 0xFFFF), 0x0000);  // m/s to decameter per hour, min 0.00km/h, max 655.35km/h
            internalPropValue.value()->value.int32Values[0] = damh;
            internalPropValue.value()->timestamp = elapsedRealtimeNano();
            LOG(INFO) << __func__ << " HONDA_EAT_TRANS_SPEED:" << internalPropValue.value()->toString();
            writeValueAndNotifyChange(*internalPropValue.value(), true);
        }
        break;
    case toInt(HondaVehicleProperty::HONDA_VSPNE):
        // sync PERF_VEHICLE_SPEED value.
        tempPropId = toInt(VehicleProperty::PERF_VEHICLE_SPEED);
        internalPropValue = mServerSidePropStore->readValue(tempPropId, 0, 0);
        if (internalPropValue.ok()) {
            float ms =  std::max(static_cast<float>(value.value.int32Values[1] / 3.6f), 0.0f);  // km/h to m/s, min 0m/s
            internalPropValue.value()->value.floatValues[0] = ms;
            internalPropValue.value()->timestamp = elapsedRealtimeNano();
            LOG(INFO) << __func__ << " PERF_VEHICLE_SPEED:" << internalPropValue.value()->toString();
            writeValueAndNotifyChange(*internalPropValue.value(), true);
        }
        // sync HONDA_EAT_TRANS_SPEED value.
        tempPropId = toInt(HondaVehicleProperty::HONDA_EAT_TRANS_SPEED);
        internalPropValue = mServerSidePropStore->readValue(tempPropId, 0, 0);
        if (internalPropValue.ok()) {
            int damh =  std::max(std::min(value.value.int32Values[1] * 100, 0xFFFF), 0x0000);  // km/h to decameter per hour, min 0.00km/h, max 655.35km/h
            internalPropValue.value()->value.int32Values[0] = damh;
            internalPropValue.value()->timestamp = elapsedRealtimeNano();
            LOG(INFO) << __func__ << " HONDA_EAT_TRANS_SPEED:" << internalPropValue.value()->toString();
            writeValueAndNotifyChange(*internalPropValue.value(), true);
        }
        break;
    case toInt(HondaVehicleProperty::HONDA_EAT_TRANS_SPEED):
        // sync PERF_VEHICLE_SPEED value.
        tempPropId = toInt(VehicleProperty::PERF_VEHICLE_SPEED);
        internalPropValue = mServerSidePropStore->readValue(tempPropId, 0, 0);
        if (internalPropValue.ok()) {
            float ms =  std::max(static_cast<float>(value.value.int32Values[0] / 100.0f / 3.6f), 0.0f);  // decameter per hour to m/s, min 0m/s
            internalPropValue.value()->value.floatValues[0] = ms;
            internalPropValue.value()->timestamp = elapsedRealtimeNano();
            LOG(INFO) << __func__ << " PERF_VEHICLE_SPEED:" << internalPropValue.value()->toString();
            writeValueAndNotifyChange(*internalPropValue.value(), true);
        }
        // sync HONDA_VSPNE value.
        tempPropId = toInt(HondaVehicleProperty::HONDA_VSPNE);
        internalPropValue = mServerSidePropStore->readValue(tempPropId, 0, 0);
        if (internalPropValue.ok()) {
            int kmh =  std::max(std::min(value.value.int32Values[0] / 100, 0xFA), 0x00);  // decameter per hour to km/h, min 0km/h, max 250km/h
            internalPropValue.value()->value.int32Values[0] = 0x0FA0;  // 4000rpm
            internalPropValue.value()->value.int32Values[1] = kmh;
            internalPropValue.value()->value.int32Values[2] = kmh;
            internalPropValue.value()->value.int32Values[3] = kmh;
            internalPropValue.value()->value.int32Values[4] = value.value.int32Values[0];   // decameter per hour
            internalPropValue.value()->timestamp = elapsedRealtimeNano();
            LOG(INFO) << __func__ << " HONDA_VSPNE:" << internalPropValue.value()->toString();
            writeValueAndNotifyChange(*internalPropValue.value(), true);
        }
        break;
    case toInt(HondaVehicleProperty::HONDA_AT):
        // sync PARKING_BRAKE_ON value.
        tempPropId = toInt(VehicleProperty::PARKING_BRAKE_ON);
        internalPropValue = mServerSidePropStore->readValue(tempPropId, 0, 0);
        if (internalPropValue.ok()) {
            if (value.value.int32Values[9] == 0x01 /* parking position ON */) {
                internalPropValue.value()->value.int32Values[0] = 0x01;  // parking position ON
            } else {
                internalPropValue.value()->value.int32Values[0] = 0x00;  // parking position OFF
            }
            internalPropValue.value()->timestamp = elapsedRealtimeNano();
            LOG(INFO) << __func__ << " PARKING_BRAKE_ON:" << internalPropValue.value()->toString();
            writeValueAndNotifyChange(*internalPropValue.value(), true);
        }
        // sync GEAR_SELECTION value.
        tempPropId = toInt(VehicleProperty::GEAR_SELECTION);
        internalPropValue = mServerSidePropStore->readValue(tempPropId, 0, 0);
        if (internalPropValue.ok()) {
            if (value.value.int32Values[0] == 0x01) {
                internalPropValue.value()->value.int32Values[0] = toInt(VehicleGear::GEAR_1);
            } else if (value.value.int32Values[1] == 0x01) {
                internalPropValue.value()->value.int32Values[0] = toInt(VehicleGear::GEAR_2);
            } else if (value.value.int32Values[2] == 0x01) {
                internalPropValue.value()->value.int32Values[0] = toInt(VehicleGear::GEAR_3);
            } else if (value.value.int32Values[3] == 0x01) {
                internalPropValue.value()->value.int32Values[0] = toInt(VehicleGear::GEAR_DRIVE);
            } else if (value.value.int32Values[4] == 0x01) {
                internalPropValue.value()->value.int32Values[0] = toInt(VehicleGear::GEAR_NEUTRAL);
            } else if (value.value.int32Values[5] == 0x01) {
                internalPropValue.value()->value.int32Values[0] = toInt(VehicleGear::GEAR_PARK);
            } else if (value.value.int32Values[6] == 0x01) {
                internalPropValue.value()->value.int32Values[0] = toInt(VehicleGear::GEAR_REVERSE);
            }
            internalPropValue.value()->timestamp = elapsedRealtimeNano();
            LOG(INFO) << __func__ << " GEAR_SELECTION:" << internalPropValue.value()->toString();
            writeValueAndNotifyChange(*internalPropValue.value(), true);
        }
        break;
    case toInt(VehicleProperty::GEAR_SELECTION):
        // sync HONDA_AT value.
        tempPropId = toInt(HondaVehicleProperty::HONDA_AT);
        internalPropValue = mServerSidePropStore->readValue(tempPropId, 0, 0);
        if (internalPropValue.ok()) {
            switch (value.value.int32Values[0]) {
            case toInt(VehicleGear::GEAR_1):
                internalPropValue.value()->value.int32Values[0] = 0x01;  // at position1 ON
                internalPropValue.value()->value.int32Values[1] = 0x00;  // at position2 OFF
                internalPropValue.value()->value.int32Values[2] = 0x00;  // at position3 OFF
                internalPropValue.value()->value.int32Values[3] = 0x00;  // drive position OFF
                internalPropValue.value()->value.int32Values[4] = 0x00;  // neutral position OFF
                internalPropValue.value()->value.int32Values[5] = 0x00;  // parking position OFF
                internalPropValue.value()->value.int32Values[6] = 0x00;  // reverse position OFF
                break;
            case toInt(VehicleGear::GEAR_2):
                internalPropValue.value()->value.int32Values[0] = 0x00;  // at position1 OFF
                internalPropValue.value()->value.int32Values[1] = 0x01;  // at position2 ON
                internalPropValue.value()->value.int32Values[2] = 0x00;  // at position3 OFF
                internalPropValue.value()->value.int32Values[3] = 0x00;  // drive position OFF
                internalPropValue.value()->value.int32Values[4] = 0x00;  // neutral position OFF
                internalPropValue.value()->value.int32Values[5] = 0x00;  // parking position OFF
                internalPropValue.value()->value.int32Values[6] = 0x00;  // reverse position OFF
                break;
            case toInt(VehicleGear::GEAR_3):
                internalPropValue.value()->value.int32Values[0] = 0x00;  // at position1 OFF
                internalPropValue.value()->value.int32Values[1] = 0x00;  // at position2 OFF
                internalPropValue.value()->value.int32Values[2] = 0x01;  // at position3 ON
                internalPropValue.value()->value.int32Values[3] = 0x00;  // drive position OFF
                internalPropValue.value()->value.int32Values[4] = 0x00;  // neutral position OFF
                internalPropValue.value()->value.int32Values[5] = 0x00;  // parking position OFF
                internalPropValue.value()->value.int32Values[6] = 0x00;  // reverse position OFF
                break;
            case toInt(VehicleGear::GEAR_DRIVE):
                internalPropValue.value()->value.int32Values[0] = 0x00;  // at position1 OFF
                internalPropValue.value()->value.int32Values[1] = 0x00;  // at position2 OFF
                internalPropValue.value()->value.int32Values[2] = 0x00;  // at position3 OFF
                internalPropValue.value()->value.int32Values[3] = 0x01;  // drive position ON
                internalPropValue.value()->value.int32Values[4] = 0x00;  // neutral position OFF
                internalPropValue.value()->value.int32Values[5] = 0x00;  // parking position OFF
                internalPropValue.value()->value.int32Values[6] = 0x00;  // reverse position OFF
                break;
            case toInt(VehicleGear::GEAR_NEUTRAL):
                internalPropValue.value()->value.int32Values[0] = 0x00;  // at position1 OFF
                internalPropValue.value()->value.int32Values[1] = 0x00;  // at position2 OFF
                internalPropValue.value()->value.int32Values[2] = 0x00;  // at position3 OFF
                internalPropValue.value()->value.int32Values[3] = 0x00;  // drive position OFF
                internalPropValue.value()->value.int32Values[4] = 0x01;  // neutral position ON
                internalPropValue.value()->value.int32Values[5] = 0x00;  // parking position OFF
                internalPropValue.value()->value.int32Values[6] = 0x00;  // reverse position OFF
                break;
            case toInt(VehicleGear::GEAR_PARK):
                internalPropValue.value()->value.int32Values[0] = 0x00;  // at position1 OFF
                internalPropValue.value()->value.int32Values[1] = 0x00;  // at position2 OFF
                internalPropValue.value()->value.int32Values[2] = 0x00;  // at position3 OFF
                internalPropValue.value()->value.int32Values[3] = 0x00;  // drive position OFF
                internalPropValue.value()->value.int32Values[4] = 0x00;  // neutral position OFF
                internalPropValue.value()->value.int32Values[5] = 0x01;  // parking position ON
                internalPropValue.value()->value.int32Values[6] = 0x00;  // reverse position OFF
                break;
            case toInt(VehicleGear::GEAR_REVERSE):
                internalPropValue.value()->value.int32Values[0] = 0x00;  // at position1 OFF
                internalPropValue.value()->value.int32Values[1] = 0x00;  // at position2 OFF
                internalPropValue.value()->value.int32Values[2] = 0x00;  // at position3 OFF
                internalPropValue.value()->value.int32Values[3] = 0x00;  // drive position OFF
                internalPropValue.value()->value.int32Values[4] = 0x00;  // neutral position OFF
                internalPropValue.value()->value.int32Values[5] = 0x00;  // parking position OFF
                internalPropValue.value()->value.int32Values[6] = 0x01;  // reverse position ON
                break;
            default:
                break;
            }
            internalPropValue.value()->timestamp = elapsedRealtimeNano();
            LOG(INFO) << __func__ << " HONDA_AT:" << internalPropValue.value()->toString();
            writeValueAndNotifyChange(*internalPropValue.value(), true);
        }
        break;
    case toInt(VehicleProperty::PARKING_BRAKE_ON):
        // sync HONDA_PARKING_BRAKE value.
        tempPropId = toInt(HondaVehicleProperty::HONDA_PARKING_BRAKE);
        internalPropValue = mServerSidePropStore->readValue(tempPropId, 0, 0);
        if (internalPropValue.ok()) {
            if (value.value.int32Values[0] == 1 /* parking brake ON */) {
                internalPropValue.value()->value.byteValues = {0, 1, 1};  // parking brake ON
            } else {
                internalPropValue.value()->value.byteValues = {0, 0, 0};  // parking brake OFF
            }
            internalPropValue.value()->timestamp = elapsedRealtimeNano();
            LOG(INFO) << __func__ << " HONDA_PARKING_BRAKE:" << internalPropValue.value()->toString();
            writeValueAndNotifyChange(*internalPropValue.value(), true);
        }
        // sync HONDA_AT value.
        tempPropId = toInt(HondaVehicleProperty::HONDA_AT);
        internalPropValue = mServerSidePropStore->readValue(tempPropId, 0, 0);
        if (internalPropValue.ok()) {
            if (value.value.int32Values[0] == 1 /* parking position ON */) {
                internalPropValue.value()->value.int32Values[9] = 0x01;  // parking position ON
            } else {
                internalPropValue.value()->value.int32Values[9] = 0x00;  // parking position OFF
            }
            internalPropValue.value()->timestamp = elapsedRealtimeNano();
            LOG(INFO) << __func__ << " HONDA_AT:" << internalPropValue.value()->toString();
            writeValueAndNotifyChange(*internalPropValue.value(), true);
        }
        break;
    case toInt(HondaVehicleProperty::HONDA_PARKING_BRAKE):
        // sync PARKING_BRAKE_ON value.
        tempPropId = toInt(VehicleProperty::PARKING_BRAKE_ON);
        internalPropValue = mServerSidePropStore->readValue(tempPropId, 0, 0);
        if (internalPropValue.ok()) {
            if (value.value.byteValues[0] == 0 && value.value.byteValues[1] == 1 &&
                value.value.byteValues[2] == 1 /* parking brake ON */) {
                internalPropValue.value()->value.int32Values[0] = 1;  // parking brake ON
            } else if (value.value.byteValues[0] == 0 && value.value.byteValues[1] == 0 &&
                       value.value.byteValues[2] == 0 /* parking brake OFF */) {
                internalPropValue.value()->value.int32Values[0] = 0;  // parking brake OFF
            }
            internalPropValue.value()->timestamp = elapsedRealtimeNano();
            LOG(INFO) << __func__ << " PARKING_BRAKE_ON:" << internalPropValue.value()->toString();
            writeValueAndNotifyChange(*internalPropValue.value(), true);
        }
        // sync HONDA_AT value.
        tempPropId = toInt(HondaVehicleProperty::HONDA_AT);
        internalPropValue = mServerSidePropStore->readValue(tempPropId, 0, 0);
        if (internalPropValue.ok()) {
            if (value.value.byteValues[0] == 0 && value.value.byteValues[1] == 1 &&
                value.value.byteValues[2] == 1 /* parking brake ON */) {
                internalPropValue.value()->value.int32Values[9] = 0x01;  // parking position ON
            } else if (value.value.byteValues[0] == 0 && value.value.byteValues[1] == 0 &&
                       value.value.byteValues[2] == 0 /* parking brake OFF */) {
                internalPropValue.value()->value.int32Values[9] = 0x00;  // parking position OFF
            }
            internalPropValue.value()->timestamp = elapsedRealtimeNano();
            LOG(INFO) << __func__ << " HONDA_AT:" << internalPropValue.value()->toString();
            writeValueAndNotifyChange(*internalPropValue.value(), true);
        }
        break;
    case toInt(VehicleProperty::IGNITION_STATE):
        // sync HONDA_IGNITION_STATE value.
        tempPropId = toInt(HondaVehicleProperty::HONDA_IGNITION_STATE);
        internalPropValue = mServerSidePropStore->readValue(tempPropId, 0, 0);
        if (internalPropValue.ok()) {
            if (value.value.int32Values[0] == toInt(VehicleIgnitionState::ON) ||
                value.value.int32Values[0] == toInt(VehicleIgnitionState::START)) {
                internalPropValue.value()->value.int32Values[0] = 1;  // ignition state ON
                internalPropValue.value()->timestamp = elapsedRealtimeNano();
                LOG(INFO) << __func__ << " HONDA_IGNITION_STATE:" << internalPropValue.value()->toString();
                writeValueAndNotifyChange(*internalPropValue.value(), true);
            } else {
                internalPropValue.value()->value.int32Values[0] = 0;  // ignition state OFF
                internalPropValue.value()->timestamp = elapsedRealtimeNano();
                LOG(INFO) << __func__ << " HONDA_IGNITION_STATE:" << internalPropValue.value()->toString();
                writeValueAndNotifyChange(*internalPropValue.value(), true);
            }
        }
        // sync HONDA_MICU_BCM value.
        tempPropId = toInt(HondaVehicleProperty::HONDA_MICU_BCM);
        internalPropValue = mServerSidePropStore->readValue(tempPropId, 0, 0);
        if (internalPropValue.ok()) {
            if (value.value.int32Values[0] == toInt(VehicleIgnitionState::ON) ||
                value.value.int32Values[0] == toInt(VehicleIgnitionState::START)) {
                internalPropValue.value()->value.int32Values[0] = 1;  // ACC ON
                internalPropValue.value()->value.int32Values[4] = 1;  // IG1 ON
                internalPropValue.value()->timestamp = elapsedRealtimeNano();
                LOG(INFO) << __func__ << " HONDA_MICU_BCM:" << internalPropValue.value()->toString();
                writeValueAndNotifyChange(*internalPropValue.value(), true);
            } else if (value.value.int32Values[0] == toInt(VehicleIgnitionState::ACC)) {
                internalPropValue.value()->value.int32Values[0] = 1;  // ACC ON
                internalPropValue.value()->value.int32Values[4] = 0;  // IG1 OFF
                internalPropValue.value()->timestamp = elapsedRealtimeNano();
                LOG(INFO) << __func__ << " HONDA_MICU_BCM:" << internalPropValue.value()->toString();
                writeValueAndNotifyChange(*internalPropValue.value(), true);
            } else {
                internalPropValue.value()->value.int32Values[0] = 0;  // ACC OFF
                internalPropValue.value()->value.int32Values[4] = 0;  // IG1 OFF
                internalPropValue.value()->timestamp = elapsedRealtimeNano();
                LOG(INFO) << __func__ << " HONDA_MICU_BCM:" << internalPropValue.value()->toString();
                writeValueAndNotifyChange(*internalPropValue.value(), true);
            }
        }
        break;
    case toInt(HondaVehicleProperty::HONDA_IGNITION_STATE):
        LOG(WARNING) << __func__ << " HONDA_IGNITION_STATE does not support setting value synchronization.";
        break;
    case toInt(HondaVehicleProperty::HONDA_MICU_BCM):
        LOG(WARNING) << __func__ << " HONDA_MICU_BCM does not support setting value synchronization.";
        break;
    default:
        break;
    }
}

bool FakeVehicleHardware::deleteFile(const char *file_name) {
    // Existence of file.
    FILE *file = fopen(file_name, "r");
    if (!file) {
        // If it doesn't exist, do nothing.
        return true;
    }
    fclose(file);

    return !remove(file_name);
}

bool FakeVehicleHardware::writeJsonFile(const char *file_path, const Json::Value jsonRoot) {
    std::string json_str = jsonRoot.toStyledString();
    const char *data = json_str.c_str();
    size_t size = strlen(data);

    // Delete the file if it already exists.
    if (!deleteFile(file_path)) {
        LOG(ERROR) << __func__ << ":json file write NG:file delete failure";
        return false;
    }

    // File open.
    FILE *o_file = fopen(file_path, "w");
    if (o_file == nullptr) {
        LOG(ERROR) << __func__ << ":json file write NG:file open failure";
        return false;
    }

    // Write data.
    if (fwrite(data, sizeof(char), size, o_file) < size) {
        LOG(ERROR) << __func__ << ":json file write NG:file write failure";
        // File close.
        fclose(o_file);
        return false;
    }

    // Write data to storage.
    fflush(o_file);
    fsync(fileno(o_file));

    // Allow access from others.
    chmod(file_path, 0777);

    // File close.
    fclose(o_file);

    LOG(DEBUG) << __func__ << ":Json file write complete";
    return true;
}

bool FakeVehicleHardware::readJsonFile(const char *file_path, Json::Value &jsonRoot) {
    // Read a file from the file path.
    std::ifstream ifs(file_path);
    if (!ifs) {
        LOG(WARNING) << __func__ << ":json file read NG:couldn't open " << file_path << " for parsing.";
        return false;
    }

    Json::CharReaderBuilder builder;
    std::string errorMessage;
    if (!Json::parseFromStream(builder, ifs, &jsonRoot, &errorMessage)) {
        LOG(ERROR) << __func__ << ":json file read NG:Failed to parse JSON file. Error: "
                   << errorMessage.c_str();
        return false;
    }

    LOG(DEBUG) << __func__ << ":Json file read OK";
    return true;
}

bool FakeVehicleHardware::checkNonVolatileVP(const int32_t &propId) {
    LOG(DEBUG) << "Check for NonVolatileVP start";

    for (const int32_t &checkPropId : kNonVolatile) {
        if (propId == checkPropId) {
            return true;
        }
    }
    return false;
}

void FakeVehicleHardware::writeNonVolatileVP() {
    LOG(DEBUG) << "NonVolatileVP json write start";

    // Create the Json data to be written to the file.
    Json::Value jsonRoot(Json::objectValue);

    for (size_t i = 0; i < defaultconfig::getDefaultConfigs().size(); i++) {
        int32_t tempPropId = defaultconfig::getDefaultConfigs()[i].config.prop;

        if (!checkNonVolatileVP(tempPropId)) {
            continue;
        }

        size_t areaConfigSize = defaultconfig::getDefaultConfigs()[i].config.areaConfigs.size();
        for (size_t j = 0; j <= areaConfigSize; j++) {
            int tempAreaId = 0;
            if (areaConfigSize != 0) {
                if (j == areaConfigSize) {
                    continue;
                }
                tempAreaId = defaultconfig::getDefaultConfigs()[i].config.areaConfigs[j].areaId;
            }

            // Value for propId and areaId
            Json::Value jsonValue(Json::objectValue);

            if (vpToJsonValue(tempPropId, tempAreaId, jsonValue)) {
                // Convert propId to string for Json property names.
                std::string propIdStr = std::to_string(tempPropId);
                std::string areaIdStr = std::to_string(tempAreaId);

                jsonRoot[propIdStr][areaIdStr] = jsonValue;
            }
        }
    }
    writeJsonFile(FILE_NAME_NON_VOLATILE, jsonRoot);
}

void FakeVehicleHardware::updateNonVolatileVP(const int32_t &propId, const int32_t &areaId) {
    LOG(DEBUG) << "NonVolatileVP json update start";

    // Load existing Json data file.
    Json::Value jsonRoot(Json::objectValue);

    if (!readJsonFile(FILE_NAME_NON_VOLATILE, jsonRoot)) {
        LOG(WARNING) << __func__ << ":json read : nonvolatile file not found";
        return;
    }

    // Value for propId and areaId
    Json::Value jsonValue(Json::objectValue);

    if (vpToJsonValue(propId, areaId, jsonValue)) {
        // Convert propId to string for Json property names.
        std::string propIdStr = std::to_string(propId);
        std::string areaIdStr = std::to_string(areaId);
        jsonRoot[propIdStr][areaIdStr] = jsonValue;

        writeJsonFile(FILE_NAME_NON_VOLATILE, jsonRoot);
    }
}

bool FakeVehicleHardware::vpToJsonValue(const int32_t &propId, const int32_t &areaId, Json::Value &jsonValue) {
    // Check for nullptr
    auto internalPropValue = mServerSidePropStore->readValue(propId, areaId, 0);
    if (internalPropValue.ok()) {
        LOG(WARNING) << __func__ << ":Property ID : " << propId << " Area ID : " << areaId << " is null";
        return false;
    }

    // Value for VehiclePropertyType::MIXED.
    Json::Value mixedValue(Json::objectValue);

    // Arrays by variable type.
    Json::Value jsonInt32Values(Json::arrayValue);
    Json::Value jsonFloatValues(Json::arrayValue);
    Json::Value jsonInt64Values(Json::arrayValue);
    Json::Value jsonBytes(Json::arrayValue);

    // Registration of setting values by VehiclePropertyType.
    switch (getPropType(propId)) {
    case VehiclePropertyType::BOOLEAN:
    case VehiclePropertyType::INT32:
    case VehiclePropertyType::INT32_VEC:
        for (const int32_t &i32v : internalPropValue.value()->value.int32Values) {
            jsonInt32Values.append(i32v);
        }
        jsonValue["value"] = jsonInt32Values;
        break;
    case VehiclePropertyType::INT64:
    case VehiclePropertyType::INT64_VEC:
        for (const int64_t &i64v : internalPropValue.value()->value.int64Values) {
            jsonInt64Values.append((Json::Int64)i64v);
        }
        jsonValue["value"] = jsonInt64Values;
        break;
    case VehiclePropertyType::FLOAT:
    case VehiclePropertyType::FLOAT_VEC:
        for (const float &fv : internalPropValue.value()->value.floatValues) {
            jsonFloatValues.append(static_cast<double>(fv));
        }
        jsonValue["value"] = jsonFloatValues;
        break;
    case VehiclePropertyType::BYTES:
        for (const uint8_t &fv : internalPropValue.value()->value.byteValues) {
            jsonBytes.append((Json::UInt)fv);
        }
        jsonValue["value"] = jsonBytes;
        break;
    case VehiclePropertyType::STRING:
        jsonValue["value"] = (std::string)internalPropValue.value()->value.stringValue;
        break;
    case VehiclePropertyType::MIXED:
        for (const int32_t &i32v : internalPropValue.value()->value.int32Values) {
            jsonInt32Values.append(i32v);
        }
        for (const int64_t &i64v : internalPropValue.value()->value.int64Values) {
            jsonInt64Values.append((Json::Int64)i64v);
        }
        for (const float &fv : internalPropValue.value()->value.floatValues) {
            jsonFloatValues.append(static_cast<double>(fv));
        }
        for (const uint8_t &byte : internalPropValue.value()->value.byteValues) {
            jsonBytes.append((Json::UInt)byte);
        }
        mixedValue["int32Values"] = jsonInt32Values;
        mixedValue["int64Values"] = jsonInt64Values;
        mixedValue["floatValues"] = jsonFloatValues;
        mixedValue["byteValues"] = jsonBytes;
        mixedValue["stringValue"] = (std::string)internalPropValue.value()->value.stringValue;
        jsonValue["value"] = mixedValue;
        break;
    default:
        LOG(ERROR) << __func__ << ":unsupported type for property";
        return false;
    }

    return true;
}

std::list<VehiclePropValue> FakeVehicleHardware::readNonVolatileVPValueSettings() {
    std::list<VehiclePropValue> nonVolatileVPValueSettings;

    // Read json file.
    Json::Value jsonRoot;

    if (!readJsonFile(FILE_NAME_NON_VOLATILE, jsonRoot)) {
        LOG(WARNING) << __func__ << ":json read : nonvolatile file not found";
        return nonVolatileVPValueSettings;
    }

    // Get all property names.
    Json::Value::Members propIdList = jsonRoot.getMemberNames();
    for (const std::string &propIdStr : propIdList) {
        int propId = atoi(propIdStr.c_str());

        Json::Value::Members areaIdList = jsonRoot[propIdStr].getMemberNames();
        for (const std::string &areaIdStr : areaIdList) {
            int areaId = atoi(areaIdStr.c_str());

            // Get the destination of the VP setting.
            VehiclePropValue nonVolatilePropValue;
            nonVolatilePropValue.prop = propId;
            nonVolatilePropValue.areaId = areaId;

            // Reading of setting values by VehiclePropertyType.
            int valueIndex = 0;
            switch (getPropType(propId)) {
            case VehiclePropertyType::BOOLEAN:
            case VehiclePropertyType::INT32:
            case VehiclePropertyType::INT32_VEC:
                for (const Json::Value &i32v : jsonRoot[propIdStr][areaIdStr]["value"]) {
                    nonVolatilePropValue.value.int32Values.resize(valueIndex + 1);
                    nonVolatilePropValue.value.int32Values[valueIndex] = i32v.asInt();
                    valueIndex++;
                }
                break;
            case VehiclePropertyType::INT64:
            case VehiclePropertyType::INT64_VEC:
                for (const Json::Value &i64v : jsonRoot[propIdStr][areaIdStr]["value"]) {
                    nonVolatilePropValue.value.int64Values.resize(valueIndex + 1);
                    nonVolatilePropValue.value.int64Values[valueIndex] = i64v.asInt64();
                    valueIndex++;
                }
                break;
            case VehiclePropertyType::FLOAT:
            case VehiclePropertyType::FLOAT_VEC:
                for (const Json::Value &fv : jsonRoot[propIdStr][areaIdStr]["value"]) {
                    nonVolatilePropValue.value.floatValues.resize(valueIndex + 1);
                    nonVolatilePropValue.value.floatValues[valueIndex] = fv.asFloat();
                    valueIndex++;
                }
                break;
            case VehiclePropertyType::BYTES:
                for (const Json::Value &byte : jsonRoot[propIdStr][areaIdStr]["value"]) {
                    nonVolatilePropValue.value.byteValues.resize(valueIndex + 1);
                    nonVolatilePropValue.value.byteValues[valueIndex] = (uint8_t)byte.asUInt();
                    valueIndex++;
                }
                break;
            case VehiclePropertyType::STRING:
                nonVolatilePropValue.value.stringValue = jsonRoot[propIdStr][areaIdStr]["value"].asString();
                break;
            case VehiclePropertyType::MIXED:
                valueIndex = 0;
                for (const Json::Value &i32v : jsonRoot[propIdStr][areaIdStr]["value"]["int32Values"]) {
                    nonVolatilePropValue.value.int32Values.resize(valueIndex + 1);
                    nonVolatilePropValue.value.int32Values[valueIndex] = i32v.asInt();
                    valueIndex++;
                }
                valueIndex = 0;
                for (const Json::Value &i64v : jsonRoot[propIdStr][areaIdStr]["value"]["int64Values"]) {
                    nonVolatilePropValue.value.int64Values.resize(valueIndex + 1);
                    nonVolatilePropValue.value.int64Values[valueIndex] = i64v.asInt64();
                    valueIndex++;
                }
                valueIndex = 0;
                for (const Json::Value &fv : jsonRoot[propIdStr][areaIdStr]["value"]["floatValues"]) {
                    nonVolatilePropValue.value.floatValues.resize(valueIndex + 1);
                    nonVolatilePropValue.value.floatValues[valueIndex] = fv.asFloat();
                    valueIndex++;
                }
                valueIndex = 0;
                for (const Json::Value &byte : jsonRoot[propIdStr][areaIdStr]["value"]["byteValues"]) {
                    nonVolatilePropValue.value.byteValues.resize(valueIndex + 1);
                    nonVolatilePropValue.value.byteValues[valueIndex] = (uint8_t)byte.asUInt();
                    valueIndex++;
                }
                nonVolatilePropValue.value.stringValue = jsonRoot[propIdStr][areaIdStr]["value"]["stringValue"].asString();
                break;
            default:
                LOG(ERROR) << __func__ << ":unsupported type for property";
                continue;
            }
            nonVolatileVPValueSettings.push_back(nonVolatilePropValue);
            LOG(INFO) << __func__ << ":json read : property :" << nonVolatilePropValue.toString();
        }
    }
    return nonVolatileVPValueSettings;
}

std::map<uint16_t, VehiclePropValue> FakeVehicleHardware::readCostomizeCustomCheckVPValueSettings() {
    std::map<uint16_t, VehiclePropValue> customCheckVPValueSettings;

    // Read json file.
    Json::Value jsonRoot;

    if (!readJsonFile(FILE_NAME_CUSTOMIZE_CUSTOMCHECK, jsonRoot)) {
        EMULOG_D("json read : customize customcheck file not found");
        return customCheckVPValueSettings;
    }

    // Get all "pfId×ecuId" names.
    Json::Value::Members pfEcuIdList = jsonRoot.getMemberNames();
    for (const std::string& pfEcuIdStr : pfEcuIdList) {
        // Exchange hex number
        uint16_t pfEcuId = strtol(pfEcuIdStr.c_str(), NULL, 16);
        VehiclePropValue customCheckPropValue;
        // propId and areaId is fixed
        customCheckPropValue.prop = (int32_t)HONDA_CUSTOMIZE_CUSTOMCHECK;
        customCheckPropValue.areaId = 0;
        int valueIndex = 0;
        for (const Json::Value& byte : jsonRoot[pfEcuIdStr]) {
            customCheckPropValue.value.byteValues.resize(valueIndex + 1);
            customCheckPropValue.value.byteValues[valueIndex] = (uint8_t)strtol(byte.asString().c_str(), NULL, 16);
            valueIndex++;
        }
        customCheckVPValueSettings[pfEcuId] = (customCheckPropValue);
        EMULOG_I("json read : pfEcuId : 0x%x, property : %s", pfEcuId, customCheckPropValue.toString().c_str());
    }
    return customCheckVPValueSettings;
}

void FakeVehicleHardware::readCostomizeCustomGetResponseVPValueSettings() {
    // Read json file.
    Json::Value jsonRoot;

    if (!readJsonFile(FILE_NAME_CUSTOMIZE_CUSTOM_GET_RESPONSE, jsonRoot)) {
        EMULOG_D("json read : customize custom get res file not found");
        return;
    }

    // Get all "ecuId×menuNumber" names.
    Json::Value::Members pfEcuIdList = jsonRoot.getMemberNames();
    for (const std::string& pfEcuIdStr : pfEcuIdList) {
        // Exchange hex number
        uint16_t pfEcuId = strtol(pfEcuIdStr.c_str(), NULL, 16);
        std::array<uint8_t, kCustomSettingVal> value;
        int valueIndex = 0;
        for (const Json::Value& byte : jsonRoot[pfEcuIdStr]) {
            if (valueIndex >= kCustomSettingVal) {
                // Ignore out of range values
                break;
            }
            value[valueIndex] = (uint8_t)strtol(byte.asString().c_str(), NULL, 16);
            valueIndex++;
        }
        mCustomizeCustomStore.writeValue(pfEcuId, value);
        EMULOG_I("json read : pfEcuId : 0x%x, byteValues = [6]{ %d,%d,%d,%d,%d,%d }", pfEcuId, value[0], value[1], value[2], value[3], value[4], value[5]);
    }
    mCustomizeCustomStore.setDefaultValue();
    return;
}

std::map<int32_t, VehiclePropValue> FakeVehicleHardware::readIndirectCommunicationVPValueSettings() {
    std::map<int32_t, VehiclePropValue> indirectVPValueSettings;

    // Read json file.
    Json::Value jsonRoot;

    if (!readJsonFile(FILE_NAME_INDIRECT_COMMUNICATION, jsonRoot)) {
        EMULOG_D("json read : indirect communication file not found");
        return indirectVPValueSettings;
    }

    indirectVPValueSettings = readDefaultIndirectCommunicationVPValueSettings();

    // Get vehicle settings names.
    Json::Value::Members vpList = jsonRoot.getMemberNames();
    for (const std::string& vpStr : vpList) {
        // Exchange hex number
        VehiclePropValue indirectPropValue;
        indirectPropValue.prop = getHondaVehicleSettingValue(vpStr.c_str());
        if (indirectPropValue.prop == 0x00) {
            continue;
        }
        int valueIndex = 0;
        for (const Json::Value& byte : jsonRoot[vpStr]) {
            indirectPropValue.value.byteValues.resize(valueIndex + 1);
            indirectPropValue.value.byteValues[valueIndex] = (uint8_t)strtol(byte.asString().c_str(), NULL, 16);
            valueIndex++;
        }
        indirectVPValueSettings[indirectPropValue.prop] = (indirectPropValue);
        EMULOG_I("json read : prop : 0x%x, property : %s", indirectPropValue.prop, indirectVPValueSettings[indirectPropValue.prop].toString().c_str());
    }
    return indirectVPValueSettings;
}

std::map<int32_t, VehiclePropValue> FakeVehicleHardware::readDefaultIndirectCommunicationVPValueSettings() {
    std::map<int32_t, VehiclePropValue> defaultValueSettings;
    for (size_t i = 0; i < defaultconfig::getDefaultConfigs().size(); i++) {
        int32_t tempPropId = defaultconfig::getDefaultConfigs()[i].config.prop;
        switch (tempPropId) {
        case HONDA_CUSTOMIZE_DWS_INIT_REQUEST_RECEIVED:
        case HONDA_CUSTOMIZE_ACC_BUZZER:
        case HONDA_CUSTOMIZE_CMBS_ALART:
        case HONDA_CUSTOMIZE_LKAS_BUZZER:
        case HONDA_CUSTOMIZE_RDMS:
        case HONDA_CUSTOMIZE_DAAS:
        case HONDA_CUSTOMIZE_IACC_SETUP:
        case HONDA_CUSTOMIZE_SIF:
        case HONDA_CUSTOMIZE_INTELLIGENT_LED_CST_STT:
        case HONDA_CUSTOMIZE_REV_MATCH:
        case HONDA_CUSTOMIZE_HSS:
        case HONDA_CUSTOMIZE_ADB:
        case HONDA_CUSTOMIZE_TSR_DISP_ONOFF:
        case HONDA_CUSTOMIZE_TSR_WARN_STATUS:
        case HONDA_CUSTOMIZE_TSR_WARN_KPH:
        case HONDA_CUSTOMIZE_TSR_WARN_MPH:
        case HONDA_CUSTOMIZE_TSR_SRBUZ:
        case HONDA_CUSTOMIZE_PEAEMG:
        case HONDA_CUSTOMIZE_ALC_CUST:
        case HONDA_CUSTOMIZE_BSI_SCM_CUST:
        case HONDA_CUSTOMIZE_FCTW_ATT:
        case HONDA_CUSTOMIZE_ACC_CSA:
        case HONDA_CUSTOMIZE_PKS_RR:
        case HONDA_CUSTOMIZE_EAS_PH:
        case HONDA_CUSTOMIZE_EAS_LC:
        case HONDA_CUSTOMIZE_IDS_ENGINE:
        case HONDA_CUSTOMIZE_IDS_SUSPENSION:
        case HONDA_CUSTOMIZE_IDS_STEERING:
        case HONDA_CUSTOMIZE_IDS_GAUGE:
        case HONDA_CUSTOMIZE_IDS_IDLESTOP:
        case HONDA_CUSTOMIZE_IDS_REVMATCH:
        case HONDA_CUSTOMIZE_IDS_PTSOUND:
        case HONDA_CUSTOMIZE_IDS_ACC:
        case HONDA_CUSTOMIZE_IDS_LIGHTING:
        case HONDA_EVS_ONOFF_STATUS:
        case HONDA_EVS_IDS_COLLABO_STATUS:
        case HONDA_EVS_SOUND_TYPE_STATUS:
        case HONDA_EVS_VOLUME_STATUS:
        case HONDA_EVS_BALANCE_TYPE_ALL_STATUS:
        case HONDA_EVS_BALANCE_TYPE_DR_STATUS:
        case HONDA_EVS_BALANCE_TYPE_FR_STATUS:
        case HONDA_EVS_BALANCE_TYPE_RR_STATUS:
        case HONDA_EVS_SOUND_TYPE_PRESET_STATUS:
        case HONDA_CUSTOMIZE_APS_AUTO_EPB:
        case HONDA_CUSTOMIZE_APS_DETECT_SOUND:
        case HONDA_CUSTOMIZE_APS_MEMORY_DELE:
        case HONDA_ROFMOD_THEME_COLOR:
        case HONDA_ROFMOD_ROOMLIGHT:
        case HONDA_ROFMOD_BRI:
        case HONDA_LID_DOORLOCK_SW:
        case HONDA_CUSTOMIZE_ELM_RRFOG_CHANGE_MODE:
        case HONDA_CUSTOMIZE_ELM_LIGHT_CHANGE_MODE:
        case HONDA_CUSTOMIZE_ELM_FRFOG_CHANGE_MODE:
        case HONDA_CUSTOMIZE_DPMS_DR_MASG_MODE:
        case HONDA_CUSTOMIZE_DPMS_AS_MASG_MODE:
        case HONDA_CUSTOMIZE_DPMS_MIRCOUNT1:
        case HONDA_CUSTOMIZE_DPMS_MIR_SELECT:
        case HONDA_CUSTOMIZE_DPMS_MIR_REVERSE:
        case HONDA_ROFMOD_SETTING_FB:
        case HONDA_CUSTOMIZE_GAUGE_PATTERN:
        case HONDA_CUSTOMIZE_WARNING_MSG:
        case HONDA_CUSTOMIZE_EXTEMP_DISP_C:
        case HONDA_CUSTOMIZE_EXTEMP_DISP_F:
        case HONDA_CUSTOMIZE_MTGEAR_DISP:
        case HONDA_CUSTOMIZE_TRIPA_RESET_GAS:
        case HONDA_CUSTOMIZE_TRIPA_RESET_PHEV:
        case HONDA_CUSTOMIZE_TRIPA_RESET_BEV:
        case HONDA_CUSTOMIZE_TRIPA_RESET_FCV:
        case HONDA_CUSTOMIZE_TRIPB_RESET_GAS:
        case HONDA_CUSTOMIZE_TRIPB_RESET_PHEV:
        case HONDA_CUSTOMIZE_TRIPB_RESET_BEV:
        case HONDA_CUSTOMIZE_TRIPB_RESET_FCV:
        case HONDA_CUSTOMIZE_ALARM_VOL:
        case HONDA_CUSTOMIZE_REVERSE_ALARM:
        case HONDA_CUSTOMIZE_REV_INDICATOR:
        case HONDA_CUSTOMIZE_AMBIENT_METER:
        case HONDA_CUSTOMIZE_IDLESTOP_GUIDANCE:
        case HONDA_CUSTOMIZE_TBT_DISPLAY:
        case HONDA_CUSTOMIZE_REARSEAT_REMINDER:
        case HONDA_CUSTOMIZE_DRIVERANGE_NOTIFY:
        case HONDA_CUSTOMIZE_TIREANGLE_MONITOR:
        case HONDA_CUSTOMIZE_DRIVE_UNIT:
        case HONDA_CUSTOMIZE_CONSUMPTION_UNIT_KM:
        case HONDA_CUSTOMIZE_SHIFTUP_TIMING_ALARM:
        case HONDA_CUSTOMIZE_LANGUAGE_EU:
        case HONDA_CUSTOMIZE_LANGUAGE_US:
        case HONDA_CUSTOMIZE_LANGUAGE_CN:
        case HONDA_CUSTOMIZE_LANGUAGE_PT:
        case HONDA_CUSTOMIZE_LANGUAGE_KR:
        case HONDA_CUSTOMIZE_GAUGE_PATTERN_PF3:
        case HONDA_CUSTOMIZE_TBT_DISPLAY_HUD:
        case HONDA_CUSTOMIZE_ADAS_DISPLAY_HUD:
        case HONDA_CUSTOMIZE_SPEEDTSR_DISPLAY_HUD:
        case HONDA_CUSTOMIZE_SPEED_DISPLAY_HUD:
        case HONDA_CUSTOMIZE_DETAIL_DISPLAY_HUD:
        case HONDA_CUSTOMIZE_TRAIL_GAUGE:
        case HONDA_CUSTOMIZE_MET_PS_MID:
        case HONDA_CUSTOMIZE_GET_DID_SETTING:
        case HONDA_CUSTOMIZE_SW_TURN_MODE:
        case HONDA_CUSTOMIZE_EAS_PARK_MODE:
        case HONDA_CUSTOMIZE_EAS_WELCOME_MODE:
        case HONDA_CUSTOMIZE_IDS_RESET_STATUS2:
        case HONDA_CUSTOMIZE_IDS_ENGINE2:
        case HONDA_CUSTOMIZE_IDS_STEERING2:
        case HONDA_CUSTOMIZE_IDS_SUSPENSION2:
        case HONDA_CUSTOMIZE_IDS_AWD2:
        case HONDA_CUSTOMIZE_PW_SW_LOCK:
        case HONDA_CUSTOMIZE_SUNLOOF_KEYLESS:
        case HONDA_CUSTOMIZE_WIP_MAINTENANCE_MODE:
        case HONDA_CUSTOMIZE_SEAT_DIRECTION:
        case HONDA_CUSTOMIZE_SEATCOLUMN_DIRECTION:
        case HONDA_CUSTOMIZE_IDS_PTSOUND2:
        case HONDA_CUSTOMIZE_ADAS_NOTIF_LED_CST_STT:
        case HONDA_CUSTOMIZE_TLI_RED_FUN:
        case HONDA_CUSTOMIZE_TLI_GREEN_FUN:
        case HONDA_METER_HUD_ONOFF:
        case HONDA_DIMMING_GLASS_STATE:
        case HONDA_CUSTOMIZE_DPMS_DR_MASG_STATUS:
        case HONDA_CUSTOMIZE_DPMS_AS_MASG_STATUS:
        case HONDA_HVAC_TEMPVARIATION:
        case HONDA_HVAC_RR_FUNC_FEATURE:
        case HONDA_CUSTOMIZE_HVAC_LEARN_AUTO_FUNC:
        case HONDA_CUSTOMIZE_MVC_INTERLOCK:
        case HONDA_CUSTOMIZE_REFERLINE:
        case HONDA_CUSTOMIZE_HUD_CONTENTS:
        case HONDA_CUSTOMIZE_DID_CONTENTS:
        case HONDA_CUSTOMIZE_TBT_DISPLAY_DID:
        case HONDA_CUSTOMIZE_AMN_DISPLAY_DID:
        case HONDA_CUSTOMIZE_AUDIO_DISPLAY_DID:
        case HONDA_CUSTOMIZE_HFL_DISPLAY_DID:
        case HONDA_CUSTOMIZE_SR_DISPLAY_DID:
        case HONDA_CUSTOMIZE_SMS_DISPLAY_DID:
        case HONDA_CUSTOMIZE_COMPANION_DISPLAY_DID:
        case HONDA_ENTRY_CPD_VEHICLE_CONFIG:
        case HONDA_METER_LANGUAGE_RECEIVE_RESPONSE:
        case HONDA_LID_DR_OPCL_SW:
        case HONDA_LID_AS_OPCL_SW:
        case HONDA_LID_RD_OPCL_SW:
        case HONDA_LID_RA_OPCL_SW:
        case HONDA_CUSTOMIZE_AD_MAIN_STT:
        case HONDA_CUSTOMIZE_ACC_CC_MODE_STT:
        case HONDA_CUSTOMIZE_ACC_LIM_MODE_STT:
        case HONDA_CUSTOMIZE_LKAS_ENABLE_STT:
        case HONDA_CUSTOMIZE_AHD_CUST_ONOFF:
        case HONDA_CUSTOMIZE_ACC_DIST_STT:
        case HONDA_CUSTOMIZE_ACC_SLA_START_SPEED_STT:
        case HONDA_CUSTOMIZE_IACC_ONOFF_STT:
        case HONDA_CUSTOMIZE_ACC_SLA_MODE_WO_AUTO_STT:
        case HONDA_CUSTOMIZE_ACC_SLA_MODE_W_AUTO_STT:
        case HONDA_CUSTOMIZE_ACC_SLA_OFFSET_KPH_STT:
        case HONDA_CUSTOMIZE_ACC_SLA_OFFSET_MPH_STT:
        case HONDA_CUSTOMIZE_ACC_BLINK_AXEL_STT:
        case HONDA_CUSTOMIZE_ACC_RESUME_ONOFF_STT:
        case HONDA_CUSTOMIZE_ACC_AXELMODE_STT:
        case HONDA_CUSTOMIZE_ADA_ONOFF_STT:
        case HONDA_CUSTOMIZE_ADA_FUNC_STT:
        case HONDA_CUSTOMIZE_ADA_LONINTENSITY_STT:
        case HONDA_CUSTOMIZE_ADA_LONCURVE_STT:
        case HONDA_CUSTOMIZE_ADA_LATINTENSITY_STT:
        case HONDA_CUSTOMIZE_AILD_HANDS_OFF_ALARM_STT:
        case HONDA_CUSTOMIZE_ALCA_GAP_SEARCH_CUST_STT:
        case HONDA_CUSTOMIZE_ALCA_TURN_LEVER_CUST_STT:
        case HONDA_CUSTOMIZE_ALCA_ALC_CUST:
        case HONDA_CUSTOMIZE_ALCR_ALC_ONOFF_CUST_STT:
        case HONDA_CUSTOMIZE_ALCR_PAS_CUST_RESULT:
        case HONDA_CUSTOMIZE_ALC_NAVI_CUST_STT:
        case HONDA_CUSTOMIZE_ALCR_NAV_CUST_RESULT:
        case HONDA_CUSTOMIZE_AMN_ONOFF_STT:
        case HONDA_CUSTOMIZE_AMN_NAV_TIMING_STT:
        case HONDA_CUSTOMIZE_AMN_HOV_ENABLE_STT:
        case HONDA_CUSTOMIZE_AMN_EXPRESS_ENABLE_STT:
        case HONDA_CUSTOMIZE_AMN_INTERRUPT_DISP_STT:
        case HONDA_BIOENT_CUSTOM_MENU_BIOENT_INTERFACE:
        case HONDA_CUSTOMIZE_WELCOME_SOUND:
        case HONDA_CUSTOM_ADAS_E_BSI_ONOFF:
        case HONDA_CUSTOM_ADAS_E_BSI_RANGE:
        case HONDA_CUSTOM_ADAS_E_BSI_2R:
        case HONDA_CUSTOMIZE_C_LID_AUTO:
        case HONDA_CUSTOMIZE_EC_LID_CLOSE_TIMER:
        case HONDA_CUSTOMIZE_EC_LID_AUTO_CLOSE:
        case HONDA_CUSTOM_ADAS_STATUS_CMBS:
        case HONDA_CUSTOM_ADAS_FCTW:
        case HONDA_CUSTOM_DAM_ALLOFF_SW:
        case HONDA_CUSTOM_ADAS_AILD_ENABLE:
        case HONDA_CUSTOM_ADAS_ALC_ONOFF_CUST:
        case HONDA_CUSTOM_ADAS_E_BSI_CUST:
        case HONDA_CUSTOM_ADAS_SAFETY_SW_STATUS_PKS:
        case HONDA_CUSTOM_ADAS_SAFETY_SW_STATUS_LSAEB:
        case HONDA_CUSTOM_ADAS_PKS_RR_STATUS:
        case HONDA_CORE_CUSTOM_SNP_CRP:
        case HONDA_CUSTOMIZE_RDMS_ONE:
        case HONDA_CUSTOMIZE_RDMS_TWO:
        case HONDA_CUSTOMIZE_EW_CUST_RESULT:
        case HONDA_CUSTOMIZE_DMC_DROWSY_SETTING:
        case HONDA_CUSTOMIZE_DMC_DIST_SETTING:
        case HONDA_CUSTOMIZE_AES_ONOFF_STT:
        case HONDA_CUSTOMIZE_DESS_CUST_STT:
        case HONDA_CUSTOMIZE_EW_FUNC_ONOFF_STT:
        case HONDA_CUSTOMIZE_EW_LV1_ONOFF_STT:
        case HONDA_CUSTOMIZE_EW_ELATCH_ONOFF_STT:
        case HONDA_CUSTOMIZE_EW_ELATCH_SENSE_STT:
        case HONDA_CUSTOMIZE_BSINFO_VCHANGE_BCF:
        case HONDA_CUSTOMIZE_ALARM_TYPE:
        case HONDA_CUSTOMIZE_BSI_BUZZER_VOLUME_STT:
        case HONDA_CUSTOMIZE_BSI_RANGE_STT:
        case HONDA_CUSTOMIZE_BSI_2R_STT:
        case HONDA_CUSTOMIZE_BSI_TRAILER_STT:
        case HONDA_CUSTOMIZE_BSI_TRAILER_TYPE_STT:
        case HONDA_CUSTOMIZE_BSI_TRAILER_WIDTH_STT:
        case HONDA_CUSTOMIZE_BSI_TRAILER_LENGTH_STT:
        case HONDA_CUSTOMIZE_PCDW_ONOFF_STT:
        case HONDA_CUSTOMIZE_ODSF_ONOFF:
        case HONDA_CUSTOMIZE_FCM_INFO_TSR_SRBUZ_ONOFF:
        case HONDA_CUSTOMIZE_DAM_EYECLOSED_STT:
        case HONDA_METER_HUD_ILLUMI:
        case HONDA_CUSTOMIZE_VOL_DISPLAY_DID:
        case HONDA_CUSTOMIZE_HEADLIGHT_TIMER:
        case HONDA_CUSTOMIZE_WELCOM_LIGHT_1:
        case HONDA_CUSTOMIZE_WELCOM_LIGHT_2:
        case HONDA_CUSTOMIZE_WELCOM_LIGHT_1_2:
        case HONDA_EVS_BALANCE_STAT: {
            VehiclePropValue defaultPropValue;
            defaultPropValue.prop = tempPropId;
            defaultPropValue.value = defaultconfig::getDefaultConfigs()[i].initialValue;
            defaultValueSettings[tempPropId] = (defaultPropValue);
            break;
        }
        default:
            break;
        }
    }
    return defaultValueSettings;
}

int32_t FakeVehicleHardware::getHondaVehicleSettingValue(std::string val_str) {
    int32_t value = 0;
    if (val_str == "HONDA_CUSTOMIZE_DWS_INIT_REQUEST_RECEIVED") {
        value = (int32_t)HONDA_CUSTOMIZE_DWS_INIT_REQUEST_RECEIVED;
    } else if (val_str == "HONDA_CUSTOMIZE_ACC_BUZZER") {
        value = (int32_t)HONDA_CUSTOMIZE_ACC_BUZZER;
    } else if (val_str == "HONDA_CUSTOMIZE_CMBS_ALART") {
        value = (int32_t)HONDA_CUSTOMIZE_CMBS_ALART;
    } else if (val_str == "HONDA_CUSTOMIZE_LKAS_BUZZER") {
        value = (int32_t)HONDA_CUSTOMIZE_LKAS_BUZZER;
    } else if (val_str == "HONDA_CUSTOMIZE_RDMS") {
        value = (int32_t)HONDA_CUSTOMIZE_RDMS;
    } else if (val_str == "HONDA_CUSTOMIZE_DAAS") {
        value = (int32_t)HONDA_CUSTOMIZE_DAAS;
    } else if (val_str == "HONDA_CUSTOMIZE_IACC_SETUP") {
        value = (int32_t)HONDA_CUSTOMIZE_IACC_SETUP;
    } else if (val_str == "HONDA_CUSTOMIZE_SIF") {
        value = (int32_t)HONDA_CUSTOMIZE_SIF;
    } else if (val_str == "HONDA_CUSTOMIZE_INTELLIGENT_LED_CST_STT") {
        value = (int32_t)HONDA_CUSTOMIZE_INTELLIGENT_LED_CST_STT;
    } else if (val_str == "HONDA_CUSTOMIZE_REV_MATCH") {
        value = (int32_t)HONDA_CUSTOMIZE_REV_MATCH;
    } else if (val_str == "HONDA_CUSTOMIZE_HSS") {
        value = (int32_t)HONDA_CUSTOMIZE_HSS;
    } else if (val_str == "HONDA_CUSTOMIZE_ADB") {
        value = (int32_t)HONDA_CUSTOMIZE_ADB;
    } else if (val_str == "HONDA_CUSTOMIZE_TSR_DISP_ONOFF") {
        value = (int32_t)HONDA_CUSTOMIZE_TSR_DISP_ONOFF;
    } else if (val_str == "HONDA_CUSTOMIZE_TSR_WARN_STATUS") {
        value = (int32_t)HONDA_CUSTOMIZE_TSR_WARN_STATUS;
    } else if (val_str == "HONDA_CUSTOMIZE_TSR_WARN_KPH") {
        value = (int32_t)HONDA_CUSTOMIZE_TSR_WARN_KPH;
    } else if (val_str == "HONDA_CUSTOMIZE_TSR_WARN_MPH") {
        value = (int32_t)HONDA_CUSTOMIZE_TSR_WARN_MPH;
    } else if (val_str == "HONDA_CUSTOMIZE_TSR_SRBUZ") {
        value = (int32_t)HONDA_CUSTOMIZE_TSR_SRBUZ;
    } else if (val_str == "HONDA_CUSTOMIZE_PEAEMG") {
        value = (int32_t)HONDA_CUSTOMIZE_PEAEMG;
    } else if (val_str == "HONDA_CUSTOMIZE_ALC_CUST") {
        value = (int32_t)HONDA_CUSTOMIZE_ALC_CUST;
    } else if (val_str == "HONDA_CUSTOMIZE_BSI_SCM_CUST") {
        value = (int32_t)HONDA_CUSTOMIZE_BSI_SCM_CUST;
    } else if (val_str == "HONDA_CUSTOMIZE_FCTW_ATT") {
        value = (int32_t)HONDA_CUSTOMIZE_FCTW_ATT;
    } else if (val_str == "HONDA_CUSTOMIZE_ACC_CSA") {
        value = (int32_t)HONDA_CUSTOMIZE_ACC_CSA;
    } else if (val_str == "HONDA_CUSTOMIZE_PKS_RR") {
        value = (int32_t)HONDA_CUSTOMIZE_PKS_RR;
    } else if (val_str == "HONDA_CUSTOMIZE_EAS_PH") {
        value = (int32_t)HONDA_CUSTOMIZE_EAS_PH;
    } else if (val_str == "HONDA_CUSTOMIZE_EAS_LC") {
        value = (int32_t)HONDA_CUSTOMIZE_EAS_LC;
    } else if (val_str == "HONDA_CUSTOMIZE_IDS_ENGINE") {
        value = (int32_t)HONDA_CUSTOMIZE_IDS_ENGINE;
    } else if (val_str == "HONDA_CUSTOMIZE_IDS_SUSPENSION") {
        value = (int32_t)HONDA_CUSTOMIZE_IDS_SUSPENSION;
    } else if (val_str == "HONDA_CUSTOMIZE_IDS_STEERING") {
        value = (int32_t)HONDA_CUSTOMIZE_IDS_STEERING;
    } else if (val_str == "HONDA_CUSTOMIZE_IDS_GAUGE") {
        value = (int32_t)HONDA_CUSTOMIZE_IDS_GAUGE;
    } else if (val_str == "HONDA_CUSTOMIZE_IDS_IDLESTOP") {
        value = (int32_t)HONDA_CUSTOMIZE_IDS_IDLESTOP;
    } else if (val_str == "HONDA_CUSTOMIZE_IDS_REVMATCH") {
        value = (int32_t)HONDA_CUSTOMIZE_IDS_REVMATCH;
    } else if (val_str == "HONDA_CUSTOMIZE_IDS_PTSOUND") {
        value = (int32_t)HONDA_CUSTOMIZE_IDS_PTSOUND;
    } else if (val_str == "HONDA_CUSTOMIZE_IDS_ACC") {
        value = (int32_t)HONDA_CUSTOMIZE_IDS_ACC;
    } else if (val_str == "HONDA_CUSTOMIZE_IDS_LIGHTING") {
        value = (int32_t)HONDA_CUSTOMIZE_IDS_LIGHTING;
    } else if (val_str == "HONDA_EVS_ONOFF_STATUS") {
        value = (int32_t)HONDA_EVS_ONOFF_STATUS;
    } else if (val_str == "HONDA_EVS_IDS_COLLABO_STATUS") {
        value = (int32_t)HONDA_EVS_IDS_COLLABO_STATUS;
    } else if (val_str == "HONDA_EVS_SOUND_TYPE_STATUS") {
        value = (int32_t)HONDA_EVS_SOUND_TYPE_STATUS;
    } else if (val_str == "HONDA_EVS_VOLUME_STATUS") {
        value = (int32_t)HONDA_EVS_VOLUME_STATUS;
    } else if (val_str == "HONDA_EVS_BALANCE_TYPE_ALL_STATUS") {
        value = (int32_t)HONDA_EVS_BALANCE_TYPE_ALL_STATUS;
    } else if (val_str == "HONDA_EVS_BALANCE_TYPE_DR_STATUS") {
        value = (int32_t)HONDA_EVS_BALANCE_TYPE_DR_STATUS;
    } else if (val_str == "HONDA_EVS_VOLUME_STATUS") {
        value = (int32_t)HONDA_EVS_VOLUME_STATUS;
    } else if (val_str == "HONDA_EVS_BALANCE_TYPE_FR_STATUS") {
        value = (int32_t)HONDA_EVS_BALANCE_TYPE_FR_STATUS;
    } else if (val_str == "HONDA_EVS_BALANCE_TYPE_RR_STATUS") {
        value = (int32_t)HONDA_EVS_BALANCE_TYPE_RR_STATUS;
    } else if (val_str == "HONDA_EVS_SOUND_TYPE_PRESET_STATUS") {
        value = (int32_t)HONDA_EVS_SOUND_TYPE_PRESET_STATUS;
    } else if (val_str == "HONDA_CUSTOMIZE_APS_AUTO_EPB") {
        value = (int32_t)HONDA_CUSTOMIZE_APS_AUTO_EPB;
    } else if (val_str == "HONDA_CUSTOMIZE_APS_DETECT_SOUND") {
        value = (int32_t)HONDA_CUSTOMIZE_APS_DETECT_SOUND;
    } else if (val_str == "HONDA_CUSTOMIZE_APS_MEMORY_DELE") {
        value = (int32_t)HONDA_CUSTOMIZE_APS_MEMORY_DELE;
    } else if (val_str == "HONDA_ROFMOD_THEME_COLOR") {
        value = (int32_t)HONDA_ROFMOD_THEME_COLOR;
    } else if (val_str == "HONDA_ROFMOD_ROOMLIGHT") {
        value = (int32_t)HONDA_ROFMOD_ROOMLIGHT;
    } else if (val_str == "HONDA_ROFMOD_BRI") {
        value = (int32_t)HONDA_ROFMOD_BRI;
    } else if (val_str == "HONDA_LID_DOORLOCK_SW") {
        value = (int32_t)HONDA_LID_DOORLOCK_SW;
    } else if (val_str == "HONDA_CUSTOMIZE_ELM_RRFOG_CHANGE_MODE") {
        value = (int32_t)HONDA_CUSTOMIZE_ELM_RRFOG_CHANGE_MODE;
    } else if (val_str == "HONDA_CUSTOMIZE_ELM_LIGHT_CHANGE_MODE") {
        value = (int32_t)HONDA_CUSTOMIZE_ELM_LIGHT_CHANGE_MODE;
    } else if (val_str == "HONDA_CUSTOMIZE_ELM_FRFOG_CHANGE_MODE") {
        value = (int32_t)HONDA_CUSTOMIZE_ELM_FRFOG_CHANGE_MODE;
    } else if (val_str == "HONDA_CUSTOMIZE_DPMS_DR_MASG_MODE") {
        value = (int32_t)HONDA_CUSTOMIZE_DPMS_DR_MASG_MODE;
    } else if (val_str == "HONDA_CUSTOMIZE_DPMS_AS_MASG_MODE") {
        value = (int32_t)HONDA_CUSTOMIZE_DPMS_AS_MASG_MODE;
    } else if (val_str == "HONDA_CUSTOMIZE_DPMS_MIRCOUNT1") {
        value = (int32_t)HONDA_CUSTOMIZE_DPMS_MIRCOUNT1;
    } else if (val_str == "HONDA_CUSTOMIZE_DPMS_MIR_SELECT") {
        value = (int32_t)HONDA_CUSTOMIZE_DPMS_MIR_SELECT;
    } else if (val_str == "HONDA_CUSTOMIZE_DPMS_MIR_REVERSE") {
        value = (int32_t)HONDA_CUSTOMIZE_DPMS_MIR_REVERSE;
    } else if (val_str == "HONDA_ROFMOD_SETTING_FB") {
        value = (int32_t)HONDA_ROFMOD_SETTING_FB;
    } else if (val_str == "HONDA_METER_LANGUAGE_RECEIVE_RESPONSE") {
        value = (int32_t)HONDA_METER_LANGUAGE_RECEIVE_RESPONSE;
    } else if (val_str == "HONDA_CUSTOMIZE_GAUGE_PATTERN") {
        value = (int32_t)HONDA_CUSTOMIZE_GAUGE_PATTERN;
    } else if (val_str == "HONDA_CUSTOMIZE_WARNING_MSG") {
        value = (int32_t)HONDA_CUSTOMIZE_WARNING_MSG;
    } else if (val_str == "HONDA_CUSTOMIZE_EXTEMP_DISP_C") {
        value = (int32_t)HONDA_CUSTOMIZE_EXTEMP_DISP_C;
    } else if (val_str == "HONDA_CUSTOMIZE_EXTEMP_DISP_F") {
        value = (int32_t)HONDA_CUSTOMIZE_EXTEMP_DISP_F;
    } else if (val_str == "HONDA_CUSTOMIZE_MTGEAR_DISP") {
        value = (int32_t)HONDA_CUSTOMIZE_MTGEAR_DISP;
    } else if (val_str == "HONDA_CUSTOMIZE_TRIPA_RESET_GAS") {
        value = (int32_t)HONDA_CUSTOMIZE_TRIPA_RESET_GAS;
    } else if (val_str == "HONDA_CUSTOMIZE_TRIPA_RESET_PHEV") {
        value = (int32_t)HONDA_CUSTOMIZE_TRIPA_RESET_PHEV;
    } else if (val_str == "HONDA_CUSTOMIZE_TRIPA_RESET_BEV") {
        value = (int32_t)HONDA_CUSTOMIZE_TRIPA_RESET_BEV;
    } else if (val_str == "HONDA_CUSTOMIZE_TRIPA_RESET_FCV") {
        value = (int32_t)HONDA_CUSTOMIZE_TRIPA_RESET_FCV;
    } else if (val_str == "HONDA_CUSTOMIZE_TRIPB_RESET_GAS") {
        value = (int32_t)HONDA_CUSTOMIZE_TRIPB_RESET_GAS;
    } else if (val_str == "HONDA_CUSTOMIZE_TRIPB_RESET_PHEV") {
        value = (int32_t)HONDA_CUSTOMIZE_TRIPB_RESET_PHEV;
    } else if (val_str == "HONDA_CUSTOMIZE_TRIPB_RESET_BEV") {
        value = (int32_t)HONDA_CUSTOMIZE_TRIPB_RESET_BEV;
    } else if (val_str == "HONDA_CUSTOMIZE_TRIPB_RESET_FCV") {
        value = (int32_t)HONDA_CUSTOMIZE_TRIPB_RESET_FCV;
    } else if (val_str == "HONDA_CUSTOMIZE_ALARM_VOL") {
        value = (int32_t)HONDA_CUSTOMIZE_ALARM_VOL;
    } else if (val_str == "HONDA_CUSTOMIZE_REVERSE_ALARM") {
        value = (int32_t)HONDA_CUSTOMIZE_REVERSE_ALARM;
    } else if (val_str == "HONDA_CUSTOMIZE_REV_INDICATOR") {
        value = (int32_t)HONDA_CUSTOMIZE_REV_INDICATOR;
    } else if (val_str == "HONDA_CUSTOMIZE_AMBIENT_METER") {
        value = (int32_t)HONDA_CUSTOMIZE_AMBIENT_METER;
    } else if (val_str == "HONDA_CUSTOMIZE_IDLESTOP_GUIDANCE") {
        value = (int32_t)HONDA_CUSTOMIZE_IDLESTOP_GUIDANCE;
    } else if (val_str == "HONDA_CUSTOMIZE_TBT_DISPLAY") {
        value = (int32_t)HONDA_CUSTOMIZE_TBT_DISPLAY;
    } else if (val_str == "HONDA_CUSTOMIZE_REARSEAT_REMINDER") {
        value = (int32_t)HONDA_CUSTOMIZE_REARSEAT_REMINDER;
    } else if (val_str == "HONDA_CUSTOMIZE_DRIVERANGE_NOTIFY") {
        value = (int32_t)HONDA_CUSTOMIZE_DRIVERANGE_NOTIFY;
    } else if (val_str == "HONDA_CUSTOMIZE_TIREANGLE_MONITOR") {
        value = (int32_t)HONDA_CUSTOMIZE_TIREANGLE_MONITOR;
    } else if (val_str == "HONDA_CUSTOMIZE_DRIVE_UNIT") {
        value = (int32_t)HONDA_CUSTOMIZE_DRIVE_UNIT;
    } else if (val_str == "HONDA_CUSTOMIZE_CONSUMPTION_UNIT_KM") {
        value = (int32_t)HONDA_CUSTOMIZE_CONSUMPTION_UNIT_KM;
    } else if (val_str == "HONDA_CUSTOMIZE_SHIFTUP_TIMING_ALARM") {
        value = (int32_t)HONDA_CUSTOMIZE_SHIFTUP_TIMING_ALARM;
    } else if (val_str == "HONDA_CUSTOMIZE_LANGUAGE_EU") {
        value = (int32_t)HONDA_CUSTOMIZE_LANGUAGE_EU;
    } else if (val_str == "HONDA_CUSTOMIZE_LANGUAGE_US") {
        value = (int32_t)HONDA_CUSTOMIZE_LANGUAGE_US;
    } else if (val_str == "HONDA_CUSTOMIZE_LANGUAGE_CN") {
        value = (int32_t)HONDA_CUSTOMIZE_LANGUAGE_CN;
    } else if (val_str == "HONDA_CUSTOMIZE_LANGUAGE_PT") {
        value = (int32_t)HONDA_CUSTOMIZE_LANGUAGE_PT;
    } else if (val_str == "HONDA_CUSTOMIZE_LANGUAGE_KR") {
        value = (int32_t)HONDA_CUSTOMIZE_LANGUAGE_KR;
    } else if (val_str == "HONDA_CUSTOMIZE_GAUGE_PATTERN_PF3") {
        value = (int32_t)HONDA_CUSTOMIZE_GAUGE_PATTERN_PF3;
    } else if (val_str == "HONDA_CUSTOMIZE_TBT_DISPLAY_HUD") {
        value = (int32_t)HONDA_CUSTOMIZE_TBT_DISPLAY_HUD;
    } else if (val_str == "HONDA_CUSTOMIZE_ADAS_DISPLAY_HUD") {
        value = (int32_t)HONDA_CUSTOMIZE_ADAS_DISPLAY_HUD;
    } else if (val_str == "HONDA_CUSTOMIZE_SPEEDTSR_DISPLAY_HUD") {
        value = (int32_t)HONDA_CUSTOMIZE_SPEEDTSR_DISPLAY_HUD;
    } else if (val_str == "HONDA_CUSTOMIZE_SPEED_DISPLAY_HUD") {
        value = (int32_t)HONDA_CUSTOMIZE_SPEED_DISPLAY_HUD;
    } else if (val_str == "HONDA_CUSTOMIZE_DETAIL_DISPLAY_HUD") {
        value = (int32_t)HONDA_CUSTOMIZE_DETAIL_DISPLAY_HUD;
    } else if (val_str == "HONDA_CUSTOMIZE_TRAIL_GAUGE") {
        value = (int32_t)HONDA_CUSTOMIZE_TRAIL_GAUGE;
    } else if (val_str == "HONDA_CUSTOMIZE_MET_PS_MID") {
        value = (int32_t)HONDA_CUSTOMIZE_MET_PS_MID;
    } else if (val_str == "HONDA_CUSTOMIZE_GET_DID_SETTING") {
        value = (int32_t)HONDA_CUSTOMIZE_GET_DID_SETTING;
    } else if (val_str == "HONDA_CUSTOMIZE_SW_TURN_MODE") {
        value = (int32_t)HONDA_CUSTOMIZE_SW_TURN_MODE;
    } else if (val_str == "HONDA_CUSTOMIZE_EAS_PARK_MODE") {
        value = (int32_t)HONDA_CUSTOMIZE_EAS_PARK_MODE;
    } else if (val_str == "HONDA_CUSTOMIZE_EAS_WELCOME_MODE") {
        value = (int32_t)HONDA_CUSTOMIZE_EAS_WELCOME_MODE;
    } else if (val_str == "HONDA_CUSTOMIZE_IDS_RESET_STATUS2") {
        value = (int32_t)HONDA_CUSTOMIZE_IDS_RESET_STATUS2;
    } else if (val_str == "HONDA_CUSTOMIZE_IDS_ENGINE2") {
        value = (int32_t)HONDA_CUSTOMIZE_IDS_ENGINE2;
    } else if (val_str == "HONDA_CUSTOMIZE_IDS_STEERING2") {
        value = (int32_t)HONDA_CUSTOMIZE_IDS_STEERING2;
    } else if (val_str == "HONDA_CUSTOMIZE_IDS_SUSPENSION2") {
        value = (int32_t)HONDA_CUSTOMIZE_IDS_SUSPENSION2;
    } else if (val_str == "HONDA_CUSTOMIZE_IDS_AWD2") {
        value = (int32_t)HONDA_CUSTOMIZE_IDS_AWD2;
    } else if (val_str == "HONDA_CUSTOMIZE_PW_SW_LOCK") {
        value = (int32_t)HONDA_CUSTOMIZE_PW_SW_LOCK;
    } else if (val_str == "HONDA_CUSTOMIZE_SUNLOOF_KEYLESS") {
        value = (int32_t)HONDA_CUSTOMIZE_SUNLOOF_KEYLESS;
    } else if (val_str == "HONDA_CUSTOMIZE_WIP_MAINTENANCE_MODE") {
        value = (int32_t)HONDA_CUSTOMIZE_WIP_MAINTENANCE_MODE;
    } else if (val_str == "HONDA_CUSTOMIZE_SEAT_DIRECTION") {
        value = (int32_t)HONDA_CUSTOMIZE_SEAT_DIRECTION;
    } else if (val_str == "HONDA_CUSTOMIZE_SEATCOLUMN_DIRECTION") {
        value = (int32_t)HONDA_CUSTOMIZE_SEATCOLUMN_DIRECTION;
    } else if (val_str == "HONDA_CUSTOMIZE_IDS_PTSOUND2") {
        value = (int32_t)HONDA_CUSTOMIZE_IDS_PTSOUND2;
    } else if (val_str == "HONDA_CUSTOMIZE_ADAS_NOTIF_LED_CST_STT") {
        value = (int32_t)HONDA_CUSTOMIZE_ADAS_NOTIF_LED_CST_STT;
    } else if (val_str == "HONDA_CUSTOMIZE_TLI_RED_FUN") {
        value = (int32_t)HONDA_CUSTOMIZE_TLI_RED_FUN;
    } else if (val_str == "HONDA_CUSTOMIZE_TLI_GREEN_FUN") {
        value = (int32_t)HONDA_CUSTOMIZE_TLI_GREEN_FUN;
    } else if (val_str == "HONDA_METER_HUD_ONOFF") {
        value = (int32_t)HONDA_METER_HUD_ONOFF;
    } else if (val_str == "HONDA_CUSTOMIZE_DPMS_DR_MASG_STATUS") {
        value = (int32_t)HONDA_CUSTOMIZE_DPMS_DR_MASG_STATUS;
    } else if (val_str == "HONDA_CUSTOMIZE_DPMS_AS_MASG_STATUS") {
        value = (int32_t)HONDA_CUSTOMIZE_DPMS_AS_MASG_STATUS;
    } else if (val_str == "HONDA_HVAC_TEMPVARIATION") {
        value = (int32_t)HONDA_HVAC_TEMPVARIATION;
    } else if (val_str == "HONDA_HVAC_RR_FUNC_FEATURE") {
        value = (int32_t)HONDA_HVAC_RR_FUNC_FEATURE;
    } else if (val_str == "HONDA_DIMMING_GLASS_STATE") {
        value = (int32_t)HONDA_DIMMING_GLASS_STATE;
    } else if (val_str == "HONDA_NAVI_DIMMING_GLASS_REQ") {
        value = (int32_t)HONDA_NAVI_DIMMING_GLASS_REQ;
    } else if (val_str == "HONDA_NAVI_DIMMING_GLASS_REQ_PS") {
        value = (int32_t)HONDA_NAVI_DIMMING_GLASS_REQ_PS;
    } else if (val_str == "HONDA_CUSTOMIZE_HVAC_LEARN_AUTO_FUNC") {
        value = (int32_t)HONDA_CUSTOMIZE_HVAC_LEARN_AUTO_FUNC;
    } else if (val_str == "HONDA_CUSTOMIZE_MVC_INTERLOCK") {
        value = (int32_t)HONDA_CUSTOMIZE_MVC_INTERLOCK;
    } else if (val_str == "HONDA_CUSTOMIZE_REFERLINE") {
        value = (int32_t)HONDA_CUSTOMIZE_REFERLINE;
    } else if (val_str == "HONDA_CUSTOMIZE_HUD_CONTENTS") {
        value = (int32_t)HONDA_CUSTOMIZE_HUD_CONTENTS;
    } else if (val_str == "HONDA_CUSTOMIZE_DID_CONTENTS") {
        value = (int32_t)HONDA_CUSTOMIZE_DID_CONTENTS;
    } else if (val_str == "HONDA_CUSTOMIZE_TBT_DISPLAY_DID") {
        value = (int32_t)HONDA_CUSTOMIZE_TBT_DISPLAY_DID;
    } else if (val_str == "HONDA_CUSTOMIZE_AMN_DISPLAY_DID") {
        value = (int32_t)HONDA_CUSTOMIZE_AMN_DISPLAY_DID;
    } else if (val_str == "HONDA_CUSTOMIZE_AUDIO_DISPLAY_DID") {
        value = (int32_t)HONDA_CUSTOMIZE_AUDIO_DISPLAY_DID;
    } else if (val_str == "HONDA_CUSTOMIZE_HFL_DISPLAY_DID") {
        value = (int32_t)HONDA_CUSTOMIZE_HFL_DISPLAY_DID;
    } else if (val_str == "HONDA_CUSTOMIZE_SR_DISPLAY_DID") {
        value = (int32_t)HONDA_CUSTOMIZE_SR_DISPLAY_DID;
    } else if (val_str == "HONDA_CUSTOMIZE_SMS_DISPLAY_DID") {
        value = (int32_t)HONDA_CUSTOMIZE_SMS_DISPLAY_DID;
    } else if (val_str == "HONDA_CUSTOMIZE_COMPANION_DISPLAY_DID") {
        value = (int32_t)HONDA_CUSTOMIZE_COMPANION_DISPLAY_DID;
    } else if (val_str == "HONDA_ENTRY_CPD_VEHICLE_CONFIG") {
        value = (int32_t)HONDA_ENTRY_CPD_VEHICLE_CONFIG;
    } else if (val_str == "HONDA_LID_DR_OPCL_SW") {
        value = (int32_t)HONDA_LID_DR_OPCL_SW;
    } else if (val_str == "HONDA_LID_AS_OPCL_SW") {
        value = (int32_t)HONDA_LID_AS_OPCL_SW;
    } else if (val_str == "HONDA_LID_RD_OPCL_SW") {
        value = (int32_t)HONDA_LID_RD_OPCL_SW;
    } else if (val_str == "HONDA_LID_RA_OPCL_SW") {
        value = (int32_t)HONDA_LID_RA_OPCL_SW;
    } else if (val_str == "HONDA_CUSTOMIZE_AD_MAIN_STT") {
        value = (int32_t)HONDA_CUSTOMIZE_AD_MAIN_STT;
    } else if (val_str == "HONDA_CUSTOMIZE_ACC_CC_MODE_STT") {
        value = (int32_t)HONDA_CUSTOMIZE_ACC_CC_MODE_STT;
    } else if (val_str == "HONDA_CUSTOMIZE_ACC_LIM_MODE_STT") {
        value = (int32_t)HONDA_CUSTOMIZE_ACC_LIM_MODE_STT;
    } else if (val_str == "HONDA_CUSTOMIZE_LKAS_ENABLE_STT") {
        value = (int32_t)HONDA_CUSTOMIZE_LKAS_ENABLE_STT;
    } else if (val_str == "HONDA_CUSTOMIZE_AHD_CUST_ONOFF") {
        value = (int32_t)HONDA_CUSTOMIZE_AHD_CUST_ONOFF;
    } else if (val_str == "HONDA_CUSTOMIZE_ACC_DIST_STT") {
        value = (int32_t)HONDA_CUSTOMIZE_ACC_DIST_STT;
    } else if (val_str == "HONDA_CUSTOMIZE_ACC_SLA_START_SPEED_STT") {
        value = (int32_t)HONDA_CUSTOMIZE_ACC_SLA_START_SPEED_STT;
    } else if (val_str == "HONDA_CUSTOMIZE_ACC_CSA") {
        value = (int32_t)HONDA_CUSTOMIZE_ACC_CSA;
    } else if (val_str == "HONDA_CUSTOMIZE_ACC_BUZZER") {
        value = (int32_t)HONDA_CUSTOMIZE_ACC_BUZZER;
    } else if (val_str == "HONDA_CUSTOMIZE_IACC_ONOFF_STT") {
        value = (int32_t)HONDA_CUSTOMIZE_IACC_ONOFF_STT;
    } else if (val_str == "HONDA_CUSTOMIZE_ACC_SLA_MODE_WO_AUTO_STT") {
        value = (int32_t)HONDA_CUSTOMIZE_ACC_SLA_MODE_WO_AUTO_STT;
    } else if (val_str == "HONDA_CUSTOMIZE_ACC_SLA_MODE_W_AUTO_STT") {
        value = (int32_t)HONDA_CUSTOMIZE_ACC_SLA_MODE_W_AUTO_STT;
    } else if (val_str == "HONDA_CUSTOMIZE_ACC_SLA_OFFSET_KPH_STT") {
        value = (int32_t)HONDA_CUSTOMIZE_ACC_SLA_OFFSET_KPH_STT;
    } else if (val_str == "HONDA_CUSTOMIZE_ACC_SLA_OFFSET_MPH_STT") {
        value = (int32_t)HONDA_CUSTOMIZE_ACC_SLA_OFFSET_MPH_STT;
    } else if (val_str == "HONDA_CUSTOMIZE_ACC_BLINK_AXEL_STT") {
        value = (int32_t)HONDA_CUSTOMIZE_ACC_BLINK_AXEL_STT;
    } else if (val_str == "HONDA_CUSTOMIZE_ACC_RESUME_ONOFF_STT") {
        value = (int32_t)HONDA_CUSTOMIZE_ACC_RESUME_ONOFF_STT;
    } else if (val_str == "HONDA_CUSTOMIZE_ACC_AXELMODE_STT") {
        value = (int32_t)HONDA_CUSTOMIZE_ACC_AXELMODE_STT;
    } else if (val_str == "HONDA_CUSTOMIZE_ADA_ONOFF_STT") {
        value = (int32_t)HONDA_CUSTOMIZE_ADA_ONOFF_STT;
    } else if (val_str == "HONDA_CUSTOMIZE_ADA_FUNC_STT") {
        value = (int32_t)HONDA_CUSTOMIZE_ADA_FUNC_STT;
    } else if (val_str == "HONDA_CUSTOMIZE_ADA_LONINTENSITY_STT") {
        value = (int32_t)HONDA_CUSTOMIZE_ADA_LONINTENSITY_STT;
    } else if (val_str == "HONDA_CUSTOMIZE_ADA_LONCURVE_STT") {
        value = (int32_t)HONDA_CUSTOMIZE_ADA_LONCURVE_STT;
    } else if (val_str == "HONDA_CUSTOMIZE_ADA_LATINTENSITY_STT") {
        value = (int32_t)HONDA_CUSTOMIZE_ADA_LATINTENSITY_STT;
    } else if (val_str == "HONDA_CUSTOMIZE_AILD_HANDS_OFF_ALARM_STT") {
        value = (int32_t)HONDA_CUSTOMIZE_AILD_HANDS_OFF_ALARM_STT;
    } else if (val_str == "HONDA_CUSTOMIZE_ALC_CUST") {
        value = (int32_t)HONDA_CUSTOMIZE_ALC_CUST;
    } else if (val_str == "HONDA_CUSTOMIZE_ALCA_GAP_SEARCH_CUST_STT") {
        value = (int32_t)HONDA_CUSTOMIZE_ALCA_GAP_SEARCH_CUST_STT;
    } else if (val_str == "HONDA_CUSTOMIZE_ALCA_TURN_LEVER_CUST_STT") {
        value = (int32_t)HONDA_CUSTOMIZE_ALCA_TURN_LEVER_CUST_STT;
    } else if (val_str == "HONDA_CUSTOMIZE_ALCA_ALC_CUST") {
        value = (int32_t)HONDA_CUSTOMIZE_ALCA_ALC_CUST;
    } else if (val_str == "HONDA_CUSTOMIZE_ALCR_ALC_ONOFF_CUST_STT") {
        value = (int32_t)HONDA_CUSTOMIZE_ALCR_ALC_ONOFF_CUST_STT;
    } else if (val_str == "HONDA_CUSTOMIZE_ALCR_PAS_CUST_RESULT") {
        value = (int32_t)HONDA_CUSTOMIZE_ALCR_PAS_CUST_RESULT;
    } else if (val_str == "HONDA_CUSTOMIZE_ALC_NAVI_CUST_STT") {
        value = (int32_t)HONDA_CUSTOMIZE_ALC_NAVI_CUST_STT;
    } else if (val_str == "HONDA_CUSTOMIZE_ALCR_NAV_CUST_RESULT") {
        value = (int32_t)HONDA_CUSTOMIZE_ALCR_NAV_CUST_RESULT;
    } else if (val_str == "HONDA_CUSTOMIZE_AMN_ONOFF_STT") {
        value = (int32_t)HONDA_CUSTOMIZE_AMN_ONOFF_STT;
    } else if (val_str == "HONDA_CUSTOMIZE_AMN_NAV_TIMING_STT") {
        value = (int32_t)HONDA_CUSTOMIZE_AMN_NAV_TIMING_STT;
    } else if (val_str == "HONDA_CUSTOMIZE_AMN_HOV_ENABLE_STT") {
        value = (int32_t)HONDA_CUSTOMIZE_AMN_HOV_ENABLE_STT;
    } else if (val_str == "HONDA_CUSTOMIZE_AMN_EXPRESS_ENABLE_STT") {
        value = (int32_t)HONDA_CUSTOMIZE_AMN_EXPRESS_ENABLE_STT;
    } else if (val_str == "HONDA_CUSTOMIZE_AMN_INTERRUPT_DISP_STT") {
        value = (int32_t)HONDA_CUSTOMIZE_AMN_INTERRUPT_DISP_STT;
    } else if (val_str == "HONDA_CUSTOMIZE_LKAS_BUZZER") {
        value = (int32_t)HONDA_CUSTOMIZE_LKAS_BUZZER;
    } else if (val_str == "HONDA_CUSTOMIZE_SIF") {
        value = (int32_t)HONDA_CUSTOMIZE_SIF;
    } else if (val_str == "HONDA_CUSTOMIZE_ADAS_NOTIF_VIB_CST_STT") {
        value = (int32_t)HONDA_CUSTOMIZE_ADAS_NOTIF_VIB_CST_STT;
    } else if (val_str == "HONDA_CUSTOMIZE_ADAS_NOTIF_LED_CST_STT") {
        value = (int32_t)HONDA_CUSTOMIZE_ADAS_NOTIF_LED_CST_STT;
    } else if (val_str == "HONDA_CUSTOMIZE_INTELLIGENT_LED_CST_STT") {
        value = (int32_t)HONDA_CUSTOMIZE_INTELLIGENT_LED_CST_STT;
    } else if (val_str == "HONDA_BIOENT_CUSTOM_MENU_BIOENT_INTERFACE") {
        value = (int32_t)HONDA_BIOENT_CUSTOM_MENU_BIOENT_INTERFACE;
    } else if (val_str == "HONDA_CUSTOMIZE_WELCOME_SOUND") {
        value = (int32_t)HONDA_CUSTOMIZE_WELCOME_SOUND;
    } else if (val_str == "HONDA_CUSTOM_ADAS_E_BSI_ONOFF") {
        value = (int32_t)HONDA_CUSTOM_ADAS_E_BSI_ONOFF;
    } else if (val_str == "HONDA_CUSTOM_ADAS_E_BSI_RANGE") {
        value = (int32_t)HONDA_CUSTOM_ADAS_E_BSI_RANGE;
    } else if (val_str == "HONDA_CUSTOM_ADAS_E_BSI_2R") {
        value = (int32_t)HONDA_CUSTOM_ADAS_E_BSI_2R;
    } else if (val_str == "HONDA_CUSTOMIZE_C_LID_AUTO") {
        value = (int32_t)HONDA_CUSTOMIZE_C_LID_AUTO;
    } else if (val_str == "HONDA_CUSTOMIZE_EC_LID_CLOSE_TIMER") {
        value = (int32_t)HONDA_CUSTOMIZE_EC_LID_CLOSE_TIMER;
    } else if (val_str == "HONDA_CUSTOMIZE_EC_LID_AUTO_CLOSE") {
        value = (int32_t)HONDA_CUSTOMIZE_EC_LID_AUTO_CLOSE;
    } else if (val_str == "HONDA_CUSTOM_ADAS_STATUS_CMBS") {
        value = (int32_t)HONDA_CUSTOM_ADAS_STATUS_CMBS;
    } else if (val_str == "HONDA_CUSTOM_ADAS_FCTW") {
        value = (int32_t)HONDA_CUSTOM_ADAS_FCTW;
    } else if (val_str == "HONDA_CUSTOM_DAM_ALLOFF_SW") {
        value = (int32_t)HONDA_CUSTOM_DAM_ALLOFF_SW;
    } else if (val_str == "HONDA_CUSTOM_ADAS_AILD_ENABLE") {
        value = (int32_t)HONDA_CUSTOM_ADAS_AILD_ENABLE;
    } else if (val_str == "HONDA_CUSTOM_ADAS_ALC_ONOFF_CUST") {
        value = (int32_t)HONDA_CUSTOM_ADAS_ALC_ONOFF_CUST;
    } else if (val_str == "HONDA_CUSTOM_ADAS_E_BSI_CUST") {
        value = (int32_t)HONDA_CUSTOM_ADAS_E_BSI_CUST;
    } else if (val_str == "HONDA_CUSTOM_ADAS_SAFETY_SW_STATUS_PKS") {
        value = (int32_t)HONDA_CUSTOM_ADAS_SAFETY_SW_STATUS_PKS;
    } else if (val_str == "HONDA_CUSTOM_ADAS_SAFETY_SW_STATUS_LSAEB") {
        value = (int32_t)HONDA_CUSTOM_ADAS_SAFETY_SW_STATUS_LSAEB;
    } else if (val_str == "HONDA_CUSTOM_ADAS_PKS_RR_STATUS") {
        value = (int32_t)HONDA_CUSTOM_ADAS_PKS_RR_STATUS;
    } else if (val_str == "HONDA_CORE_CUSTOM_SNP_CRP") {
        value = (int32_t)HONDA_CORE_CUSTOM_SNP_CRP;
    } else if (val_str == "HONDA_CUSTOMIZE_RDMS_ONE") {
        value = (int32_t)HONDA_CUSTOMIZE_RDMS_ONE;
    } else if (val_str == "HONDA_CUSTOMIZE_RDMS_TWO") {
        value = (int32_t)HONDA_CUSTOMIZE_RDMS_TWO;
    } else if (val_str == "HONDA_CUSTOMIZE_EW_CUST_RESULT") {
        value = (int32_t)HONDA_CUSTOMIZE_EW_CUST_RESULT;
    } else if (val_str == "HONDA_CUSTOMIZE_DMC_DROWSY_SETTING") {
        value = (int32_t)HONDA_CUSTOMIZE_DMC_DROWSY_SETTING;
    } else if (val_str == "HONDA_CUSTOMIZE_DMC_DIST_SETTING") {
        value = (int32_t)HONDA_CUSTOMIZE_DMC_DIST_SETTING;
    } else if (val_str == "HONDA_CUSTOMIZE_AES_ONOFF_STT") {
        value = (int32_t)HONDA_CUSTOMIZE_AES_ONOFF_STT;
    } else if (val_str == "HONDA_CUSTOMIZE_DESS_CUST_STT") {
        value = (int32_t)HONDA_CUSTOMIZE_DESS_CUST_STT;
    } else if (val_str == "HONDA_CUSTOMIZE_EW_FUNC_ONOFF_STT") {
        value = (int32_t)HONDA_CUSTOMIZE_EW_FUNC_ONOFF_STT;
    } else if (val_str == "HONDA_CUSTOMIZE_EW_LV1_ONOFF_STT") {
        value = (int32_t)HONDA_CUSTOMIZE_EW_LV1_ONOFF_STT;
    } else if (val_str == "HONDA_CUSTOMIZE_EW_ELATCH_ONOFF_STT") {
        value = (int32_t)HONDA_CUSTOMIZE_EW_ELATCH_ONOFF_STT;
    } else if (val_str == "HONDA_CUSTOMIZE_EW_ELATCH_SENSE_STT") {
        value = (int32_t)HONDA_CUSTOMIZE_EW_ELATCH_SENSE_STT;
    } else if (val_str == "HONDA_CUSTOMIZE_BSINFO_VCHANGE_BCF") {
        value = (int32_t)HONDA_CUSTOMIZE_BSINFO_VCHANGE_BCF;
    } else if (val_str == "HONDA_CUSTOMIZE_ALARM_TYPE") {
        value = (int32_t)HONDA_CUSTOMIZE_ALARM_TYPE;
    } else if (val_str == "HONDA_CUSTOMIZE_BSI_BUZZER_VOLUME_STT") {
        value = (int32_t)HONDA_CUSTOMIZE_BSI_BUZZER_VOLUME_STT;
    } else if (val_str == "HONDA_CUSTOMIZE_BSI_RANGE_STT") {
        value = (int32_t)HONDA_CUSTOMIZE_BSI_RANGE_STT;
    } else if (val_str == "HONDA_CUSTOMIZE_BSI_2R_STT") {
        value = (int32_t)HONDA_CUSTOMIZE_BSI_2R_STT;
    } else if (val_str == "HONDA_CUSTOMIZE_BSI_TRAILER_STT") {
        value = (int32_t)HONDA_CUSTOMIZE_BSI_TRAILER_STT;
    } else if (val_str == "HONDA_CUSTOMIZE_BSI_TRAILER_TYPE_STT") {
        value = (int32_t)HONDA_CUSTOMIZE_BSI_TRAILER_TYPE_STT;
    } else if (val_str == "HONDA_CUSTOMIZE_BSI_TRAILER_WIDTH_STT") {
        value = (int32_t)HONDA_CUSTOMIZE_BSI_TRAILER_WIDTH_STT;
    } else if (val_str == "HONDA_CUSTOMIZE_BSI_TRAILER_LENGTH_STT") {
        value = (int32_t)HONDA_CUSTOMIZE_BSI_TRAILER_LENGTH_STT;
    } else if (val_str == "HONDA_CUSTOMIZE_PCDW_ONOFF_STT") {
        value = (int32_t)HONDA_CUSTOMIZE_PCDW_ONOFF_STT;
    } else if (val_str == "HONDA_CUSTOMIZE_ODSF_ONOFF") {
        value = (int32_t)HONDA_CUSTOMIZE_ODSF_ONOFF;
    } else if (val_str == "HONDA_CUSTOMIZE_FCM_INFO_TSR_SRBUZ_ONOFF") {
        value = (int32_t)HONDA_CUSTOMIZE_FCM_INFO_TSR_SRBUZ_ONOFF;
    } else if (val_str == "HONDA_CUSTOMIZE_DAM_EYECLOSED_STT") {
        value = (int32_t)HONDA_CUSTOMIZE_DAM_EYECLOSED_STT;
    } else if (val_str == "HONDA_METER_HUD_ILLUMI") {
        value = (int32_t)HONDA_METER_HUD_ILLUMI;
    } else if (val_str == "HONDA_CUSTOMIZE_VOL_DISPLAY_DID") {
        value = (int32_t)HONDA_CUSTOMIZE_VOL_DISPLAY_DID;
    } else if (val_str == "HONDA_CUSTOMIZE_HEADLIGHT_TIMER") {
        value = (int32_t)HONDA_CUSTOMIZE_HEADLIGHT_TIMER;
    } else if (val_str == "HONDA_CUSTOMIZE_WELCOM_LIGHT_1") {
        value = (int32_t)HONDA_CUSTOMIZE_WELCOM_LIGHT_1;
    } else if (val_str == "HONDA_CUSTOMIZE_WELCOM_LIGHT_2") {
        value = (int32_t)HONDA_CUSTOMIZE_WELCOM_LIGHT_2;
    } else if (val_str == "HONDA_CUSTOMIZE_WELCOM_LIGHT_1_2") {
        value = (int32_t)HONDA_CUSTOMIZE_WELCOM_LIGHT_1_2;
    } else if (val_str == "HONDA_EVS_BALANCE_STAT") {
        value = (int32_t)HONDA_EVS_BALANCE_STAT;
    }
    return value;
}

bool FakeVehicleHardware::writePartsConfigurationSignalDefaultValue() {
    // Create Json data to be written to the file.
    Json::Value jsonRoot(Json::objectValue);

    // Register signals and values to json data.
    for (const auto &setting : kPartsConfigurationDefaultSettings) {
        jsonRoot[setting.first] = setting.second;
    }

    return writeJsonFile(FILE_NAME_PARTS_CONFIGURATION, jsonRoot);
}

std::list<VehiclePropConfig> FakeVehicleHardware::readPartsConfigurationConfigSettings(const Json::Value jsonSignalList) {
    std::list<VehiclePropConfig> partsConfigurationConfigSettings;
    VehiclePropConfig updatePropConfig;

    auto internalPropValue = mServerSidePropStore->readValue(0, 0, 0);
    auto internalPropConfig = mServerSidePropStore->getConfig(0);

    std::list<int32_t> featureSignalsVehiclePropIds{toInt(HondaVehicleProperty::HONDA_WINDOW_STATE_ANTIPINCHI),
                                                    toInt(HondaVehicleProperty::HONDA_WINDOW_STATE_CONFLICT)};

    if (jsonSignalList["C_PW_DRAUTO_Feature"].asInt() == PW_DRAUTO_NO_EQUIPPED &&
        jsonSignalList["C_PW_FRAUTO_Feature"].asInt() == PW_FRAUTO_EQUIPPED &&
        jsonSignalList["C_PW_ALLAUTO_Feature"].asInt() == PW_ALLAUTO_NO_EQUIPPED) {
        std::list<int32_t> areaIds{WINDOW_2_LEFT, WINDOW_2_RIGHT};
        for (int32_t propId : featureSignalsVehiclePropIds) {
            internalPropConfig = mServerSidePropStore->getConfig(propId);
            if (internalPropConfig.ok()) {
                updatePropConfig = copyVPConfig(internalPropConfig.value());
                deleteAreaConfigs(updatePropConfig, areaIds);
                partsConfigurationConfigSettings.push_back(updatePropConfig);
            }
        }
    } else if (jsonSignalList["C_PW_DRAUTO_Feature"].asInt() == PW_DRAUTO_EQUIPPED &&
               jsonSignalList["C_PW_FRAUTO_Feature"].asInt() == PW_FRAUTO_NO_EQUIPPED &&
               jsonSignalList["C_PW_ALLAUTO_Feature"].asInt() == PW_ALLAUTO_NO_EQUIPPED) {
        internalPropValue = mServerSidePropStore->readValue(toInt(VehicleProperty::INFO_DRIVER_SEAT), 0, 0);
        if (internalPropValue.ok()) {
            if (internalPropValue.value()->value.int32Values[0] == SEAT_1_LEFT) {
                std::list<int32_t> areaIds{WINDOW_1_RIGHT, WINDOW_2_LEFT, WINDOW_2_RIGHT};
                for (int32_t propId : featureSignalsVehiclePropIds) {
                    internalPropConfig = mServerSidePropStore->getConfig(propId);
                    if (internalPropConfig.ok()) {
                        updatePropConfig = copyVPConfig(internalPropConfig.value());
                        deleteAreaConfigs(updatePropConfig, areaIds);
                        partsConfigurationConfigSettings.push_back(updatePropConfig);
                    }
                }
            } else if (internalPropValue.value()->value.int32Values[0] == SEAT_1_RIGHT) {
                std::list<int32_t> areaIds{WINDOW_1_LEFT, WINDOW_2_LEFT, WINDOW_2_RIGHT};
                for (int32_t propId : featureSignalsVehiclePropIds) {
                    internalPropConfig = mServerSidePropStore->getConfig(propId);
                    if (internalPropConfig.ok()) {
                        updatePropConfig = copyVPConfig(internalPropConfig.value());
                        deleteAreaConfigs(updatePropConfig, areaIds);
                        partsConfigurationConfigSettings.push_back(updatePropConfig);
                    }
                }
            }
            if (jsonSignalList["C_SR_Feature"].asInt() == 0) {
                std::list<int32_t> areaIds{WINDOW_ROOF_TOP_1};
                for (int32_t propId : featureSignalsVehiclePropIds) {
                    internalPropConfig = mServerSidePropStore->getConfig(propId);
                    if (internalPropConfig.ok()) {
                        updatePropConfig = copyVPConfig(internalPropConfig.value());
                        deleteAreaConfigs(updatePropConfig, areaIds);
                        partsConfigurationConfigSettings.push_back(updatePropConfig);
                    }
                }
            }
        }
    }

    if (jsonSignalList["C_VAR_HCS"].asInt() == VAR_HCS_HEATER) {
        internalPropConfig = mServerSidePropStore->getConfig(toInt(VehicleProperty::HVAC_SEAT_TEMPERATURE));
        if (internalPropConfig.ok()) {
            updatePropConfig = copyVPConfig(internalPropConfig.value());
            for (VehicleAreaConfig& areaConfig : updatePropConfig.areaConfigs) {
                areaConfig.minInt32Value = SEAT_TEMPERATURE_HEATED_OFF;
            }
            partsConfigurationConfigSettings.push_back(updatePropConfig);
        }

        internalPropConfig = mServerSidePropStore->getConfig(toInt(HondaVehicleProperty::HONDA_HCS_STATE_NOTIMER));
        if (internalPropConfig.ok()) {
            updatePropConfig = copyVPConfig(internalPropConfig.value());
            for (VehicleAreaConfig& areaConfig : updatePropConfig.areaConfigs) {
                areaConfig.maxInt32Value = HCS_STATE_NOTIMER_LOW_HEAT;
            }
            partsConfigurationConfigSettings.push_back(updatePropConfig);
        }
    } else if (jsonSignalList["C_VAR_HCS"].asInt() == VAR_HCS_VENTILATION) {
        internalPropConfig = mServerSidePropStore->getConfig(toInt(VehicleProperty::HVAC_SEAT_TEMPERATURE));
        if (internalPropConfig.ok()) {
            updatePropConfig = copyVPConfig(internalPropConfig.value());
            for (VehicleAreaConfig& areaConfig : updatePropConfig.areaConfigs) {
                areaConfig.maxInt32Value = SEAT_TEMPERATURE_COOLED_OFF;
            }
            partsConfigurationConfigSettings.push_back(updatePropConfig);
        }

        internalPropConfig = mServerSidePropStore->getConfig(toInt(HondaVehicleProperty::HONDA_HCS_STATE_NOTIMER));
        if (internalPropConfig.ok()) {
            updatePropConfig = copyVPConfig(internalPropConfig.value());
            for (VehicleAreaConfig& areaConfig : updatePropConfig.areaConfigs) {
                areaConfig.minInt32Value = HCS_STATE_NOTIMER_HIGH_FAN_COOLING;
            }
            partsConfigurationConfigSettings.push_back(updatePropConfig);
        }
    }

    internalPropConfig = mServerSidePropStore->getConfig(toInt(VehicleProperty::HVAC_AUTO_ON));
    if (internalPropConfig.ok()) {
        std::list<int32_t> areaIds{};
        if (jsonSignalList["C_AUTO_RR_FEATURE"].asInt() == 1) {
            areaIds.push_back(HVAC_ALL);
        } else {
            areaIds.push_back(SEAT_1_LEFT | SEAT_1_RIGHT);
            areaIds.push_back(SEAT_2_LEFT | SEAT_2_CENTER | SEAT_2_RIGHT);
        }
        updatePropConfig = copyVPConfig(internalPropConfig.value());
        deleteAreaConfigs(updatePropConfig, areaIds);
        partsConfigurationConfigSettings.push_back(updatePropConfig);
    }

    internalPropConfig = mServerSidePropStore->getConfig(toInt(VehicleProperty::HVAC_FAN_SPEED));
    if (internalPropConfig.ok()) {
        std::list<int32_t> areaIds{};
        if (jsonSignalList["C_FAN_RR_FEATURE"].asInt() == 1) {
            areaIds.push_back(HVAC_ALL);
        } else {
            areaIds.push_back(SEAT_1_LEFT | SEAT_1_RIGHT);
            areaIds.push_back(SEAT_2_LEFT | SEAT_2_CENTER | SEAT_2_RIGHT);
        }
        updatePropConfig = copyVPConfig(internalPropConfig.value());
        deleteAreaConfigs(updatePropConfig, areaIds);
        partsConfigurationConfigSettings.push_back(updatePropConfig);
    }

    internalPropConfig = mServerSidePropStore->getConfig(toInt(VehicleProperty::HVAC_FAN_DIRECTION));
    if (internalPropConfig.ok()) {
        std::list<int32_t> areaIds{};
        if (jsonSignalList["C_MODE_RR_FEATURE"].asInt() == 1) {
            areaIds.push_back(HVAC_ALL);
        } else {
            areaIds.push_back(SEAT_1_LEFT | SEAT_1_RIGHT);
            areaIds.push_back(SEAT_2_LEFT | SEAT_2_CENTER | SEAT_2_RIGHT);
        }
        updatePropConfig = copyVPConfig(internalPropConfig.value());
        deleteAreaConfigs(updatePropConfig, areaIds);
        partsConfigurationConfigSettings.push_back(updatePropConfig);
    }

    internalPropConfig = mServerSidePropStore->getConfig(toInt(VehicleProperty::HVAC_FAN_DIRECTION_AVAILABLE));
    if (internalPropConfig.ok()) {
        std::list<int32_t> areaIds{};
        if (jsonSignalList["C_MODE_RR_FEATURE"].asInt() == 1) {
            areaIds.push_back(HVAC_ALL);
        } else {
            areaIds.push_back(SEAT_1_LEFT | SEAT_1_RIGHT);
            areaIds.push_back(SEAT_2_LEFT | SEAT_2_CENTER | SEAT_2_RIGHT);
        }
        updatePropConfig = copyVPConfig(internalPropConfig.value());
        deleteAreaConfigs(updatePropConfig, areaIds);
        partsConfigurationConfigSettings.push_back(updatePropConfig);
    }

    internalPropConfig = mServerSidePropStore->getConfig(toInt(VehicleProperty::HVAC_TEMPERATURE_SET));
    if (internalPropConfig.ok()) {
        std::list<int32_t> areaIds{};
        if (jsonSignalList["C_SETTEMP_RR_FEATURE"].asInt() == 1) {
            areaIds.push_back(HVAC_LEFT);
            areaIds.push_back(HVAC_RIGHT);
            if (jsonSignalList["C_ACVARIATION"].asInt() == 1) {
                int32_t infoDriverSeatValue = FcpComm::getInstance()->getInfoDriverSeatValue();
                if (infoDriverSeatValue == FCP_VALUE_ERROR) {
                    auto result = mServerSidePropStore->readValuesForProperty(toInt(VehicleProperty::INFO_DRIVER_SEAT));
                    if (result.ok()) {
                        infoDriverSeatValue = result.value().at(0)->value.int32Values[0];
                    }
                }
                switch (infoDriverSeatValue) {
                case SEAT_1_LEFT:
                    areaIds.push_back(SEAT_1_RIGHT);
                    break;
                case SEAT_1_RIGHT:
                    areaIds.push_back(SEAT_1_LEFT);
                    break;
                default:
                    break;
                }
            }
        } else {
            areaIds.push_back(SEAT_1_LEFT);
            areaIds.push_back(SEAT_1_RIGHT);
            areaIds.push_back(SEAT_2_LEFT | SEAT_2_CENTER | SEAT_2_RIGHT);
            if (jsonSignalList["C_ACVARIATION"].asInt() == 1) {
                int32_t infoDriverSeatValue = FcpComm::getInstance()->getInfoDriverSeatValue();
                if (infoDriverSeatValue == FCP_VALUE_ERROR) {
                    auto result = mServerSidePropStore->readValuesForProperty(toInt(VehicleProperty::INFO_DRIVER_SEAT));
                    if (result.ok()) {
                        infoDriverSeatValue = result.value().at(0)->value.int32Values[0];
                    }
                }
                switch (infoDriverSeatValue) {
                case SEAT_1_LEFT:
                    areaIds.push_back(HVAC_RIGHT);
                    break;
                case SEAT_1_RIGHT:
                    areaIds.push_back(HVAC_LEFT);
                    break;
                default:
                    break;
                }
            }
        }
        if (jsonSignalList["C_TEMPVARIATION_AC"].asInt() == TEMPVARIATION_AC_CHINA) {
            updatePropConfig = copyVPConfig(internalPropConfig.value());
            deleteAreaConfigs(updatePropConfig, areaIds);
            for (VehicleAreaConfig& areaConfig : updatePropConfig.areaConfigs) {
                areaConfig.minFloatValue = TEMPVARIATION_AC_CHINA_MIN;
                areaConfig.maxFloatValue = TEMPVARIATION_AC_CHINA_MAX;
            }
            updatePropConfig.configArray.assign(TEMPVARIATION_AC_CHINA_CONFIG_ARRAY.begin(), TEMPVARIATION_AC_CHINA_CONFIG_ARRAY.end());
            partsConfigurationConfigSettings.push_back(updatePropConfig);
        } else if (jsonSignalList["C_TEMPVARIATION_AC"].asInt() == TEMPVARIATION_AC_AUTO_CLOOLER_HEATER_1 ||
                   jsonSignalList["C_TEMPVARIATION_AC"].asInt() == TEMPVARIATION_AC_AUTO_CLOOLER_HEATER_2) {
            updatePropConfig = copyVPConfig(internalPropConfig.value());
            deleteAreaConfigs(updatePropConfig, areaIds);
            for (VehicleAreaConfig& areaConfig : updatePropConfig.areaConfigs) {
                areaConfig.minFloatValue = TEMPVARIATION_AC_AUTO_CLOOLER_HEATER_MIN;
                areaConfig.maxFloatValue = TEMPVARIATION_AC_AUTO_CLOOLER_HEATER_MAX;
            }
            updatePropConfig.configArray.assign(TEMPVARIATION_AC_AUTO_CLOOLER_HEATER_CONFIG_ARRAY.begin(), TEMPVARIATION_AC_AUTO_CLOOLER_HEATER_CONFIG_ARRAY.end());
            partsConfigurationConfigSettings.push_back(updatePropConfig);
        } else if (jsonSignalList["C_TEMPVARIATION_AC"].asInt() == TEMPVARIATION_AC_KC_KE) {
            updatePropConfig = copyVPConfig(internalPropConfig.value());
            deleteAreaConfigs(updatePropConfig, areaIds);
            for (VehicleAreaConfig& areaConfig : updatePropConfig.areaConfigs) {
                areaConfig.minFloatValue = TEMPVARIATION_AC_KC_KE_MIN;
                areaConfig.maxFloatValue = TEMPVARIATION_AC_KC_KE_MAX;
            }
            updatePropConfig.configArray.assign(TEMPVARIATION_AC_KC_KE_CONFIG_ARRAY.begin(), TEMPVARIATION_AC_KC_KE_CONFIG_ARRAY.end());
            partsConfigurationConfigSettings.push_back(updatePropConfig);
        } else if (jsonSignalList["C_TEMPVARIATION_AC"].asInt() == TEMPVARIATION_AC_KA) {
            updatePropConfig = copyVPConfig(internalPropConfig.value());
            deleteAreaConfigs(updatePropConfig, areaIds);
            for (VehicleAreaConfig& areaConfig : updatePropConfig.areaConfigs) {
                areaConfig.minFloatValue = TEMPVARIATION_AC_KA_MIN;
                areaConfig.maxFloatValue = TEMPVARIATION_AC_KA_MAX;
            }
            updatePropConfig.configArray.assign(TEMPVARIATION_AC_KA_CONFIG_ARRAY.begin(), TEMPVARIATION_AC_KA_CONFIG_ARRAY.end());
            partsConfigurationConfigSettings.push_back(updatePropConfig);
        } else {
            updatePropConfig = copyVPConfig(internalPropConfig.value());
            deleteAreaConfigs(updatePropConfig, areaIds);
            partsConfigurationConfigSettings.push_back(updatePropConfig);
        }
    }

    return partsConfigurationConfigSettings;
}

std::list<VehiclePropConfig> FakeVehicleHardware::readPartsConfigurationRemoveConfigs(const Json::Value jsonSignalList) {
    std::list<VehiclePropConfig> partsConfigurationRemoveConfigs;
    VehiclePropConfig removePropConfig;

    if (jsonSignalList["C_MAXAC_AVAIL_INFO"].asInt() == MAXAC_UNAVAILABLE) {
        removePropConfig.prop = toInt(VehicleProperty::HVAC_MAX_AC_ON);
        partsConfigurationRemoveConfigs.push_back(removePropConfig);
    }

    if (jsonSignalList["C_VAR_HCS"].asInt() == VAR_HCS_HEATER) {
        removePropConfig.prop = toInt(VehicleProperty::HVAC_SEAT_VENTILATION);
        partsConfigurationRemoveConfigs.push_back(removePropConfig);
    }

    if (jsonSignalList["C_FEATURE_HSW"].asInt() == FEATURE_HSW_NO_EQUIPPED) {
        removePropConfig.prop = toInt(VehicleProperty::HVAC_STEERING_WHEEL_HEAT);
        partsConfigurationRemoveConfigs.push_back(removePropConfig);

        removePropConfig.prop = toInt(HondaVehicleProperty::HONDA_HSW_LASTINPUT);
        partsConfigurationRemoveConfigs.push_back(removePropConfig);
    }

    if (jsonSignalList["C_TEMPVARIATION_AC"].asInt() == TEMPVARIATION_AC_MANUAL) {
        removePropConfig.prop = toInt(VehicleProperty::HVAC_TEMPERATURE_DISPLAY_UNITS);
        partsConfigurationRemoveConfigs.push_back(removePropConfig);

        removePropConfig.prop = toInt(VehicleProperty::HVAC_TEMPERATURE_SET);
        partsConfigurationRemoveConfigs.push_back(removePropConfig);
    }

    if (jsonSignalList["C_SS_Feature"].asInt() == SS_FEATURE_NO_EQUIPPED) {
        removePropConfig.prop = toInt(HondaVehicleProperty::HONDA_SUNSHADE_MOVE);
        partsConfigurationRemoveConfigs.push_back(removePropConfig);

        removePropConfig.prop = toInt(HondaVehicleProperty::HONDA_SUNSHADE_POS);
        partsConfigurationRemoveConfigs.push_back(removePropConfig);

        removePropConfig.prop = toInt(HondaVehicleProperty::HONDA_SUNSHADE_STATE_ANTIPINCHI);
        partsConfigurationRemoveConfigs.push_back(removePropConfig);

        removePropConfig.prop = toInt(HondaVehicleProperty::HONDA_SUNSHADE_STATE_CONFLICT);
        partsConfigurationRemoveConfigs.push_back(removePropConfig);
    }

    if (jsonSignalList["C_SR_Feature"].asInt() == SR_FEATURE_NO_EQUIPPED) {
        removePropConfig.prop = toInt(HondaVehicleProperty::HONDA_SUNROOF_TILT_MOVE);
        partsConfigurationRemoveConfigs.push_back(removePropConfig);

        removePropConfig.prop = toInt(HondaVehicleProperty::HONDA_SUNROOF_TILT_POS);
        partsConfigurationRemoveConfigs.push_back(removePropConfig);
    }

    if (jsonSignalList["C_HUD_HPA_ILLUMI"].asInt() == HUD_HPA_ILLUMI_NO_EQUIPPED ||
        jsonSignalList["C_HUD_HPA_ILLUMI_MAX"].asInt() == HUD_HPA_ILLUMI_MAX_NO_EQUIPPED) {
        removePropConfig.prop = toInt(HondaVehicleProperty::HONDA_GET_SET_HUD_ILLUMI);
        partsConfigurationRemoveConfigs.push_back(removePropConfig);
    }

    if (jsonSignalList["C_HUD_HPA_POSI"].asInt() == HUD_HPA_POSI_NO_EQUIPPED ||
        jsonSignalList["C_HUD_HPA_POSI_MAX"].asInt() == HUD_HPA_POSI_MAX_NO_EQUIPPED) {
        removePropConfig.prop = toInt(HondaVehicleProperty::HONDA_GET_SET_HUD_POSITION);
        partsConfigurationRemoveConfigs.push_back(removePropConfig);
    }

    if (jsonSignalList["C_HUD_HPA_ROTATION"].asInt() == HUD_HPA_ROTATION_NO_EQUIPPED ||
        jsonSignalList["C_HUD_HPA_ROTATION_MAX"].asInt() == HUD_HPA_ROTATION_MAX_NO_EQUIPPED) {
        removePropConfig.prop = toInt(HondaVehicleProperty::HONDA_GET_SET_HUD_ROTATION);
        partsConfigurationRemoveConfigs.push_back(removePropConfig);
    }

    if (jsonSignalList["C_METER_MID_ILLUMI_DAY"].asInt() == METER_MID_ILLUMI_DAY_NO_EQUIPPED &&
        jsonSignalList["C_METER_MID_ILLUMI_NIGHT"].asInt() == METER_MID_ILLUMI_NIGHT_NO_EQUIPPED &&
        jsonSignalList["C_METER_MID_ILLUMI_AID"].asInt() == METER_MID_ILLUMI_AID_NO_EQUIPPED) {
        removePropConfig.prop = toInt(HondaVehicleProperty::HONDA_SET_MID_ILLUMI_SHIFT);
        partsConfigurationRemoveConfigs.push_back(removePropConfig);
    }

    if (jsonSignalList["C_METER_TB_ATTENTION_MONITOR"].asInt() == METER_TB_ATTENTION_MONITOR_NO_FUNCTION) {
        removePropConfig.prop = toInt(HondaVehicleProperty::HONDA_GET_ATTENTION_LEVEL);
        partsConfigurationRemoveConfigs.push_back(removePropConfig);
    }

    if (jsonSignalList["C_METER_TB_AVERAGE_FUEL"].asInt() == METER_TB_AVERAGE_FUEL_NO_FUNCTION ||
        jsonSignalList["C_METER_MID_TRIP_AB"].asInt() == METER_MID_TRIP_AB_NO_EQUIPPED) {
        removePropConfig.prop = toInt(HondaVehicleProperty::HONDA_GET_AVERAGE_FUEL);
        partsConfigurationRemoveConfigs.push_back(removePropConfig);
    }

    if (jsonSignalList["C_METER_TB_AVERAGE_SPEED"].asInt() == METER_TB_AVERAGE_SPEED_NO_FUNCTION ||
        jsonSignalList["C_METER_MID_TRIP_AB"].asInt() == METER_MID_TRIP_AB_NO_EQUIPPED) {
        removePropConfig.prop = toInt(HondaVehicleProperty::HONDA_GET_AVERAGE_SPEED);
        partsConfigurationRemoveConfigs.push_back(removePropConfig);
    }

    if (jsonSignalList["C_METER_TB_ELAPSED_TIME"].asInt() == METER_TB_ELAPSED_TIME_NO_FUNCTION) {
        removePropConfig.prop = toInt(HondaVehicleProperty::HONDA_GET_ELAPSED_TIME);
        partsConfigurationRemoveConfigs.push_back(removePropConfig);
    }

    if (jsonSignalList["C_METER_TB_FUEL_HISTORY"].asInt() == METER_TB_FUEL_HISTORY_NO_FUNCTION) {
        removePropConfig.prop = toInt(HondaVehicleProperty::HONDA_GET_AVERAGE_FUEL_HISTORY);
        partsConfigurationRemoveConfigs.push_back(removePropConfig);

        removePropConfig.prop = toInt(HondaVehicleProperty::HONDA_GET_AVERAGE_FUEL_HISTORY_1);
        partsConfigurationRemoveConfigs.push_back(removePropConfig);
    }

    if (jsonSignalList["C_METER_TB_MAINTENANCE"].asInt() == METER_TB_MAINTENANCE_NO_FUNCTION ||
        !(jsonSignalList["C_MAINT_INFO"].asInt() == MAINT_INFO_SMART_MAINT_1)) {
        removePropConfig.prop = toInt(HondaVehicleProperty::HONDA_GET_SMART_MAINT_US);
        partsConfigurationRemoveConfigs.push_back(removePropConfig);
    }

    if (jsonSignalList["C_METER_TB_MAINTENANCE"].asInt() == METER_TB_MAINTENANCE_NO_FUNCTION ||
        !(jsonSignalList["C_MAINT_INFO"].asInt() == MAINT_INFO_SMART_MAINT_1 ||
        jsonSignalList["C_MAINT_INFO"].asInt() == MAINT_INFO_SMART_MAINT_2 ||
        jsonSignalList["C_MAINT_INFO"].asInt() == MAINT_INFO_SMART_MAINT_5 ||
        jsonSignalList["C_MAINT_INFO"].asInt() == MAINT_INFO_SMART_MAINT_6 ||
        jsonSignalList["C_MAINT_INFO"].asInt() == MAINT_INFO_SMART_MAINT_7)) {
        removePropConfig.prop = toInt(HondaVehicleProperty::HONDA_GET_ITEM_CODE);
        partsConfigurationRemoveConfigs.push_back(removePropConfig);
    }

    if (jsonSignalList["C_RAPIDCOOL_AVAIL_INFO"].asInt() == RAPIDCOOL_AVAIL_INFO_UNAVAILABLE) {
        removePropConfig.prop = toInt(HondaVehicleProperty::HONDA_HVAC_RAPID_COOL_ON);
        partsConfigurationRemoveConfigs.push_back(removePropConfig);
    }

    return partsConfigurationRemoveConfigs;
}

std::list<VehiclePropValue> FakeVehicleHardware::readPartsConfigurationValueSettings(const Json::Value jsonSignalList) {
    std::list<VehiclePropValue> partsConfigurationVPValueSettings;
    VehiclePropValue updatePropValue;

    auto internalPropValue = mServerSidePropStore->readValue(0, 0, 0);

    if (jsonSignalList["C_MAXAC_AVAIL_INFO"].asInt() == MAXAC_UNAVAILABLE) {
        updatePropValue.prop = toInt(HondaVehicleProperty::HONDA_HVAC_MAX_COOL_FEATURE);
        updatePropValue.value.int32Values.resize(1);
        updatePropValue.value.int32Values[0] = HVAC_MAX_COOL_NO_EQUIPPED;
        partsConfigurationVPValueSettings.push_back(updatePropValue);
    } else if (jsonSignalList["C_MAXAC_AVAIL_INFO"].asInt() == MAXAC_AVAILABLE) {
        updatePropValue.prop = toInt(HondaVehicleProperty::HONDA_HVAC_MAX_COOL_FEATURE);
        updatePropValue.value.int32Values.resize(1);
        updatePropValue.value.int32Values[0] = HVAC_MAX_COOL_EQUIPPED;
        partsConfigurationVPValueSettings.push_back(updatePropValue);
    } else if (!(jsonSignalList["C_MAXAC_AVAIL_INFO"].asInt() == MAXAC_UNAVAILABLE ||
                 jsonSignalList["C_MAXAC_AVAIL_INFO"].asInt() == MAXAC_AVAILABLE)) {
        internalPropValue = mServerSidePropStore->readValue(toInt(HondaVehicleProperty::HONDA_HVAC_MAX_COOL_FEATURE));
        if (internalPropValue.ok()) {
            if (!(internalPropValue.value()->value.int32Values[0] == HVAC_MAX_COOL_NO_EQUIPPED ||
                  internalPropValue.value()->value.int32Values[0] == HVAC_MAX_COOL_EQUIPPED)) {
                updatePropValue.prop = toInt(HondaVehicleProperty::HONDA_HVAC_MAX_COOL_FEATURE);
                updatePropValue.value.int32Values.resize(1);
                updatePropValue.value.int32Values[0] = HVAC_MAX_COOL_UNCERTAIN;
                partsConfigurationVPValueSettings.push_back(updatePropValue);
            }
        }
    }
    if (jsonSignalList["C_VAR_HCS"].asInt() == VAR_HCS_HEATER) {
        updatePropValue.prop = toInt(HondaVehicleProperty::HONDA_HCS_FEATURE);
        updatePropValue.value.int32Values.resize(2);
        updatePropValue.value.int32Values[0] = HCS_FEATURE_EQUIPPED;
        updatePropValue.value.int32Values[1] = HCS_FEATURE_NO_EQUIPPED;
        partsConfigurationVPValueSettings.push_back(updatePropValue);
    } else if (jsonSignalList["C_VAR_HCS"].asInt() == VAR_HCS_VENTILATION) {
        updatePropValue.prop = toInt(HondaVehicleProperty::HONDA_HCS_FEATURE);
        updatePropValue.value.int32Values.resize(2);
        updatePropValue.value.int32Values[0] = HCS_FEATURE_NO_EQUIPPED;
        updatePropValue.value.int32Values[1] = HCS_FEATURE_EQUIPPED;
        partsConfigurationVPValueSettings.push_back(updatePropValue);
    } else if (jsonSignalList["C_VAR_HCS"].asInt() == VAR_HCS_HEATER_VENTILATION) {
        updatePropValue.prop = toInt(HondaVehicleProperty::HONDA_HCS_FEATURE);
        updatePropValue.value.int32Values.resize(2);
        updatePropValue.value.int32Values[0] = HCS_FEATURE_EQUIPPED;
        updatePropValue.value.int32Values[1] = HCS_FEATURE_EQUIPPED;
        partsConfigurationVPValueSettings.push_back(updatePropValue);
    } else if (!(jsonSignalList["C_VAR_HCS"].asInt() == VAR_HCS_HEATER ||
                 jsonSignalList["C_VAR_HCS"].asInt() == VAR_HCS_VENTILATION ||
                 jsonSignalList["C_VAR_HCS"].asInt() == VAR_HCS_HEATER_VENTILATION)) {
        internalPropValue = mServerSidePropStore->readValue(toInt(HondaVehicleProperty::HONDA_HCS_FEATURE), 0, 0);
        if (internalPropValue.ok()) {
            if (!(internalPropValue.value()->value.int32Values[0] == HCS_FEATURE_EQUIPPED ||
                 internalPropValue.value()->value.int32Values[0] == HCS_FEATURE_NO_EQUIPPED) &&
                (internalPropValue.value()->value.int32Values[1] == HCS_FEATURE_EQUIPPED ||
                 internalPropValue.value()->value.int32Values[1] == HCS_FEATURE_NO_EQUIPPED)) {
                updatePropValue.prop = toInt(HondaVehicleProperty::HONDA_HCS_FEATURE);
                updatePropValue.value.int32Values.resize(2);
                updatePropValue.value.int32Values[0] = HCS_FEATURE_UNCERTAIN;
                updatePropValue.value.int32Values[1] = HCS_FEATURE_UNCERTAIN;
                partsConfigurationVPValueSettings.push_back(updatePropValue);
            }
        }
    }
    if (jsonSignalList["C_FEATURE_HSW"].asInt() == FEATURE_HSW_NO_EQUIPPED) {
        updatePropValue.prop = toInt(HondaVehicleProperty::HONDA_HSW_FEATURE);
        updatePropValue.value.int32Values.resize(1);
        updatePropValue.value.int32Values[0] = jsonSignalList["C_FEATURE_HSW"].asInt();
        partsConfigurationVPValueSettings.push_back(updatePropValue);

        updatePropValue.value.int32Values.resize(16);
        updatePropValue.prop = toInt(HondaVehicleProperty::HONDA_HVAC_PERSONAL_ACSET);
        updatePropValue.value.int32Values[0] = 0;
        updatePropValue.value.int32Values[14] = HSW_NO_REQUEST;
        partsConfigurationVPValueSettings.push_back(updatePropValue);
    } else if (jsonSignalList["C_FEATURE_HSW"].asInt() == FEATURE_HSW_EQUIPPED ||
               jsonSignalList["C_FEATURE_HSW"].asInt() == FEATURE_HSW_UNCERTAIN ||
               jsonSignalList["C_FEATURE_HSW"].asInt() == FEATURE_HSW_HIGH_EQUIPPED) {
        updatePropValue.prop = toInt(HondaVehicleProperty::HONDA_HSW_FEATURE);
        updatePropValue.value.int32Values.resize(1);
        updatePropValue.value.int32Values[0] = jsonSignalList["C_FEATURE_HSW"].asInt();
        partsConfigurationVPValueSettings.push_back(updatePropValue);
    }

    if (jsonSignalList["C_TEMPVARIATION_AC"].asInt() == TEMPVARIATION_AC_CHINA) {
        updatePropValue.prop = toInt(VehicleProperty::HVAC_TEMPERATURE_SET);
        updatePropValue.value.int32Values.resize(0);
        updatePropValue.value.floatValues.resize(1);
        updatePropValue.value.floatValues[0] = TEMPVARIATION_AC_CHINA_INIT;
        partsConfigurationVPValueSettings.push_back(updatePropValue);
    } else if (jsonSignalList["C_TEMPVARIATION_AC"].asInt() == TEMPVARIATION_AC_AUTO_CLOOLER_HEATER_1 ||
               jsonSignalList["C_TEMPVARIATION_AC"].asInt() == TEMPVARIATION_AC_AUTO_CLOOLER_HEATER_2) {
        updatePropValue.prop = toInt(VehicleProperty::HVAC_TEMPERATURE_SET);
        updatePropValue.value.int32Values.resize(0);
        updatePropValue.value.floatValues.resize(1);
        updatePropValue.value.floatValues[0] = TEMPVARIATION_AC_AUTO_CLOOLER_HEATER_INIT;
        partsConfigurationVPValueSettings.push_back(updatePropValue);
    } else if (jsonSignalList["C_TEMPVARIATION_AC"].asInt() == TEMPVARIATION_AC_KC_KE) {
        updatePropValue.prop = toInt(VehicleProperty::HVAC_TEMPERATURE_SET);
        updatePropValue.value.int32Values.resize(0);
        updatePropValue.value.floatValues.resize(1);
        updatePropValue.value.floatValues[0] = TEMPVARIATION_AC_KC_KE_INIT;
        partsConfigurationVPValueSettings.push_back(updatePropValue);
    } else if (jsonSignalList["C_TEMPVARIATION_AC"].asInt() == TEMPVARIATION_AC_KA) {
        updatePropValue.prop = toInt(VehicleProperty::HVAC_TEMPERATURE_SET);
        updatePropValue.value.int32Values.resize(0);
        updatePropValue.value.floatValues.resize(1);
        updatePropValue.value.floatValues[0] = TEMPVARIATION_AC_KA_INIT;
        partsConfigurationVPValueSettings.push_back(updatePropValue);
    }

    if (jsonSignalList["C_TEMPVARIATION_AC"].asInt() == TEMPVARIATION_AC_CHINA) {
        updatePropValue.prop = toInt(VehicleProperty::HVAC_TEMPERATURE_VALUE_SUGGESTION);
        updatePropValue.value.int32Values.resize(0);
        updatePropValue.value.floatValues.assign(TEMPVARIATION_SUGGESTION_AC_CHINA_INIT_ARRAY.begin(), TEMPVARIATION_SUGGESTION_AC_CHINA_INIT_ARRAY.end());
        partsConfigurationVPValueSettings.push_back(updatePropValue);
    } else if (jsonSignalList["C_TEMPVARIATION_AC"].asInt() == TEMPVARIATION_AC_AUTO_CLOOLER_HEATER_1 ||
               jsonSignalList["C_TEMPVARIATION_AC"].asInt() == TEMPVARIATION_AC_AUTO_CLOOLER_HEATER_2) {
        updatePropValue.prop = toInt(VehicleProperty::HVAC_TEMPERATURE_VALUE_SUGGESTION);
        updatePropValue.value.int32Values.resize(0);
        updatePropValue.value.floatValues.assign(TEMPVARIATION_SUGGESTION_AC_AUTO_CLOOLER_HEATER_INIT_ARRAY.begin(), TEMPVARIATION_SUGGESTION_AC_AUTO_CLOOLER_HEATER_INIT_ARRAY.end());
        partsConfigurationVPValueSettings.push_back(updatePropValue);
    } else if (jsonSignalList["C_TEMPVARIATION_AC"].asInt() == TEMPVARIATION_AC_KC_KE) {
        updatePropValue.prop = toInt(VehicleProperty::HVAC_TEMPERATURE_VALUE_SUGGESTION);
        updatePropValue.value.int32Values.resize(0);
        updatePropValue.value.floatValues.assign(TEMPVARIATION_SUGGESTION_AC_KC_KE_INIT_ARRAY.begin(), TEMPVARIATION_SUGGESTION_AC_KC_KE_INIT_ARRAY.end());
        partsConfigurationVPValueSettings.push_back(updatePropValue);
    } else if (jsonSignalList["C_TEMPVARIATION_AC"].asInt() == TEMPVARIATION_AC_KA) {
        updatePropValue.prop = toInt(VehicleProperty::HVAC_TEMPERATURE_VALUE_SUGGESTION);
        updatePropValue.value.int32Values.resize(0);
        updatePropValue.value.floatValues.assign(TEMPVARIATION_SUGGESTION_AC_KA_INIT_ARRAY.begin(), TEMPVARIATION_SUGGESTION_AC_KA_INIT_ARRAY.end());
        partsConfigurationVPValueSettings.push_back(updatePropValue);
    }

    return partsConfigurationVPValueSettings;
}

VehiclePropConfig FakeVehicleHardware::copyVPConfig(const VehiclePropConfig *srcConfig) {
    VehiclePropConfig destConfig;

    destConfig.prop = srcConfig->prop;
    destConfig.changeMode = srcConfig->changeMode;
    destConfig.access = srcConfig->access;
    destConfig.areaConfigs = srcConfig->areaConfigs;
    destConfig.configArray = srcConfig->configArray;
    destConfig.configString = srcConfig->configString;
    destConfig.minSampleRate = srcConfig->minSampleRate;
    destConfig.maxSampleRate = srcConfig->maxSampleRate;
    return destConfig;
}

void FakeVehicleHardware::deleteAreaConfigs(VehiclePropConfig &config, const std::list<int32_t> tempAreaIds) {
    std::vector<VehicleAreaConfig> areaConfigs = config.areaConfigs;
    for (int32_t tempAreaId : tempAreaIds) {
        for (std::vector<VehicleAreaConfig>::iterator itr = areaConfigs.begin(); itr != areaConfigs.end(); itr++) {
            if ((*itr).areaId == tempAreaId) {
                itr = areaConfigs.erase(itr);
                break;
            }
        }
    }
    config.areaConfigs = areaConfigs;
}
}  // namespace fake
}  // namespace vehicle
}  // namespace automotive
}  // namespace hardware
}  // namespace android
