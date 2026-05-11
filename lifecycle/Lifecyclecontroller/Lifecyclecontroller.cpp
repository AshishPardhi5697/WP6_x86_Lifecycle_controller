// vendor/honda/lifecycle/Lifecyclecontroller/Lifecyclecontroller.cpp
//
// Behavior (ACK timing aligned with REPORT):
//   "On" => During Cold boot state the IVI is in WAIT_FOR_VHAL state, when send the ON request from HLD 
//            IVI goes from WAIT_FOR_VHAL to ON state.
//   "Suspend" => Deep Sleep (Suspend-to-RAM): send REQ [SHUTDOWN_PREPARE(1), SLEEP_IMMEDIATELY(4)]
//            wait for REPORT=DEEP_SLEEP_ENTRY(2) then send ACK to client,
//            then send REQ [FINISHED(3), 0] to complete
//
//   "Shutdown" => Shutdown immediate:          send REQ [SHUTDOWN_PREPARE(1), SHUTDOWN_IMMEDIATELY(1)]
//            wait for REPORT=SHUTDOWN_START(5) then send ACK to client,
//            then send REQ [FINISHED(3), 0] to complete
//   "Resume" => In Resume usecase, first the IVI goes into suspend state, then it captures the Snapshot, when we do VM shutdown
//                and then we Restore the IVI with Snapshot, it goes from WAIT_FOR_VHAL to ON state.
// Logs to logcat (logd). Subscribes to AP_POWER_STATE_REPORT and uses it to drive state.

#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>
#include <linux/vm_sockets.h>
#include <errno.h>

#include <android/binder_manager.h>
#include <android/binder_process.h>
#include <android-base/logging.h>
#include <android-base/strings.h>

#include <aidl/com/honda/hardware/automotive/vehicle/emulatormessage/IEmulatorMessageInterface.h>
#include <aidl/com/honda/hardware/automotive/vehicle/emulatormessage/BnEmulatorMessageCallback.h>

#include "VehicleHalProto.pb.h"

#include <atomic>
#include <chrono>
#include <cstring>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <vector>

using ::aidl::com::honda::hardware::automotive::vehicle::emulatormessage::BnEmulatorMessageCallback;
using ::aidl::com::honda::hardware::automotive::vehicle::emulatormessage::IEmulatorMessageInterface;

static constexpr int kListenPort = 6000;

// Properties
static constexpr int32_t kApPowerStateReqProp    = 0x11410A00;
static constexpr int32_t kApPowerStateReportProp = 0x11410A01;

// VehiclePropertyType::MASK equivalent
static constexpr int32_t kVehiclePropertyTypeMask = 0x00FF0000;

// VehicleApPowerStateReq
static constexpr int32_t REQ_ON               = 0;
static constexpr int32_t REQ_SHUTDOWN_PREPARE = 1;
static constexpr int32_t REQ_CANCEL_SHUTDOWN  = 2;
static constexpr int32_t REQ_FINISHED         = 3;

// VehicleApPowerStateShutdownParam
static constexpr int32_t PARAM_SHUTDOWN_IMMEDIATELY = 1;
static constexpr int32_t PARAM_CAN_SLEEP            = 2;
static constexpr int32_t PARAM_SHUTDOWN_ONLY        = 3;
static constexpr int32_t PARAM_SLEEP_IMMEDIATELY    = 4;

// VehicleApPowerStateReport
static constexpr int32_t RPT_WAIT_FOR_VHAL      = 1;
static constexpr int32_t RPT_DEEP_SLEEP_ENTRY   = 2;
static constexpr int32_t RPT_DEEP_SLEEP_EXIT    = 3;
static constexpr int32_t RPT_SHUTDOWN_POSTPONE  = 4;
static constexpr int32_t RPT_SHUTDOWN_START     = 5;
static constexpr int32_t RPT_ON                 = 6;
static constexpr int32_t RPT_SHUTDOWN_PREPARE   = 7;

static std::optional<std::shared_ptr<IEmulatorMessageInterface>> getEmuMsgSvc() {
    const std::string instance = std::string(IEmulatorMessageInterface::descriptor) + "/default";
    // Deprecated warning is OK; keeping for compatibility with your tree.
    AIBinder* b = AServiceManager_getService(instance.c_str());
    if (!b) {
        LOG(ERROR) << "Lifecyclecontroller: AServiceManager_getService failed for " << instance;
        return std::nullopt;
    }
    auto svc = IEmulatorMessageInterface::fromBinder(::ndk::SpAIBinder(b));
    if (!svc) {
        LOG(ERROR) << "Lifecyclecontroller: fromBinder failed for " << instance;
        return std::nullopt;
    }
    LOG(INFO) << "Lifecyclecontroller: Connected to " << instance;
    return svc;
}

// Shared state updated by callback:
static std::atomic<int32_t> g_lastReportState{-1};
static std::atomic<int32_t> g_lastReportParam{0};

class VsockEmuMsgCallback : public BnEmulatorMessageCallback {
public:
    ::ndk::ScopedAStatus onReceived(const std::vector<uint8_t>& buffer) override {
        vhal_proto::EmulatorMessage message;
        if (!message.ParseFromArray(buffer.data(), static_cast<int32_t>(buffer.size()))) {
            LOG(ERROR) << "Lifecyclecontroller: callback parse failed";
            return ndk::ScopedAStatus::ok();
        }

        if (message.msg_type() != vhal_proto::SET_PROPERTY_ASYNC || message.value_size() <= 0) {
            return ndk::ScopedAStatus::ok();
        }

        const auto& v = message.value(0);
        if (v.prop() != kApPowerStateReportProp) return ndk::ScopedAStatus::ok();

        // Report convention: int32Values[0]=state, int32Values[1]=param (if present)
        int32_t state = -1;
        int32_t param = 0;
        if (v.int32_values_size() >= 1) state = v.int32_values(0);
        if (v.int32_values_size() >= 2) param = v.int32_values(1);

        g_lastReportState.store(state, std::memory_order_release);
        g_lastReportParam.store(param, std::memory_order_release);

        // Log it (helpful for debugging)
        std::string ints;
        for (int i = 0; i < v.int32_values_size(); i++) {
            ints += std::to_string(v.int32_values(i));
            if (i + 1 < v.int32_values_size()) ints += ",";
        }
        LOG(INFO) << "Lifecyclecontroller: AP_POWER_STATE_REPORT received"
                  << " int32=[" << ints << "]"
                  << " (state=" << state << ", param=" << param << ")";
        return ndk::ScopedAStatus::ok();
    }
};

static bool registerReportCallback(const std::shared_ptr<IEmulatorMessageInterface>& svc,
                                   std::shared_ptr<VsockEmuMsgCallback>& cb) {
    if (!svc) return false;
    if (!cb) cb = ::ndk::SharedRefBase::make<VsockEmuMsgCallback>();

    auto st = svc->registerCallback(cb);
    if (!st.isOk()) {
        LOG(ERROR) << "Lifecyclecontroller: registerCallback failed: " << st.getDescription();
        return false;
    }
    LOG(INFO) << "Lifecyclecontroller: Registered callback (AP_POWER_STATE_REPORT subscription enabled)";
    return true;
}

static std::vector<uint8_t> buildEmulatorMessage_ApPowerReq(int32_t state, int32_t param) {
    vhal_proto::EmulatorMessage msg;
    vhal_proto::VehiclePropValue* protoValue = msg.add_value();

    msg.set_status(vhal_proto::RESULT_OK);
    msg.set_msg_type(vhal_proto::SET_PROPERTY_CMD);

    protoValue->set_prop(kApPowerStateReqProp);
    protoValue->set_value_type(kApPowerStateReqProp & kVehiclePropertyTypeMask);

    protoValue->set_timestamp(0);
    protoValue->set_area_id(0);
    protoValue->set_status(static_cast<vhal_proto::VehiclePropStatus>(0));

    protoValue->add_int32_values(state);
    protoValue->add_int32_values(param);

    const size_t numBytes = static_cast<size_t>(msg.ByteSizeLong());
    std::vector<uint8_t> buffer(numBytes);
    if (!msg.SerializeToArray(buffer.data(), static_cast<int32_t>(numBytes))) {
        LOG(ERROR) << "Lifecyclecontroller: SerializeToArray failed";
        return {};
    }
    return buffer;
}

static bool sendPowerReq(const std::shared_ptr<IEmulatorMessageInterface>& svc,
                         int32_t state, int32_t param) {
    auto buf = buildEmulatorMessage_ApPowerReq(state, param);
    if (buf.empty()) return false;

    auto st = svc->sendEmulatorMessage(buf);
    if (!st.isOk()) {
        LOG(ERROR) << "Lifecyclecontroller: sendEmulatorMessage failed: " << st.getDescription();
        return false;
    }
    return true;
}

static bool sendPowerReport(const std::shared_ptr<IEmulatorMessageInterface>& svc,
                            int32_t reportState, int32_t reportParam) {
    vhal_proto::EmulatorMessage msg;
    vhal_proto::VehiclePropValue* protoValue = msg.add_value();

    msg.set_status(vhal_proto::RESULT_OK);
    msg.set_msg_type(vhal_proto::SET_PROPERTY_CMD);

    protoValue->set_prop(kApPowerStateReportProp);
    protoValue->set_value_type(kApPowerStateReportProp & kVehiclePropertyTypeMask);

    protoValue->set_timestamp(0);
    protoValue->set_area_id(0);
    protoValue->set_status(static_cast<vhal_proto::VehiclePropStatus>(0));

    protoValue->add_int32_values(reportState);
    protoValue->add_int32_values(reportParam);

    const size_t numBytes = static_cast<size_t>(msg.ByteSizeLong());
    std::vector<uint8_t> buffer(numBytes);
    if (!msg.SerializeToArray(buffer.data(), static_cast<int32_t>(numBytes))) {
        LOG(ERROR) << "Lifecyclecontroller: SerializeToArray failed (REPORT)";
        return false;
    }

    auto st = svc->sendEmulatorMessage(buffer);
    if (!st.isOk()) {
        LOG(ERROR) << "Lifecyclecontroller: sendEmulatorMessage failed (REPORT): " << st.getDescription();
        return false;
    }
    return true;
}

static bool waitForReportState(int32_t expectedState,
                               std::chrono::milliseconds timeout,
                               std::chrono::milliseconds poll = std::chrono::milliseconds(25)) {
    const auto start = std::chrono::steady_clock::now();
    while (true) {
        const int32_t cur = g_lastReportState.load(std::memory_order_acquire);
        if (cur == expectedState) return true;

        if (std::chrono::steady_clock::now() - start >= timeout) {
            LOG(ERROR) << "Lifecyclecontroller: timeout waiting for REPORT state=" << expectedState
                       << ", last=" << cur;
            return false;
        }
        std::this_thread::sleep_for(poll);
    }
}

static void sendAckWithState(int fd, int32_t state) {
    char msg[32];
    int n = snprintf(msg, sizeof(msg), "ACK:%d\n", state);
    (void)send(fd, msg, (size_t)n, 0);
}

static void sendNack(int fd) {
    (void)send(fd, "NACK\n", 5, 0);
}

// IMPORTANT: handleCmd now sends ACK/NACK itself at the correct moment.
static void handleCmdAndReply(int clientFd,
                              const std::shared_ptr<IEmulatorMessageInterface>& svc,
                              const std::string& raw) {
    const std::string msg = android::base::Trim(raw);

    // Reset last report to avoid matching stale state immediately.
    // Do not reset for:
    //   "Syspend" - suspend flow can already be in/near the expected state.
    //   "Resume" - resume/restore status query must be allowed to use the
    //           latest AP_POWER_STATE_REPORT, because ON may be reported
    //           before HLB sends the RESUME request.
    if (msg != "Suspend" && msg != "Resume") {
        g_lastReportState.store(-1, std::memory_order_release);
        g_lastReportParam.store(0, std::memory_order_release);
    }

    if (!svc) {
        LOG(ERROR) << "Lifecyclecontroller: svc null";
        sendNack(clientFd);
        return;
    }

    if (msg == "Suspend") {
        LOG(INFO) << "Lifecyclecontroller: CMD Suspend => DEEP_SLEEP (Suspend)";

        // Request deep sleep without postponing
        if (!sendPowerReq(svc, REQ_SHUTDOWN_PREPARE, PARAM_SLEEP_IMMEDIATELY)) {
            sendNack(clientFd);
            return;
        }

        // Wait for AP to report DEEP_SLEEP_ENTRY (2)
        if (!waitForReportState(RPT_DEEP_SLEEP_ENTRY, std::chrono::seconds(20))) {
            sendNack(clientFd);
            return;
        }

        sendAckWithState(clientFd, RPT_DEEP_SLEEP_ENTRY); // 2

        return;
    }

    if (msg == "Shutdown") {
        LOG(INFO) << "Lifecyclecontroller: CMD Shutdown => SHUTDOWN_IMMEDIATE";

        if (!sendPowerReq(svc, REQ_SHUTDOWN_PREPARE, PARAM_SHUTDOWN_IMMEDIATELY)) {
            sendNack(clientFd);
            return;
        }

        // Wait for AP to report SHUTDOWN_START (5)
        if (!waitForReportState(RPT_SHUTDOWN_START, std::chrono::seconds(25))) {
            sendNack(clientFd);
            return;
        }

        sendAckWithState(clientFd, RPT_SHUTDOWN_START); // 5

        if (!sendPowerReq(svc, REQ_FINISHED, 0)) {
            LOG(ERROR) << "Lifecyclecontroller: FINISHED send failed after ACK (Shutdown)";
        }
        return;
    }
    
    // For Snapshot Resume / Restore
    // HLB/mock_mcu sends "RESUME" after doRestore_IVI() succeeds.
    // Do not send REQ_ON here. In snapshot resume, CPMS/CarPowerManagementService
    // should already come back to ON by itself. This command only verifies the
    // latest AP_POWER_STATE_REPORT and replies ACK:6 when ON is observed.
    if (msg == "Resume") {
        LOG(INFO) << "Lifecyclecontroller: CMD Resume => RESUME_STATUS_CHECK (wait for ON)";

        const int32_t cur = g_lastReportState.load(std::memory_order_acquire);
        LOG(INFO) << "Lifecyclecontroller: CMD Resume current AP_POWER_STATE_REPORT=" << cur
                  << ", expected ON=" << RPT_ON;

        if (cur != RPT_ON) {
            if (!waitForReportState(RPT_ON, std::chrono::seconds(60))) {
                LOG(ERROR) << "Lifecyclecontroller: CMD Resume resume check failed; ON not observed";
                sendNack(clientFd);
                return;
            }
        }

        LOG(INFO) << "Lifecyclecontroller: CMD Resume resume check success; sending ACK:6";
        sendAckWithState(clientFd, RPT_ON); // 6
        return;
    }

    //For On Usecase
        if (msg == "On") {
        LOG(INFO) << "Lifecyclecontroller: CMD On => COLD_START_RELEASE (WAIT_FOR_VHAL -> ON)";

        // We expect the VM to already be in WAIT_FOR_VHAL.
        const int32_t cur = g_lastReportState.load(std::memory_order_acquire);
        if (cur != RPT_WAIT_FOR_VHAL) {
            LOG(WARNING) << "Lifecyclecontroller: CMD On received but last REPORT is not WAIT_FOR_VHAL, last="
                         << cur;
        }

        // Send AP_POWER_STATE_REQ = ON
        if (!sendPowerReq(svc, REQ_ON, 0)) {
            sendNack(clientFd);
            return;
        }

        // Wait until AP reports ON
        if (!waitForReportState(RPT_ON, std::chrono::seconds(20))) {
            sendNack(clientFd);
            return;
        }

        // ACK only after ON is confirmed
        sendAckWithState(clientFd, RPT_ON); // 6
        return;
    }
    
}

static int createVsockListenSocket() {
    int fd = socket(AF_VSOCK, SOCK_STREAM, 0);
    if (fd < 0) {
        LOG(ERROR) << "Lifecyclecontroller: socket(AF_VSOCK) failed errno=" << errno
                   << " (" << strerror(errno) << ")";
        return -1;
    }

    sockaddr_vm sa{};
    sa.svm_family = AF_VSOCK;
    sa.svm_port = static_cast<uint32_t>(kListenPort);
    sa.svm_cid = VMADDR_CID_ANY;

    if (bind(fd, reinterpret_cast<sockaddr*>(&sa), sizeof(sa)) < 0) {
        LOG(ERROR) << "Lifecyclecontroller: bind(vsock) failed errno=" << errno
                   << " (" << strerror(errno) << ")";
        close(fd);
        return -1;
    }
    if (listen(fd, 8) < 0) {
        LOG(ERROR) << "Lifecyclecontroller: listen(vsock) failed errno=" << errno
                   << " (" << strerror(errno) << ")";
        close(fd);
        return -1;
    }
    return fd;
}

static std::string recvOnce(int fd) {
    char buf[256];
    memset(buf, 0, sizeof(buf));
    const ssize_t n = recv(fd, buf, sizeof(buf) - 1, 0);
    if (n <= 0) return "";
    std::string s(buf, buf + n);
    while (!s.empty() && (s.back() == '\n' || s.back() == '\r')) s.pop_back();
    return s;
}

int main(int /*argc*/, char** argv) {
    android::base::SetLogger(android::base::LogdLogger());
    android::base::InitLogging(argv);

    ABinderProcess_setThreadPoolMaxThreadCount(2);
    ABinderProcess_startThreadPool();

    int listenFd = createVsockListenSocket();
    if (listenFd < 0) return 1;

    LOG(INFO) << "Lifecyclecontroller: listening on VSOCK port " << kListenPort;

    std::shared_ptr<IEmulatorMessageInterface> svc;
    std::shared_ptr<VsockEmuMsgCallback> cb;  // keep alive

    // Connect and subscribe
    if (auto s = getEmuMsgSvc(); s) {
        svc = *s;
        (void)registerReportCallback(svc, cb);
    } else {
        LOG(WARNING) << "Lifecyclecontroller: initial connect failed; will retry on first request";
    }

    while (true) {
        int c = accept(listenFd, nullptr, nullptr);
        if (c < 0) continue;

        std::string msg = recvOnce(c);
        if (msg.empty()) {
            sendNack(c);
            close(c);
            continue;
        }

        // Ensure svc exists
        if (!svc) {
            if (auto s = getEmuMsgSvc(); s) {
                svc = *s;
                (void)registerReportCallback(svc, cb);
            }
        }

        if (!svc) {
            sendNack(c);
            close(c);
            continue;
        }

        // Handle command and send ACK/NACK at the right time
        handleCmdAndReply(c, svc, msg);

        close(c);
    }

    return 0;
}
