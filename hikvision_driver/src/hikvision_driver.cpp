#include "hikvision_driver/hikvision_driver.hpp"

#include <algorithm>
#include <atomic>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>

#include <camera_info_manager/camera_info_manager.hpp>
#include <hikvision_interface/msg/hik_image_info.hpp>
#include <hikvision_interface/srv/trigger_software_stamped.hpp>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include <MvCameraControl.h>

#define MV_CHECK_THROW(logger, func, ...)                                          \
    do {                                                                           \
        int nRet = func(__VA_ARGS__);                                              \
        if (MV_OK != nRet) {                                                       \
            RCLCPP_ERROR(logger, "hikvision sdk error: " #func " = 0x%X", nRet);  \
            throw std::runtime_error("Hikvision SDK init error: " #func);          \
        }                                                                          \
    } while (0)

#define MV_CHECK_WARN(logger, func, ...)                                           \
    do {                                                                           \
        int nRet = func(__VA_ARGS__);                                              \
        if (MV_OK != nRet) {                                                       \
            RCLCPP_WARN(logger, "hikvision sdk warning: " #func " = 0x%X", nRet); \
        }                                                                          \
    } while (0)

using hikvision_interface::msg::HikImageInfo;
using hikvision_interface::srv::TriggerSoftwareStamped;

namespace {

builtin_interfaces::msg::Time ToBuiltinTime(uint64_t ns) {
    builtin_interfaces::msg::Time stamp;
    stamp.sec = static_cast<int32_t>(ns / 1000000000ull);
    stamp.nanosec = static_cast<uint32_t>(ns % 1000000000ull);
    return stamp;
}

uint64_t ScaleTicksToNs(uint64_t ticks, uint64_t tick_frequency) {
    if (tick_frequency == 0) {
        return 0;
    }
    __int128 scaled = static_cast<__int128>(ticks) * 1000000000ll;
    return static_cast<uint64_t>(scaled / tick_frequency);
}

const char *PtpStatusToString(unsigned int status) {
    switch (status) {
        case 0: return "Initializing";
        case 1: return "Faulty";
        case 2: return "Disabled";
        case 3: return "Listening";
        case 4: return "PreMaster";
        case 5: return "Master";
        case 6: return "Passive";
        case 7: return "Uncalibrated";
        case 8: return "Slave";
        default: return "Unknown";
    }
}

bool IsPtpLocked(unsigned int status) {
    return status == 8 || status == 5;
}

}  // namespace

namespace hikvision_driver {

struct HikvisionDriver::Impl {
    struct PendingTrigger {
        uint64_t trigger_seq = 0;
        uint64_t t_app_issue_ns = 0;
        uint64_t t_drv_before_ns = 0;
        uint64_t t_drv_after_ns = 0;
    };

    std::unique_ptr<rclcpp::Logger> logger;

    void *handle = nullptr;
    std::string camera_name;
    bool software_trigger_enabled = true;
    image_transport::Publisher img_pub;
    std::shared_ptr<rclcpp::Publisher<HikImageInfo>> p_info_pub;
    static void image_callback_ex(unsigned char *pData, MV_FRAME_OUT_INFO_EX *pFrameInfo, void *pUser);

    rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr param_callback_handle;

    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub;
    std::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_manager;
    std::string frame_id;

    rclcpp::Service<TriggerSoftwareStamped>::SharedPtr trigger_service_;

    std::mutex trigger_mtx_;
    std::deque<PendingTrigger> pending_triggers_;
    uint64_t next_trigger_seq_ = 1;

    std::atomic<unsigned int> ptp_status_{2};
    std::atomic<uint64_t> timestamp_tick_frequency_{0};
    std::atomic<uint32_t> frame_counter_{0};
    // Two-point frequency calibration
    std::atomic<int64_t> ref_dev_ticks_{0};
    std::atomic<int64_t> ref_host_ns_{0};
    std::atomic<uint64_t> calibrated_tick_freq_{0};
    std::atomic<bool> freq_calibrated_{false};

    bool query_ptp_status(unsigned int &status) const {
        MVCC_ENUMVALUE value;
        std::memset(&value, 0, sizeof(value));
        int ret = MV_CC_GetEnumValue(handle, "GevIEEE1588Status", &value);
        if (ret != MV_OK) {
            return false;
        }
        status = value.nCurValue;
        return true;
    }

    bool query_timestamp_tick_frequency(uint64_t &tick_frequency) const {
        MVCC_INTVALUE_EX value;
        std::memset(&value, 0, sizeof(value));
        int ret = MV_CC_GetIntValueEx(handle, "GevTimestampTickFrequency", &value);
        if (ret != MV_OK || value.nCurValue <= 0) {
            return false;
        }
        tick_frequency = static_cast<uint64_t>(value.nCurValue);
        return true;
    }

    void refresh_ptp_state(const rclcpp::Logger &node_logger, bool force_log = false) {
        unsigned int queried_status = ptp_status_.load();
        if (query_ptp_status(queried_status)) {
            unsigned int previous = ptp_status_.exchange(queried_status);
            if (force_log || previous != queried_status) {
                RCLCPP_INFO(node_logger, "PTP status = %s (%u)", PtpStatusToString(queried_status), queried_status);
            }
        } else if (force_log) {
            RCLCPP_WARN(node_logger, "Failed to query GevIEEE1588Status; PTP lock state unknown");
        }

        uint64_t tick_frequency = 0;
        if (query_timestamp_tick_frequency(tick_frequency)) {
            uint64_t previous = timestamp_tick_frequency_.exchange(tick_frequency);
            if (force_log || previous != tick_frequency) {
                RCLCPP_INFO(node_logger, "Camera timestamp tick frequency = %lu Hz", tick_frequency);
            }
        } else if (force_log) {
            RCLCPP_WARN(node_logger, "Failed to query GevTimestampTickFrequency");
        }
    }
};

void HikvisionDriver::Impl::image_callback_ex(unsigned char *pData, MV_FRAME_OUT_INFO_EX *pFrameInfo, void *pUser) {
    auto node = reinterpret_cast<HikvisionDriver *>(pUser);
    auto &impl = *node->pImpl;

    uint32_t frame_counter = impl.frame_counter_.fetch_add(1) + 1;
    if (frame_counter == 1 || frame_counter % 60 == 0) {
        impl.refresh_ptp_state(node->get_logger(), frame_counter == 1);
    }

    const uint64_t dev_stamp_ticks = (static_cast<uint64_t>(pFrameInfo->nDevTimeStampHigh) << 32ull) |
                                     static_cast<uint64_t>(pFrameInfo->nDevTimeStampLow);
    const uint64_t host_stamp = static_cast<uint64_t>(pFrameInfo->nHostTimeStamp);
    const uint64_t tick_frequency = impl.timestamp_tick_frequency_.load();
    const unsigned int ptp_status = impl.ptp_status_.load();
    const bool ptp_locked = IsPtpLocked(ptp_status);

    uint64_t exposure_ns = 0;
    bool ptp_time_valid = false;
    if (ptp_locked && tick_frequency > 0 && dev_stamp_ticks > 0) {
        if (!impl.freq_calibrated_.load()) {
            // Two-point frequency calibration: camera-reported tick_frequency may be wrong.
            // Measure the real frequency from the ratio of camera ticks to host time.
            int64_t host_ns = static_cast<int64_t>(host_stamp) * 1000000ll;
            if (impl.ref_dev_ticks_.load() == 0) {
                impl.ref_dev_ticks_.store(static_cast<int64_t>(dev_stamp_ticks));
                impl.ref_host_ns_.store(host_ns);
            } else {
                int64_t delta_ticks = static_cast<int64_t>(dev_stamp_ticks) - impl.ref_dev_ticks_.load();
                int64_t delta_host_ns = host_ns - impl.ref_host_ns_.load();
                if (delta_host_ns > 5'000'000'000ll) {  // ≥5s baseline for <0.04% freq error
                    uint64_t cal_freq = static_cast<uint64_t>(
                        static_cast<double>(delta_ticks) * 1'000'000'000.0 / delta_host_ns);
                    impl.calibrated_tick_freq_.store(cal_freq);
                    impl.freq_calibrated_.store(true);
                    RCLCPP_INFO(node->get_logger(),
                        "PTP freq calibrated: %lu Hz (reported %lu Hz, err %.1f%%)",
                        cal_freq, tick_frequency,
                        (static_cast<double>(cal_freq) / tick_frequency - 1.0) * 100.0);
                }
            }
        }
        if (impl.freq_calibrated_.load()) {
            int64_t ref_ticks = impl.ref_dev_ticks_.load();
            int64_t ref_host = impl.ref_host_ns_.load();
            uint64_t freq = impl.calibrated_tick_freq_.load();
            __int128 scaled = static_cast<__int128>(static_cast<int64_t>(dev_stamp_ticks) - ref_ticks) * 1'000'000'000ll;
            int64_t raw_ns = ref_host + static_cast<int64_t>(scaled / freq);
            exposure_ns = static_cast<uint64_t>(raw_ns);
            ptp_time_valid = true;
        }
    }

    Impl::PendingTrigger matched_trigger;
    bool has_matched_trigger = false;
    {
        std::lock_guard<std::mutex> lk(impl.trigger_mtx_);
        if (!impl.pending_triggers_.empty()) {
            matched_trigger = impl.pending_triggers_.front();
            impl.pending_triggers_.pop_front();
            has_matched_trigger = true;
        }
    }

    int64_t delta_ctrl_ns = 0;
    int64_t delta_drv_ns = 0;
    if (has_matched_trigger && ptp_time_valid) {
        delta_ctrl_ns = static_cast<int64_t>(exposure_ns) - static_cast<int64_t>(matched_trigger.t_app_issue_ns);
        delta_drv_ns = static_cast<int64_t>(exposure_ns) - static_cast<int64_t>(matched_trigger.t_drv_before_ns);
    }

    auto p_img_msg = std::make_unique<sensor_msgs::msg::Image>();
    if (pFrameInfo->nFrameLen > p_img_msg->data.max_size()) {
        RCLCPP_ERROR_ONCE(node->get_logger(), "image bytes exceed max available size");
        return;
    }
    p_img_msg->header.frame_id = impl.frame_id;
    p_img_msg->header.stamp.nanosec = static_cast<uint32_t>(host_stamp % 1000ull) * 1000000ull;
    p_img_msg->header.stamp.sec = static_cast<int32_t>(host_stamp / 1000ull);
    p_img_msg->is_bigendian = false;
    p_img_msg->width = pFrameInfo->nWidth;
    p_img_msg->height = pFrameInfo->nHeight;
    if (pFrameInfo->enPixelType == PixelType_Gvsp_BayerRG8) {
        p_img_msg->step = pFrameInfo->nWidth * 1;
        p_img_msg->encoding = sensor_msgs::image_encodings::BAYER_RGGB8;
    } else if (pFrameInfo->enPixelType == PixelType_Gvsp_BayerBG8) {
        p_img_msg->step = pFrameInfo->nWidth * 1;
        p_img_msg->encoding = sensor_msgs::image_encodings::BAYER_BGGR8;
    } else if (pFrameInfo->enPixelType == PixelType_Gvsp_BayerGR8) {
        p_img_msg->step = pFrameInfo->nWidth * 1;
        p_img_msg->encoding = sensor_msgs::image_encodings::BAYER_GRBG8;
    } else if (pFrameInfo->enPixelType == PixelType_Gvsp_BayerGB8) {
        p_img_msg->step = pFrameInfo->nWidth * 1;
        p_img_msg->encoding = sensor_msgs::image_encodings::BAYER_GBRG8;
    } else if (pFrameInfo->enPixelType == PixelType_Gvsp_Mono8) {
        p_img_msg->step = pFrameInfo->nWidth * 1;
        p_img_msg->encoding = sensor_msgs::image_encodings::MONO8;
    } else if (pFrameInfo->enPixelType == PixelType_Gvsp_RGB8_Packed) {
        p_img_msg->step = pFrameInfo->nWidth * 3;
        p_img_msg->encoding = sensor_msgs::image_encodings::RGB8;
    } else if (pFrameInfo->enPixelType == PixelType_Gvsp_BGR8_Packed) {
        p_img_msg->step = pFrameInfo->nWidth * 3;
        p_img_msg->encoding = sensor_msgs::image_encodings::BGR8;
    } else {
        RCLCPP_ERROR_ONCE(node->get_logger(), "unsupport pixel format: %d", static_cast<int>(pFrameInfo->enPixelType));
        return;
    }

    if (pFrameInfo->nFrameLen < (p_img_msg->height * p_img_msg->step)) {
        RCLCPP_ERROR(node->get_logger(), "nFrameLen < required data size, len=%d", pFrameInfo->nFrameLen);
        return;
    }
    p_img_msg->data.resize(p_img_msg->height * p_img_msg->step);
    std::copy_n(pData, p_img_msg->data.size(), p_img_msg->data.data());

    auto p_info_msg = std::make_unique<HikImageInfo>();
    p_info_msg->header.frame_id = impl.camera_name;
    p_info_msg->header.stamp = p_img_msg->header.stamp;
    p_info_msg->dev_stamp = ptp_time_valid ? ToBuiltinTime(exposure_ns) : builtin_interfaces::msg::Time();
    p_info_msg->frame_num = pFrameInfo->nFrameNum;
    p_info_msg->gain = pFrameInfo->fGain;
    p_info_msg->exposure = pFrameInfo->fExposureTime;
    p_info_msg->red = pFrameInfo->nRed;
    p_info_msg->green = pFrameInfo->nGreen;
    p_info_msg->blue = pFrameInfo->nBlue;
    p_info_msg->matched_trigger = has_matched_trigger;
    p_info_msg->ptp_locked = ptp_locked;
    p_info_msg->ptp_time_valid = ptp_time_valid;
    p_info_msg->trigger_seq = has_matched_trigger ? matched_trigger.trigger_seq : 0;
    p_info_msg->trigger_index = pFrameInfo->nTriggerIndex;
    p_info_msg->t_app_issue = has_matched_trigger ? ToBuiltinTime(matched_trigger.t_app_issue_ns) : builtin_interfaces::msg::Time();
    p_info_msg->t_drv_before = has_matched_trigger ? ToBuiltinTime(matched_trigger.t_drv_before_ns) : builtin_interfaces::msg::Time();
    p_info_msg->t_drv_after = has_matched_trigger ? ToBuiltinTime(matched_trigger.t_drv_after_ns) : builtin_interfaces::msg::Time();
    p_info_msg->delta_ctrl_ns = delta_ctrl_ns;
    p_info_msg->delta_drv_ns = delta_drv_ns;
    p_info_msg->dev_timestamp_ticks = dev_stamp_ticks;
    p_info_msg->timestamp_tick_frequency = tick_frequency;
    p_info_msg->chunk_second_count = pFrameInfo->nSecondCount;
    p_info_msg->chunk_cycle_count = pFrameInfo->nCycleCount;
    p_info_msg->chunk_cycle_offset = pFrameInfo->nCycleOffset;

    if (has_matched_trigger) {
        if (ptp_time_valid) {
            RCLCPP_INFO(node->get_logger(),
                        "[MATCH] seq=%lu frame=%u trig_idx=%u Δctrl=%.3f ms Δdrv=%.3f ms ptp=%s",
                        matched_trigger.trigger_seq,
                        pFrameInfo->nFrameNum,
                        pFrameInfo->nTriggerIndex,
                        delta_ctrl_ns / 1e6,
                        delta_drv_ns / 1e6,
                        PtpStatusToString(ptp_status));
        } else {
            RCLCPP_WARN(node->get_logger(),
                        "[MATCH] seq=%lu frame=%u matched but PTP timestamp invalid (status=%s, tick_freq=%lu)",
                        matched_trigger.trigger_seq,
                        pFrameInfo->nFrameNum,
                        PtpStatusToString(ptp_status),
                        tick_frequency);
        }
    }

    impl.p_info_pub->publish(std::move(p_info_msg));

    auto cam_info_msg = std::make_unique<sensor_msgs::msg::CameraInfo>(impl.cinfo_manager->getCameraInfo());
    cam_info_msg->header = p_img_msg->header;
    impl.camera_info_pub->publish(std::move(cam_info_msg));

    impl.img_pub.publish(std::move(p_img_msg));
}

HikvisionDriver::HikvisionDriver(const rclcpp::NodeOptions &options)
    : rclcpp::Node("hikvision_driver_node", options), pImpl(std::make_unique<Impl>()) {
    auto logger = get_logger();
    pImpl->logger = std::make_unique<rclcpp::Logger>(logger);

    declare_parameter<std::string>("camera_name", "");
    pImpl->camera_name = get_parameter("camera_name").as_string();
    if (pImpl->camera_name.empty()) {
        RCLCPP_ERROR(logger, "Parameter 'camera_name' is empty! You must specify a valid camera name.");
        throw std::runtime_error("Parameter 'camera_name' is missing.");
    }
    RCLCPP_INFO(logger, "trying to open camera: '%s'", pImpl->camera_name.c_str());

    declare_parameter<std::string>("frame_id", pImpl->camera_name);
    pImpl->frame_id = get_parameter("frame_id").as_string();
    declare_parameter<std::string>("camera_info_url", "");

    declare_parameter<double>("exposure_time", 20000.0);
    declare_parameter<double>("gain", 15.0);
    declare_parameter<std::string>("pixel_format", "RGB8");
    declare_parameter<std::string>("trigger_mode", "software");

    auto qos = rclcpp::SensorDataQoS().reliable();
    pImpl->img_pub = image_transport::create_publisher(this, "image_raw", qos.get_rmw_qos_profile());
    pImpl->p_info_pub = create_publisher<HikImageInfo>("info", qos);
    pImpl->camera_info_pub = create_publisher<sensor_msgs::msg::CameraInfo>("camera_info", qos);
    pImpl->cinfo_manager = std::make_shared<camera_info_manager::CameraInfoManager>(
        this, pImpl->camera_name, get_parameter("camera_info_url").as_string());

    MV_CC_DEVICE_INFO_LIST stDeviceList;
    std::memset(&stDeviceList, 0, sizeof(stDeviceList));
    MV_CHECK_THROW(logger, MV_CC_EnumDevices, MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);

    for (uint32_t nDeviceId = 0; nDeviceId < stDeviceList.nDeviceNum; nDeviceId++) {
        auto *pDeviceInfo = stDeviceList.pDeviceInfo[nDeviceId];
        const char *pUserDefinedName = nullptr;
        if (pDeviceInfo->nTLayerType == MV_GIGE_DEVICE) {
            pUserDefinedName = reinterpret_cast<const char *>(pDeviceInfo->SpecialInfo.stGigEInfo.chUserDefinedName);
            if (pUserDefinedName == pImpl->camera_name) {
                int nIp1 = ((pDeviceInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0xff000000) >> 24);
                int nIp2 = ((pDeviceInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x00ff0000) >> 16);
                int nIp3 = ((pDeviceInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x0000ff00) >> 8);
                int nIp4 = (pDeviceInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x000000ff);
                RCLCPP_INFO(logger, "[%s]: GIGE, %s, %d.%d.%d.%d", pUserDefinedName,
                            pDeviceInfo->SpecialInfo.stGigEInfo.chModelName, nIp1, nIp2, nIp3, nIp4);
            }
        } else if (pDeviceInfo->nTLayerType == MV_USB_DEVICE) {
            pUserDefinedName = reinterpret_cast<const char *>(pDeviceInfo->SpecialInfo.stUsb3VInfo.chUserDefinedName);
            if (pUserDefinedName == pImpl->camera_name) {
                RCLCPP_INFO(logger, "[%s]: USB, %s", pUserDefinedName,
                            pDeviceInfo->SpecialInfo.stUsb3VInfo.chModelName);
            }
        } else {
            RCLCPP_WARN(logger, "type(%d) not support", pDeviceInfo->nTLayerType);
        }

        if (pUserDefinedName != pImpl->camera_name) {
            continue;
        }

        MV_CHECK_THROW(logger, MV_CC_CreateHandle, &pImpl->handle, pDeviceInfo);
        MV_CHECK_THROW(logger, MV_CC_OpenDevice, pImpl->handle);

        {
            int nRet = MV_CC_SetBoolValue(pImpl->handle, "GevIEEE1588", true);
            if (MV_OK == nRet) {
                RCLCPP_INFO(logger, "GevIEEE1588 (PTP) enabled");
            } else {
                RCLCPP_WARN(logger, "GevIEEE1588 not supported or failed (0x%X)", nRet);
            }
        }

        {
            int nRet = MV_CC_SetBoolValue(pImpl->handle, "ChunkModeActive", true);
            if (MV_OK != nRet) {
                RCLCPP_WARN(logger, "ChunkModeActive failed (0x%X) — camera may not support ChunkData", nRet);
            } else {
                MV_CC_SetEnumValueByString(pImpl->handle, "ChunkSelector", "Exposure");
                MV_CC_SetBoolValue(pImpl->handle, "ChunkEnable", true);
                MV_CC_SetEnumValueByString(pImpl->handle, "ChunkSelector", "Timestamp");
                nRet = MV_CC_SetBoolValue(pImpl->handle, "ChunkEnable", true);
                if (MV_OK == nRet) {
                    RCLCPP_INFO(logger, "ChunkData enabled: Exposure + Timestamp chunks active");
                } else {
                    RCLCPP_WARN(logger, "ChunkData Timestamp enable failed (0x%X)", nRet);
                }
            }
        }

        std::string pixel_format = get_parameter("pixel_format").as_string();
        if (pixel_format != "Keep") {
            static const std::map<std::string, MvGvspPixelType> kPixelFormatMap = {
                {"Mono8", PixelType_Gvsp_Mono8},       {"RGB8", PixelType_Gvsp_RGB8_Packed},
                {"BGR8", PixelType_Gvsp_BGR8_Packed},  {"BayerRG8", PixelType_Gvsp_BayerRG8},
                {"BayerBG8", PixelType_Gvsp_BayerBG8}, {"BayerGR8", PixelType_Gvsp_BayerGR8},
                {"BayerGB8", PixelType_Gvsp_BayerGB8},
            };
            auto it = kPixelFormatMap.find(pixel_format);
            if (it == kPixelFormatMap.end()) {
                RCLCPP_WARN(logger, "unknown pixel_format '%s', keeping camera default", pixel_format.c_str());
            } else {
                MV_CHECK_THROW(logger, MV_CC_SetEnumValue, pImpl->handle, "PixelFormat", it->second);
            }
        }

        double init_exposure = get_parameter("exposure_time").as_double();
        double init_gain = get_parameter("gain").as_double();
        MV_CHECK_THROW(logger, MV_CC_SetEnumValue, pImpl->handle, "ExposureAuto", 0);
        MV_CHECK_THROW(logger, MV_CC_SetFloatValue, pImpl->handle, "ExposureTime", static_cast<float>(init_exposure));
        MV_CHECK_THROW(logger, MV_CC_SetFloatValue, pImpl->handle, "Gain", static_cast<float>(init_gain));
        MV_CHECK_THROW(logger, MV_CC_SetEnumValue, pImpl->handle, "BalanceWhiteAuto", 2);
        MV_CHECK_WARN(logger, MV_CC_SetEnumValueByString, pImpl->handle, "AcquisitionMode", "Continuous");

        std::string trigger_mode = get_parameter("trigger_mode").as_string();
        if (trigger_mode == "software") {
            pImpl->software_trigger_enabled = true;
            MV_CHECK_THROW(logger, MV_CC_SetEnumValue, pImpl->handle, "TriggerMode", 1);
            MV_CHECK_THROW(logger, MV_CC_SetEnumValue, pImpl->handle, "TriggerSource", 7);
            RCLCPP_INFO(logger, "Trigger mode: software (TriggerMode=On, TriggerSource=Software)");
        } else if (trigger_mode == "continuous") {
            pImpl->software_trigger_enabled = false;
            MV_CHECK_THROW(logger, MV_CC_SetEnumValue, pImpl->handle, "TriggerMode", 0);
            RCLCPP_INFO(logger, "Trigger mode: continuous (TriggerMode=Off)");
        } else {
            RCLCPP_ERROR(logger, "Invalid trigger_mode '%s'. Use 'software' or 'continuous'.", trigger_mode.c_str());
            throw std::runtime_error("Invalid trigger_mode parameter");
        }

        pImpl->refresh_ptp_state(logger, true);

        pImpl->trigger_service_ = this->create_service<TriggerSoftwareStamped>(
            "trigger_software",
            [this](const std::shared_ptr<TriggerSoftwareStamped::Request> req,
                   std::shared_ptr<TriggerSoftwareStamped::Response> res) {
                if (!pImpl->software_trigger_enabled) {
                    res->success = false;
                    res->message = "trigger_mode is continuous; software trigger is disabled";
                    res->trigger_seq = 0;
                    res->t_drv_before_ns = 0;
                    res->t_drv_after_ns = 0;
                    RCLCPP_WARN(this->get_logger(), "Ignoring TriggerSoftware request while trigger_mode=continuous");
                    return;
                }

                uint64_t t_drv_before_ns = static_cast<uint64_t>(this->now().nanoseconds());
                int nRet = MV_CC_SetCommandValue(pImpl->handle, "TriggerSoftware");
                uint64_t t_drv_after_ns = static_cast<uint64_t>(this->now().nanoseconds());

                res->t_drv_before_ns = t_drv_before_ns;
                res->t_drv_after_ns = t_drv_after_ns;

                if (nRet == MV_OK) {
                    Impl::PendingTrigger pending;
                    {
                        std::lock_guard<std::mutex> lk(pImpl->trigger_mtx_);
                        pending.trigger_seq = pImpl->next_trigger_seq_++;
                        pending.t_app_issue_ns = req->t_app_issue_ns;
                        pending.t_drv_before_ns = t_drv_before_ns;
                        pending.t_drv_after_ns = t_drv_after_ns;
                        pImpl->pending_triggers_.push_back(pending);
                    }

                    res->success = true;
                    res->message = "ok";
                    res->trigger_seq = pending.trigger_seq;
                } else {
                    res->success = false;
                    res->message = "TriggerSoftware failed";
                    res->trigger_seq = 0;
                    RCLCPP_WARN(this->get_logger(), "TriggerSoftware failed: 0x%X", nRet);
                }
            });
        RCLCPP_INFO(logger, "TriggerSoftware service ready at '%s'",
                    pImpl->trigger_service_->get_service_name());

        pImpl->param_callback_handle = this->add_on_set_parameters_callback(
            [this, logger](const std::vector<rclcpp::Parameter> &parameters) {
                rcl_interfaces::msg::SetParametersResult result;
                result.successful = true;

                auto set_float = [&](const char *node_name, const char *unit, double value) {
                    MVCC_FLOATVALUE range;
                    std::memset(&range, 0, sizeof(range));
                    int ret = MV_CC_GetFloatValue(pImpl->handle, node_name, &range);
                    if (ret != MV_OK) {
                        result.successful = false;
                        result.reason = std::string("Failed to query range for ") + node_name;
                        return;
                    }
                    float val = static_cast<float>(value);
                    if (val < range.fMin || val > range.fMax) {
                        result.successful = false;
                        result.reason = std::string(node_name) + " out of range [" +
                                        std::to_string(range.fMin) + ", " + std::to_string(range.fMax) + "]";
                        RCLCPP_WARN(logger, "%s %.1f %s out of range [%.1f, %.1f]",
                                    node_name, val, unit, range.fMin, range.fMax);
                        return;
                    }
                    ret = MV_CC_SetFloatValue(pImpl->handle, node_name, val);
                    if (ret == MV_OK) {
                        RCLCPP_INFO(logger, "Dynamically set %s to: %.1f %s", node_name, val, unit);
                    } else {
                        result.successful = false;
                        result.reason = std::string("Failed to set ") + node_name + " via SDK";
                    }
                };

                for (const auto &param : parameters) {
                    if (param.get_name() == "exposure_time") {
                        set_float("ExposureTime", "us", param.as_double());
                    } else if (param.get_name() == "gain") {
                        set_float("Gain", "dB", param.as_double());
                    }
                }
                return result;
            });

        MV_CHECK_THROW(logger, MV_CC_RegisterImageCallBackEx, pImpl->handle, &HikvisionDriver::Impl::image_callback_ex, this);
        MV_CHECK_THROW(logger, MV_CC_StartGrabbing, pImpl->handle);
        break;
    }

    if (pImpl->handle == nullptr) {
        RCLCPP_ERROR(logger, "camera '%s' not found", pImpl->camera_name.c_str());
    }
}

HikvisionDriver::~HikvisionDriver() {
    if (pImpl->handle == nullptr) {
        return;
    }
    auto logger = get_logger();

    MV_CHECK_WARN(logger, MV_CC_StopGrabbing, pImpl->handle);
    MV_CHECK_WARN(logger, MV_CC_CloseDevice, pImpl->handle);
    MV_CHECK_WARN(logger, MV_CC_DestroyHandle, pImpl->handle);
    pImpl->handle = nullptr;
}

}  // namespace hikvision_driver

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(hikvision_driver::HikvisionDriver)
