#include "hikvision_driver/hikvision_driver.hpp"

// std
#include <cstring>
#include <map>
#include <stdexcept>

// ros
#include <hikvision_interface/msg/hik_image_info.hpp>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/image_encodings.hpp>

// hikvision sdk
#include <MvCameraControl.h>
// camera info
#include <sensor_msgs/msg/camera_info.hpp>
#include <camera_info_manager/camera_info_manager.hpp>

// 用于构造函数：失败时抛出异常打断流程，防止传入空指针
#define MV_CHECK_THROW(logger, func, ...)                                      \
    do {                                                                       \
        int nRet = func(__VA_ARGS__);                                          \
        if (MV_OK != nRet) {                                                   \
            RCLCPP_ERROR(logger, "hikvision sdk error: " #func " = 0x%X", nRet); \
            throw std::runtime_error("Hikvision SDK init error: " #func);      \
        }                                                                      \
    } while (0)

// 用于析构函数：失败时只打印警告（C++析构函数中严禁抛出异常）
#define MV_CHECK_WARN(logger, func, ...)                                       \
    do {                                                                       \
        int nRet = func(__VA_ARGS__);                                          \
        if (MV_OK != nRet) {                                                   \
            RCLCPP_WARN(logger, "hikvision sdk warning: " #func " = 0x%X", nRet); \
        }                                                                      \
    } while (0)

using hikvision_interface::msg::HikImageInfo;

namespace hikvision_driver {

struct HikvisionDriver::Impl {
    std::unique_ptr<rclcpp::Logger> logger;

    void *handle = nullptr;
    std::string camera_name;
    image_transport::Publisher img_pub;
    std::shared_ptr<rclcpp::Publisher<HikImageInfo>> p_info_pub;
    static void image_callback_ex(unsigned char *pData, MV_FRAME_OUT_INFO_EX *pFrameInfo, void *pUser);
    
    // 新增：用于维持动态参数回调生命周期的句柄
    rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr param_callback_handle;

    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub;
    std::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_manager;
    std::string frame_id;
};

void HikvisionDriver::Impl::image_callback_ex(unsigned char *pData, MV_FRAME_OUT_INFO_EX *pFrameInfo, void *pUser) {
    auto node = reinterpret_cast<HikvisionDriver *>(pUser);

    uint64_t dev_stamp = (uint64_t)pFrameInfo->nDevTimeStampHigh << 32ull | (uint64_t)pFrameInfo->nDevTimeStampLow;
    uint64_t host_stamp = pFrameInfo->nHostTimeStamp;

    auto p_img_msg = std::make_unique<sensor_msgs::msg::Image>();
    if (pFrameInfo->nFrameLen > p_img_msg->data.max_size()) {
        RCLCPP_ERROR_ONCE(node->get_logger(), "image bytes exceed max available size");
        return;
    }
    p_img_msg->header.frame_id = node->pImpl->frame_id;
    p_img_msg->header.stamp.nanosec = host_stamp % 1000ull * 1000000ull;
    p_img_msg->header.stamp.sec = host_stamp / 1000ull;
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
        RCLCPP_ERROR_ONCE(node->get_logger(), "unsupport pixel format: %d", (int)pFrameInfo->enPixelType);
        return;
    }
    // 1. 检查长度并填充图像数据
    if (pFrameInfo->nFrameLen < (p_img_msg->height * p_img_msg->step)) {
        RCLCPP_ERROR(node->get_logger(), "nFrameLen < required data size, len=%d", pFrameInfo->nFrameLen);
        return;
    }
    p_img_msg->data.resize(p_img_msg->height * p_img_msg->step);
    std::copy_n(pData, p_img_msg->data.size(), p_img_msg->data.data());

    // 2. 构造并发布自定义硬件 Info 消息
    auto p_info_msg = std::make_unique<hikvision_interface::msg::HikImageInfo>();
    p_info_msg->header.frame_id = node->pImpl->camera_name;
    p_info_msg->header.stamp.nanosec = host_stamp % 1000ull * 1000000ull;
    p_info_msg->header.stamp.sec = host_stamp / 1000ull;
    p_info_msg->dev_stamp.nanosec = dev_stamp % 1000000000ull;
    p_info_msg->dev_stamp.sec = dev_stamp / 1000000000ull;
    p_info_msg->frame_num = pFrameInfo->nFrameNum;
    p_info_msg->gain = pFrameInfo->fGain;
    p_info_msg->exposure = pFrameInfo->fExposureTime;
    p_info_msg->red = pFrameInfo->nRed;
    p_info_msg->green = pFrameInfo->nGreen;
    p_info_msg->blue = pFrameInfo->nBlue;
    node->pImpl->p_info_pub->publish(std::move(p_info_msg));

    // 3. 构造并发布 CameraInfo 消息
    auto cam_info_msg = std::make_unique<sensor_msgs::msg::CameraInfo>(node->pImpl->cinfo_manager->getCameraInfo());
    cam_info_msg->header = p_img_msg->header; // 先使用 p_img_msg 的数据
    node->pImpl->camera_info_pub->publish(std::move(cam_info_msg));

    // 4. 【最后一步】发布 Image 消息 (一旦 move，p_img_msg 将失效)
    node->pImpl->img_pub.publish(std::move(p_img_msg));
}

HikvisionDriver::HikvisionDriver(const rclcpp::NodeOptions &options)
    : rclcpp::Node("hikvision_driver_node", options), pImpl(std::make_unique<Impl>()) {
    auto logger = get_logger();
    pImpl->logger = std::make_unique<rclcpp::Logger>(logger);

    // 给定空字符串作为默认值，防止 as_string() 抛出异常
    declare_parameter<std::string>("camera_name", ""); 
    pImpl->camera_name = get_parameter("camera_name").as_string();
    
    // 现在如果没传名字，这段友好的报错逻辑就能被完美触发了
    if (pImpl->camera_name.empty()) {
        RCLCPP_ERROR(logger, "Parameter 'camera_name' is empty! You must specify a valid camera name.");
        throw std::runtime_error("Parameter 'camera_name' is missing.");
    }
    RCLCPP_INFO(logger, "trying to open camera: '%s'", pImpl->camera_name.c_str());

    // 声明 frame_id 和 camera_info_url
    declare_parameter<std::string>("frame_id", pImpl->camera_name);
    pImpl->frame_id = get_parameter("frame_id").as_string();
    declare_parameter<std::string>("camera_info_url", "");

    // 新增：声明曝光和增益的动态参数（默认值：20000us 曝光，15dB 增益）
    declare_parameter<double>("exposure_time", 20000.0);
    declare_parameter<double>("gain", 15.0);

    // 新增：像素格式可在配置文件中指定。
    // "Keep" 表示不修改相机当前格式（由 MVS 客户端预先配置）；
    // 也可指定 "RGB8" / "BGR8" / "Mono8" / "BayerRG8" / "BayerBG8" / "BayerGR8" / "BayerGB8"。
    declare_parameter<std::string>("pixel_format", "RGB8");

    // 图像数据流使用 SensorDataQoS（BEST_EFFORT），避免大图像在 RELIABLE 下阻塞/重传堆积。
    auto qos = rclcpp::SensorDataQoS();
    pImpl->img_pub = image_transport::create_publisher(this, "image_raw", qos.get_rmw_qos_profile());
    pImpl->p_info_pub = create_publisher<HikImageInfo>("info", qos);

    pImpl->camera_info_pub = create_publisher<sensor_msgs::msg::CameraInfo>("camera_info", qos);
    pImpl->cinfo_manager = std::make_shared<camera_info_manager::CameraInfoManager>(
        this, pImpl->camera_name, get_parameter("camera_info_url").as_string());

    MV_CC_DEVICE_INFO_LIST stDeviceList;
    memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
    MV_CHECK_THROW(logger, MV_CC_EnumDevices, MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);

    for (uint32_t nDeviceId = 0; nDeviceId < stDeviceList.nDeviceNum; nDeviceId++) {
        auto *pDeviceInfo = stDeviceList.pDeviceInfo[nDeviceId];
        const char *pUserDefinedName = nullptr;
        if (pDeviceInfo->nTLayerType == MV_GIGE_DEVICE) {
            pUserDefinedName = (const char *)pDeviceInfo->SpecialInfo.stGigEInfo.chUserDefinedName;
            if (pUserDefinedName == pImpl->camera_name) {
                int nIp1 = ((pDeviceInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0xff000000) >> 24);
                int nIp2 = ((pDeviceInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x00ff0000) >> 16);
                int nIp3 = ((pDeviceInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x0000ff00) >> 8);
                int nIp4 = (pDeviceInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x000000ff);
                RCLCPP_INFO(logger, "[%s]: GIGE, %s, %d.%d.%d.%d", pUserDefinedName,
                            pDeviceInfo->SpecialInfo.stGigEInfo.chModelName, nIp1, nIp2, nIp3, nIp4);
            }
        } else if (pDeviceInfo->nTLayerType == MV_USB_DEVICE) {
            pUserDefinedName = (const char *)pDeviceInfo->SpecialInfo.stUsb3VInfo.chUserDefinedName;
            if (pUserDefinedName == pImpl->camera_name) {
                RCLCPP_INFO(logger, "[%s]: USB, %s", pUserDefinedName,
                            pDeviceInfo->SpecialInfo.stUsb3VInfo.chModelName);
            }
        } else {
            RCLCPP_WARN(logger, "type(%d) not support", pDeviceInfo->nTLayerType);
        }
        if (pUserDefinedName == pImpl->camera_name) {
            MV_CHECK_THROW(logger, MV_CC_CreateHandle, &pImpl->handle, pDeviceInfo);
            MV_CHECK_THROW(logger, MV_CC_OpenDevice, pImpl->handle);

            // 像素格式：根据 pixel_format 参数设置；"Keep" 时保持相机当前格式。
            std::string pixel_format = get_parameter("pixel_format").as_string();
            if (pixel_format != "Keep") {
                static const std::map<std::string, MvGvspPixelType> kPixelFormatMap = {
                    {"Mono8", PixelType_Gvsp_Mono8},         {"RGB8", PixelType_Gvsp_RGB8_Packed},
                    {"BGR8", PixelType_Gvsp_BGR8_Packed},    {"BayerRG8", PixelType_Gvsp_BayerRG8},
                    {"BayerBG8", PixelType_Gvsp_BayerBG8},   {"BayerGR8", PixelType_Gvsp_BayerGR8},
                    {"BayerGB8", PixelType_Gvsp_BayerGB8},
                };
                auto it = kPixelFormatMap.find(pixel_format);
                if (it == kPixelFormatMap.end()) {
                    RCLCPP_WARN(logger, "unknown pixel_format '%s', keeping camera default", pixel_format.c_str());
                } else {
                    MV_CHECK_THROW(logger, MV_CC_SetEnumValue, pImpl->handle, "PixelFormat", it->second);
                }
            }

            // 初始化相机参数
            double init_exposure = get_parameter("exposure_time").as_double();
            double init_gain = get_parameter("gain").as_double();
            MV_CHECK_THROW(logger, MV_CC_SetEnumValue, pImpl->handle, "ExposureAuto", 0); // 关闭自动曝光
            MV_CHECK_THROW(logger, MV_CC_SetFloatValue, pImpl->handle, "ExposureTime", static_cast<float>(init_exposure));
            MV_CHECK_THROW(logger, MV_CC_SetFloatValue, pImpl->handle, "Gain", static_cast<float>(init_gain));
            MV_CHECK_THROW(logger, MV_CC_SetEnumValue, pImpl->handle, "BalanceWhiteAuto", 2); // 自动白平衡

            // 新增：注册动态参数监听回调（实现实时滑块控制）
            pImpl->param_callback_handle = this->add_on_set_parameters_callback(
                [this, logger](const std::vector<rclcpp::Parameter> &parameters) {
                    rcl_interfaces::msg::SetParametersResult result;
                    result.successful = true;

                    // 在设置前查询 SDK 的 [min, max] 并裁剪，越界值会被 SDK 直接拒绝。
                    auto set_float = [&](const char *node_name, const char *unit, double value) {
                        MVCC_FLOATVALUE range;
                        memset(&range, 0, sizeof(range));
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
                            RCLCPP_WARN(logger, "%s %.1f %s out of range [%.1f, %.1f]", node_name, val, unit,
                                        range.fMin, range.fMax);
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
                }
            );

            MV_CHECK_THROW(logger, MV_CC_RegisterImageCallBackEx, pImpl->handle, &HikvisionDriver::Impl::image_callback_ex,
                     this);
            MV_CHECK_THROW(logger, MV_CC_StartGrabbing, pImpl->handle);
            break;
        }
    }
    if (pImpl->handle == nullptr) {
        RCLCPP_ERROR(logger, "camera '%s' not found", pImpl->camera_name.c_str());
    }
}

HikvisionDriver::~HikvisionDriver() {
    if (pImpl->handle == nullptr) return;
    auto logger = get_logger();

    MV_CHECK_WARN(logger, MV_CC_StopGrabbing, pImpl->handle);
    MV_CHECK_WARN(logger, MV_CC_CloseDevice, pImpl->handle);
    MV_CHECK_WARN(logger, MV_CC_DestroyHandle, pImpl->handle);
    pImpl->handle = nullptr;
}

}  // namespace hikvision_driver

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(hikvision_driver::HikvisionDriver);