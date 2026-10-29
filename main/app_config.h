#pragma once

#define APP_NAME             "xiao-imu-ble"
#define APP_VERSION_MAJOR    1
#define APP_VERSION_MINOR    1
#define APP_VERSION_PATCH    0
#define APP_DEVICE_NAME      "XIAO-IMU-RAW"
#define APP_EVENT_QUEUE_LEN  16

// #define APP_IMU_PACKET_SIZE  28
#define APP_IMU_RAW_PACKET_SIZE 36
#define APP_STATUS_SIZE      64

#include "sdkconfig.h"
#if defined(CONFIG_IMU_RAW_RATE_HZ)
#define APP_IMU_NORMAL_RATE_HZ CONFIG_IMU_RAW_RATE_HZ
#elif defined(CONFIG_IMU_DMP_RATE_HZ)
#define APP_IMU_NORMAL_RATE_HZ CONFIG_IMU_DMP_RATE_HZ
#else
#define APP_IMU_NORMAL_RATE_HZ 1
#endif
#ifdef CONFIG_IMU_SLOW_RATE_HZ
#define APP_IMU_SLOW_RATE_HZ CONFIG_IMU_SLOW_RATE_HZ
#else
#define APP_IMU_SLOW_RATE_HZ 1
#endif
