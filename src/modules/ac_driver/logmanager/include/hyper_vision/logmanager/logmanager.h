/*********************************************************************************************************************
  Copyright 2025 RoboSense Technology Co., Ltd

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

  http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*********************************************************************************************************************/
#ifndef LOGMANAGER_H
#define LOGMANAGER_H

#include "spdlog/sinks/basic_file_sink.h"
#include "spdlog/sinks/stdout_color_sinks.h"
#include "spdlog/spdlog.h"
#include <iostream>

namespace robosense {
namespace log {

class RSLogConfig {
public:
  using Ptr = std::shared_ptr<RSLogConfig>;
  using ConstPtr = std::shared_ptr<const RSLogConfig>;

public:
  RSLogConfig() { reset(); }
  ~RSLogConfig() = default;

public:
  void reset() {
    log_file_dir_path = "";
    log_file_path = "";
    log_level = spdlog::level::level_enum::info;
    is_log_file_trunc = false;
  }

public:
  int32_t log_level;
  std::string log_file_dir_path;
  std::string log_file_path;
  bool is_log_file_trunc;
};

class RSLogManager {
public:
  using Ptr = std::shared_ptr<RSLogManager>;
  using ConstPtr = std::shared_ptr<const RSLogManager>;

public:
  RSLogManager() = default;
  ~RSLogManager() = default;

public:
  static int init(const RSLogConfig &log_config) {
    if (RSLogManager::is_logger_init) {
      return 0;
    }

    std::vector<spdlog::sink_ptr> sinks;
    std::shared_ptr<spdlog::sinks::stdout_color_sink_mt> stdout_sink_ptr =
        nullptr;
    try {
      stdout_sink_ptr = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
    } catch (...) {
      return -1;
    }
    sinks.push_back(stdout_sink_ptr);

    if (!log_config.log_file_path.empty()) {
      std::shared_ptr<spdlog::sinks::basic_file_sink_mt> basic_file_sink_ptr =
          nullptr;
      try {
        basic_file_sink_ptr =
            std::make_shared<spdlog::sinks::basic_file_sink_mt>(
                log_config.log_file_path, log_config.is_log_file_trunc);
      } catch (...) {
        return -2;
      }
      sinks.push_back(basic_file_sink_ptr);
    }

    try {
      logger = std::make_shared<spdlog::logger>("ac_driver_logger",
                                                sinks.begin(), sinks.end());
    } catch (...) {
      return -3;
    }

    RSLogManager::logger->set_level(
        static_cast<spdlog::level::level_enum>(log_config.log_level));
    RSLogManager::logger->flush_on(spdlog::level::level_enum::err);
    logger->set_pattern("[%n][%Y-%m-%d %H:%M:%S.%e][%l][%t]%v");
    RSLogManager::is_logger_init = true;

    return 0;
  }

public:
  static std::shared_ptr<spdlog::logger> logger;
  static bool is_logger_init;
};

} // namespace log
} // namespace robosense

#ifndef FILENAME
#ifdef __GNUC__
#define FILENAME(x) (strrchr(x, '/') ? strrchr(x, '/') + 1 : x)
#include <sched.h>
inline int32_t getProcessorNumber() { return sched_getcpu(); }
#elif _MSC_VER
#define FILENAME(x) (strrchr(x, '\\') ? strrchr(x, '\\') + 1 : x)
#include <windows>
inline int32_t getProcessorNumber() {
  PROCESSOR_NUMBER procNumber;
  GetCurrentProcessorNumberEx(&procNumber);
  return procNumber.Number;
}
#endif
#endif // FILENAME
#define GET_CPUNO getProcessorNumber()

#if ENABLE_SPDLOG_GET_CPU_NUM
#if ENABLE_SPDLOG_USE_FULL_FILE_NAME
#define RS_SPDLOG_TRACE(format, ...)                                           \
  if (robosense::log::RSLogManager::is_logger_init) {                          \
    robosense::log::RSLogManager::logger->trace(                               \
        (std::string("[{}:{}][{}] ") + format).c_str(), __FILE__, __LINE__,    \
        GET_CPUNO, ##__VA_ARGS__);                                             \
  }

#define RS_SPDLOG_DEBUG(format, ...)                                           \
  if (robosense::log::RSLogManager::is_logger_init) {                          \
    robosense::log::RSLogManager::logger->debug(                               \
        (std::string("[{}:{}][{}] ") + format).c_str(), __FILE__, __LINE__,    \
        GET_CPUNO, ##__VA_ARGS__);                                             \
  }

#define RS_SPDLOG_INFO(format, ...)                                            \
  if (robosense::log::RSLogManager::is_logger_init) {                          \
    robosense::log::RSLogManager::logger->info(                                \
        (std::string("[{}:{}][{}] ") + format).c_str(), __FILE__, __LINE__,    \
        GET_CPUNO, ##__VA_ARGS__);                                             \
  }

#define RS_SPDLOG_WARN(format, ...)                                            \
  if (robosense::log::RSLogManager::is_logger_init) {                          \
    robosense::log::RSLogManager::logger->warn(                                \
        (std::string("[{}:{}][{}] ") + format).c_str(), __FILE__, __LINE__,    \
        GET_CPUNO, ##__VA_ARGS__);                                             \
  }

#define RS_SPDLOG_ERROR(format, ...)                                           \
  if (robosense::log::RSLogManager::is_logger_init) {                          \
    robosense::log::RSLogManager::logger->error(                               \
        (std::string("[{}:{}][{}] ") + format).c_str(), __FILE__, __LINE__,    \
        GET_CPUNO, ##__VA_ARGS__);                                             \
  }

#define RS_SPDLOG_CRITICAL(format, ...)                                        \
  if (robosense::log::RSLogManager::is_logger_init) {                          \
    robosense::log::RSLogManager::logger->critical(                            \
        (std::string("[{}:{}][{}] ") + format).c_str(), __FILE__, __LINE__,    \
        GET_CPUNO, ##__VA_ARGS__);                                             \
  }
#else

#define RS_SPDLOG_TRACE(format, ...)                                           \
  if (robosense::log::RSLogManager::is_logger_init) {                          \
    robosense::log::RSLogManager::logger->trace(                               \
        (std::string("[{}:{}][{}] ") + format).c_str(), FILENAME(__FILE__),    \
        __LINE__, GET_CPUNO, ##__VA_ARGS__);                                   \
  }

#define RS_SPDLOG_DEBUG(format, ...)                                           \
  if (robosense::log::RSLogManager::is_logger_init) {                          \
    robosense::log::RSLogManager::logger->debug(                               \
        (std::string("[{}:{}][{}] ") + format).c_str(), FILENAME(__FILE__),    \
        __LINE__, GET_CPUNO, ##__VA_ARGS__);                                   \
  }

#define RS_SPDLOG_INFO(format, ...)                                            \
  if (robosense::log::RSLogManager::is_logger_init) {                          \
    robosense::log::RSLogManager::logger->info(                                \
        (std::string("[{}:{}][{}] ") + format).c_str(), FILENAME(__FILE__),    \
        __LINE__, GET_CPUNO, ##__VA_ARGS__);                                   \
  }

#define RS_SPDLOG_WARN(format, ...)                                            \
  if (robosense::log::RSLogManager::is_logger_init) {                          \
    robosense::log::RSLogManager::logger->warn(                                \
        (std::string("[{}:{}][{}] ") + format).c_str(), FILENAME(__FILE__),    \
        __LINE__, GET_CPUNO, ##__VA_ARGS__);                                   \
  }

#define RS_SPDLOG_ERROR(format, ...)                                           \
  if (robosense::log::RSLogManager::is_logger_init) {                          \
    robosense::log::RSLogManager::logger->error(                               \
        (std::string("[{}:{}][{}] ") + format).c_str(), FILENAME(__FILE__),    \
        __LINE__, GET_CPUNO, ##__VA_ARGS__);                                   \
  }

#define RS_SPDLOG_CRITICAL(format, ...)                                        \
  if (robosense::log::RSLogManager::is_logger_init) {                          \
    robosense::log::RSLogManager::logger->critical(                            \
        (std::string("[{}:{}][{}] ") + format).c_str(), FILENAME(__FILE__),    \
        __LINE__, GET_CPUNO, ##__VA_ARGS__);                                   \
  }

#endif // ENABLE_SPDLOG_USE_FULL_FILE_NAME

#else
#if ENABLE_SPDLOG_USE_FULL_FILE_NAME
#define RS_SPDLOG_TRACE(format, ...)                                           \
  if (robosense::log::RSLogManager::is_logger_init) {                          \
    robosense::log::RSLogManager::logger->trace(                               \
        (std::string("[{}:{}] ") + format).c_str(), __FILE__, __LINE__,        \
        ##__VA_ARGS__);                                                        \
  }

#define RS_SPDLOG_DEBUG(format, ...)                                           \
  if (robosense::log::RSLogManager::is_logger_init) {                          \
    robosense::log::RSLogManager::logger->debug(                               \
        (std::string("[{}:{}] ") + format).c_str(), __FILE__, __LINE__,        \
        ##__VA_ARGS__);                                                        \
  }

#define RS_SPDLOG_INFO(format, ...)                                            \
  if (robosense::log::RSLogManager::is_logger_init) {                          \
    robosense::log::RSLogManager::logger->info(                                \
        (std::string("[{}:{}] ") + format).c_str(), __FILE__, __LINE__,        \
        ##__VA_ARGS__);                                                        \
  }

#define RS_SPDLOG_WARN(format, ...)                                            \
  if (robosense::log::RSLogManager::is_logger_init) {                          \
    robosense::log::RSLogManager::logger->warn(                                \
        (std::string("[{}:{}] ") + format).c_str(), __FILE__, __LINE__,        \
        ##__VA_ARGS__);                                                        \
  }

#define RS_SPDLOG_ERROR(format, ...)                                           \
  if (robosense::log::RSLogManager::is_logger_init) {                          \
    robosense::log::RSLogManager::logger->error(                               \
        (std::string("[{}:{}] ") + format).c_str(), __FILE__, __LINE__,        \
        ##__VA_ARGS__);                                                        \
  }

#define RS_SPDLOG_CRITICAL(format, ...)                                        \
  if (robosense::log::RSLogManager::is_logger_init) {                          \
    robosense::log::RSLogManager::logger->critical(                            \
        (std::string("[{}:{}] ") + format).c_str(), __FILE__, __LINE__,        \
        ##__VA_ARGS__);                                                        \
  }
#else

#define RS_SPDLOG_TRACE(format, ...)                                           \
  if (robosense::log::RSLogManager::is_logger_init) {                          \
    robosense::log::RSLogManager::logger->trace(                               \
        (std::string("[{}:{}] ") + format).c_str(), FILENAME(__FILE__),        \
        __LINE__, ##__VA_ARGS__);                                              \
  }

#define RS_SPDLOG_DEBUG(format, ...)                                           \
  if (robosense::log::RSLogManager::is_logger_init) {                          \
    robosense::log::RSLogManager::logger->debug(                               \
        (std::string("[{}:{}] ") + format).c_str(), FILENAME(__FILE__),        \
        __LINE__, ##__VA_ARGS__);                                              \
  }

#define RS_SPDLOG_INFO(format, ...)                                            \
  if (robosense::log::RSLogManager::is_logger_init) {                          \
    robosense::log::RSLogManager::logger->info(                                \
        (std::string("[{}:{}] ") + format).c_str(), FILENAME(__FILE__),        \
        __LINE__, ##__VA_ARGS__);                                              \
  }

#define RS_SPDLOG_WARN(format, ...)                                            \
  if (robosense::log::RSLogManager::is_logger_init) {                          \
    robosense::log::RSLogManager::logger->warn(                                \
        (std::string("[{}:{}] ") + format).c_str(), FILENAME(__FILE__),        \
        __LINE__, ##__VA_ARGS__);                                              \
  }

#define RS_SPDLOG_ERROR(format, ...)                                           \
  if (robosense::log::RSLogManager::is_logger_init) {                          \
    robosense::log::RSLogManager::logger->error(                               \
        (std::string("[{}:{}] ") + format).c_str(), FILENAME(__FILE__),        \
        __LINE__, ##__VA_ARGS__);                                              \
  }

#define RS_SPDLOG_CRITICAL(format, ...)                                        \
  if (robosense::log::RSLogManager::is_logger_init) {                          \
    robosense::log::RSLogManager::logger->critical(                            \
        (std::string("[{}:{}] ") + format).c_str(), FILENAME(__FILE__),        \
        __LINE__, ##__VA_ARGS__);                                              \
  }

#endif // ENABLE_SPDLOG_USE_FULL_FILE_NAME

#endif // ENABLE_SPDLOG_GET_CPU_NUM

#endif // LOGMANAGER_H
