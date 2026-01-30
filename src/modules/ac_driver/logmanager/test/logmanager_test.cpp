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

#include "hyper_vision/logmanager/logmanager.h"

int main(int argc, char **argv) {
  const std::string &test_log_path = "/home/sti/test_spdlog/log.txt";
  const int test_log_level = 2;
  const bool test_log_file_trunc = true;
  robosense::log::RSLogConfig config;
  config.log_file_path = test_log_path;
  config.log_level = test_log_level;
  config.is_log_file_trunc = test_log_file_trunc;

  robosense::log::RSLogManager::init(config);

  RS_SPDLOG_TRACE("测试 SpdLog Trace");
  RS_SPDLOG_DEBUG("测试 SpdLog Debug");
  RS_SPDLOG_INFO("测试 SpdLog Info");
  RS_SPDLOG_WARN("测试 SpdLog Warn");
  RS_SPDLOG_ERROR("测试 SpdLog Error");
  RS_SPDLOG_CRITICAL("测试 SpdLog Critical");

  return 0;
}