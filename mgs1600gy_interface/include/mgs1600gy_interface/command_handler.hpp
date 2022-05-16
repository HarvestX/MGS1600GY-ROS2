// Copyright 2022 HarvestX Inc
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


#pragma once

#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>

#include "mgs1600gy_interface/command_handler/parser.hpp"

#include "mgs1600gy_interface/command_handler/rx_configuration.hpp"
#include "mgs1600gy_interface/command_handler/rx_realtime.hpp"
#include "mgs1600gy_interface/command_handler/tx_buffer.hpp"
#include "mgs1600gy_interface/command_handler/tx_configuration.hpp"
#include "mgs1600gy_interface/command_handler/tx_maintenance.hpp"
#include "mgs1600gy_interface/command_handler/tx_realtime.hpp"

#include "mgs1600gy_interface/port_handler.hpp"

namespace mgs1600gy_interface
{
class CommandHandler
{
public:
  static const int MAGNET_SENSOR_NUM = 16;
  static const int GYRO_SENSOR_NUM = 3;
  const std::unique_ptr<const RxConfiguration> rx_config_generator;
  const std::unique_ptr<const RxRealtime> rx_realtime_generator;
  const std::unique_ptr<const TxBuffer> tx_buffer_generator;
  const std::unique_ptr<const TxConfiguration> tx_config_generator;
  const std::unique_ptr<const TxMaintenance> tx_mainte_generator;
  const std::unique_ptr<const TxRealtime> tx_realtime_generator;

  const std::unique_ptr<PortHandler> port_handler_;

  std::array<int, MAGNET_SENSOR_NUM> mg_data;
  std::array<int, GYRO_SENSOR_NUM> gyro_data;

private:
  const rclcpp::Logger logger_;
  const std::unique_ptr<Parser<int, MAGNET_SENSOR_NUM>> mg_parser_;
  const std::unique_ptr<Parser<int, GYRO_SENSOR_NUM>> gyro_parser_;

public:
  CommandHandler() = delete;
  explicit CommandHandler(std::unique_ptr<PortHandler>);
  ~CommandHandler();

  // rx_configuration
  // bool readANAM();
  // bool readBADJ();
  // bool readBRUN();
  // bool readDIM();
  // bool readFCAL();
  // bool readGRNG();
  // bool readMMOD();
  // bool readPWMM();
  // bool readRSBR();
  // bool readSCRO();
  // bool readTINV();
  // bool readTMS();
  // bool readTPOL();
  // bool readTWDT();
  // bool readTXOF();
  // bool readZADJ();


  // rx_realtime
  // bool readB(const int);
  // bool readMGD();
  // bool readMGM();
  // bool readMGM(const int);
  bool readMZ();
  // bool readMZ(const int);
  // bool readT();
  // bool readMGT();
  // bool readMGT(const int);
  // bool readVAR(const int);
  // bool readGY();
  // bool readGY(const int);
  // bool readMGS();
  // bool readMGX();
  bool readANG();
  // bool readANG(const int);

  // tx_buffer
  bool writeRepeat();
  bool writeRepeatEvery(const int);
  bool writeClear();

  // tx_configuration
  // bool writeANAM(const int);
  // bool writeBADJ(const int);
  // bool writeBRUN(const int);
  // bool writeDIM(const int);
  // bool writeFCAL(const int);
  // bool writeGRNG(const int);
  // bool writeMMOD(const int);
  // bool writePWMM(const int);
  // bool writeRSBR(const int);
  // bool writeSCRO(const int);
  // bool writeTINV(const int);
  // bool writeTMS(const int);
  // bool writeTPOL(const int);
  // bool writeTWDT(const int);
  // bool writeTXOF(const int);
  // bool writeZADJ(const int);

  // tx_maintenance
  // bool writeCLSAV();
  // bool writeCLRST(const int);
  // bool writeEELD();
  // bool writeEERST(const int);
  // bool writeEESAV();
  // bool writeGREF();
  // bool writeGZER();
  // bool writeZERO();

  // tx_realtime
  // bool writeB(const int, const int);
  // bool writeR(const int);
  // bool writeTV();
  // bool writeVAR(const int, const int);
  // bool writeTX();
  // bool writeANG(const int, const int);
  // bool writeZER();

  bool send(const std::string &&);
  bool sendRecv(const std::string &&, std::string &);
  bool recv(std::string &);

  bool parseMgData(const std::string &);
  bool parseGyroData(const std::string &);

  void showMgData();
  void showGyroData();
};
}  // namespace mgs1600gy_interface
