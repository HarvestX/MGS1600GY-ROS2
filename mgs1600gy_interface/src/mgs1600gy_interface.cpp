// Copyright 2022 HarvestX Inc.
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

#include "mgs1600gy_interface/mgs1600gy_interface.hpp"


namespace mgs1600gy_interface
{

Mgs1600gyInterface::Mgs1600gyInterface(
  const std::string & port_name, const std::chrono::nanoseconds & timeout)
: TIMEOUT_(timeout)
{
  this->port_handler_ = std::make_unique<PortHandler>(port_name);
}

bool Mgs1600gyInterface::init()
{
  // Dry run to check can open port or not
  return this->port_handler_->configure(115200);
}

bool Mgs1600gyInterface::activate()
{
  if (!this->port_handler_->open()) {
    RCLCPP_ERROR(this->getLogger(), "Failed to open port");
    return false;
  }

  this->packet_handler_ =
    std::make_shared<PacketHandler>(this->port_handler_.get());

  this->maintenance_commander_ = std::make_unique<MaintenanceCommander>(
    this->packet_handler_, this->TIMEOUT_);

  this->realtime_commander_ = std::make_unique<RealtimeCommander>(
    this->packet_handler_, this->TIMEOUT_);

  this->stopQueries();
  return true;
}

bool Mgs1600gyInterface::deactivate()
{
  if (this->mode_ == RealtimeCommander::MODE::QUERY) {
    this->stopQueries();
  }

  this->port_handler_->close();
  this->packet_handler_ = nullptr;
  this->realtime_commander_ = nullptr;

  return true;
}

bool Mgs1600gyInterface::setQueries(const PacketPool::PACKET_TYPE & packet_type) noexcept
{
  if (!this->read(packet_type)) {
    RCLCPP_ERROR(
      this->getLogger(), "Failed to set query: %s",
      PacketPool::packetTypeToString(packet_type).c_str());
    return false;
  }
  RCLCPP_INFO(
    this->getLogger(), "Packet type: %s successfully stocked.",
    PacketPool::packetTypeToString(packet_type).c_str());
  this->queries_.emplace_back(packet_type);
  return true;
}

bool Mgs1600gyInterface::startQueries(const uint32_t & every_ms) noexcept
{
  if (this->queries_.empty()) {
    RCLCPP_ERROR(this->getLogger(), "Query is empty");
    return false;
  }

  std::stringstream query_ss;
  for (size_t i = 0; i < this->queries_.size(); ++i) {
    query_ss << PacketPool::packetTypeToString(this->queries_.at(i));
    if (i < this->queries_.size() - 1) {
      query_ss << ", ";
    }
  }
  RCLCPP_INFO(
    this->getLogger(), "Following Commands will repeatedly executed [%s]",
    query_ss.str().c_str());

  const bool state = this->processResponse(this->realtime_commander_->startQuery(every_ms));

  if (state) {
    this->mode_ = RealtimeCommander::MODE::QUERY;
  }
  return state;
}

bool Mgs1600gyInterface::stopQueries() noexcept
{
  const bool & ret = this->processResponse(this->realtime_commander_->clearQuery());
  if (ret) {
    this->mode_ = RealtimeCommander::MODE::NORMAL;
    this->queries_.clear();
  }
  return ret;
}

bool Mgs1600gyInterface::read(const PacketPool::PACKET_TYPE & packet_type)
{
  RESPONSE_STATE state;
  using PT = PacketPool::PACKET_TYPE;
  switch (packet_type) {
    case PT::MZ:
      {
        state = this->realtime_commander_->readMZ(this->mz_data_, this->mode_);
        break;
      }
    case PT::ANG:
      {
        state = this->realtime_commander_->readANG(this->ang_data_, this->mode_);
        break;
      }
    case PT::GY:
      {
        state = this->realtime_commander_->readGY(this->gy_data_, this->mode_);
        break;
      }
    default:
      {
        RCLCPP_ERROR(this->getLogger(), "Invalid packet type given");
        return false;
      }
  }

  return this->processResponse(state);
}

bool Mgs1600gyInterface::readAll()
{
  using PT = PacketPool::PACKET_TYPE;
  for (size_t i = 0; i < static_cast<size_t>(PT::END_PACKET_TYPE); ++i) {
    if (!this->read(static_cast<PT>(i))) {
      return false;
    }
  }
  return true;
}

void Mgs1600gyInterface::getMzData(std::array<float, 16> & out) const noexcept
{
  std::copy(this->mz_data_.begin(), this->mz_data_.end(), out.begin());
}

void Mgs1600gyInterface::getAngData(std::array<float, 3> & out) const noexcept
{
  std::copy(this->ang_data_.begin(), this->ang_data_.end(), out.begin());
}

void Mgs1600gyInterface::getGyData(std::array<float, 3> & out) const noexcept
{
  std::copy(this->gy_data_.begin(), this->gy_data_.end(), out.begin());
}

void Mgs1600gyInterface::setImuCovariance(
  const std::vector<double> & orient_cov, const std::vector<double> & ang_vel_cov)
{
  const auto setter = [](const std::vector<double> & in, std::array<double, 9> & out) {
      assert(in.size() == 3);
      for (size_t i = 0; i < in.size(); ++i) {
        for (size_t j = 0; j < in.size(); ++j) {
          const size_t idx = i * in.size() + j;
          if (i == j) {
            out.at(idx) = in[i];
          } else {
            out.at(idx) = 0.0;
          }
        }
      }
    };

  setter(orient_cov, this->imu_orientation_cov_);
  setter(ang_vel_cov, this->imu_angular_vel_cov_);
}

Imu::UniquePtr Mgs1600gyInterface::getImu(const std_msgs::msg::Header & header) const noexcept
{
  static const float TO_RADIAN = 0.1 * M_PI / 180.0;

  static const size_t Y_IDX = static_cast<size_t>(AxisIndex::YAW);
  static const size_t P_IDX = static_cast<size_t>(AxisIndex::PITCH);
  static const size_t R_IDX = static_cast<size_t>(AxisIndex::ROLL);

  tf2::Quaternion quat;
  quat.setEuler(
    fmod(this->ang_data_[Y_IDX] * TO_RADIAN, 2.0 * M_PI),
    fmod(this->ang_data_[P_IDX] * TO_RADIAN, 2.0 * M_PI),
    fmod(this->ang_data_[R_IDX] * TO_RADIAN, 2.0 * M_PI));
  auto imu_msg = std::make_unique<Imu>();
  imu_msg->header = header;
  imu_msg->orientation.w = quat.getW();
  imu_msg->orientation.x = quat.getX();
  imu_msg->orientation.y = quat.getY();
  imu_msg->orientation.z = quat.getZ();

  std::copy(
    this->imu_orientation_cov_.begin(), this->imu_orientation_cov_.end(),
    imu_msg->orientation_covariance.begin());

  imu_msg->angular_velocity.x = this->gy_data_[P_IDX] * TO_RADIAN;
  imu_msg->angular_velocity.y = this->gy_data_[Y_IDX] * TO_RADIAN;
  imu_msg->angular_velocity.z = this->gy_data_[R_IDX] * TO_RADIAN;

  std::copy(
    this->imu_angular_vel_cov_.begin(), this->imu_angular_vel_cov_.end(),
    imu_msg->angular_velocity_covariance.begin());

  imu_msg->linear_acceleration_covariance.at(0) = -1;  // UNUSED
  return imu_msg;
}

void Mgs1600gyInterface::getImage(
  cv::Mat * out, const float & MIN, const float & MAX) const noexcept
{
  Utils::convertBGR(this->mz_data_, out, MIN, MAX);
}

void Mgs1600gyInterface::getImage(
  std::array<float, 16 * 3> out, const float & MIN, const float & MAX) const noexcept
{
  Utils::convertBRG(this->mz_data_, out, MIN, MAX);
}

bool Mgs1600gyInterface::setAllAngleZero() const noexcept
{
  if (!this->setAngle(AxisIndex::PITCH, 0.0)) {
    return false;
  }
  if (!this->setAngle(AxisIndex::YAW, 0.0)) {
    return false;
  }
  if (!this->setAngle(AxisIndex::ROLL, 0.0)) {
    return false;
  }
  return true;
}

bool Mgs1600gyInterface::setAngle(const AxisIndex & idx, const float & rad) const noexcept
{
  // degree times 100
  static const float RAD2STEP = 100.0 * 180.0 / M_PI;
  const bool ret = this->processResponse(
    this->realtime_commander_->writeANG(
      static_cast<int>(idx) + 1,
      static_cast<int>(std::round(rad * RAD2STEP))));
  rclcpp::sleep_for(10ms);
  return ret;
}

bool Mgs1600gyInterface::calibrateMagnet() const noexcept
{
  RCLCPP_WARN(
    this->getLogger(),
    "Calibrating for magnet sensor. "
    "Position the sensor away from any magnetic or ferrous material...");
  if (!this->processResponse(this->maintenance_commander_->writeZERO())) {
    return false;
  }
  rclcpp::sleep_for(10ms);
  if (!this->processResponse(this->maintenance_commander_->writeCLSAV())) {
    return false;
  }
  return true;
}

bool Mgs1600gyInterface::calibrateGyro() const noexcept
{
  RCLCPP_WARN(this->getLogger(), "Calibrating for Gyro. Keep the sensor totally immobile...");
  if (!this->processResponse(this->maintenance_commander_->writeGZER())) {
    return false;
  }
  rclcpp::sleep_for(10ms);
  if (!this->processResponse(this->maintenance_commander_->writeCLSAV())) {
    return false;
  }

  for (int i = 5; i > 0; --i) {
    RCLCPP_INFO(this->getLogger(), "Waiting for calibration finish %ds ...", i);
    rclcpp::sleep_for(1s);
  }
  return true;
}

const rclcpp::Logger Mgs1600gyInterface::getLogger() noexcept
{
  return rclcpp::get_logger("Mgs1600gyInterface");
}

bool Mgs1600gyInterface::processResponse(const RESPONSE_STATE & state) noexcept
{
  bool ret = false;
  switch (state) {
    case RESPONSE_STATE::OK:
      ret = true;
      break;
    case RESPONSE_STATE::ERROR_INVALID_INPUT:
      RCLCPP_ERROR(Mgs1600gyInterface::getLogger(), "Invalid input given.");
      break;
    case RESPONSE_STATE::ERROR_NO_RESPONSE:
      RCLCPP_ERROR(Mgs1600gyInterface::getLogger(), "Mgs1600gy is not responded.");
      break;
    case RESPONSE_STATE::ERROR_PARSE_FAILED:
      RCLCPP_ERROR(Mgs1600gyInterface::getLogger(), "Failed to parse received response.");
      break;
    case RESPONSE_STATE::ERROR_PARSE_RESULT_INCOMPATIBLE:
      RCLCPP_ERROR(Mgs1600gyInterface::getLogger(), "Responded parameters length is incompatible.");
      break;
    case RESPONSE_STATE::ERROR_UNKNOWN:
      RCLCPP_ERROR(Mgs1600gyInterface::getLogger(), "Unknown error.");
      break;
    default:
      RCLCPP_ERROR(Mgs1600gyInterface::getLogger(), "Invalid state given");
  }
  return ret;
}

}  // namespace mgs1600gy_interface
