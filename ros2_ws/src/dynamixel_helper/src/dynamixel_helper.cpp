#include "dynamixel_helper/dynamixel_helper.h"
#include <cmath>
#include <iostream>

DynamixelHelper::DynamixelHelper(std::string port) {
  this->portHandler = dynamixel::PortHandler::getPortHandler(port.c_str());
  this->packetHandler = dynamixel::PacketHandler::getPacketHandler();
}

DynamixelHelper::~DynamixelHelper() {
  this->portHandler->closePort();
  delete this->portHandler;
  delete this->packetHandler;
}

void DynamixelHelper::openPort() {
  if (this->portHandler->openPort())
    std::clog << "Successfully opened the port!" << std::endl;
  else {
    std::cerr << "Failed to open the port, exiting..." << std::endl;
    std::exit(1);
  }
}

void DynamixelHelper::setBaudrate(const int baudrate) {
  if (this->portHandler->setBaudRate(baudrate))
    std::clog << "Successfully changed the baudrate!" << std::endl;
  else {
    std::cerr << "Failed to change the baudrate, exiting..." << std::endl;
    std::exit(1);
  }
}

void DynamixelHelper::ledEnable(uint8_t id) { this->writeMotor(id, 65, 1, 1); }

void DynamixelHelper::ledDisable(uint8_t id) { this->writeMotor(id, 65, 0, 1); }

void DynamixelHelper::groupLedEnable(std::vector<uint8_t> ids) {
  // Send ones to enable
  std::vector<uint32_t> data(ids.size(), 1);
  this->groupWriteMotor(ids, data, 65, 1);
}

void DynamixelHelper::groupLedDisable(std::vector<uint8_t> ids) {
  // Send zeroes to disable
  std::vector<uint32_t> data(ids.size(), 0);
  this->groupWriteMotor(ids, data, 65, 1);
}

void DynamixelHelper::torqueEnable(uint8_t id) {
  this->writeMotor(id, 64, 1, 1);
  std::clog << "[ID: " << (int)id << "] Torque Enabled" << std::endl;
}

void DynamixelHelper::torqueDisable(uint8_t id) {
  this->writeMotor(id, 64, 0, 1);
  std::clog << "[ID: " << (int)id << "] Torque Disabled" << std::endl;
}

void DynamixelHelper::groupTorqueEnable(std::vector<uint8_t> ids) {
  std::vector<uint32_t> data(ids.size(), 1);
  this->groupWriteMotor(ids, data, 64, 1);
}

void DynamixelHelper::groupTorqueDisable(std::vector<uint8_t> ids) {
  std::vector<uint32_t> data(ids.size(), 0);
  this->groupWriteMotor(ids, data, 64, 1);
}

void DynamixelHelper::setAngle(uint8_t id, double val) {
  this->writeMotor(id, 116, (uint32_t)(val * 180.0 / M_PI / 0.088), 4);
}

double DynamixelHelper::getAngle(uint8_t id) {
  return readMotor(id, 132) * 0.088 * M_PI / 180.0;
}

void DynamixelHelper::groupSetAngle(std::vector<uint8_t> ids,
                                    std::vector<double> vals) {
  std::vector<uint32_t> data(ids.size());
  for (size_t i = 0; i < ids.size(); i++)
    data[i] = (uint32_t)(vals[i] * 180.0 / M_PI / 0.088);

  groupWriteMotor(ids, data, 116, 4);
}

std::vector<double> DynamixelHelper::groupGetAngle(std::vector<uint8_t> ids) {
  std::vector<uint32_t> retrieved_data = this->groupReadMotor(ids, 132, 4);
  std::vector<double> present_positions;
  for (size_t i = 0; i < retrieved_data.size(); i++)
    present_positions.push_back(retrieved_data[i] * 0.088 * M_PI / 180.0);

  return present_positions;
}

void DynamixelHelper::writeMotor(uint8_t id, uint16_t address, uint32_t data,
                                 uint16_t byte_size) {
  int comm_result = COMM_TX_FAIL;
  uint8_t error = 0;

  if (byte_size == 1)
    comm_result = this->packetHandler->write1ByteTxRx(this->portHandler, id,
                                                      address, data, &error);
  else if (byte_size == 4)
    comm_result = this->packetHandler->write4ByteTxRx(this->portHandler, id,
                                                      address, data, &error);

  if (comm_result != COMM_SUCCESS) {
    std::cerr << this->packetHandler->getTxRxResult(comm_result) << std::endl;
    std::exit(1);
  } else if (error != 0) {
    std::cerr << this->packetHandler->getRxPacketError(error) << std::endl;
    std::exit(1);
  }
}

uint32_t DynamixelHelper::readMotor(uint8_t id, uint16_t address) {
  uint32_t present_position;

  int comm_result = COMM_TX_FAIL;
  uint8_t error = 0;

  comm_result = this->packetHandler->read4ByteTxRx(
      this->portHandler, id, address, &present_position, &error);
  if (comm_result != COMM_SUCCESS) {
    std::cerr << this->packetHandler->getTxRxResult(comm_result) << std::endl;
    std::exit(1);
  } else if (error != 0) {
    std::cerr << this->packetHandler->getRxPacketError(error) << std::endl;
    std::exit(1);
  }

  return present_position;
}

void DynamixelHelper::groupWriteMotor(std::vector<uint8_t> ids,
                                      std::vector<uint32_t> data,
                                      uint16_t address, uint16_t byte_size) {
  dynamixel::GroupSyncWrite groupSyncWrite(
      this->portHandler, this->packetHandler, address, byte_size);

  std::vector<uint8_t> param_data(byte_size);
  for (size_t i = 0; i < ids.size(); i++) {
    // Convert data to raw data for transfer
    for (int j = 0; j < byte_size; j++) {
      if (j % 2 == 0) {
        if (j % 4 < 2)
          param_data[j] = DXL_LOBYTE(DXL_LOWORD(data[i]));
        else
          param_data[j] = DXL_LOBYTE(DXL_HIWORD(data[i]));
      } else {
        if (j % 4 < 2)
          param_data[j] = DXL_HIBYTE(DXL_LOWORD(data[i]));
        else
          param_data[j] = DXL_HIBYTE(DXL_HIWORD(data[i]));
      }
    }

    // Add each goal to each motor write parameter
    if (!groupSyncWrite.addParam(ids[i], &param_data[0])) {
      std::cerr << "[ID: " << ids[i] << "] GroupSyncWrite AddParam failed"
                << std::endl;
      std::exit(1);
    }
  }

  // Send write packet
  int comm_result = groupSyncWrite.txPacket();
  if (comm_result != COMM_SUCCESS) {
    std::cerr << this->packetHandler->getTxRxResult(comm_result) << std::endl;
  }

  // Clear stored write values
  groupSyncWrite.clearParam();
}

std::vector<uint32_t> DynamixelHelper::groupReadMotor(std::vector<uint8_t> ids,
                                                      uint16_t address,
                                                      uint16_t byte_size) {
  dynamixel::GroupSyncRead groupSyncRead(this->portHandler, this->packetHandler,
                                         address, byte_size);

  // Add motors to groupsyncread for present position reading
  for (auto id : ids) {
    if (!groupSyncRead.addParam(id)) {
      std::cerr << "[ID: " << id << "] GroupSyncRead AddParam failed"
                << std::endl;
      std::exit(1);
    }
  }

  // GroupSyncRead present positions
  int comm_result = groupSyncRead.txRxPacket();
  if (comm_result != COMM_SUCCESS) {
    std::cerr << this->packetHandler->getTxRxResult(comm_result) << std::endl;
  }

  std::vector<uint32_t> received_data;
  // Check if GroupSyncRead data is available for each motor and save value
  for (auto id : ids) {
    if (!groupSyncRead.isAvailable(id, address, byte_size)) {
      std::cerr << "[ID: " << id << "] GroupSyncRead GetData failed"
                << std::endl;
      std::exit(1);
    }

    received_data.push_back(groupSyncRead.getData(id, address, byte_size));
  }

  return received_data;
}
