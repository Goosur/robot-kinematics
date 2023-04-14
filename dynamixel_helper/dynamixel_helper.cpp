#include "dynamixel_helper.h"
#include <cmath>
#include <iostream>

DynamixelHelper::DynamixelHelper(const char *port) {
  this->portHandler = dynamixel::PortHandler::getPortHandler(port);
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

void DynamixelHelper::groupLedEnable(vector<uint8_t> ids) {
  // Convert ids vector into basic array for low level dynamixel communication
  auto *ids_array = new uint8_t[ids.size()];
  for (int i = 0; i < ids.size(); i++)
    ids_array[i] = ids[i];

  uint32_t data[ids.size()];
  for (int i = 0; i < ids.size(); i++)
    data[i] = 1;

  this->groupWriteMotor(ids_array, ids.size(), data, 65, 1);

  // Clean up
  delete[] ids_array;
}

void DynamixelHelper::groupLedDisable(vector<uint8_t> ids) {
  // Convert ids vector into basic array for low level dynamixel communication
  auto *ids_array = new uint8_t[ids.size()];
  for (int i = 0; i < ids.size(); i++)
    ids_array[i] = ids[i];

  uint32_t data[ids.size()];
  for (int i = 0; i < ids.size(); i++)
    data[i] = 0;

  this->groupWriteMotor(ids_array, ids.size(), data, 65, 1);

  // Clean up
  delete[] ids_array;
}

void DynamixelHelper::torqueEnable(uint8_t id) {
  this->writeMotor(id, 64, 1, 1);
  std::clog << "[ID: " << (int)id << "] Torque Enabled" << std::endl;
}

void DynamixelHelper::torqueDisable(uint8_t id) {
  this->writeMotor(id, 64, 0, 1);
  std::clog << "[ID: " << (int)id << "] Torque Disabled" << std::endl;
}

void DynamixelHelper::groupTorqueEnable(vector<uint8_t> ids) {
  // Convert ids vector into basic array for low level dynamixel communication
  auto *ids_array = new uint8_t[ids.size()];
  for (int i = 0; i < ids.size(); i++)
    ids_array[i] = ids[i];

  uint32_t data[ids.size()];
  for (int i = 0; i < ids.size(); i++)
    data[i] = 1;

  this->groupWriteMotor(ids_array, ids.size(), data, 64, 1);

  // Clean up
  delete[] ids_array;
}

void DynamixelHelper::groupTorqueDisable(vector<uint8_t> ids) {
  // Convert ids vector into basic array for low level dynamixel communication
  auto *ids_array = new uint8_t[ids.size()];
  for (int i = 0; i < ids.size(); i++)
    ids_array[i] = ids[i];

  uint32_t data[ids.size()];
  for (int i = 0; i < ids.size(); i++)
    data[i] = 0;

  this->groupWriteMotor(ids_array, ids.size(), data, 64, 1);

  // Clean up
  delete[] ids_array;
}

void DynamixelHelper::setAngle(uint8_t id, double val) {
  this->writeMotor(id, 116, (uint32_t)(val * 180.0 / M_PI / 0.088), 4);
}

double DynamixelHelper::getAngle(uint8_t id) {
  return readMotor(id, 132) * 0.088 * M_PI / 180.0;
}

void DynamixelHelper::groupSetAngle(vector<uint8_t> ids, vector<double> vals) {
  // Convert ids vector into basic array for low level dynamixel communication
  auto *ids_array = new uint8_t[ids.size()];
  for (int i = 0; i < ids.size(); i++)
    ids_array[i] = ids[i];

  uint32_t data[ids.size()];

  for (int i = 0; i < ids.size(); i++)
    data[i] = (uint32_t)(vals[i] * 180.0 / M_PI / 0.088);

  groupWriteMotor(ids_array, ids.size(), data, 116, 4);

  // Clean up
  delete[] ids_array;
}

vector<double> DynamixelHelper::groupGetAngle(vector<uint8_t> ids) {
  // Convert ids vector into basic array for low level dynamixel communication
  auto *ids_array = new uint8_t[ids.size()];
  for (int i = 0; i < ids.size(); i++)
    ids_array[i] = ids[i];

  uint32_t *retrieved_data = groupReadMotor(ids_array, ids.size(), 132, 4);
  vector<double> present_positions;
  for (int i = 0; i < ids.size(); i++)
    present_positions.push_back(retrieved_data[i] * 0.088 * M_PI / 180.0);

  return present_positions;
}

/*
 *
 * PRIVATE METHODS
 *
 */

void DynamixelHelper::writeMotor(uint8_t id, uint16_t address, uint32_t data,
                                 uint16_t byte_size) {
  if (byte_size == 1)
    this->comm_result = this->packetHandler->write1ByteTxRx(
        this->portHandler, id, address, data, &this->error);
  else if (byte_size == 4)
    this->comm_result = this->packetHandler->write4ByteTxRx(
        this->portHandler, id, address, data, &this->error);

  if (this->comm_result != COMM_SUCCESS) {
    std::cerr << this->packetHandler->getTxRxResult(this->comm_result)
              << std::endl;
    std::exit(1);
  } else if (this->error != 0) {
    std::cerr << this->packetHandler->getRxPacketError(this->error)
              << std::endl;
    std::exit(1);
  }
}

uint32_t DynamixelHelper::readMotor(uint8_t id, uint16_t address) {
  uint32_t present_position;

  this->comm_result = this->packetHandler->read4ByteTxRx(
      this->portHandler, id, address, &present_position, &this->error);
  if (this->comm_result != COMM_SUCCESS) {
    std::cerr << this->packetHandler->getTxRxResult(this->comm_result)
              << std::endl;
    std::exit(1);
  } else if (this->error != 0) {
    std::cerr << this->packetHandler->getRxPacketError(this->error)
              << std::endl;
    std::exit(1);
  }

  return present_position;
}

void DynamixelHelper::groupWriteMotor(uint8_t *ids, size_t ids_size,
                                      uint32_t *data, uint16_t address,
                                      uint16_t byte_size) {
  dynamixel::GroupSyncWrite groupSyncWrite(
      this->portHandler, this->packetHandler, address, byte_size);

  uint8_t param_data[byte_size];
  for (int i = 0; i < (int)ids_size; i++) {
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
    this->addparam_result = groupSyncWrite.addParam(ids[i], param_data);
    if (!this->addparam_result) {
      std::cerr << "[ID: " << ids[i] << "] GroupSyncWrite AddParam failed"
                << std::endl;
      std::exit(1);
    }
  }

  // Send write packet
  this->comm_result = groupSyncWrite.txPacket();
  if (this->comm_result != COMM_SUCCESS) {
    std::cerr << this->packetHandler->getTxRxResult(this->comm_result)
              << std::endl;
  }

  // Clear stored write values
  groupSyncWrite.clearParam();
}

uint32_t *DynamixelHelper::groupReadMotor(uint8_t *ids, size_t ids_size,
                                          uint16_t address,
                                          uint16_t byte_size) {
  dynamixel::GroupSyncRead groupSyncRead(portHandler, packetHandler, address,
                                         byte_size);

  uint32_t *received_data = new uint32_t[ids_size];

  // Add motors to groupsyncread for present position reading
  for (int i = 0; i < (int)ids_size; i++) {
    this->addparam_result = groupSyncRead.addParam(ids[i]);
    if (!this->addparam_result) {
      std::cerr << "[ID: " << ids[i] << "] GroupSyncRead AddParam failed"
                << std::endl;
      std::exit(1);
    }
  }

  // GroupSyncRead present positions
  this->comm_result = groupSyncRead.txRxPacket();
  if (this->comm_result != COMM_SUCCESS) {
    std::cerr << this->packetHandler->getTxRxResult(this->comm_result)
              << std::endl;
  }

  // Check if GroupSyncRead data is available for each motor and save value
  for (int i = 0; i < (int)ids_size; i++) {
    this->getdata_result =
        groupSyncRead.isAvailable(ids[i], address, byte_size);
    if (!this->getdata_result) {
      std::cerr << "[ID: " << ids[i] << "] GroupSyncRead GetData failed"
                << std::endl;
      std::exit(1);
    }

    received_data[i] = groupSyncRead.getData(ids[i], address, byte_size);
  }

  return received_data;
}
