#ifndef DYNAMIXEL_HELPER_H
#define DYNAMIXEL_HELPER_H

#include <dynamixel_sdk/dynamixel_sdk.h>
#include <string>
#include <vector>

class DynamixelHelper {
public:
  /**
   * Create new dynamixel helper for specified device
   */
  DynamixelHelper(std::string port);

  /**
   * @brief Close porthandler and delete handler pointers.
   */
  ~DynamixelHelper();

  /**
   * @brief Attach to physical device and open channel of communication
   */
  void openPort();

  /**
   * @brief Set communication rate of the port handler
   * @param baudrate - Number of signals sent per second
   */
  void setBaudrate(const int baudrate);

  /**
   * @brief Enable an individual motor led
   * @param id - ID of the motor of interest
   */
  void ledEnable(uint8_t id);

  /**
   * @brief Disable an individual motor led
   * @param id - ID of the motor of interest
   */
  void ledDisable(uint8_t id);

  /**
   * @brief Enable leds on a group of motors
   * @param ids - Vector of motor ids
   */
  void groupLedEnable(std::vector<uint8_t> ids);

  /**
   * @brief Disable leds on a group of motors
   * @param ids - Vector of motor ids
   */
  void groupLedDisable(std::vector<uint8_t> ids);

  /**
   * @brief Enable torque on an individual motor
   * @param id - ID of the motor of interest
   */
  void torqueEnable(uint8_t id);

  /**
   * @brief Disable torque on an individual motor
   * @param id - ID of the motor of interest
   */
  void torqueDisable(uint8_t id);

  /**
   * @brief Enable torque on a group of motors
   * @param ids - Vector of motor ids
   */
  void groupTorqueEnable(std::vector<uint8_t> ids);

  /**
   * @brief Disable torque on a group of motors
   * @param ids - Vector of motor ids
   */
  void groupTorqueDisable(std::vector<uint8_t> ids);

  /**
   * @brief Command a motor to move to a specific angle
   * @param id - ID of the motor of interest
   * @param val - Angle in radians to move to
   */
  void setAngle(uint8_t id, double val);

  /**
   * @brief Retrieve the current angle of a motor
   * @param id - ID of the motor of interest
   * @return double - The current angle of a motor in radians
   */
  double getAngle(uint8_t id);

  /**
   * @brief Command a group of motors to move to specific angles
   * @param ids - Vector of motor ids
   * @param vals - Vector of angles to move to in radians
   */
  void groupSetAngle(std::vector<uint8_t> ids, std::vector<double> vals);

  /**
   * @brief Retrieve the current angles of a group of motors
   * @param ids - Vector of motor ids
   * @return vector<double> - Current angles of the motors of interest in
   * radians
   */
  std::vector<double> groupGetAngle(std::vector<uint8_t> ids);

  /**
   * @brief Send a serial packet to write to a register of a motor
   * @param id - ID of the motor of interest
   * @param address - Memory address of the register to write to
   * @param data - Data to store in the register of interest
   * @param byte_size - Size of the data being stored
   */
  void writeMotor(uint8_t id, uint16_t address, uint32_t data,
                  uint16_t byte_size);

  /**
   * @brief Send a serial packet to request the value of a register of a motor
   * @param id - ID of the motor of interest
   * @param address - Memory address of the register to read from
   * @return uint32_t - The contents of the requested register
   */
  uint32_t readMotor(uint8_t id, uint16_t address);

  /**
   * @brief Send a serial packet to simultaneously write to a register of
   * multiple motors
   * @param ids - Vector of IDs of the motors of interest
   * @param data - Vector of data to write to the motor registers
   * @param address - Address to write to for every motor
   * @param byte_size - Size of the register to write
   */
  void groupWriteMotor(std::vector<uint8_t> ids, std::vector<uint32_t> data,
                       uint16_t address, uint16_t byte_size);

  /**
   * @brief Send a serial packet to simultaneously read from a register of
   * multiple motors
   * @param ids - Vector of IDs of the motors of interest
   * @param address - Address to read from for every motor
   * @param byte_size - Size of the register to read from
   * @return std::vector<uint8_t> - Vector of joint angles read from the motors
   */
  std::vector<uint32_t> groupReadMotor(std::vector<uint8_t> ids,
                                       uint16_t address, uint16_t byte_size);

private:
  // Serial communication handlers
  dynamixel::PortHandler *portHandler;
  dynamixel::PacketHandler *packetHandler;
};

#endif
