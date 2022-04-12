#ifndef DYNAMIXEL_HELPER_H
#define DYNAMIXEL_HELPER_H

#include "dynamixel_sdk.h"

class DynamixelHelper
{
    public:
        DynamixelHelper(const char *port);
        ~DynamixelHelper();
        void openPort();
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
		 * @param ids - Array of motor ids
		 * @param ids_size - Size of the motor id array
		 */
        void groupLedEnable(uint8_t *ids, size_t ids_size);

		/**
		 * @brief Disable leds on a group of motors
		 * @param ids - Array of motor ids
		 * @param ids_size - Size of the motor id array
		 */
        void groupLedDisable(uint8_t *ids, size_t ids_size);


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
		 * @param ids - Array of motor ids
		 * @param ids_size - Size of the motor id array
		 */
        void groupTorqueEnable(uint8_t *ids, size_t ids_size);

		/**
		 * @brief Disable torque on a group of motors
		 * @param ids - Array of motor ids
		 * @param ids_size - Size of the motor id array
		 */
        void groupTorqueDisable(uint8_t *ids, size_t ids_size);

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
		 * @param ids - Array of motor ids
		 * @param ids_size - Size of the motor id array
		 * @param vals - Array of angles to move to in radians
		 */
        void groupSetAngle(uint8_t *ids, size_t ids_size, double *vals);

		/**
		 * @brief Retrieve the current angles of a group of motors
		 * @param ids - Array of motor ids
		 * @param ids_size - Size of the motor id array
		 * @return double - Current angles of the motors of interest in radians
		 */
        double* groupGetAngle(uint8_t *ids, size_t ids_size);

    private:
        // Serial communication handlers
        dynamixel::PortHandler *portHandler;
        dynamixel::PacketHandler *packetHandler;

        // Communication results/errors
        int comm_result = COMM_TX_FAIL;
        bool addparam_result = false;
        bool getdata_result = false;
        uint8_t error = 0;

        // Private methods
		/**
		 * @brief Send a serial packet to write to a register of a motor
		 * @param id - ID of the motor of interest
		 * @param address - Memory address of the register to write to
		 * @param data - Data to store in the register of interest
		 * @param byte_size - Size of the data being stored
		 */
        void writeMotor(uint8_t id, uint16_t address, uint32_t data, uint16_t byte_size);

		/**
		 * @brief Send a serial packet to request the value of a register of a motor
		 * @param id - ID of the motor of interest
		 * @param address - Memory address of the register to read from
		 * @return uint32_t - The contents of the requested register
		 */
        uint32_t readMotor(uint8_t id, uint16_t address);

		/**
		 * @brief Send a serial packet to simultaneously write to a register of multiple motors
		 * @param ids - Array of IDs of the motors of interest
		 * @param ids_size - Size of the motor id array
		 * @param data - Array of data to write to the motor registers
		 * @param address - Address to write to for every motor
		 * @param byte_size - Size of the register to write
		 */
        void groupWriteMotor(uint8_t *ids, size_t ids_size, uint32_t *data, uint16_t address, uint16_t byte_size);

		/**
		 * @brief Send a serial packet to simultaneously read from a register of multiple motors
		 * @param ids - Array of IDs of the motors of interest
		 * @param ids_size - Size of the motor id array
		 * @param address - Address to read from for every motor
		 * @param byte_size - Size of the register to read from
		 * @return
		 */
        uint32_t* groupReadMotor(uint8_t *ids, size_t ids_size, uint16_t address, uint16_t byte_size);
};

#endif
