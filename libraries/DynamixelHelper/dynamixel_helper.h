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

        void ledEnable(uint8_t id);
        void ledDisable(uint8_t id);
        void groupLedEnable(uint8_t *ids, size_t ids_size);
        void groupLedDisable(uint8_t *ids, size_t ids_size);


        void torqueEnable(uint8_t id);
        void torqueDisable(uint8_t id);
        void groupTorqueEnable(uint8_t *ids, size_t ids_size);
        void groupTorqueDisable(uint8_t *ids, size_t ids_size);

        void setAngle(uint8_t id, double val);
        double getAngle(uint8_t id);
        void groupSetAngle(uint8_t *ids, size_t ids_size, double *vals);
        double* groupGetAngle(uint8_t *ids, size_t ids_size);

    private:
        // Communication handlers
        dynamixel::PortHandler *portHandler;
        dynamixel::PacketHandler *packetHandler;

        // Communication results/errors
        int comm_result = COMM_TX_FAIL;
        bool addparam_result = false;
        bool getdata_result = false;
        uint8_t error = 0;

        // Private methods
        void writeMotor(uint8_t id, uint16_t address, uint32_t data, uint16_t byte_size);
        uint32_t readMotor(uint8_t id, uint16_t address);
        void groupWriteMotor(uint8_t *ids, size_t ids_size, uint32_t *data, uint16_t address, uint16_t byte_size);
        uint32_t* groupReadMotor(uint8_t *ids, size_t ids_size, uint16_t address, uint16_t byte_size);
};

#endif
