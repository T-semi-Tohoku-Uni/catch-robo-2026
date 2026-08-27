#pragma once

#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <vector>
#include <stdexcept>
#include <cstring>
#include <unistd.h>
#include <string>

class CanBridge
{
public:
    struct RxData_struct
    {
        int canid;
        std::vector<uint8_t> data;
    };

    CanBridge(const std::string& Ifname);
    ~CanBridge();
    void send_float(int canid, const std::vector<float> &txdata_f);
    void send_int(int canid, const std::vector<int> &txdata_i);
    void send_bytes(int canid, const std::vector<uint8_t> &txdata_b);
    bool receive_data(RxData_struct &out);

    std::vector<float> rxdata_to_float(const RxData_struct &rxdata);
    std::vector<int> rxdata_to_int(const RxData_struct &rxdata);
    void shutdown();
    
private:
    const std::string ifname;
    int sock;
    union Data
    {
        uint32_t data_ui32;
        float data_f32;
    };
};
