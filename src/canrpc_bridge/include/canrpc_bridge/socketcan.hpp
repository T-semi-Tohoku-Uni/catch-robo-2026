#pragma once

#include <cstdint>
#include <chrono>
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
    enum class SocketMode
    {
        Blocking,
        NonBlocking,
    };

    struct RxData_struct
    {
        int canid;
        std::vector<uint8_t> data;
    };

    CanBridge(
        const std::string &interface_name,
        SocketMode socket_mode,
        int tx_retry_timeout_ms = 5);
    ~CanBridge();
    void send_bytes(int canid, const std::vector<uint8_t> &txdata_b);
    bool receive_data(RxData_struct &out);

    void shutdown();

private:
    const std::string ifname;
    const std::chrono::milliseconds tx_retry_timeout_;
    int sock;
    void send_frame(const canfd_frame &frame);
};
