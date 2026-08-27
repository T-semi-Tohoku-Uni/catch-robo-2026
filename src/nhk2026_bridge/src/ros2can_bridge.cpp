#include "ros2can_bridge.hpp"

#include <cerrno>
#include <poll.h>

CanBridge::CanBridge(const std::string &Ifname)
: ifname(Ifname), sock(-1)
{
    this->sock = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (this->sock < 0)
    {
        throw std::runtime_error("failed to open socket");
    }

    int enable_canfd = 1;
    if (setsockopt(this->sock, SOL_CAN_RAW, CAN_RAW_FD_FRAMES,
    &enable_canfd, sizeof(enable_canfd)) < 0)
    {
        close(this->sock);
        throw std::runtime_error("failed to socketopt canfd");
    }

    ifreq ifr{};
    std::strncpy(ifr.ifr_name, ifname.c_str(), IFNAMSIZ - 1);
    if (ioctl(this->sock, SIOCGIFINDEX, &ifr) < 0)
    {
        close(this->sock);
        throw std::runtime_error("failed to use ioctl");
    }
    sockaddr_can addr{};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(this->sock, (sockaddr*)&addr, sizeof(addr)) < 0)
    {
        close(this->sock);
        throw std::runtime_error("failed to bind");
    }
}

CanBridge::~CanBridge()
{
    if (this->sock >= 0)
    {
        close(this->sock);
        this->sock = -1;
    }
}

void CanBridge::send_float(int canid, const std::vector<float> &txdata_f)
{
    int byte_length = (int)txdata_f.size() * 4;
    if (byte_length > 64)
    {
        throw std::runtime_error("Data Length is too long");
    }

    canfd_frame frame{};
    frame.can_id = canid;
    frame.len = byte_length;
    frame.flags |= CANFD_BRS;
    for (int i = 0; i < (int)txdata_f.size(); i++)
    {
        union Data data;
        data.data_f32 = txdata_f[i];
        frame.data[i*4    ] = (uint8_t)((data.data_ui32 >> 24) & 0xff);
        frame.data[i*4 + 1] = (uint8_t)((data.data_ui32 >> 16) & 0xff);
        frame.data[i*4 + 2] = (uint8_t)((data.data_ui32 >>  8) & 0xff);
        frame.data[i*4 + 3] = (uint8_t)((data.data_ui32      ) & 0xff);
    }
    int nbytes = write(this->sock, &frame, sizeof(frame));
    if (nbytes != sizeof(frame))
    {
        throw std::runtime_error("failed to write");
    }
}

void CanBridge::send_int(int canid, const std::vector<int> &txdata_i)
{
    int byte_length = (int)txdata_i.size() *4;
    if (byte_length > 64)
    {
        throw std::runtime_error("Data Length is too long");
    }

    canfd_frame frame{};
    frame.can_id = canid;
    frame.len = byte_length;
    frame.flags |= CANFD_BRS;
    for (size_t i = 0; i < txdata_i.size(); i++)
    {
        uint32_t val = static_cast<uint32_t>(txdata_i[i]);
        frame.data[i*4    ] = (uint8_t)((val >> 24) & 0xff);
        frame.data[i*4 + 1] = (uint8_t)((val >> 16) & 0xff);
        frame.data[i*4 + 2] = (uint8_t)((val >>  8) & 0xff);
        frame.data[i*4 + 3] = (uint8_t)((val      ) & 0xff);
    }
    int nbytes = write(this->sock, &frame, sizeof(frame));
    if (nbytes != sizeof(frame))
    {
        throw std::runtime_error("failed to write");
    }
}

void CanBridge::send_bytes(int canid, const std::vector<uint8_t> &txdata_b)
{
    const int byte_length = static_cast<int>(txdata_b.size());
    if (byte_length > 64)
    {
        throw std::runtime_error("Data Length is too long");
    }

    canfd_frame frame{};
    frame.can_id = canid;
    frame.len = byte_length;
    frame.flags |= CANFD_BRS;

    if (byte_length > 0)
    {
        std::memcpy(frame.data, txdata_b.data(), static_cast<size_t>(byte_length));
    }

    int nbytes = write(this->sock, &frame, sizeof(frame));
    if (nbytes != sizeof(frame))
    {
        throw std::runtime_error("failed to write");
    }
}

bool CanBridge::receive_data(RxData_struct &out)
{
    for (;;)
    {
        if (this->sock < 0)
        {
            return false;
        }

        pollfd pfd{};
        pfd.fd = this->sock;
        pfd.events = POLLIN;
        const int pr = poll(&pfd, 1, 200); // 200ms timeout to allow shutdown
        if (pr < 0)
        {
            if (errno == EINTR)
            {
                continue;
            }
            throw std::runtime_error(std::string("failed to poll: ") + std::strerror(errno));
        }
        if (pr == 0)
        {
            // timeout: no data
            return false;
        }

        canfd_frame frame{};
        const int nbytes = read(this->sock, &frame, sizeof(frame));
        if (nbytes < 0)
        {
            if (errno == EINTR)
            {
                continue;
            }
            throw std::runtime_error(std::string("failed to read: ") + std::strerror(errno));
        }
        if (nbytes != static_cast<int>(sizeof(canfd_frame)) &&
            nbytes != static_cast<int>(sizeof(can_frame)))
        {
            throw std::runtime_error("failed to read: unexpected frame size " + std::to_string(nbytes));
        }
        if (frame.len > CANFD_MAX_DLEN)
        {
            throw std::runtime_error("failed to read: invalid data length");
        }

        out.canid = frame.can_id;
        out.data.assign(frame.data, frame.data + frame.len);
        return true;
    }
}

std::vector<float> CanBridge::rxdata_to_float(const RxData_struct &rxdata)
{
    size_t vector_len = rxdata.data.size() / 4;

    std::vector<float> rxdata_f;
    rxdata_f.resize(vector_len);

    for (size_t i = 0; i < vector_len; i++)
    {
        uint32_t rxdata_ui32 = static_cast<uint32_t>(rxdata.data[i*4] << 24 | rxdata.data[i*4 + 1] << 16 | rxdata.data[i*4 + 2] << 8 | rxdata.data[i*4 + 3]);
        union Data data;
        data.data_ui32 = rxdata_ui32;
        rxdata_f[i] = data.data_f32;
    }

    return rxdata_f;
}

std::vector<int> CanBridge::rxdata_to_int(const RxData_struct &rxdata)
{
    size_t vector_len = rxdata.data.size() / 4;

    std::vector<int> rxdata_i;
    rxdata_i.resize(vector_len);

    for (size_t i = 0; i < vector_len; i++)
    {
        uint32_t rxdata_ui32 = (static_cast<uint32_t>(rxdata.data[i*4])     << 24) |
                               (static_cast<uint32_t>(rxdata.data[i*4 + 1]) << 16) |
                               (static_cast<uint32_t>(rxdata.data[i*4 + 2]) << 8)  |
                               (static_cast<uint32_t>(rxdata.data[i*4 + 3]));

        rxdata_i[i] = static_cast<int32_t>(rxdata_ui32);
    }

    return rxdata_i;
}

void CanBridge::shutdown()
{
    if (sock >= 0) close(sock);
    sock = -1;
}
