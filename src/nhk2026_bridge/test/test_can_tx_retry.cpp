#include "ros2can_bridge.hpp"

#include <array>
#include <cerrno>
#include <chrono>
#include <cstdarg>
#include <cstring>
#include <fcntl.h>
#include <functional>
#include <iostream>
#include <sstream>
#include <time.h>

namespace
{
using Nanoseconds = std::chrono::nanoseconds;
using Milliseconds = std::chrono::milliseconds;
constexpr int fake_socket = 100;

void require(bool condition, const char *expression, int line)
{
    if (!condition) {
        throw std::runtime_error(
            "check failed at line " + std::to_string(line) + ": " + expression);
    }
}

#define CHECK(expression) require(static_cast<bool>(expression), #expression, __LINE__)

struct Reply
{
    ssize_t bytes{CANFD_MTU};
    int error{0};
    Nanoseconds duration{0};
};

struct Attempt
{
    canfd_frame frame;
    Nanoseconds time;
};

Nanoseconds now{0};
Nanoseconds oversleep{0};
std::vector<Reply> replies;
Reply fallback;
std::vector<Attempt> attempts;
std::vector<Nanoseconds> sleeps;
int socket_count = 0;
int close_count = 0;

void reset(std::vector<Reply> outcomes = {}, Reply final_reply = {})
{
    now = Nanoseconds{0};
    oversleep = Nanoseconds{0};
    replies = std::move(outcomes);
    fallback = final_reply;
    attempts.clear();
    sleeps.clear();
}

std::string failure(const std::function<void()> &operation)
{
    try {
        operation();
    } catch (const std::runtime_error &error) {
        return error.what();
    }
    throw std::runtime_error("operation unexpectedly succeeded");
}

struct Sender
{
    const char *name;
    int id;
    std::vector<uint8_t> payload;
    std::function<void(CanBridge &)> send;
    std::function<void(CanBridge &)> send_oversize;
};

void check_frames(const Sender &sender)
{
    canfd_frame expected{};
    expected.can_id = sender.id;
    expected.len = sender.payload.size();
    expected.flags = CANFD_BRS;
    std::memcpy(expected.data, sender.payload.data(), sender.payload.size());
    CHECK(!attempts.empty());
    for (const auto &attempt : attempts) {
        CHECK(std::memcmp(&attempt.frame, &expected, sizeof(expected)) == 0);
    }
}

void check_diagnostic(const std::string &message, const Sender &sender, int error, int timeout)
{
    std::ostringstream id;
    id << "CAN ID=0x" << std::hex << sender.id;
    for (const auto &part : {
        std::string("failed to write: ") + std::strerror(error),
        std::string("interface=vcan-test"), id.str(),
        std::string("length=") + std::to_string(sender.payload.size()),
        std::string("errno=") + std::to_string(error),
        std::string("attempts=") + std::to_string(attempts.size()),
        std::string("retry_timeout_ms=") + std::to_string(timeout)}) {
        CHECK(message.find(part) != std::string::npos);
    }
}

void check_sender(const Sender &sender)
{
    CanBridge bridge("vcan-test", CanBridge::SocketMode::NonBlocking);
    reset();
    sender.send(bridge);
    CHECK(attempts.size() == 1 && sleeps.empty());
    CHECK(now == Nanoseconds{0});
    check_frames(sender);

    for (const int error : {ENOBUFS, EAGAIN, EWOULDBLOCK, EINTR}) {
        reset({{-1, error}, {-1, error}, {CANFD_MTU, 0}});
        sender.send(bridge);
        CHECK(attempts.size() == 3 && sleeps.size() == 2);
        CHECK(now == Milliseconds{2});
        CHECK(attempts[0].time == Milliseconds{0});
        CHECK(attempts[1].time == Milliseconds{1});
        CHECK(attempts[2].time == Milliseconds{2});
        check_frames(sender);

        reset({}, {-1, error});
        const auto message = failure([&]() {sender.send(bridge);});
        CHECK(attempts.size() == 5 && sleeps.size() == 5);
        CHECK(now == Milliseconds{5});
        for (std::size_t index = 0; index < attempts.size(); ++index) {
            CHECK(attempts[index].time == Milliseconds{index});
            CHECK(sleeps[index] == Milliseconds{1});
        }
        check_frames(sender);
        check_diagnostic(message, sender, error, 5);
    }

    reset({{-1, EINTR}, {-1, EAGAIN}, {-1, ENOBUFS}, {CANFD_MTU, 0}});
    sender.send(bridge);
    CHECK(attempts.size() == 4 && now == Milliseconds{3});
    check_frames(sender);

    for (const int error : {ENETDOWN, ENODEV, EBADF, EINVAL, EIO}) {
        reset({}, {-1, error});
        const auto message = failure([&]() {sender.send(bridge);});
        CHECK(attempts.size() == 1 && sleeps.empty());
        check_frames(sender);
        check_diagnostic(message, sender, error, 5);
    }

    for (const ssize_t bytes : {ssize_t{0}, ssize_t{CAN_MTU}, ssize_t{CANFD_MTU - 1}}) {
        reset({}, {bytes, ENOBUFS});
        const auto message = failure([&]() {sender.send(bridge);});
        CHECK(attempts.size() == 1 && sleeps.empty());
        CHECK(message.find("unexpected frame size " + std::to_string(bytes)) != std::string::npos);
        check_frames(sender);
    }

    reset({}, {-1, ENOBUFS, Milliseconds{6}});
    auto message = failure([&]() {sender.send(bridge);});
    CHECK(attempts.size() == 1 && sleeps.empty());
    CHECK(now == Milliseconds{6});
    check_diagnostic(message, sender, ENOBUFS, 5);

    reset({}, {-1, ENOBUFS});
    oversleep = Milliseconds{10};
    message = failure([&]() {sender.send(bridge);});
    CHECK(attempts.size() == 1 && sleeps.size() == 1);
    CHECK(now == Milliseconds{11});
    check_diagnostic(message, sender, ENOBUFS, 5);

    reset();
    CHECK(failure([&]() {sender.send_oversize(bridge);}) == "Data Length is too long");
    CHECK(attempts.empty() && sleeps.empty());

    CanBridge no_retry("vcan-test", CanBridge::SocketMode::Blocking, 0);
    reset();
    sender.send(no_retry);
    CHECK(attempts.size() == 1 && sleeps.empty());
    for (const int error : {ENOBUFS, EAGAIN, EWOULDBLOCK, EINTR}) {
        reset({}, {-1, error});
        message = failure([&]() {sender.send(no_retry);});
        CHECK(attempts.size() == 1 && sleeps.empty());
        check_diagnostic(message, sender, error, 0);
    }

    CanBridge short_retry("vcan-test", CanBridge::SocketMode::NonBlocking, 2);
    reset({{-1, ENOBUFS}, {-1, ENOBUFS}, {CANFD_MTU, 0}});
    message = failure([&]() {sender.send(short_retry);});
    CHECK(attempts.size() == 2 && now == Milliseconds{2});
    CHECK(sleeps.size() == 2);
    check_frames(sender);
    check_diagnostic(message, sender, ENOBUFS, 2);
    std::cout << "PASS " << sender.name << ": retry classification, deadlines, payload and diagnostics\n";
}
}  // namespace

extern "C" int __wrap_socket(int domain, int type, int protocol)
{
    CHECK(domain == PF_CAN && type == SOCK_RAW && protocol == CAN_RAW);
    ++socket_count;
    return fake_socket;
}

extern "C" int __wrap_ioctl(int fd, unsigned long request, ...)
{
    CHECK(fd == fake_socket);
    va_list args;
    va_start(args, request);
    auto *ifr = va_arg(args, ifreq *);
    va_end(args);
    if (request == SIOCGIFINDEX) {
        ifr->ifr_ifindex = 37;
    } else {
        CHECK(request == SIOCGIFMTU);
        ifr->ifr_mtu = CANFD_MTU;
    }
    return 0;
}

extern "C" int __wrap_setsockopt(int fd, int level, int option, const void *value, socklen_t size)
{
    CHECK(fd == fake_socket && level == SOL_CAN_RAW && option == CAN_RAW_FD_FRAMES);
    CHECK(size == sizeof(int) && *static_cast<const int *>(value) == 1);
    return 0;
}

extern "C" int __wrap_bind(int fd, const sockaddr *address, socklen_t size)
{
    CHECK(fd == fake_socket && size == sizeof(sockaddr_can));
    const auto *can_address = reinterpret_cast<const sockaddr_can *>(address);
    CHECK(can_address->can_family == AF_CAN && can_address->can_ifindex == 37);
    return 0;
}

extern "C" int __wrap_fcntl(int fd, int command, ...)
{
    CHECK(fd == fake_socket);
    if (command == F_GETFL) {
        return 0;
    }
    CHECK(command == F_SETFL);
    va_list args;
    va_start(args, command);
    const int flags = va_arg(args, int);
    va_end(args);
    CHECK((flags & O_NONBLOCK) != 0);
    return 0;
}

extern "C" int __wrap_close(int fd)
{
    CHECK(fd == fake_socket);
    ++close_count;
    return 0;
}

extern "C" ssize_t __wrap_write(int fd, const void *buffer, size_t size)
{
    CHECK(fd == fake_socket && size == sizeof(canfd_frame));
    const auto reply = attempts.size() < replies.size() ? replies[attempts.size()] : fallback;
    Attempt attempt{};
    std::memcpy(&attempt.frame, buffer, size);
    attempt.time = now;
    attempts.push_back(attempt);
    now += reply.duration;
    errno = reply.error;
    return reply.bytes;
}

// Replace the libstdc++ clock and sleeps only in this test executable.
extern "C" std::chrono::steady_clock::time_point
__wrap__ZNSt6chrono3_V212steady_clock3nowEv() noexcept
{
    return std::chrono::steady_clock::time_point(now);
}

extern "C" int __wrap_nanosleep(const timespec *request, timespec *)
{
    const auto duration = std::chrono::seconds(request->tv_sec) + Nanoseconds{request->tv_nsec};
    CHECK(duration > Nanoseconds{0});
    sleeps.push_back(duration);
    now += duration + oversleep;
    errno = EIO;  // Verify that the write error survives later syscall changes.
    return 0;
}

int main()
{
    try {
        const std::array<Sender, 3> senders{{
            {"int", 0x401, {0, 0, 0, 56, 0xff, 0xff, 0xff, 0xff},
                [](CanBridge &bridge) {bridge.send_int(0x401, {56, -1});},
                [](CanBridge &bridge) {bridge.send_int(0x401, std::vector<int>(17));}},
            {"float", 0x200, {0, 0, 0, 0, 0x3f, 0x80, 0, 0, 0xbf, 0x80, 0, 0, 0x3f, 0, 0, 0},
                [](CanBridge &bridge) {bridge.send_float(0x200, {0.0f, 1.0f, -1.0f, 0.5f});},
                [](CanBridge &bridge) {bridge.send_float(0x200, std::vector<float>(17));}},
            {"bytes", 0x300, {0, 0xff, 7},
                [](CanBridge &bridge) {bridge.send_bytes(0x300, {0, 0xff, 7});},
                [](CanBridge &bridge) {bridge.send_bytes(0x300, std::vector<uint8_t>(65));}}
        }};
        for (const auto &sender : senders) {
            check_sender(sender);
        }
        for (const int invalid_timeout : {-1, 1001}) {
            const auto count = socket_count;
            bool rejected = false;
            try {
                CanBridge bridge("vcan-test", CanBridge::SocketMode::Blocking, invalid_timeout);
            } catch (const std::invalid_argument &) {
                rejected = true;
            }
            CHECK(rejected && socket_count == count);
        }
        {
            CanBridge maximum("vcan-test", CanBridge::SocketMode::NonBlocking, 1000);
            reset();
            maximum.send_int(0x401, {56});
            CHECK(attempts.size() == 1 && sleeps.empty());
        }
        CHECK(socket_count == close_count);
        std::cout << "PASS timeout bounds and cleanup; no real sockets or wall-clock sleeps\n";
        return 0;
    } catch (const std::exception &error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
