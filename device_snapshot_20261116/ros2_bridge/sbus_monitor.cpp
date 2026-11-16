/**
 * @file sbus_monitor.cpp
 * @brief SBUS channel monitor used to map sticks/switches to CH1..CH18.
 */

#include <array>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>

#include <asm/termbits.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>

struct SbusFrame {
    std::array<uint16_t, 18> channel{};
    bool frame_lost{false};
    bool failsafe{false};
};

class SbusReader {
public:
    explicit SbusReader(const std::string& device) {
        fd_ = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (fd_ < 0) throw std::runtime_error("cannot open SBUS device: " + device);
        termios2 tio{};
        if (::ioctl(fd_, TCGETS2, &tio) < 0) throw std::runtime_error("TCGETS2 failed");
        tio.c_iflag = IGNPAR;
        tio.c_oflag = 0;
        tio.c_lflag = 0;
        tio.c_cflag &= ~(CBAUD | CSIZE | PARODD);
        tio.c_cflag |= BOTHER | CS8 | PARENB | CSTOPB | CLOCAL | CREAD;
        tio.c_ispeed = 100000;
        tio.c_ospeed = 100000;
        tio.c_cc[VMIN] = 0;
        tio.c_cc[VTIME] = 0;
        if (::ioctl(fd_, TCSETS2, &tio) < 0) throw std::runtime_error("TCSETS2 failed");
    }
    ~SbusReader() { if (fd_ >= 0) ::close(fd_); }

    bool poll(SbusFrame& output) {
        uint8_t input[128];
        bool got_frame = false;
        for (;;) {
            const auto n = ::read(fd_, input, sizeof(input));
            if (n <= 0) break;
            for (ssize_t i = 0; i < n; ++i) {
                if (position_ == 0) {
                    if (input[i] != 0x0f) continue;
                    buffer_[position_++] = input[i];
                } else {
                    buffer_[position_++] = input[i];
                    if (position_ == buffer_.size()) {
                        if (decode(output)) got_frame = true;
                        position_ = 0;
                    }
                }
            }
        }
        return got_frame;
    }
private:
    bool decode(SbusFrame& out) const {
        if (buffer_[0] != 0x0f) return false;
        for (int ch = 0; ch < 16; ++ch) {
            const int bit = ch * 11;
            const int byte_index = 1 + bit / 8;
            const int shift = bit % 8;
            uint32_t value = buffer_[byte_index];
            if (byte_index + 1 < 23) value |= static_cast<uint32_t>(buffer_[byte_index + 1]) << 8;
            if (byte_index + 2 < 23) value |= static_cast<uint32_t>(buffer_[byte_index + 2]) << 16;
            out.channel[ch] = static_cast<uint16_t>((value >> shift) & 0x07ff);
        }
        out.channel[16] = (buffer_[23] & 0x01) ? 2047 : 0;
        out.channel[17] = (buffer_[23] & 0x02) ? 2047 : 0;
        out.frame_lost = (buffer_[23] & 0x04) != 0;
        out.failsafe = (buffer_[23] & 0x08) != 0;
        return true;
    }
    int fd_{-1};
    std::array<uint8_t, 25> buffer_{};
    std::size_t position_{0};
};

const char* posName(uint16_t v) {
    if (v <= 800) return "LOW";
    if (v >= 1199) return "HIGH";
    return "MID";
}

int main(int argc, char** argv) {
    const std::string dev = argc > 1 ? argv[1] : "/dev/ttyS1";
    SbusReader sbus(dev);
    SbusFrame f;
    std::cout << "SBUS monitor on " << dev << "\n"
              << "XML uses CH1..CH18. Old factory logs ch0 == CH1.\n"
              << "Move one stick/switch at a time and watch which CH changes. Ctrl+C to stop.\n";
    auto last = std::chrono::steady_clock::now() - std::chrono::milliseconds(1000);
    while (true) {
        if (sbus.poll(f)) {
            auto now = std::chrono::steady_clock::now();
            if (now - last >= std::chrono::milliseconds(200)) {
                for (int i = 0; i < 18; ++i) {
                    std::cout << "CH" << (i+1) << '=' << f.channel[i] << '(' << posName(f.channel[i]) << ")";
                    if (i != 17) std::cout << ' ';
                }
                std::cout << " frame_lost=" << f.frame_lost << " failsafe=" << f.failsafe << std::endl;
                last = now;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
}
