#ifndef LINUX_BOARD_CODE_HPP
#define LINUX_BOARD_CODE_HPP


#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <cmath>
#include <cstdint>

#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <poll.h>
#include <errno.h>

#include <algorithm>
#include <cctype>
#include <filesystem>
#include <set>
#include <sstream>

#define X_STEP_LIMIT 55000
#define Y_STEP_LIMIT 54500

struct Point {
    int32_t x;
    int32_t y;
};

using SINGLE_TOOL_POINTS = std::vector<Point>;
using ALL_TOOLS          = std::vector<SINGLE_TOOL_POINTS>;

constexpr double dxf_unit_to_um = 25.4 * 100; // inches to steps (each step is 0.0001 meters)

// ---------------- UART helpers ----------------

static void uart_write_all(int fd, const void* data, size_t size)
{
    const uint8_t* p = static_cast<const uint8_t*>(data);
    while (size > 0) {
        ssize_t n = write(fd, p, size);
        if (n < 0) {
            perror("write");
            return;
        }
        p += n;
        size -= static_cast<size_t>(n);
    }
}

// Read exactly N bytes with timeout (ms). Returns true on success.
static bool uart_read_exact(int fd, void* out, size_t nbytes, int timeout_ms)
{
    uint8_t* outb = static_cast<uint8_t*>(out);
    size_t got = 0;

    while (got < nbytes) {
        struct pollfd pfd{};
        pfd.fd = fd;
        pfd.events = POLLIN;

        int pr = poll(&pfd, 1, timeout_ms);
        if (pr == 0) return false;          // timeout
        if (pr < 0) {
            if (errno == EINTR) continue;
            perror("poll(uart)");
            return false;
        }

        if (pfd.revents & POLLIN) {
            ssize_t r = read(fd, outb + got, nbytes - got);
            if (r < 0) {
                if (errno == EINTR) continue;
                perror("read");
                return false;
            }
            if (r == 0) return false; // unexpected EOF-ish
            got += static_cast<size_t>(r);
        }
    }
    return true;
}

// Non-blocking check if UART has at least 1 byte available (timeout_ms may be 0)
static bool uart_has_data(int fd, int timeout_ms)
{
    struct pollfd pfd{};
    pfd.fd = fd;
    pfd.events = POLLIN;
    int pr = poll(&pfd, 1, timeout_ms);
    return (pr > 0) && (pfd.revents & POLLIN);
}

// Non-blocking check if stdin has input available
static bool stdin_has_data(int timeout_ms)
{
    struct pollfd pfd{};
    pfd.fd = STDIN_FILENO;
    pfd.events = POLLIN;
    int pr = poll(&pfd, 1, timeout_ms);
    return (pr > 0) && (pfd.revents & POLLIN);
}

static bool send_state_request_and_wait_ack(int uart_fd, uint8_t requested_state, int timeout_ms = 2000)
{
    // Send: 0x55 <state>
    uint8_t msg[2] = {0x55, requested_state};
    uart_write_all(uart_fd, msg, sizeof(msg));

    // Wait for: 0x55 <state>, while skipping progress frames 0x55 0x07
    int remaining_ms = timeout_ms;
    while (remaining_ms > 0) {
        int step_timeout = (remaining_ms > 100) ? 100 : remaining_ms;
        uint8_t ack[2] = {0, 0};
        if (!uart_read_exact(uart_fd, ack, sizeof(ack), step_timeout)) {
            remaining_ms -= step_timeout;
            continue;
        }

        if (ack[0] == 0x55 && ack[1] == requested_state) {
            return true;
        }

        if (ack[0] == 0x55 && ack[1] == 0x07) {
            uint8_t progress_payload[4];
            if (!uart_read_exact(uart_fd, progress_payload, sizeof(progress_payload), step_timeout)) {
                return false;
            }
        }
    }

    return false;
}

static bool wait_for_mcu_state(int uart_fd, uint8_t expected_state, int timeout_ms = 0)
{
    // If timeout_ms == 0, this becomes a blocking wait (poll waits forever)
    uint8_t msg[2] = {0, 0};
    int wait_ms = timeout_ms;

    if (timeout_ms == 0) {
        // Block forever in 1s chunks so we can handle EINTR cleanly
        while (true) {
            if (uart_read_exact(uart_fd, msg, sizeof(msg), 1000)) {
                if (msg[0] == 0x55 && msg[1] == expected_state) return true;
            }
        }
    }

    // Timed wait
    if (!uart_read_exact(uart_fd, msg, sizeof(msg), wait_ms)) return false;
    return (msg[0] == 0x55) && (msg[1] == expected_state);
}

static bool send_speed_and_wait_ack(int uart_fd, uint8_t speed_level, int timeout_ms = 2000)
{
    if (speed_level < 1 || speed_level > 10) {
        return false;
    }

    uint8_t msg[2] = {0x56, speed_level};
    uart_write_all(uart_fd, msg, sizeof(msg));

    uint8_t ack[2] = {0, 0};
    if (!uart_read_exact(uart_fd, ack, sizeof(ack), timeout_ms)) {
        return false;
    }

    return (ack[0] == 0x56) && (ack[1] == speed_level);
}

// Your existing toolpath transfer framing
static void send_toolpaths(int uart_fd, const ALL_TOOLS& all_tools)
{
    constexpr uint8_t START_TRANSFER = 0xAA;
    constexpr uint8_t TOOL_BEGIN     = 0xAB;
    constexpr uint8_t TOOL_END       = 0xAC;
    constexpr uint8_t END_TRANSFER   = 0xAF;

    uart_write_all(uart_fd, &START_TRANSFER, 1);

    for (const auto& tool : all_tools) {
        uart_write_all(uart_fd, &TOOL_BEGIN, 1);

        uint16_t count = static_cast<uint16_t>(tool.size());
        uart_write_all(uart_fd, &count, sizeof(count)); // little-endian

        for (const auto& p : tool) {
            uart_write_all(uart_fd, &p.x, sizeof(int32_t));
            uart_write_all(uart_fd, &p.y, sizeof(int32_t));
        }

        uart_write_all(uart_fd, &TOOL_END, 1);
    }

    uart_write_all(uart_fd, &END_TRANSFER, 1);
}

static int open_and_configure_uart(const char* dev)
{
    int fd = open(dev, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        perror("open UART");
        return -1;
    }

    struct termios tty{};
    if (tcgetattr(fd, &tty) != 0) {
        perror("tcgetattr");
        close(fd);
        return -1;
    }

    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD);
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    tty.c_iflag = 0;
    tty.c_oflag = 0;
    tty.c_lflag = 0;

    // Make reads return quickly (so state 2 can poll)
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 0;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        perror("tcsetattr");
        close(fd);
        return -1;
    }

    return fd;
}

// ---------------- DXF parsing (your logic, just wrapped) ----------------

static bool parse_dxf_to_all_tools(const char* path, ALL_TOOLS& all_tools_out)
{
    std::ifstream file(path);
    if (!file) {
        std::cerr << "Error: could not open file: " << path << "\n";
        return false;
    }

    std::string code, value;
    bool in_entities_section = false;
    bool in_lwpolyline = false;
    bool in_polyline = false;     // Old-format POLYLINE support
    bool in_vertex = false;        // Within a POLYLINE, reading VERTEX
    double cord_in_inches = 0;

    SINGLE_TOOL_POINTS current_tool_points;
    ALL_TOOLS all_tools;

    while (std::getline(file, code) && std::getline(file, value)) {

        // ENTITIES section tracking
        if ((code == "  2") && (value == "ENTITIES")) {
            in_entities_section = true;
        }
        if ((code == "  0") && (value == "ENDSEC")) {
            in_entities_section = false;
        }

        // end of LWPOLYLINE or POLYLINE
        if ((in_lwpolyline || in_polyline) && (code == "  0")) {
            if (value == "SEQEND" && in_polyline) {
                // End of old-format POLYLINE
                in_polyline = false;
                in_vertex = false;
                all_tools.push_back(current_tool_points);
                current_tool_points.clear();
            } else if (in_lwpolyline) {
                // End of LWPOLYLINE (different entity follows)
                all_tools.push_back(current_tool_points);
                current_tool_points.clear();
                in_lwpolyline = false;
            }
        }

        // beginning of LWPOLYLINE
        if (in_entities_section && (code == "  0") && (value == "LWPOLYLINE")) {
            current_tool_points.clear();
            in_lwpolyline = true;
        }
        
        // beginning of old-format POLYLINE
        if (in_entities_section && (code == "  0") && (value == "POLYLINE")) {
            current_tool_points.clear();
            in_polyline = true;
            in_vertex = false;
        }

        // VERTEX entity within POLYLINE
        if (in_polyline && (code == "  0") && (value == "VERTEX")) {
            in_vertex = true;
        }

        // Handle coordinate codes
        if ((in_lwpolyline || in_vertex) && (code == " 10")) {
            Point current_point{};
            cord_in_inches = std::stod(value);
            current_point.x = static_cast<int32_t>(std::llround(cord_in_inches * dxf_unit_to_um));

            //check if point is out of bounds of CNC X limits
            if (current_point.x < 0 || current_point.x > X_STEP_LIMIT) {
                std::cerr << "Warning: point x=" << current_point.x << " is out of bounds. Limit is"
                 << X_STEP_LIMIT << " steps). Clamping.\n";
                if (current_point.x < 0) {
                    current_point.x = 0;
                } else {
                    current_point.x = X_STEP_LIMIT;
                }
            }

            if (!std::getline(file, code)) break;
            if (!std::getline(file, value)) break;

            if (code == " 20") {
                cord_in_inches = std::stod(value);
                current_point.y = static_cast<int32_t>(std::llround(cord_in_inches * dxf_unit_to_um));

                //check if point is out of bounds of CNC Y limits
                if (current_point.y < 0 || current_point.y > Y_STEP_LIMIT) {
                    std::cerr << "Warning: point y=" << current_point.y << " is out of bounds. Limit is"
                     << Y_STEP_LIMIT << " steps). Clamping.\n";
                    if (current_point.y < 0) {
                        current_point.y = 0;
                    } else {
                        current_point.y = Y_STEP_LIMIT;
                    }
                }

                current_tool_points.push_back(current_point);
                if (in_vertex) {
                    in_vertex = false;  // Done with this VERTEX
                }
            }
        }
    }

    all_tools_out = std::move(all_tools);
    return true;
}

// ---------------- main + state machine ----------------

static std::vector<std::filesystem::path> list_dxf_files(const std::filesystem::path& dir)
{
    std::vector<std::filesystem::path> files;

    if (!std::filesystem::exists(dir) || !std::filesystem::is_directory(dir)) {
        return files;
    }

    for (const auto& entry : std::filesystem::directory_iterator(dir)) {
        if (!entry.is_regular_file()) continue;

        std::string ext = entry.path().extension().string();
        std::transform(ext.begin(), ext.end(), ext.begin(),
            [](unsigned char c) { return static_cast<char>(std::tolower(c)); });

        if (ext == ".dxf") {
            files.push_back(entry.path());
        }
    }

    std::sort(files.begin(), files.end(),
        [](const std::filesystem::path& a, const std::filesystem::path& b) {
            return a.filename().string() < b.filename().string();
        });

    return files;
}

static bool parse_index_choice(const std::string& line, size_t max_value, size_t& out_index_1based)
{
    std::istringstream iss(line);
    size_t idx = 0;
    char trailing = 0;
    if (!(iss >> idx) || (iss >> trailing)) {
        return false;
    }
    if (idx == 0 || idx > max_value) {
        return false;
    }
    out_index_1based = idx;
    return true;
}

#endif