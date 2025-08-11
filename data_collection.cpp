#include <iostream>
#include <sstream>
#include <fstream>
#include <limits>
#include <fcntl.h>    // File control
#include <termios.h>  // Terminal control, tcflush
#include <unistd.h>   // Read/write
#include <cstring>    // memset
#include <thread>     // std::this_thread
#include <chrono>     // std::chrono

enum Move {
    Rock = 0,
    Paper = 1,
    Scissors = 2
};

// std::vector<int> parse_ints(const std::string& s) {
//     // parse string s and populate result array with each line's number
//     std::vector<int> result;
//     size_t start = 0;
//     size_t end;

//     while ((end = s.find('\n', start)) != std::string::npos) {
//         if (end > start) {  // Ignore empty lines
//             result.push_back(std::stoi(s.substr(start, end - start)));
//         }
//         start = end + 1;
//     }

//     // Handle final line if no trailing newline
//     if (start < s.size()) {
//         result.push_back(std::stoi(s.substr(start)));
//     }

//     return result;
// }

bool file_exists (const std::string& filename) {
    std::ifstream f(filename);
    return f.good();
}

void stripNewlines(char* s) {
    size_t len = std::strlen(s);
    while (len > 0 && (s[len - 1] == '\n' || s[len - 1] == '\r')) {
        s[--len] = '\0';
    }
}

int main() {
    int GESTURE = Rock; // Paper Scissors
    int NUM_LOOPS = 5;
    if (GESTURE == 0) {
        std::cout << "bet\n";
    }
    const char* portname = "/dev/tty.usbmodemB0818497F0182";  // Change this to your actual port
    const char* data_file_name = "./data/train.csv"; // change this to your actual data file
    std::cout << "Connecting to " << portname << std::endl;
    int serial_port = open(portname, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (serial_port < 0) {
        std::cerr << "Error " << errno << " opening " << portname << ": " << strerror(errno) << std::endl;
        return 1;
    }

    // Configure serial port
    termios tty;
    memset(&tty, 0, sizeof tty);

    if (tcgetattr(serial_port, &tty) != 0) {
        std::cerr << "Error from tcgetattr: " << strerror(errno) << std::endl;
        return 1;
    }

    // Set Baud Rate
    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);

    // 8N1 mode
    tty.c_cflag &= ~PARENB; // No parity
    tty.c_cflag &= ~CSTOPB; // One stop bit
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;     // 8 data bits

    // Turn off flow control
    tty.c_cflag &= ~CRTSCTS;

    tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines

    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO;    // Disable echo
    tty.c_lflag &= ~ECHOE;
    tty.c_lflag &= ~ISIG;    // Disable interpretation of INTR, QUIT and SUSP

    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);

    tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes
    tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed

    // Set blocking read
    tty.c_cc[VMIN]  = 1;    // Wait for at least 1 character
    tty.c_cc[VTIME] = 20;    // 1s timeout

    if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
        std::cerr << "Error from tcsetattr: " << strerror(errno) << std::endl;
        return 1;
    }
    std::cout << "Connected and configured " << portname << std::endl;

    // open data stream for appending
    std::ofstream data_file;
    if (file_exists(data_file_name)) {
        data_file.open(data_file_name, std::ios::app); // append
    } else {
        data_file.open(data_file_name, std::ios::out); // create and write
        data_file << "label, features\n";
    }

    if (!data_file) {
        std::cerr << "Failed to open the data file.\n";
        return 1;
    }

    int buf_size = 256;
    char read_buf[buf_size+1];
    std::__1::chrono::steady_clock::time_point ttime;
    int num_vals = 20;

    std::cout << "Starting loop..." << std::endl;
    tcflush(serial_port, TCIFLUSH);
    for (int z = 0; z < NUM_LOOPS; z++) {

        data_file << GESTURE << ',';
        memset(read_buf, '\0', buf_size+1);
        int n = 0;

        while (n < num_vals) {
            int num_bytes = read(serial_port, &read_buf, sizeof(read_buf));
            if (num_bytes > 0) {
                try {
                    int val = std::stoi(read_buf);
                    std::string s = "";
                    if (n < num_vals-1) {
                        s.append(",");
                    }
                    data_file << val << s;
                    n++;
                } catch (...) {
                    continue;
                }
            }
        }
        data_file << std::endl;
    }

    data_file.close();
    close(serial_port);
    return 0;
}
