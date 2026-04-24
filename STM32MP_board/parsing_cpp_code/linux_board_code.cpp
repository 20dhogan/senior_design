#include "linux_board_code.hpp"

int main(int argc, char* argv[])
{ //Begin int main

    size_t state = 0;

    // Optional arg: directory to scan for DXF files. Default is current directory.
    std::filesystem::path dxf_dir = std::filesystem::current_path();
    if (argc > 2) {
        std::cerr << "Usage: dxf_dump [dxf_directory]\n";
        return 1;
    }
    if (argc == 2) {
        dxf_dir = std::filesystem::path(argv[1]);
    }

    if (!std::filesystem::exists(dxf_dir) || !std::filesystem::is_directory(dxf_dir)) {
        std::cerr << "Error: directory does not exist: " << dxf_dir << "\n";
        return 1;
    }

    ALL_TOOLS all_tools;
    std::filesystem::path selected_dxf_file;
    std::set<std::string> completed_files;
    uint8_t selected_speed = 5;
    uint16_t selected_nozzle_steps = 3000;

    // NOTE: change this if needed (you already found the right one)
    int uart_fd = open_and_configure_uart("/dev/ttyUSB0");
    if (uart_fd < 0) return 1;

    // Flush any stale bytes sitting in RX (optional but helpful)
    while (uart_has_data(uart_fd, 0)) {
        uint8_t junk;
        (void)read(uart_fd, &junk, 1);
    }

    while (1) { // infinite loop

        switch (state) {

            case 0: { // Idle
                auto dxf_files = list_dxf_files(dxf_dir);
                if (dxf_files.empty()) {
                    std::cout << "\nNo .dxf files found in: " << dxf_dir << "\n";
                    std::cout << "Add files, then press Enter to rescan.\n> ";
                    std::string wait_line;
                    std::getline(std::cin, wait_line);
                    break;
                }

                std::cout << "\nIn idle state. Available DXF files in " << dxf_dir << ":\n";
                for (size_t i = 0; i < dxf_files.size(); i++) {
                    const std::string file_name = dxf_files[i].filename().string();
                    const bool completed = (completed_files.find(file_name) != completed_files.end());
                    std::cout << "  " << (i + 1) << ") " << file_name;
                    if (completed) std::cout << " [completed]";
                    std::cout << "\n";
                }

                std::cout << "Choose file number to start, R to rescan, or Q to quit.\n> ";
                std::string line;
                std::getline(std::cin, line);
                if (line.empty()) break;

                char c = line[0];
                if (c == 'q' || c == 'Q') {
                    close(uart_fd);
                    return 0;
                }
                if (c == 'r' || c == 'R') {
                    break;
                }

                size_t selected_index_1based = 0;
                if (!parse_index_choice(line, dxf_files.size(), selected_index_1based)) {
                    std::cout << "Invalid selection.\n";
                    break;
                }

                selected_dxf_file = dxf_files[selected_index_1based - 1];
                all_tools.clear();
                if (!parse_dxf_to_all_tools(selected_dxf_file.string().c_str(), all_tools)) {
                    std::cout << "Failed to parse: " << selected_dxf_file.filename().string() << "\n";
                    break;
                }

                std::cout << "Loaded " << selected_dxf_file.filename().string()
                          << " with " << all_tools.size() << " outlines.\n";
                std::cout << "How many mm should the nozzle lower? (example: 12.5)\n> ";
                std::getline(std::cin, line);
                if (line.empty()) {
                    std::cout << "No nozzle depth entered. Using 30.00 mm.\n";
                    selected_nozzle_steps = 3000;
                } else {
                    double nozzle_mm = 0.0;
                    std::istringstream depth_stream(line);
                    char trailing = 0;
                    if (!(depth_stream >> nozzle_mm) || (depth_stream >> trailing) || nozzle_mm < 0.0) {
                        std::cout << "Invalid nozzle depth. Using 30.00 mm.\n";
                        selected_nozzle_steps = 3000;
                    } else {
                        const long rounded_steps = std::lround(nozzle_mm * 100.0);
                        if (rounded_steps > 65535L) {
                            std::cout << "Nozzle depth too large. Using max 655.35 mm.\n";
                            selected_nozzle_steps = 65535;
                        } else {
                            selected_nozzle_steps = static_cast<uint16_t>(rounded_steps);
                        }
                    }
                }
                std::cout << "Choose speed (1-10), where 10 is fastest.\n> ";
                std::getline(std::cin, line);
                if (line.empty()) {
                    std::cout << "No speed entered. Using speed 5.\n";
                    selected_speed = 5;
                } else {
                    size_t speed_index_1based = 0;
                    if (!parse_index_choice(line, 10, speed_index_1based)) {
                        std::cout << "Invalid speed. Using speed 5.\n";
                        selected_speed = 5;
                    } else {
                        selected_speed = static_cast<uint8_t>(speed_index_1based);
                    }
                }
                std::cout << "Type start (S) to transfer this file, or any other key to cancel.\n> ";
                std::getline(std::cin, line);
                if (line.empty()) break;
                c = line[0];
                if (c == 's' || c == 'S') {
                    // Request state 1 from MCU
                    uint8_t req[4] = {
                        0x55,
                        0x01,
                        static_cast<uint8_t>(selected_nozzle_steps & 0xFF),
                        static_cast<uint8_t>((selected_nozzle_steps >> 8) & 0xFF)
                    };
                    uart_write_all(uart_fd, req, sizeof(req));

                    uint8_t ack[2] = {0, 0};
                    if (uart_read_exact(uart_fd, ack, sizeof(ack), 2000)) {
                        if (ack[0] == 0x55 && ack[1] == 0x01) {
                            state = 1;
                        } else if (ack[0] == 0x55 && ack[1] == 0x05) {
                            std::cout << "Can't transfer data, machine is reseting\n";
                        } else {
                            std::cout << "No ACK from microcontroller for state 1. Staying in idle.\n";
                        }
                    } else {
                        std::cout << "No ACK from microcontroller for state 1. Staying in idle.\n";
                    }
                }
                break;
            }

            case 1: { // Transfer data
                std::cout << "\nTransferring data...\n";

                // send all the data just like your cpp file did
                send_toolpaths(uart_fd, all_tools);

				//wait for signal that all transfering has been recieved
				uint8_t ack[1] = {0};
				if (!uart_read_exact(uart_fd, ack, sizeof(ack), 200000)) {
					std::cout << "Microcontroller failed to recive complete data\n";
					break;
				}
				if (ack[0] != 0x54) {
                    if (ack[0] == 0xE1) {
                        std::cout << "Transfer failed: microcontroller ran out of memory while loading toolpaths.\n";
                    } else {
                        std::cout << "Unexpected response from microcontroller: 0x" << std::hex << (int)ack[0] << std::dec << "\n";
                    }
					break;
				}
				
                std::cout << "Done transferring data to microcontroller. Type start (S) to begin cutting.\n> ";
                std::string line;
                std::getline(std::cin, line);
                if (line.empty()) break;

                char c = line[0];
                if (c == 's' || c == 'S') {
                    if (!send_speed_and_wait_ack(uart_fd, selected_speed)) {
                        std::cout << "No ACK from microcontroller for speed update. Staying in transfer state.\n";
                        break;
                    }
                    // Request state 2
                    if (send_state_request_and_wait_ack(uart_fd, 0x02)) {
                        state = 2;
                    } else {
                        std::cout << "No ACK from microcontroller for state 2. Staying in transfer state.\n";
                    }
                }
                break;
            }

            case 2: { // Cutting
                std::cout << "\nCutting...\n";
                std::cout << "Type pause (P) to pause cutting.\n";

                // Here we poll both:
                // - keyboard for P/p
                // - UART for (0x55,0x04) meaning micro says "done/stop"
                while (state == 2) {

                    // Check for UART done signal: 0x55 0x04
                    if (uart_has_data(uart_fd, 10)) {
                        uint8_t msg[2];
                        if (uart_read_exact(uart_fd, msg, 2, 10)) {
                            if (msg[0] == 0x55 && msg[1] == 0x04) {
                                state = 4;
                                break;
                            } else if (msg[0] == 0x55 && msg[1] == 0x07) {
                                uint8_t payload[4];
                                if (uart_read_exact(uart_fd, payload, sizeof(payload), 50)) {
                                    uint16_t completed = (uint16_t)payload[0] | ((uint16_t)payload[1] << 8);
                                    uint16_t total = (uint16_t)payload[2] | ((uint16_t)payload[3] << 8);
                                    std::cout << completed << "/" << total << " objects cut\n";
                                }
                            }
                            // otherwise ignore unknown messages here
                        }
                    }

                    // Check for keyboard
                    if (stdin_has_data(10)) {
                        std::string line;
                        std::getline(std::cin, line);
                        if (!line.empty()) {
                            char c = line[0];
                            if (c == 'p' || c == 'P') {
                                // Request pause state 3
                                if (send_state_request_and_wait_ack(uart_fd, 0x03)) {
                                    state = 3;
                                    break;
                                } else {
                                    std::cout << "No ACK for pause request. Still cutting.\n";
                                }
                            }
                        }
                    }
                }
                break;
            }

            case 3: { // Paused
                std::cout << "\nIn pause state\n";
                std::cout << "Press start (S) to resume or stop (X) to stop.\n> ";

                std::string line;
                std::getline(std::cin, line);
                if (line.empty()) break;

                char c = line[0];
                if (c == 's' || c == 'S') {
                    // Resume -> request state 2
                    if (send_state_request_and_wait_ack(uart_fd, 0x02, 6000)) {
                        state = 2;
                    } else {
                        std::cout << "No ACK for resume request. Staying paused.\n";
                    }
                } else if (c == 'x' || c == 'X') {
                    // Stop -> request state 4
                    if (send_state_request_and_wait_ack(uart_fd, 0x04)) {
                        state = 4;
                    } else {
                        std::cout << "No ACK for stop request. Staying paused.\n";
                    }
                }
                break;
            }

            case 4: { // Freeing memory / returning to idle when MCU says 0x55 0x00
                std::cout << "\nFreeing memory...\n";

                all_tools.clear();
                all_tools.shrink_to_fit();

                std::cout << "Remove the foam, then type ready (R) to reset the machine.\n> ";
                std::string line;
                std::getline(std::cin, line);
                if (line.empty()) break;

                char c = line[0];
                if (c == 'r' || c == 'R') {
                    if (send_state_request_and_wait_ack(uart_fd, 0x00, 6000)) {
                        if (!selected_dxf_file.empty()) {
                            completed_files.insert(selected_dxf_file.filename().string());
                        }
                        std::cout << "Foam acknowledged. Machine reset/idle started.\n";
                        state = 0;
                    } else {
                        std::cout << "No ACK from microcontroller for reset-to-idle request. Staying in free state.\n";
                    }
                }
                break;
            }

            default:
                state = 0;
                break;
        }
    }

    close(uart_fd);
    return 0;
} // end main
