#include <iostream>
#include <fstream>
#include <string>

int main(int argc, char* argv[])
{
    if (argc != 2) {
        std::cerr << "Usage: dxf_dump <path_to_dxf>\n";
        return 1;
    }

    std::ifstream file(argv[1]);
    if (!file) {
        std::cerr << "Error: could not open file: " << argv[1] << "\n";
        return 1;
    }

    std::string line;
    size_t lineNumber = 1;

    while (std::getline(file, line)) {
        std::cout << lineNumber << ": " << line << "\n";
        ++lineNumber;
    }

    return 0;
}
