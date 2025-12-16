#include <iostream>
#include <fstream>
#include <string>
#include <vector>

struct Point{
	double x;
	double y;
};

using SINGLE_TOOL_POINTS = std::vector<Point>; //Each of these vectors contains points for one singe tool.
using ALL_TOOLS = std::vector<SINGLE_TOOL_POINTS>; //There is only one of these vectors and it contains all the vectors representing a tool.

int main(int argc, char* argv[])
{
    if (argc != 2) { //exacly one dxf file is expected
        std::cerr << "Usage: dxf_dump <path_to_dxf>\n";
        return 1;
    }

    std::ifstream file(argv[1]);
    if (!file) {
        std::cerr << "Error: could not open file: " << argv[1] << "\n";
        return 1;
    }
	
    std::string code, value;
	size_t poly_count = 0;
	size_t line_number = 0;
	bool in_entities_section = false;
	bool in_lwpolyline = false;
	
	SINGLE_TOOL_POINTS current_tool_points;
	ALL_TOOLS all_tools;

	
    while ((std::getline(file, code)) && (std::getline(file, value))) {
        
		//Identifying if we are in the ENTITIES section
		if((code == "  2") && (value == "ENTITIES")){
			in_entities_section = true;
			//std::cout << "in entities" << std::endl;
		} 
		if((code == "  0") && (value == "ENDSEC")){
			in_entities_section = false;
			//std::cout << "leaving entites" << std::endl;
		}
		
		//Identify if we are the end of a polyline
		if(in_lwpolyline && code == "  0"){
			//std::cout << "end of polyline" << std::endl;
			all_tools.push_back(current_tool_points);
			in_lwpolyline = false;
		}

		//Identify if we are at the beginning of a polyline
		if((in_entities_section) && (code == "  0") && (value == "LWPOLYLINE")){
			//std::cout << "begining of polyline" << std::endl;
			current_tool_points.clear();
			in_lwpolyline = true;

		} else {

			if((in_lwpolyline) && (code == " 10")){
				Point current_point;
				current_point.x = std::stod(value);
				
				//Now we need to read the next pair to grab the y cordinate
				std::getline(file, code);
				std::getline(file, value);
				if(code == " 20"){
					current_point.y = std::stod(value);
					current_tool_points.push_back(current_point);
				} else {
					std::cout << "This should never print" << std::endl;
				}
			}
		}			
    }

	std::cout << "Outlines found: " << all_tools.size() << std::endl << std::endl;
		
	for (size_t i = 0; i < all_tools.size(); i++) {
		std::cout << "Outline " << i+1
		  << " has " << all_tools[i].size()
		  << " points\n";
	}
	std::cout << std::endl;

	for(size_t i = 0; i < all_tools.size(); i++){
		std::cout << "\nOUTLINE: " << i+1 << " points" << std::endl;
		for(size_t j = 0; j < all_tools[i].size(); j++){
			std::cout << "X: " << all_tools[i][j].x << "\t\tY: " << all_tools[i][j].y << std::endl;
		}
	}
    return 0;
}

