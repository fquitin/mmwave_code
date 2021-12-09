//
// Copyright ULB BEAMS-EE
// Author: François QUITIN
//


#include <uhd/exception.hpp>
#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/utils/safe_main.hpp>
#include <uhd/utils/static.hpp>
#include <uhd/utils/thread.hpp>
#include <stdint.h>
#include <boost/algorithm/string.hpp>
#include <boost/format.hpp>
#include <boost/math/special_functions/round.hpp>
#include <boost/program_options.hpp>
#include <chrono>
#include <csignal>
#include <iostream>
#include <string>
#include <thread>

#include <fstream>

#include "constants.h"
#include "aip_functions.h"
#include "/usr/local/include/libserial/SerialPort.h"
using namespace LibSerial ;

namespace po = boost::program_options;

bool stop_signal_called = false;




/***********************************************************************
 * Main function
 **********************************************************************/
int UHD_SAFE_MAIN(int argc, char* argv[])
{
    // variables to be set by po
    std::string 	name_serial_port;
    int 			ver_aip;

    int 		nbr_directions = 3;
    
 
    
    // The following vector contains the phase shift between antennas (in degrees)
    std::string possible_degrees[17] = {"DEG_0","DEG_11_25","DEG_22_25","DEG_33_75","DEG_45","DEG_56_25","DEG_67_5","DEG_78_75","DEG_90","DEG_101_2","DEG_112_5","DEG_123_7","DEG_135","DEG_146_2","DEG_157_5","DEG_168_7","DEG_180"};	
    // The beam directions corresponding to the previous phase shifts are given below (in degrees)
    std::string possible_angles[17] = {"0.00", "4.00", "8.00", "11.50", "15.50", "19.50", "23.50", "28.00", "32.50", "37.00", "41.50", "46.50", "52.00", "57.50", "64.50", "72.00", "78.00"};			
    std::string possible_directions[4] = {"LEFT", "RIGHT", "UP", "DOWN"};
    int gain = 0; 
    int gain_list[4] = {0,0,0,0};
    std::string active_list[4] = {"1111", "1111", "1111", "1111"};
    

    // setup the program options
    po::options_description desc("Allowed options");
    // clang-format off
    desc.add_options()
		("help", "help message")
		("serialport", po::value<std::string>(&name_serial_port)->default_value("/dev/ttyUSB0"), "Serial port of the mmWave array")
		("ver-aip", po::value<int>(&ver_aip)->default_value(0), "verbose mmWave arrays on or off")
        
    ;
    // clang-format on
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    // print the help message
    if (vm.count("help")) {
        std::cout << boost::format("mmWave array tester") << std::endl;
        return ~0;
    }
    

    
    // ======================================
    // Open serial port of the mmWave array
    // ======================================
    // Create and open the serial port for communication with the mmWave array.
    std::cout << boost::format("Create and open the serial port for mmWave array on %s...") % name_serial_port << std::endl;
    SerialPort my_serial_port( name_serial_port );
    std::string my_string;  
    // Set serial port parameters 
    std::cout << boost::format("Set serial port parameters ...") << std::endl;
    my_serial_port.SetBaudRate( LibSerial::BaudRate::BAUD_115200 );
    my_serial_port.SetCharacterSize( LibSerial::CharacterSize::CHAR_SIZE_8 );
    my_serial_port.SetStopBits( LibSerial::StopBits::STOP_BITS_1 ) ;
    my_serial_port.SetParity( LibSerial::Parity::PARITY_NONE );
    

	int mode_init = 2;
	std::cout << boost::format("Setting AiP to %s - %s °") % "UP" % "0"  << std::endl;
    send_to_aip(&my_serial_port, "DEG_0", "UP", gain_list, gain, active_list, mode_init, ver_aip);
    std::cout << boost::format("Setting AiP to %s - %s °") % "UP" % "0"  << std::endl;
    send_to_aip(&my_serial_port, "DEG_0", "UP", gain_list, gain, active_list, mode_init, ver_aip);
    std::cout << boost::format("Setting AiP to %s - %s °") % "LEFT" % "0"  << std::endl;
    send_to_aip(&my_serial_port, "DEG_0", "LEFT", gain_list, gain, active_list, mode_init, ver_aip);
	
    
  
	// ==============================================================
	// Start looping over all AiP directions and Rx baseband samples
	// ==============================================================
	
	
	init_aip(&my_serial_port, ver_aip);
	
	std::string degrees;
	std::string angle; 
	std::string direction;
	int mode = 2; // 0 for TX/RX off, 1 for TX, 2 for RX
	direction = possible_directions[0];
	for (int cpt_directions = 16; cpt_directions > -1; cpt_directions--)
    //for (int cpt_directions = 0; cpt_directions < nbr_directions; cpt_directions++)
    {
    	// Setting AiP beam direction
    	degrees = possible_degrees[cpt_directions];
    	angle = possible_angles[cpt_directions];	
    	std::cout << boost::format("Setting AiP to %s - %s °") % direction % angle  << std::endl;
    	send_to_aip_fast(&my_serial_port, degrees, direction, gain_list, gain, active_list, mode, ver_aip);
    	
		usleep(100000);
	}
	
	
    
    // Disable AiP
    disable_aip(&my_serial_port, ver_aip);
    
    // Close serial port
    std::cout << std::endl << "Close serial port ..." << std::endl << std::endl;
    my_serial_port.Close();
    

    // finished
    std::cout << std::endl << "Done!" << std::endl << std::endl;
    return EXIT_SUCCESS;
}
