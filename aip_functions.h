//
// Copyright ULB BEAMS-EE
// Author: François QUITIN
//

#include <iostream>
#include <string>
#include "/usr/local/include/libserial/SerialPort.h"
using namespace LibSerial ;




/***********************************************************************
 * Auxiliary functions for serial communications with mmWave array
 **********************************************************************/
 
// Create string of instructions containing configuration of array operations
std::string* create_register_list(std::string degrees, std::string direction, int* gain_list, int gain, std::string* active_list, int mode)
{
	// Convert the active_list to hexadecimal equivalent for the AiP
	std::string* active_list_hex ;
	active_list_hex = active_list_to_hex(active_list);

	// Convert the gain_list to hexadecimal equivalent for the AiP
	std::string* gain_list_hex ;
	gain_list_hex = gain_list_to_hex(gain_list);

	// Convert the angles to hexadecimal equivalent for the AiP
	std::string* angle_list_hex ;
	angle_list_hex = angle_to_reg(degrees, direction);

	// Create final register list
	std::string* register_list = new std::string[4];
	for (int i=0; i<4; i++){
		register_list[i].append("000");
		register_list[i].append(std::to_string(mode));
		register_list[i].append(active_list_hex[i]);
		register_list[i].append(std::to_string(gain));
		register_list[i].append(gain_list_hex[i]);
		register_list[i].append(angle_list_hex[i]);
	}     
	//std::cout << boost::format(" REGISTER LIST: {%s, %s, %s, %s}") % register_list[0] % register_list[1] % register_list[2] % register_list[3] << std::endl;
	return register_list; 
}
 
 
 
// Write string on serial port and read response
std::string write_read_serial(SerialPort* my_serial_port, std::string my_string)
{
    int timeout_ms = 20; // timeout value in milliseconds
    char next_char;      // variable to store the read result
    std::string rx_string;
    int timeout = 0;

    // Write to serial port
    my_serial_port->Write( my_string );
    //std::cout << boost::format("  -- to serial port: %s") % my_string << std::endl;

    // Read from serial port until timeout
    rx_string = "";
    while (!(timeout)){
    	try
    	{
	    my_serial_port->ReadByte( next_char, timeout_ms );
	    //std::cout << boost::format("  -- from serial port: %s") % next_char << std::endl;
	    rx_string.push_back( next_char );
    	}
    	catch (const ReadTimeout&)
    	{	
	    
	    timeout = 1;
    	}	
    }
    //std::cout << boost::format("    -- response from serial port: %s" ) % rx_string << std::endl;
    return rx_string;
}



/***********************************************************************
 * Functions to control mmWave array
 **********************************************************************/
 
// Send command to mmWave AiP
void send_to_aip(SerialPort* my_serial_port, std::string degrees, std::string direction, int* gain_list, int gain, std::string* active_list, int mode)
{
    std::string my_string; 
    std::string* register_list = create_register_list(degrees, direction, gain_list, gain, active_list, mode);
      
    // Initialize the mmWave array package
    //std::cout << boost::format("Initialize mmWave array...") << std::endl;
    write_read_serial(my_serial_port, "AT+DUT=0158\r\0");
    write_read_serial(my_serial_port, "AT+AIPCONFIG=0202\r\0");
    write_read_serial(my_serial_port, "AT+ADRNUM=001\r\0");
    // TODO: check if all responses = AMO_OK
    
    // Initialize chip registers
    my_string = "AT+REG=";
    my_string.append(REG1);
    my_string.append("\r\0");
    for (int i=0; i<4; i++){
    	write_read_serial(my_serial_port, my_string);
    }
    // TODO: check if all responses = CHIP_OK
    write_read_serial(my_serial_port, "AT+SEND?\r\0");
    
    // Write insctructions for each chip
    for (int i=0; i<4; i++){
        my_string = "AT+REG=";
        my_string.append(register_list[i]);
        my_string.append("\r\0");
        write_read_serial(my_serial_port, my_string);
    }
    // TODO: check if all responses = CHIP_OK
    write_read_serial(my_serial_port, "AT+SEND?\r\0");
    
    // Read temperature of each chip
    my_string = "AT+REG=";
    my_string.append(REG_TEMP);
    my_string.append("\r\0");
    for (int i=0; i<4; i++){
        write_read_serial(my_serial_port, my_string);
    }
    // TODO: check if all responses = CHIP_OK
    write_read_serial(my_serial_port, "AT+SEND?\r\0");
    
    // Enable Tx or Rx
    if (mode == 1){
		//std::cout << boost::format("Enabling Tx..") << std::endl;
		write_read_serial(my_serial_port, "AT+TXEN=1\r\0");
    }
    else if (mode == 2){
        //std::cout << boost::format("Enabling Rx..") << std::endl;
		write_read_serial(my_serial_port, "AT+RXEN=1\r\0");
    }   
    else if (mode == 0){
    	//std::cout << boost::format("Disabling Tx and Rx of AiP..") << std::endl;
		write_read_serial(my_serial_port, "AT+TXEN=0\r\0");
		write_read_serial(my_serial_port, "AT+RXEN=0\r\0");
	}
    // TODO: check if all responses = AMO_OK
      
}
 
// Disable Tx/Rx of mmWave AiP
void disable_aip(SerialPort* my_serial_port)
{
    std::cout << boost::format("Disabling Tx and Rx of AiP..") << std::endl;
    write_read_serial(my_serial_port, "AT+TXEN=0\r\0");
    write_read_serial(my_serial_port, "AT+RXEN=0\r\0");
    // TODO: check if all responses = AMO_OK
}


