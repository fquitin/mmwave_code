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

#include "constants.h"
#include "/usr/local/include/libserial/SerialPort.h"
using namespace LibSerial ;

namespace po = boost::program_options;



/***********************************************************************
 * Signal handlers
 **********************************************************************/
static bool stop_signal_called = false;
void sig_int_handler(int)
{
    stop_signal_called = true;
}


/***********************************************************************
 * Auxiliary functions for serial communications with mmWave array
 **********************************************************************/
 
 // Create string of instructions for array operations
 std::string* create_register_list(std::string degrees, std::string direction, int* gain_list, int gain, std::string* active_list, int mode)
 {
     // Convert the active_list to hexadecimal equivalent for the AiP
     std::string active_list_hex[4] = {"","","",""};
     for (int i=0; i<4; i++){
     	if (active_list[i] == "0000"){active_list_hex[i] = "f"; }
     	else if(active_list[i] == "0001"){active_list_hex[i] = "e"; }
     	else if(active_list[i] == "0010"){active_list_hex[i] = "d"; }
     	else if(active_list[i] == "0011"){active_list_hex[i] = "c"; }
     	else if(active_list[i] == "0100"){active_list_hex[i] = "b"; }
     	else if(active_list[i] == "0101"){active_list_hex[i] = "a"; }
     	else if(active_list[i] == "0110"){active_list_hex[i] = "9"; }
     	else if(active_list[i] == "0111"){active_list_hex[i] = "8"; }
     	else if(active_list[i] == "1000"){active_list_hex[i] = "7"; }
     	else if(active_list[i] == "1001"){active_list_hex[i] = "6"; }
     	else if(active_list[i] == "1010"){active_list_hex[i] = "5"; }
     	else if(active_list[i] == "1011"){active_list_hex[i] = "4"; }
     	else if(active_list[i] == "1100"){active_list_hex[i] = "3"; }
     	else if(active_list[i] == "1101"){active_list_hex[i] = "2"; }
     	else if(active_list[i] == "1110"){active_list_hex[i] = "1"; }
     	else if(active_list[i] == "1111"){active_list_hex[i] = "0"; }
     }
     // Convert the gain_list to hexadecimal equivalent for the AiP
     std::string gain_list_hex[4] = {"","","",""};
     for (int i=0; i<4; i++){
     	if (gain_list[i] == 0){gain_list_hex[i] = "0000"; }
     	else if(gain_list[i] == 1){gain_list_hex[i] = "1111"; }
     	else if(gain_list[i] == 2){gain_list_hex[i] = "2222"; }
     	else if(gain_list[i] == 3){gain_list_hex[i] = "3333"; }
     	else if(gain_list[i] == 4){gain_list_hex[i] = "4444"; }
     	else if(gain_list[i] == 5){gain_list_hex[i] = "5555"; }
     	else if(gain_list[i] == 6){gain_list_hex[i] = "6666"; }
     	else if(gain_list[i] == 7){gain_list_hex[i] = "7777"; }
     	else if(gain_list[i] == 8){gain_list_hex[i] = "8888"; }
     	else if(gain_list[i] == 9){gain_list_hex[i] = "9999"; }
     	else if(gain_list[i] == 10){gain_list_hex[i] = "aaaa"; }
     	else if(gain_list[i] == 11){gain_list_hex[i] = "bbbb"; }
     	else if(gain_list[i] == 12){gain_list_hex[i] = "cccc"; }
     	else if(gain_list[i] == 13){gain_list_hex[i] = "dddd"; }
     	else if(gain_list[i] == 14){gain_list_hex[i] = "eeee"; }
     	else if(gain_list[i] == 15){gain_list_hex[i] = "ffff"; }
     }
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
     std::cout << boost::format(" REGISTER LIST: {%s, %s, %s, %s}") % register_list[0] % register_list[1] % register_list[2] % register_list[3] << std::endl;
     return register_list; 
 }
 
 
 
// Write string on serial port and read response
std::string write_read_serial(SerialPort* my_serial_port, std::string my_string)
{
    int timeout_ms = 5; // timeout value in milliseconds
    char next_char;      // variable to store the read result
    std::string rx_string;
    int timeout = 0;

    // Write to serial port
    my_serial_port->Write( my_string );
    std::cout << boost::format("  -- to serial port: %s") % my_string << std::endl;

    // Read from serial port until timeout
    rx_string = "";
    while (!(timeout)){
    	try
    	{
	    my_serial_port->ReadByte( next_char, timeout_ms );
	    std::cout << boost::format("  -- from serial port: %s") % next_char << std::endl;
	    rx_string.push_back( next_char );
    	}
    	catch (const ReadTimeout&)
    	{	
	    
	    timeout = 1;
    	}	
    }
    std::cout << boost::format("    -- response from serial port: %s" ) % rx_string << std::endl;
    return rx_string;
}
 
 
// Send command to mmWave AiP
void send_to_aip(SerialPort* my_serial_port, std::string degrees, std::string direction, int* gain_list, int gain, std::string* active_list, int mode)
{
    std::string my_string; 
    std::string* register_list = create_register_list(degrees, direction, gain_list, gain, active_list, mode);
      
    // Initialize the mmWave array package
    std::cout << boost::format("Initialize mmWave array...") << std::endl;
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
	std::cout << boost::format("Enabling Tx..") << std::endl;
	write_read_serial(my_serial_port, "AT+TXEN=1\r\0");
    }
    else if (mode == 0){
        std::cout << boost::format("Enabling Rx..") << std::endl;
	write_read_serial(my_serial_port, "AT+RXEN=1\r\0");
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
 


/***********************************************************************
 * Main function
 **********************************************************************/
int UHD_SAFE_MAIN(int argc, char* argv[])
{
    // variables to be set by po
    std::string args, ant1, ant2, subdev, ref, pps, channel_list, name_serial_port;
    uint64_t total_num_samps;
    size_t spb;
    double rate, freq1, gain1, freq2, gain2;

    // setup the program options
    po::options_description desc("Allowed options");
    // clang-format off
    desc.add_options()
        ("help", "help message")
        ("args", po::value<std::string>(&args)->default_value("addr=192.168.192.30"), "single uhd device address args")
        ("spb", po::value<size_t>(&spb)->default_value(0), "samples per buffer, 0 for default")
        ("nsamps", po::value<uint64_t>(&total_num_samps)->default_value(0), "total number of samples to transmit (0 for infinite)")
        ("rate", po::value<double>(&rate)->default_value(1000000), "rate of outgoing samples")
        ("freq1", po::value<double>(&freq1)->default_value(4000000000), "RF chain 1 center frequency in Hz")
	("gain1", po::value<double>(&gain1)->default_value(0), "gain for the RF chain 1")
	("ant1", po::value<std::string>(&ant1)->default_value("TX/RX"), "antenna selection RF chain 1")
	("freq2", po::value<double>(&freq2)->default_value(6000000000), "RF chain 2 center frequency in Hz")
	("gain2", po::value<double>(&gain2)->default_value(31.5), "gain for the RF chain 2")
        ("ant2", po::value<std::string>(&ant2)->default_value("TX/RX"), "antenna selection RF chain 2")
        ("subdev", po::value<std::string>(&subdev)->default_value("A:0 B:0"), "subdevice specification")
        ("ref", po::value<std::string>(&ref)->default_value("internal"), "clock reference (internal, external, mimo, gpsdo)")
        ("pps", po::value<std::string>(&pps)->default_value("internal"), "PPS source (internal, external, mimo, gpsdo)")
        ("channels", po::value<std::string>(&channel_list)->default_value("0,1"), "which channels to use (specify \"0\", \"1\", \"0,1\", etc)")
	("serialport", po::value<std::string>(&name_serial_port)->default_value("/dev/ttyUSB0"), "Serial port of the mmWave array")
        
    ;
    // clang-format on
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    // print the help message
    if (vm.count("help")) {
        std::cout << boost::format("mmWave Tx %s") % desc << std::endl;
        return ~0;
    }
    
    
    /***********************************************************************
     * Open serial port of the mmWave array
     **********************************************************************/

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

    
    /***********************************************************************
     * The following code is to be done for each operation of the mmWave array
     **********************************************************************/
     
    std::string possible_degrees[17] = {"DEG_0","DEG_11_25","DEG_22_25","DEG_33_75","DEG_45","DEG_56_25","DEG_67_5","DEG_78_75","DEG_90","DEG_101_2","DEG_112_5","DEG_123_7","DEG_135","DEG_146_2","DEG_157_5","DEG_168_7","DEG_180"};				
    std::string possible_directions[4] = {"UP", "DOWN", "LEFT", "RIGHT"};
   
    int gain = 1; 
    int gain_list[4] = {0,0,0,0};
    std::string active_list[4] = {"1111", "1111", "1111", "1111"};
    int mode = 1;
    send_to_aip(&my_serial_port, possible_degrees[0], possible_directions[2], gain_list, gain, active_list, mode);
    
    /***********************************************************************
     *  
     **********************************************************************/
    
    

    
    // create a usrp device
    std::cout << std::endl;
    std::cout << boost::format("Creating the usrp device with: %s...") % args << std::endl;
    uhd::usrp::multi_usrp::sptr usrp = uhd::usrp::multi_usrp::make(args);
    
    // always select the subdevice first, the channel mapping affects the other settings
    if (vm.count("subdev"))
        usrp->set_tx_subdev_spec(subdev);

    // detect which channels to use
    std::vector<std::string> channel_strings;
    std::vector<size_t> channel_nums;
    boost::split(channel_strings, channel_list, boost::is_any_of("\"',"));
    for (size_t ch = 0; ch < channel_strings.size(); ch++) {
        size_t chan = std::stoi(channel_strings[ch]);
        if (chan >= usrp->get_tx_num_channels())
            throw std::runtime_error("Invalid channel(s) specified.");
        else
            channel_nums.push_back(std::stoi(channel_strings[ch]));
    }

    // Lock mboard clocks
    if (vm.count("ref")) {
        usrp->set_clock_source(ref);
    }
    
    std::cout << boost::format("Using Device: %s") % usrp->get_pp_string() << std::endl;

    // set the sample rate
    std::cout << boost::format("Setting TX Rate: %f Msps...") % (rate / 1e6) << std::endl;
    usrp->set_tx_rate(rate);
    std::cout << boost::format("Actual TX Rate: %f Msps...") % (usrp->get_tx_rate() / 1e6)
              << std::endl
              << std::endl;

    // set the center frequency, rf gain and antenna for RF chain 1
    std::cout << boost::format("Setting RF chain 1 TX Freq: %f MHz...") % (freq1 / 1e6) << std::endl;
    uhd::tune_request_t tune_request_1(freq1);
    usrp->set_tx_freq(tune_request_1, 0);
    std::cout << boost::format("Actual RF chain 1 TX Freq: %f MHz...") % (usrp->get_tx_freq(0) / 1e6) << std::endl << std::endl;
    
    std::cout << boost::format("Setting RF chain 1 TX Gain: %f dB...") % gain1 << std::endl;
    usrp->set_tx_gain(gain1, 0);
    std::cout << boost::format("Actual RF chain 1 TX Gain: %f dB...") % usrp->get_tx_gain(0) << std::endl << std::endl;
    
    usrp->set_tx_antenna(ant1, 0);
    
    
    // set the center frequency, rf gain and antenna for RF chain 2
    std::cout << boost::format("Setting RF chain 2 TX Freq: %f MHz...") % (freq2 / 1e6) << std::endl;
    uhd::tune_request_t tune_request_2(freq2);
    usrp->set_tx_freq(tune_request_2, 1);
    std::cout << boost::format("Actual RF chain 2 TX Freq: %f MHz...") % (usrp->get_tx_freq(0) / 1e6) << std::endl << std::endl;
    
    std::cout << boost::format("Setting RF chain 2 TX Gain: %f dB...") % gain2 << std::endl;
    usrp->set_tx_gain(gain2, 1);
    std::cout << boost::format("Actual RF chain 2 TX Gain: %f dB...") % usrp->get_tx_gain(0) << std::endl << std::endl;
    
    usrp->set_tx_antenna(ant2, 1);
    
    std::this_thread::sleep_for(std::chrono::seconds(1)); // allow for some setup time
    



    /*
    // pre-compute the waveform values
    const wave_table_class wave_table(wave_type, ampl);
    const size_t step =
        boost::math::iround(wave_freq / usrp->get_tx_rate() * wave_table_len);
    size_t index = 0;

    // create a transmit streamer
    // linearly map channels (index0 = channel0, index1 = channel1, ...)
    uhd::stream_args_t stream_args("fc32", otw);
    stream_args.channels             = channel_nums;
    uhd::tx_streamer::sptr tx_stream = usrp->get_tx_stream(stream_args);

    // allocate a buffer which we re-use for each channel
    if (spb == 0) {
        spb = tx_stream->get_max_num_samps() * 10;
    }
    std::vector<std::complex<float>> buff(spb);
    std::vector<std::complex<float>*> buffs(channel_nums.size(), &buff.front());

    // pre-fill the buffer with the waveform
    for (size_t n = 0; n < buff.size(); n++) {
        buff[n] = wave_table(index += step);
    }

    std::cout << boost::format("Setting device timestamp to 0...") << std::endl;
    if (channel_nums.size() > 1) {
        // Sync times
        if (pps == "mimo") {
            UHD_ASSERT_THROW(usrp->get_num_mboards() == 2);

            // make mboard 1 a slave over the MIMO Cable
            usrp->set_time_source("mimo", 1);

            // set time on the master (mboard 0)
            usrp->set_time_now(uhd::time_spec_t(0.0), 0);

            // sleep a bit while the slave locks its time to the master
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        } else {
            if (pps == "internal" or pps == "external" or pps == "gpsdo")
                usrp->set_time_source(pps);
            usrp->set_time_unknown_pps(uhd::time_spec_t(0.0));
            std::this_thread::sleep_for(
                std::chrono::seconds(1)); // wait for pps sync pulse
        }
    } else {
        usrp->set_time_now(0.0);
    }

    // Check Ref and LO Lock detect
    std::vector<std::string> sensor_names;
    const size_t tx_sensor_chan = channel_nums.empty() ? 0 : channel_nums[0];
    sensor_names                = usrp->get_tx_sensor_names(tx_sensor_chan);
    if (std::find(sensor_names.begin(), sensor_names.end(), "lo_locked")
        != sensor_names.end()) {
        uhd::sensor_value_t lo_locked = usrp->get_tx_sensor("lo_locked", tx_sensor_chan);
        std::cout << boost::format("Checking TX: %s ...") % lo_locked.to_pp_string()
                  << std::endl;
        UHD_ASSERT_THROW(lo_locked.to_bool());
    }
    const size_t mboard_sensor_idx = 0;
    sensor_names                   = usrp->get_mboard_sensor_names(mboard_sensor_idx);
    if ((ref == "mimo")
        and (std::find(sensor_names.begin(), sensor_names.end(), "mimo_locked")
                != sensor_names.end())) {
        uhd::sensor_value_t mimo_locked =
            usrp->get_mboard_sensor("mimo_locked", mboard_sensor_idx);
        std::cout << boost::format("Checking TX: %s ...") % mimo_locked.to_pp_string()
                  << std::endl;
        UHD_ASSERT_THROW(mimo_locked.to_bool());
    }
    if ((ref == "external")
        and (std::find(sensor_names.begin(), sensor_names.end(), "ref_locked")
                != sensor_names.end())) {
        uhd::sensor_value_t ref_locked =
            usrp->get_mboard_sensor("ref_locked", mboard_sensor_idx);
        std::cout << boost::format("Checking TX: %s ...") % ref_locked.to_pp_string()
                  << std::endl;
        UHD_ASSERT_THROW(ref_locked.to_bool());
    }

    std::signal(SIGINT, &sig_int_handler);
    std::cout << "Press Ctrl + C to stop streaming..." << std::endl;

    // Set up metadata. We start streaming a bit in the future
    // to allow MIMO operation:
    uhd::tx_metadata_t md;
    md.start_of_burst = true;
    md.end_of_burst   = false;
    md.has_time_spec  = true;
    md.time_spec      = usrp->get_time_now() + uhd::time_spec_t(0.1);

    // send data until the signal handler gets called
    // or if we accumulate the number of samples specified (unless it's 0)
    uint64_t num_acc_samps = 0;
    while (true) {
        // Break on the end of duration or CTRL-C
        if (stop_signal_called) {
            break;
        }
        // Break when we've received nsamps
        if (total_num_samps > 0 and num_acc_samps >= total_num_samps) {
            break;
        }

        // send the entire contents of the buffer
        num_acc_samps += tx_stream->send(buffs, buff.size(), md);

        // fill the buffer with the waveform
        for (size_t n = 0; n < buff.size(); n++) {
            buff[n] = wave_table(index += step);
        }

        md.start_of_burst = false;
        md.has_time_spec  = false;
    }

    // send a mini EOB packet
    md.end_of_burst = true;
    tx_stream->send("", 0, md);
	
    */
    
    
    // Disable AiP
    disable_aip(&my_serial_port);
    
    // Close serial port
    std::cout << std::endl << "Close serial port ..." << std::endl << std::endl;
    my_serial_port.Close();
    
    // finished
    std::cout << std::endl << "Done!" << std::endl << std::endl;
    return EXIT_SUCCESS;
}
