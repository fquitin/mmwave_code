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
 * lo_transmit_worker function
 * A function to be used as a boost::thread_group thread for transmitting
 **********************************************************************/
void lo_transmit_worker(std::vector<std::complex<float>> data_lo,
    uhd::tx_streamer::sptr tx_stream)
{

	// allocate a buffer which we re-use for each channel
    size_t spb = tx_stream->get_max_num_samps(); 
    std::vector<std::complex<float>> buff_lo(spb);
    std::vector<std::complex<float>*> buffs(1);
	buffs[0] = &buff_lo.front(); 
	
	// setup the metadata flags
    uhd::tx_metadata_t md;
    md.start_of_burst = true;
    md.end_of_burst   = false;
    md.has_time_spec  = true;
    md.time_spec = uhd::time_spec_t(0.1); // give us 0.1 seconds to fill the tx buffers
    
	size_t index_lo = 0;
    // send data until the signal handler gets called
    while (not stop_signal_called) {
    
        // fill the buffer with the data file
        for (size_t n = 0; n < spb; n++) {
		    buff_lo[n] = data_lo[index_lo];
		    index_lo++;
		    if (index_lo == data_lo.size()){
		    	index_lo = 0;
			}
		}

        // send the entire contents of the buffer
        tx_stream->send(buffs, spb, md);

        md.start_of_burst = false;
        md.has_time_spec  = false;
    }

    // send a mini EOB packet
    md.end_of_burst = true;
    tx_stream->send("", 0, md);
}




/***********************************************************************
 * Main function
 **********************************************************************/
int UHD_SAFE_MAIN(int argc, char* argv[])
{
    // variables to be set by po
    std::string args, file, ant_bb, ant_lo, subdev_bb, subdev_lo, ref, pps, channel_list, name_serial_port;
    uint64_t total_num_samps;
    double rate_bb, rate_lo, freq_bb, gain_bb, freq_lo, gain_lo;
    
    
    std::ofstream outfile;
    uint64_t 	nbr_samps_per_direction = 5000000;
    int 		nbr_directions = 3;
    float 		seconds_in_future = 1;
    
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
		("args", po::value<std::string>(&args)->default_value("addr=192.168.192.40"), "single uhd device address args")
		("rate-bb", po::value<double>(&rate_bb)->default_value(1000000), "rate of receive incoming baseband samples")
        ("rate-lo", po::value<double>(&rate_lo)->default_value(1000000), "rate of transmit outgoing LO samples")
		("freq-bb", po::value<double>(&freq_bb)->default_value(4000000000), "BB RF chain center frequency in Hz")
		("gain-bb", po::value<double>(&gain_bb)->default_value(30), "gain for the BB RF chain")
		("ant-bb", po::value<std::string>(&ant_bb)->default_value("TX/RX"), "antenna selection BB RF chain")
		("freq-lo", po::value<double>(&freq_lo)->default_value(6000000000), "LO RF chain center frequency in Hz")
		("gain-lo", po::value<double>(&gain_lo)->default_value(31.5), "gain for the LO RF chain")
		("ant-lo", po::value<std::string>(&ant_lo)->default_value("TX/RX"), "antenna selection LO RF chain")
		("subdev-bb", po::value<std::string>(&subdev_bb)->default_value("A:0"), "BB subdevice specification")
		("subdev-lo", po::value<std::string>(&subdev_lo)->default_value("B:0"), "LO subdevice specification")
		("ref", po::value<std::string>(&ref)->default_value("external"), "clock reference (internal, external, gpsdo)")
		("pps", po::value<std::string>(&pps)->default_value("external"), "PPS source (internal, external, gpsdo)")
		("serialport", po::value<std::string>(&name_serial_port)->default_value("/dev/ttyUSB0"), "Serial port of the mmWave array")
        
    ;
    // clang-format on
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    // print the help message
    if (vm.count("help")) {
        std::cout << boost::format("mmWave Rx using a single USRP-X310 for LO and BB signals%s") % desc << std::endl;
        return ~0;
    }
    
    // Open the output file
    outfile.open("//home/francois/uhd-3.15.0.0/host/build/mmwave_code/outfile.dat", std::ofstream::binary);
    if (outfile.is_open()){
	printf("Output file opened correctly. \n"); }
    else{
	printf("OUTPUT FILE NOT OPENED !!! \n"); }
    
    
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
	
    
    
    // create usrp RX device (with BB-RX and LO-TX)
    std::cout << boost::format("Creating the USRP-RX-BB device with: %s...") % args << std::endl;
    uhd::usrp::multi_usrp::sptr usrp_rx_bb = uhd::usrp::multi_usrp::make(args);
    std::cout << boost::format("Creating the USRP-RX-LO device with: %s...") % args << std::endl;
    uhd::usrp::multi_usrp::sptr usrp_rx_lo = uhd::usrp::multi_usrp::make(args);
    
    // always select the subdevice first, the channel mapping affects the other settings
    std::cout << boost::format("Setting subdevice USRP-RX-BB device to: %s...") % subdev_bb << std::endl;
    usrp_rx_bb->set_rx_subdev_spec(subdev_bb); 
    std::cout << boost::format("Setting subdevice USRP-RX-LO device to: %s...") % subdev_lo << std::endl;
    usrp_rx_lo->set_tx_subdev_spec(subdev_lo);
    std::cout << boost::format("Using USRP-RX-BB Device: %s") % usrp_rx_bb->get_pp_string() << std::endl;
    std::cout << boost::format("Using USRP-RX-LO Device: %s") % usrp_rx_lo->get_pp_string() << std::endl;
    
    // Lock mboard clocks
    if (vm.count("ref")) {
    	usrp_rx_bb->set_clock_source(ref);
    	usrp_rx_lo->set_clock_source(ref);
    }

    // set the sample rate
    std::cout << boost::format("Setting USRP-RX-BB Rx Rate: %f Msps...") % (rate_bb / 1e6) << std::endl;
    usrp_rx_bb->set_rx_rate(rate_bb);
    std::cout << boost::format("Actual SRP-RX-BB Rx Rate: %f Msps...") % (usrp_rx_bb->get_rx_rate() / 1e6) << std::endl;
    std::cout << boost::format("Setting USRP-RX-LO Tx Rate: %f Msps...") % (rate_lo / 1e6) << std::endl;
    usrp_rx_lo->set_tx_rate(rate_lo);
    std::cout << boost::format("Actual SRP-RX-LO Tx Rate: %f Msps...") % (usrp_rx_lo->get_tx_rate() / 1e6) << std::endl;
    
    // set the center frequency, rf gain and antenna for the BB RF chain
    std::cout << boost::format("Setting USRP-RX BB Freq: %f MHz...") % (freq_bb / 1e6) << std::endl;
    uhd::tune_request_t tune_request_bb(freq_bb);
    usrp_rx_bb->set_rx_freq(tune_request_bb, 0);
    std::cout << boost::format("Actual USRP-RX BB Freq: %f MHz...") % (usrp_rx_bb->get_rx_freq(0) / 1e6) << std::endl;
    std::cout << boost::format("Setting USRP-RX BB Gain: %f dB...") % gain_bb << std::endl;
    usrp_rx_bb->set_rx_gain(gain_bb, 0);
    std::cout << boost::format("Actual USRP-RX BB Gain: %f dB...") % usrp_rx_bb->get_rx_gain(0) << std::endl;
    usrp_rx_bb->set_rx_antenna(ant_bb, 0);
    
    // set the center frequency, rf gain and antenna for the LO RF chain
    std::cout << boost::format("Setting USRP-RX LO Freq: %f MHz...") % (freq_lo / 1e6) << std::endl;
    uhd::tune_request_t tune_request_lo(freq_lo);
    usrp_rx_lo->set_tx_freq(tune_request_lo, 0);
    std::cout << boost::format("Actual USRP-RX LO Freq: %f MHz...") % (usrp_rx_lo->get_tx_freq(0) / 1e6) << std::endl;
    std::cout << boost::format("Setting USRP-RX LO Gain: %f dB...") % gain_lo << std::endl;
    usrp_rx_lo->set_tx_gain(gain_lo, 0);
    std::cout << boost::format("Actual USRP-RX LO Gain: %f dB...") % usrp_rx_lo->get_tx_gain(0) << std::endl;
    usrp_rx_lo->set_tx_antenna(ant_lo, 0);
    
    // allow for some setup time
    std::this_thread::sleep_for(std::chrono::seconds(1)); 
    
    // Setting timestamp and time source (only for USRP-RX-BB, USRP-RX-LO follows automatically)
    std::cout << boost::format("Setting USRP-RX timestamp to 0...") << std::endl;
    usrp_rx_bb->set_time_source(pps);
    usrp_rx_bb->set_time_unknown_pps(uhd::time_spec_t(0.0));
    std::this_thread::sleep_for(std::chrono::seconds(1)); // wait for pps sync pulse
    usrp_rx_bb->set_time_now(0.0);
    
    // Check Ref and LO Lock detect
    std::vector<std::string> sensor_names;
    sensor_names = usrp_rx_bb->get_rx_sensor_names(0);
    if (std::find(sensor_names.begin(), sensor_names.end(), "lo_locked")
        != sensor_names.end()) {
        uhd::sensor_value_t lo_locked = usrp_rx_bb->get_rx_sensor("lo_locked", 0);
        std::cout << boost::format("Checking RX: %s ...") % lo_locked.to_pp_string() << std::endl;
        UHD_ASSERT_THROW(lo_locked.to_bool());
    }
    sensor_names = usrp_rx_lo->get_tx_sensor_names(0);
    if (std::find(sensor_names.begin(), sensor_names.end(), "lo_locked")
        != sensor_names.end()) {
        uhd::sensor_value_t lo_locked = usrp_rx_lo->get_tx_sensor("lo_locked", 0);
        std::cout << boost::format("Checking TX: %s ...") % lo_locked.to_pp_string() << std::endl;
        UHD_ASSERT_THROW(lo_locked.to_bool());
    }
    
    const size_t mboard_sensor_idx = 0;
    sensor_names = usrp_rx_bb->get_mboard_sensor_names(mboard_sensor_idx);
    if ((ref == "external")
        and (std::find(sensor_names.begin(), sensor_names.end(), "ref_locked")
                != sensor_names.end())) {
        uhd::sensor_value_t ref_locked =
            usrp_rx_bb->get_mboard_sensor("ref_locked", mboard_sensor_idx);
        std::cout << boost::format("Checking RX: %s ...") % ref_locked.to_pp_string()
                  << std::endl;
        UHD_ASSERT_THROW(ref_locked.to_bool());
    }
    sensor_names = usrp_rx_lo->get_mboard_sensor_names(mboard_sensor_idx);
    if ((ref == "external")
        and (std::find(sensor_names.begin(), sensor_names.end(), "ref_locked")
                != sensor_names.end())) {
        uhd::sensor_value_t ref_locked =
            usrp_rx_lo->get_mboard_sensor("ref_locked", mboard_sensor_idx);
        std::cout << boost::format("Checking TX: %s ...") % ref_locked.to_pp_string()
                  << std::endl;
        UHD_ASSERT_THROW(ref_locked.to_bool());
    }
    
    
    // Generate LO signals to transmit
    std::vector<std::complex<float>> data_lo(10000);
    for (size_t i = 0; i < data_lo.size(); i++){
        data_lo[i] = 1.0;
    }
    
    // create a transmit streamer
    std::vector<size_t> channel_nums = {0};
    uhd::stream_args_t stream_args("fc32", "sc16");
    stream_args.channels = channel_nums;
    uhd::tx_streamer::sptr tx_stream = usrp_rx_lo->get_tx_stream(stream_args);
    
    // ================================
    // start LO transmit worker thread
    // ================================
    std::cout << boost::format("Starting LO transmitter thread...") << std::endl;
    boost::thread_group transmit_thread;
    transmit_thread.create_thread(boost::bind(&lo_transmit_worker, data_lo, tx_stream));
    
      
    // create a receive streamer
    uhd::rx_streamer::sptr rx_stream = usrp_rx_bb->get_rx_stream(stream_args);
    
	
    //meta-data will be filled in by recv()
    uhd::rx_metadata_t md;
    
    // allocate a buffer which we re-use for each channel
    size_t spb = rx_stream->get_max_num_samps(); 
    std::vector<std::complex<float>> buff_bb(spb);
    std::vector<std::complex<float>*> buffs(1);
    buffs[0] = &buff_bb.front(); 
    
    //the first call to recv() will block this many seconds before receiving
    double timeout = seconds_in_future + 0.1; //timeout 
    
    //setup streaming
	total_num_samps = nbr_samps_per_direction ;
	std::cout << boost::format("Begin streaming %u samples, %f seconds in the future...") % total_num_samps % seconds_in_future << std::endl;
	uhd::stream_cmd_t stream_cmd(uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS);
	//stream_cmd.num_samps = total_num_samps;
	stream_cmd.stream_now = false;
	stream_cmd.time_spec = uhd::time_spec_t(seconds_in_future);
	rx_stream->issue_stream_cmd(stream_cmd);
	
	
	// ==============================================================
	// Start looping over all AiP directions and Rx baseband samples
	// ==============================================================
	std::string degrees;
	std::string angle; 
	std::string direction;
	int mode = 2; // 0 for TX/RX off, 1 for TX, 2 for RX
    for (int cpt_directions = 0; cpt_directions < nbr_directions; cpt_directions++)
    {
    	// Setting AiP beam direction
    	degrees = possible_degrees[cpt_directions];
    	angle = possible_angles[cpt_directions];
    	direction = possible_directions[1];
    	mode = 1;  
    	float time_now = usrp_rx_bb->get_time_now().get_real_secs() ;    	
    	std::cout << boost::format("Setting AiP to %s - %s ° at time %f") % direction % angle % time_now << std::endl;
    	send_to_aip(&my_serial_port, degrees, direction, gain_list, gain, active_list, mode);
    	
    	if (outfile.is_open()) {
			outfile << std::endl << "AiP data" << std::endl ;
			outfile << boost::format("%s - %s degrees at time %f") % direction % angle % time_now;
			outfile << std::endl;
		}
    	
    	// Receive "nbr_samps_per_direction" samples
    	if (outfile.is_open()) {
			outfile << std::endl << "USRP data" << std::endl ;
		}
    	size_t num_acc_samps = 0; //number of accumulated samples
		while(num_acc_samps < nbr_samps_per_direction){
		    //receive a single packet
		    size_t num_rx_samps = rx_stream->recv(&buffs.front(), buffs.size(), md, timeout, true);

		    //use a small timeout for subsequent packets
		    timeout = 0.1;
		    
		    //handle the error code
		    if (md.error_code == uhd::rx_metadata_t::ERROR_CODE_TIMEOUT) break;
		    if (md.error_code != uhd::rx_metadata_t::ERROR_CODE_NONE){
		        throw std::runtime_error(str(boost::format(
		            "Receiver error %s"
		        ) % md.strerror()));
		    }
		    
		    if (outfile.is_open()) {
				outfile.write((const char*)&buffs.front(), num_rx_samps*sizeof(std::complex<float>));
			}
			
			num_acc_samps += num_rx_samps;
		}
		std::cout << boost::format("  -- Received %f samples") % num_acc_samps << std::endl;
		if (outfile.is_open()) {
			outfile << std::endl;
		}
	}
	
	// Stop streaming from USRP
	stream_cmd.stream_mode = uhd::stream_cmd_t::STREAM_MODE_STOP_CONTINUOUS;
	stream_cmd.stream_now = true;
	rx_stream->issue_stream_cmd(stream_cmd);
    
    // Disable AiP
    disable_aip(&my_serial_port);
    
    // Close serial port
    std::cout << std::endl << "Close serial port ..." << std::endl << std::endl;
    my_serial_port.Close();
    
    
    // Stopping LO transmitter thread
    stop_signal_called = true;
    
    // finished
    std::cout << std::endl << "Done!" << std::endl << std::endl;
    return EXIT_SUCCESS;
}
