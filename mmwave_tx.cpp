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
#include "aip_functions.h"
#include "/usr/local/include/libserial/SerialPort.h"
using namespace LibSerial ;

namespace po = boost::program_options;

bool stop_signal_called = false;

/***********************************************************************
 * lo_transmit_worker function
 * A function to be used as a boost::thread_group thread for transmitting
 **********************************************************************/
void transmit_worker(std::vector<std::complex<float>> data_bb,
    std::vector<std::complex<float>> data_lo,
    uhd::tx_streamer::sptr tx_stream)
{

	// allocate a buffer which we re-use for each channel
    size_t spb = tx_stream->get_max_num_samps(); 
    std::vector<std::complex<float>> buff_bb(spb);
    std::vector<std::complex<float>> buff_lo(spb);
    std::vector<std::complex<float>*> buffs(2);
    buffs[0] = &buff_bb.front(); 
	buffs[1] = &buff_lo.front(); 

	
	// setup the metadata flags
    uhd::tx_metadata_t md;
    md.start_of_burst = true;
    md.end_of_burst   = false;
    md.has_time_spec  = true;
    md.time_spec = uhd::time_spec_t(1.0); // give us 1.0 seconds to fill the tx buffers
    
    size_t index_bb = 0;
	size_t index_lo = 0;
    // send data until the signal handler gets called
    while (not stop_signal_called) {
    
        // fill the buffer with the data file
        for (size_t n = 0; n < spb; n++) {
		    buff_bb[n] = data_bb[index_bb];
		    buff_lo[n] = data_lo[index_lo];
		    index_bb++;
		    index_lo++;
		    if (index_bb == data_bb.size()){
		    	index_bb = 0;
			}
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
    std::string args, file, ant_bb, ant_lo, subdev_tx, ref, pps, channel_list, name_serial_port;
    uint64_t total_num_samps;
    double rate, freq_bb, gain_bb, freq_lo, gain_lo;
    
    
    uint64_t 	nbr_samps_per_direction = 500000;
    int 		nbr_directions = 9;
    int 		ver_aip = 0;
    
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
		("args", po::value<std::string>(&args)->default_value("addr=192.168.192.50"), "single uhd device address args")
		("file", po::value<std::string>(&file)->default_value(""), "name of the file to read binary samples from (if empty, LO signal is sent)")
		("nsamps", po::value<uint64_t>(&total_num_samps)->default_value(0), "total number of samples to transmit (0 for infinite)")
		("rate", po::value<double>(&rate)->default_value(1000000), "rate of outgoing samples")
		("freq-bb", po::value<double>(&freq_bb)->default_value(4000000000), "RF chain 1 center frequency in Hz")
		("gain-bb", po::value<double>(&gain_bb)->default_value(30), "gain for the RF chain 1")
		("ant-bb", po::value<std::string>(&ant_bb)->default_value("TX/RX"), "antenna selection RF chain 1")
		("freq-lo", po::value<double>(&freq_lo)->default_value(6000000000), "RF chain 2 center frequency in Hz")
		("gain-lo", po::value<double>(&gain_lo)->default_value(31.5), "gain for the RF chain 2")
		("ant-lo", po::value<std::string>(&ant_lo)->default_value("TX/RX"), "antenna selection RF chain 2")
		("subdev-tx", po::value<std::string>(&subdev_tx)->default_value("A:0 B:0"), "subdevice specification")
		("ref", po::value<std::string>(&ref)->default_value("external"), "clock reference (internal, external, gpsdo)")
		("pps", po::value<std::string>(&pps)->default_value("external"), "PPS source (internal, external, gpsdo)")
		("channels", po::value<std::string>(&channel_list)->default_value("0,1"), "which channels to use (specify \"0\", \"1\", \"0,1\", etc)")
		("serialport", po::value<std::string>(&name_serial_port)->default_value("/dev/ttyUSB0"), "Serial port of the mmWave array")
        
    ;
    // clang-format on
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    // print the help message
    if (vm.count("help")) {
        std::cout << boost::format("mmWave Tx using a single USRP-X310 for LO and BB signals%s") % desc << std::endl;
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
    
    int mode_init = 1;
	std::cout << boost::format("Setting AiP to %s - %s °") % "UP" % "0"  << std::endl;
    send_to_aip(&my_serial_port, "DEG_0", "UP", gain_list, gain, active_list, mode_init, ver_aip);
    std::cout << boost::format("Setting AiP to %s - %s °") % "UP" % "0"  << std::endl;
    send_to_aip(&my_serial_port, "DEG_0", "UP", gain_list, gain, active_list, mode_init, ver_aip);
    std::cout << boost::format("Setting AiP to %s - %s °") % "LEFT" % "0"  << std::endl;
    send_to_aip(&my_serial_port, "DEG_0", "LEFT", gain_list, gain, active_list, mode_init, ver_aip);

    
    
    // create usrp TX device
    std::cout << boost::format("Creating the USRP-TX device with: %s...") % args << std::endl;
    uhd::usrp::multi_usrp::sptr usrp_tx = uhd::usrp::multi_usrp::make(args);
    
    // always select the subdevice first, the channel mapping affects the other settings
    std::cout << boost::format("Setting subdevice USRP-TX device to: %s...") % subdev_tx << std::endl;
    usrp_tx->set_tx_subdev_spec(subdev_tx); 
    std::cout << boost::format("Using USRP-TX Device: %s") % usrp_tx->get_pp_string() << std::endl;
    
    // Lock mboard clocks
    if (vm.count("ref")) {
    	usrp_tx->set_clock_source(ref);
    }

    
    // set the sample rate
    std::cout << boost::format("Setting TX Rate: %f Msps...") % (rate / 1e6) << std::endl;
    usrp_tx->set_tx_rate(rate);
    std::cout << boost::format("Actual TX Rate: %f Msps...") % (usrp_tx->get_tx_rate() / 1e6) << std::endl;

    
    
    // set the center frequency, rf gain and antenna for the BB RF chain
    std::cout << boost::format("Setting USRP-TX BB Freq: %f MHz...") % (freq_bb / 1e6) << std::endl;
    uhd::tune_request_t tune_request_bb(freq_bb);
    usrp_tx->set_tx_freq(tune_request_bb, 0);
    std::cout << boost::format("Actual USRP-TX BB Freq: %f MHz...") % (usrp_tx->get_tx_freq(0) / 1e6) << std::endl;
    std::cout << boost::format("Setting USRP-TX BB Gain: %f dB...") % gain_bb << std::endl;
    usrp_tx->set_tx_gain(gain_bb, 0);
    std::cout << boost::format("Actual USRP-TX BB Gain: %f dB...") % usrp_tx->get_tx_gain(0) << std::endl;
    usrp_tx->set_tx_antenna(ant_bb, 0);
    
    // set the center frequency, rf gain and antenna for the LO RF chain
    std::cout << boost::format("Setting USRP-TX LO Freq: %f MHz...") % (freq_lo / 1e6) << std::endl;
    uhd::tune_request_t tune_request_lo(freq_lo);
    usrp_tx->set_tx_freq(tune_request_lo, 1);
    std::cout << boost::format("Actual USRP-TX LO Freq: %f MHz...") % (usrp_tx->get_tx_freq(1) / 1e6) << std::endl;
    std::cout << boost::format("Setting USRP-TX LO Gain: %f dB...") % gain_lo << std::endl;
    usrp_tx->set_tx_gain(gain_lo, 1);
    std::cout << boost::format("Actual USRP-TX LO Gain: %f dB...") % usrp_tx->get_tx_gain(1) << std::endl;
    usrp_tx->set_tx_antenna(ant_lo, 1);
    
    // allow for some setup time
    std::this_thread::sleep_for(std::chrono::seconds(1)); 
    
    // Setting timestamp and time source
    std::cout << boost::format("Setting USRP-TX timestamp to 0...") << std::endl;
    usrp_tx->set_time_source(pps);
    usrp_tx->set_time_unknown_pps(uhd::time_spec_t(0.0));
    std::this_thread::sleep_for(std::chrono::seconds(1)); // wait for pps sync pulse
    usrp_tx->set_time_now(0.0);

    // Check Ref and LO Lock detect 
    std::vector<std::string> sensor_names;
    sensor_names = usrp_tx->get_tx_sensor_names(0);
    if (std::find(sensor_names.begin(), sensor_names.end(), "lo_locked")
        != sensor_names.end()) {
        uhd::sensor_value_t lo_locked = usrp_tx->get_tx_sensor("lo_locked", 0);
        std::cout << boost::format("Checking TX: %s ...") % lo_locked.to_pp_string() << std::endl;
        UHD_ASSERT_THROW(lo_locked.to_bool());
    }
    const size_t mboard_sensor_idx = 0;
    sensor_names = usrp_tx->get_mboard_sensor_names(mboard_sensor_idx);
    if ((ref == "external")
        and (std::find(sensor_names.begin(), sensor_names.end(), "ref_locked")
                != sensor_names.end())) {
        uhd::sensor_value_t ref_locked =
            usrp_tx->get_mboard_sensor("ref_locked", mboard_sensor_idx);
        std::cout << boost::format("Checking TX: %s ...") % ref_locked.to_pp_string()
                  << std::endl;
        UHD_ASSERT_THROW(ref_locked.to_bool());
    }
    
    // Generate BB and LO signals to transmit
    // BB data
    std::vector<std::complex<float>> data_bb(10000);
    for (size_t i = 0; i < 1000; i++){
        data_bb[i] = (2*(rand() % 2) -1) + (2*(rand() % 2) -1)*1j ;
    }
    for (size_t i = 1000; i < data_bb.size(); i++){
        data_bb[i] = 0.0;
    }
    //LO data
    std::vector<std::complex<float>> data_lo(10000);
    for (size_t i = 0; i < data_lo.size(); i++){
        data_lo[i] = 1.0;
    }
    
    // create a transmit streamer
    std::vector<size_t> channel_nums = {0, 1};
    uhd::stream_args_t stream_args("fc32", "sc16");
    stream_args.channels = channel_nums;
    uhd::tx_streamer::sptr tx_stream = usrp_tx->get_tx_stream(stream_args);

    
    // =============================
    // start transmit worker thread
    // =============================
    boost::thread_group transmit_thread;
    transmit_thread.create_thread(boost::bind(&transmit_worker, data_bb, data_lo, tx_stream));

	// =====================================
	// Start looping over all AiP directions
	// =====================================
	float time_next_direction = 1.0; 	// initial time of first transmission
	std::string degrees;
	std::string angle; 
	std::string direction;
	int mode = 0; // 0 for TX/RX off, 1 for TX, 2 for RX
	
	
	//for (int cpt_leftright = 0; cpt_leftright < 1; cpt_leftright++)
	for (int cpt_leftright = 0; cpt_leftright < 2; cpt_leftright++)
	{
		if (cpt_leftright == 0)
		{
			direction = possible_directions[0];
			//for (int cpt_directions = 2; cpt_directions > -1; cpt_directions--)
			for (int cpt_directions = 16; cpt_directions > -1; cpt_directions--)
			{
				// Setting AiP beam direction
				degrees = possible_degrees[cpt_directions];
				angle = possible_angles[cpt_directions];
				
				mode = 1;     	
				std::cout << boost::format("Setting AiP to %s - %s ° at time %f") % direction % angle % usrp_tx->get_time_now().get_real_secs() << std::endl;
				send_to_aip(&my_serial_port, degrees, direction, gain_list, gain, active_list, mode, ver_aip);
				//std::cout << boost::format("  -- time now: %f") % usrp_tx->get_time_now().get_real_secs() << std::endl;
				
				// Blocking call to let USRP transmit until it's time for next direction
				time_next_direction += nbr_samps_per_direction/rate; 
				while(usrp_tx->get_time_now().get_real_secs()<time_next_direction){
					// wait
				}
			}
		}
		else
		{
			direction = possible_directions[1];
			for (int cpt_directions = 0; cpt_directions < 17; cpt_directions++)
				{
					// Setting AiP beam direction
					degrees = possible_degrees[cpt_directions];
					angle = possible_angles[cpt_directions];
					
					mode = 1;     	
					std::cout << boost::format("Setting AiP to %s - %s ° at time %f") % direction % angle % usrp_tx->get_time_now().get_real_secs() << std::endl;
					send_to_aip(&my_serial_port, degrees, direction, gain_list, gain, active_list, mode, ver_aip);
					
					// Blocking call to let USRP transmit until it's time for next direction
					time_next_direction += nbr_samps_per_direction/rate; 
					while(usrp_tx->get_time_now().get_real_secs()<time_next_direction){
						// wait
					}
				}
		}
	}

    
    // Disable AiP
    disable_aip(&my_serial_port, ver_aip);
    
    // Close serial port
    std::cout << std::endl << "Close serial port ..." << std::endl << std::endl;
    my_serial_port.Close();
    
    
    // Stopping LO transmitter thread
    stop_signal_called = true;
    
    // finished
    std::cout << std::endl << "Done!" << std::endl << std::endl;
    return EXIT_SUCCESS;
}
