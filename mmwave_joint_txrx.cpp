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
 * tx_worker function
 * A function to be used as a boost::thread_group thread for transmitting
 **********************************************************************/
void tx_worker(std::vector<std::complex<float>> data_bb,
    std::vector<std::complex<float>> data_lo,
    uhd::tx_streamer::sptr stream_tx)
{
	// allocate a buffer which we re-use for each channel
    size_t spb = stream_tx->get_max_num_samps(); 
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
        stream_tx->send(buffs, spb, md);

        md.start_of_burst = false;
        md.has_time_spec  = false;
    }

    // send a mini EOB packet
    md.end_of_burst = true;
    stream_tx->send("", 0, md);
}


/***********************************************************************
 * rx_lo_worker function
 * A function to be used as a boost::thread_group thread for transmitting
 **********************************************************************/
void rx_lo_worker(std::vector<std::complex<float>> data_lo,
    uhd::tx_streamer::sptr stream_rx_lo)
{

	// allocate a buffer which we re-use for each channel
    size_t spb = stream_rx_lo->get_max_num_samps(); 
    std::vector<std::complex<float>> buff_lo(spb);
    std::vector<std::complex<float>*> buffs(1);
	buffs[0] = &buff_lo.front(); 
	
	// setup the metadata flags
    uhd::tx_metadata_t md;
    md.start_of_burst = true;
    md.end_of_burst   = false;
    md.has_time_spec  = true;
    md.time_spec = uhd::time_spec_t(1.0); // give us 0.1 seconds to fill the tx buffers
    
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
        stream_rx_lo->send(buffs, spb, md);

        md.start_of_burst = false;
        md.has_time_spec  = false;
    }

    // send a mini EOB packet
    md.end_of_burst = true;
    stream_rx_lo->send("", 0, md);
}





/***********************************************************************
 * Main function
 **********************************************************************/
int UHD_SAFE_MAIN(int argc, char* argv[])
{
    
    // variable definitions
    std::string 	args_tx, args_rx, name_serial_port_tx, name_serial_port_rx, ref; 
    int 			mode_tx, mode_rx, ver_aip; 
    double 			rate_tx, rate_rx, freq_bb, freq_lo, gain_tx_bb, gain_rx_bb, gain_lo; 
    std::ofstream 	outfile;
    uint64_t 		nbr_samps_per_degree;
    
    // variables with initializations
    int 			gain_tx				= 0; 									// attenuation of entire Tx mmWave array
    int 			gain_list_tx[4] 	= {0,0,0,0}; 							// attenuation of each chip of Tx mmWave array
    std::string 	active_list_tx[4] 	= {"1111", "1111", "1111", "1111"}; 	// active antennas of Tx mmWave array
    int 			gain_rx				= 0; 									// attenuation of entire Rx mmWave array
	int 			gain_list_rx[4] 	= {0,0,0,0}; 							// attenuation of each chip of Rx mmWave array
    std::string 	active_list_rx[4] 	= {"1111", "1111", "1111", "1111"}; 	// active antennas of Rx mmWave array
    std::string 	subdev_tx 			= "A:0 B:0";
    std::string		subdev_rx_bb 		= "A:0";
    std::string		subdev_rx_lo		= "B:0";
    std::string 	ant_bb 				= "TX/RX";
    std::string 	ant_lo 				= "TX/RX";
    std::string 	all_directions[4] 	= {"LEFT", "RIGHT", "UP", "DOWN"};
	std::string 	all_degrees[17] 	= {"DEG_0","DEG_11_25","DEG_22_25","DEG_33_75","DEG_45","DEG_56_25","DEG_67_5","DEG_78_75","DEG_90",
											  "DEG_101_2","DEG_112_5","DEG_123_7","DEG_135","DEG_146_2","DEG_157_5","DEG_168_7","DEG_180"};	
    std::string 	all_angles[17] 		= {"0.00", "4.00", "8.00", "11.50", "15.50", "19.50", "23.50", "28.00", "32.50",
    										 "37.00", "41.50", "46.50", "52.00", "57.50", "64.50", "72.00", "78.00"};			
	float 			seconds_in_future 	= 2.0;
	int 			nbr_degrees 		= 17; // nbr of beams in one direction from broadside, between 1 and 17
	int 			nbr_directions 		= 2;  // nbr of directions, between 1 and 4 (2 to sweep from left to right)
    
    
    // setup the program options
    po::options_description desc("Allowed options");
    // clang-format off
    desc.add_options()
		("help", "help message")
		("args-tx", po::value<std::string>(&args_tx)->default_value("addr=192.168.192.30"), "USRP IP address for Tx") 
		("args-rx", po::value<std::string>(&args_rx)->default_value("addr=192.168.192.40"), "USRP IP address for Rx") 
		("serialport-tx", po::value<std::string>(&name_serial_port_tx)->default_value("/dev/ttyUSB0"), "Serial port of the Tx mmWave array")
		("serialport-rx", po::value<std::string>(&name_serial_port_rx)->default_value("/dev/ttyUSB1"), "Serial port of the Rx mmWave array")
		("ref", po::value<std::string>(&ref)->default_value("external"), "clock reference (internal, external, gpsdo)")
		("rate-tx", po::value<double>(&rate_tx)->default_value(1000000), "sample rate of Tx")
		("rate-rx", po::value<double>(&rate_rx)->default_value(1000000), "sample rate of Rx")
		("freq-bb", po::value<double>(&freq_bb)->default_value(4000000000), "Center frequency of Tx and Rx baseband signal in Hz")
		("freq-lo", po::value<double>(&freq_lo)->default_value(6000000000), "Center frequency of Tx and Rx LO signal in Hz")
		("gain-tx-bb", po::value<double>(&gain_tx_bb)->default_value(30), "Gain of Tx baseband signal in dB")
		("gain-rx-bb", po::value<double>(&gain_rx_bb)->default_value(30), "Gain of Rx baseband signal in dB")
		("gain-lo", po::value<double>(&gain_lo)->default_value(31.5), "Gain of the LO chain (for Tx and Rx)")
		("nsamps-per-degree", po::value<uint64_t>(&nbr_samps_per_degree)->default_value(500000), "Number of samples per Tx/Rx beam direction")
		("ver-aip", po::value<int>(&ver_aip)->default_value(0), "verbose mmWave arrays on or off")
    ;
    // clang-format on
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    // print the help message
    if (vm.count("help")) {
        std::cout << boost::format("Software to control mmWave Tx and Rx jointly. %s") % desc << std::endl;
        return ~0;
    }
    
    // Open the output file
    outfile.open("//home/francois/uhd-3.15.0.0/host/build/mmwave_code/outfile.dat", std::ofstream::binary);
    if (outfile.is_open()){
		printf("Output file opened correctly. \n"); }
    else{
		printf("OUTPUT FILE NOT OPENED !!! \n"); }
    
    
    // ======================================================
    // Open and initialize serial port of the mmWave array Tx
    // ======================================================
    // Create and open the serial port for communication with the mmWave array Tx.
    std::cout << boost::format("Create and open the serial port for mmWave array Tx on %s...") % name_serial_port_tx << std::endl;
    SerialPort my_serial_port_tx( name_serial_port_tx );
    // Set serial port parameters 
    std::cout << boost::format("  -- Set serial port parameters ...") << std::endl;
    my_serial_port_tx.SetBaudRate( LibSerial::BaudRate::BAUD_115200 );
    my_serial_port_tx.SetCharacterSize( LibSerial::CharacterSize::CHAR_SIZE_8 );
    my_serial_port_tx.SetStopBits( LibSerial::StopBits::STOP_BITS_1 ) ;
    my_serial_port_tx.SetParity( LibSerial::Parity::PARITY_NONE );
    // Initialize mmWave array Tx
    mode_tx = 1; // Tx mode
	std::cout << boost::format("  -- Setting AiP to %s - %s °") % "UP" % "0"  << std::endl;
    send_to_aip(&my_serial_port_tx, "DEG_0", "UP", gain_list_tx, gain_tx, active_list_tx, mode_tx, ver_aip);
    std::cout << boost::format("  -- Setting AiP to %s - %s °") % "UP" % "0"  << std::endl;
    send_to_aip(&my_serial_port_tx, "DEG_0", "UP", gain_list_tx, gain_tx, active_list_tx, mode_tx, ver_aip);
    std::cout << boost::format("  -- Setting AiP to %s - %s °") % "LEFT" % "0"  << std::endl;
    send_to_aip(&my_serial_port_tx, "DEG_0", "LEFT", gain_list_tx, gain_tx, active_list_tx, mode_tx, ver_aip);
    
    
    // ======================================================
    // Open and initialize serial port of the mmWave array Rx
    // ======================================================
    // Create and open the serial port for communication with the mmWave array Tx.
    std::cout << boost::format("Create and open the serial port for mmWave array Rx on %s...") % name_serial_port_rx << std::endl;
    SerialPort my_serial_port_rx( name_serial_port_rx );
    // Set serial port parameters 
    std::cout << boost::format("  -- Set serial port parameters ...") << std::endl;
    my_serial_port_rx.SetBaudRate( LibSerial::BaudRate::BAUD_115200 );
    my_serial_port_rx.SetCharacterSize( LibSerial::CharacterSize::CHAR_SIZE_8 );
    my_serial_port_rx.SetStopBits( LibSerial::StopBits::STOP_BITS_1 ) ;
    my_serial_port_rx.SetParity( LibSerial::Parity::PARITY_NONE );
    // Initialize mmWave array Tx
    mode_rx = 2; // Tx mode
	std::cout << boost::format("  -- Setting AiP to %s - %s °") % "UP" % "0"  << std::endl;
    send_to_aip(&my_serial_port_rx, "DEG_0", "UP", gain_list_rx, gain_rx, active_list_rx, mode_rx, ver_aip);
    std::cout << boost::format("  -- Setting AiP to %s - %s °") % "UP" % "0"  << std::endl;
    send_to_aip(&my_serial_port_rx, "DEG_0", "UP", gain_list_rx, gain_rx, active_list_rx, mode_rx, ver_aip);
    std::cout << boost::format("  -- Setting AiP to %s - %s °") % "LEFT" % "0"  << std::endl;
    send_to_aip(&my_serial_port_rx, "DEG_0", "LEFT", gain_list_rx, gain_rx, active_list_rx, mode_rx, ver_aip);
    
    
    // =============================================
    // Create and initialize USRP Tx and Rx devices
    // =============================================
    // Create USRP devices
    std::cout << boost::format("Creating the USRP-Tx device with: %s...") % args_tx << std::endl;
    uhd::usrp::multi_usrp::sptr usrp_tx = uhd::usrp::multi_usrp::make(args_tx);
    std::cout << boost::format("Creating the USRP-Rx-BB device with: %s...") % args_rx << std::endl;
    uhd::usrp::multi_usrp::sptr usrp_rx_bb = uhd::usrp::multi_usrp::make(args_rx);
    std::cout << boost::format("Creating the USRP-Rx-LO device with: %s...") % args_rx << std::endl;
    uhd::usrp::multi_usrp::sptr usrp_rx_lo = uhd::usrp::multi_usrp::make(args_rx);
    
    // always select the subdevice first, the channel mapping affects the other settings
    std::cout << boost::format("Setting subdevice USRP-Tx device to: %s...") % subdev_tx << std::endl;
    std::cout << boost::format("Setting subdevice USRP-Rx-BB device to: %s...") % subdev_rx_bb << std::endl;
    std::cout << boost::format("Setting subdevice USRP-Rx-LO device to: %s...") % subdev_rx_lo << std::endl;
    usrp_tx->set_tx_subdev_spec(subdev_tx); 
    usrp_rx_bb->set_rx_subdev_spec(subdev_rx_bb); 
    usrp_rx_lo->set_tx_subdev_spec(subdev_rx_lo);
    std::cout << boost::format("Using USRP-Tx Device: %s") % usrp_tx->get_pp_string() << std::endl;
    std::cout << boost::format("Using USRP-Rx-BB Device: %s") % usrp_rx_bb->get_pp_string() << std::endl;
    std::cout << boost::format("Using USRP-Rx-LO Device: %s") % usrp_rx_lo->get_pp_string() << std::endl;
    
    // Lock mboard clocks
    if (vm.count("ref")) {
    	usrp_tx->set_clock_source(ref);
    	usrp_rx_bb->set_clock_source(ref);
    	usrp_rx_lo->set_clock_source(ref);
    }
    
    // set the sample rate
    std::cout << boost::format("Setting USRP-Tx Rate: %f Msps...") % (rate_tx / 1e6) << std::endl;
    usrp_tx->set_tx_rate(rate_tx);
    std::cout << boost::format("Actual USRP-Tx Rate: %f Msps...") % (usrp_tx->get_tx_rate() / 1e6) << std::endl;
    std::cout << boost::format("Setting USRP-Rx-BB Rx Rate and USRP-Rx-LO Tx Rate: %f Msps...") % (rate_rx / 1e6) << std::endl;
    usrp_rx_bb->set_rx_rate(rate_rx);
    usrp_rx_lo->set_tx_rate(rate_rx);
    std::cout << boost::format("Actual USRP-Rx-BB Rx Rate and USRP-Rx-LO Tx Rate: %f Msps and %f Msps...") % (usrp_rx_bb->get_rx_rate() / 1e6) % (usrp_rx_lo->get_tx_rate() / 1e6)<< std::endl;
	
	// set the center frequency of the baseband and LO chains
    std::cout << boost::format("Setting USRP-Tx BB and USRP-Rx BB Freq: %f MHz...") % (freq_bb / 1e6) << std::endl;
    uhd::tune_request_t tune_request_bb(freq_bb);
    usrp_tx->set_tx_freq(tune_request_bb, 0);
    usrp_rx_bb->set_rx_freq(tune_request_bb, 0);
    std::cout << boost::format("Actual USRP-Tx BB and USRP-Rx BB Freq: %f MHz and %f MHz...") % (usrp_tx->get_tx_freq(0) / 1e6) % (usrp_rx_bb->get_rx_freq(0) / 1e6) << std::endl;
	std::cout << boost::format("Setting USRP-Tx LO and USRP-Rx LO Freq: %f MHz...") % (freq_lo / 1e6) << std::endl;
    uhd::tune_request_t tune_request_lo(freq_lo);
    usrp_tx->set_tx_freq(tune_request_lo, 1);
    usrp_rx_lo->set_tx_freq(tune_request_lo, 0);
    std::cout << boost::format("Actual USRP-Tx LO and USRP-Rx LO Freq: %f MHz and %f Msps...") % (usrp_tx->get_tx_freq(1) / 1e6) % (usrp_rx_lo->get_tx_freq(0)) << std::endl;
    
	// set the gains of the baseband and LO chains
    std::cout << boost::format("Setting USRP-Tx BB Gain: %f dB...") % gain_tx_bb << std::endl;
    usrp_tx->set_tx_gain(gain_tx_bb, 0);
    std::cout << boost::format("Actual USRP-Tx BB Gain: %f dB...") % usrp_tx->get_tx_gain(0) << std::endl;
    std::cout << boost::format("Setting USRP-Rx BB Gain: %f dB...") % gain_rx_bb << std::endl;
    usrp_rx_bb->set_rx_gain(gain_rx_bb, 0);
    std::cout << boost::format("Actual USRP-Rx BB Gain: %f dB...") % usrp_rx_bb->get_rx_gain(0) << std::endl;
    std::cout << boost::format("Setting USRP-Tx LO Gain: %f dB...") % gain_lo << std::endl;
    usrp_tx->set_tx_gain(gain_lo, 1);
    std::cout << boost::format("Actual USRP-Tx LO Gain: %f dB...") % usrp_tx->get_tx_gain(1) << std::endl;
    std::cout << boost::format("Setting USRP-Rx LO Gain: %f dB...") % gain_lo << std::endl;
    usrp_rx_lo->set_tx_gain(gain_lo, 0);
    std::cout << boost::format("Actual USRP-Rx LO Gain: %f dB...") % usrp_rx_lo->get_tx_gain(0) << std::endl;
    
    // set the Tx and Rx antenna ports
    std::cout << boost::format("Setting USRP Tx and Rx antenna ports...") << std::endl;
    usrp_tx->set_tx_antenna(ant_bb, 0);
    usrp_tx->set_tx_antenna(ant_lo, 1);
    usrp_rx_bb->set_rx_antenna(ant_bb, 0);
    usrp_rx_lo->set_tx_antenna(ant_lo, 0);
    
    // allow for some setup time
    std::this_thread::sleep_for(std::chrono::seconds(1)); 
    
    // Setting timestamp and time source
    std::cout << boost::format("Setting USRP Tx and Rx timestamps to 0...") << std::endl;
    usrp_tx->set_time_source(ref);
    usrp_tx->set_time_unknown_pps(uhd::time_spec_t(0.0));
    usrp_rx_bb->set_time_source(ref);
    usrp_rx_bb->set_time_unknown_pps(uhd::time_spec_t(0.0));
    std::this_thread::sleep_for(std::chrono::seconds(1)); // wait for pps sync pulse
    usrp_tx->set_time_now(0.0);
    usrp_rx_bb->set_time_now(0.0);
    
    // Check Ref and LO Lock detect for USRP Tx
    std::vector<std::string> sensor_names;
    sensor_names = usrp_tx->get_tx_sensor_names(0);
    if (std::find(sensor_names.begin(), sensor_names.end(), "lo_locked")
        != sensor_names.end()) {
        uhd::sensor_value_t lo_locked = usrp_tx->get_tx_sensor("lo_locked", 0);
        std::cout << boost::format("Checking TX: %s ...") % lo_locked.to_pp_string() << std::endl;
        UHD_ASSERT_THROW(lo_locked.to_bool());
    }
    size_t mboard_sensor_idx = 0;
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

    // Check Ref and LO Lock detect for USRP Rx
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
    mboard_sensor_idx = 0;
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
    
    
    
    // ================================================
    // Create signals to transmit and UHD Tx streamers
    // ================================================
    
    // Generate baseband data to transmit
    std::vector<std::complex<float>> data_bb(10000);
    srand (1);
    for (size_t i = 0; i < 1000; i++){
        data_bb[i] = (2*(rand() % 2) -1) + (2*(rand() % 2) -1)*1j ;
    }
    for (size_t i = 1000; i < data_bb.size(); i++){
        data_bb[i] = 0.0;
    }
    
    // Generate LO signals to transmit
    std::vector<std::complex<float>> data_lo(10000);
    for (size_t i = 0; i < data_lo.size(); i++){
        data_lo[i] = 1.0;
    }
    
    // create a transmit streamer for USRP-Tx
    std::vector<size_t> channel_nums_tx = {0, 1};
    uhd::stream_args_t stream_args_tx("fc32", "sc16");
    stream_args_tx.channels = channel_nums_tx;
    uhd::tx_streamer::sptr stream_tx = usrp_tx->get_tx_stream(stream_args_tx);
    
    // create a transmit streamer for USRP-Rx-LO
    std::vector<size_t> channel_nums_rx_lo = {0};
    uhd::stream_args_t stream_args_rx_lo("fc32", "sc16");
    stream_args_rx_lo.channels = channel_nums_rx_lo;
    uhd::tx_streamer::sptr stream_rx_lo = usrp_rx_lo->get_tx_stream(stream_args_rx_lo);
    
    
    // =========================================================
    // start USRP-Tx worker thread and USRP-Rx-LO worker thread
    // =========================================================
    std::cout << boost::format("Starting USRP-Tx thread...") << std::endl;
    boost::thread_group tx_thread;
    tx_thread.create_thread(boost::bind(&tx_worker, data_bb, data_lo, stream_tx));
    std::cout << boost::format("Starting USRP-Rx-LO thread...") << std::endl;
    boost::thread_group rx_lo_thread;
    rx_lo_thread.create_thread(boost::bind(&rx_lo_worker, data_lo, stream_rx_lo));
    
    
    // ====================
    // Create Rx streamers
    // ====================
    // create a receive streamer
    std::vector<size_t> channel_nums_rx_bb = {0};
    uhd::stream_args_t stream_args_rx_bb("fc32", "sc16");
    stream_args_rx_bb.channels = channel_nums_rx_bb;
    uhd::rx_streamer::sptr rx_stream = usrp_rx_bb->get_rx_stream(stream_args_rx_bb);
    
    //meta-data will be filled in by recv()
    uhd::rx_metadata_t md;
    
    // allocate a buffer which we re-use for each channel
	size_t spb = rx_stream->get_max_num_samps(); 
    std::vector<std::complex<float>> 	buff_bb(spb);
    std::vector<std::complex<float>*> 	buffs(1);
    buffs[0] = &buff_bb.front(); 
    
    //the first call to recv() will block this many seconds before receiving
    double timeout = seconds_in_future + 0.1; //timeout 
    
    //setup streaming
	std::cout << boost::format("Begin streaming , %f seconds in the future...")  % seconds_in_future << std::endl;
	uhd::stream_cmd_t stream_cmd(uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS);
	stream_cmd.stream_now = false;
	stream_cmd.time_spec = uhd::time_spec_t(seconds_in_future);
	rx_stream->issue_stream_cmd(stream_cmd);
	
    
    // ========================================================
	// Start looping over all AiP directions at Tx and Rx side
	// ========================================================
	
    //float 			time_next_direction = 1.0; 	// initial time of first transmission
    std::string 	direction_tx, direction_rx;
    std::string 	degrees_tx, degrees_rx;
	std::string 	angle_tx, angle_rx; 
	
	mode_tx = 1;
	mode_rx = 2;
	
	init_aip(&my_serial_port_tx, ver_aip);
	init_aip(&my_serial_port_rx, ver_aip);
	
	// Loop over all Tx angles
    for (int cpt_direction_tx = 0; cpt_direction_tx < nbr_directions; cpt_direction_tx++){
    	for (int cpt_degrees_tx = 0; cpt_degrees_tx < nbr_degrees; cpt_degrees_tx++){
    		direction_tx = all_directions[cpt_direction_tx];
			if (cpt_direction_tx == 0){
				degrees_tx 	= all_degrees[nbr_degrees-1-cpt_degrees_tx];
				angle_tx 	= all_angles[nbr_degrees-1-cpt_degrees_tx];
			}
			else if (cpt_direction_tx == 1){
				degrees_tx 	= all_degrees[cpt_degrees_tx];
				angle_tx 	= all_angles[cpt_degrees_tx];
			}
			// Setting Tx AiP
			std::cout << boost::format("Setting Tx AiP to %s - %s ° at time %f") % direction_tx % angle_tx % usrp_tx->get_time_now().get_real_secs() << std::endl;
			send_to_aip_fast(&my_serial_port_tx, degrees_tx, direction_tx, gain_list_tx, gain_tx, active_list_tx, mode_tx, ver_aip);
    		
    		// Loop over all Rx angles
    		for (int cpt_direction_rx = 0; cpt_direction_rx < nbr_directions; cpt_direction_rx++){
    			for (int cpt_degrees_rx = 0; cpt_degrees_rx < nbr_degrees; cpt_degrees_rx++){
    				direction_rx = all_directions[cpt_direction_rx];
    				if (cpt_direction_rx == 0){
    					degrees_rx 	= all_degrees[nbr_degrees-1-cpt_degrees_rx];
    					angle_rx 	= all_angles[nbr_degrees-1-cpt_degrees_rx];
    				}
    				else if (cpt_direction_rx == 1){
    					degrees_rx 	= all_degrees[cpt_degrees_rx];
    					angle_rx 	= all_angles[cpt_degrees_rx];
    				}
    				// Setting Rx AiP
    				float time_now = usrp_rx_bb->get_time_now().get_real_secs() ;    	
					std::cout << boost::format("Setting Rx AiP to %s - %s ° at time %f") % direction_rx % angle_rx % time_now << std::endl;
					send_to_aip_fast(&my_serial_port_rx, degrees_rx, direction_rx, gain_list_rx, gain_rx, active_list_rx, mode_rx, ver_aip);
    				
    				// Write Rx and Tx AiP data to file
    				if (outfile.is_open()) {
						outfile << std::endl << "AiP Tx data" << std::endl ;
						outfile << boost::format("%s - %s degrees at time %f") % direction_tx % angle_tx % time_now;
						outfile << std::endl << "AiP Rx data" << std::endl ;
						outfile << boost::format("%s - %s degrees at time %f") % direction_rx % angle_rx % time_now;
						outfile << std::endl;
					}
					
					// Receive "nbr_samps_per_degree" samples
					if (outfile.is_open()) {
						outfile << std::endl << "USRP data" << std::endl ;
					}
					size_t num_acc_samps = 0; //number of accumulated samples
					while(num_acc_samps < nbr_samps_per_degree){
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
			}    	
			
    	}    
    }
    
    
    
    
    
    // ======================
    // Closing up everything 
    // ======================
    
    // Disable AiP Tx and Rx
    std::cout << std::endl << "Disabling mmWave Tx and Rx ..." << std::endl;
    disable_aip(&my_serial_port_tx, ver_aip);
    disable_aip(&my_serial_port_rx, ver_aip);
    
    // Close serial port
    std::cout << "Close serial ports for mmWave Tx and Rx ..." << std::endl;
    my_serial_port_tx.Close();
    my_serial_port_rx.Close();
    
    // Stopping all transmitter threads
    stop_signal_called = true;
    
    // finished
    std::cout << std::endl << "Done!" << std::endl << std::endl;
    return EXIT_SUCCESS;
}
