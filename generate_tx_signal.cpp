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

namespace po = boost::program_options;

/***********************************************************************
 * Main function
 **********************************************************************/
int UHD_SAFE_MAIN(int argc, char* argv[])
{
    
    // setup the program options
    po::options_description desc("Allowed options");
    // clang-format off
    desc.add_options()
		("help", "help message")
    ;
    // clang-format on
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    // print the help message
    if (vm.count("help")) {
        std::cout << boost::format("Generate the transmitted signal: ") % desc << std::endl;
        return ~0;
    }
    
    
    // Generate baseband data to transmit
    std::vector<std::complex<float>> data_bb(10000);
    srand (1);
    for (size_t i = 0; i < 1000; i++){
        data_bb[i] = (2*(rand() % 2) -1) + (2*(rand() % 2) -1)*1j ;
    }
    for (size_t i = 1000; i < data_bb.size(); i++){
        data_bb[i] = 0.0;
    }
    
    // Print out baseband data packet
    for (size_t i = 0; i < 1000; i++){
        std::cout << boost::format("tx_packet(%f) = %f + (%f)*1i ;") % (i+1) % std::real(data_bb[i]) % std::imag(data_bb[i]) << std::endl;
    }
    
   
    
    // finished
    std::cout << std::endl << "Done!" << std::endl << std::endl;
    return EXIT_SUCCESS;
}
