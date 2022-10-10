#
# Copyright 2010-2015 Ettus Research LLC
# Copyright 2018 Ettus Research, a National Instruments Company
#
# SPDX-License-Identifier: GPL-3.0-or-later
#

########################################################################
# mmwave_code applications
########################################################################
set(mmwave_code_sources
    mmwave_tx.cpp
    mmwave_rx.cpp
    mmwave_array_test.cpp
    mmwave_joint_txrx.cpp
    generate_tx_signal.cpp
    mmwave_array_turnRxOn.cpp
)


#for each source: build an executable and install
foreach(mmwave_code_source ${mmwave_code_sources})
    get_filename_component(mmwave_code_name ${mmwave_code_source} NAME_WE)
    add_executable(${mmwave_code_name} ${mmwave_code_source})
    target_link_libraries(${mmwave_code_name} 
    	uhd 
    	/usr/local/lib/libserial.so
    	${Boost_LIBRARIES})
    UHD_INSTALL(TARGETS ${mmwave_code_name} RUNTIME DESTINATION ${PKG_LIB_DIR}/mmwave_code COMPONENT mmwave_code)
endforeach(mmwave_code_source)

