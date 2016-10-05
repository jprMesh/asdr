// File: masdr.cpp
//
// MASDR Project 2016
// WPI MQP E-Project number: 
// Members: Jonas Rogers
//          Kyle Piette
//          Max Li
//          Narut Akadejdechapanich
//          Scott Iwanicki
// Advisor: Professor Alex Wyglinski
// Sponsor: Gryphon Sensors

#include "masdr.h"

/******************************************************************************/
Masdr::Masdr() {
    process_done = false;
    transmit_done = false;
    initialize_peripherals();
    initialize_uhd();
    update_status();
}

/******************************************************************************/
Masdr::~Masdr() {
    shutdown_uhd();
}

/******************************************************************************/
void Masdr::update_status() {
    // update phy_status from peripherals
}

/******************************************************************************/
void Masdr::do_action() {
    if (soft_status == IDLE && phy_status.is_stationary) {
        begin_sampling();
        soft_status = SAMPLE;
    } else if (soft_status == SAMPLE && !phy_status.is_stationary) {
        stop_sampling();
        begin_processing();
        soft_status = PROCESS;
    } else if (soft_status == PROCESS && process_done) {
        process_done = false;
        transmit_data();
        soft_status = TRANSMIT;
    } else if (soft_status == TRANSMIT && transmit_done) {
        transmit_done = false;
        transmit(SoftStatus.IDLE); // Notify ground station of idleness
        soft_status = IDLE;
    }
}

/******************************************************************************/
void Masdr::initialize_uhd() {
    
    uhd::set_thread_priority_safe();
    int spb = 10000; //Numbers of samples in a buffer
    int rate = 640000; //Cannot = 0
    float freq_rx = 2.4e9; //Set rx frequency
    float freq_tx = 5.8e9; //set tx frequency
    int gain = 40;
    std::string ant  = "RX/TX";  //ant can be "RX/TX" or "RX2"
    wirefmt = "sc16"; //or sc8
    setup_time = 1.0; //sec setup
     
    //Create USRP object
    uhd::usrp::multi_usrp::sptr usrp = uhd::usrp::multi_usrp::make(args);
    //Lock mboard clocks
    usrp->set_clock_source("internal"); //internal, external, mimo
    //set rx rate
    usrp->set_rx_rate(rate);
    //Set rx freq. 
    //10/03/16 MHLI: Setting different freqs for tx and rx, not sure if works yet.
    uhd::tune_request_t tune_request_rx(freq_rx);
    uhd::tune_request_t tune_request_tx(freq_tx);
    usrp->set_tx_freq(tune_request_tx);
    usrp->set_rx_freq(tune_request_rx);
    //Set gain
    usrp->set_rx_gain(gain);
    
    //set the antenna
    if (ant != "NULL") usrp->set_rx_antenna(ant);
    //allow for some setup time
    boost::this_thread::sleep(boost::posix_time::seconds(setup_time)); 
    //check Ref and LO Lock detect
    check_locked_sensor(usrp->get_rx_sensor_names(0), "lo_locked", boost::bind(&uhd::usrp::multi_usrp::get_rx_sensor, usrp, _1, 0), setup_time);

     //create a receive streamer
    uhd::stream_args_t stream_args(cpu_format,wire_format); //Initialize the format of memory
    uhd::rx_streamer::sptr rx_stream = usrp->get_rx_stream(stream_args); //Can only be called once.
    toRecv.recv_stream = rx_stream; 
       
    //setup streaming
    uhd::stream_cmd_t stream_cmd(uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS);
    stream_cmd.num_samps = size_t(0);
    stream_cmd.stream_now = true;
    
    // stream_cmd.time_spec = uhd::time_spec_t(); //holds the time.
    // rx_stream->issue_stream_cmd(stream_cmd);   //sends the stream command to initialize.
}

/******************************************************************************/
void reconfig_uhd(int txrx) {

}

/******************************************************************************/
void Masdr::initialize_peripherals() {

}

/******************************************************************************/
void Masdr::begin_sampling() {

}

/******************************************************************************/
void Masdr::stop_sampling() {

}

/******************************************************************************/
void Masdr::begin_processing() {

}

/******************************************************************************/
void Masdr::transmit(const void *msg, int len) {

}

/******************************************************************************/
void Masdr::transmit_data() {
    // call transmit
}

/******************************************************************************/
int main(int argc, char const *argv[]) {

    Masdr masdr;

    while(1) {
        masdr.update_status();
        masdr.do_action();
    }
    return 0;
}
