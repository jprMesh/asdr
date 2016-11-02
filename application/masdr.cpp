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

    //Initialize linked list of received buffer.
    standard_RecvNode.heading = 0;
    standard_RecvNode.next = NULL;
    recv_head.heading = 0;
    recv_head.next = NULL;
    
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
    
    ///10/31/16 MHLI: FILLER INFORMATION,REPLACE WITH CORRECT UPDATING
    phy_status.heading = 0;
    phy_status.is_rotating = false;
    phy_status.is_stationary = false;
    phy_status.location[0] = 0;
    phy_status.location[1] = 0;
    phy_status.location[2] = 0;

}

/******************************************************************************/
void Masdr::do_action() {
    if (soft_status == IDLE && phy_status.is_stationary && phy_status.is_rotating) {
        begin_sampling();
        soft_status = SAMPLE;

    } else if(soft_status == SAMPLE && phy_status.is_stationary && phy_status.is_rotating) {
        //Start new buffer

    } else if (soft_status == SAMPLE) {
        stop_sampling();
        begin_processing();
        soft_status = PROCESS;

    } else if (soft_status == PROCESS && process_done) {
        process_done = false;
        transmit_data();
        soft_status = TRANSMIT;

    } else if (soft_status == TRANSMIT && transmit_done) {
        transmit_done = false;
        // Notify ground station of idleness
        soft_status = IDLE;
    }
}

/******************************************************************************/
void Masdr::initialize_uhd() {
    /// 10/07/16 MHLI: Currently only going to config for rx.

    uhd::set_thread_priority_safe();

    int rate = 5e6;
    float freq_rx = 2400000000; //Set rx frequency to 2.4 GHz
    float freq_tx = 900e6; //set tx frequency
    int gain = 40;
    std::string rx_ant = "RX2"; //ant can be "TX/RX" or "RX2"
    std::string tx_ant = "TX/RX"; //ant can be "TX/RX" or "RX2"
    std::string wirefmt = "sc16"; //or sc8
    int setup_time = 1.0; //sec setup

    int bw =0; ///10/31/16 MHLI: Should probably be width of wifi stuff
    //Create USRP object
    uhd::usrp::multi_usrp::sptr usrp = uhd::usrp::multi_usrp::make((std::string)"");
    //Lock mboard clocks
    usrp->set_clock_source("internal"); //internal, external, mimo
    //set rates.
    usrp->set_rx_rate(rate);
    usrp->set_tx_rate(rate);
    //Set frequencies. 
    uhd::tune_request_t tune_request_rx(freq_rx);
    usrp->set_rx_freq(tune_request_rx); 
    uhd::tune_request_t tune_request_tx(freq_tx);
    usrp->set_tx_freq(tune_request_tx);
    //Set gain
    usrp->set_rx_gain(gain);
    usrp->set_tx_gain(gain);
    //set the antennas
    usrp->set_rx_antenna(rx_ant);
    usrp->set_tx_antenna(tx_ant);

    //allow for some setup time
    boost::this_thread::sleep(boost::posix_time::seconds(setup_time)); 
    //check Ref and LO Lock detect
    check_locked_sensor(usrp->get_rx_sensor_names(0),
                        "lo_locked",
                        boost::bind(&uhd::usrp::multi_usrp::get_rx_sensor,
                                    usrp, _1, 0),
                        setup_time);

    //create streamers
    //Initialize the format of memory (CPU format, wire format)
    uhd::stream_args_t stream_args("fc32","sc16");
    rx_stream = usrp->get_rx_stream(stream_args); //Can only be called once.
    tx_stream = usrp->get_tx_stream(stream_args); //Can only be called once.
}

/******************************************************************************/
void reconfig_uhd(int txrx) {

}

/******************************************************************************/
void Masdr::initialize_peripherals() {

}

/******************************************************************************/
void Masdr::begin_sampling() {
    uhd::stream_cmd_t stream_cmd(
        uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS);
    stream_cmd.num_samps = size_t(0);
    stream_cmd.stream_now = true;
    stream_cmd.time_spec = uhd::time_spec_t(); // Holds the time.
    rx_stream->issue_stream_cmd(stream_cmd);   // Initialize the stream
}

/******************************************************************************/
void Masdr::stop_sampling() {
    uhd::stream_cmd_t stream_cmd(
        uhd::stream_cmd_t::STREAM_MODE_STOP_CONTINUOUS);
    rx_stream->issue_stream_cmd(stream_cmd);
}

/******************************************************************************/
void Masdr::rx_test(){
    int i; //Counter, to help test
    std::complex<float> testbuf[100];
    std::cout << "Entered rx_test" << std::endl;

    begin_sampling();
    std::cout << "Began sampling" << std::endl;
    rx_stream->recv(testbuf, 100, md, 3.0, false);
    std::cout << "First Buff done" << std::endl;
        
    while (i < 5000) {
        rx_stream->recv(testbuf, 100, md, 3.0, false);
        ++i;
    }

    stop_sampling();
    std::cout << "Stopped sampling" << std::endl;
}

/******************************************************************************/
void Masdr::tx_test() {
    int i; //Counter, to help test
    std::complex<float> testbuf[100];
    std::cout << "Entered tx_test" << std::endl;

    //Initialize test buffer.
    memset(testbuf, 0, 100 * sizeof(std::complex<float>));
    i = 0;


    uhd::tx_metadata_t md;
    md.start_of_burst = false;
    md.end_of_burst = false;

    std::cout << "Began transmit" << std::endl;
    tx_stream->send(testbuf, 100, md);
    std::cout << "First Buff done" << std::endl;
        
    while (i < 5000) {
        tx_stream->send(testbuf, 100, md);
        ++i;
    }

    std::cout << "Stopped transmit" << std::endl;
}

/******************************************************************************/
void Masdr::begin_processing() {
 //Energy Detection
    //Threshold?
}

/******************************************************************************/
void Masdr::transmit(const void *msg, int len) { 
    //raised cosine pulse shaping
    //Mod Scheme: BPSK
    //tx
}

/******************************************************************************/
void Masdr::transmit_data() {
    //Form Packet
    //Interleave
    //Crc
    // call transmit
}

/******************************************************************************/
void Masdr::shutdown_uhd() {

};

/******************************************************************************/
bool check_locked_sensor(std::vector<std::string> sensor_names,
                         const char* sensor_name,
                         get_sensor_fn_t get_sensor_fn,
                         double setup_time){
    if (std::find(sensor_names.begin(),
                  sensor_names.end(),
                  sensor_name) == sensor_names.end()) {
        return false;
    }
 
    boost::system_time start = boost::get_system_time();
    boost::system_time first_lock_time;
 
    std::cout << boost::format("Waiting for \"%s\": ") % sensor_name;
    std::cout.flush();
 
    while(1) {
        if (!first_lock_time.is_not_a_date_time()
            && (boost::get_system_time()
                > (first_lock_time + boost::posix_time::seconds(setup_time)))) {
            std::cout << " locked." << std::endl;
            break;
        }
        if (get_sensor_fn(sensor_name).to_bool()) {
            if (first_lock_time.is_not_a_date_time()) {
                first_lock_time = boost::get_system_time();
            }
            std::cout << "+" << std::flush;
        }
        else {
            first_lock_time = boost::system_time(); //reset to 'not a date time'
 
            if (boost::get_system_time()
                > (start + boost::posix_time::seconds(setup_time))) {
                std::cout << std::endl;
                throw std::runtime_error(str(boost::format(
                    "Timed out waiting for consecutive locks on sensor \"%s\""
                    ) % sensor_name));
            }
            std::cout << "_" << std::flush;
        }
        boost::this_thread::sleep(boost::posix_time::milliseconds(100));
    }
    std::cout << std::endl;
    return true;
}

/******************************************************************************/
void handle_sigint(int) {
    stop_signal_called = true;
    exit(0);
}

/******************************************************************************/
int UHD_SAFE_MAIN(int argc, char *argv[]) {
    signal(SIGINT, handle_sigint);
    Masdr masdr;
    //Test transmission and receiving.
    masdr.rx_test();
    masdr.tx_test();

    while(1) {
        masdr.update_status();
        masdr.state_transition();
    }
    return EXIT_SUCCESS;
}

