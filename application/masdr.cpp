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
#include "utils.h"

/******************************************************************************/
Masdr::Masdr() {
    process_done = false;
    transmit_done = false;

    //Initialize linked list of received buffer.
    recv_head->heading = 0;
    recv_head->next = NULL;
    
    initialize_peripherals();
    initialize_uhd();
    update_status();
}

/******************************************************************************/
Masdr::~Masdr() {
    shutdown_uhd();
    delete recv_head->next;
}

/******************************************************************************/
void Masdr::update_status() {
    // update phy_status from peripherals
    
    ///10/31/16 MHLI: FILLER INFORMATION,REPLACE WITH CORRECT UPDATING
    phy_status.heading = 0;
    phy_status.is_stat_and_rot = false;
    phy_status.location[0] = 0;
    phy_status.location[1] = 0;
    phy_status.location[2] = 0;

}

/******************************************************************************/
void Masdr::state_transition() {
    if (soft_status == IDLE && phy_status.is_stat_and_rot) {
        begin_sampling();
        soft_status = SAMPLE;

    } else if (soft_status == SAMPLE && !phy_status.is_stat_and_rot) {
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
void Masdr::repeat_action() {
    if (soft_status == SAMPLE) {
        //Start new buffer
        RecvNode *new_rec = new RecvNode; // should be initialized to 0.
        curr_recv_buff->next = new_rec;
        curr_recv_buff = curr_recv_buff->next;
        curr_recv_buff->heading = phy_status.heading;
        rx_stream->recv(curr_recv_buff->recv_buf, RBUF_SIZE, md, 3.0, false);

    } else if (soft_status == PROCESS) {
        ;

    } else if (soft_status == TRANSMIT) {
        ;

    } else if (soft_status == IDLE) {
        ;
    }
}

/******************************************************************************/
    ///11/3/16 MHLI: WIP
void Masdr::test_RecvNode() {

    RecvNode *curent_head = new RecvNode;
    recvNode *front = new_rec;
    front->

    RecvNode *new_rec = new RecvNode; // should be initialized to 0.
    new_rec->heading = 0;
    new_rec->next = NULL;
    curr_recv_buff->next = new_rec;
    curr_recv_buff = curr_recv_buff->next;
    curr_recv_buff->heading = phy_status.heading;
    

}


/******************************************************************************/
void Masdr::initialize_peripherals() {

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
void Masdr::reconfig_uhd(int txrx) {

}

/******************************************************************************/
void Masdr::shutdown_uhd() {

};

/******************************************************************************/
void Masdr::begin_sampling() {
    // Initialize save buffer
    curr_recv_buff = recv_head;
    curr_recv_buff->heading = phy_status.heading;
    rx_stream->recv(curr_recv_buff->recv_buf,RBUF_SIZE,md,3.0,false);
    // Initialize new sampling stream
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
    // Clear linked list
    delete recv_head->next;
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
    // FFT
    // fftw_complex in[FFT_N], out[FFT_N];
    // fftw_plan p = fftw_create_plan(FFT_N, FFTW_FORWARD, FFTW_ESTIMATE);
    // fftw_one(p, in, out);
    // fftw_destroy_plan(p);
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
int UHD_SAFE_MAIN(int argc, char *argv[]) {
    signal(SIGINT, handle_sigint);
    Masdr masdr;
    //Test transmission and receiving.
    masdr.rx_test();
    masdr.tx_test();

    while(1) {
        masdr.update_status();
        masdr.state_transition();
        masdr.repeat_action();
    }
    return EXIT_SUCCESS;
}
