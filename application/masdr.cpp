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

    // Initialize linked list of received buffer.
    recv_head.heading = 0;
    recv_head.next = NULL;
    
    //Initialize FFTW
    fft_in = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * N_FFT);
    fft_out = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * N_FFT);
    fft_p = fftw_plan_dft_1d(N_FFT, fft_in, fft_out, FFTW_FORWARD, FFTW_MEASURE);

    initialize_peripherals();
    initialize_uhd();
    update_status();
    
}

/******************************************************************************/
Masdr::~Masdr() {
    //Shutdown fftw
    fftw_destroy_plan(fft_p);
    fftw_free(fft_in); 
    fftw_free(fft_out);
    shutdown_uhd();
    delete recv_head.next;
    delete trans_head;
}
/******************************************************************************/
/***************************STATE TRANSITIONS**********************************/
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
        // Receive into current buffer
        curr_recv_buf->heading = phy_status.heading;
        rx_stream->recv(curr_recv_buf->recv_buf,RBUF_SIZE,md,3.0,false);
        // Start new buffer
        RecvNode *new_node = new RecvNode; // should be initialized to 0.
        curr_recv_buf->next = new_node;
        curr_recv_buf = curr_recv_buf->next;

    } else if (soft_status == PROCESS) {
        ;

    } else if (soft_status == TRANSMIT) {
        ;

    } else if (soft_status == IDLE) {
        ;
    }
}

/******************************************************************************/
/**************************INITIALIZATIONS*************************************/
/******************************************************************************/
void Masdr::initialize_peripherals() {

}

/******************************************************************************/
void Masdr::initialize_uhd() {
    /// 10/07/16 MHLI: Currently only going to config for rx.

    uhd::set_thread_priority_safe();

    int rate = 25e6;
    float freq_rx = 2.4e9; //Set rx frequency to 2.4 GHz
    //float freq_rx = 700e6; //11/6/16 MHLI: TEST, for when we only have 900 MHz Ant
    
    float freq_tx = 905e6; //set tx frequency
    int gain = 50;
    std::string rx_ant = "RX2"; //ant can be "TX/RX" or "RX2"
    std::string tx_ant = "TX/RX"; //ant can be "TX/RX" or "RX2"
    std::string wirefmt = "sc16"; //or sc8
    int setup_time = 1.0; //sec setup

    //int bw =10e6; ///10/31/16 MHLI: UP to 56e6
    int rx_bw = 20e6;
    int tx_bw = 0;
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
    //Set BW
    usrp->set_rx_bandwidth(rx_bw);
    usrp->set_tx_bandwidth(tx_bw);
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

}

/******************************************************************************/
/************************************SAMPLE************************************/
/******************************************************************************/

void Masdr::begin_sampling() {
    // Initialize new sampling stream
    uhd::stream_cmd_t stream_cmd(
        uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS);
    stream_cmd.num_samps = size_t(0);
    stream_cmd.stream_now = true;
    stream_cmd.time_spec = uhd::time_spec_t(); // Holds the time.
    rx_stream->issue_stream_cmd(stream_cmd);   // Initialize the stream
    // Set current buffer to head
    curr_recv_buf = &recv_head;
}

/******************************************************************************/
void Masdr::stop_sampling() {
    uhd::stream_cmd_t stream_cmd(
        uhd::stream_cmd_t::STREAM_MODE_STOP_CONTINUOUS);
    rx_stream->issue_stream_cmd(stream_cmd);
    // Clear linked list
    delete recv_head.next;
    recv_head.next = NULL;
}

/******************************************************************************/
/********************************PROCESSING************************************/
/******************************************************************************/
void Masdr::begin_processing() {

    //while data structure is null, energy detect current buffer,
    bool hasEnergy = energy_detection(curr_recv_buf->recv_buf,RBUF_SIZE);
    if(hasEnergy) {
     //   run_fft(curr_recv_buf->recv_buf);
     //   match_filt();
      //  localize();
    }
}

/******************************************************************************/
bool Masdr::energy_detection(std::complex<float> *sig_in, int size){
    float acc=0,max=0, mag;
    int i;
    for (i = 0; i < size; i++) {
        mag = sqrt(sig_in[i].real()*sig_in[i].real()+sig_in[i].imag()*sig_in[i].imag());
        acc += mag;
        if(max < mag)
            max = mag;
    }

    // if(DEBUG_THRESH) {    
    //     std::cout<< max ;        
    //     for (i=0; i < (int)(mag*1000); i++){
    //         std::cout << "#";
    //     }
    //     std::cout<<std::endl;
    // }

    if(max > THRESH_E)
        return true;
    else
        return false;
}

/******************************************************************************/
/*********************************TRANSMISSION*********************************/
/******************************************************************************/
void Masdr::transmit_data() {
    if (trans_head == NULL) {
        std::cout << "No values to transmit" << std::endl;
        return;
    }
    int i; // looping
    int bias = 0; // compensating shift for adding more data
    TransNode* trans_temp = trans_head;
    std::complex<float> transmitBuffer[TBUF_SIZE];

    // used to perform binary operations on floats
    union {
        float input;
        int output;
    } data;

    std::cout << "Starting packaging" << std::endl;
    while (trans_temp != NULL) {
        //packing 33 start bits
        for(i = 0; i < 33; i++) {
            transmitBuffer[i+bias] = std::complex<float>(1,0);
        }
        bias += 33; // compensate for adding start bits

        //packing magnetometer data
        data.input = trans_temp->heading;
        for(i = 0; i < 32; i++) {
            if ((data.output >> (31 - i)) & 1)
                transmitBuffer[i+bias] = std::complex<float>(1,0);
            else 
                transmitBuffer[i+bias] = std::complex<float>(-1,0);
        }
        bias += 32; // compensate for adding magnetometer data

        //packing gps data
        data.input = trans_temp->gps[0];
        for(i = 0; i < 32; i++) {
            if ((data.output >> (31 - i)) & 1)
                transmitBuffer[i+bias] = std::complex<float>(1,0);
            else 
                transmitBuffer[i+bias] = std::complex<float>(-1,0);
        }
        bias += 32; // compensate for adding gps data 0

        data.input = trans_temp->gps[1];
        for(i = 0; i < 32; i++) {
            if ((data.output >> (31 - i)) & 1)
                transmitBuffer[i+bias] = std::complex<float>(1,0);
            else 
                transmitBuffer[i+bias] = std::complex<float>(-1,0);
        }
        bias += 32; // compensate for adding gps data 1

        data.input = trans_temp->gps[2];
        for(i = 0; i < 32; i++) {
            if ((data.output >> (31 - i)) & 1)
                transmitBuffer[i+bias] = std::complex<float>(1,0);
            else 
                transmitBuffer[i+bias] = std::complex<float>(-1,0);
        }
        bias += 32; // compensate for adding gps data 2

        //packing data
        data.input = trans_temp->data;
        for(i = 0; i < 32; i++) {
            if ((data.output >> (31 - i)) & 1)
                transmitBuffer[i+bias] = std::complex<float>(1,0);
            else 
                transmitBuffer[i+bias] = std::complex<float>(-1,0);
        }
        bias += 32; // compensate for adding data

        //packing 33 end bits
        for(i = 0; i < 33; i++) {
            transmitBuffer[i+bias] = std::complex<float>(-1,0);
        }
        
        std::cout << "Transmitting..." << std::endl;
        transmit(transmitBuffer, TBUF_SIZE);
        trans_temp = trans_temp->next;
    }

    // delete the values after sending
    delete trans_head;
    // set to null for checking when filling in the linkedlist
    trans_head = NULL;
    curr_trans_buf = NULL;
    
    // TESTING
    // for(i = 0; i < TBUF_SIZE; i++) {
    //     transmitBuffer[i] = std::complex<float>(1,0);
    // }

    std::cout << "Done with transmit" << std::endl;
}
/******************************************************************************/

void Masdr::transmit(std::complex<float> *msg, int len) { 
    uhd::tx_metadata_t md;
    md.start_of_burst = false;
    md.end_of_burst = false;
    tx_stream->send(msg, len * sizeof(std::complex<float>), md);
}

/******************************************************************************/
/************************************TESTS*************************************/
/******************************************************************************/
void Masdr::rx_test(){
    int i=0,j, numLoops; //Counter, to hel, p 
    float accum, max_inBuf=0,max_periodic = 0, max_total=0, mag_squared;
    std::complex<float> testbuf[RBUF_SIZE];
    std::cout << "Entered rx_test" << std::endl;

    begin_sampling();
    std::cout << "Began sampling" << std::endl;
    rx_stream->recv(testbuf, RBUF_SIZE, md, 3.0, false);
    std::cout << "First Buff done" << std::endl;

    if(DEBUG_THRESH)
        while (1){//(i < 5000) {
        // accum = 0;
        // rx_stream->recv(testbuf, RBUF_SIZE, md, 3.0, false);
        
        // for(j=0;j<RBUF_SIZE;j++) {
        //     mag_squared = (sqrt(testbuf[j].real() *testbuf[j].real()
        //               + testbuf[j].imag()*testbuf[j].imag()));
        //     accum += mag_squared;

        //     if(mag_squared > max_inBuf)
        //         max_inBuf = mag_squared;
        //     if(mag_squared > max_total)
        //         max_total = mag_squared;
        //     if(mag_squared > max_periodic)
        //         max_periodic = mag_squared;
        // }
        // if(!(i%20)){
        //     std::cout<<(int)accum*SCALE_ACC;
        //     for (i=0; i < (int)accum*SCALE_ACC; i++){
        //         std::cout << "#";
        //     }
        //    std::cout<<std::endl;
        //     // std::cout<< i <<": ";
        // //std::cout<< "Max in buf: "<<max_inBuf <<std::endl;
        // //std::cout<< "Max in total"<< max_total <<std::endl;
        // // std::cout << "Max in Periodic: "<<max_periodic <<std::endl;
        // }
        // max_inBuf = 0;
        // //std::cout << "Received Value: "<<accum<<std::endl;
        // ++i;
        // if(i > 5000){
        //     i = 0;
        //     max_periodic = 0;
        // }

        rx_stream->recv(testbuf, RBUF_SIZE, md, 3.0, false);
        std::cout<<energy_detection(testbuf, RBUF_SIZE)<<std::endl;
    }
    else 
        numLoops = 5000;


    while(i<numLoops && !DEBUG_THRESH){
        rx_stream->recv(testbuf, RBUF_SIZE, md, 3.0, false);
    }

    stop_sampling();
    std::cout << "Stopped sampling" << std::endl;
    std::cout <<"RX test done." <<std::endl<<std::endl;
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
    std::cout <<"Tx test done." <<std::endl<<std::endl;
}

/******************************************************************************/
void Masdr::mag_test() {
    float deg;
    init_mag();
    while(1){
        deg=read_mag();
        std::cout<< "Mag Reading: "<<deg <<std::endl;
        usleep(4000000);
    }
        
}
/******************************************************************************/
void Masdr::RecvNode_test() {
    std::cout << "Begin testing" << std::endl;
    recv_head.heading = 180;
    curr_recv_buf = &recv_head;
    for (int i=0; i<400; ++i) {
        RecvNode *newnode = new RecvNode;
        newnode->heading = int(curr_recv_buf->heading + 10) % 360;
        curr_recv_buf->next = newnode;
        curr_recv_buf = curr_recv_buf->next;
    }
    std::cout << "Done filling list" << std::endl;
    std::cout << recv_head.next->heading << std::endl;
    std::cout << recv_head.next->next->heading << std::endl;
    std::cout << recv_head.next->next->next->heading << std::endl;
    delete recv_head.next;
    recv_head.next = NULL;
    std::cout << "Done freeing list" << std::endl;
    std::cout << recv_head.next << std::endl;
}

/******************************************************************************/
void Masdr::energy_test(){
    //Test energy detection stuff.
    std::cout<<"Energy test done." <<std::endl<<std::endl;
}
/******************************************************************************/
void Masdr::transmit_data_test(){
    int i;
    std::cout <<"In tx data test" <<std::endl;
    transmit_data();    
}
/******************************************************************************/
void Masdr::fft_test(){
    int i, numTests = 100, max_index = 0, real_index;
    int freq = 3000000;
    int freq_scale = 25000000;
    double max_mag = 0, magnitude;
    fftw_plan p2;
    fftw_complex *fft_in2 = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * N_FFT);
    p2 = fftw_plan_dft_1d(N_FFT, fft_out, fft_in2, FFTW_BACKWARD, FFTW_MEASURE);


    for(i = 0; i <N_FFT; i++){
        // in[0][i] = cos(2 * PI * freq * i / freq_scale);
        fft_in[i][0] = 1;
        fft_in[i][1] = 0;
    }


    std::cout<< "Test Forward"<<std::endl<<std::endl;
    fftw_execute(fft_p); /* repeat as needed */   
    for (i = 0; i < N_FFT; i++){
        magnitude =sqrt(fft_out[i][0]*fft_out[i][0] + fft_out[i][1]*fft_out[i][1]);
        if(magnitude > max_mag){
            max_mag = magnitude;
            max_index = i;
        }
        std::cout << "Real: "<<fft_out[i][0]<< "\tImaginary: " << fft_out[i][1]<<std::endl;
    }

    std::cout << "Max Magnitude: "<<max_mag<< " at index: " << max_index<<std::endl;
    std::cout << "Bucket represents " <<max_index * freq_scale /2/ N_FFT <<std::endl;
    
    std::cout<< "Test Backward" <<std::endl<<std::endl;    
    // for(i = 0; i < N_FFT; i++){
    //     out[i][0] = 0;
    //     out[i][1] = 0;
    // } 
    // out[0][N_FFT/2] = 1;
    max_mag = 0;
   fftw_execute(p2);
   for (i = 0; i < N_FFT; i++){
            magnitude =sqrt(fft_in2[i][0]*fft_in2[i][0] + fft_in2[i][1]*fft_in2[i][1]);
            if(magnitude > max_mag){
                max_mag = magnitude;
                max_index = i;
            }
        std::cout << "Real: "<<fft_in2[i][0]<< "\tImaginary: " << fft_in2[i][1]<<std::endl; 
    }   


        std::cout << "Max Magnitude: "<<max_mag/N_FFT<< " at index: " << max_index<<std::endl;
        std::cout << "Bucket represents " <<max_index * freq_scale /2/ N_FFT <<std::endl;
        
}

/******************************************************************************/
int UHD_SAFE_MAIN(int argc, char *argv[]) {
    signal(SIGINT, handle_sigint);
    Masdr masdr;

    if(G_DEBUG){
        if(DEBUG_THRESH) masdr.rx_test();
        if(DEBUG_TX) masdr.tx_test();
        if(DEBUG_ENERGY)masdr.energy_test();
        if(DEBUG_MAG)masdr.mag_test();
        if(DEBUG_FFT) masdr.fft_test();
        if(DEBUG_TX_DATA) masdr.transmit_data_test();
    }
    /// 11/6/16 MHLI: Commented out while we test energy functions
    else{
        while(1) {
            masdr.update_status();
            masdr.state_transition();
            masdr.repeat_action();
        }
    }

    return EXIT_SUCCESS;
}

/******************************************************************************/

