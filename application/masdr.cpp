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
    soft_status = IDLE;
    // Initialize linked list of received buffer.
    recv_head.heading = 0;
    recv_head.next = NULL;
    
    //Initialize FFTW
    fft_in = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * N_FFT);
    fft_out = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * N_FFT);
    fft_p = fftw_plan_dft_1d(N_FFT, fft_in, fft_out, FFTW_FORWARD, FFTW_MEASURE);

    //Intialize OFDM Match Filter head
    //OFDM HEAD DETAILS:
        //Ignore first and last 174 buckets.
        //Each OFDM bucket is 16 fft buckets.
        //They are located at 254-270, 480-415, 640-615,864-880
    int i;
    for(i=0;i<N_FFT;i++){
        int ratio = N_FFT/64; //Number of fft bins in an OFDM bin.

        //Account for the fact that only middle 52 OFDM bins are used.
        if (i < 6 * ratio || i > 58 * ratio){
            ofdm_head[i][0] = 0;
            ofdm_head[i][1] = 0;
        }
        else if(i >= (6+6) * ratio && i < (6+7) * ratio){
            ofdm_head[i][0] = 1;
            ofdm_head[i][1] = 1;
        }
        else if(i >= (12+14) * ratio && i < (12+15) * ratio) {
            ofdm_head[i][0] = 1;
            ofdm_head[i][1] = 1;
        }
        else if(i >= (26+14) * ratio && i < (26+15) * ratio) {
            ofdm_head[i][0] = 1;
            ofdm_head[i][1] = 1;
        }
        else if(i >= (40+14) * ratio && i < (40 + 15) * ratio){
            ofdm_head[i][0] = 1;
            ofdm_head[i][1] = 1;
        }
        else {
            ofdm_head[i][0] = 0;
            ofdm_head[i][1] = 0;
        }
    }
    /// 12/4/16 MHLI: Still have to figure out what omega, Ts, N_RRC should be.

    //     The RRC impulse response is given by:
    // h(t) =     pi^2      4at cos(t(a+b))+pi sin(t(b-a))
    //        ----------- * ------------------------------
    //         pi(a-b)-4a         t(16t^2 a^2 - pi^2)
    // where b= 2 pi f   (f is usually half of your symbol rate)
    // and   a= 2 pi excess bandwidth

    float time;
    //Maybe internal freq should be 905e6,idk
    int freq = (870e3/4)/2; //Currently 1/2 symbol rate
    float excess=0.2, b = 2*PI * freq, a = 2*PI*excess;
    float Ts = 1/freq;
    // float omega=2*PI*freq_tx; //2pif

    for(i=0; i < N_RRC;i++){
        if(i = N_RRC/2)
            rrcBuf[i] = 1;
        else{
            time = (i - N_RRC/2)*Ts;
            rrcBuf[i] = PI*PI/(PI*(a-b)-4*a) * 
                        4*a*time*cos(time*(a+b))+ PI *sin(time*(b-a)) /
                            time*(16 * time * time * a * a - PI * PI);
        }
    }
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
    /// 11/17/16 MHLI: Commented out to test sampling.
    if(soft_status != SAMPLE){
        begin_sampling();
    }
    soft_status = SAMPLE;
    // if (soft_status == IDLE && phy_status.is_stat_and_rot) {
    //     begin_sampling();
    //     soft_status = SAMPLE;

    // } else if (soft_status == SAMPLE && !phy_status.is_stat_and_rot) {
    //     stop_sampling();
    //     begin_processing();
    //     soft_status = PROCESS;

    // } else if (soft_status == PROCESS && process_done) {
    //     process_done = false;
    //     transmit_data();
    //     soft_status = TRANSMIT;

    // } else if (soft_status == TRANSMIT && transmit_done) {
    //     transmit_done = false;
    //     // Notify ground station of idleness
    //     soft_status = IDLE;
    // }
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
    uhd::set_thread_priority_safe();

    int rx_rate = 42e6;//To deal with the 20MHz bandwidth we have.
    int tx_rate = 870e3; //4 samples per symbol at 700kHz
    int master_rate = 42e6;
    float freq_rx = 2.4e9; //Set rx frequency to 2.4 GHz
    //float freq_rx = 700e6; //11/6/16 MHLI: TEST, for when we only have 900 MHz Ant
    
    float freq_tx = 905e6; //set tx frequency
    int gain = 50;
    std::string rx_ant = "RX2"; //ant can be "TX/RX" or "RX2"
    std::string tx_ant = "TX/RX"; //ant can be "TX/RX" or "RX2"
    std::string wirefmt = "sc16"; //or sc8
    int setup_time = 1.0; //sec setup

    //int bw =10e6; ///10/31/16 MHLI: UP to 56e6
    //Only care about 16.6 MHz of the 20 (or 8.3MHz of 10)
    int rx_bw = 20e6; //11/16/16 MHLI: Should this be 10e6 and will it do half above half below?
    int tx_bw = 300e3; 
    // int tx_bw = 0;
    //Create USRP object
    uhd::usrp::multi_usrp::sptr usrp = uhd::usrp::multi_usrp::make((std::string)"");
    //Lock mboard clocks
    usrp->set_clock_source("internal"); //internal, external, mimo
    usrp->set_master_clock_rate(master_rate);
    //set rates.
    usrp->set_rx_rate(rx_rate);
     usrp->set_tx_rate(tx_rate);

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
    //post-running wrapping up
    uhd::stream_cmd_t stream_cmd(
        uhd::stream_cmd_t::STREAM_MODE_STOP_CONTINUOUS);
    rx_stream->issue_stream_cmd(stream_cmd);
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
        run_fft(curr_recv_buf->recv_buf);
        float has_wifi =  match_filt();
        if(has_wifi != -1){
          localize();
        }
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
void  Masdr::run_fft(std::complex<float> *buff_in){
    int i;
    for(i = 0; i < N_FFT;i++){
        fft_in[i][0] = buff_in[i].real();
        fft_in[i][1] = buff_in[i].imag();
    }
    fftw_execute(fft_p);

}

/******************************************************************************/
float Masdr::match_filt(){
    //Match filter.
    int i, j;
    float match_val[2] = {0,0}, match_mag, re, im;
    for(i = 0; i < N_FFT; i++) {
        re = fft_out[i][0] * ofdm_head[i][0] - fft_out[i][1] * ofdm_head[i][1];
        im = fft_out[i][0] * ofdm_head[i][1] + fft_out[i][1] * ofdm_head[i][0];
        match_val[0] += re;
        match_val[1] += im;

    }
    match_mag = sqrt(match_val[0]*match_val[0]+match_val[1]*match_val[1]);

    if(match_mag > THRESH_MATCH)
        return match_mag;
    else
        return 0;
}

/******************************************************************************/
float * Masdr::localize(){
    //Calculate distance from RSS
    //calculate angle from distance
    return NULL;
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

    /*
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
        */
        /// 12/4/15 MHLI:Deal with root raised cosine.
        std::complex<float> transmitBuffer_rrc[SPS*TBUF_SIZE];
        std::complex<float> transmitBuffer_final[SPS*TBUF_SIZE];
        for(i = 0; i < SPS*TBUF_SIZE; i++){
            if(!i%SPS)
                transmitBuffer_rrc[i] = transmitBuffer[i/SPS];
            else
                transmitBuffer_rrc[i] = 0;
        }

        int j,k;
        float accum = 0;
        //Convolve w/ root raised cosine.
        for(i=0; i < SPS*TBUF_SIZE;i++){
            for(j=0; j < N_RRC;j++){
                k = i-j;
                if(!k)
                    accum += 0;
                else
                    accum += transmitBuffer_rrc[k].real()*rrcBuf[j];
            }
            transmitBuffer_final[i] = {accum,0};
        }

        /*
        std::cout << "Transmitting..." << std::endl;
        transmit(transmitBuffer, TBUF_SIZE);
        trans_temp = trans_temp->next;
    }

    // delete the values after sending
    delete trans_head;
    // set to null for checking when filling in the linkedlist
    trans_head = NULL;
    curr_trans_buf = NULL;
    */
    
    //TESTING
    //Use DBPSK in transmission ASSUME NO CHANGE IS 0.
    std::cout << "Start transmit" << std::endl;
    for(i = 0; i < TBUF_SIZE; i++) {
        // if(i%6 == 0 || 6 == 5)
        //     transmitBuffer[i] = std::complex<float>(1,0);
        // else
            transmitBuffer[i] = std::complex<float>(-1,0);

    }
        uhd::tx_metadata_t md;
    md.start_of_burst = false;
    md.end_of_burst = false;
    while(1) { //!stop_signal_called)
        //transmit(transmitBuffer, TBUF_SIZE);
        tx_stream->send(transmitBuffer, TBUF_SIZE, md);

    }
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
    for (i = 0; i <100; i++) {
        testbuf[i] = std::complex<float> (1,0);
    }
    
    i = 0;


    uhd::tx_metadata_t md;
    md.start_of_burst = false;
    md.end_of_burst = false;

    std::cout << "Began transmit" << std::endl;
    tx_stream->send(testbuf, 100, md);
    std::cout << "First Buff done" << std::endl;
        
    while (1) {
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
    while(1){ //!stop_signal_called)
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
void Masdr::match_test(){
    //Test match filt stuff.
    float test_val;
    int i, k=0;
    begin_sampling();
    while(1) { //!stop_signal_called)
        begin_sampling();
        rx_stream->recv(testbuf,RBUF_SIZE,md,3.0,false);
        // stop_sampling();
        // std::cout<<"Rec'd"<<std::endl;
        // std::cout<< testbuf[500].real();
        // std::cout<<std::endl;
       
        run_fft(testbuf);
        // std::cout<<"FFT'd"<<std::endl;
        // std::cout<< fft_out[500][0]<<std::endl;
        test_val = match_filt();
        std::cout<<test_val;
        for(i = 0; i < (int)test_val*5; i++)
            std::cout<<'#';
        std::cout<<std::endl;
        // if(!(k%50000))
        //     std::cout<<match_filt()<<std::endl;

        // if(k > 1000000)
        //     k = 0;
         // std::cout<< match_filt()<<std::endl;
    }

    std::cout<<"Match filter test done." <<std::endl<<std::endl;
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
        fft_in[i][0] = cos(2 * PI * freq * i / freq_scale);
        //fft_in[i][0] = 1;
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


        std::cout << "Max Magnitude: "<<max_mag<< " at index: " << max_index<<std::endl;
        std::cout << "Bucket represents " <<max_index * freq_scale /2/ N_FFT <<std::endl;
        
}

/******************************************************************************/
int UHD_SAFE_MAIN(int argc, char *argv[]) {
    std::signal(SIGINT, handle_sigint);
    Masdr masdr;

    if(G_DEBUG){
        if(DEBUG_THRESH) masdr.rx_test();
        if(DEBUG_TX) masdr.tx_test();
        if(DEBUG_MATCH)masdr.match_test();
        if(DEBUG_MAG)masdr.mag_test();
        if(DEBUG_FFT) masdr.fft_test();
        if(DEBUG_TX_DATA) masdr.transmit_data_test();
    }

    else{
        while(1) { //!stop_signal_called)
            // if(G_DEBUG) std::cout<<"Entered While"<<std::endl;
            masdr.update_status();
            // if(G_DEBUG) std::cout<< "While has looped once" <<std::endl;
            masdr.state_transition();
            masdr.repeat_action();

        }
    }

    return EXIT_SUCCESS;
}

