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
#include "kalman_filt.h"
#include <iostream>
#include <fstream>
/******************************************************************************/
Masdr::Masdr() {
    // Initialize software status
    process_done = false;
    transmit_done = false;
    soft_status = IDLE;

    // Initialize received sample buffer
    rb_index = 0;

    // Initialize FFTW
    fft_in = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * N_FFT);
    fft_out = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * N_FFT);
    fft_p = fftw_plan_dft_1d(N_FFT, fft_in, fft_out,
                             FFTW_FORWARD, FFTW_MEASURE);

    //Intialize OFDM Match Filter head
    //OFDM HEAD DETAILS:
        //Ignore first and last 174 buckets.
        //Each OFDM bucket is 16 fft buckets.
        //They are located at 254-270, 480-415, 640-615,864-880
    int i;
    for (i = 0; i < N_FFT; ++i) {
        int ratio = N_FFT/64; // Number of fft bins in an OFDM bin.

        // Account for the fact that only middle 52 OFDM bins are used.
        if (i < 6 * ratio || i > 58 * ratio) {
            ofdm_head[i][0] = 0;
            ofdm_head[i][1] = 0;
        }
        else if(i >= (6+6) * ratio && i < (6+7) * ratio) {
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
        else if(i >= (40+14) * ratio && i < (40 + 15) * ratio) {
            ofdm_head[i][0] = 1;
            ofdm_head[i][1] = 1;
        }
        else {
            ofdm_head[i][0] = 0;
            ofdm_head[i][1] = 0;
        }
    }

    float time;
    // Maybe internal freq should be 905e6,idk
    int freq = 1e6/SPS; //Currently 1/2 symbol rate
    float excess=0.2;
    float Ts = 1/freq;
    // float omega=2*PI*freq_tx; //2pif

    for (i = 0; i < N_RRC;i++) {
        if (i == N_RRC/2)
            rrcBuf[i] = 1;
        else{
            time = (i - N_RRC/2)*Ts;
            rrcBuf[i] = (sin(PI*time/Ts*(1-excess))+4*excess*time/Ts*cos(PI*time/Ts*(1+excess))) 
                        /(PI*time/Ts*(1-(4*excess*time/Ts)*(4*excess*time/Ts)));

        }
    }

    initialize_peripherals();
    initialize_uhd();
    update_status();
    do_sample = true;

    boost::thread* s_thr = new boost::thread(boost::bind(&Masdr::sample, this));
}

/******************************************************************************/
Masdr::~Masdr() {
    do_sample = false;
    fftw_destroy_plan(fft_p);
    fftw_free(fft_in);
    fftw_free(fft_out);
    delete trans_head;
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
    int tx_rate = 1e6; //4 samples per symbol at 700kHz
    int master_rate = 42e6;
    float freq_rx = 2.462e9; //Set rx frequency to 2.4 GHz //Set to 2.412 for channel 1 

    float freq_tx = 905e6; //set tx frequency
    int gain = 50;// Default: 8dB
    std::string rx_ant = "RX2"; //ant can be "TX/RX" or "RX2"
    std::string tx_ant = "TX/RX"; //ant can be "TX/RX" or "RX2"
    std::string wirefmt = "sc16"; //or sc8
    int setup_time = 1.0; //sec setup

    int rx_bw = 20e6; //11/16/16 MHLI: Should this be 10e6 and will it do half above half below?
    int tx_bw = 300e3;

    //Create USRP object
    usrp = uhd::usrp::multi_usrp::make((std::string)"");
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
/***************************STATE TRANSITIONS**********************************/
/******************************************************************************/
void Masdr::update_status() {
    phy_status.heading = 0;
    phy_status.is_stat_and_rot = false;
    phy_status.location[0] = 0;
    phy_status.location[1] = 0;
    phy_status.location[2] = 0;
}

/******************************************************************************/
void Masdr::state_transition() {
    if (soft_status == IDLE) {
        soft_status = PROCESS;
        begin_processing();

    } else if (soft_status == PROCESS && process_done) {
        soft_status = TRANSMIT;
        process_done = false;
        transmit_data();

    } else if (soft_status == TRANSMIT && transmit_done) {
        soft_status = IDLE;
        transmit_done = false;
    }
}

/******************************************************************************/
void Masdr::repeat_action() {
    if (soft_status == SAMPLE) {
        ;
        
    } else if (soft_status == PROCESS) {
        ;

    } else if (soft_status == TRANSMIT) {
        ;

    } else if (soft_status == IDLE) {
        ;
    }
}

/******************************************************************************/
/************************************SAMPLE************************************/
/******************************************************************************/

void Masdr::sample() {
    // Create new sampling stream
    uhd::stream_cmd_t start_strm_cmd(
        uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS);
    start_strm_cmd.num_samps = size_t(0);
    start_strm_cmd.stream_now = true;
    start_strm_cmd.time_spec = uhd::time_spec_t(); // Holds the time.
    rx_stream->issue_stream_cmd(start_strm_cmd);   // Initialize the stream
    while (do_sample) {
        rb_index = WRAP_RBUF(rb_index + 1);
        recv_buf[rb_index].heading = phy_status.heading;
        rx_stream->recv(recv_buf[rb_index].samples,
                        RBUF_SIZE, md, 3.0, false);
        boost::this_thread::interruption_point();
    }
    // Issue command to close stream
    uhd::stream_cmd_t stop_strm_cmd(
        uhd::stream_cmd_t::STREAM_MODE_STOP_CONTINUOUS);
    rx_stream->issue_stream_cmd(stop_strm_cmd);
}

/******************************************************************************/
/********************************PROCESSING************************************/
/******************************************************************************/
void Masdr::begin_processing() {
    int i;
    samp_block proc_buf[RBUF_BLOCKS/2];
    float energy = 0;

    for (i = 0; i < RBUF_BLOCKS/2; ++i) {
        proc_buf[i] = recv_buf[WRAP_RBUF(rb_index - RBUF_BLOCKS/2 + i)];
    }
    for (i = 0; i < RBUF_BLOCKS/2; ++i) {
        energy += energy_detection(proc_buf[i].samples, RBUF_SIZE);
    }
    if(energy > THRESH_E) {
        for (i = 0; i < RBUF_BLOCKS/2; ++i) {
            run_fft(proc_buf[rb_index].samples);
            float has_wifi =  match_filt();
            if(has_wifi != -1)
                usrp->get_rx_sensor("rssi",0).to_real(); ///Not stored to anything rn
        }
    }
}

/******************************************************************************/
float Masdr::energy_detection(std::complex<float> *sig_in, int size) {
    int i;
    float acc = 0;
    float max = 0;
    float mag;

    for (i = 0; i < size; i++) {
        mag = sqrt(sig_in[i].real() * sig_in[i].real()
                   + sig_in[i].imag() * sig_in[i].imag());
        acc += mag;
        if (mag > max)
            max = mag;
    }

    if(DEBUG_THRESH) {
        std::cout << max;
        for (i = 0; i < (int)(mag*1000); i++){
            std::cout << "#";
        }
        std::cout << std::endl;
    }

    return max;
}

/******************************************************************************/
void Masdr::run_fft(std::complex<float> *buff_in) {
    int i;
    for(i = 0; i < N_FFT; ++i){
        fft_in[i][0] = buff_in[i].real();
        fft_in[i][1] = buff_in[i].imag();
    }
    fftw_execute(fft_p);
}

/******************************************************************************/
float Masdr::match_filt() {
    int i;
    int j;
    float match_val[2] = {0,0};
    float match_mag;
    float re;
    float im;

    for (i = 0; i < N_FFT; i++) {
        re = fft_out[i][0] * ofdm_head[i][0] - fft_out[i][1] * ofdm_head[i][1];
        im = fft_out[i][0] * ofdm_head[i][1] + fft_out[i][1] * ofdm_head[i][0];
        match_val[0] += re;
        match_val[1] += im;
    }
    match_mag = sqrt(match_val[0] * match_val[0]
                     + match_val[1] * match_val[1]);
    if (match_mag > THRESH_MATCH)
        return match_mag;
    else
        return 0;
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

    std::ofstream ofs;
    ofs.open ("/home/mqp/Results.bin", std::ofstream::out | std::ofstream::app);

    ofs << transmitBuffer << std::endl;


    ofs.close();

    // uhd::tx_metadata_t md;
    // md.start_of_burst = false;
    // md.end_of_burst = false;

    while(1) {
        // tx_stream->send(transmitBuffer,TBUF_SIZE, md);

    }
    std::cout << "Done with transmit" << std::endl;
}
}

/******************************************************************************/
void Masdr::transmit(std::complex<float> *msg, int len) {
    uhd::tx_metadata_t md;
    md.start_of_burst = false;
    md.end_of_burst = false;
    tx_stream->send(msg, len , md);
}

/******************************************************************************/
/************************************TESTS*************************************/
/******************************************************************************/
void Masdr::rx_test() {
    int i = 0;
    int j = 0;
    int numLoops; //Counter, to help
    float accum;
    float max_inBuf = 0;
    float max_periodic = 0;
    float max_total = 0;
    float mag_squared;
    double rssi;
    int TX_Power = 0; //Transmit power of a wireless AP at 2.4 GHz
    double dist_avg;
    double calc_avg[1024];

    std::complex<float> testbuf[RBUF_SIZE];

    std::cout << "Entered rx_test" << std::endl;
    sample();
    std::cout << "Began sampling" << std::endl;
    rx_stream->recv(testbuf, RBUF_SIZE, md, 3.0, false);
    std::cout << "First Buff done" << std::endl;

    if(DEBUG_THRESH)
        while (1){//(i < 5000) {
        rx_stream->recv(testbuf, RBUF_SIZE, md, 3.0, false);
        rssi = usrp->get_rx_sensor("rssi",0).to_real();
        if(rssi > -78){
            calc_avg[i++] = pow(10,((rssi)/-20));         
        }

        if (i > 1023) {
            for (j = 0; j < 1024; j++) {
                dist_avg += calc_avg[j];
            }
            std::cout << dist_avg/1024 << std::endl;
            dist_avg = 0;   
            i = 0;
        }
        
        //std::cout << energy_detection(testbuf, RBUF_SIZE) << std::endl;
    }
    do_sample = false;
    std::cout << "Stopped sampling" << std::endl;
    std::cout << "RX test done." << std::endl << std::endl;
}
    
/******************************************************************************/
void Masdr::tx_test() {
    int i; //Counter, to help test
    std::complex<float> testbuf[100];

    std::cout << "Entered tx_test" << std::endl;
    //Initialize test buffer.
    for (i = 0; i < 100; ++i) {
        testbuf[i] = std::complex<float> (1,0);
    }

    uhd::tx_metadata_t md;
    md.start_of_burst = false;
    md.end_of_burst = false;

    std::cout << "Began transmit" << std::endl;
    tx_stream->send(testbuf, 100, md);
    std::cout << "First Buff done" << std::endl;

    i = 0;
    while (1) {
        tx_stream->send(testbuf, 100, md);
        ++i;
    }

    std::cout << "Stopped transmit" << std::endl;
    std::cout << "Tx test done." <<std::endl<<std::endl;
}

/******************************************************************************/
void Masdr::match_test() {
    //Test match filt stuff.
    float test_val;
    int i;

    while(1) {
        for (int j = 0; j < RBUF_BLOCKS/2; ++j) {
            run_fft(recv_buf[WRAP_RBUF(rb_index - RBUF_BLOCKS/2 + j)].samples);

            test_val = match_filt();
            //std::cout << test_val;
            for (i = 0; i < (int)test_val*5; ++i)
                std::cout << '#';
            std::cout << std::endl;
        }
    }

    std::cout << "Match filter test done." << std::endl << std::endl;
}

/******************************************************************************/
void Masdr::transmit_data_test() {
    int i;
    std::cout << "In tx data test" << std::endl;
    transmit_data();
}

/******************************************************************************/
int UHD_SAFE_MAIN(int argc, char *argv[]) {
    std::signal(SIGINT, handle_sigint);
    Masdr masdr;

    if (G_DEBUG) {
        if (DEBUG_THRESH) masdr.rx_test();
        if (DEBUG_TX) masdr.tx_test();
        if (DEBUG_MATCH) masdr.match_test();
        if (DEBUG_TX_DATA) masdr.transmit_data_test();
    }

    else {
        while(1) { //!stop_signal_called)
            masdr.update_status();
            masdr.state_transition();
            masdr.repeat_action();
        }
    }

    return EXIT_SUCCESS;
}
