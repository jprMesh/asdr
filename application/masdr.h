// File: masdr.h
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

#ifndef __masdr_h__
#define __masdr_h__

// Standard Libraries
#include <iostream>
#include <csignal>
#include <complex>
#include <cmath>
// UHD Libraries
#include <uhd/types/tune_request.hpp>
#include <uhd/utils/thread_priority.hpp>
#include <uhd/utils/safe_main.hpp>
#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/exception.hpp>
// FFT Library
#include <fftw3.h>
// Other includes
#include "utils.h"

#define G_DEBUG 1
#if G_DEBUG
    #define DEBUG_THRESH 0
    #define DEBUG_MATCH 1
    #define DEBUG_TX 0
    #define DEBUG_MAG 0
    #define DEBUG_FFT 0
    #define DEBUG_TX_DATA 0
    #define SCALE_ACC 0
#else
    #define DEBUG_THRESH 0
    #define DEBUG_MATCH 0
    #define DEBUG_TX 0
    #define DEBUG_MAG 0
    #define DEBUG_FFT 0
    #define DEBUG_TX_DATA 0
    #define SCALE_ACC 0
#endif

    ///11/16/16 MHLI: At a sample rate of 45MHz, OFDM transmitted 360 samps.
#define N_FFT 1024  ///11/16/16 MHLI: 16384 is a bit overkill, 
                    ///considering the ofdm carrier is binned w/ 64 bin fft.
                    ///Going to use 1024 for now.

#define N_RRC 1024 /// 12/4/16 MHLI: Currently not the right number

#define RBUF_BLOCKS 16 /// Num blocks in rolling buffer. MUST BE POWER OF 2.
#define WRAP_RBUF(x) (x & (RBUF_BLOCKS - 1)) /// Wrap buffer around

/**
 * @brief MASDR Application Class
 * 
 * This is the class for the MASDR Application. It contains all of the
 * funcitonality that the platform will have. The platform consists of an SBC
 * connected to a USRP SDR with an antenna.
 */
class Masdr {
public:
    /**
     * @brief Initialize a Masdr object.
     */
    Masdr();

    /**
     * @brief Stop all functionality and destroy Masdr object.
     */
    ~Masdr();

    /**
     * @brief Update platform status
     * 
     * Updates the location, heading, and stationarity.
     */
    void update_status();

    /**
     * @brief Handle software state transitions based on the current status.
     * 
     * If the system is not currently idle, do not interrupt the current
     * process. (This may need to change, but I don't see it being necessary)
     */
    void state_transition();

    /**
     * @brief Do any repetitive action associated with the current state.
     * 
     * Gets called every loop and perfoms an action based on the current value
     * of soft_status. In sample mode, this means starting a new recv buffer.
     */
    void repeat_action();

    /**
     * @brief Test the receive functionality.
     * 
     * Test the functionality of the rx calling within the program.
     * Probably remove in a bit.
     */
    void rx_test();

    /** 
     * @brief Test the transmit functionality.
     * 
     * Test the functionality of the tx calling within the program.
     * Probably remove in a bit.
     */
    void tx_test();
    
    /** 
     * @brief Test the magnetometer functionality.
     * 
     * Test the functionality of the mag calling within the program.
     * Probably remove in a bit.
     */
    void mag_test();
    
    /**
     * @brief Test the match filter amount. REMOVE LATER
     */
    void match_test();

    /**
     * @brief Test the transmit data. REMOVE LATER
     */
    void transmit_data_test();

    /**
     * @brief Figuring out how fftw works, REMOVE LATER
     */
    void fft_test();

private:
    /**
     * @brief Initialize any peripherals being used
     * 
     * This will be the GPS receiver and maybe an external memory device.
     */
    void initialize_peripherals();

    /**
     * @brief Initialize the UHD interface to the SDR
     * 
     * Initialize all components necessary for the interface to the USRP SDR
     * using the UHD library.
     */
    void initialize_uhd();

    /**
     * @brief Create a new thread to do the sampling.
     */
    void sample();
    
    /**
     * @brief Detect if there's any energy detected on the bandwidth being measured.
     * 
     * @param sig_in ///THIS IS NEEDED
     * @param size ///THIS IS NEEDED
     * 
     * @return ///THIS IS NEEDED
     */
    float energy_detection(std::complex<float> *sig_in, int size);
    
    /**
     * @brief Transfer buffer to fft_in, and run FFT.
     * 
     * @param ///THIS IS NEEDED
     */
    void run_fft(std::complex<float> *);
    
    /**
     * @brief Look for OFDM header.
     */
    float match_filt();

    /**
     * @brief Locate signal. Returns x,y coordinates based on GPS.
     */
    float* rss();

    /**
     * @brief Command the SDR to stop taking samples.
     */
    void stop_sampling();

    /**
     * @brief Start processing the collected samples.
     */
    void begin_processing();

    /**
     * @brief General transmission method.
     * 
     * @param msg Pointer to packet to send.
     * @param len Size of packet to be sent.
     */
    void transmit(std::complex<float> *msg, int len);

    /**
     * @brief Transmit data to ground station
     * 
     * Transmit sampling location and directions for signals to ground station.
     */
    void transmit_data();

    uhd::usrp::multi_usrp::sptr usrp;
    std::complex<float> testbuf[RBUF_SIZE]; ///< Testing if structure is too big.
    float rrcBuf[N_RRC]; ///< 4 samples per symbol.
    uhd::rx_streamer::sptr rx_stream; ///< The UHD rx streamer
    uhd::tx_streamer::sptr tx_stream; ///< The UHD tx streamer
    uhd::rx_metadata_t md; ///< UHD Metadata
    PhyStatus phy_status; ///< Physical status of the platform
    SoftStatus soft_status; ///< The current stage of the software on the SBC
    samp_block recv_buf[RBUF_BLOCKS]; ///< Rolling buffer of rcvd sample blocks
    int rb_index; ///< Index of next insertion into recv_buff.
    TransNode* trans_head; ///< Head node in linked list buffer for transmitting
    TransNode* curr_trans_buf; ///< Last item in linked list.
    fftw_plan fft_p; ///< FFTW Plan
    fftw_complex ofdm_head[N_FFT]; //< Expected OFDM Header. 
    fftw_complex *fft_in, *fft_out; ///< Buffers for FFT.
    bool process_done; ///< Set when data processing has completed
    bool transmit_done; ///< Set when data transmission has completed
    bool do_sample; ///< Disable to gracefully shutdown sampling.
};

#endif // __masdr_h__
