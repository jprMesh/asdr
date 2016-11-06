// File: utils.h
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

#ifndef __utils_h__
#define __utils_h__

#include <iostream>
// UHD libraries
#include <uhd/usrp/multi_usrp.hpp>
// Boost libraries
#include <boost/format.hpp>
#include <boost/thread.hpp>
//FFT Library
#include <fftw3.h>
// Buffer sizes
#define RBUF_SIZE 16384
#define FFT_N 16384
#define THRESH_E 0.002 ///11/3/16 MHLI: currently arbitrarily picked

/**
 * Linked list node structure for received sample buffer and heading.
 */
typedef struct recvnode {
    float heading; ///< Heading in degrees from north, according to magnetometer
    float recv_buf[RBUF_SIZE]; ///< USRP samples from current direction
    struct recvnode* next; ///< Next recorded block, either a pointer or NULL

    /**
     * @brief Delete the next item in the list.
     *
     * This will call recursively until everything after the element it was
     * initially called on is deleted.
     */
    ~recvnode() {
        delete next;
    }
} RecvNode;

/**
 * Status of the software on the SBC.
 */
typedef enum {
    SAMPLE,
    PROCESS,
    TRANSMIT,
    IDLE,
} SoftStatus;

/**
 * Physical status of the platform.
 * 
 * Includes location, heading, and stationarity.
 */
typedef struct {
    double location[3]; ///< Location as an array of lat, long, and height.
    double heading; ///< Heading in degrees from north.
    bool is_stat_and_rot; ///< Currently stationary and rotating.
} PhyStatus;

/**
 * Structure for header message before data transmission.
 * 
 * Includes sampling location and number of hits, indicating how many TxHit
 * messages will be following.
 */
typedef struct {
    unsigned int tx_id; ///< Transmission ID number to link with data packets.
    double location[3]; ///< Location of this sampling session.
    int num_hits; ///< Number of signals detected. (Need to discuss and clarify)
} TxHeader;

/**
 * Structure for transmission of data concerning a single detected signal.
 */
typedef struct {
    unsigned int tx_id; ///< Transmission ID number to link with header packet.
    double heading; ///< Heading in degrees from North of detected signal.
    double strength; ///< Strength of detected signal.
} TxHit;

/**
 * @brief Handle a SIGINT nicely.
 */
void handle_sigint(int);

/**
 * type provided by UHD, find documentation at http://files.ettus.com/manual/
 */
typedef boost::function<uhd::sensor_value_t (const std::string&)>
    get_sensor_fn_t;

/**
 * Function provided by UHD, find documentation at http://files.ettus.com/manual/
 */
bool check_locked_sensor(std::vector<std::string> sensor_names,
                         const char* sensor_name,
                         get_sensor_fn_t get_sensor_fn,
                         double setup_time);

#endif // __utils_h__
