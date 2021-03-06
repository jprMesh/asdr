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

// Standard Libraries
#include <iostream>
#include <stdio.h>
#include <errno.h>
#include <string.h>
// UHD Libraries
#include <uhd/usrp/multi_usrp.hpp>
// Boost Libraries
#include <boost/format.hpp>
#include <boost/thread.hpp>
// GPS libraries
#include <gps.h>
#include <unistd.h>
#include <math.h>
#include <pthread.h> //Using pthread for GPS

// SDR buffer sizes
#define RBUF_SIZE 16384
#define TBUF_SIZE 162   //11/6/16 NARUT: dependent 
                        //on our packet size 5 floats (32*3) + (33*2) start/end
#define SPS 4 //4 samples per symbol.

//GPS constants
#define GPS_BUF_SIZE 60  // Hold the past 6 seconds of samples

// Energy detection constants
#define THRESH_E 0.1 ///11/14/16 MHLI: Picked based on received information.
#define THRESH_MATCH 0 //11/14/16 MHLI: 20,25 would work probably, especially in 

// Standard defines
#define PI      3.14159265359
#define BIT0    0x00000001
#define BIT1    0x00000002
#define BIT2    0x00000004
#define BIT3    0x00000008
#define BIT4    0x00000010
#define BIT5    0x00000020
#define BIT6    0x00000040
#define BIT7    0x00000080
#define BIT8    0x00000100
#define BIT9    0x00000200
#define BIT10   0x00000400
#define BIT11   0x00000800
#define BIT12   0x00001000
#define BIT13   0x00002000
#define BIT14   0x00004000
#define BIT15   0x00008000
#define BIT16   0x00010000
#define BIT17   0x00020000
#define BIT18   0x00040000
#define BIT19   0x00080000
#define BIT20   0x00100000
#define BIT21   0x00200000
#define BIT22   0x00400000
#define BIT23   0x00800000
#define BIT24   0x01000000
#define BIT25   0x02000000
#define BIT26   0x04000000
#define BIT27   0x08000000
#define BIT28   0x10000000
#define BIT29   0x20000000
#define BIT30   0x40000000
#define BIT31   0x80000000

/**
 * Structure for received sample buffer and heading.
 */
typedef struct samp_block {
    float heading; ///< Heading in degrees from north, according to magnetometer
    std::complex<float> samples[RBUF_SIZE]; ///< USRP samples
} Sampblock;

/**
* Structure for GPS Position
*/
typedef struct gps_dat {
    float x;
    float y;
    float v_x;
    float v_y;
    float e_x;
    float e_y;
} GPSData;

/**
 * Linked list node structure of data to be packaged then transmitted.
 */
typedef struct transnode {
    float heading; ///< Heading in degrees from north, according to magnetometer
    float gps[3]; ///< The GPS location for this data
    float data; ///< The data that we want to transmit
    struct transnode* next; ///< Next recorded block, either a pointer or NULL

    /**
     * @brief Delete the next item in the list.
     *
     * This will call recursively until everything after the element it was
     * initially called on is deleted.
     */
    ~transnode() {
        delete next;
    }
} TransNode;

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
 * @brief Test FFT functionality.
 */
void fft_test();

/**
 * type provided by UHD, find documentation at http://files.ettus.com/manual/
 */
typedef boost::function<uhd::sensor_value_t (const std::string&)>
    get_sensor_fn_t;

/**
 * Function provided by UHD.
 * 
 * Documentation at http://files.ettus.com/manual/
 */
bool check_locked_sensor(std::vector<std::string> sensor_names,
                         const char* sensor_name,
                         get_sensor_fn_t get_sensor_fn,
                         double setup_time);

/**
 * Initialize USB GPS
 */
int init_gps();

/**
 * Reads latitude, longitude, and time from GPS and puts it in FIFO
 */
void *poll_gps();

/**
 * Read data from GPS FIFO
 */
void get_gps_data(double *latitude, double *longitude, double *time);

/**
 * Shuts down GPS
 */
void rem_gps();

#endif // __utils_h__
