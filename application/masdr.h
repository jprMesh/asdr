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

//Other included libraries
#include <uhd/types/tune_request.hpp>
#include <uhd/utils/thread_priority.hpp>
#include <uhd/utils/safe_main.hpp>
#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/exception.hpp>
#include <boost/format.hpp>
#include <boost/thread.hpp> /// 10/3/16 MHLI: Not sure where this and pthread
                            // diverge, but if one of you can use only
                            // one or only the other, go ahead.
#include <cmath>
#include <iostream>
#include <csignal>
#include <complex>
#include <pthread.h>

#define RBUF_SIZE 500

/// Status of the software on the SBC.
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
    bool is_stationary; ///< Current stationarity.
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
 * idk what this is
 */
typedef boost::function<uhd::sensor_value_t (const std::string&)>
    get_sensor_fn_t;

bool stop_signal_called = false; ///< Global for keyboard interrupts

/**
 * Check Sensor declaration
 * 
 * ///10/7/16 MHLI: Someone will probably want to change this delcaration.
 */
bool check_locked_sensor(std::vector<std::string> sensor_names,
                         const char* sensor_name,
                         get_sensor_fn_t get_sensor_fn,
                         double setup_time);

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
     * @brief Do actions based on the current status.
     * 
     * If the system is not currently idle, do not interrupt the current
     * process. (This may need to change, but I don't see it being necessary)
     */
    void do_action();

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
    
private:
    /**
     * @brief Initialize the UHD interface to the SDR
     * 
     * Initialize all components necessary for the interface to the USRP SDR
     * using the UHD library.
     */
    void initialize_uhd();

    /**
     * @brief Reconfigure the SDR interface for transmitting or receiving.
     * 
     * @param txrx Whether to configure for transmitting or receiving.
     */
    void reconfig_uhd(int txrx);

    /**
     * @brief Initialize any peripherals being used
     * 
     * This will be the GPS receiver and maybe an external memory device.
     */
    void initialize_peripherals();

    /**
     * @brief Fork off a new process to start the SDR taking samples.
     */
    void begin_sampling();

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
    void transmit(const void *msg, int len);

    /**
     * @brief Transmit data to ground station
     * 
     * Transmit sampling location and directions for signals to ground station.
     */
    void transmit_data();

    /**
     * @brief Gracefully stop the SDR.
     * 
     * Stop all SDR processing and close any connections to the USRP SDR.
     */
    void shutdown_uhd();

    uhd::rx_streamer::sptr rx_stream;///< The UHD rx streamer
    uhd::rx_streamer::sptr tx_stream;///< The UHD tx streamer
    std::complex<float> rbuf[RBUF_SIZE];///< Buffer that signals get received into.
    uhd::rx_metadata_t md;///< UHD Metadata
    PhyStatus phy_status; ///< Physical status of the platform.
    SoftStatus soft_status; ///< The current stage of the software on the SBC. 
    bool process_done; ///< Set when data processing has completed.
    bool transmit_done; ///< Set when data transmission has completed.
};
