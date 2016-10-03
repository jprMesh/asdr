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


/// Status of the software on the SBC.
typedef enum {
    SAMPLE,
    PROCESS,
    TRANSMIT,
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
     * @brief Command the SDR to begin taking samples.
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

    PhyStatus phy_status; ///< Physical status of the platform.
    SoftStatus soft_status; ///< The current stage of the software on the SBC.
    bool was_stationary; ///< Previous stationary state.

private:
    /**
     * @brief Update platform status
     * 
     * Updates the location, heading, and stationarity.
     */
    void update_status();

    /**
     * @brief Initialize the UHD interface to the SDR
     * 
     * Initialize all components necessary for the interface to the USRP SDR
     * using the UHD library.
     */
    void initialize_uhd();

    /**
     * @brief Initialize any peripherals being used
     * 
     * This will be the GPS receiver and maybe an external memory device.
     */
    void initialize_peripherals();

    /**
     * @brief Gracefully stop the SDR.
     * 
     * Stop all SDR processing and close any connections to the USRP SDR.
     */
    void shutdown_uhd();
};
