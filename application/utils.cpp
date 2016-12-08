// File: utils.cpp
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

#include "utils.h"
static bool stop_signal_called=false; ///< Global for keyboard interrupts

int gps_running = 1; // flag to start/stop gps polling 
                     // need to recall poll_gps after setting flag back to 1
double gps_buff[GPS_BUF_SIZE][3]; //GPS data buffer. 
                                 //0 is latitude 
                                 //1 is longitude 
                                 //2 is timestamp                                 
volatile int gps_buf_head = 0; //current gps buffer head
struct gps_data_t gps_data__;    //GPS struct


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
    rem_gps();
    exit(0);
}

/******************************************************************************/
int init_gps(){
    int rc,x;
    gps_running =1;
    if ((rc = gps_open("localhost", "2947", &gps_data__)) == -1) {
        printf("code: %d, reason: %s\n", rc, gps_errstr(rc));
        gps_running = 0;
        return 0;
    }

    //set gps stream to watch JSON
    gps_stream(&gps_data__, WATCH_ENABLE | WATCH_JSON, NULL);

    //initialize gps buffer to 0s
    for(rc=0; rc<GPS_BUF_SIZE; rc++){
        for(x=0; x<3; x++){
            gps_buff[rc][x] = 0;
        }
    }
    return 1;
}


/******************************************************************************/
void *poll_gps(void *unused){
    int rc;
    while (gps_running) {
        // wait for 2 seconds to receive data
        if (gps_waiting (&gps_data__, 2000000)) {
            /* read data */
            if ((rc = gps_read(&gps_data__)) == -1) {
                printf("error occured reading gps data. code: %d, reason: %s\n", 
                       rc, gps_errstr(rc));
            } else {

                // Write data from the GPS receiver
                if ((gps_data__.status == STATUS_FIX) && 
                  (gps_data__.fix.mode == MODE_2D || gps_data__.fix.mode == MODE_3D) &&
                  !isnan(gps_data__.fix.latitude) && !isnan(gps_data__.fix.longitude)) {

                    gps_buff[gps_buf_head][0] = gps_data__.fix.latitude;
                    gps_buff[gps_buf_head][1] = gps_data__.fix.longitude;
                    gps_buff[gps_buf_head][2] = gps_data__.fix.time;
                    // use gps_data__.fix.--- to access
                    // int    mode;        /* Mode of fix */
                    //   #define MODE_NOT_SEEN   0/* mode update not seen yet */
                    //   #define MODE_NO_FIX     1    /* none */
                    //   #define MODE_2D         2    /* good for latitude/longitude */
                    //   #define MODE_3D         3    /* good for altitude/climb too */
                    // double ept;         /* Expected time uncertainty */
                    // double latitude;    /* Latitude in degrees (valid if mode >= 2) */
                    // double epy;         /* Latitude position uncertainty, meters */
                    // double longitude;   /* Longitude in degrees (valid if mode >= 2) */
                    // double epx;         /* Longitude position uncertainty, meters */
                    // double altitude;    /* Altitude in meters (valid if mode == 3) */
                    // double epv;         /* Vertical position uncertainty, meters */
                    // double track;       /* Course made good (relative to true north) */
                    // double epd;         /* Track uncertainty, degrees */
                    // double speed;       /* Speed over ground, meters/sec */
                    // double eps;         /* Speed uncertainty, meters/sec */
                    // double climb;       /* Vertical speed, meters/sec */
                    // double epc;         /* Vertical speed uncertainty */

                    //Loop buffer
                    gps_buf_head = (gps_buf_head + 1) % GPS_BUF_SIZE;

                } else {
                    printf("no GPS data available\n");
                }
            }
        }
    }
    pthread_exit(NULL);
}

/******************************************************************************/

void get_gps_data(double *latitude, double *longitude, double *time){
    int pos = (gps_buf_head - 1) % GPS_BUF_SIZE;
    *latitude  = gps_buff[pos][0];
    *longitude = gps_buff[pos][1];
    *time      = gps_buff[pos][2];
    return;
}


/******************************************************************************/

void rem_gps(){
    gps_running = 0;
    gps_stream(&gps_data__, WATCH_DISABLE, NULL);
    gps_close (&gps_data__);
}


/******************************************************************************/
void init_mag(){

    int opResult = 0; // for error checking of operations
    
    //i2cHandle = open("/dev/i2c-2", O_RDWR);
 
    // Tell the I2C peripheral that the device address isn't a 10-bit
    //if(ioctl(i2cHandle, I2C_TENBIT, 0) != 0){
    //    perror("Error setting address length");
    //}
   
    // Tell the I2C peripheral what the address of the magnetometer is
    //and set the magnetometer to the slave
    //if(ioctl(i2cHandle, I2C_SLAVE, MAG_I2C_ADDRESS) != 0){
     //   perror("Error setting device address");
    //}
    //int temp = lsm303dlhc_mag_init();
    
    

  
}


/******************************************************************************/
float read_mag(){
//    MagAxesRaw_t axes;
//    axes.AXIS_X=0;
//    axes.AXIS_Y=0;
//    axes.AXIS_Z=0;
    //if(MEMS_ERROR == GetMagAxesRaw(&axes) ){
    //    perror("Error reading Magnetometer data\n");
    //}
    float Pi = 3.14159;
    float heading =0;
    // Calculate the angle of the vector y,x
//    float heading = (atan2((float)axes.AXIS_Y,(float)axes.AXIS_X) * 180) / Pi;
  
    // Normalize to 0-360
//    if (heading < 0)
//    {
//      heading = 360 + heading;
    //}
    return heading;
    
}
