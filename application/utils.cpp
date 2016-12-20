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


