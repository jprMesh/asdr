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

bool stop_signal_called = false; ///< Global for keyboard interrupts

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
    exit(0);
}
