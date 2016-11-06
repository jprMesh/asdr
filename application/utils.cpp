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
#include "lsm303dlhc_driver.c"
#include <math.h> 
bool stop_signal_called = false; ///< Global for keyboard interrupts
int i2cHandle =0;

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

/******************************************************************************/
void init_mag(){

    int opResult = 0; // for error checking of operations
    
    i2cHandle = open("/dev/i2c-2", O_RDWR);
 
    // Tell the I2C peripheral that the device address isn't a 10-bit
    if(ioctl(i2cHandle, I2C_TENBIT, 0) != 0){
        perror("Error setting address length");
    }
   
    // Tell the I2C peripheral what the address of the magnetometer is
    //and set the magnetometer to the slave
    if(ioctl(i2cHandle, I2C_SLAVE, MAG_I2C_ADDRESS) != 0){
        perror("Error setting device address");
    }
  
    //Set sampling rate
    //if(MEMS_ERROR == SetODR_M(ODR_220Hz_M)){
    //    perror("Error setting ODR_M\n");
    //}
  
    //Enable x and y axis. Disable z axis
    if(MEMS_ERROR == SetAxis(X_ENABLE | Y_ENABLE | Z_DISABLE)){
        perror("Error setting axis\n");
    }
  
    //Set the scale to default
    if(MEMS_ERROR == SetFullScale(FULLSCALE_2)){
        perror("Error setting scale\n");
    }
  
    //Set gain to default
    if(MEMS_ERROR == SetGainMag(GAIN_1100_M)){
        perror("Error setting Gain\n");
    }
 
    //Tell the Magnetometer to sleep
    if(MEMS_ERROR == SetModeMag(SLEEP_MODE)){
        perror("Error setting Mode to sleep\n");
    }
}

/******************************************************************************/
float read_mag(){
    MagAxesRaw_t axes;
    if(MEMS_ERROR == GetMagAxesRaw(&axes) ){
        perror("Error reading Magnetometer data\n");
    }
    float Pi = 3.14159;
    // Calculate the angle of the vector y,x
    float heading = (atan2((float)axes.AXIS_Y,(float)axes.AXIS_X) * 180) / Pi;
  
    // Normalize to 0-360
    if (heading < 0)
    {
      heading = 360 + heading;
    }
    return heading;
    
}
