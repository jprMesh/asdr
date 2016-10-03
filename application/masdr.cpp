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

/******************************************************************************/
Masdr::Masdr() {
    initialize_peripherals();
    initialize_uhd();
    update_status();
}

/******************************************************************************/
Masdr::~Masdr() {
    shutdown_uhd();
}

/******************************************************************************/
void Masdr::update_status() {
    was_stationary = phy_status.is_stationary;
}

/******************************************************************************/
void Masdr::initialize_uhd() {

}

/******************************************************************************/
void Masdr::initialize_peripherals() {

}

/******************************************************************************/
void Masdr::begin_sampling() {

}

/******************************************************************************/
void Masdr::stop_sampling() {

}

/******************************************************************************/
void Masdr::begin_processing() {

}

/******************************************************************************/
int main(int argc, char const *argv[]) {

    Masdr masdr;

    while(1) {
        if (masdr.phy_status.is_stationary && !masdr.was_stationary) {
            masdr.begin_sampling();
            masdr.soft_status = SAMPLE;
        } else if (!masdr.phy_status.is_stationary && masdr.was_stationary) {
            masdr.stop_sampling();
            masdr.begin_processing();
            masdr.soft_status = PROCESS;
        }
    }
    return 0;
}
