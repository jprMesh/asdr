//
// Copyright 2010-2011,2014 Ettus Research LLC
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//

#include <uhd/types/tune_request.hpp>
#include <uhd/utils/thread_priority.hpp>
#include <uhd/utils/safe_main.hpp>
#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/exception.hpp>
#include <boost/program_options.hpp>
#include <boost/format.hpp>
#include <boost/thread.hpp>
#include <iostream>
#include <fstream>
#include <csignal>
#include <complex>
#include <fftw3.h>

#define N_FFT 1024
namespace po = boost::program_options;

fftw_complex *fft_in, *fft_out; ///< Buffers for FFT.
fftw_plan fft_p;
fftw_complex ofdm_head[N_FFT]; //< Expected OFDM Header.    
float match_val[2] = {0,0}, match_mag, re, im;

static bool stop_signal_called = false;
void sig_int_handler(int){stop_signal_called = true;}

template<typename samp_type> void recv_to_file(
    uhd::usrp::multi_usrp::sptr usrp,
    const std::string &cpu_format,
    const std::string &wire_format,
    const std::string &file,
    size_t samps_per_buff,
    unsigned long long num_requested_samples,
    double time_requested = 0.0,
    bool stats = false,
    bool null = false,
    bool continue_on_bad_packet = false
){
    int i;
    unsigned long long num_total_samps = 0;
    //create a receive streamer
    uhd::stream_args_t stream_args(cpu_format,wire_format);
    uhd::rx_streamer::sptr rx_stream = usrp->get_rx_stream(stream_args);

    uhd::rx_metadata_t md;
    samp_type buff[samps_per_buff];
    std::ofstream outfile;
    if (not null)
        outfile.open(file.c_str(), std::ofstream::binary);
    bool overflow_message = true;

    //setup streaming
    uhd::stream_cmd_t stream_cmd((num_requested_samples == 0)?
        uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS:
        uhd::stream_cmd_t::STREAM_MODE_NUM_SAMPS_AND_DONE
    );
    stream_cmd.num_samps = size_t(num_requested_samples);
    stream_cmd.stream_now = true;
    stream_cmd.time_spec = uhd::time_spec_t();
    rx_stream->issue_stream_cmd(stream_cmd);

    boost::system_time start = boost::get_system_time();
    unsigned long long ticks_requested = (long)(time_requested * (double)boost::posix_time::time_duration::ticks_per_second());
    boost::posix_time::time_duration ticks_diff;
    boost::system_time last_update = start;
    unsigned long long last_update_samps = 0;

    typedef std::map<size_t,size_t> SizeMap;
    SizeMap mapSizes;

    while(not stop_signal_called and (num_requested_samples != num_total_samps or num_requested_samples == 0)) {
        boost::system_time now = boost::get_system_time();

        size_t num_rx_samps = rx_stream->recv(buff, samps_per_buff, md, 3.0, 0);

        if (md.error_code == uhd::rx_metadata_t::ERROR_CODE_TIMEOUT) {
            std::cout << boost::format("Timeout while streaming") << std::endl;
            break;
        }
        if (md.error_code == uhd::rx_metadata_t::ERROR_CODE_OVERFLOW){
            if (overflow_message) {
                overflow_message = false;
                std::cerr << boost::format(
                    "Got an overflow indication. Please consider the following:\n"
                    "  Your write medium must sustain a rate of %fMB/s.\n"
                    "  Dropped samples will not be written to the file.\n"
                    "  Please modify this example for your purposes.\n"
                    "  This message will not appear again.\n"
                ) % (usrp->get_rx_rate()*sizeof(samp_type)/1e6);
            }
            continue;
        }
        if (md.error_code != uhd::rx_metadata_t::ERROR_CODE_NONE){
            std::string error = str(boost::format("Receiver error: %s") % md.strerror());
            if (continue_on_bad_packet){
                std::cerr << error << std::endl;
                continue;
            }
            else
                throw std::runtime_error(error);
        }

        num_total_samps += num_rx_samps;
        
        ///11/18/16 MHLI:  
        //Copy buff.front() into an FFT
        //Execute FFT
        fftw_execute(fft_p);
        //Match filter.
        for(i = 0; i < N_FFT; i++) {
            re = fft_out[i][0] * ofdm_head[i][0] - fft_out[i][1] * ofdm_head[i][1];
            im = fft_out[i][0] * ofdm_head[i][1] + fft_out[i][1] * ofdm_head[i][0];
            match_val[0] += re;
            match_val[1] += im;
        }
        match_mag = sqrt(match_val[0]*match_val[0]+match_val[1]*match_val[1]);
        //Log Match filter result.
        if (outfile.is_open())
            outfile.write((const char*)buff, num_rx_samps*sizeof(samp_type));

        ticks_diff = now - start;
        if (ticks_requested > 0){
            if ((unsigned long long)ticks_diff.ticks() > ticks_requested)
                break;
        }
    }

    stream_cmd.stream_mode = uhd::stream_cmd_t::STREAM_MODE_STOP_CONTINUOUS;
    rx_stream->issue_stream_cmd(stream_cmd);

    if (outfile.is_open())
        outfile.close();

    if (stats) {
        std::cout << std::endl;

        double t = (double)ticks_diff.ticks() / (double)boost::posix_time::time_duration::ticks_per_second();
        std::cout << boost::format("Received %d samples in %f seconds") % num_total_samps % t << std::endl;
        double r = (double)num_total_samps / t;
        std::cout << boost::format("%f Msps") % (r/1e6) << std::endl;

    }
}

typedef boost::function<uhd::sensor_value_t (const std::string&)> get_sensor_fn_t;

bool check_locked_sensor(std::vector<std::string> sensor_names, const char* sensor_name, get_sensor_fn_t get_sensor_fn, double setup_time){
    if (std::find(sensor_names.begin(), sensor_names.end(), sensor_name) == sensor_names.end())
        return false;

    boost::system_time start = boost::get_system_time();
    boost::system_time first_lock_time;

    std::cout << boost::format("Waiting for \"%s\": ") % sensor_name;
    std::cout.flush();

    while (true) {
        if ((not first_lock_time.is_not_a_date_time()) and
                (boost::get_system_time() > (first_lock_time + boost::posix_time::seconds(setup_time))))
        {
            std::cout << " locked." << std::endl;
            break;
        }
        if (get_sensor_fn(sensor_name).to_bool()){
            if (first_lock_time.is_not_a_date_time())
                first_lock_time = boost::get_system_time();
            std::cout << "+";
            std::cout.flush();
        }
        else {
            first_lock_time = boost::system_time();	//reset to 'not a date time'

            if (boost::get_system_time() > (start + boost::posix_time::seconds(setup_time))){
                std::cout << std::endl;
                throw std::runtime_error(str(boost::format("timed out waiting for consecutive locks on sensor \"%s\"") % sensor_name));
            }
            std::cout << "_";
            std::cout.flush();
        }
        boost::this_thread::sleep(boost::posix_time::milliseconds(100));
    }
    std::cout << std::endl;
    return true;
}

int UHD_SAFE_MAIN(int argc, char *argv[]){
    uhd::set_thread_priority_safe();



    //Initialize OFDM_head
   int i;
    for(i=0;i<N_FFT;i++){
        int ratio = N_FFT/64; //Number of fft bins in an OFDM bin.

        //Account for the fact that only middle 52 OFDM bins are used.
        if (i < 6 * ratio || i > 58 * ratio){
            ofdm_head[i][0] = 0;
            ofdm_head[i][1] = 0;
        }
        else if(i >= (6+6) * ratio && i < (6+7) * ratio){
            ofdm_head[i][0] = 1;
            ofdm_head[i][1] = 1;
        }
        else if(i >= (12+14) * ratio && i < (12+15) * ratio) {
            ofdm_head[i][0] = 1;
            ofdm_head[i][1] = 1;
        }
        else if(i >= (26+14) * ratio && i < (26+15) * ratio) {
            ofdm_head[i][0] = 1;
            ofdm_head[i][1] = 1;
        }
        else if(i >= (40+14) * ratio && i < (40 + 15) * ratio){
            ofdm_head[i][0] = 1;
            ofdm_head[i][1] = 1;
        }
        else {
            ofdm_head[i][0] = 0;
            ofdm_head[i][1] = 0;
        }
    }
    //Initialize FFT.
        fft_in = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * N_FFT); 
    fft_out = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * N_FFT);
    fft_p = fftw_plan_dft_1d(N_FFT, fft_in, fft_out, FFTW_FORWARD, FFTW_MEASURE);   

    //Initialize USRP
    std::string file = "usrp_samples.dat",  \
                type = "short",             \
                ant = "RX2",                \
                ref = "internal",           \
                wirefmt = "sc16",           \
                args;
    
    short       total_num_samps = 0,        \
                total_time = 0,             \
                spb=1024,                   \
                setup_time=1,               \
                gain=40;
    
    double      rate = 1e6,                 \
                freq=2.4e9;
    
    bool    stats = 0,                      \
            null = 0,                       \
            continue_on_bad_packet = 0;


    //variables to be set by po
    
    //create a usrp device
    uhd::usrp::multi_usrp::sptr usrp = uhd::usrp::multi_usrp::make(args);

    //Lock mboard clocks
    usrp->set_clock_source(ref);

    //set the sample rate
    usrp->set_rx_rate(rate);
     
    uhd::tune_request_t tune_request(freq);
    usrp->set_rx_freq(tune_request);
        
    usrp->set_rx_gain(gain);    
    

    //set the IF filter bandwidth
    // if (vm.count("bw")) {
    //     std::cout << boost::format("Setting RX Bandwidth: %f MHz...") % (bw/1e6) << std::endl;
    //     usrp->set_rx_bandwidth(bw);
    //     std::cout << boost::format("Actual RX Bandwidth: %f MHz...") % (usrp->get_rx_bandwidth()/1e6) << std::endl << std::endl;
    // }

    //set the antenna
    usrp->set_rx_antenna(ant);

    boost::this_thread::sleep(boost::posix_time::seconds(setup_time)); //allow for some setup time

    //check Ref and LO Lock detect
    
    check_locked_sensor(usrp->get_rx_sensor_names(0), "lo_locked", boost::bind(&uhd::usrp::multi_usrp::get_rx_sensor, usrp, _1, 0), setup_time);


    if (total_num_samps == 0){
        std::signal(SIGINT, &sig_int_handler);
        std::cout << "Press Ctrl + C to stop streaming..." << std::endl;
    }

#define recv_to_file_args(format) \
    (usrp, format, wirefmt, file, spb, total_num_samps, total_time, stats, null, continue_on_bad_packet)
    //recv to file
    if (type == "double") recv_to_file<std::complex<double> >recv_to_file_args("fc64");
    else if (type == "float") recv_to_file<std::complex<float> >recv_to_file_args("fc32");
    else if (type == "short") recv_to_file<std::complex<short> >recv_to_file_args("sc16");
    else throw std::runtime_error("Unknown type " + type);

    //finished
    std::cout << std::endl << "Done!" << std::endl << std::endl;

    return EXIT_SUCCESS;
}
