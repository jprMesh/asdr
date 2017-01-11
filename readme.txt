How to use:
The different components of the project are held in separate folders. MATLAB scripts
are contained in the folder MATLAB_Scripts. The main C++ MASDR framework code is in
application, while the script that rotates the drone is in drone_code. The C++ sdr
logging code is in iq_to_file. The python code that takes the rss values and gps values
to localize is in localization. The report is in LaTeX, and is in the report folder.

To build the c++ programs, run the following commands in a linux terminal.

cd ~destination_folder~
mkdir build
cd build
cmake ..
make

