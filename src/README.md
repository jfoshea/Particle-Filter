# Particle Filter (Kidnapped Vehicle Project)

## Overview 
This project implements a 2 dimensional particle filter in C++. The particle filter is given a map and some initial localization information, similar to what a GPS would provide. At each time step the filter will also get observation and control data.

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros

## Build Instructions 
1. Clone the Particle Filter git repository
    ```  
    $ git clone https://github.com/jfoshea/Particle-Filter.git
    ```
2. Make a build directory if it does not exist
    ```  
    $ mkdir build && cd build 
    ```
3. Compile 
    ```  
    $ cmake .. && make 
    ```
4. Run the Particle Filter 
    ```  
    $ ./particle_filter 
    ```
