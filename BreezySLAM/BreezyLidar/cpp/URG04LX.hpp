/**
*
* URG04LX.hpp - C++ header for BreezyLidar URG-04LX class
*
* Copyright (C) 2014 Simon D. Levy

* This code is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as 
* published by the Free Software Foundation, either version 3 of the 
* License, or (at your option) any later version.
* 
* This code is distributed in the hope that it will be useful,     
* but WITHOUT ANY WARRANTY without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
* 
* You should have received a copy of the GNU Lesser General Public License 
* along with this code.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <string>
#include <iostream>
using namespace std; 

/**
* A class for the Hokuyo URG-04LX Lidar unit.
*/
class URG04LX
{
    
public:
    
/**
* Builds a URG04LX object.
* @param debug set to true for debugging output
* 
*/    
URG04LX(const bool debug = false);

/**
* Connects to a URG04-LX device.
* @param device  device name
* @param baud_rate baud rate (irrelevant for USB)
* 
*/    
int connect(const std::string device, int baud_rate = 115200);

/**
* Deallocates this URG04LX object.
* 
*/
~URG04LX();

/**
* Gets a scan from the URG-04LX.
* @param gets scan scan range values in mm. Should be large enough to hold all 
         682 values.
  @return number of range values obtained       
*/
int getScan(unsigned int * range);


friend ostream& operator<< (ostream & out, URG04LX & urg);


private:    
    
    void * hokuyo;
};
