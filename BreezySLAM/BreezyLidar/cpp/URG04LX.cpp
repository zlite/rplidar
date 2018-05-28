/**
*
* URG04LX.cpp - C++ code for BreezyLidar URG04LX class
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


#include "URG04LX.hpp"

#include "hokuyo.h"


URG04LX::URG04LX(const bool debug)

{
    this->hokuyo = hokuyo_create("URG04LX::URG04LX", debug);
}

URG04LX::~URG04LX()
{
    hokuyo_destroy(this->hokuyo, "URG04LX::~URG04LX");
    
}

int URG04LX::connect(const std::string device, int baud_rate)
{
    return hokuyo_connect(this->hokuyo, "URG04LX::connect", (char *)device.c_str(), baud_rate);    
}
  
int URG04LX::getScan(unsigned int * range)
{        
    return hokuyo_get_scan(this->hokuyo, "URG04LX::getScan", range);
}


ostream& operator<< (ostream & out, URG04LX & urg)
{
    char str[1000];
    
    hokuyo_get_str(urg.hokuyo, "ostream& operator<<", str);
        
    out << str;
    
    return out;
}












    
