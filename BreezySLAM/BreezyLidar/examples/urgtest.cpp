/*
urtest.cpp : C++ lest program for BreezyLidar on Hokuyo URG-04LX
             
Copyright (C) 2014 Simon D. Levy

This code is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as 
published by the Free Software Foundation, either version 3 of the 
License, or (at your option) any later version.

This code is distributed in the hope that it will be useful,     
but WITHOUT ANY WARRANTY without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU Lesser General Public License 
along with this code.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <stdio.h>

#include "URG04LX.hpp"

//static const char * DEVICE = "/dev/tty.usbmodem1421";
static const char * DEVICE = "/dev/ttyACM0";

static const int    NITER  = 20;

int main()
{
	
	URG04LX laser;
	
	laser.connect(DEVICE);
	
	cout << "===============================================================" << endl;
	cout << laser << endl;
	cout << "===============================================================" << endl;
		
	for (int i=1;i<=NITER;i++) 
	{	    
		unsigned int data[1000];
		
		int ndata = laser.getScan(data);
		
		printf("Iteration: %3d: ", i);
		
		if (ndata) 
		{
			printf("got %3d data points\n", ndata);
		}
		else
		{
			printf("=== SCAN FAILED ===\n");
		}
	}
	
	return 0;
}
