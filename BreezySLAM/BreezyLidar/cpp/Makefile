# Makefile : builds libbreezylidar.so C++ library
#
# Copyright (C) Simon D. Levy 2014
#
# This code is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as 
# published by the Free Software Foundation, either version 3 of the 
# License, or (at your option) any later version.
#
# This code is distributed in the hope that it will be useful,     
# but WITHOUT ANY WARRANTY without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
#  You should have received a copy of the GNU Lesser General Public License 
#  along with this code.  If not, see <http:#www.gnu.org/licenses/>.
#  You should also have received a copy of the Parrot Parrot AR.Drone 
#  Development License and Parrot AR.Drone copyright notice and disclaimer 
#  and If not, see 
#   <https:#projects.ardrone.org/attachments/277/ParrotLicense.txt> 
# and
#   <https:#projects.ardrone.org/attachments/278/ParrotCopyrightAndDisclaimer.txt>.

# Where you want to put the library
LIBDIR = /usr/local/lib

# Set library extension based on OS
ifeq ("$(shell uname)","Darwin")
  LIBEXT = dylib
else ifeq ("$(shell uname)","Linux")
  CFLAGS = -fPIC
  LIBEXT = so
else
  LIBEXT = dll
endif

all: libbreezylidar.$(LIBEXT)

libbreezylidar.$(LIBEXT): URG04LX.o serial_device.o hokuyo.o
	g++ -shared URG04LX.o serial_device.o hokuyo.o -o libbreezylidar.$(LIBEXT) -lm -lpthread

URG04LX.o : URG04LX.cpp URG04LX.hpp ../c/hokuyo.h
	$(CXX) -Wall -c -I../c URG04LX.cpp -fPIC

serial_device.o : ../c/serial_device.c ../c/serial_device.h ../c/message_utils.h
	$(CXX) -Wall -c -I../c ../c/serial_device.c -fPIC

hokuyo.o : ../c/hokuyo.c ../c/hokuyo.h ../c/message_utils.h
	$(CXX) -Wall -c -I../c ../c/hokuyo.c -fPIC

install: libbreezylidar.$(LIBEXT)
	cp libbreezylidar.$(LIBEXT) $(LIBDIR)
	
doc:
	doxygen

clean:
	rm -rf  libbreezylidar.$(LIBEXT) *.o Documentation \#* *~
