/*

serial_device.h Support for interacting with serial devices

Adapted by Simon D. Levy from downloaded from

  http://uri-imaging.googlecode.com/svn/trunk/Softwares/Hokuyo/
  
on 16 July 2014.

This code is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as 
published by the Free Software Foundation, either version 3 of the 
License, or (at your option) any later version.

This code is distributed in the hope that it will be useful,     
but WITHOUT ANY WARRANTY without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU Lesser General Public License 
along with this code.  If not, see <http:#www.gnu.org/licenses/>.
*/

void * serial_device_create(const char * caller);

void   serial_device_destroy(void * v, const char * caller);

int    serial_device_connect(void * v, const char * caller, const char * device, int speed);

int    serial_device_disconnect(void * v, const char * caller);

bool   serial_device_is_connected(void * v, const char * caller);

int    serial_device_set_baud_rate(void * v, const char * caller, int baud); 

int    serial_device_flush_input_buffer(void * v, const char * caller);                        

int    serial_device_read_chars(void * v, const char * caller, char * data, int byte_count, int timeout_us);     

int    serial_device_write_chars(void * v, const char * caller, const char * data, int byte_count, int delay_us);      

int    serial_device_set_io_block_w_timeout(void * v, const char * caller);

int    serial_device_set_io_nonblock_poll_w_delay_w_timeout(void * v, const char * caller, int delay);

int    serial_device_set_io_block_wo_timeout(void * v, const char * caller);

int    serial_device_set_io_nonblock_wo_timeout(void * v, const char * caller);

int    serial_device_set_io_block_w_timeout_w_term_sequence(void * v, const char * caller, const char * termsequence, int numtermchars, bool rettermsequence);

