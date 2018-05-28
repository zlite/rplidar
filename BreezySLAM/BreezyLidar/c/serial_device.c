/*

serial_device.c Support for interacting with serial devices

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

#include <stdio.h>
#include <string.h>
#include <unistd.h> 
#include <fcntl.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <sys/time.h>
#include <errno.h>
#include <stdlib.h>
#include <stdbool.h>

#include "serial_device.h"
#include "message_utils.h"

#define MAX_DEVICE_NAME_LENGTH 128
#define MAX_NUM_TERM_CHARS     128

/*  io modes */ 
enum 
{
    IO_BLOCK_W_TIMEOUT,
    IO_NONBLOCK_POLL_W_DELAY_W_TIMEOUT,
    IO_BLOCK_WO_TIMEOUT,
    IO_NONBLOCK_WO_TIMEOUT,
    IO_BLOCK_W_TIMEOUT_W_TERM_SEQUENCE,
    IO_BLOCK_WO_TIMEOUT_W_TERM_SEQUENCE
};


typedef struct serial_device_t
{
    char device[MAX_DEVICE_NAME_LENGTH];   	/* devince name */
    speed_t baud;                          	/* baud rate */
    int fd;                                	/* file descriptor */
    bool connected;                        	/* status */
    int block;                             	/* block / non-block IO */
    
    int ioMode, delay_us, numTermChars;
    char termSequence[MAX_NUM_TERM_CHARS];
    bool retTermSequence;
    
    struct termios oldterm, newterm;       	/* terminal structs */
    
} serial_device_t;



/* Local routines =========================================================== */

static int set_blocking_io(serial_device_t * s, const char * caller)
{  
    /* check whether we are connected to the device */
    if (!s->connected)
    {
        return error_return(caller, "set_blocking_io: not connected to the device");
    }  
    
    /* only set blocking if not already set */
    if (s->block != 1)
    {
        /* Read the flags */
        int flags;
        if((flags = fcntl(s->fd,F_GETFL)) < 0) 
        {
            return error_return(caller, "set_blocking_io: unable to get device flags");
        } 
        
        /* Set the new flags */
        if(fcntl(s->fd,F_SETFL,flags & (~O_NONBLOCK)) < 0)
        {
            return error_return(caller, "set_blocking_io: unable to set device flags");
        }
        
        s->block=1;
    }
    return 0;
}

static int set_nonblocking_io(serial_device_t * s, const char * caller)
{
    /* check whether we are connected to the device */
    if (!s->connected)
    {
        return error_return(caller, "set_nonblocking_io: not connected to the device");   
    }  
    
    /* only set non-blocking if not already set */
    if (s->block != 0)
    {
        /* Read the flags */
        int flags;
        if((flags = fcntl(s->fd,F_GETFL)) < 0)
        {
            return error_return(caller, "set_nonblocking_io: unable to retrieve device flags");   
        }
        
        /* Set the new flags */
        if(fcntl(s->fd,F_SETFL,flags | O_NONBLOCK) < 0)
        {
            return error_return(caller, "set_nonblocking_io: unable to set device flags");
        }
        
        s->block=0;
    }
    
    return 0;
}


/* convert integer speed to baud rate setting */
static int speed_to_baud(const char * caller, int speed, speed_t * baud)
{
    switch (speed) 
    {
    case 2400:
        *baud=B2400;
        return 0;
    case 4800:
        *baud=B4800;
        return 0;
    case 9600:
        *baud=B9600;
        return 0;
    case 19200:
        *baud=B19200;
        return 0;
    case 38400:
        *baud=B38400;
        return 0;
    case 57600:
        *baud=B57600;
        return 0;
    case 115200:
        *baud=B115200;
        return 0;
        
    default:
        return error_return(caller, "speed_to_baud: unknown baud rate");
    }
}


/* Exported routines ======================================================== */

void * serial_device_create(const char * caller)
{
    serial_device_t * s = (serial_device_t *)malloc(sizeof(serial_device_t));
    
    s->connected = false;
    
    return (void *)s;
}

void serial_device_destroy(void * v, const char * caller)
{
    serial_device_disconnect(v, caller);
    
    free(v);
}

int serial_device_connect(void * v, const char * caller, const char * device, int speed)
{

    serial_device_t * s = (serial_device_t *)v;
    
    if (s->connected)
    {
        message_on_debug(true, caller, "serial_device_connect: Warning: already connected");
        return 0;
    }
    
    /* Store the device name */
    strcpy(s->device, device);
        
    /* Open the device */
    if((s->fd = open(s->device, O_RDWR | O_NOCTTY)) < 0)
    {
        return error_return(caller, "serial_device_connect: Unable to open serial port %s", s->device);
    }
    
    /* Update the connected flag */
    s->connected=true;
    
    /* Save current attributes so they can be restored after use */
    if( tcgetattr( s->fd, &s->oldterm ) < 0 )
    {
        close(s->fd);
        s->connected=false;
        return error_return(caller, "serial_device_connect: Unable to get old serial port attributes");
    }
    
    /* Set up the terminal and set the baud rate */
    if (serial_device_set_baud_rate(v, caller, speed))
    {
        close(s->fd);
        s->connected=false;
        return error_return(caller, "serial_device_connect: Unable to set baud rate");
    }
    
    /* Set the default IO mode */
    if (serial_device_set_io_block_w_timeout(v, caller))
    {
        close(s->fd);
        s->connected=false;
        return error_return(caller, "serial_device_connect: Unable to set the io mode");
    }
    
    return 0;
}


int serial_device_disconnect(void * v, const char * caller)
{
    serial_device_t * s = (serial_device_t *)v;
    
    /* check whether we are connected to the device  */
    if (!s->connected)
    {
        return 0;
    }
    
    /*  Restore old terminal settings */
    if(tcsetattr(s->fd, TCSANOW, &s->oldterm) < 0)
    {
        message_on_debug(true, caller, "serial_device_disconnect: Failed to restore attributes!");
    }
    
    /*  Actually close the device */
    if(close(s->fd) != 0)
    {
        s->connected = false;
        return error_return(caller, "serial_device_disconnect: Failed to close device!");
    }
    
    s->connected=false;
    
    return 0;
}

bool serial_device_is_connected(void * v, const char * caller)
{
    serial_device_t * s = (serial_device_t *)v;

    return s->connected;    
}


int serial_device_set_baud_rate(void * v, const char * caller, int speed)
{
    serial_device_t * s = (serial_device_t *)v;
    
    speed_t tempBaud = 0;
    
    /* check whether we are connected to the device */
    if (!s->connected)
    {
        return error_return(caller, "serial_device_set_baud_rate: not connected to the device");
    }
    
    /* convert the integer speed value to speed_t if needed */
    if (speed_to_baud(caller, speed, &tempBaud))
    {
        return error_return(caller, "serial_device_set_baud_rate: bad baud rate");
    }
    
    /* get current port settings */
    if (tcgetattr(s->fd, &s->newterm ) < 0 )
    {
        return error_return(caller, "serial_device_set_baud_rate: Unable to get serial port attributes");
    }
    
    /* cfmakeraw initializes the port to standard configuration. Use this! */
    cfmakeraw( &s->newterm );
    
    /* set input baud rate */
    if (cfsetispeed( &s->newterm, tempBaud ) < 0 )
    {
        return error_return(caller, "serial_device_set_baud_rate: Unable to set baud rate");
    }
    
    /* set output baud rate */
    if (cfsetospeed( &s->newterm, tempBaud ) < 0 )
    {
        return error_return(caller, "serial_device_set_baud_rate: Unable to set baud rate");
    }
    
    /* set new attributes  */
    if( tcsetattr(s->fd, TCSAFLUSH, &s->newterm ) < 0 )
    {
        return error_return(caller, "serial_device_set_baud_rate: Unable to set serial port attributes");
    }
    
    /* make sure queue is empty */
    tcflush(s->fd, TCIOFLUSH);
    
    /* save the baud rate value */
    s->baud = tempBaud;
    
    return 0;
}


int serial_device_flush_input_buffer(void * v, const char * caller)
{
    serial_device_t * s = (serial_device_t *)v;
    
    char c[1000];
    
    if (!s->connected)
    {
        return error_return(caller, "serial_device_flush_input_buffer: not connected to the device");
    }
    
    /* TODO tcflush for some reason does not work..  */
    /* tcflush(s->fd, TCIFLUSH); */
    
    int block=s->block;
    set_nonblocking_io(s, caller);
    
    /* read off all the chars */
    while (read(s->fd,c,1000) > 0){}
    
    if (block==1)
    {
        set_blocking_io(s, caller);
    }
    
    return 0;
}

int serial_device_read_chars(void * v, const char * caller, char * data, int byte_count, int timeout_us)
{
    serial_device_t * s = (serial_device_t *)v;
    
    if (!s->connected)
    {
        return error_return(caller, "serial_device_read_chars: not connected to the device");
    }  
    
    fd_set watched_fds;
    struct timeval timeout, start, end;
    int bytes_read_total = 0;
    int bytes_left = byte_count;
    int retval;
    int bytes_read;
    int charsMatched=0;
    
    switch (s->ioMode)
    {
        
    case IO_BLOCK_W_TIMEOUT:
        /* set up for the "select" call */
        FD_ZERO(&watched_fds);
        FD_SET(s->fd, &watched_fds);
        timeout.tv_sec = timeout_us / 1000000;
        timeout.tv_usec = timeout_us % 1000000;
        
        
        while (bytes_left) 
        {
            if ((retval = select(s->fd + 1, &watched_fds, NULL, NULL, &timeout)) < 1)   /*block until at least 1 char is available or timeout */
            {                                                                           /* error reading chars */
                if (retval < 0)
                {
                    perror("ReadChars");
                }
                else                                                                    /*timeout */
                {
                    message_on_debug(true, caller, "ReadChars: timeout");
                }
                return bytes_read_total;
            }
            bytes_read        = read(s->fd, &(data[bytes_read_total]), bytes_left);
            
            if (bytes_read > 0)
            {
                bytes_read_total += bytes_read;
                bytes_left       -= bytes_read;
            }
        }
        return bytes_read_total;
        
        
        
    case IO_NONBLOCK_POLL_W_DELAY_W_TIMEOUT:
        gettimeofday(&start,NULL);
        
        while (bytes_left) 
        {
            bytes_read = read(s->fd,&(data[bytes_read_total]),bytes_left);
            if ( bytes_read < 1)
            {
                /* If a time out then return false */
                gettimeofday(&end,NULL);
                if((end.tv_sec*1000000 + end.tv_usec) - (start.tv_sec*1000000 + start.tv_usec) > timeout_us) 
                {
                    message_on_debug(true, caller, "ReadChars: timeout");
                    return bytes_read_total;
                }
                usleep(s->delay_us);
                continue;
            }
            
            bytes_read_total += bytes_read;
            bytes_left       -= bytes_read;
        }
        return bytes_read_total;
        
        
    case IO_BLOCK_WO_TIMEOUT:
        while (bytes_left) 
        {
            bytes_read = read(s->fd,&(data[bytes_read_total]),bytes_left);
            if (bytes_read < 1)
            {
                return -1;
            }
            
            bytes_read_total += bytes_read;
            bytes_left       -= bytes_read;
        }
        return bytes_read_total;
        
    case IO_NONBLOCK_WO_TIMEOUT:
        bytes_read = read(s->fd,&(data[0]),bytes_left);
        if (bytes_read < 0) bytes_read=0;
        return bytes_read;
        
        
    case IO_BLOCK_W_TIMEOUT_W_TERM_SEQUENCE:
        
        /* set up for the "select" call */
        FD_ZERO(&watched_fds);
        FD_SET(s->fd, &watched_fds);
        timeout.tv_sec = timeout_us / 1000000;
        timeout.tv_usec = timeout_us % 1000000;
        
        while (bytes_left) 
        {
            if ((retval = select(s->fd + 1, &watched_fds, NULL, NULL, &timeout)) < 1)   /* block until at least 1 char is available or timeout */
            {                                                                         /* error reading chars */
                if (retval < 0) 
                {
                    perror("ReadChars");
                }
                else                                                                    /* timeout */
                {
                    message_on_debug(true, caller, "ReadChars: timeout. The terminating sequence has not been read");
                }
                return -1;
            }
            bytes_read = read(s->fd, &(data[bytes_read_total]), 1);
            
            if (bytes_read==1)
            {
                if (data[bytes_read_total]==s->termSequence[charsMatched])
                {
                    charsMatched++;
                }
                else 
                {
                    charsMatched=0;
                }
                
                bytes_read_total += bytes_read;
                bytes_left       -= bytes_read;
                
                if (charsMatched==s->numTermChars)
                {
                    if (s->retTermSequence)
                    {
                        return bytes_read_total;
                    }
                    
                    else 
                    {
                        return bytes_read_total-s->numTermChars;
                    }
                }
            }
        }
        
        return error_return(caller, "serial_device_read_chars: Read too much data. The terminating sequence has not been read");
        
    default:
        
        return error_return(caller, "serial_device_read_chars: Bad io mode ");
    }
    
}

int serial_device_write_chars(void * v, const char * caller, const char * data, int byte_count, int delay_us)
{
    serial_device_t * s = (serial_device_t *)v;
    
    int bytes_written_total=0;
    int bytes_written;
    int bytes_left=byte_count;
    
    
    if (!s->connected)
    {
        return error_return(caller, "serial_device_write_chars: not connected to the device");
    }
    
    if (delay_us==0)
    {
        bytes_written_total=write(s->fd,data,byte_count);
    }
    
    else
    {
        while (bytes_left)
        {
            bytes_written=write(s->fd,&(data[bytes_written_total]),1);
            if (bytes_written < 0)
            {
                perror("ReadChars");
            }
            if (bytes_written < 1)
            {
                message_on_debug(true, caller, "WriteChars: Could not write a char");
                return bytes_written_total;
            }
            
            bytes_written_total += bytes_written;
            bytes_left       -= bytes_written;
            usleep(delay_us);
        }
    }  
    
    tcdrain(s->fd);   /* wait till all the data written to the file descriptor is transmitted */
    
    return bytes_written_total;
}

int serial_device_set_io_block_w_timeout(void * v, const char * caller)
{
    serial_device_t * s = (serial_device_t *)v;
    
    /* check whether we are connected to the device */
    if (!s->connected)
    {
        return error_return(caller, "serial_device_set_io_block_w_timeout: not connected to the device");
    }
    
    if (set_blocking_io(s, caller))
    {
        return error_return(caller, "serial_device_set_io_block_w_timeout: could not set blocking io");
    }
    
    s->ioMode=IO_BLOCK_W_TIMEOUT;
    s->delay_us=0;
    s->numTermChars=0;
    s->retTermSequence=false;
    
    return 0;
}    

int serial_device_set_io_nonblock_poll_w_delay_w_timeout(void * v, const char * caller, int delay_us)
{
    serial_device_t * s = (serial_device_t *)v;
    
    if (!s->connected)
    {
        return error_return(caller, "serial_device_set_io_nonblock_poll_w_delay_w_timeout: not connected to the device");
    }
    
    if (set_nonblocking_io(s, caller))
    {
        return error_return(caller, "serial_device_set_io_nonblock_poll_w_delay_w_timeout: could not set non-blocking io");
    }
    
    s->ioMode = IO_NONBLOCK_POLL_W_DELAY_W_TIMEOUT; 
    s->delay_us = delay_us;
    s->numTermChars = 0;
    s->retTermSequence = false;
    
    return 0;   
}

int serial_device_set_io_block_wo_timeout(void * v, const char * caller)
{
    serial_device_t * s = (serial_device_t *)v;
    
    if (!s->connected)
    {
        return error_return(caller, "serial_device_set_io_block_wo_timeout: not connected to the device");
    }
    
    if (set_blocking_io(s, caller))
    {
        return error_return(caller, "serial_device_set_io_block_wo_timeout: could not set blocking io");
    }
    s->ioMode = IO_BLOCK_WO_TIMEOUT;
    s->delay_us = 0;
    s->numTermChars = 0;
    s->retTermSequence = false;
    
    return 0;   
}

int serial_device_set_io_nonblock_wo_timeout(void * v, const char * caller)
{
    serial_device_t * s = (serial_device_t *)v;
    
    if (!s->connected)
    {
        return error_return(caller, "serial_device_se_io_nonblock_wo_timeout: not connected to the device");
    }
    
    if (set_nonblocking_io(s, caller))
    {
        return error_return(caller, "serial_device_se_io_nonblock_wo_timeout: could not set non-blocking io");
    }
    
    s->ioMode = IO_NONBLOCK_WO_TIMEOUT;
    s->delay_us = 0;
    s->numTermChars = 0;
    s->retTermSequence = false;
    
    return 0;
}

int serial_device_set_io_block_w_timeout_w_term_sequence(void * v, const char * caller, const char * termSequence, int numTermChars, bool retTermSequence)
{
    serial_device_t * s = (serial_device_t *)v;
    
    if (!s->connected)
    {
        return error_return(caller, "serial_device_set_io_block_w_timeout_w_term_sequence: not connected to the device");
    }
    
    if (set_blocking_io(s, caller))
    {
        return error_return(caller, "serial_device_set_io_block_w_timeout_w_term_sequence: could not set blocking io");
    }
    
    if (numTermChars < 1 || numTermChars > MAX_NUM_TERM_CHARS)
    {
        return error_return(caller, "serial_device_set_io_block_w_timeout_w_term_sequence: bad number of chars");
    }
    s->ioMode = IO_BLOCK_W_TIMEOUT_W_TERM_SEQUENCE;
    s->numTermChars = numTermChars;
    s->retTermSequence = retTermSequence;
    memcpy(s->termSequence, termSequence, numTermChars*sizeof(char));
    s->delay_us = 0;
    
    return 0;
}
