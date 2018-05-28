/*

hokuyo.c Functions for for Hokuyo classes in BreezyLidar

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

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <unistd.h>
#include <math.h>
#include <pthread.h>
#include <errno.h>
#include <signal.h>
#include <sys/time.h>

#include "hokuyo.h"
#include "serial_device.h"
#include "message_utils.h"

#define MAXSTR 		       100
#define MAX_DATA_LENGTH    10000
#define MAX_PACKET_LENGTH  15000
#define MAX_LINE_LENGTH    100

static const int NUM_TEST_BAUD_RETRIES = 2;
static const int MAX_NUM_POINTS        = 3000;

enum
{
	TYPE_URG_04LX = 0,
	TYPE_UTM_30LX = 1
};

static const char * TYPE_URG_04LX_STRING = "SOKUIKI Sensor URG-04LX";
static const char * TYPE_UTM_30LX_STRING = "SOKUIKI Sensor TOP-URG UTM-30LX";

/* everything except the UTM-30LX ME scan */
static const int SCAN_REGULAR = 0;

/* only for UTM-30LX scan, which is weird in terms of  */
/* returning command confirmation, so need to take special care */
/* in handling the responses */
static const int SCAN_SPECIAL_ME = 1;

/* character encoding */
enum 
{
    TWO_DIGITS =   2,
    THREE_DIGITS = 3
};

/* SCIP modes */
/*static const int SCIP1  = 0; */
static const int SCIP20 = 1;

static const int SERIAL = 0;

static const int NUM_STOP_LASER_RETRIES     = 5;
static const int LASER_STOP_DELAY_US        = 50000;
static const int IDLE_USEC                  = 100000;
static const int TIMED_WAIT_USEC            = 1000;

static const int URG_04LX_GET_SCAN_TIMEOUT_US = 500000;
static const int UTM_30LX_GET_SCAN_TIMEOUT_US = 500000;

static const char * VEND_STR = "VEND";
static const char * PROD_STR = "PROD";
static const char * FIRM_STR = "FIRM";
static const char * PROT_STR = "PROT";
static const char * SERI_STR = "SERI";
static const char * MODL_STR = "MODL";

static const char * DMIN_STR = "DMIN";
static const char * DMAX_STR = "DMAX";
static const char * ARES_STR = "ARES";
static const char * AMIN_STR = "AMIN";
static const char * AMAX_STR = "AMAX";
static const char * AFRT_STR = "AFRT";
static const char * SCAN_STR = "SCAN";

static const int READER_MAX_NUM_ERRORS_BEFORE_RESTART = 3;
static const int READER_GET_SCAN_TIMEOUT_MSEC         = 400;
static const int READER_SET_SCAN_PARAMS_TIMEOUT_MSEC  = 500;


/* XXX for now, support only URG04LX */
static const int SCAN_START  = 44;           
static const int SCAN_END    = 725;            
static const int SCAN_SKIP   = 1;
static const int ENCODING    = THREE_DIGITS; 
static const int SCAN_TYPE   = 0;         


typedef struct hokuyo_t
{
	/* device type (URG-04LX or UTM-30LX) */
	int type;
	
	/* scan-ready flag */
	bool scan_ready;
	
	/* serial-device object that's used for communication */
	void * sd;
	
	/* thread support */
	pthread_mutex_t data_mutex;
	pthread_cond_t data_cond;
	
	/* a path to the device at which the sick can be accessed.*/
	char device[MAXSTR];
	
	/* sensor information */
	char vendor[MAXSTR];
	char product[MAXSTR];
	char firmware[MAXSTR];
	char protocol[MAXSTR];
	char serial[MAXSTR];
	char model[MAXSTR];
	char dmin[MAXSTR];
	char dmax[MAXSTR];
	char ares[MAXSTR];
	char amin[MAXSTR];
	char amax[MAXSTR];
	char afrt[MAXSTR];
	
	char scan[MAXSTR];
	double dist_min, dist_max, angle_res, angle_min, angle_max, scan_rate, dist_res;
	int count_min, count_max, count_zero;
	
	/* hokuyo mode (scip1 or scip2.0) */
	int mode;
	
	/* is the scanner streaming data? */
	bool streaming;
	
	/* scan start count */
	int scan_start;
	
	/* scan end count */
	int scan_end;
	
	/* this effectively sets the resolution (or so-called cluster count, aka "skip value") */
	/* however in urg 04_lx special value of this variable is used to obtain intensity data */
	int scan_skip;
	
	/* 2 or 3 char encoding*/
	int encoding;
	
	/*regular or special me intensity scan for utm-30_lx */
	int scan_type;
	
	/* length of measurement packet, in bytes, including the eol chars */
	int packet_length;
	
	/* communication protocol for the device: serial port or tcp (serial device server) */
	int com_protocol;
	
	/* the baud rate at which to communicate with the hokuyo */
	int baud;
	
	/* flag for debugging output */
	bool debug;
	
	/* status flags */
	bool active;
	bool need_to_stop_laser;
	
	/* scan data */
	unsigned int data[MAX_DATA_LENGTH];
	int num_data;
	
	/* threading support */
	pthread_cond_t settings_cond;
	pthread_mutex_t settings_mutex;
	pthread_t thread;
	bool thread_is_running;
	
	
} hokuyo_t;



/* Local helpers ============================================================ */

static int _check_result(const char * s, char * line)
{
    return strncmp(s, line, 2);    
}

static bool _is_connected(hokuyo_t * h, const char * caller)
{
    return serial_device_is_connected(h->sd, caller);       
}

/* verify the checksum of the line, which is the last character of the line */
static int _check_line_checksum(hokuyo_t * h, const char * caller, char * line, int max_length)
{
	
	char * end_line;
	int line_length;
	char sum=0;
	
	end_line=(char *)memchr((void *)line, 0x0a, max_length);
	if (!end_line)
	{
		message_on_debug(h->debug, caller, "_check_line_checksum: end of line not found");
	}
	
	line_length=end_line-line;
	
	/* sanity check */
	if ( (line_length < 0) || (line_length > MAX_LINE_LENGTH))
	{
		message_on_debug(h->debug, caller, "_check_line_checksum: bad line length");
	}
	
	/* empty line */
	if (line_length==0)
	{
		return 0;
	}
	
	/* compute the checksum up to the checksum character */
	int j;
	for (j=0;j<line_length-1;j++)
	{
		sum+=line[j];
	}
	
	/* encode the sum */
	sum = (sum & 0x3f) + 0x30;
	
	if (sum!=line[line_length-1])
	{
		message_on_debug(h->debug, caller, "_check_line_checksum: checksum error");
	}
	
	return (line_length-1);
}


/* read one line from the sensor and optionally verify the checksum */
/* max specifies the maximum expected length of the line */
static int _read_line(hokuyo_t * h, const char * caller, char * line,int max_chars, int timeout_us,bool check_sum)
{
	char seq[1]={0x0a};   /* lf */
	
	char tmp[1000];
	sprintf(tmp, "%s:%s", caller, "_read_line");
	const char * caller2 = (const char *)tmp;
	
	if (!_is_connected(h, caller2))
	{
		return error_return(caller2, "not connected to the sensor!");
	}
	
	/* set the io mode to return when lf is read or timeout or read too much */
	serial_device_set_io_block_w_timeout_w_term_sequence(h->sd, caller2, seq, 1, true);
	
	/* read the line */
	int num_chars = serial_device_read_chars(h->sd, caller2, line, max_chars, timeout_us);
	
	if ( num_chars< 1)
	{
		if (h->debug)
		{
			return error_return(caller2, "terminating character was not read");
		}
	}
	
	/* empty line, therefore return */
	if (num_chars==1)
	{
		if (line[0]==0x0a)
			return 0;
		
		else
		{
			if (h->debug)
			{
				return error_return(caller2, "error: read one char and it was not an endl");
			}
		}
	}
	
	if (!check_sum)
	{
		return num_chars-1;
	}
	
	
	if (_check_line_checksum(h, caller2, line, num_chars) != num_chars-2)
	{
		if (h->debug)
		{
			return error_return(caller2, "checksum error");
		}
	}
	
	return (num_chars-2);
}


static int _send_command2(hokuyo_t * h, const char * caller, const char * cmd)
{
    char buf[MAXSTR];
    sprintf(buf, "%s\n", cmd);
    
	return serial_device_write_chars(h->sd, caller, buf, strlen(buf), 0) != strlen(buf);
}

static int _laser_on_off(hokuyo_t * h, const char * caller, bool turn_on)
{
	char line[MAX_LINE_LENGTH];
	
	char tmp[1000];
	sprintf(tmp, "%s:%s", caller, "_laser_on_off");
	const char * caller2 = (const char *)tmp;

	if (!_is_connected(h, caller2))
	{
		return error_return(caller2, "not connected to the sensor!");
	}
		
	const char * resp1 = "00";
	const char * resp2 = turn_on ? "02" : "00";
	const char * cmd = turn_on ? "BM" : "QT";
	
	/* clear out the serial input buffer   */
	serial_device_flush_input_buffer(h->sd, caller2);
	
	/* turn the laser on or off */
	if (_send_command2(h, caller2, (char *)cmd))
	{
		message_on_debug(h->debug, caller2, "could not write command");
	}
	
	/* read back the echo of the command */
	int timeout_us=200000;
	int line_len= _read_line(h, caller2, line, MAX_LINE_LENGTH, timeout_us, false);
	if (line_len < 0)
	{
		message_on_debug(h->debug, caller2, "could not read line 1");
	}
	
	if (strncmp(cmd,line,line_len))
	{
		message_on_debug(h->debug, caller2, "response does not match (line 1):");
	}
	
	/* read back the status response */
	line_len = _read_line(h, caller, line, MAX_LINE_LENGTH, timeout_us, true);
	if (line_len < 0)
	{
		message_on_debug(h->debug, caller2, ": could not read line 2");
	}
	
	if ( (strncmp(resp1, line, line_len)!=0) && (strncmp(resp2,line,line_len)!=0))
	{
		message_on_debug(h->debug, caller2, ": response does not match (line 2):");
	}
	
	/* read off the LF */
	line_len = _read_line(h, caller, line,MAX_LINE_LENGTH,timeout_us,false);
	if (line_len < 0)
	{
		message_on_debug(h->debug, caller2, ": could not read line 3");
	}
	
	if (!turn_on)
	{
		h->streaming = false;
	}
	
	
	return 0;
}


/* send the command to turn on/off the laser */
static int _laser_on(hokuyo_t * h, const char * caller) 
{    
	_laser_on_off(h, caller, false);
	
	if (_laser_on_off(h, caller, true))
	{
		return error_return(caller, "_laser_on:unable to turn off the laser");
	}
	
	message_on_debug(h->debug, caller, "_laser_on: Laser has been turned on");
	
	return 0;
}

static int _laser_off(void * v, const char * caller) 
{
	hokuyo_t * h = (hokuyo_t *)v;
	
	int i;
	for (i=0; i<NUM_STOP_LASER_RETRIES; i++)
	{
		if (!_laser_on_off(h, caller, false))
		{
			message_on_debug(h->debug, caller, "_laser_off: laser has been turned off");
			return 0;
		}
		
		else if (i == NUM_STOP_LASER_RETRIES-1)
		{
			message_on_debug(h->debug, caller, "_laser_off: unable to shut off laser");
		}
		usleep(LASER_STOP_DELAY_US);
	}
	
	return -1;
}


/* try to get status information from the sensor, using a given baud rate
this will only succeed if it is able to switch to SCIP2.0 mode
and send QT (stop laser command) and receive appropriate confirmation,
described in SCIP2.0 protocol */
static int _test_baud_rate(hokuyo_t * h, const char * caller, const int baud_rate)
{
	char tmp[1000];
	sprintf(tmp, "%s:%s", caller, "_test_baud_rate");
	const char * caller2 = (const char *)tmp;

	if (!_is_connected(h, caller2))
	{
		return error_return(caller2, "not connected to the sensor!");
	}
		
	int i;
	for (i=0;i<NUM_TEST_BAUD_RETRIES;i++)
	{
		/*  Attempt to get status information at the current baud  */
		message_on_debug(h->debug, caller2, " testing baud rate %d", baud_rate);
		
		/*  Set the host terminal baud rate to the test speed  */
		if (!serial_device_set_baud_rate(h->sd, caller2, baud_rate))
		{       
			
			/* stop the laser in case it was streaming */
			_laser_off(h, caller);
			
			/* verify that the mode is set to SCIP2.0 by sending the laser off command */
			if (!_laser_off(h, caller))
			{
				message_on_debug(h->debug, caller2, "SCIP2.0 Mode set.");
				h->baud = baud_rate;
				return 0;
			}
			
			else
			{
				return error_return(caller2, "setting SCIP2.0 failed!" );
			}
			
		}
		
		else
		{
			return error_return(caller2, "setting baud rate failed!");
		}
		
		usleep(IDLE_USEC);
	}
	
	return -1;
}


static int _create_scan_request(hokuyo_t * h, const char * caller,
	int scan_start, int scan_end, int scan_skip,  int encoding, 
	int scan_type, int num_scans, char * req, 
	bool * need_to_read_off_cmd_and_status)
{
	/* error checking */
	if (scan_start < 0)
	{
		return error_return(caller, "_create_scan_request: bad scan_start");
	}
	
	if (scan_end < 0)
	{
		return error_return(caller, "_create_scan_request: bad scan_end");
	}
	
	if ((h->type == TYPE_UTM_30LX) && (scan_end > MAX_NUM_POINTS_UTM_30LX))
	{
		return error_return(caller, "_create_scan_request: scan_end is greater than maximum number of points for UTM-30LX");
	}
	
	if ((h->type == TYPE_URG_04LX) && (scan_end > MAX_NUM_POINTS_URG_04LX))
	{
		return error_return(caller, "_create_scan_request: scan_end is greater than maximum number of points for URG-04LX");
	}
	
	/* correct order of start and end */
	if (scan_end <= scan_start)
	{
		return error_return(caller, "_create_scan_request: scan_end < scan_start");
	}
	
	/* valid number of scans */
	if ((num_scans < 0) || (num_scans > 99))
	{
		return error_return(caller, "_create_scan_request: bad num_scans");
	}
	
	/*  */
	if ((h->type == TYPE_UTM_30LX) && (encoding == TWO_DIGITS))
	{
		return error_return(caller, "_create_scan_request: UTM-30LX does not support 2-digit data mode");
	}
	
	/* single scan */
	if (num_scans==1)
	{
		if (scan_type==SCAN_REGULAR)
		{  
			if (encoding == THREE_DIGITS)
			{
				sprintf(req,"GD%04d%04d%02x",scan_start,scan_end,scan_skip);
				*need_to_read_off_cmd_and_status = false;
			}
			
			else if (encoding == TWO_DIGITS)
			{
				sprintf(req,"GS%04d%04d%02x",scan_start,scan_end,scan_skip);
				*need_to_read_off_cmd_and_status = false;
			}
			
			else 
			{
				return error_return(caller, "_create_scan_request: invalid selection of character encoding");
			}
		} 
		
		else if ((scan_type==SCAN_SPECIAL_ME) && (h->type == TYPE_UTM_30LX) ) 
		{
			if (encoding == THREE_DIGITS)
			{
				sprintf(req,"ME%04d%04d%02x0%02d",scan_start,scan_end,scan_skip,num_scans);
				*need_to_read_off_cmd_and_status = true;
			}
			
			else
			{
				return error_return(caller, "_create_scan_request: invalid selection of character encoding");
			}
		}
		else 
		{
			return error_return(caller, "_create_scan_request: invalid scan type");
		}
	}
	
	/* streaming */
	else if (num_scans==0)
	{
		*need_to_read_off_cmd_and_status = true;
		if (scan_type==SCAN_REGULAR)
		{  
			if (encoding == THREE_DIGITS)
				sprintf(req,"MD%04d%04d%02x0%02d",scan_start,scan_end,scan_skip,num_scans);
			
			else if (encoding == TWO_DIGITS)
				sprintf(req,"MS%04d%04d%02x0%02d",scan_start,scan_end,scan_skip,num_scans);
			
			else
			{
				return error_return(caller, "_create_scan_request: invalid selection of character encoding");
			}
		} 
		
		else if ((scan_type==SCAN_SPECIAL_ME) && (h->type == TYPE_UTM_30LX) )
		{
			if (encoding == THREE_DIGITS)
				sprintf(req,"ME%04d%04d%02x0%02d",scan_start,scan_end,scan_skip,num_scans);
			else 
			{
				return error_return(caller, "_create_scan_request: invalid selection of character encoding");
			}
		}
		
		else 
		{
			return error_return(caller, "_create_scan_request: invalid scan type");
		}
	}
	
	else 
	{
		return error_return(caller, "_create_scan_request: invalid number of scans");
	}
	
	return 0;
}


static int _parse_string(char * buf, const char * name, char * value)
{
	char tmp[MAXSTR];
	strcpy(tmp, buf);
	
	char * p = strchr(tmp, ':');
	
	*p = 0;
	
	if (strcmp(tmp, name))
	{
		return -1;
	}
	
	p++;
	
	*strchr(p, ';') = 0;
	
	strcpy(value, p);
	
	return 0;
}

/* read the packet either as a one blocked read or look for the terminating sequence */
static int _read_packet(hokuyo_t * h, const char * caller, char * data, int * packet_length, int timeout_us)
{
	char tmp[1000];
	sprintf(tmp, "%s:%s", caller, "_read_packet");
	const char * caller2 = (const char *)tmp;

	if (!_is_connected(h, caller2))
	{
		return error_return(caller2, "not connected to the sensor");
	}
	
	/* if packet length is greater than zero, it means that we already know */
	/* the expected length, therefore just use that - dont need to explicitly */
	/* search for the end of packet - just need to check for it at the end. */
	if (*packet_length > 0)
	{
		serial_device_set_io_block_w_timeout(h->sd, caller2);  
		if (serial_device_read_chars(h->sd, caller2, data, *packet_length, timeout_us) != *packet_length)
		{
			return error_return(caller2, "could not read the number of expected characters");
		}
	}
	
	/* packet length is unknown, therefore we need to find it */
	else 
	{
		char term_sequence[2] = {0x0a, 0x0a};
		
		/* set the io mode to return when the termination sequence is read */
		/* then we know that we've reached the end of some packet */
		serial_device_set_io_block_w_timeout_w_term_sequence(h->sd, caller2, term_sequence, 2, true);
		
		/* read until the end sequence is found or until read too much */
		int chars_read=serial_device_read_chars(h->sd, caller2, data,MAX_PACKET_LENGTH,timeout_us);
		
		if (chars_read < 0)
		{
			return error_return(caller2, "could not read the terminating character while trying to determine the packet length");
		}
		
		*packet_length = chars_read;
	}
	
	return 0;
}


/* extract the data by verifying the checksum and removing it along with l_fs */
static int _extract_packet(hokuyo_t * h, const char * caller, char * full_packet, char * extracted_packet, int packet_length)
{
	int len_nlfc=0;     /* length of data without l_fs and checksum */
	int line_length;
	int line_count=0;
	
	
	if (packet_length < 1)
	{
		return error_return(caller, "_extract_packet: bad length");
	}
	
	line_length = _check_line_checksum(h, caller, full_packet, MAX_LINE_LENGTH);
	
	/* line length will be zero at the end of packet, when the last lf is read */
	while (line_length > 0)
	{
		line_count++;
		
		/* copy the data */
		memcpy(extracted_packet,full_packet,line_length);
		
		extracted_packet+=line_length;
		full_packet+=(line_length+2);     /* advance the pointer 2 chars ahead to skip the checksum and endline char */
		len_nlfc+=line_length;
		
		/* check the next line */
		line_length=_check_line_checksum(h, caller, full_packet, MAX_LINE_LENGTH);
	}
	if (line_length < 0)
	{
		return error_return(caller, "_extract_packet: error extracting a line!!!");
	}
	
	return len_nlfc;
}

/* decode the packet using 2- or 3-character enconding */
static int _decode_packet( hokuyo_t * h, const char * caller, char * extracted_packet, unsigned int * extracted_data, int extracted_length, int encoding)
{
	int data_length = 0;
	int i;
	
	if (extracted_length < 1)
	{
		return error_return(caller, "_decode_packet: bad length!!!");
	}
	
	switch (encoding)
	{
		
	case TWO_DIGITS:
		data_length = 0;
		for(i=0;i<extracted_length-1;i++)   /* fixme: do we really need -1 here? */
		{        
			*extracted_data=((extracted_packet[i] - 0x30)<<6) + extracted_packet[i+1] - 0x30;
			i++;
			data_length++;
			if (data_length > MAX_NUM_POINTS)
			{
				return error_return(caller, "_decode_packet: returned too many data points");
			}
		}
		break;
		
	case THREE_DIGITS:
		data_length = 0;
		for(i=0;i<extracted_length-1;i++)   /* fixme: do we really need -1 here? */
		{        
			*extracted_data=((extracted_packet[i] - 0x30)<<12) + ((extracted_packet[i+1] - 0x30)<<6) + extracted_packet[i+2] - 0x30;
			i+=2;
			extracted_data++;
			data_length++;
			if (data_length > MAX_NUM_POINTS)
			{
				return error_return(caller, "_decode_packet: returned too many data points");
			}
		}
		break;
	default:
		return error_return(caller, "_decode_packet: bad encoding");
	}
	
	return data_length;
}



static int _set_baud_rate(hokuyo_t * h, const char * caller, const int baud_rate)
{
	char request[16];
	char line[MAX_LINE_LENGTH];
	
	char tmp[1000];
	sprintf(tmp, "%s:%s", caller, "_set_baud_rate");
	const char * caller2 = (const char *)tmp;
	
	if (!_is_connected(h, caller2))
	{
		return error_return(caller2, "not connected to the sensor!");
	}
	
	switch (baud_rate) 
	{
	case 19200:
		strcpy(request,"SS019200");
		break;
	case 115200:
		strcpy(request,"SS115200");
		break;
	default:
		return error_return(caller2, "invalid baudrate");
	}
	
	/*  Send the command to Hokuyo */
	
	serial_device_flush_input_buffer(h->sd, caller2);
	
	if (_send_command2(h, caller2, request))
	{
		return error_return(caller2, "unable to send command to hokuyo");
	}
	
	int timeout_us = 100000;
	
	/*  Receive the confirmation that the command was accepted */
	int line_len= _read_line(h, caller2, line, MAX_LINE_LENGTH,timeout_us,false);
	if ( line_len < 0)
	{
		return error_return(caller, "_set_baud_rate: could not read a line");
	}
	
	if (strncmp(request,line,line_len))
	{
		return error_return(caller, "set_baud_rate: response does not match");
	}
	
	/*  Receive the confirmation that the baud rate changed */
	line_len= _read_line(h, caller, line,MAX_LINE_LENGTH,timeout_us,true);
	if (line_len < 0)
	{
		return error_return(caller, "set_baud_rate: could not read a line");
	}
	
	if ( _check_result("00",line) && _check_result("03", line) && _check_result("04",line) )
	{
		return error_return(caller, "set_baud_rate: response does not match");
	}
	
	/*  Set the host terminal baud rate to the test speed */
	if (serial_device_set_baud_rate(h->sd, caller2, baud_rate))
	{
		return error_return(caller, "_set_baud_rate: could not set terminal baud rate");
	}
	
	h->baud = baud_rate;
	
	return 0;   
}


static int _get_sensor_info(hokuyo_t * h, const char * caller, int mode)
{
	char line[MAX_LINE_LENGTH];
	
	char tmp[1000];
	sprintf(tmp, "%s:%s", caller, "_get_sensor_info");
	const char * caller2 = (const char *)tmp;

	if (!_is_connected(h, caller2))
	{
		return error_return(caller2, "not connected to the sensor!");
	}
	
	serial_device_flush_input_buffer(h->sd, caller2);
		
	if (_send_command2(h, caller2, "VV"))
	{
		return error_return(caller2, "could not write request");
	}
	
	int timeout_us = 200000;
	
	/* read the "VV" back without checksum */
	int line_len = _read_line(h, caller2, line,MAX_LINE_LENGTH,timeout_us,false);
	if (line_len < 0)
	{
		return error_return(caller2, "could not read a line");
	}
	
	if (_check_result(line,"VV"))
	{
		return error_return(caller2, "response does not match:");
	}
	
	/* read the "00" status back with the checksum */
	line_len = _read_line(h, caller, line,MAX_LINE_LENGTH,timeout_us,true);
	if (line_len < 0)
	{
		return error_return(caller2, "could not read a line");
	}
	
	if (_check_result(line, "00"))
	{
		return error_return(caller2, "response does not match:");
	}
	
	
	/* Vendor */
	line_len = _read_line(h, caller, line,MAX_LINE_LENGTH,timeout_us,false);
	if (line_len < 0)
	{
		return error_return(caller2, "could not read vendor line");
	}
	
	/* parse the vendor information */
	if (_parse_string(line,  VEND_STR, h->vendor) != 0)
	{
		return error_return(caller2, "could not parse vendor info");
	}
	
	/* Product */
	line_len = _read_line(h, caller2, line,MAX_LINE_LENGTH,timeout_us,false);
	if (line_len < 0)
	{
		return error_return(caller2, "could not read product line");
	}
	
	/* parse the product information */
	if (_parse_string(line,  PROD_STR, h->product) != 0)
	{
		return error_return(caller2, "could not parse product info");
	}
	
	line[line_len-2]=0;  /* last two characters are weird so just ignore them */
	if (strstr(line, TYPE_UTM_30LX_STRING))
	{
		h->type = TYPE_UTM_30LX;
		h->dist_res = 0.001;
		message_on_debug(h->debug, caller2, "sensor identified as HOKUYO TOP-URG UTM_30LX");
	}
	else if (strstr(line, TYPE_URG_04LX_STRING))
	{
		h->type = TYPE_URG_04LX;
		h->dist_res = 0.001;
		message_on_debug(h->debug, caller2, "sensor identified as HOKUYO URG 04LX");
	}
	
	else
	{
		message_on_debug(h->debug, caller2, "<<<WARNING>>> Sensor could not be identified! Seeting sensor type to URG-04LX");
		h->type =  TYPE_URG_04LX;
	}
	
	/* Firmware */
	line_len = _read_line(h, caller2, line,MAX_LINE_LENGTH,timeout_us,false);
	if (line_len < 0)
	{
		return error_return(caller2, "could not read firmware line");
	}
	
	/* parse the firmware information */
	if (_parse_string(line, FIRM_STR, h->firmware) != 0)
	{
		return error_return(caller2, "could not parse firmware info");
	}
	
	/* Protocol */
	line_len = _read_line(h, caller2, line,MAX_LINE_LENGTH,timeout_us,false);
	if (line_len < 0)
	{
		return error_return(caller2, "could not read protocol line");
	}
	
	/* parse the protocol information */
	if (_parse_string(line,  PROT_STR, h->protocol) != 0)
	{
		return error_return(caller2, "could not parse protocol info");
	}
	
	/* Serial */
	line_len = _read_line(h, caller, line,MAX_LINE_LENGTH,timeout_us,false);
	if (line_len < 0)
	{
		return error_return(caller2, "could not read serial number line");
	}
	
	/* parse the serial number information */
	if (_parse_string(line,  SERI_STR, h->serial) != 0)
	{
		return error_return(caller2, "could not parse serial number info");
	}
	
	/*  read off the rest of the info from the buffer */
	usleep(IDLE_USEC); 
	serial_device_flush_input_buffer(h->sd, caller2);
	
	return 0;    
}

static int _get_sensor_params(hokuyo_t * h, const char * caller)
{
	char line[MAX_LINE_LENGTH];
	
	char tmp[1000];
	sprintf(tmp, "%s:%s", caller, "_get_sensor_params");
	const char * caller2 = (const char *)tmp;

	if (!_is_connected(h, caller2))
	{
		return error_return(caller2, "not connected to the sensor!");
	}
	
	serial_device_flush_input_buffer(h->sd, caller2);
		
	/* ******************************************************************* */
	
	if (_send_command2(h, caller2, "PP"))
	{
		return error_return(caller2, "could not write request");
	}
	
	int timeout_us = 200000;
		
	/* read the "PP" back without checksum */
	if ((_read_line(h, caller, line, MAX_LINE_LENGTH,timeout_us,false)) < 0)
	{
		return error_return(caller2, "could not read a line");
	}
		
	if (_check_result("PP", line))
	{
		return error_return(caller2, "response does not match:");
	}
		
	/* read the "00" status back with the checksum */
	if ((_read_line(h, caller, line,MAX_LINE_LENGTH,timeout_us,true)) < 0)
	{
		return error_return(caller2, "could not read a line");
	}
	
	if (_check_result("00", line))
	{
		return error_return(caller, "_get_sensor_params:response does not match:");
	}
	
	/* model */
	if ((_read_line(h, caller, line,MAX_LINE_LENGTH,timeout_us,false)) < 0)
	{
		return error_return(caller2, "could not read model line");
	}
		
	/* parse the model information */
	if (_parse_string(line, MODL_STR, h->model))
	{
		return error_return(caller2, "could not parse model info");
	}
	
	/* minimum distance */
	if ((_read_line(h, caller, line,MAX_LINE_LENGTH,timeout_us,false)) < 0)
	{
		return error_return(caller2, "could not read dmin line");
	}
	
	/* parse the dmin information */
	if (_parse_string(line, DMIN_STR, h->dmin))
	{
		return error_return(caller2, "could not parse dmin info");
	}
	
	
	/* maximum distance */
	if ((_read_line(h, caller, line,MAX_LINE_LENGTH,timeout_us,false)) < 0)
	{
		return error_return(caller2, "could not read dmax line");
	}
	
	/* parse the dmax information */
	if (_parse_string(line, DMAX_STR, h->dmax))
	{
		return error_return(caller2, "could not parse dmax info");
	}
	
	
	/* angular resolution */
	if ((_read_line(h, caller, line,MAX_LINE_LENGTH,timeout_us,false)) < 0)
	{
		return error_return(caller2, "could not read ares line");
	}
	
	/* parse the ares information */
	if (_parse_string(line, ARES_STR, h->ares))
	{
		return error_return(caller2, "could not parse ares info");
	}
	
	
	/* minimum angle count */
	if ((_read_line(h, caller, line,MAX_LINE_LENGTH,timeout_us,false)) < 0)
	{
		return error_return(caller2, "could not read amin line");
	}
	
	/* parse the amin information */
	if (_parse_string(line,  AMIN_STR, h->amin))
	{
		return error_return(caller2, "could not parse amin info");
	}
	
	/* max angle count */
	if ((_read_line(h, caller, line,MAX_LINE_LENGTH,timeout_us,false)) < 0)
	{
		return error_return(caller2, "could not read amax line");
	}
	
	/* parse the amax information */
	if (_parse_string(line,  AMAX_STR, h->amax))
	{
		return error_return(caller2, "could not parse amax info");
	}
	
	/* front angle count */
	if ((_read_line(h, caller, line,MAX_LINE_LENGTH,timeout_us,false)) < 0)
	{
		return error_return(caller2, "could not read afrt line");
	}
	
	/* parse the amax information */
	if (_parse_string(line, AFRT_STR, h->afrt))
	{
		return error_return(caller2, "could not parse afrt info");
	}
	
	
	/* scan rate */
	if ((_read_line(h, caller, line,MAX_LINE_LENGTH,timeout_us,false)) < 0)
	{
		return error_return(caller2, "could not read scan line");
	}
	
	/* parse the amax information */
	if (_parse_string(line, SCAN_STR, h->scan))
	{
		return error_return(caller2, "could not parse _scan info");
	}
	
	
	h->dist_min = (int)strtol(h->dmin,NULL,10) / 1000.0;  /* convert to meters */
	h->dist_max = (int)strtol(h->dmax,NULL,10) / 1000.0;  
	h->angle_res = 2 * M_PI / (int)strtol(h->ares,NULL,10);  /* convert from total counts to actual resolution */
	
	/* error checking */
	if (h->dist_min <= 0)
	{
		return error_return(caller2, "bad dmin");
	}
	
	if (h->dist_max < h->dist_min)
	{
		return error_return(caller2, "bad dmax");
	}
	
	if (h->angle_res <= 0)
	{
		return error_return(caller2, "bad ares");
	}
	
	h->count_min = (int)strtol(h->amin,NULL,10);
	h->count_max = (int)strtol(h->amax,NULL,10);
	h->count_zero = (int)strtol(h->afrt,NULL,10);
	
	if (h->count_min > h->count_zero)
	{
		return error_return(caller2, "bad amin");
	}
	
	if (h->count_max < h->count_zero)
	{
		return error_return(caller2, "bad amax");
	}
	h->angle_min = ( h->count_min - h->count_zero) * h->angle_res;
	h->angle_max = ( h->count_max - h->count_zero) * h->angle_res;
	h->scan_rate = strtol(h->scan,NULL,10) / 60.0;   /* Hz */
	
	if (h->scan_rate < 0)
	{
		return error_return(caller2, "bad scan rate");
	}
	
	/*  read off the rest of the info from the buffer */
	usleep(IDLE_USEC);
	serial_device_flush_input_buffer(h->sd, caller2);
	
	return 0;    
}


static int _get_sensor_info_and_params(hokuyo_t * h, const char * caller)
{
	char tmp[1000];
	sprintf(tmp, "%s:%s", caller, "_get_sensor_info_and_params");
	const char * caller2 = (const char *)tmp;

	if (!_is_connected(h, caller2))
	{
		return error_return(caller2, "not connected to the sensor!");
	}
	
	if  (_get_sensor_info(h, caller, h->mode)) 
	{
		return error_return(caller2, "unable to get status!");
	}
	
	if  (_get_sensor_params(h, caller)) 
	{
		return error_return(caller2, "unable to get sensor params!");
	}
	
	return 0;
}



static void _lock_data_mutex(void * v)
{
	hokuyo_t * h = (hokuyo_t *)v;
	
	pthread_mutex_lock(&h->data_mutex);
}

static void _unlock_data_mutex(void * v)
{
	hokuyo_t * h = (hokuyo_t *)v;
	
	pthread_mutex_unlock(&h->data_mutex);
}
static void _data_cond_signal(void * v)
{
	hokuyo_t * h = (hokuyo_t *)v;
	
	pthread_cond_signal(&h->data_cond);
}

static int data__cond_wait(void * v, int timeout_ms)
{
	hokuyo_t * h = (hokuyo_t *)v;
	
	struct timeval time_start;
	struct timespec max_wait_time;
	gettimeofday(&time_start,NULL);
	
	max_wait_time.tv_sec=time_start.tv_sec + ((time_start.tv_usec < 1000000-timeout_ms*1000) ? 0:1);
	max_wait_time.tv_nsec=(time_start.tv_usec*1000 + timeout_ms*1000000)%1000000000;
	return pthread_cond_timedwait(&h->data_cond,&(h->data_mutex),&max_wait_time);
}


/* do we have a fresh scan? this is used by hokuyo_reader class */
bool _is_scan_ready(void * v)
{
	hokuyo_t * h = (hokuyo_t *)v;
	
	return h->scan_ready;
}


/* initializes the hokuyo */
static int _connect(void * v, const char * caller,  char * device, const int baud_rate, const int mode)
{	
	char tmp[1000];
	sprintf(tmp, "%s:%s", caller, "_connect");
	const char * caller2 = (const char *)tmp;
	
	hokuyo_t * h = (hokuyo_t *)v;

	strcpy(h->device, device);
	
	message_on_debug(h->debug, caller2, "attempting to connect to device %s", device);

	/* connect to the device */
	if (serial_device_connect(h->sd, caller2, device, baud_rate)) 
	{
		return error_return(caller2, "failed");        
	}
	
	message_on_debug(h->debug, caller2, "connected");
	
	/* find out what rate the sensor is currently running at. _baud will be set to the current baud rate. */	
	
	if (!_test_baud_rate(h, caller2,  115200)) 
	{
		message_on_debug(h->debug, caller2, "Hokuyo baud rate is 115200bps...");
	}
	else if (!_test_baud_rate(h, caller2,  19200))
	{
		message_on_debug(h->debug, caller2, "Hokuyo baud rate is 19200bps...");
	}
	else if (!_test_baud_rate(h, caller2,  38400))
	{
		message_on_debug(h->debug, caller2, "Hokuyo baud rate is 38400bps...");
	}
	else 
	{
		serial_device_disconnect(h->sd, caller2);
		
		return error_return(caller, "_connect:failed to detect baud rate!\n"
			"make sure that the sensor is upgraded to SCIP 2.0\n"
			"the sensor does not have start in SCIP2.0 but must support it");        
	}
	
	fflush(stdout);
	
	/* Make sure requested baud isn't already set */
	if (h->baud == baud_rate) 
	{
		message_on_debug(h->debug, caller2, "Hokuyo is already operating at that baud rate");
	}
	
	/* need to set the baud rate */
	else 
	{
		message_on_debug(h->debug, caller2, "attempting to set requested baud rate...");
		
		if (!_set_baud_rate(h, caller, baud_rate))
		{
			/* success */
			message_on_debug(h->debug, caller2, "baud rate set");
		} 
		
		/* could not set the baud rate */
		else 
		{
			serial_device_disconnect(h->sd, caller2);
			return error_return(caller2, "could not set the requested Hokuyo baud rate");
		}
	}
	
	/*  get the status data from sensor to determine its type */
	if (_get_sensor_info_and_params(h, caller2))
	{
		serial_device_disconnect(h->sd, caller2);
		return error_return(caller2, "could not get status data from sensor");
	}
	
	message_on_debug(h->debug, caller2, "turning the laser on");
	
	if (_laser_on(h, caller))
	{
		serial_device_disconnect(h->sd, caller2);
		return error_return(caller2, "unable to turn the laser on");
	}
	
	message_on_debug(h->debug, caller2, "initialization complete");
	
	return 0;
}


/* disconnect from hokuyo */
static int _disconnect(void * v, const char * caller) 
{
	char tmp[1000];
	sprintf(tmp, "%s:%s", caller, "_disconnect");
	const char * caller2 = (const char *)tmp;
	
	hokuyo_t * h = (hokuyo_t *)v;
	
	if (!_is_connected(h, caller2))
	{
		return 0;
	}
	
	if (!_laser_off(v, caller))
	{
		
		message_on_debug(h->debug, caller2, "laser has been shut off");
	}
	
	else 
	{
		message_on_debug(h->debug, caller2, "not able to shut off the laser");
	}
	
	serial_device_disconnect(h->sd, caller2);
	
	return 0;
}


/*confirm that the scan request was accepted */
static int _confirm_scan(void * v, const char * caller,  char * req, int timeout_us)
{
	hokuyo_t * h = (hokuyo_t *)v;
	
	char line[MAX_LINE_LENGTH];
	int line_len;    
	
	/* read back the echo of the command */
	line_len= _read_line(h, caller, line, MAX_LINE_LENGTH, timeout_us, false);
	if (line_len < 0)
	{
		error_return(caller, "_confirm_scan: could not read echo of the command (line 1)");
	}
	
	if (strncmp(req,line,line_len))
	{
		return error_return(caller, "_confirm_scan: response does not match the echo of the command (line 1)");
	}
	
	/* read back the status response */
	line_len = _read_line(h, caller, line,MAX_LINE_LENGTH,timeout_us,true);
	if (line_len < 0)
	{
		return error_return(caller, "_confirm_scan: could not read status response(line 2)");
	}
	
	if (_check_result("00", line))
	{
		message_on_debug(h->debug, caller, "_confirm_scan: status response does not match (line 2):");
		
		if (!_check_result("10",line))
		{
			message_on_debug(h->debug, caller, "_confirm_scan: (response from scanner)it looks like we need to turn the laser back on");
			_laser_on(h, caller);
		}
		
		else if (!_check_result("01", line))
		{
			message_on_debug(h->debug, caller, "_confirm_scan: (response from scanner) Starting step has non-numeric value");
		}
		
		else if (_check_result("02",line))
		{
			message_on_debug(h->debug, caller, "_confirm_scan: (response from scanner) End step has non-numeric value");
		}
		
		else if (!_check_result("03",line))
		{
			message_on_debug(h->debug, caller, "_confirm_scan: (response from scanner) Cluster count (skip) has non-numeric value");
		}
		
		else if (!_check_result("04",line))
		{
			message_on_debug(h->debug, caller, "_confirm_scan: (response from scanner) End step is out of range");
		}
		
		else if (!_check_result("05",line))
		{
			message_on_debug(h->debug, caller, "_confirm_scan: (response from scanner) End step is smaller than starting step");
		}
		
		else 
		{
			message_on_debug(h->debug, caller, "_confirm_scan: (response from scanner) Looks like hardware trouble");
		}
		
		return -1;
	}
	
	return 0;
}


/* get a scan from hokuyo:proide a pointer to allocated buffer, reference 
var with number of points (void * v, const char * caller,  ranges) and scan parameters */
int _get_scan(void * v, const char * caller,  unsigned int * range, int * n_range, int scan_start, 
	int scan_end, int scan_skip, int encoding, int scan_type,  int num_scans)
{
	char tmp[1000];
	sprintf(tmp, "%s:%s", caller, "_get_scan");
	const char * caller2 = (const char *)tmp;
	
	hokuyo_t * h = (hokuyo_t *)v;
	
	char req[128];
	char good_status[2];
	char line[MAX_LINE_LENGTH];
	char full_packet[MAX_PACKET_LENGTH];
	char extracted_packet[MAX_PACKET_LENGTH];
	int line_len;
	bool need_to_request_scan = false;
	
	if (!_is_connected(h, caller2))
	{
		return error_return(caller2, " not connected to the sensor!");
	}
	
	
	/* settings have changed, therefore we need to figure out the packet length */
	if (
		(h->scan_start != scan_start) || 
		(h->scan_end != scan_end) || 
		(h->scan_skip != scan_skip) || 
		(h->encoding != encoding) || 
		(h->scan_type!=scan_type))
	{
		h->packet_length = -1;
		h->scan_ready = false;
		
		if (h->streaming)
		{
			if (!_laser_off(h, caller))
			{
				message_on_debug(h->debug, caller2, " laser has been shut off");
				usleep(IDLE_USEC);
			}
			
			else
			{
				message_on_debug(h->debug, caller2, " could not shut off the laser");
			}
		}

    		
		message_on_debug(h->debug, caller2, "need to check packet size");
	}
	
	else
	{
		message_on_debug(h->debug, caller2, "reusing old packet size");
	}
	
	memcpy(good_status,"99",2*sizeof(char));
	bool need_to_read_off_cmd_and_status = false;
	if (_create_scan_request(h, caller,
		scan_start, scan_end, scan_skip, 
		encoding, scan_type, num_scans, 
		req, &need_to_read_off_cmd_and_status))
	{
		return error_return(caller2, " could not create scan request string!");
	}
	
	if (num_scans == 1)
	{
		need_to_request_scan = true;
	}
	else if (num_scans == 0)
	{
		if (!h->streaming)
		{
			need_to_request_scan=true;
		}
		
		else 
		{
			need_to_request_scan=false;
		}
	}
	
	else 
	{
		return error_return(caller2, " invalid number of scans");
	}
	
	
	int timeout_us = 0;
	switch (h->type)
	{
	case TYPE_URG_04LX:
		timeout_us = URG_04LX_GET_SCAN_TIMEOUT_US;
		break;
		
	case TYPE_UTM_30LX:
		timeout_us = UTM_30LX_GET_SCAN_TIMEOUT_US;
		break;
	}
	
	/* only send the request command if we need to */
	/* this means only when we request a single scan or to start streaming */
	if (need_to_request_scan)
	{
	    
	    serial_device_flush_input_buffer(h->sd, caller);
	    
	    /* write the data request command */
	    if (_send_command2(h, caller, req))
	    {
	        return error_return(caller2, "could not write command");
	    }
	    		
		if (_confirm_scan(v, caller, req, timeout_us))
		{
			return error_return(caller2, " could not confirm scan!" );
		}
		
		/* the read was successful, so it means that if we requested streaming, we got it */
		if (num_scans == 0)
		{
			h->streaming = true;
		}
		
		if ((num_scans==0) || (scan_type==SCAN_SPECIAL_ME))
		{
			/* read off the LF character */
			line_len = _read_line(h, caller, line, MAX_LINE_LENGTH, timeout_us, false);
			if (line_len < 0)
			{
				return error_return(caller2, " could not read LF (line 3) ");
			}
			
			if (line_len > 0)
			{
				return error_return(caller2, " response does not match LF" );
			}
		}
	}
	
	if (need_to_read_off_cmd_and_status)
	{
		/* read back the echo of the command */
		line_len=_read_line(h, caller, line,MAX_LINE_LENGTH, timeout_us, false);
		if (line_len < 0)
		{
			return error_return(caller2, " could not read echo of the command (line @1)" );
		}
		
		if (strncmp(req,line,line_len-2))     /* XXX: not checking the number of remaining scans */
		{
			return error_return(caller2, " echo of the command does not match (line @1):");
		}
		
		/* read back the status response */
		line_len=_read_line(h, caller, line,MAX_LINE_LENGTH,timeout_us,true);
		if (line_len < 0)
		{
			return error_return(caller2, " could not read line @2" );
		}
		
		if (strncmp(good_status,line,2))
		{
			return error_return(caller2, " response does not match (line @2):");
		}
	}
	
	
	/* read the timestamp */
	line_len=_read_line(h, caller, line,MAX_LINE_LENGTH,timeout_us,true);
	if (line_len < 0)
	{
		return error_return(caller2, " could not read timestamp (line @3)" );
	}
	
	/* read the whole packet */
	if (_read_packet(h, caller, full_packet, &h->packet_length, timeout_us))
	{
		return error_return(caller2, " could not read data packet" );
	}
	
	/* verify the checksum, and remove LF and checksum chars from data */
	int extracted_length = _extract_packet(h, caller, full_packet, extracted_packet, h->packet_length);
	if (extracted_length < 0)
	{
		return error_return(caller2, " could not extract data from packet" );
	}
	
	/* decode the packet into the provided buffer */
	_lock_data_mutex(h);
	*n_range = _decode_packet(h, caller, extracted_packet, range, extracted_length, encoding);
	if ( (*n_range < 0) || (*n_range>MAX_NUM_POINTS))
	{
		h->scan_ready=false;    
		_unlock_data_mutex(h);        
		return error_return(caller2, " returned too many data points" );
		/* if (scan_ready != NULL) *scan_ready=false; */
	}
	
	/* if (scan_ready != NULL) *scan_ready=true; */
	h->scan_ready = true;
	
	/* signal that the newest data is available */
	_data_cond_signal(h);
	_unlock_data_mutex(h);
	
	/* update the scan params because the scan was successful */
	h->scan_start = scan_start;
	h->scan_end = scan_end;
	h->scan_skip = scan_skip;
	h->encoding = encoding;
	h->scan_type = scan_type;
	
	/* the read was successful, so it means that if we requested streaming, we got it */
	if (num_scans==0)
	{
		h->streaming = true;
	}
	
	return 0;
}



/* find the start of packet in case of loss of synchronization */
static int _find_packet_start(void * v, const char * caller) 
{
	char tmp[1000];
	sprintf(tmp, "%s:%s", caller, "_find_packet_start");
	const char * caller2 = (const char *)tmp;
	
	hokuyo_t * h = (hokuyo_t *)v;
	
	static char data[MAX_PACKET_LENGTH];
	char seq[2]={0x0A,0x0A};   /* two LFs */
	
	if (_is_connected(h, caller2))
	{
		return error_return(caller2, "not connected to the sensor!");
	}
	
	/* set the io mode to return when the termination sequence is read
	then we know that we've reached the end of some packet */ 
	serial_device_set_io_block_w_timeout_w_term_sequence(h->sd, caller2, seq, 2, true);
	
	/* read until the end sequence is found or until read too much */
	int num_chars = serial_device_read_chars(h->sd, caller2, data, MAX_PACKET_LENGTH, 0);
	
	if (num_chars < 1 )
	{
		return error_return(caller2, "terminating character was not read");
	}
	
	return 0;
}


static void _set_scan_unready(void * v)
{
	hokuyo_t * h = (hokuyo_t *)v;
	
	h->scan_ready = false;
}


static int _cond_wait(hokuyo_t * hokuyo, int timeout_ms)
{ 
	struct timeval time_start;
	struct timespec max_wait_time;
	gettimeofday(&time_start,NULL);
	
	max_wait_time.tv_sec = time_start.tv_sec + ((time_start.tv_usec < 1000000-timeout_ms*1000) ? 0:1);
	max_wait_time.tv_nsec =(time_start.tv_usec*1000 + timeout_ms*1000000)%1000000000;
	return pthread_cond_timedwait(&hokuyo->settings_cond,&(hokuyo->settings_mutex),&max_wait_time);
}


static double _get_current_time_sec(void)
{
	struct timeval time;
	gettimeofday(&time, NULL);
	return  (time.tv_sec + time.tv_usec*0.000001); 
}

static void * _run(void * v)
{
	hokuyo_t * h = (hokuyo_t *)v;
	
	sigset_t sigs;
	sigfillset(&sigs);
	pthread_sigmask(SIG_BLOCK,&sigs,NULL);
	struct timeval timeStart;
	struct timeval timeStop;
	struct timeval totalTimeStart;
	struct timeval totalTimeStop;
	unsigned int cntr=0;
	
	bool connected, active;
	
	int numErrors=0;
	
	gettimeofday(&totalTimeStart,NULL);
	gettimeofday(&totalTimeStop,NULL);
	
	while(1)
	{
		cntr++;
		gettimeofday(&timeStart,NULL);
		pthread_testcancel();
		
		/*  create a local copy of the scan params to be used later */
		int scan_start   = SCAN_START;
		int scan_end     = SCAN_END;
		int scan_skip    = SCAN_SKIP;
		int encoding     = ENCODING;
		int scan_type    = SCAN_TYPE;
		
		connected        = _is_connected(h, "_run");
		active           = h->active;
		
		if (h->need_to_stop_laser)
		{
			if (!_laser_off(h, (char *)"_run"))
			{
				message_on_debug(h->debug, "_run", "Laser has been shut off");
				h->need_to_stop_laser=false;
			}
			
			else 
			{
				message_on_debug(h->debug, "_run", "Unable to shut off the laser");
			}
			
			/*  XXX: count number of times errors occur */
		}
		
		
		pthread_cond_signal(&h->settings_cond);
		
		if (connected && active)
		{
			int error = _get_scan(
                                h, 
				(char *)"_run", 
				h->data, &h->num_data, 
				scan_start, scan_end, 
				scan_skip, 
				encoding, 
				scan_type, 
				0);
			
			if (!error)
			{
				numErrors=0;
			} 
			
			else 
			{
				numErrors++;
				message_on_debug(h->debug, "_run", "Could not read a scan from the sensor"); 
				if (!_find_packet_start(h, (char *)"_run"))
				{
					message_on_debug(h->debug, "_run", "Resynchronized with the stream"); 
				}
				
				if (numErrors>=READER_MAX_NUM_ERRORS_BEFORE_RESTART)
				{
					h->need_to_stop_laser=true;
					numErrors=0;
				}
				
				/*  XXX: need to do some error handling here */
				/* pthread_exit(NULL); */
			}
		}
		
		else 
		{
			/* just idle */
			usleep(IDLE_USEC);
		}
		
		gettimeofday(&timeStop,NULL);
		
	}
	
	return NULL;
	
} /*  _run */



/* Exported functions ======================================================= */


void * hokuyo_create(const char * caller, bool debug)
{
	char tmp[1000];
	sprintf(tmp, "%s:%s", caller, "hokuyo_create");
	const char * caller2 = (const char *)tmp;
	
	hokuyo_t * h = (hokuyo_t *)malloc(sizeof(hokuyo_t));
	
	h->type = TYPE_URG_04LX;
	h->baud = 0;
	h->com_protocol = SERIAL;
	strcpy(h->device, "Unknown");
	h->mode = SCIP20;
	h->streaming = false;
	h->debug = debug;
	
	/* initialize vars to invalid values */
	h->scan_start      = -1;
	h->scan_end        = -1;
	h->scan_skip       = -1;
	h->encoding        = -1;
	h->scan_type       = -1;
	h->packet_length   = -1;
	h->dist_min        = -1;
	h->dist_max        = -1;
	h->dist_res        = -1;
	h->angle_res       = -1;
	h->angle_min       =  0;
	h->angle_max       =  0;
	h->scan_rate       = -1;
	h->count_min       = -1;
	h->count_max       = -1;
	h->count_zero      = -1;
	
	/* initialize mutexes */
	pthread_mutex_init(&h->data_mutex, NULL);
	pthread_cond_init(&h->data_cond, NULL);    
	
	/* create serial device interface */
	h->sd = serial_device_create(caller2);
	
	_set_scan_unready(h);
	
	h->active = false;
	h->need_to_stop_laser = false;
	
	h->thread_is_running = false;
	
	pthread_mutex_init(&h->settings_mutex,NULL);
	pthread_cond_init(&h->settings_cond,NULL);
	
	h->num_data =0;	
	
	return h;
}
		
int hokuyo_connect(void * v, const char * caller, char * device, int baud_rate)
{
	char tmp[1000];
	sprintf(tmp, "%s:%s", caller, "hokuyo_connect");
	const char * caller2 = (const char *)tmp;

	hokuyo_t * h = (hokuyo_t *)v;
	

	if (_connect(h, caller2, device, baud_rate, SCIP20))
	{
		return -1;
	}
	
	
	/*  start the thread */
	if (pthread_create(&h->thread, NULL, _run, (void *)h)) 
	{
		return -1;
	}
	
	h->thread_is_running = true;
	
	/*  copy the new values */
	h->need_to_stop_laser=true;
	
	h->active = true;
	
	double timeStartSec = _get_current_time_sec();
	
	/* this loop makes sure that timedwait does not return before the time expires - recommended in manual for this function.. */
	while (true)
	{        
		if ((_get_current_time_sec() - timeStartSec)*1000 >= READER_SET_SCAN_PARAMS_TIMEOUT_MSEC)
		{
			break;
		}
		
		if (_cond_wait(h, READER_SET_SCAN_PARAMS_TIMEOUT_MSEC) == ETIMEDOUT)
		{
			message_on_debug(h->debug, caller2, "timed out");
		}
		
		usleep(TIMED_WAIT_USEC);
	}
	
	return 0;
}

void hokuyo_destroy(void * v, const char * caller)
{    
	hokuyo_t * h = (hokuyo_t *)v;

	if (h->thread_is_running)
	{
		message_on_debug(h->debug, caller, "hokuyo_destroy: stopping thread...");
		pthread_cancel(h->thread);
		pthread_join(h->thread,NULL);
		h->thread_is_running=false;
	}
	
	if (_is_connected(h, caller))
	{
		message_on_debug(h->debug, caller, "hokuyo_destroy: disconnecting from device..."); 
		_disconnect(h, (char *)caller);		
	}
	
	pthread_mutex_destroy(&h->settings_mutex);
	pthread_cond_destroy(&h->settings_cond);
}



int hokuyo_get_scan(void * v, const char * caller, unsigned int * range)
{
	hokuyo_t * h = (hokuyo_t *)v;
	
	if (!_is_connected(h, caller))
	{
	    return 0;
	}
	
	int n_range = 0;
	
	double timeStartSec = _get_current_time_sec();
	
	_lock_data_mutex(h);
	
	while (!_is_scan_ready(h))
	{         		    
		if ((_get_current_time_sec() - timeStartSec)*1000 >= READER_GET_SCAN_TIMEOUT_MSEC)
		{
			break;
		}
		
		else if (!_is_scan_ready(h))
		{
			usleep(TIMED_WAIT_USEC);
		}
		
		int ret = data__cond_wait(h, READER_GET_SCAN_TIMEOUT_MSEC);
		
		if (ret==ETIMEDOUT)
		{
			message_on_debug(h->debug, caller, "hokuyo_get_scan: timed out");
		}
	}	
	
	/* need to verify that the scan is actually ready */
	if (_is_scan_ready(h))
	{
		/* new scan is ready! */
		memcpy(range, h->data, h->num_data*sizeof(unsigned int));
		n_range = h->num_data;
		_set_scan_unready(h);	    
	} 
	else 
	{	    
		message_on_debug(h->debug, caller, "hokuyo_get_scan: could not get a scan from the sensor");
	}
	
	_unlock_data_mutex(h);
	
	return n_range;
}

void hokuyo_get_str(void * v, const char * caller, char * s)
{    
	char tmp[1000];
	sprintf(tmp, "%s:%s", caller, "hokuyo_get_str");
	const char * caller2 = (const char *)tmp;
	
	hokuyo_t * h = (hokuyo_t *)v;
	
	if (_is_connected(h, caller2))
	{
	    sprintf(s, "Model:    %s\nFirmware: %s\nSerial #: %s\nProtocol: %s\nVendor:   %s", 
	        h->model, h->firmware, h->serial, h->protocol, h->vendor);
	}
	else
	{
	    sprintf(s, "not connected");
	}
	
}




double hokuyo_get_dist_min(void * v)
{
	hokuyo_t * h = (hokuyo_t *)v;
	
	return h->dist_min;
}

double hokuyo_get_dist_max(void * v)
{
	hokuyo_t * h = (hokuyo_t *)v;
	
	return h->dist_max;
}

double hokuyo_get_angle_res(void * v)
{
	hokuyo_t * h = (hokuyo_t *)v;
	
	return h->angle_res;
}

double hokuyo_get_angle_min(void * v)
{
	hokuyo_t * h = (hokuyo_t *)v;
	
	return h->angle_min;
}

double hokuyo_get_angle_max(void * v)
{
	hokuyo_t * h = (hokuyo_t *)v;
	
	return h->angle_max;
}

double hokuyo_get_scan_rate(void * v)
{
	hokuyo_t * h = (hokuyo_t *)v;
	
	return h->scan_rate;
}

double hokuyo_get_dist_res(void * v)
{
	hokuyo_t * h = (hokuyo_t *)v;
	
	return h->dist_res;
}

int hokuyo_get_sensor_type(void * v)
{
	hokuyo_t * h = (hokuyo_t *)v;
	
	return h->type;
}

int hokuyo_get_count_zero(void * v)
{
	hokuyo_t * h = (hokuyo_t *)v;
	
	return h->count_zero;
}

int hokuyo_get_count_min(void * v)
{
	hokuyo_t * h = (hokuyo_t *)v;
	
	return h->count_min;
}

int hokuyo_get_count_max(void * v)
{
	hokuyo_t * h = (hokuyo_t *)v;
	
	return h->count_max;
}

int hokuyo_get_type(void * v)
{
	hokuyo_t * h = (hokuyo_t *)v;
	
	return h->type;
}






