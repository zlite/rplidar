/*

hokuyo.h Constant and function declarations for Hokuyo classes in BreezyLidar

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

/* actually 769 is the max number of points for range only scan for URG-04LX,  */
/* but when range+intensity+agc is requested, it goes up to 771 */
static const int MAX_NUM_POINTS_URG_04LX    = 771;

static const int MAX_NUM_POINTS_UTM_30LX    = 1081*2;

/* Unused for USB */
static const int DEFAULT_BAUD_RATE          = 115200;

#include <stdbool.h>

/* These functions are used by BreezyLidar */
void * hokuyo_create(const char * caller, bool debug);
int    hokuyo_connect(void * v, const char * caller, char * device, int baud_rate);
void   hokuyo_destroy(void * v, const char * caller);
int    hokuyo_get_scan(void * v, const char * caller, unsigned int * range);
void   hokuyo_get_str(void * v, const char * caller, char * s);

/* These functions are available but unused */
double hokuyo_get_angle_max(void * v);
double hokuyo_get_angle_min(void * v);
double hokuyo_get_angle_res(void * v);
double hokuyo_get_dist_max(void * v);
double hokuyo_get_dist_min(void * v);
double hokuyo_get_dist_res(void * v);
double hokuyo_get_scan_rate(void * v);
int    hokuyo_get_count_max(void * v);
int    hokuyo_get_count_min(void * v);
int    hokuyo_get_count_zero(void * v);
int    hokuyo_get_sensor_type(void * v);
int    hokuyo_get_type(void * v);
