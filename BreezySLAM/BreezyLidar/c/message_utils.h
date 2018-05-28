/*
message_utils.h : messge utilities for C extensions in Python, Matlab, C++

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
#include <stdarg.h>

static void message_on_debug(bool debug, const char * caller, const char * fmt, ...)
{
    if (debug)
    {
        char formatlf[1000];
        sprintf(formatlf, "%s: %s\n", caller, fmt);
        
        va_list argptr;
        va_start(argptr, fmt);
        vfprintf(stderr, formatlf, argptr);
        va_end(argptr);
    }
}

static int error_return(const char * caller, const char * fmt, ...)
{
    char formatlf[1000];
    sprintf(formatlf, "%s: %s\n", caller, fmt);
    
    va_list argptr;
    va_start(argptr, fmt);
    vfprintf(stderr, formatlf, argptr);
    va_end(argptr);
    
    return -1;
}
