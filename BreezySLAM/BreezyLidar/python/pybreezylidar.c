/*
pybreezylidar.c : C extensions for BreezyLidar Python

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

#include <Python.h>
#include <structmember.h>

#include "pyextension_utils.h"
#include "../c/hokuyo.h"


// URG04LX class  ---------------------------------------------------------------

typedef struct 
{
    PyObject_HEAD
    
    void * hokuyo;
    
    unsigned int * range;
    
} URG04LX;

static void URG04LX_dealloc(URG04LX* self)
{                
    hokuyo_destroy(self->hokuyo, "breezylidar:~URG04LX");
    
    free(self->range);
    
    Py_TYPE(self)->tp_free((PyObject*)self);
}

static PyObject * URG04LX_new(PyTypeObject *type, PyObject *args, PyObject *kw)
{                    
    URG04LX *self;
    
    self = (URG04LX *)type->tp_alloc(type, 0);
    
    return (PyObject *)self;
}

static int URG04LX_init(URG04LX *self, PyObject *args, PyObject *kw)
{       
    char * devname;
    int baudrate = DEFAULT_BAUD_RATE;
    int debug = 0;
	
    
    const char * keywords[] = {"device", "baudrate", "debug", NULL};
    
    if (!PyArg_ParseTupleAndKeywords(args, kw, "s|ii", (char **)keywords, &devname, &baudrate, &debug))
    {
        return error_on_raise_argument_exception("URG04LX.__init__");
    }
    
    self->hokuyo = hokuyo_create("breezylidar.URG04LX", debug);
    
    if (hokuyo_connect(self->hokuyo, "breezylidar.URG04LX", devname, baudrate))
    {
        return error_on_raise_argument_exception("URG04LX");
    }

    self->range = (unsigned int *)malloc(MAX_NUM_POINTS_URG_04LX*sizeof(unsigned int));
   
    return 0;    
}


static PyObject * URG04LX_str(URG04LX *self)
{        
    char str[1000];
    
    hokuyo_get_str(self->hokuyo, "URG04LX_str", str);
    
    return  PyUnicode_FromString(str);
}

static PyObject * URG04LX_getScan(URG04LX *self)
{                
    int nrange = hokuyo_get_scan(self->hokuyo, "URG04LX.getScan", self->range);
    
    PyObject * rangelist = PyList_New(nrange);
        
    int k;
    for (k=0; k<nrange; ++k)
    {
        PyList_SetItem(rangelist, k, PyLong_FromLong(self->range[k]));
    }
    
    return rangelist;
}



static PyMethodDef URG04LX_methods[] = 
{
    
    {"getScan", (PyCFunction)URG04LX_getScan, METH_VARARGS, 
        "URG04LX.getScan() returns scan values"
    },
   
    {NULL}  // Sentinel 
};


static PyMemberDef URG04LX_members[] = {
    /*
    {"XXX", T_DOUBLE, offsetof(URG04LX, XXX), 0, "COMMENT"},
    */
    {NULL}  /* Sentinel */
};

#define TP_DOC_URG04LX \
"A class for reading from Hokuyo URG-04LX Lidar units.\n" \
"URG04LX.__init__(device, baud_rate=115200, debug=false)"

static PyTypeObject pybreezylidar_URG04LXType = 
{
    #if PY_MAJOR_VERSION < 3
    PyObject_HEAD_INIT(NULL)
    0,                                          // ob_size
    #else
    PyVarObject_HEAD_INIT(NULL, 0)
    #endif
    "breezylidar.URG04LX",                  // tp_name
    sizeof(URG04LX),                           // tp_basicsize
    0,                                          // tp_itemsize
    (destructor)URG04LX_dealloc,               // tp_dealloc
    0,                                          // tp_print
    0,                                          // tp_getattr
    0,                                          // tp_setattr
    0,                                          // tp_compare
    (reprfunc)URG04LX_str,                     // tp_repr
    0,                                          // tp_as_number
    0,                                          // tp_as_sequence
    0,                                          // tp_as_URG04LXping
    0,                                          // tp_hash 
    0,                                          // tp_call
    (reprfunc)URG04LX_str,                     // tp_str
    0,                                          // tp_getattro
    0,                                          // tp_setattro
    0,                                          // tp_as_buffer
    Py_TPFLAGS_DEFAULT | Py_TPFLAGS_BASETYPE,   // tp_flags
    TP_DOC_URG04LX,                            // tp_doc 
    0,                                          // tp_traverse 
    0,                                          // tp_clear 
    0,                                          // tp_richcompare 
    0,                                          // tp_weaklistoffset 
    0,                                          // tp_iter 
    0,                                          // tp_iternext 
    URG04LX_methods,         					// tp_methods 
    URG04LX_members,          					// tp_members 
    0,                                          // tp_getset 
    0,                                          // tp_base 
    0,                                          // tp_dict 
    0,                                          // tp_descr_get 
    0,                                          // tp_descr_set 
    0,                                          // tp_dictoffset 
    (initproc)URG04LX_init,                    // tp_init 
    0,                                          // tp_alloc 
    URG04LX_new,                               // tp_new 
};


// breezylidar module ------------------------------------------------------------


static PyMethodDef module_methods[] = 
{
    {NULL, NULL, 0, NULL}        /* Sentinel */
};

static void add_classes(PyObject * module)
{
    add_class(module, &pybreezylidar_URG04LXType, "URG04LX");
}

static int types_are_ready(void)
{
    return  type_is_ready(&pybreezylidar_URG04LXType);
}

#if PY_MAJOR_VERSION < 3

PyMODINIT_FUNC
initpybreezylidar(void) 
{    
    if (!types_are_ready())    {
        return;
    }
    
    PyObject * module = Py_InitModule("pybreezylidar", module_methods);
    
    if (module == NULL)
    {
        return;
    }
    
    add_classes(module);    
}

#else

static PyModuleDef moduledef = 
{
    PyModuleDef_HEAD_INIT,
    "pybreezylidar",
    "BreezyLidar module",  
    -1,                 // m_size
    module_methods,     
    NULL,               // m_reload
    NULL,               // m_traverse
    NULL,               // m_clear
    NULL                // m_free
};

PyMODINIT_FUNC
PyInit_pybreezylidar(void) 
{    
    if (!types_are_ready())
    {
        return NULL;
    }
    
    PyObject* module = PyModule_Create(&moduledef);
    
    if (module == NULL)
    {
        return NULL;
    }
    
    add_classes(module);
    
    return module;
}

#endif

