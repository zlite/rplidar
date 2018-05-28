BreezyLidar
===========

BreezyLidar - Simple, efficient, Lidar access for Linux computers in Python and C++

This repository contains everything you need to start working with the popular 
<a href="http://www.robotshop.com/en/hokuyo-urg-04lx-ug01-scanning-laser-rangefinder.html">
Hokuyo URG-04LX</a> Lidar unit on your Linux computer.  It is designed for
robotics applications on a single-board Linux computer like RaspberryPi or ODROID.

<p>
BreezyLidar was inspired by the <a href="http://home.wlu.edu/~lambertk/#Software">Breezy</a>
approach to Graphical User Interfaces developed by my colleague 
<a href="http://home.wlu.edu/~lambertk/">Ken Lambert</a>: an object-oriented
Application Programming Interface that is simple enough for beginners to use,
but that is efficient enough to scale-up to real world problems. 
As shown in the following code fragment, the API is extremely 
simple: a constructor that accepts the port (device) name on which the
unit is connected, and method for accessing the scans (range values):

<p>
<tt>
  from breezylidar import URG04LX
<p>
  laser = <b>URG04LX</b>('dev/tty/ACM0')
<p>
  while True:
<p>
    scan = laser.<b>getScan</b>()
<p>
    # <i>do something with scan, like 
<a href="http://home.wlu.edu/~levys/software/breezyslam/">SLAM</a></i>
<p>
</tt>
<p>

<h3>Installing for Python</h3>

<p>
The BreezyLidar installation uses the popular
<a href="http://docs.python.org/2/distutils/introduction.html">distutils</a> 
approach to installing Python packages, so all you should have to do is
download the repositry, cd to the directory where you put it, and do 

  <h3><b><tt>sudo python3 setup.py install</tt></b></h3>

For a quick demo, you can then cd to <tt><b>examples</b></tt> and do

  <h3><b><tt>make test</tt></b></h3>

This will attempt to connect to the Lidar, print out some information about it,
and read 20 scans.  I've found that it can take up to half a minute for the 
URG-04LX to be ready after plugging in its USB cable, so it's best to wait
a little between plugging it in and running the program.  Depending on how your
system is configured, you may need to run the program as root:

  <h3><b><tt>sudo make test</tt></b></h3>

or you may need to change the <tt><b>DEVICE</b></tt> specified at the top of urgtest.py to
<tt><b>/dev/ttyACM1</b></tt> or whatever file your URG-04LX maps to.


</p>

If you have the 
<a href="http://tkinter.unpythonic.net/wiki/How_to_install_Tkinter">Python Tkinter</a>
package installed, you can do

  <h3><b><tt>python3 urgplot.py</tt></b></h3>

to see a real-time plot of the scans.

<p>

To see what other features are available, do 

  <h3><b><tt>pydoc breezylidar</tt></b></h3>

<h3>Installing for C++</h3>

Just cd to the <b>cpp</b> directory, and do

  <h3><b><tt>sudo make install</tt></b></h3>

This will put the <tt><b>libbreezylidar</b></tt> shareable library in your <tt><b>/usr/local/lib</b></tt>
directory.  If you keep your shared libraries elsewhere, just change the <tt><b>LIBDIR</b></tt>
variable at the top of the Makefile.

<p>

For a quick demo, you can then cd to <tt><b>breezylidar/examples</b></tt> and do

  <h3><b><tt>make cpptest</tt></b></h3>

<p>

Again, you'll need to change the <tt><b>LIBDIR</b></tt> variable at the top of 
the Makefile in this directory as well, if you don't use <tt><b>/usr/local/lib</b></tt>.
You should also have this lib directory in your LD_LIBRARY_PATH environment variable.
I do this by putting the following line in my <b><tt>~/.bashrc</tt></b> file:

<b>
<pre>
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib
</pre>
</b>

<p>

To see what other features are available, go back to the <b>cpp</b> directory and do

  <h3><b><tt>make doc</tt></b></h3>

and then open  <tt><b>Documentation/html/index.html</b></tt> in a web browser.

<h3>Copyright, licensing, and questions</h3>

Copyright and licensing information (Gnu 
<a href="https://www.gnu.org/licenses/lgpl.html">LGPL</a>) 
can be found in the header of each source file.   If you have another Lidar model and are
interested using BreezyLidar with it, or have any other questions, please 
<a href="mailto:simon.d.levy@gmail.com">contact</a> me.

<h3>Acknowledgments</h3>

This work was supported in part by a  Commonwealth Research Commercialization Fund
grant from the Center for Innovative Technology (CRCF #MF14F-011-MS). 

</body>

</html>

