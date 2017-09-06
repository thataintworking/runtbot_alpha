# runtbot
This is the code for a little robot I am working that will
randomly move around the house. It is intended to be a 
learning exercise more than anything.

I am in the process if offloading the motor drive process
to an ATmega328.

---

**shutdown_button.py**  
This script needs to be run in the background at startup.
It watches for BCM pin 19 to be pulled down to ground and
then released. Once it detects this, it executes a graceful
system shutdown. You can put the following command in 
/etc/rc.local at the bottom right before the exit command:

`/home/pi/runtbot/venv/bin/python3 /home/pi/runtbot/shutdown_button.py &`

Note the ampersand at the end. This script does not exit so
it must be put in the background when started.
