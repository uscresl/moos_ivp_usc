To run this simulation (assuming you have installed MOOS-IvP):

In a terminal tab:
$ pAntler simple-auv.moos

To start vehicle:
- click on the DEPLOY button

You should see the AUV start going to a waypoint pattern.
(You can use the arrow keys to move in the pMarineViewer window, i/o to zoom in/out)

Waypoints can be changed before start-up in simple-auv.bhv
Waypoints can be changed during the mission by sending an update. 
For example, in a new terminal tab:
$ uPokeDB simple-auv.moos WPT_UPDATE=points=50,-50:60,-60:70,-80
gives the WPT_UPDATE variable the content "points=50,-50:60,-60:70,-80", which
updates the waypoint behavior's points parameter.
$ uPokeDB simple-auv.moos WPT_UPDATE=depth=15
should update the depth behavior that runs in conjunction with the wpt bhv.
You cannot see the depth in 3D, but the vehicle depth can be read from one of
the information fields at the bottom of the pMarineViewer window.

You can also monitor variables in the MOOSDB (data buffer) by using 'uMS',
in a new terminal tab:
$ uMS
- click 'Connect'

I set it up such that you can switch between waypoint and heading/speed/depth
control. There are buttons to select on the pMarineViewer, or you can change it
before starting the simulation in the .bhv file MOOS var initialization bit.

Again, h/s/d behavior parameters can be updates using the updates parameter.
I made them all listen to the same variable "HSD_UPDATE", because I figured that
would be easy for whoever is publishing, and the parameter names are different
anyway, so it should not conflict.
To update all at once, use '#' as a separator. 
For example:
$ uPokeDB simple-auv.moos HSD_UPDATE=heading=90#speed=1.5#depth=25
See also the documentation (moosivp-helm.pdf - section 7.2.2)

All contents of the MOOSDB are being logged into the log files in the MOOSLog_*
timestamped directory.
A useful file is the *.alog file. 
This one you can either look at with vi(m), do a 'grep' on,
or you can open it with the program 'alogview':
$ alogview *.alog
which allows you to replay the mission, see the behavior output, and plot 
variable values.
