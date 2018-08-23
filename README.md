# ASV River Mapping
Framework for ASV autonomous control and river mapping (Oceanserver Q-Boat).

Contacts: John Lee (johlee@hmc.edu), Jane Wu (jhwu@hmc.edu)

### Python Library Requirements
tl;dr: Run code and figure out which python libraries you're missing by looking at the errors..

List of Python libraries (MUST use Python 3, use pip3 to install):
+ xbee
+ digi-xbee
+ tkinter
+ GDAL
+ utm
+ matplotlib
+ numpy
+ Pillow (aka PIL)
+ pyserial (aka serial)

### Folder Descriptions
+ ```ASV_Controller```: Files used by shoreside computer and Raspberry Pi. The Pi is onboard the ASV, and communicates with the shore-side computer via X-Bee communication.
+ ```Maps```: GEOTIFF files for various locations, including Caltech and Kern River.
+ ```Motion_Planning```: RRT-type planner, sequentially plans trajectories to travel back and forth across river.
+ ```pi_readwrite```: Arduino file for IMU and servo control. Communicates with the Raspberry Pi via serial.
+ ```Deprecated Files```: Files that are no longer used.

### Mission Planning Instructions
1. (Optional) To start the GUI in simulation mode, go to ASV_controller.py and set ```self.mode = "SIM MODE"```. Make sure to change this back to ```self.mode = "HARDWARE MODE"``` before real deployments.
2. Start the GUI by running in a terminal window:
```
$ python3 ASV_gui.py
```
3. (Optional) Use "Border Configuration" buttons to trace/clear/load/save border. This serves as a guide for determining where waypoints can be placed.
4. Check compass offset under "Compass Calibration." The default offset is -12 degrees.
5. Check/update control parameters under "Control" depending on mission. Most important values are "Fwd Limit/Bwd Limit" because this determines the max motor thrust (0-1000 rpm).
6. Enable/disable "Transect Mission." If enabled, the ASV will repeat the specified waypoints until "Stop ASV" is pressed.
7. In the "Mission Planning" panel, either load a mission or manually add waypoints. To change the type of controller used to reach a particular waypoint (point track=upstream vs. transect=across river), click on the point in the map (click "Done Add Waypoints" if necessary).
8. Click "Start Mission." This WON'T start the ASV, but it will send all the mission waypoints to the ASV. Check the ASV ssh terminal window to make sure all waypoints were sent (checksum should be correct) AND the "!STARTMISSION" message MUST have been sent.
9. If everything is good, click "Start ASV" and the mission will begin.
10. At any point, click "Stop ASV" to stop the ASV from moving (but if ASV re-started, mission will resume). "Abort Mission" will clear all the waypoints on the ASV.

***CURRENT BUG: IF MISSION RE-STARTED, FIRST POINT WILL BE POINT TRACK***


### Deployment Instructions
*NOTE: Once the RC transmitter is turned on, it CANNOT be turned off while the ASV is still on. Otherwise both motors will start spinning at max speed... On the other hand, the transmitter can be turned on while the ASV is already on.*

1. Connect 3 batteries to the connectors labeled '24V' in the ASV.

    *IMPORTANT: connect red-red and black-black!!!*

2. Connect Pi USB adapter to power bank inside ASV (should be near cardboard box).
3. Turn red key on VCU 90 degrees clockwise to turn on ASV. Servos will move - it's just initializing.
4. Optional (but mandatory when in river): Turn on RC transmitter. It should beep repeatedly for half a second to indicate it has connected to the ASV.
5. Turn on WiFi. Pi should automatically connect... Connect shoreshide computer to same Wifi and ssh into Pi.
6. Plug in X-Bee to a USB port on the shoreside computer.
7. On the Pi, run the following to start the ASV program:
```
$ cd Documents/asv-river-mapping/ASV_Controller
$ stty -echo #You won't be able to see what you type anymore!
$ python3 ASV_main.py
```
8. In a different terminal window (locally), run the following to start the GUI:
```
$ cd ASV_Controller
$ python3 ASV_gui.py
```
9. To terminate both programs, exit out of the GUI.

### Shutdown Instructions
1. Turn off RC transmitter.
2. Turn red key on VCU 90 degrees counterclockwise (back to original position) to turn off ASV.
3. Unplug the 3 battery connectors.
4. Unplug the Pi USB adapter.
