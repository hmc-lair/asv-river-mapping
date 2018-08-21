# ASV River Mapping
Framework for ASV autonomous control and river mapping (Oceanserver Q-Boat).

Contacts: John Lee (johlee@hmc.edu), Jane Wu (jhwu@hmc.edu)

### Folder Descriptions
+ ```ASV_Controller```: Files used by shoreside computer and Raspberry Pi. The Pi is onboard the ASV, and communicates with the shore-side computer via X-Bee communication.
+ ```Maps```: GEOTIFF files for various locations, including Caltech and Kern River.
+ ```Motion_Planning```: RRT-type planner, sequentially plans trajectories to travel back and forth across river.
+ ```pi_readwrite```: Arduino file for IMU and servo control. Communicates with the Raspberry Pi via serial.
+ ```Deprecated Files```: Files that are no longer used.

### ASV Terminology
+ VCU = Vessel Control Unit. The VCU connects directly to the motors and servo.

...

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
