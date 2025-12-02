# ART DTRACK ROS2 bridge

This repo connects the ART DTRACK marker based 3D tracking system with ROS2 over python.
Parts of the code can additionally be used in python-only environments where ROS2 is not available.

## Synchronizing Clocks
It is crucial to synchronize the clocks of the DTRACK controller and the system that runs this code. Otherwise, the measured transformationes will have incorrect timestamps which will lead to various major bugs in your software.

If you use Ubuntu, you can synchronize clocks using `chrony`.\
Install it with  `sudo apt install chrony`\
Edit `/etc/chrony/chrony.conf` and add the following line to the end:\
`allow`\
Restart the chrony service with `sudo systemctl restart chronyd.service` \
In the DTRACK software interface go to `Hardware`->`Time` and enter the IP of your computer under the NTP option.

After connection the interface might display a different time than your computer. This is due to usage of timezones. Our software should automatically detect the timezone and correct the timestamp accordingly. However, it is a good practice to check the time stamps in the ROS messages to make sure that everything worked correctly.

A common pitfall that we ran into was having multiple network interfaces on the machine (i.e. an extra wifi). If you encounter issues with the DTRACK not connecting to chrony, try disabling the other network interfaces that are not connected to the DTRACK controller.

## Activate DTRACK 
The DTRACK controller needs to be configured to send out the data via UDP. This can be easily activated using the DTRACK interface in the `Output` menu.
Keep the default selection of data types.\
Naturally, it is also necessary to start the tracking afterwards.

## Run with ROS2

The `art_dtrack_tf` node receives the UDP packages and broadcasts named transforms. To do so, several ROS2 parameters are required:

|Name|Type|Description|Example|
|-|-|-|-|
|**Required**|
|**`ip`**|`string`|The IP address that you entered in the DTRACK interface output menu. |`192.168.0.53`|
|**`bodies`**|`string`|The list of tf names used for broadcasting. Should match the order of calibrated bodies in the ART DTRACK software. |`['pen', 'antlers']`|
|Optional|
|`frame`|`string`|The tracked body frames are broadcasted relative to this frame. |`base_track`|
|`port`|`int`|The port of the UDP communication. |`4100`|
|`buffer_size`|`int`|Length of the buffer used to receive one UDP message. Set this high enough so the entire message will fit. |`1024`|
|`timeout`|`double`|How long to wait for one UDP message in seconds. |`3.0`|

Run the node directly with the command
```bash
ros2 run art_dtrack_ros art_dtrack_tf --ros-args -p ip:="XXX.XXX.XXX.XXX" -p bodies:="['mybody1', 'antlers', 'pen']" # -p frame:="world"

```

## Run with python

Use the `/art_dtrack` directory in your project directly to receive UDP messages. You can either copy it over or create a symlink to the directory. This minimal example shows how you can use the `ArtDtrackReceiver`:

```python
from art_dtrack.art_dtrack_receiver import ArtDtrackReceiver, DtrackBody

# initialize the receiver object
receiver = ArtDtrackReceiver(ip="XXX.XXX.XXX.XXX", port=4100, con_timeout=3.0)

# Loop infinitely to process all incoming UDP messages
while True:
    # block the thread until one UDP message arrives
    msg: DtrackBody = receiver.receive(buffer_size=1024)
    # do something with the parsed message object
    print(msg)
```
