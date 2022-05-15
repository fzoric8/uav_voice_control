# UAV voice control
This is BSc assignment on Faculty of Electrical Engineering and Computing in Zagreb, Croatia ([FER](https://www.fer.unizg.hr/en)). The goal of this assignment was to produce a controller for remote control of an unmanned aerial vehicle on vocal commands. Speech recognition engine that is used in this assignment is `PocketSphinx` and it is implement in script `voice_rec.py`. Controller is implement in script `uav_control.py`.

## Installation

### Voice recognition
Firstly install all packages which are required for running `PocketSphinx`: 

```bash
sudo apt-get install python3-pip
sudo apt-get install libasound-dev
```
Extract `pa_stable_candidate_v190700_rc2_20210331.tgz`:

```bash
tar zxvf pa_stable_candidate_v190700_rc2_20210331.tgz
cd portaudio
./configure
make
make install
ldconfig
```

Now install `PocketSphinx`:

```bash
pip3 install pyaudio
pip3 install pyttsx3
python3 -m pip install --upgrade pip setuptools wheel
pip3 install --upgrade pocketsphinx
sudo apt-get install gstreamer1.0-pocketsphinx
```

### UAV control
To start uav simulation in Gazebo, build docker image from [mmuav_ros](https://github.com/larics/docker_files/tree/master/ros-melodic/mmuav_ros). There is explained how to start simulation.

## Run

Firstly start simulation.

Open new terminal window:
```bash
source ~/<developer_ws>/devel/setup.bash
rosrun uav_voice_control voice_rec.py 
```

Open new terminal window:
```bash
source ~/<developer_ws>/devel/setup.bash
rosrun uav_voice_control uav_control.py 
```

NOTE: If script `voice_rec.py` does not recognize words, try fixing thresholds in `/uav_voice_control/src/key.list`.
