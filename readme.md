# What is Wingman?

Wingman is an open-source continuation of a project I started working on during my time at Team Swift. Essentially, the aim is to remove the headache of using pymavlink. What this does is wrap all the complicated back-end around an easy-to-use interface. You can always _optionally_ access the backend, but if you don't care, it'll handle it for you.

Since, for now, this is a one-man project I can't guarantee that I'll be able to maintain it well, and it'll likely start by only serving the functionality I will actually use, but my hope is that with time this can grow into something useful for more people, which is why I'm open sourcing it.

## How to install

The _plan_ is that you'll just run: 
``` bash
pip3 install wingman-ardupilot
```
and it'll just install on your device. Imma check out how to register with a package tracker but for now you'll have to follow this install process:
``` bash
cd wingman-ardupilot
pip install .
pip install -r requirements.txt
```
## Control Interface Model
Wingman talks to the UAV using one of three "techniques" known as "control interfaces":
- Brain->Brain
- Brain->Body
- Brain->Bone

### Why?
Simply put, adhering to this interface model allows us to significantly simplify communication with the UAV. No more memorising mavlink instructions or trying to debug edge cases, the library will handle the monitoring and control and let the programmer focus on what really matters.

If the user requires the UAV to maintain a specific velocity, they may simply set velocity and let wingman handle controlling the UAV to ensure the velocity is maintained.

### Brain->Brain Interface
The Brain->Brain interface is a high-level command intended to tell the UAV a general intention, and allow the UAV to execute the intention itself. This entails instructions such as waypoints, geofences, or altitude holding. Here, you are less concerned about the exact way something is achieved and more with achieving said goal.

### Brain->Body Interface
The Brain->Body interface is a middle-layer command, that allows us to specify desired conditions of the UAV. Such as setting the desired velocity or altitude, or even specifying things like glide slope.

### Brain->Bone Interface
Finally, the Brain->Bone interface offers the lowest-level control of the UAV. It allows us to control a specific element such as an attached servo or relay. 

Brain->Bone is also the layer which is mostly automated by wingman. If there is built-in functionality (such as waypoint following) in ardupilot, the Brain->Brain command need just to pass along the desired instruction. However, if an advanced functionality is implemented by Wingman, such as a barrel roll trick (for example), wingman will implement the bone-level commands for this. 

## Interfacing with Lua Scripts
In my experience, the "bone level" commands are handled much better using lua scripting, so I will look into a way to interface this with lua scripts running on the autopilot.