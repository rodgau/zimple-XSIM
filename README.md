
![](http://i.imgur.com/Bf7Ey7L.png)
# Overview #

*zimple-XSIM* is an [Electric Imp](http://electricimp.com) motor controller interface for the [X-Sim](http://www.x-sim.de) motion simulator software.
*X-Sim* is a Windows application designed to power motion simulator hardware for racing and flightsim games. The *zimple-XSIM* project explores the feasibility of Electric Imp powered motion simulators.

The firmware here enables the Imp to interface with the *X-Sim* software to translate its positioning commands into precise motor control for your simulator hardware.

**EX.**  Racing Game ↔ *X-Sim* ↔ **Imp** ↔ H-bridge ↔ DC Motors ↔ Roll/Pitch

The Imp connects to the PC via a USB cable to receive streamed position commands, and to H-bridge motor controllers to provide the required PWM motion signals. While designed to permit the use of affordable DC motors, this firmware could easily be adapted to work with any motor driver type. 

My personal goals for the project are two-fold:


1. to explore if the Imp is appropriate for such a applications, and
2. to provide an interesting project to learn about the *Electric Imp*, and web-enabled embedded devices in general.

So, please understand this is not intended to evolve into any polished state of completion. Nevertheless, even in its current state I hope this code provides useful insight and ideas.

# Vision #
Firmware features I currently envision include:

- support for at least 2 DOF (perhaps up to 6 DOF if using the imp002 hardware variant)
- wirelessly configurable via Windows, Android, iPhone, and browser based web-apps 
- data logging to the Cloud
- real-time performance charts viewable on tablet or PC.
- etc...
   

# Release   #
This is an initial root "internal" development version only. Not yet fully functional, nor fully tested. This will be the base to which further feature branches will be pushed.

It's still buggy, and it's still raw, but has these core features:

- single axis control
- adjustable PI feedback control
- Precise signal update rate of 400/sec (max)
- 10x potentiometer oversampling: basic averaging only
- basic HTTP RESTful API for web-based device control 
- performance monitoring options:
	- raw serial data stream output, or
	- *X-SimCTRL* software compatibility
	- verbose terminal emulator output


### Device-side Functionality  ###
- all core functions requiring real-time response are implemented here
- all processing is either event-based or triggered by hardware timers (no inefficient and imprecise 'Arduino-like' software loops are used anywhere)
- PI and agression settings can be changed without reflashing firmware 
- heartbeat output to server.log


### Agent-side Functionality  ###
- device configuration saves to Cloud and has a 'last known good' restore capability to facilitate machine tuning and experimentation.
- REST API
