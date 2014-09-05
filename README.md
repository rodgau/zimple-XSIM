## Welcome to zimple-XSIM ##


This is an [electric imp](http://electricimp.com) motor controller for the [X-Sim](http://www.x-sim.de) motion simulator software.
# Overview #

*X-Sim* is a Windows application designed to power motion simulator hardware. The *zimple-XSIM* project contains firmware written for the electric imp microcontroller allowing you to interface your DC motors to *X-Sim*. The imp accepts position commands from *X-Sim* and translates them into appropriate PWM signals for your motor driver hardware. 

The goal of this project for me is to explore if the imp can act as an accurate, and very responsive PID-based motor controller for *X-Sim*-based motion simulators. By "explore", I mean that my main goal is to learn about the electric imp, and web-enabled embedded devices in general, not necessarily to evolve this into a fully mature product.

Firmware features I currently envision include:

- support for at least 2 DOF (perhaps up to 6 DOF)
- wireless configuration and control via Windows (and perhaps an Android app) 
- web-enabled for data logging and real-time performance charts
- community support (share tuning setups)?
- etc...

 
   

### Release Functionality  ###
This is an initial root "internal" development version only. Not yet fully functional, nor fully tested. This will be the base to which further feature branches will be pushed.

- single axis control only
- PID feedback control: P-only
- PWM signal update rate: 400/sec (max)
- 10x potentiometer oversampling: basic averaging only
- no configuration user interface yet 
- machine performance monitoring options:
	- raw serial data stream
	- X-SimCTRL software
	- verbose terminal emulator output


### Device-side Functionality  ###
- all core functions requiring real-time response are implemented here
- all processing is either event-based or triggered by hardware timers
- heartbeat output to server.log


### Agent-side Functionality  ###
None yet.
