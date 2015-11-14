/** Motor controller for X-SIM Motion Simulator Software.
 * Designed to run on electric imp001 microcontroller.
 * 
 * Pre-release dev version.
 * 
 * Revision History
 * Build
 * 000  Cloned from _zimple_branch_1 build 128.
 *      Working single axis PWM + dir model. P-only. Not yet web-enabled, no interface.
 * 035  Agent commands added:
 *        motorsEnable. Command to either enable or disable all motor outputs
 *        /pid/Kp, Ki, Kd: set PID parameters (only Kp functional for now)
 *      pid initialization function added
 * 039+ added ping-pong agent-imp watchdog
 * 099  App-->Agent control added:
 *      Auto-loads factory controller settings from agent-side 'firmware' into imp Cloud upon first-ever bootup.
 *      Agent mediates and validates between mobile app and controller for any request to change controller settings.
 *      All preferences held within Agent. Any valid change synches to both controller and imp Cloud automatically.
 *      Controller loads its configuration from imp Cloud every power-up (i.e. auto disaster-recovery)
 *      HTTP handlers added: burn, revert, factory, WIPE
 *        burn:     takes snapshot of current controller settings. Saves to imp Cloud.
 *        revert:   reverts back to last known good controller settings.
 *        factory:  restores all controller settings to factory defaults
 *        WIPE:     (internal testing only)
 * 126  Agent: changed Kp,Ki,Kd parsing from url-based to key/value pairs in an http content/text body
 *      Good working base.
 *
*/

const BUILD = "126";
pwmFreq <- 5000; // PWM pulse frequency, Hz
Kp <- 0.0; // defaults prior to Cloud reload
Ki <- 0.0;
Kd <- 0.0;

const PROC_FREQ = 200; // main machine update Hz (POTs polled, setpoint grabbed, PID calc'ed, motor outputs updated)
const DASH_FREQ = 20; // dashboard serial output update freq, Hz
const XCTRL_FREQ = 50; // rate we send data back out to XsimCTRL 
const OVERSAMPLING = 10; // no. of ADC readings to average per pin (channel)
const DELIM  = "\x41\x42\xFF"; // XsimCTRL motion stream delimiter
const FIFOSIZE = 80; // size of imp's UART FIFO, bytes

numMotors <- 2; // #of motors of physical machine
DELIM_LEN <- DELIM.len();
frameSize <- DELIM_LEN + numMotors;

motorsOn <- 0; // enable or disable all motor output

count_motionFrames <- 0; // increment upon serial port motion frame detected
count_motionBufOverruns <- 0; // increments upon valid serial port motion frame detected                      
count_motionStreamErrors <- 0; // increment if delimiters missing from serial motion stream
motionBuf <- blob( FIFOSIZE + 1 ); // raw serial motion data dumped here to parse through. size of UART FIFO + at least 1.

serialVerbose <- false; // raw or verbose machine serial output (set true for output to DAQfactory, etc.)
dashboardOn <- false; // stream machine dashboard data to serial
xsimctrlOn <- true; // set machine's serial output protocol to XsimCTRL


/* main machine state array initialized here. Refreshed each update cycle.
*  Contains each motor as [ setpoint, position, error, dutyCycle, direction, position_min, position_max, pinPWM, pinDir ]
*  i.e. 9 attributes as columnar index elements:
*/
motor <- array(numMotors);
for (local i = 0 ; i < numMotors ; i++)
{
		motor[i] = array(9, 0);
}
// the above 2D array accessed as: motor[motorNumber][attribute_index]

/**----------------- initialize motor parameters -------------------
 */


// alias pin9 as motor1_output for PWM output at 5000 Hz pulse frequency
motor1_output <- hardware.pin9;
motor1_output.configure( PWM_OUT, 1.0/pwmFreq, 0.0 );

// alias pin8 as digital output for motor1_direction
motor1_direction <- hardware.pin8;
motor1_direction.configure( DIGITAL_OUT );

// alias pin2 analog input as motor1 POT position
motor1_pot <- hardware.pin2;
motor1_pot.configure(ANALOG_IN);

// alias pin1 as digital output for led
led <- hardware.pin1;
led.configure( DIGITAL_OUT );
ledState <- 0;

// alias pin5, pin7 as UART57
serial <- hardware.uart57;
serial.configure(19200, 8, PARITY_NONE, 1, NO_CTSRTS);


motor[0][7] = motor1_output;
motor[0][8] = motor1_direction;

// imp ADC Sampler callback
// each pin gets oversampled and then filter averaged here
// updates motor[] global machine state
function samplesReady(buffer, length)
{
	local val, valFiltered, tmp, max, min;
	local i, m, channelNum, numSamples;
	local str = "";

// 	ledState = 1-ledState; //debug
//  led.write (ledState); //debug

	if (length > 0)
	{
		// for each pin, average its samples, find min/max:
		for ( local m = 0; m < numMotors; ++m )
		{
			tmp = 0;
			val = 0;
			max = 0;
			min = 65535;
			for ( i = 2*m; i < length; i += (2 * numMotors) )
			{
				val = buffer[i] + buffer[i+1] * 256; // shift little endian
				if ( val > max )  max = val;
				else if ( val < min) min = val;
				tmp += val;
			}
			// calc average for our 'filtered' value
			numSamples = (length / 2) / numMotors; // #samples = buffsize/2 / #channels
			valFiltered = tmp / numSamples;
			motor [m][1] = valFiltered;
			motor [m][5] = min;
			motor [m][6] = max;
		}
	}
	else
	{
		server.log("ADC buff overrun");
	}

// 	ledState = 1-ledState; //debug
//   led.write (ledState); //debug
  
  getMotion(); // get setpoint
	feedbackLoop();

// 	ledState = 1-ledState; //debug
//   led.write (ledState); //debug

}


/* -Sampler rules (general):
			-Sampler can be used to generate an averaged samples output at a desired frequency
			-consumes 2 bytes per sample (16 bits) into a buffer
			-the two bytes are stored backwards, so true value = buffer1[i] + buffer1[i+1] * 256
			-all pins sampled simultaneously, i.e. at each sample cycle
			-when buffer fills, a callback function is called which can be used to analyze/average samples
			-so callback freq = 2 * ( sampling freq ) / (buffsize)
		Example: freq = 1000 hz, buffsize 2000, callback freq = 2 * (1000) / (2000) = 1 hz
		
		But, we want our callback freq = PROC_FREQ. Also, note that buffsize = #samples * 2, so
			-Sampler sampling freq = #oversamples * PROC_FREQ (Sampler's sampling freq)
			-buffsize = 2 * #oversamples desired (set our buffer size to this)
		Example: PROC_FREQ of 100 hz, OVERSAMPLING = 10,
			-use a buffsize of 2*10 = 20, and a Sampler frequency of (100) hz.
		
		Pin considerations:
			-sampling 1 pin:  use 2 buffers, each of size buffsize
			-2 pins:          2 buffers, each of size (2 * buffsize)
			-3 pins:          2 buffers, each of size (3 * buffsize)
			 generally...     2 buffers, each of size (#pins * 2 * #oversamples)
			-data for pins is interleaved:
			    buffer[ pin1, pin2, pin1, pin2,... ], (where pinx is a 2-byte pair of byte_lower, byte_upper)
			
		Callback processing considerations:
			-the CB function is blocking, meaning, it must finish before the next CB is called
			-if it doesn't, the Sampler generates a buffer overrun condition
			-this is communicated by passing a NULL buffer and a length value of 0 to the CB
			-this state should be checked inside the CB.
			-overruns can be avoided by
*/

// 3 buffers here required if verbose dashboard logging enabled (2 buffs gives a few overRuns )
buffer1 <- blob( 2 * OVERSAMPLING * numMotors);
buffer2 <- blob( 2 * OVERSAMPLING * numMotors);
buffer3 <- blob( 2 * OVERSAMPLING * numMotors);
// buffer4 <- blob( 2 * OVERSAMPLING * numMotors);
// buffer5 <- blob( 2 * OVERSAMPLING * numMotors);
// buffer6 <- blob( 2 * OVERSAMPLING * numMotors);
// buffer7 <- blob( 2 * OVERSAMPLING * numMotors);
// buffer8 <- blob( 2 * OVERSAMPLING * numMotors);

//DEBUG:  adjust the passed array [motor1_pot...] to have one element for each motor in machine
//        (hardcoding 2, [motor1_pot, motor1_pot], just for testing xsimCTRL s/w)
hardware.sampler.configure(  [motor1_pot, motor1_pot], (OVERSAMPLING * PROC_FREQ), [buffer1,buffer2,buffer3], samplesReady);

// function to read setpoint, compare to motor position, and calc error and motor PWM output
function feedbackLoop() 
{
	 // register this function to fire at PROC_FREQ frequency
//TEST call via Sampler() instead:	imp.wakeup( 1.0/PROC_FREQ, feedbackLoop);

	for( local m = 0; m < numMotors; ++m )
	{
		local setpoint = motor [m][0]; // get setpoint, 0-65535
		local motorPot = motor [m][1]; // get motor position, 0-65535
		local error = motorPot - setpoint;
		local dutyCycle = Kp * error;
		
		// catch duty cycle clipping
		if ( dutyCycle > 1.0 )
			dutyCycle = 1.0;
		else if ( dutyCycle < -1.0 )
			dutyCycle = -1.0;
			
		// set direction
		local dir;
		if ( dutyCycle < 0.0 ) {
			dir = 1;
			dutyCycle = -dutyCycle;
		}
		else
			dir = 0;
	
		if ( m ==0  ) //debug:  only motor0 for now
		{
  		motor [m][8].write(dir);
  		if (motorsOn == 1)
  		  motor [m][7].write( dutyCycle );
  		else
  		  motor [m][7].write( 0 );
		
  		// update machine state array with above calcs
  		// each motor has [ setpoint, position, error, dutyCycle, direction, position_min, position_max]
  		motor [m][2] = error;
  		motor [m][3] = dutyCycle;
  		motor [m][4] = dir;
		}
	
 }
}

function getMotion()
{

	local i, s, leftovers;

 if ( motionBuf.eos == 1)
  {
    // buffer overRun
    ++count_motionBufOverruns;
  }
	else
    motionBuf.writeblob( serial.readblob( FIFOSIZE) ); // use this, in reality

	local bufLen = motionBuf.tell();
	  
/*  motionBuf.seek(0); //debug
  server.log("motionBuf.len["+motionBuf.len()+"]"); //debug
  server.log( format("%s",motionBuf.tostring())); //debug
 	server.log( format("starting motionBuf length[%d]",bufLen)); //debug
*/		

	if ( bufLen >= frameSize ) // if enough data has arrived, search for valid frames
	{
		motionBuf.seek(0); // starting at beginning

		do
		{	// look for a valid frame here
		
		// 	server.log("checking at index["+motionBuf.tell()+"]"); //DEBUG
			s = motionBuf.readstring(DELIM_LEN);
		// 	server.log("after header advanced, tell["+motionBuf.tell()+"]"); //debug
			
			if ( s == DELIM )
			{
        ++count_motionFrames;
			 // server.log("header found"); //debug
				for ( i = 0; i< numMotors; ++i )	
				{ // get all data
					motor[i][0] = motionBuf.readn('b') << 8; // map to 0-65535 (assumes data 0-255)
				// 	server.log( format("data[%i]",motor[i][0] >> 8)); //debug
				}
			}
			else
			{
				motionBuf.seek( 1-DELIM_LEN, 'c' );	// header not found, move down 1 and try again
				// server.log("frame not found["+s+"]"); //debug
			}
			
		} while ( motionBuf.tell() + frameSize <= bufLen ) // if still room for another, look again
		
		// no more full frames left to process, so copy leftover bytes to front of buffer		
		leftovers = bufLen - motionBuf.tell();
		// server.log("checking leftovers..."); //debug
		// server.log("#leftovers["+leftovers+"]"); //debug
		
		if ( leftovers > 0 )
		{			
			local tmp = motionBuf.readblob( leftovers );
		// 	server.log("leftovers are["+tmp.tostring()+"]"); //debug
			motionBuf.seek(0);
			motionBuf.writeblob (tmp);
		}
		else
			motionBuf.seek(0);
			
		// server.log("final motionBuf.tell["+motionBuf.tell()+"]"); //debug
		// motionBuf.seek(-leftovers, 'c'); server.log("final motionBuf["+motionBuf.readstring(leftovers)+"]"); //debug
		
	} // end frame processing
// 	else server.log("buf too short to process"); //debug
	  

} // end function


// function to send heartBeat to server
function heartbeat()
{
	server.log( "BUILD["+BUILD+"] Motion Frames/Overruns/Errors[" + count_motionFrames +
	                           "/" + count_motionBufOverruns + 
	                           "/" + count_motionStreamErrors +"] " +
	                           "Kp[" + print_k(Kp) + "] " +
	                           "Ki[" + print_k(Ki) + "] " + 
	                           "Kd[" + print_k(Kd) + "] " + 
	                           "motors[" + motorsOn + "]" );
  // register function to fire once every 3 secs
	imp.wakeup( 3.0 , heartbeat );
}

// function for continuous ping-pong watchdog between device and agent
function ping()
{
  // Send a 'ping' message to the server with the current millis counter
  agent.send("ping", hardware.millis());
}

// function to act on pong delay from server, if any 
function return_from_imp(startMillis)
{
  // Get the current time
  local endMillis = hardware.millis();
  // Calculate how long the round trip took
  local diff = endMillis - startMillis;
  // Log it
  server.log("Round trip took: " + diff + "ms");
  // Wake up in 5 seconds and ping again
  imp.wakeup(5.0, ping);
}
 
// function to listen for 'pong' message from the agent, call return_from_imp()
agent.on("pong", return_from_imp);
 

// function to send dashboard data out serial port for monitoring and logging
function dashboardOut()
{
	local str, m;
	
	if ( serialVerbose )
	{
		m = 0; // just motor #1 for now
		str = format ( "SP: %u GP: %u ERROR: %i DutyCycle: %u Direction: GPmin: %u GPmax: %u\r\n",
										motor[m][0], // setpoint (filtered)
										motor[m][1], // pot reading (filtered)
										motor[m][2], // error calc
										(motor[m][3] * 100.0), // duty cycle %
										motor[m][4], // motor direction
										motor[m][5], // pot reading min in oversampled group
										motor[m][6] ); // pot reading max in oversampled group
	}
	else
	{
		m = 0; // just motor #1 for now (coma-delimited with carriage return)
		str = format ( "%u,%u,%i,%u,%u,%u,%u\r",
										motor[m][0], // setpoint (filtered)
										motor[m][1], // pot reading (filtered)
										motor[m][2], // error calc
										(motor[m][3] * 100.0), // duty cycle %
										motor[m][4], // motor direction
										motor[m][5], // pot reading min in oversampled group
										motor[m][6] ); // pot reading max in oversampled group
	}
	serial.write( str );
	 // register function to fire at set freq
	imp.wakeup( 1.0/DASH_FREQ, dashboardOut );
}

// function to send machine state info out serial port to X-Sim-CTRL app
function xsimctrlOut ()
{
	local str;
	
	str = format( "[%02X%04X%04X%02X%02X%04X%04X%02X]", motor[0][0]>>8, motor[0][1]>>8,0,0,motor[1][0]>>8, motor[1][1]>>8,0,0 );
	serial.write (str);
	
	 // register function to fire at set freq
	if( xsimctrlOn )
		imp.wakeup( 1.0/XCTRL_FREQ, xsimctrlOut );
}

//-------------- function to scale k for display
function print_k( k )
{
  return (k * 10 * 65535).tointeger(); // this is not the "true" k (scaled to be more legible as a printout)
}

//------------ function to restore controller preferences provided by agent
function setPrefs(prefsTable)
{
  Kp = prefsTable.Kp;
  Ki = prefsTable.Ki;
  Kd = prefsTable.Kd;
  server.log("Controller preferences set.");
}

/*//-------------- function to save controller preferences to imp Cloud
function savePrefs()
{
  // Cloud table for persistent storage
  local controllerPrefs <- {};
  controllerPrefs.Kp <- Kp;
  controllerPrefs.Ki <- Ki;
  controllerPrefs.Kd <- Kd;
  return controllerPrefs;
}
*/
//------------ function to either enable or disable all motor output
function setMotors(motorState)
{
  motorsOn = motorState;
  ledState = motorState;
  server.log( "Command executed: motorsEnable[" + motorState + "]");
}

/*-------------------------------- Agent handlers ------------------------------
*/

agent.on("controller.set.prefs", setPrefs); // Register handler for incoming preferences data
// REPLACED agent.on("controller.save.prefs", savePrefs); // Register handler for outgoing preferences data
agent.on("motorsEnable", setMotors); // register handler for "motorsEnable" messages from the agent
agent.on("setKp", function(x) { server.log("Received Kp["+x+"]"); Kp = x }); // set Kp global
agent.on("setKi", function(x) { server.log("Received Ki["+x+"]"); Ki = x }); // set Ki global
agent.on("setKd", function(x) { server.log("Received Kd["+x+"]"); Kd = x }); // set Kd global


/**---------------------- start the loops ---------------------------------
 * 
 */

server.log("Requesting controller preferences from agent...")
agent.send("controller.get.prefs", 1); // Issue a request for device preferences
heartbeat();
ping(); // start the ping-pong watchdog
hardware.sampler.start();
if (dashboardOn) dashboardOut();
if (xsimctrlOn) xsimctrlOut();


