/** Motor control via simple Proportional feedback loop.
* Setpoint via manual POT input. Error read from second POT attached to motor shaft.
* Multiple timer loops run concurrently:
*   -feedbackLoop() handles PID calcs (100 hz)
*   -dashboardOut() handles output logging stream (20 hz)
*   -heartbeat() 1 hz server.log in leui of LED flasher
*   -imp's Sampler() configured for all ADC potentiometer capture (100 hz).
* 
* Revisions History
*   Forked from Build 200 with these changes:
*   000 -as is (19200 baud, PROC_FREQ 100, DASH_FREQ=20): OK
*   004 -comment out unused serialData echo function and move serial pin conf to top, add BUILD# into heartbeat
*       Tests:  OK. Runs "forever" while being viewed in DAQfactory.
*   005 -add: xsimctrl output only (NO serial input code) as a test. 2 Hz send only.
*       OK
*   006 -increase XCTRL_FREQ to 20 hz: OK
*   009 -motor state array cleaned up
*       -remove all references to positional pot, as transition to serial streaming
*       -setpoint mapped to getpoint just to see the echo to xsimctrl
*       OK
*   014 -add getMotion() to receive setpoints from xsimCTRL app @1 hz, sends to xsimCTRL still @ 20 hz.
*       OK
*   015 -increase xsimCTRL receive to 20 hz
*       CRASH/REBOOT at getMotion line 255 "index '7' does not exist"
*   020 -fixed motionFrameSeek sizing error. Should use #axis in received data stream, not numMotors
*       OK
*   024 -increase PROC_FREQ from 1 hz to 100 hz
*       OK, but only getting 30 motion frames detected/sec when 100/sec sent (xsimCTRL @ 10 ms)
*   025 increasing PROC_FREQ to 300 hz: no effect
*       decrease xsimCTRL to 5 ms: no effect (receives max out at ~30/sec)
*   026 -led toggle added to debug (dir pin used). Logic analyzer confirms following:
*       getMotion freq: 50 hz actual (300 hz setting). PROC_FREQ@100: still 50 hz actual. Disable feedback loop: no diff.
*   030 disable xsimCTRL output too:  immediate CRASH/freeze
*   031 all loops disabled, except getMotion (xsimCTRL not running)
*       OK.  Still only 50 hz when getMotion freq @ 100 hz.
*   033 ADC Sampler disabled
*       52 hz (+2 hz improvement only.
*   037 try calling getMotion() from the existing Sampler callback instead.
*       OK.  getMotion() now runs at full PROC_FREQ as intended,
*            but still never get more than 30 frames/sec from xsimCTRL.
*   038 all loops re-enabled at full regular speeds. xsimCTRL output re-enabled (both send and receive)
*       OK, but still have the 30 hz frame detection limit.
*       Enabling receive inside xsimCTRL freezes firmware!
*   040 revert back to calling getMotion as a imp.wakeup
*       OK
*   041 try calling getMotion() from the existing Sampler callback again.
*       OK at PROC_FREQ/XCTRL_FREQ = 100/20 hz and XsimCtrl sending at 50 ms.
*   042 try increasing xsimCtrl send to 10 ms:
*       OK
*   043 increase baud to 57600
*       Locks up when until click Receive in xsimCTRL (ok if just sending)
*   044 try same at 38400 bauds
*       same lockup result.
*   045 19200 baud again.
*       receive still crashes firware!
*   047 revert everything back to "normal"
*       worked once. Next time, back to crashing again.
*       try rebooting PC.
*       OK (wtf...). Try again. locks up. Then, never works again.
*   054 found that wiring affects things. re-wired so 3v3 no longer tied to Sparkfun USB UART BoB.
*       Also using just its 5-pin header instead of the side headers (not sure if relevant).
*       Found that the logic analyser UART analysis of this is WAY WAY cleaner, with basically no bad packets anymore.
*       OK:  at 1 hz proc_freq. need try faster...
*   057 proc_freq 100 Hz:
*       Almost good. sometimes when turning receive off in xsimctrl, firmware freezes.
*   058 BUG found in getMotion: do loops inifinitely every bad frame. added 2nd motionBuf.seek into bad frame else
*       still locks when xsim set to receive. set xsimctrl=false. still locks.
*   062 change uart config flag to NO_RX
*       Behaves as expected. nuthin happens
*   065 BUG found in getMotion. 058 fix was not enough. Any legit attempts to seek the motionBuf blob pointer
*       past start (which happens every invalid stream) fails to move blob's pointer at all. Added check and seek(0).
*       OK!
*   069 try calling getMotion() from the existing Sampler callback again.
*       OK. Can even increase xctrl_freq to 100 hz no problem.
*   070 poking around 070+...
*   096 getMotion() function v1.1 (completely rewritten)
*       ADC buf overrun once (but I had LOTS of debug logging enbled)
*   097 all debug logging off, increase PROC_FREQ to 10.
*       OK
*   107 increased PROC_FREQ to 100.
*       re-enabled feedback and xsimCTRL output loops
*       ok.
*   108 performances tests (build 107-xxx)
*       PROC_FREQ and XCTRL_FREQ both at imp.wakeup(0.01) and high 65 hz source stream rates (max)
*       ADC Sampler() + getMotion() togeter takes xx ms.
*       getMotion() alone takes only 0.6 ms average (0.34 to 0.9)
*       feedback() loop measures at 20 ms (50 Hz). This is due to imp.wakeup limitation.
*   110 feedback() call also moved inside SamplesReady().
*       OK. 
*       feedback() duration = 0.4 ms (consistent)
*       Sampler() total duration now = 1.9 ms (Sampler @ 100 Hz, 10 ms period).
*       So, could handle ~500 Mhz?
*       400 Hz test:  OK
*   117 BUG:  feedbackLoop():  can't hanle >1 motors. Added hardware.pins (PWM and dir) as elements of motor[] state.
*   128 BUG.117 fixed.
*/

const BUILD = "128";
pwmFreq <- 5000; // PWM pulse frequency, Hz
// proportional feedback gain (1/32768 gives 100% duty cycle when error is half the ADC range):
Kp <- 1.0/32768.0; 

const PROC_FREQ = 200; // main machine update Hz (POTs polled, setpoint grabbed, PID calc'ed, motor outputs updated)
const DASH_FREQ = 20; // dashboard serial output update freq, Hz
const XCTRL_FREQ = 50; // rate we send data back out to XsimCTRL 
const OVERSAMPLING = 10; // no. of ADC readings to average per pin (channel)
const DELIM  = "\x41\x42\xFF"; // XsimCTRL motion stream delimiter
const FIFOSIZE = 80; // size of imp's UART FIFO, bytes

numMotors <- 2; // #of motors of physical machine
DELIM_LEN <- DELIM.len();
frameSize <- DELIM_LEN + numMotors;

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

// alias pin9 as motor1_output for PWM output at 5000 Hz pulse frequency
motor1_output <- hardware.pin9;
motor1_output.configure( PWM_OUT, 1.0/pwmFreq, 0.0 );
//motor[0][7] <- hardware.pin9;
//motor[0][7].configure( PWM_OUT, 1.0/pwmFreq, 0.0 );

// alias pin8 as digital output for motor1_direction
// motor[0][8] <- hardware.pin8;
// motor[0][8].configure( DIGITAL_OUT );
motor1_direction <- hardware.pin8;
motor1_direction.configure( DIGITAL_OUT );
// led <- hardware.pin8;
// led.configure( DIGITAL_OUT );
// ledState <- 0;

// alias pin2 analog input as motor1 POT position
motor1_pot <- hardware.pin2;
motor1_pot.configure(ANALOG_IN);

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
// buffer6 <- blob( 2 * OVERSAMPLING * numMotors);le
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
		  //motor1_direction.write( dir );
		  //motor1_output.write( dutyCycle );
			motor [m][8].write(dir);
			motor [m][7].write( dutyCycle );
		
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
	server.log( "BUILD["+BUILD+"] Motion Frames[" + count_motionFrames +
	                           "] overruns[" + count_motionBufOverruns + 
	                           "] errors[" + count_motionStreamErrors +"]" );
		 // register function to fire once every 1 sec
	imp.wakeup( 1.0 , heartbeat );
}

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

// (UNUSED) function to echo received serial data to server log
/*function serialData() {
	
	// Read the UART for data sent by PC
	local b = serial.read();
	while(b != -1) {
		// As long as UART read value is not -1, we're getting data
		local str = format("SERIAL ECHO: %c", b);
		server.log( str );
		b = serial.read();
	}
	
}
*/

// start the loops
heartbeat();
hardware.sampler.start();
// called inside Sampler instead feedbackLoop();
if (dashboardOn) dashboardOut();
if (xsimctrlOn) xsimctrlOut();