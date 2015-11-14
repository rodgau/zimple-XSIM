/** (Agent-side code)
 *
 * TO-DO:
 * 
 * -add code to keep configuration app's display in sync with agent's cached settings
 * 
*/

controllerPrefs <- {}; // table for server-side cache of controler preferences

//------------- function to initialize controller preferences table with factory defaults
function _initFactory()
{
  server.log("Initializing controller preferences cache with factory defaults ...");
  controllerPrefs.clear();
  controllerPrefs.Kp <- 0.0; // reload all slots with factory defaults
  controllerPrefs.Ki <- 0.0;
  controllerPrefs.Kd <- 0.0;
  controllerPrefs.Kp_lng <- 0.0; // (last known goods)
  controllerPrefs.Ki_lng <- 0.0;
  controllerPrefs.Kd_lng <- 0.0;
}

//------------------- for testing only (null table -> cache) --------------
function _wipeCache()
{
  server.log("Writing null table to cache...");
  controllerPrefs.clear();
}

//------------- function to restore controller preferences table from imp Cloud
function reloadPrefs()
{
  server.log("Searching imp Cloud for user preferences...");
  local table = server.load(); // cache<-imp Cloud
  if (table.len() != 0) 
  {
    server.log("User preferences found. Controller cache reloaded.");
    controllerPrefs = table; // cache<-imp Cloud (if a previous save exist)
  }
  else
    server.log("No user preferences found.");
  
  local error = server.save(controllerPrefs);
}

//-------------- function to scale k for display
function print_k( k )
{
  return (k * 10 * 65535).tointeger();
}

//------------ function to set PID value Kp
function set_Kp(KpScale)
{
  // our app passed a 0-100 value to request a scaled change to Kp
  if ( KpScale > 100.0 ) KpScale = 100.0;
  else if ( KpScale < 0.0 ) KpScale = 0.0;
  return (KpScale/10.0) * (1.0/65535.0); // base K is 1/65535 (means 100% PWM for 100% error (65535)
                                      //  K_scale/10 is reasonable.
  server.log( "Command executed: setKp[" + KpScale + "]");
  server.log( "Kp[" + print_k(Kp) + "]");
}

//------------ function to set PID value Ki
function set_Ki(KiScale)
{
  // our app passed a 0-100 value to request a scaled change to Ki
  if ( KiScale > 100.0 ) KiScale = 100.0;
  else if ( KiScale < 0.0 ) KiScale = 0.0;
  return (KiScale/10.0) * (1.0/65535.0); // base K is 1/65535 (means 100% PWM for 100% error (65535)
                                      //  K_scale/10 is reasonable.
  server.log( "Command executed: setKi[" + KiScale + "]");
  server.log( "Ki[" + print_k(Ki) + "]");
}

//------------ function to set PID value Kd
function set_Kd(KdScale)
{
  // our app passed a 0-100 value to request a scaled change to Kd
  if ( KdScale > 100.0 ) KdScale = 100.0;
  else if ( KdScale < 0.0 ) KdScale = 0.0;
  return (KdScale/10.0) * (1.0/65535.0); // base K is 1/65535 (means 100% PWM for 100% error (65535)
                                      //  K_scale/10 is reasonable.
  server.log( "Command executed: setKd[" + KdScale + "]");
  server.log( "Kd[" + print_k(Kd) + "]");
}

//------------- function to overwrite cached last known good preferences with current
function updateLastKnownGood()
{
  controllerPrefs.Kp_lng = controllerPrefs.Kp;
  controllerPrefs.Ki_lng = controllerPrefs.Ki;
  controllerPrefs.Kd_lng = controllerPrefs.Kd;
  server.log("Cached last known good controller preferences updated with current cached settings.")
}

//------------- function to overwrite cached current preferences with last known good
function revertLastKnownGood()
{
  controllerPrefs.Kp = controllerPrefs.Kp_lng;
  controllerPrefs.Ki = controllerPrefs.Ki_lng;
  controllerPrefs.Kd = controllerPrefs.Kd_lng;
  server.log("Cached controller preferences reverted to cached last known good.")
}


// Log the URLs we need
// server.log("Turn motors On: " + http.agenturl() + "?motorsEnable=1"); // debug
// server.log("Turn motors Off: " + http.agenturl() + "?motorsEnable=0"); // debug

/**------------------- our http requests handler -----------------------------------------
 * 
 * Base URL:  https://agent.electricimp.com/<your_impDevice_ID>
 * URL entry points for above base URL (REST api commands):
 *  ?motorsEnable=0   disable all motors
 *  ?motorsEnable=1   enable all motors
 *  ?settings=burn    take snapshot of current device settings as last known good, and save to imp Cloud
 *  ?settings=revert  revert device settings to last known good
 *  ?settings=factory revert device settings to factory defaults
 *  /pid?Kp=x         set Kp equal to x (Proportional control). x is an integer (0-100)
 *  /pid?Ki=x         set Ki equal to x (Integral control). x is an integer (0-100)
 *  /pid?Kd=x         set Kd equal to x (Derivative control). x is an integer (0-100)
*/
function requestHandler(request, response) {
  try {
    // DEBUG LOGGING
    server.log("http method [" + request.method);
    server.log("http path[" + request.path);
    server.log("http query [" + request.query + "]");
    server.log("http body[" + request.body +"]");
    server.log("http headers[" + request.headers);
    
    local path = request.path.tolower();
    
    if ("settings" in request.query)
    {
      if (request.query.settings == "burn" )
      {
        // take snapshot of current settings as our new last known good
        server.log("Command received: settings=burn" );
        updateLastKnownGood(); // update cache
        server.log("Syncing settings to imp Cloud...");
        local error = server.save(controllerPrefs); // cache->imp Cloud
        // send a response back saying everything was OK.
        response.send(200, "OK");
      }
      else if (request.query.settings == "revert" )
      {
        // revert controller settings to last known good
        server.log("Command received: settings=revert" );
        revertLastKnownGood(); // update cache
        server.log("Syncing settings to imp Cloud...");
        local error = server.save(controllerPrefs); // cache->imp Cloud
        device.send("controller.set.prefs", controllerPrefs); // cache->controller
        // send a response back saying everything was OK.
        response.send(200, "OK");
      }
      else if (request.query.settings == "factory" )
      {
        // restore controller settings to factory defaults
        server.log("Command received: settings=factory" );
        _initFactory(); // factory defaults->cache
        server.log("Syncing settings to imp Cloud...");
        local error = server.save(controllerPrefs); // cache->imp Cloud
        device.send("controller.set.prefs", controllerPrefs); // cache->controller
        // send a response back saying everything was OK.
        response.send(200, "OK");
      }
      else if (request.query.settings == "WIPE" )
      {
        // write empty table into imp Cloud (for testing only)
        server.log("Command received: settings=WIPE" );
        _wipeCache();
        local error = server.save(controllerPrefs); // cache->imp Cloud
        // send a response back saying everything was OK.
        response.send(200, "OK");
      }
      else
      {
        response.send(400, "Invalid query. settings params: burn, revert, factory, WIPE");
      }
    }
    else if ("motorsEnable" in request.query)
    {
      if (request.query.motorsEnable == "1" || request.query.motorsEnable == "0")
      {
        // convert the query parameter to an integer
        local motorState = request.query.motorsEnable.tointeger();
        server.log("Command received: motorsEnable[" + motorState + "]");
    
        // send "motorsEnable" message to device, and send motorState as the data
        device.send("motorsEnable", motorState); 
        // send a response back saying everything was OK.
        response.send(200, "OK");
      }
      else
      {
        response.send(400, "Invalid query. motorsEnable params: 0, 1");
      }
    }
    else if ( path == "/pid" || path == "/pid/" )
    {
      // this entry point means all data's been passed as a text/plain body of key/value pairs (; delimited)
      local got_pid = false;
      local data = http.urldecode(request.body);
      
      if ("Kp" in data)
      {
        local x = data.Kp.tointeger(); // passed value 0-100
        server.log("Command received Kp[" + x + "]");
        controllerPrefs.Kp = set_Kp(x); // update cache with actual Kp
        server.log("Syncing settings to imp Cloud...");
        local error = server.save(controllerPrefs); // cache->imp Cloud
        server.log("Sending setting to controller...")
        device.send("setKp", controllerPrefs.Kp ); // cache->controller
        got_pid = true;
      }
      if ("Ki" in data)
      {
        local x = data.Ki.tointeger(); // passed value 0-100
        server.log("Command received Ki[" + x + "]");
        controllerPrefs.Ki = set_Ki(x); // update cache with actual Ki
        server.log("Syncing settings to imp Cloud...");
        local error = server.save(controllerPrefs); // cache->imp Cloud
        server.log("Sending setting to controller...")
        device.send("setKi", controllerPrefs.Ki ); // cache->controller
        got_pid = true;
      }
      if ("Kd" in data)
      {
        local x = data.Kd.tointeger(); // passed value 0-100
        server.log("Command received Kd[" + x + "]");
        controllerPrefs.Kd = set_Kd(x); // update cache with actual Kd
        server.log("Syncing settings to imp Cloud...");
        local error = server.save(controllerPrefs); // cache->imp Cloud
        server.log("Sending setting to controller...")
        device.send("setKd", controllerPrefs.Kd ); // cache->controller
        got_pid = true;
      }

      if ( got_pid == true )
      {
        response.send(200, "OK");
      }
      else
      {
        //  '/pid' query has no valid body content attached
        response.send(400, "Invalid query. pid text/plain body missing");
      }
    }
  
    // no valid queries in request whatsoever
    else response.send(400, "BAD REQUEST");
  
  } catch (ex)
    {
      response.send(500, "Internal Server Error: " + ex);
    }
}

//------- function to relay ping-pong watchdog between device and agent
function start_time(startTime)
{
// echo the device's ping start as a 'pong' message immediately
device.send("pong", startTime);
}
 
// When we get a 'ping' message from the device, call start_time()
device.on("ping", start_time);

//------- function to give device its preferences from Cloud
function sendPrefs(value)
{
  // controller has requested its current prefs
  server.log("Sending preferences to controller...")
  device.send("controller.set.prefs", controllerPrefs);
}

// Register device-sent request for its prefs
device.on("controller.get.prefs", sendPrefs);



/**---------------- main entry point ---------------------------
 */
 
_initFactory(); // init internal table
reloadPrefs(); // user saved settings (if any) -> controller preferences cache
server.log("Registering HTTP handler...")
http.onrequest(requestHandler); // register the HTTP handler
server.log("Waiting for HTTP requests...")


