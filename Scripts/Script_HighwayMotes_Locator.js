// Constants for positioning and movement
var STATIONARY_SPACING = 500;
var MOBILE_RECT_HEIGHT = 1;
var MOBILE_SPACING_MIN = 0.1;
var MOBILE_SPACING_MAX = 0.3;
var DELTA_SPEED = 0.1;
var SPEED_MIN = 0.2;
var SPEED_MAX = 0.5;
var MAX_X = 100000;
var MAX_Y = 100;
var DIRECTION_CHANGE_INTERVAL = 500;
var SIMULATION_TICK_DELAY = 500;
var VERTICAL_DISTANCE = 20;

// Arrays to hold motes and their attributes
var stationaryMotes = [];
var mobileMotes = [];
var moteSpeeds = [];
var moteDirections = [];
var moteDistances = [];
var moteFlags = [];

// NEW: frozen map (mote ID -> true) for STOP/GO
var frozen = {};

// Flags for mobile mote positions
var POSITION_MIDDLE = "MIDDLE";
var POSITION_UP = "UP";
var POSITION_DOWN = "DOWN";
var DIRECTION_TO_MIDDLE = "TO_MIDDLE";
var FLAG_RETURNING = "RETURNING";

// Initialization of motes
log.log("Initializing motes...\n");
var motes = sim.getMotes();
log.log("Got all the motes: " + motes.length + "\n");
for (var i = 0; i < motes.length; i++) {
  var mote = motes[i];
  var name = mote.getType().getDescription();
  if (name.indexOf("Static") !== -1) {
    stationaryMotes.push(mote);
  } else if (name.indexOf("Mobile") !== -1) {
    mobileMotes.push(mote);
  } else {
    log.log("Error: Mote " + i + " has invalid name: " + name + "\n");
//    sim.stopSimulation();
//    break;
  }
}

// Position stationary motes
for (var i = 0; i < stationaryMotes.length; i++) {
  var mote = stationaryMotes[i];
  var x = i * STATIONARY_SPACING;
  var y = 0;
  mote.getInterfaces().getPosition().setCoordinates(x, y, 0);
}

// Position mobile motes and assign attributes
var xOffset = 0;
for (var i = 0; i < mobileMotes.length; i++) {
  var mote = mobileMotes[i];
  var spacing = MOBILE_SPACING_MIN + Math.random() * (MOBILE_SPACING_MAX - MOBILE_SPACING_MIN);
  xOffset += spacing;
  var yOffset = Math.random() * MOBILE_RECT_HEIGHT;
  mote.getInterfaces().getPosition().setCoordinates(xOffset, yOffset, 0);
  // keep original behavior:
  var speed = SPEED_MIN + Math.floor(Math.random() * (SPEED_MAX - SPEED_MIN));
  moteSpeeds.push(speed);
  moteDirections.push("RIGHT");
  moteDistances.push(0);
  moteFlags.push(POSITION_MIDDLE);
}

// Function to schedule next movement
function scheduleNextMove() {
  GENERATE_MSG(SIMULATION_TICK_DELAY / 10, "move_next");
}
scheduleNextMove();

function handleLocationRequest(mote) {
  var pos = mote.getInterfaces().getPosition();
  var ts_ms = Math.floor(time/1000);
  var x = pos.getXCoordinate();
  var y = pos.getYCoordinate();
  var id = mote.getID();
  var response = "LOC " + id + " " + Math.round(10*x) + " " + Math.round(10*y) + " " + ts_ms;
  mote.getInterfaces().get("Serial").writeString(response + "\n");
}

// Main loop for movement
while (true) {
  YIELD();

  // --- Command parsing: STOP <id> / GO <id> with serial echo --------
  if (typeof msg === "string") {
    var s = ("" + msg).trim();
    // emit echo to the sender mote (if any)
    var src = sim.getMoteWithID(id);

    // STOP <id>
    var mStop = s.match(/^STOP\s+(\d+)$/i);
    if (mStop) {
      var stopId = parseInt(mStop[1], 10);
      frozen[stopId] = true;
      if (src && src.getInterfaces().get("Serial")) {
        src.getInterfaces().get("Serial").writeString("OK: STOP " + stopId + "\n");
      }
      continue;
    }

    // GO <id>
    var mGo = s.match(/^GO\s+(\d+)$/i);
    if (mGo) {
      var goId = parseInt(mGo[1], 10);
      delete frozen[goId];
      if (src && src.getInterfaces().get("Serial")) {
        src.getInterfaces().get("Serial").writeString("OK: GO " + goId + "\n");
      }
      continue;
    }
  }

  // Respond to UART request from any mote
  if (typeof msg === "string" && msg.indexOf("REQ_LOC") >= 0) {
    var reqMote = sim.getMoteWithID(id);
    if (reqMote != null) {
      handleLocationRequest(reqMote);
    }
    continue;
  }

  // Tick for movement
  if (typeof msg === "string" && msg === "move_next") {
    for (var i = 0; i < mobileMotes.length; i++) {
      var mote = mobileMotes[i];
      var pos = mote.getInterfaces().getPosition();
      var x = pos.getXCoordinate();
      var y = pos.getYCoordinate();

      // A) STOP/GO override (frozen motes do not move)
      var moteId = mote.getID();
      if (frozen[moteId]) {
        moteSpeeds[i] = 0;
        pos.setCoordinates(x, y, 0);
        moteDistances[i] = 0;
        continue;
      }

      // B) LED monitor: RED ON -> stop; RED OFF & base speed==0 -> resume with random base
      var leds = mote.getInterfaces().getLED ? mote.getInterfaces().getLED() : null;
      var redOn = false;
      if (leds && leds.isRedOn) {
        try { redOn = leds.isRedOn(); } catch (e) { redOn = false; }
      }
      if (redOn) {
        moteSpeeds[i] = 0;
        pos.setCoordinates(x, y, 0);
        moteDistances[i] = 0;
        continue;
      } else {
        if (moteSpeeds[i] === 0) {
          moteSpeeds[i] = SPEED_MIN + Math.random() * (SPEED_MAX - SPEED_MIN);
        }
      }

      // Original speed update with jitter
      var speed = moteSpeeds[i] + (Math.floor(Math.random() * 3) - 1)*DELTA_SPEED;
      if (speed < SPEED_MIN) speed = SPEED_MIN;
      if (speed > SPEED_MAX) speed = SPEED_MAX;

      var direction = moteDirections[i];
      if (direction === "RIGHT") {
        x += speed;
      } else if (direction === "UP") {
        y += speed;
        if (y <= 0) {
          y = 0;
          moteDirections[i] = "RIGHT";
          moteFlags[i] = POSITION_MIDDLE;
        }
      } else if (direction === "DOWN") {
        y -= speed;
        if (y >= 0) {
          y = 0;
          moteDirections[i] = "RIGHT";
          moteFlags[i] = POSITION_MIDDLE;
        }
      } else if (direction === DIRECTION_TO_MIDDLE) {
        if (y > 0) {
          y -= speed;
          if (y <= speed) {
            y = 0;
            moteDirections[i] = "RIGHT";
            moteFlags[i] = POSITION_MIDDLE;
          }
        } else if (y < 0) {
          y += speed;
          if (y >= -speed) {
            y = 0;
            moteDirections[i] = "RIGHT";
            moteFlags[i] = POSITION_MIDDLE;
          }
        }
      }

      moteDistances[i] += speed;

      // Enforce map boundaries
      if (y > MAX_Y) {
        y = MAX_Y;
        x += speed;
        moteDirections[i] = "RIGHT";
        moteFlags[i] = POSITION_UP;
      }
      if (x < 0) x = 0;
      if (y < -MAX_Y) {
        y = -MAX_Y;
        x += speed;
        moteDirections[i] = "RIGHT";
        moteFlags[i] = POSITION_DOWN;
      }

      pos.setCoordinates(x, y, 0);
    }

    // Each mote changes direction individually after traveling enough distance
    for (var i = 0; i < mobileMotes.length; i++) {
      if (moteDistances[i] >= DIRECTION_CHANGE_INTERVAL) {
        var newDirection = moteDirections[i];

        if (moteFlags[i] === POSITION_MIDDLE) {
          newDirection = Math.random() < 0.99 ? "RIGHT" : (Math.random() < 0.5 ? "UP" :"DOWN");
        } else if (moteFlags[i] === POSITION_UP || moteFlags[i] === POSITION_DOWN) {
          newDirection = DIRECTION_TO_MIDDLE;
          moteFlags[i] = FLAG_RETURNING;
        } else if (moteFlags[i] === FLAG_RETURNING) {
          // no change until center
        }

        if (moteDirections[i] !== newDirection) {
          moteDirections[i] = newDirection;
          moteDistances[i] = 0;
        }
      }
    }

    scheduleNextMove();
    continue;
  }
}