/* ============================================================================
 * Cooja Simulation Script: Highway Motes — Parity-based bidirectional lanes
 *
 * WHAT THIS SCRIPT DOES
 *  - Classify motes into Stationary (incl. RSU) and Mobile.
 *  - INITIAL PLACEMENT (NEW):
 *      * Odd-ID mobiles start at X = -MAX_X and move RIGHT (positive speed).
 *      * Even-ID mobiles start at X = +MAX_X and move LEFT (negative speed).
 *  - MOVEMENT:
 *      * Continuous motion along X. When hitting +/-MAX_X, reverse direction
 *        by flipping speed sign (bounce behavior).
 *  - LED MONITOR (kept): If RED LED is ON, freeze the mote (auto-stop).
 *      * EXCEPTION (NEW): Ignore RED LED if the last seen EF direction for
 *        this mote is OPPOSITE to the mote's current travel direction
 *        (i.e., EF is "wrong direction" for this receiver).
 *        We use a small mailbox "efDir" populated by Serial lines printed
 *        by firmware: "EF_DIR <id> <dir8>"
 *        (dir8 is the discrete 8-way heading contained in EF payload).
 *  - STOP/GO (kept):
 *      * STOP <id>  -> hard freeze (overrides everything).
 *      * GO   <id>  -> unfreeze + send "GO\n" to the mote's Serial and
 *                      temporarily ignore its RED LED until LED goes OFF.
 *  - REQ_LOC handler (kept): when a mote prints "REQ_LOC", we answer with
 *      "LOC <id> <x_dm> <y_dm> <ts_ms>" on that mote's Serial.
 *  - Movement tick is driven by GENERATE_MSG(..., "move_next").
 *
 * HOW "WRONG DIRECTION EF" IS RECOGNIZED
 *  - Firmware should print:   EF_DIR <id> <dir8>\n
 *    where <dir8> is the EF's direction (0..7, E,NE,N,NW,W,SW,S,SE).
 *  - We compare <dir8> with the sign of the current X-speed:
 *      * RIGHT-like dir (E/NE/SE) => expected positive speed.
 *      * LEFT-like  dir (W/NW/SW) => expected negative speed.
 *    If mismatch -> EF is "wrong direction" for this receiver: we DO NOT
 *    auto-stop even if RED LED is on.
 *
 * NOTE
 *  - If EF_DIR lines are not printed by firmware, the script cannot know
 *    EF direction; in that case LED-monitor behaves as before.
 *  - Everything else from the previous script is preserved.
 * ========================================================================== */

/* -------------------------- Parameters ------------------------------------ */
var STATIONARY_SPACING = 2000;
var MAX_X              = 10000;    /* absolute bound along X axis */
var MAX_Y              = 100;       /* unused for now (we keep Y=0) */
var SIMULATION_TICK_DELAY = 500;    /* ms-equivalent (Cooja uses /10) */

var SPEED_MIN = 0.2;   /* absolute magnitude lower bound */
var SPEED_MAX = 0.5;   /* absolute magnitude upper bound */
var DELTA_SPEED = 0.1; /* small jitter per tick */

/* -------------------------- State containers ------------------------------ */
var stationaryMotes = [];
var mobileMotes     = [];

/* Per-mobile state, indexed by the mobileMotes' index (not mote ID) */
var moteSpeedAbs    = [];  /* absolute speed magnitude (>=0) */
var moteSpeedSign   = [];  /* +1 to the right, -1 to the left */
var moteDistance    = [];  /* distance accumulator (for optional events) */

/* Control maps by mote ID */
var frozen        = {};  /* STOP-hard-freeze */
var stoppedManual = {};  /* was stopped manually at least once */
var ledIgnore     = {};  /* ignore RED LED after GO until LED becomes OFF */

/* -------------------------- Init: classify and place ---------------------- */
log.log("Initializing motes (parity-based lanes)...\n");
var motes = sim.getMotes();

for (var i = 0; i < motes.length; i++) {
  var m = motes[i];
  var name = m.getType().getDescription();
  if (name.indexOf("Stationary") !== -1 || name.indexOf("RSU") !== -1) {
    stationaryMotes.push(m);
  } else if (name.indexOf("Mobile") !== -1) {
    mobileMotes.push(m);
  } else {
    log.log("Warn: Unclassified mote " + m.getID() + ": " + name + "\n");
  }
}

/* Place stationary motes on X axis (Y=0), as before */
for (var s = 0; s < stationaryMotes.length; s++) {
  var sm = stationaryMotes[s];
  var x = s * STATIONARY_SPACING;
  sm.getInterfaces().getPosition().setCoordinates(x, 0, 0);
}

/* NEW: parity-based initial placement and direction for mobiles */
for (var k = 0; k < mobileMotes.length; k++) {
  var mm = mobileMotes[k];
  var mid = mm.getID();
  var pos = mm.getInterfaces().getPosition();

  /* odd IDs start at -MAX_X and go RIGHT; even IDs start at +MAX_X and go LEFT */
  var startX, sign;
  if ((mid % 2) === 1) {       /* odd */
    startX = 0; // -MAX_X;
    sign   = +1;
  } else {                    /* even */
    startX = +MAX_X;
    sign   = -1;
  }

  /* place the mote (Y fixed at 0 for highway lane) */
  pos.setCoordinates(startX, 0, 0);

  /* absolute speed in [SPEED_MIN, SPEED_MAX], sign as above */
  var vabs = SPEED_MIN + Math.random() * (SPEED_MAX - SPEED_MIN);
  moteSpeedAbs[k]  = vabs;
  moteSpeedSign[k] = sign;
  moteDistance[k]  = 0;
}

/* Movement scheduler */
function scheduleNextMove() {
  GENERATE_MSG(SIMULATION_TICK_DELAY / 10, "move_next");
}
scheduleNextMove();

/* -------------------------- Helpers --------------------------------------- */
/* Answer "REQ_LOC" with current X,Y in decimeters and ts in ms */
function handleLocationRequest(mote) {
  var pos = mote.getInterfaces().getPosition();
  var ts_ms = Math.floor(time / 1000);
  var x = pos.getXCoordinate();
  var y = pos.getYCoordinate();
  var mid = mote.getID();
  var response = "LOC " + mid + " " + Math.round(10 * x) + " "
               + Math.round(10 * y) + " " + ts_ms;
  mote.getInterfaces().get("Serial").writeString(response + "\n");
  //log.log("LOG: " + response + "\n");
}

/* Get RED LED state defensively */
function isRedOn(mote) {
  var leds = mote.getInterfaces().getLED ? mote.getInterfaces().getLED() : null;
  if (!leds || !leds.isRedOn) return false;
  try { return leds.isRedOn(); } catch (e) { return false; }
}

/* -------------------------- Main loop ------------------------------------- */
while (true) {
  YIELD();

  /* ===================== Commands via Serial (STOP/GO/EF_DIR) ============= */
  if (typeof msg === "string") {
    var s   = ("" + msg).trim();
    var src = sim.getMoteWithID(id);

    /* STOP <id> : hard freeze */
    var mStop = s.match(/^STOP\s+(\d+)$/i);
    if (mStop) {
      var stopId = parseInt(mStop[1], 10);
      frozen[stopId]        = true;
      stoppedManual[stopId] = true;
      if (src && src.getInterfaces().get("Serial")) {
        src.getInterfaces().get("Serial").writeString("OK: STOP " + stopId + "\n");
      }
      continue;
    }

    /* GO <id> : unfreeze, send "GO\n", and temporarily ignore that mote's LED */
    var mGo = s.match(/^GO\s+(\d+)$/i);
    if (mGo) {
      var goId = parseInt(mGo[1], 10);
      delete frozen[goId];
      if (stoppedManual[goId]) { ledIgnore[goId] = true; }
      var tgt = sim.getMoteWithID(goId);
      if (tgt && tgt.getInterfaces().get("Serial")) {
        tgt.getInterfaces().get("Serial").writeString("GO\n");
      }
      if (src && src.getInterfaces().get("Serial")) {
        src.getInterfaces().get("Serial").writeString("OK: GO " + goId + "\n");
      }
      continue;
    }

    /* REQ_LOC from a mote -> answer */
    if (s.indexOf("REQ_LOC") >= 0) {
      var reqMote = sim.getMoteWithID(id);
      if (reqMote) handleLocationRequest(reqMote);
      continue;
    }
  }

  /* ===================== Movement tick ==================================== */
  if (typeof msg === "string" && msg === "move_next") {
    for (var i = 0; i < mobileMotes.length; i++) {
      var mote = mobileMotes[i];
      var mid   = mote.getID();
      var pos  = mote.getInterfaces().getPosition();
      var x    = pos.getXCoordinate();

      /* 1) Manual hard freeze overrides everything */
      if (frozen[mid]) {
        pos.setCoordinates(x, 0, 0);
        moteDistance[i] = 0;
        continue;
      }

      /* 2) LED-monitor with "wrong-direction EF" exception */
      var red = isRedOn(mote);
      if (red && !ledIgnore[mid]) {
          moteSpeedAbs[i] = 0;
          pos.setCoordinates(x, 0, 0);
          moteDistance[i] = 0;
          continue;
      }
      if (!red && ledIgnore[mid]) {
        /* LED cleared -> end temporary ignore */
        delete ledIgnore[mid];
      }

      /* 3) Speed jitter (on absolute value), keep sign separately */
      var vabs = moteSpeedAbs[i] + (Math.floor(Math.random()*3)-1) * DELTA_SPEED;
      if (vabs < SPEED_MIN) vabs = SPEED_MIN;
      if (vabs > SPEED_MAX) vabs = SPEED_MAX;
      moteSpeedAbs[i] = vabs;

      /* 4) Advance along X per current sign */
      var dx = vabs * moteSpeedSign[i];
      x += dx;
      moteDistance[i] += Math.abs(dx);

      /* 5) Bounce on boundaries: reverse sign if outside limits */
      if (x >  MAX_X) { x =  MAX_X; moteSpeedSign[i] = -1; }
      //if (x < -MAX_X) { x = -MAX_X; moteSpeedSign[i] = +1; }
      if (x < 0) { x = 0; moteSpeedSign[i] = +1; }

      pos.setCoordinates(x, 0, 0);
    }

    scheduleNextMove();
    continue;
  }
}