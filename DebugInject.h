#ifndef DEBUGINJECT_H
#define DEBUGINJECT_H

// serial test hooks, usable without any phone:
//   bg <mgdl> [angle]   inject a reading (angle -90..90, 180 = hidden)
//   demo                inject a demo curve into the history
//   time <h> <m>        set the clock (today's date if unknown)
//   status              print state
void debugInjectPoll();

#endif
