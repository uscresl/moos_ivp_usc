How to run these bench tests:

These are very simple tests to see if the iOceanServerComms process is working
properly, and the fins react to the commands given. We do not attempt to move
the propeller, because that's not very healthy out of the water.

To run, on the vehicle:

1. load & start a mission for backseat on the ecomapper frontseat, in UVC

2. go into the backseat, then in moos-ivp-usc/missions/iver_benchtest: 

$ pAntler random_spdhdgdep.moos

If everything has started correctly (no processes die on you), 
in another terminal (tab), do:
$ uPokeDB random_spdhdgdep.moos VEHICLE_UNDERWAY=true

This should move the fins to random positions. The propeller should NOT move.
If it does, either stop the frontseat missions, or immediately send:
$ uPokeDB random_spdhdgdep.moos VEHICLE_UNDERWAY=false

