How to run these bench tests:

These are very simple tests to see if the iOceanServerComms process is working
properly, and the fins react to the commands given. We do not attempt to move
the propeller, because that's not very healthy out of the water.

To run, on the vehicle:

1. go into the backseat, then in moos-ivp-usc/missions/iver_benchtest run: 
$ pAntler coordinated_spdhdgdep.moos

If everything has started correctly (no processes die on you), 
in another terminal (tab), do:
$ uPokeDB random_spdhdgdep.moos VEHICLE_UNDERWAY=true

wait a second to make sure everything runs fine. You can check that
DESIRED_HEADING and DESIRED_DEPTH are being published to the MOOSDB.

2. load & start a mission for backseat on the ecomapper frontseat, in UVC.

Note that in order to see the pitch fins move, you need to change 2 things in 
UVC:
* under Setup/Safety Rules, uncheck the 'Minimum height off the Bottom...'
* under Setup/Mission, check 'Run in Test Mode'

After you do this, you can start a backseat mission (eg. park_for_backseat).

Then you should see the fins move to set positions. The propeller should NOT move.
If it does, immediately stop the frontseat mission!!

When you are done with the bench test, make sure to switch the UVC settings back
to what they were!! (uncheck test mode, check min HFB)

To double-check what values were published and what iOceanServerComms commanded,
compare DESIRED_DEPTH and DESIRED_HEADING with BACKSEAT_TX ($OMS string)
