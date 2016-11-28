How to run these bench tests:

These are very simple tests to see if the iOceanServerComms process is working
properly, and the fins react to the commands given. We do not attempt to move
the propeller, because that's not very healthy out of the water.

To run, on the vehicle:

--------------------------------------------------------------------------------
BACKSEAT

1. Open two terminal connections to the EcoMapper's backseat (zoomer).

--------------------------------------------------------------------------------
FRONTSEAT

2. In order to see the pitch fins move, you need to change 2 things in 
   UVC:
   * under Setup/Safety Rules, uncheck the 'Minimum height off the Bottom...'
   * under Setup/Mission, 
      check 'Run in Test Mode'

--------------------------------------------------------------------------------
BACKSEAT

3. Go into the backseat, then in moos-ivp-usc/missions/iver_benchtest run: 
 $ pAntler coordinated_spdhdgdep.moos

4. Check that everything starts correctly (i.e. no processes died)

5. In another terminal (tab), do:
 $ uPokeDB coordinated_spdhdgdep.moos VEHICLE_UNDERWAY=true

Wait a bit to make sure everything runs fine. You can check that
DESIRED_HEADING and DESIRED_DEPTH are being published to the MOOSDB.
E.g. $ uXMS coordinated_spdhdgdep.moos DESIRED_HEADING DESIRED_DEPTH DESIRED_SPEED

(Note, you do not want to be commanding speed for the bench test)

--------------------------------------------------------------------------------
FRONTSEAT

6. Load a backseat mission (eg. park_for_benchtest).
 a. make sure the settings have been adapted (see 2.)
 b. start the mission

7. You should see on the frontseat that the Vehicle State switches to 
   MANUAL SERVO (magenta higlight).

Then you should see the fins move to set positions. The propeller should NOT move.
If it does, IMMEDIATELY stop the frontseat mission!!

To double-check what values were published and what iOceanServerComms commanded,
compare DESIRED_DEPTH and DESIRED_HEADING with BACKSEAT_TX ($OMS string)

8. End the benchtest by first stopping the frontseat mission, then ending
   the backseat mission

--------------------------------------------------------------------------------
RESET FRONTSEAT SETTINGS

When you are done with the bench test, make sure to switch the UVC settings back
to what they were!! (uncheck test mode, check min HFB)
