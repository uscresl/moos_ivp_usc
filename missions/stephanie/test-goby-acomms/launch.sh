#!/bin/bash 
#-------------------------------------------------------
#  Part 1: Check for and handle command-line arguments
#-------------------------------------------------------
TIME_WARP=1
JUST_MAKE="no"

for ARGI; do
    if [ "${ARGI}" = "--help" -o "${ARGI}" = "-h" ] ; then
        printf "%s [SWITCHES] [time_warp]   \n" $0
        printf "  --just_make, -j    \n" 
        printf "  --help, -h         \n" 
        exit 0;
    elif [ "${ARGI//[^0-9]/}" = "$ARGI" -a "$TIME_WARP" = 1 ]; then 
        TIME_WARP=$ARGI
    elif [ "${ARGI}" = "--just_build" -o "${ARGI}" = "-j" ] ; then
        JUST_MAKE="yes"
    else 
        printf "Bad Argument: %s \n" $ARGI
        exit 0
    fi
done

#-------------------------------------------------------
#  Part 2: Create the .moos and .bhv files. 
#-------------------------------------------------------
nsplug shorebroker.moos targ_shorebroker.moos WARP=$TIME_WARP \
VNAME="ufld_shorebroker" USC_DATA_DIR="$MOOSIVP_USC_HOME/data"   \
VPORT="9000" SHARE_LISTEN="9300"

nsplug topside.moos targ_topside.moos -f WARP=$TIME_WARP \
   VNAME="topside" USC_DATA_DIR="$MOOSIVP_USC_HOME/data"   \
   VPORT="9001" SHARE_LISTEN="9301" MODEMID="1"

VNAME1="auv1"        # The first  vehicle community
START_POS1="50,50"
START_DEPTH1="0"
START_SPEED1="1.0"
WAYPOINTS1="60,-40:60,-160:150,-160:180,-100:150,-40"
MODEMID1="2"
VTYPE1="UUV" # UUV, SHIP
nsplug auv.moos targ_$VNAME1.moos -f WARP=$TIME_WARP  \
   VNAME=$VNAME1      START_POS=$START_POS1           \
   VPORT="9002"       SHARE_LISTEN="9302"             \
   VTYPE=$VTYPE1          MODEMID=$MODEMID1
nsplug auv.bhv targ_$VNAME1.bhv -f VNAME=$VNAME1      \
    START_POS=$START_POS1 WAYPOINTS=$WAYPOINTS1       \
    START_DEPTH=$START_DEPTH1 START_SPD=$START_SPEED1 \
    VTYPE=$VTYPE1

if [ ${JUST_MAKE} = "yes" ] ; then
    exit 0
fi

#-------------------------------------------------------
#  Part 3: Launch the processes
#-------------------------------------------------------
printf "Launching ufld_shorebroker MOOS Community (WARP=%s) \n" $TIME_WARP
pAntler targ_shorebroker.moos > log_shorebroker.log &
#>& /dev/null &
sleep .25

printf "Launching $VNAME1 MOOS Community (WARP=%s) \n" $TIME_WARP
pAntler targ_$VNAME1.moos > log_$VNAME1.log &
#>& /dev/null &
sleep .25

printf "Launching topside MOOS Community (WARP=%s) \n"  $TIME_WARP
pAntler targ_topside.moos > log_topside.log &
printf "Done \n"

uMAC targ_topside.moos

printf "Killing all processes ... \n"
kill %1 %2 %3 %4
printf "Done killing processes.   \n"
