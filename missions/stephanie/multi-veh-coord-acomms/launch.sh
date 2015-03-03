#!/bin/bash 
#-------------------------------------------------------
#  Part 1: Check for and handle command-line arguments
#-------------------------------------------------------
TIME_WARP=1
JUST_MAKE="no"
NUM_VEHICLES=2

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
    elif [ "${ARGI}" = "--2auvs" ] ; then
        NUM_VEHICLES=3
    elif [ "${ARGI}" = "--3auvs" ] ; then
        NUM_VEHICLES=4
    else 
        printf "Bad Argument: %s \n" $ARGI
        exit 0
    fi
done

#-------------------------------------------------------
#  Part 2: Create the .moos and .bhv files. 
#-------------------------------------------------------
# simulation set-up
EXP_LOCATION="santafe" # santafe, arrowhead
USE_LEADER_FOLLOWER="false"
# inter-vehicle distance for formation (adaptive only, not for lf)
IVD="25" # 50 for 2auvs, 25 for 3auvs
USE_HUNGARIAN_METHOD="true" # true, false = static assignment at start

SERVERHOST="localhost" #"localhost"
nsplug meta_shoreside.moos targ_shoreside.moos -f WARP=$TIME_WARP \
   VNAME="shoreside" USC_DATA_DIR="$MOOSIVP_USC_HOME/data"        \
   SHARE_LISTEN="9300" VPORT="9000" SERVER_HOST=$SERVERHOST       \
   LOCATION=$EXP_LOCATION

# START HEADING same for all vehicles - can be customized (not needed here)
START_HEADING="230"

VNAME1="anna"        # The first  vehicle community
START_DEPTH1="0"
if [ "${EXP_LOCATION}" = "santafe" ] ; then
START_POS1="1450,275"
WAYPOINTS1="1355,220:1235,165:1180,130:1120,160:1190,200:1280,250:1385,290:1330,300:1160,255:1110,300:1065,350:1080,415:950,400:940,310:1075,330:1150,255:1490,295"
elif [ "${EXP_LOCATION}" = "arrowhead" ] ; then
START_POS1="2700,1900"
WAYPOINTS1="2600,2000:2450,2050:2280,2030:2340,2080:2495,2085:2495,2130:2365,2130:2365,2190:2470,2170:2510,2230:2555,2200:2555,2050:2600,2050:2600,2180:2665,2180:2665,2050:2700,2050:2700,2230:2710,2290:2760,2330:2760,2000"
fi
SENSOR_RANGE=30

#format=lawnmower,label=science_survey,x=2700,y=1900,width=450,height=500,lane_width=50,rows=north-south,degs=0" #,startx=2750,starty=1900
MODEMID1="1"
VTYPE1="SHIP" # UUV, SHIP
nsplug meta_vehicle.moos targ_$VNAME1.moos -f WARP=$TIME_WARP  \
   VNAME=$VNAME1  START_POS=$START_POS1  START_HDG=$START_HEADING \
   VPORT="9001"       SHARE_LISTEN="9301"                      \
   VTYPE=$VTYPE1      MODEMID=$MODEMID1                        \
   IVD=$IVD    	      SERVER_HOST=$SERVERHOST                  \
   USC_DATA_DIR="$MOOSIVP_USC_HOME/data"  LEAD_NAME=$VNAME1    \
   LOCATION=$EXP_LOCATION  LEAD_SENSOR_RANGE=$SENSOR_RANGE     \
   NR_VEHICLES=$NUM_VEHICLES  HUNGARIAN_METHOD=$USE_HUNGARIAN_METHOD
nsplug meta_vehicle.bhv targ_$VNAME1.bhv -f VNAME=$VNAME1      \
    START_POS=$START_POS1 WAYPOINTS=$WAYPOINTS1                \
    START_DEPTH=$START_DEPTH1 VTYPE=$VTYPE1 LEADER_FOLLOWER="false"

VNAME2="ferdinand"      # The second vehicle community
if [ "${EXP_LOCATION}" = "santafe" ] ; then
START_POS2="1460,305"
WAYPOINTS2=$START_POS2
elif [ "${EXP_LOCATION}" = "arrowhead" ] ; then
START_POS2="2800,1900"
WAYPOINTS2="2800,1900"
fi
START_DEPTH2="10"
MODEMID2="6"
VTYPE2="UUV" # UUV, SHIP
nsplug meta_vehicle.moos targ_$VNAME2.moos -f WARP=$TIME_WARP  \
   VNAME=$VNAME2  START_POS=$START_POS2  START_HDG=$START_HEADING \
   VPORT="9002"       SHARE_LISTEN="9302"                      \
   VTYPE=$VTYPE2      MODEMID=$MODEMID2                        \
   IVD=$IVD           SERVER_HOST=$SERVERHOST                  \
   LEAD_NAME=$VNAME1  LEADER_FOLLOWER=$USE_LEADER_FOLLOWER     \
   LOCATION=$EXP_LOCATION  LEAD_SENSOR_RANGE=$SENSOR_RANGE     \
   NR_VEHICLES=$NUM_VEHICLES  HUNGARIAN_METHOD=$USE_HUNGARIAN_METHOD
nsplug meta_vehicle.bhv targ_$VNAME2.bhv -f VNAME=$VNAME2      \
    START_POS=$START_POS2 WAYPOINTS=$WAYPOINTS2                \
    START_DEPTH=$START_DEPTH2 VTYPE=$VTYPE2 LEAD_NAME=$VNAME1  \
    LEADER_FOLLOWER=$USE_LEADER_FOLLOWER

if [ $NUM_VEHICLES -ge 3 ] ; then
VNAME3="gerard"     # The third vehicle community
if [ "${EXP_LOCATION}" = "santafe" ] ; then
START_POS3="1475,295"
WAYPOINTS3=$START_POS3
elif [ "${EXP_LOCATION}" = "arrowhead" ] ; then
START_POS3="2850,1900"
WAYPOINTS3="2850,1900"
fi
START_DEPTH3="10"
MODEMID3="7"
VTYPE3="UUV"
nsplug meta_vehicle.moos targ_$VNAME3.moos -f WARP=$TIME_WARP  \
   VNAME=$VNAME3  START_POS=$START_POS3  START_HDG=$START_HEADING \
   VPORT="9003"       SHARE_LISTEN="9303"                      \
   VTYPE=UUV          MODEMID=$MODEMID3                        \
   IVD=$IVD           SERVER_HOST=$SERVERHOST                  \
   LEAD_NAME=$VNAME1  LEADER_FOLLOWER=$USE_LEADER_FOLLOWER     \
   LOCATION=$EXP_LOCATION  LEAD_SENSOR_RANGE=$SENSOR_RANGE     \
   NR_VEHICLES=$NUM_VEHICLES  HUNGARIAN_METHOD=$USE_HUNGARIAN_METHOD
nsplug meta_vehicle.bhv targ_$VNAME3.bhv -f VNAME=$VNAME3      \
    START_POS=$START_POS3 WAYPOINTS=$WAYPOINTS3                \
    START_DEPTH=$START_DEPTH3 VTYPE=$VTYPE3 LEAD_NAME=$VNAME1  \
    LEADER_FOLLOWER=$USE_LEADER_FOLLOWER
fi

if [ $NUM_VEHICLES -ge 4 ] ; then
VNAME4="hendrik"     # The third vehicle community
if [ "${EXP_LOCATION}" = "santafe" ] ; then
START_POS4="1485,285"
WAYPOINTS4=$START_POS4
elif [ "${EXP_LOCATION}" = "arrowhead" ] ; then
START_POS4="2900,1900"
WAYPOINTS4="2900,1900"
fi
START_DEPTH4="10"
MODEMID4="8"
VTYPE4="UUV"
nsplug meta_vehicle.moos targ_$VNAME4.moos -f WARP=$TIME_WARP  \
   VNAME=$VNAME4  START_POS=$START_POS4  START_HDG=$START_HEADING \
   VPORT="9004"       SHARE_LISTEN="9304"                      \
   VTYPE=UUV          MODEMID=$MODEMID4                        \
   IVD=$IVD           SERVER_HOST=$SERVERHOST                  \
   LEAD_NAME=$VNAME1  LEADER_FOLLOWER=$USE_LEADER_FOLLOWER     \
   LOCATION=$EXP_LOCATION  LEAD_SENSOR_RANGE=$SENSOR_RANGE     \
   NR_VEHICLES=$NUM_VEHICLES  HUNGARIAN_METHOD=$USE_HUNGARIAN_METHOD
nsplug meta_vehicle.bhv targ_$VNAME4.bhv -f VNAME=$VNAME4      \
    START_POS=$START_POS4 WAYPOINTS=$WAYPOINTS4                \
    START_DEPTH=$START_DEPTH4 VTYPE=$VTYPE4 LEAD_NAME=$VNAME1  \
    LEADER_FOLLOWER=$USE_LEADER_FOLLOWER
fi


if [ ${JUST_MAKE} = "yes" ] ; then
    exit 0
fi

#-------------------------------------------------------
#  Part 3: Launch the processes
#-------------------------------------------------------
printf "Launching shoreside MOOS Community (WARP=%s) \n"  $TIME_WARP
pAntler targ_shoreside.moos > log_shoreside.log &

printf "Launching $VNAME1 MOOS Community (WARP=%s) \n" $TIME_WARP
pAntler targ_$VNAME1.moos > log_$VNAME1.log &
#>& /dev/null &
sleep .25

printf "Launching $VNAME2 MOOS Community (WARP=%s) \n" $TIME_WARP
pAntler targ_$VNAME2.moos > log_$VNAME2.log &
sleep .25

if [ $NUM_VEHICLES -ge 3 ] ; then
printf "Launching $VNAME3 MOOS Community (WARP=%s) \n" $TIME_WARP
pAntler targ_$VNAME3.moos > log_$VNAME3.log &
sleep .25
fi

if [ $NUM_VEHICLES -ge 4 ] ; then
printf "Launching $VNAME4 MOOS Community (WARP=%s) \n" $TIME_WARP
pAntler targ_$VNAME4.moos > log_$VNAME4.log &
sleep .25
fi

printf "Done \n"

uMAC targ_shoreside.moos

printf "Killing all processes ... \n"
kill %1 %2 %3 %4
printf "Done killing processes.   \n"
