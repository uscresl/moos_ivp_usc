#!/bin/bash 
#-------------------------------------------------------
#  Part 1: Check for and handle command-line arguments
#-------------------------------------------------------
TIME_WARP=1
JUST_MAKE="no"
ADAPTIVE="no"
RUN_SIMULATION="yes"
GUI="true"
ADP_START="cross"
USE_LOG_GP="yes"
DATA_DIMENSIONS=3
PP_METHOD="global_max"

for ARGI; do
    if [ "${ARGI}" = "--help" -o "${ARGI}" = "-h" ] ; then
        printf "%s [SWITCHES] [time_warp]   \n" $0
        printf "  Switches:          \n"
        printf "  --just_build, -j    \n" 
        printf "  --adaptive, -a     \n"
        printf "  --nogui, -ng       \n"
        printf "  --cross_pilot, -cp \n"
        printf "  --pp_random, -rand \n"
        printf "  --2d               \n"
        printf "  --help, -h         \n" 
        exit 0;
    elif [ "${ARGI//[^0-9]/}" = "$ARGI" -a "$TIME_WARP" = 1 ]; then 
        TIME_WARP=$ARGI
    elif [ "${ARGI}" = "--just_build" -o "${ARGI}" = "-j" ] ; then
        JUST_MAKE="yes"
    elif [ "${ARGI}" = "--adaptive" -o "${ARGI}" = "-a" ] ; then
        ADAPTIVE="yes"
    elif [ "$ARGI" = "--nogui" -o "${ARGI}" = "-ng" ]; then
        GUI="no"
    elif [ "$ARGI" = "--cross_pilot" -o "${ARGI}" = "-cp" ]; then
        ADP_START="cross"
    elif [ "$ARGI" = "--gp" -o "${ARGI}" = "-g" ]; then
        USE_LOG_GP="no"
    elif [ "$ARGI" = "--2d" ]; then
        DATA_DIMENSIONS=2
    elif [ "$ARGI" = "--pp_random" -o "${ARGI}" = "-rand" ]; then
        PP_METHOD="random"
    else 
        printf "Bad Argument: %s \n" $ARGI
        exit 0
    fi
done

# check if sim data file present
if [ ! -f 'test.csv' ]; then 
echo 'ERROR: No simulated data file presented. Please put a test.csv file in this folder'; exit 0;
fi

#-------------------------------------------------------
#  Part 2: Create the .moos and .bhv files. 
#-------------------------------------------------------
# simulation set-up
EXP_LOCATION="puddingstone" # puddingstone, santafe, arrowhead
PLUGDIR="../../../plugs" # no leading slash
MSGDIR="${MOOSIVP_USC_HOME}/proto"

# paint survey area on pMarineViewer
# orig area
PAINTSEGLIST="pts={500,1200:500,1000:900,1000:900,1200:500,1200},label=survey_area,label_color=white,edge_color=yellow,vertex_color=yellow,vertex_size=3,edge_size=3"
BHVOPREGION="label,OpRegion:400,920:400,1045:480,1215:600,1300:1000,1300:1000,920"

# config for lawnmower for actual GP model building
# orig area
LX=700
LY=1100
LW=400
LH=200

LAWNMOWER1="format=lawnmower,x=${LX},y=${LY},width=${LW},height=${LH},lane_width=20,degs=0,startx=0,starty=0"

if [ "${ADAPTIVE}" = "no" ] ; then
  # lawnmower
  LAWNMOWEREW="$LAWNMOWER1,rows=east-west,label=east-west-survey"
  LAWNMOWERNS="$LAWNMOWER1,rows=north-south,label=north-south-survey"
fi

##### specify for adaptive whether to use integrated cross or random pilot #####
if [ "${ADAPTIVE}" = "yes" ] && [ "${ADP_START}" = "cross" ] ; then
  # 1auv cross
  PILOT_PTS1=500,1000:900,1200:500,1200:900,1000
  CROSS_END1=900,1000 # last wpt
fi

if [ "${ADAPTIVE}" = "yes" ] && [ "${ADP_START}" = "random" ] ; then
  # 1auv random
  randpts=$(perl -le 'print map { 500+int(rand(400)), ",", 1000+int(rand(200)), ":" } 1..10 + ","')
  echo " 10 random points: " $randpts
  PILOT_PTS1=${randpts}
fi

# ports
SHORE_LISTEN="9300"
SHORE_VPORT="9000"
ANNA_LISTEN="9301"
ANNA_LISTEN_GP="9401"
ANNA_VPORT="9001"

# percentage of messages to drop in uFldNodeComms
DROP_PCT=0

SERVERHOST="localhost" #"localhost"
nsplug meta_shoreside.moos targ_shoreside.moos -f WARP=$TIME_WARP \
   VNAME="shoreside" USC_DATA_DIR="../../../data"        \
   SHARE_LISTEN=$SHORE_LISTEN VPORT=$SHORE_VPORT SERVER_HOST=$SERVERHOST       \
   LOCATION=$EXP_LOCATION  PLUG_DIR=$PLUGDIR  MSG_DIR=$MSGDIR   \
   PAINT_SEGLIST=$PAINTSEGLIST   SIMULATION=$RUN_SIMULATION     \
   DROP_PERCENTAGE=$DROP_PCT  USE_GUI=$GUI

# start positions per area
START_POS1="430,950"
# loiter for during hyperparameter optimization
HP_LOITER_CONFIG="format=radial,x=440,y=970,radius=10,pts=4,snap=1,label=hp_optim_loiter"

# The first vehicle community
VNAME1="anna"
START_DEPTH1="5"
WAYPOINTS1="455,980:455,965:430,965:430,980:455,980"
MODEMID1="1"
VTYPE1="UUV" # UUV, SHIP
PREDICTIONS_PREFIX1="${VNAME1}_predictions"
PSHARE_ANNA="./pShare_auv.moos"
START_HEADING="135"
SENSOR_STDEV="0.0"

nsplug meta_vehicle.moos targ_$VNAME1.moos -f WARP=$TIME_WARP  \
   VNAME=$VNAME1  START_POS=$START_POS1  START_HDG=$START_HEADING \
   VPORT=$ANNA_VPORT SHARE_LISTEN=$ANNA_LISTEN SHARE_LISTEN_GP=$ANNA_LISTEN_GP \
   SHARE_GP2=$SHAREGP2  SERVER_LISTEN=$SHORE_LISTEN \
   VTYPE=$VTYPE1      MODEMID=$MODEMID1 \
   SERVER_HOST=$SERVERHOST  LOCATION=$EXP_LOCATION             \
   PLUG_DIR=$PLUGDIR  MSG_DIR=$MSGDIR                          \
   LAWNMOWER_CONFIG=$LAWNMOWER1  PREDICTIONS_PREFIX=$PREDICTIONS_PREFIX1 \
   NR_VEHICLES=$NUM_VEHICLES  MISSION_FILE_PSHARE=$PSHARE_ANNA  \
   ADAPTIVE_WPTS=$ADAPTIVE  USE_GUI=$GUI           \
   LOG_GP=$USE_LOG_GP  SENSOR_STD_DEV=$SENSOR_STDEV \
   DATA_NUM_DIMENSIONS=$DATA_DIMENSIONS  PATH_PLANNING_METHOD=$PP_METHOD
nsplug meta_vehicle.bhv targ_$VNAME1.bhv -f VNAME=$VNAME1      \
    START_POS=$START_POS1 WAYPOINTS=$WAYPOINTS1                \
    START_DEPTH=$START_DEPTH1 VTYPE=$VTYPE1                    \
    LAWNMOWER_NS=$LAWNMOWERNS LAWNMOWER_EW=$LAWNMOWEREW        \
    HP_LOITER=$HP_LOITER_CONFIG  ADAPTIVE_WPTS=$ADAPTIVE       \
    OPREGION=$BHVOPREGION  PILOT_PTS=$PILOT_PTS1               \
    CROSS_END_WPT=$CROSS_END1

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
sleep .25

printf "Done \n"

uMAC targ_shoreside.moos

printf "Killing all processes ... \n"
kill %1 %2 %3
printf "Done killing processes.   \n"
