#!/bin/bash 
#-------------------------------------------------------
#  Part 1: Check for and handle command-line arguments
#-------------------------------------------------------
TIME_WARP=1
JUST_MAKE="no"
ADAPTIVE="no"
TDS="no"
ACOMMS="no"
NUM_VEHICLES=1
RUN_SIMULATION="yes"

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
    elif [ "${ARGI}" = "--adaptive" -o "${ARGI}" = "-a" ] ; then
        ADAPTIVE="yes"
    elif [ "${ARGI}" = "--tds" -o "${ARGI}" = "-t" ] ; then
        TDS="yes"
    elif [ "${ARGI}" = "--acomms" -o "${ARGI}" = "-c" ] ; then
        ACOMMS="yes"
    elif [ "${ARGI}" = "--2auvs" ] ; then
        NUM_VEHICLES=2
    else 
        printf "Bad Argument: %s \n" $ARGI
        exit 0
    fi
done

#-------------------------------------------------------
#  Part 2: Create the .moos and .bhv files. 
#-------------------------------------------------------
# simulation set-up
EXP_LOCATION="puddingstone" # puddingstone, santafe, arrowhead
PLUGDIR="../../../plugs" # no leading slash
MSGDIR="${MOOSIVP_USC_HOME}/proto"

PAINTSEGLIST="pts={500,1200:500,1000:900,1000:900,1200:500,1200},label=survey_area,label_color=white,edge_color=green,vertex_color=green,vertex_size=2,edge_size=2"

# config for lawnmower used to get pilot data for autotuning hyperparams GP
PILOT_LAWNMOWER_CONFIG="format=lawnmower,label=pilot-survey,x=700,y=1100,width=400,height=200,lane_width=100,degs=0,startx=0,starty=0"
PILOT_LAWNMOWER_C_NS="$PILOT_LAWNMOWER_CONFIG,rows=north-south"
PILOT_LAWNMOWER_C_EW="$PILOT_LAWNMOWER_CONFIG,rows=east-west"

if [ $NUM_VEHICLES -ge 2 ] ; then
PILOT_LM_1="format=lawnmower,label=pilot-survey,x=600,y=1100,width=200,height=200,lane_width=100,degs=0,startx=0,starty=0"
PILOT_LM_1_NS="$PILOT_LM_1,rows=north-south"
PILOT_LM_1_EW="$PILOT_LM_1,rows=east-west"
PILOT_LM_2="format=lawnmower,label=pilot-survey,x=800,y=1100,width=200,height=200,lane_width=100,degs=0,startx=0,starty=0"
PILOT_LM_2_NS="$PILOT_LM_2,rows=north-south"
PILOT_LM_2_EW="$PILOT_LM_2,rows=east-west"
fi

# loiter for during hyperparameter optimization
HP_LOITER_CONFIG="format=radial,x=440,y=970,radius=10,pts=4,snap=1,label=hp_optimization_loiter"
if [ $NUM_VEHICLES -ge 2 ] ; then
HP_LOITER_CONFIG2="format=radial,x=470,y=990,radius=10,pts=4,snap=1,label=hp_optimization_loiter"
fi

# config for lawnmower for actual GP model building
LAWNMOWER="format=lawnmower,x=700,y=1100,width=400,height=200,lane_width=20,degs=0,startx=0,starty=0"
if [ $NUM_VEHICLES -ge 2 ] ; then
LAWNMOWER1="format=lawnmower,x=600,y=1100,width=200,height=200,lane_width=20,degs=0,startx=0,starty=0"
LAWNMOWER2="format=lawnmower,x=800,y=1100,width=200,height=200,lane_width=20,degs=0,startx=0,starty=0"
else
LAWNMOWER1=$LAWNMOWER
fi

if [ "${ADAPTIVE}" = "no" ] ; then
  # lawnmower
  LAWNMOWEREW="$LAWNMOWER1,rows=east-west,label=east-west-survey"
  LAWNMOWERNS="$LAWNMOWER1,rows=north-south,label=north-south-survey"
  if [ $NUM_VEHICLES -ge 2 ] ; then
  LAWNMOWEREW2="$LAWNMOWER2,rows=east-west,label=east-west-survey"
  LAWNMOWERNS2="$LAWNMOWER2,rows=north-south,label=north-south-survey"
  fi
  # ports
  SHORE_LISTEN="8300"
  SHORE_VPORT="8000"
  ANNA_LISTEN="8301"
  ANNA_LISTEN_GP="8401"
  ANNA_VPORT="8001"
  BERNARD_LISTEN="8302"
  BERNARD_LISTEN_GP="8402"
  BERNARD_VPORT="8002"
else
  # adaptive
  
  # ports
  SHORE_LISTEN="9300"
  SHORE_VPORT="9000"
  ANNA_LISTEN="9301"
  ANNA_LISTEN_GP="9401"
  ANNA_VPORT="9001"
  BERNARD_LISTEN="9302"
  BERNARD_LISTEN_GP="9402"
  BERNARD_VPORT="9002"
fi

# percentage of messages to drop in uFldNodeComms
DROP_PCT=0

SERVERHOST="localhost" #"localhost"
nsplug meta_shoreside.moos targ_shoreside.moos -f WARP=$TIME_WARP \
   VNAME="shoreside" USC_DATA_DIR="../../../data"        \
   SHARE_LISTEN=$SHORE_LISTEN VPORT=$SHORE_VPORT SERVER_HOST=$SERVERHOST       \
   LOCATION=$EXP_LOCATION  PLUG_DIR=$PLUGDIR  MSG_DIR=$MSGDIR   \
   PAINT_SEGLIST=$PAINTSEGLIST   SIMULATION=$RUN_SIMULATION     \
   DROP_PERCENTAGE=$DROP_PCT

# START HEADING same for all vehicles - can be customized (not needed here)
START_HEADING="230"

VNAME1="anna"        # The first  vehicle community
START_DEPTH1="5"
START_POS1="430,950"
WAYPOINTS1="455,980:455,965:430,965:430,980:455,980"
MODEMID1="1"
VTYPE1="UUV" # UUV, SHIP
PREDICTIONS_PREFIX1="${VNAME1}_predictions"

# The second vehicle community
VNAME2="bernard"
START_DEPTH2="5"
START_POS2="450,950"
WAYPOINTS2="455,980:455,965:430,965:430,980:455,980"
MODEMID2="2"
VTYPE2="UUV" # UUV, SHIP
PREDICTIONS_PREFIX2="${VNAME2}_predictions"
PSHARE_BERNARD="./plugs/pShare_auv.moos"

if [ $NUM_VEHICLES -ge 2 ] ; then
PSHARE_ANNA="./plugs/pShare_auv.moos"
fi
nsplug meta_vehicle.moos targ_$VNAME1.moos -f WARP=$TIME_WARP  \
   VNAME=$VNAME1  START_POS=$START_POS1  START_HDG=$START_HEADING \
   VPORT=$ANNA_VPORT SHARE_LISTEN=$ANNA_LISTEN SHARE_LISTEN_GP=$ANNA_LISTEN_GP \
   SHARE_OTHER_GP=$BERNARD_LISTEN_GP  SERVER_LISTEN=$SHORE_LISTEN \
   VTYPE=$VTYPE1      MODEMID=$MODEMID1 \
   SERVER_HOST=$SERVERHOST  LOCATION=$EXP_LOCATION             \
   PLUG_DIR=$PLUGDIR  MSG_DIR=$MSGDIR                          \
   LAWNMOWER_CONFIG=$LAWNMOWER  PREDICTIONS_PREFIX=$PREDICTIONS_PREFIX1 \
   NR_VEHICLES=$NUM_VEHICLES  MISSION_FILE_PSHARE=$PSHARE_ANNA  \
   ADAPTIVE_WPTS=$ADAPTIVE  USE_TDS=$TDS  USE_ACOMMS=$ACOMMS
if [ $NUM_VEHICLES -ge 2 ] ; then
PILOT_LAWNMOWER_C_NS=$PILOT_LM_1_NS
PILOT_LAWNMOWER_C_EW=$PILOT_LM_1_EW
fi
nsplug meta_vehicle.bhv targ_$VNAME1.bhv -f VNAME=$VNAME1      \
    START_POS=$START_POS1 WAYPOINTS=$WAYPOINTS1                \
    START_DEPTH=$START_DEPTH1 VTYPE=$VTYPE1                    \
    PILOT_LAWNMOWER_NS=$PILOT_LAWNMOWER_C_NS                   \
    PILOT_LAWNMOWER_EW=$PILOT_LAWNMOWER_C_EW                   \
    LAWNMOWER_NS=$LAWNMOWERNS LAWNMOWER_EW=$LAWNMOWEREW        \
    HP_LOITER=$HP_LOITER_CONFIG  ADAPTIVE_WPTS=$ADAPTIVE       \
    OTHER_VEHICLE=$VNAME2

if [ $NUM_VEHICLES -ge 2 ] ; then

nsplug meta_vehicle.moos targ_$VNAME2.moos -f WARP=$TIME_WARP  \
   VNAME=$VNAME2  START_POS=$START_POS2  START_HDG=$START_HEADING \
   VPORT=$BERNARD_VPORT SHARE_LISTEN=$BERNARD_LISTEN SHARE_LISTEN_GP=$BERNARD_LISTEN_GP \
   SERVER_LISTEN=$SHORE_LISTEN    SHARE_OTHER_GP=$ANNA_LISTEN_GP \
   VTYPE=$VTYPE2      MODEMID=$MODEMID2 \
   SERVER_HOST=$SERVERHOST  LOCATION=$EXP_LOCATION             \
   PLUG_DIR=$PLUGDIR  MSG_DIR=$MSGDIR                          \
   LAWNMOWER_CONFIG=$LAWNMOWER  PREDICTIONS_PREFIX=$PREDICTIONS_PREFIX2 \
   NR_VEHICLES=$NUM_VEHICLES  MISSION_FILE_PSHARE=$PSHARE_BERNARD  \
   ADAPTIVE_WPTS=$ADAPTIVE  USE_TDS=$TDS  USE_ACOMMS=$ACOMMS
nsplug meta_vehicle.bhv targ_$VNAME2.bhv -f VNAME=$VNAME2      \
    START_POS=$START_POS2 WAYPOINTS=$WAYPOINTS2                \
    START_DEPTH=$START_DEPTH2 VTYPE=$VTYPE2                    \
    PILOT_LAWNMOWER_NS=$PILOT_LM_2_NS                   \
    PILOT_LAWNMOWER_EW=$PILOT_LM_2_EW                   \
    LAWNMOWER_NS=$LAWNMOWERNS2 LAWNMOWER_EW=$LAWNMOWEREW2        \
    HP_LOITER=$HP_LOITER_CONFIG2  ADAPTIVE_WPTS=$ADAPTIVE        \
    OTHER_VEHICLE=$VNAME1
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
sleep .25

if [ $NUM_VEHICLES -ge 2 ] ; then
printf "Launching $VNAME2 MOOS Community (WARP=%s) \n" $TIME_WARP
pAntler targ_$VNAME2.moos > log_$VNAME2.log &
sleep .25
fi

printf "Done \n"

uMAC targ_shoreside.moos

printf "Killing all processes ... \n"
kill %1 %2 %3 %4
printf "Done killing processes.   \n"
