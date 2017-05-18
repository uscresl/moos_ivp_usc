#!/bin/bash 
#-------------------------------------------------------
#  Part 1: Check for and handle command-line arguments
#-------------------------------------------------------

#default values
TIME_WARP=1
JUST_MAKE="no"
# standard: runtime
SIMULATION_MODE="no"
TOPSIDE="no"
ECOMAPPER="no"
ADAPTIVE="no"

#overwrite with arguments
printUsage ()
{
  printf "USAGE:\n"
  printf "%s [SWITCHES] [time_warp]   \n" $0
  printf "  --just_make, -j    \n" 
  printf "  --simulation, -s (includes -e, -t)  \n"
  printf "  --ecomapper, -e    \n"
  printf "  --topside, -t      \n" 
  printf "  --adaptive, -a     \n"
  printf "  --help, -h         \n" 
  exit 0;
}

if [[ $# -eq 0 ]] ; then
  printUsage
fi

for ARGI; do
    if [ "${ARGI}" = "--help" -o "${ARGI}" = "-h" ] ; then
      printUsage
    elif [ "${ARGI//[^0-9]/}" = "$ARGI" -a "$TIME_WARP" = 1 ]; then 
        TIME_WARP=$ARGI
    elif [ "${ARGI}" = "--just_build" -o "${ARGI}" = "-j" ] ; then
        JUST_MAKE="yes"
    elif [ "${ARGI}" = "--simulation" -o "${ARGI}" = "-s" ] ; then
        SIMULATION_MODE="yes"
    elif [ "${ARGI}" = "--ecomapper" -o "${ARGI}" = "-e" ] ; then
        ECOMAPPER="yes"
    elif [ "${ARGI}" = "--topside" -o "${ARGI}" = "-t" ] ; then
        TOPSIDE="yes"
    elif [ "${ARGI}" = "--adaptive" -o "${ARGI}" = "-a" ] ; then
        ADAPTIVE="yes"
    else 
        printf "Bad Argument: %s \n" $ARGI
        exit 0
    fi
done

if [ "${SIMULATION_MODE}" = "yes" ] ; then
  ECOMAPPER="yes"
  TOPSIDE="yes"
fi

#-------------------------------------------------------
#  Part 2: Create the .moos and .bhv files. 
#-------------------------------------------------------
# simulation set-up
LAKE="puddingstone"
PLUGDIR="../../../../plugs" # no leading slash
USC_DATADIR="../../../../data"

# copying from adp_sampl_1auv
PAINTSEGLIST="pts={450,1200:450,1000:750,1000:750,1200:450,1200},label=survey_area,label_color=white,edge_color=yellow,vertex_color=yellow,vertex_size=3,edge_size=3"
BHVOPREGION="label,OpRegion:400,920:400,1045:420,1210:600,1300:1000,1300:1000,920"
# loiter for during hyperparameter optimization
HP_LOITER_CONFIG="format=radial,x=440,y=970,radius=10,pts=4,snap=1,label=hp_optimization_loiter"
# old area
LX=600
LY=1100
LW=300
LH=200
LAWNMOWER1="format=lawnmower,x=${LX},y=${LY},width=${LW},height=${LH},lane_width=20,degs=0,startx=0,starty=0"
if [ "${ADAPTIVE}" = "no" ] ; then
  # lawnmower
  LAWNMOWEREW="$LAWNMOWER1,rows=east-west,label=east-west-survey"
  LAWNMOWERNS="$LAWNMOWER1,rows=north-south,label=north-south-survey"
fi
##### specify for adaptive whether to use integrated cross or random pilot #####
if [ "${ADAPTIVE}" = "yes" ] ; then
  # 1auv cross
  PILOT_PTS1=450,1000:750,1200:450,1200:750,1000
fi



if [ "${SIMULATION_MODE}" = "yes" ] ; then
  SERVERHOST_EM="localhost"
  SERVERHOST_SS="localhost"
fi
if [ "${SIMULATION_MODE}" = "no" ] ; then
  SERVERHOST_EM="192.168.10.11"
  SERVERHOST_SS="192.168.10.15"
fi

if [ "${TOPSIDE}" = "yes" -o "${JUST_MAKE}" = "yes" ] ; then
  # create shoreside.moos
  nsplug shoreside.meta shoreside.moos -f WARP=$TIME_WARP \
     VNAME="shoreside" USC_DATA_DIR="$MOOSIVP_USC_HOME/data"        \
     SHARE_LISTEN="9300" VPORT="9000" SERVER_HOST=$SERVERHOST_SS       \
     SERVER_HOST_EM=$SERVERHOST_EM  PLUG_DIR=$PLUGDIR  LOCATION=$LAKE  \
     USC_DATA_DIR=$USC_DATADIR  PAINT_SEGLIST=$PAINTSEGLIST
fi

if [ "${ECOMAPPER}" = "yes" -o "${JUST_MAKE}" = "yes" ] ; then
  # create ecomapper.moos
  VNAME1="zoomer"        # The first  vehicle community
  START_DEPTH1="0"
  START_POS1="440,950"
  START_HEADING1="0"
  MISSION_DEPTH="8"
  WAYPOINTS1="435,970:405,1020:435,970"
  MODEMID1="1"
  VTYPE1="UUV" # UUV, SHIP
  nsplug ecomapper.meta ecomapper.moos -f WARP=$TIME_WARP  \
     VNAME=$VNAME1  START_POS=$START_POS1  START_HDG=$START_HEADING1 \
     VPORT="9001"       SHARE_LISTEN="9301"                      \
     VTYPE=$VTYPE1      MODEMID=$MODEMID1                        \
     SERVER_HOST=$SERVERHOST_EM SERVER_HOST_SS=$SERVERHOST_SS    \
     SIMULATION=$SIMULATION_MODE  PLUG_DIR=$PLUGDIR  LOCATION=$LAKE \
     LAWNMOWER_CONFIG=$LAWNMOWER1  ADAPTIVE_WPTS=$ADAPTIVE  
  nsplug ecomapper_bhv.meta ecomapper.bhv -f VNAME=$VNAME1       \
      START_POS=$START_POS1 WAYPOINTS=$WAYPOINTS1                \
      START_DEPTH=$START_DEPTH1 VTYPE=$VTYPE1                    \
      WPT_DEPTH=$MISSION_DEPTH  ADAPTIVE_WPTS=$ADAPTIVE          \
      LAWNMOWER_NS=$LAWNMOWERNS LAWNMOWER_EW=$LAWNMOWEREW        \
      HP_LOITER=$HP_LOITER_CONFIG  OPREGION=$BHVOPREGION         \
      PILOT_PTS=$PILOT_PTS1
fi

if [ ${JUST_MAKE} = "yes" ] ; then
    exit 0
fi

#-------------------------------------------------------
#  Part 3: Launch the processes
#-------------------------------------------------------
if [ "${TOPSIDE}" = "yes" ] ; then
  printf "Launching shoreside MOOS Community (WARP=%s) \n"  $TIME_WARP
  pAntler shoreside.moos > log_shoreside.log &
fi

if [ "${ECOMAPPER}" = "yes" ] ; then 
  printf "Launching EcoMapper MOOS Community (WARP=%s) \n" $TIME_WARP
  pAntler ecomapper.moos > log_ecomapper.log
fi

sleep .25
printf "Done \n"

printf "Killing all processes ... \n"
kill %1 %2
printf "Done killing processes.   \n"
