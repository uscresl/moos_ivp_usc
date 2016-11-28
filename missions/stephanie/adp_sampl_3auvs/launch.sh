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
VORONOI_PARTITIONING="no"
AREA="old"
GUI="true"
CG="false"

for ARGI; do
    if [ "${ARGI}" = "--help" -o "${ARGI}" = "-h" ] ; then
        printf "%s [SWITCHES] [time_warp]   \n" $0
        printf "  Switches:          \n"
        printf "  --just_make, -j    \n" 
        printf "  --adaptive, -a     \n"
        printf "  --tds, -t          \n"
        printf "  --acomms, -c       \n"
        printf "  --voronoi, -v      \n"
        printf "  --2auvs            \n"
        printf "  --3auvs            \n"
        printf "  --bigger1, -b1     \n"
        printf "  --bigger2, -b2     \n"
        printf "  --nogui, -ng       \n"
        printf "  --cg, -cg          \n"
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
    elif [ "${ARGI}" = "--voronoi" -o "${ARGI}" = "-v" ]; then
        VORONOI_PARTITIONING="yes"
    elif [ "${ARGI}" = "--2auvs" ] ; then
        NUM_VEHICLES=2
    elif [ "${ARGI}" = "--3auvs" ] ; then
        NUM_VEHICLES=3
    elif [ "$ARGI" = "--bigger1" -o "${ARGI}" = "-b1" ]; then
        AREA="bigger1"
    elif [ "$ARGI" = "--bigger2" -o "${ARGI}" = "-b2" ]; then
        AREA="bigger2"
    elif [ "$ARGI" = "--nogui" -o "${ARGI}" = "-ng" ]; then
        GUI="no"
    elif [ "$ARGI" = "--cg" -o "${ARGI}" = "-cg" ]; then
        CG="yes"
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
if [ ${AREA} = "bigger1" ]; then
# bigger 1
PAINTSEGLIST="pts={600,900:600,1300:1200,1300:1200,900:600,900},label=survey_area,label_color=white,edge_color=yellow,vertex_color=yellow,vertex_size=3,edge_size=3"
BHVOPREGION="label,OpRegion:375,875:375,1050:600,1320:1250,1320:1250,875"
elif [ ${AREA} = "bigger2" ]; then
# bigger 2
PAINTSEGLIST="pts={700,700:700,1300:1200,1300:1200,700:700,700},label=survey_area,label_color=white,edge_color=yellow,vertex_color=yellow,vertex_size=3,edge_size=3"
BHVOPREGION="label,OpRegion:375,875:375,1050:600,1320:1250,1320:1250,650:600,650"
else
# old area
PAINTSEGLIST="pts={500,1200:500,1000:900,1000:900,1200:500,1200},label=survey_area,label_color=white,edge_color=yellow,vertex_color=yellow,vertex_size=3,edge_size=3"
BHVOPREGION="label,OpRegion:400,920:400,1045:480,1215:600,1300:1000,1300:1000,920"
fi

# loiter for during hyperparameter optimization
HP_LOITER_CONFIG="format=radial,x=440,y=970,radius=10,pts=4,snap=1,label=hp_optimization_loiter"
if [ $NUM_VEHICLES -ge 2 ] ; then
HP_LOITER_CONFIG2="format=radial,x=470,y=990,radius=10,pts=4,snap=1,label=hp_optimization_loiter"
fi
if [ $NUM_VEHICLES -ge 3 ] ; then
HP_LOITER_CONFIG3="format=radial,x=410,y=970,radius=10,pts=4,snap=1,label=hp_optimization_loiter"
fi

# config for lawnmower for actual GP model building
if [ ${AREA} = "bigger1" ]; then
# bigger1
LX=900
LY=1100
LW=600
LH=400
elif [ ${AREA} = "bigger2" ]; then
# bigger2
LX=950
LY=1000
LW=500
LH=600
else
# old area
LX=700
LY=1100
LW=400
LH=200
fi

LAWNMOWER="format=lawnmower,x=${LX},y=${LY},width=${LW},height=${LH},lane_width=20,degs=0,startx=0,starty=0"
if [ $NUM_VEHICLES -eq 2 ] ; then
LW2v=$[LW/2]
LX1=$[LX-LW/4]
LAWNMOWER1="format=lawnmower,x=${LX1},y=${LY},width=${LW2v},height=${LH},lane_width=20,degs=0,startx=0,starty=0"
LX2=$[LX+LW/4]
LAWNMOWER2="format=lawnmower,x=${LX2},y=${LY},width=${LW2v},height=${LH},lane_width=20,degs=0,startx=0,starty=0"
elif [ $NUM_VEHICLES -ge 3 ] ; then
LW3v=$[LW/3]
LX1=$[LX-LW/3]
LAWNMOWER1="format=lawnmower,x=${LX1},y=${LY},width=${LW3v},height=${LH},lane_width=20,degs=0,startx=0,starty=0"
LAWNMOWER2="format=lawnmower,x=${LX},y=${LY},width=${LW3v},height=${LH},lane_width=20,degs=0,startx=0,starty=0"
LX3=$[LX+LW/3]
LAWNMOWER3="format=lawnmower,x=${LX3},y=${LY},width=${LW3v},height=${LH},lane_width=20,degs=0,startx=0,starty=0"
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
  if [ $NUM_VEHICLES -ge 3 ] ; then
  LAWNMOWEREW3="$LAWNMOWER3,rows=east-west,label=east-west-survey"
  LAWNMOWERNS3="$LAWNMOWER3,rows=north-south,label=north-south-survey"
  fi
fi

# ports
SHORE_LISTEN="9300"
SHORE_VPORT="9000"
ANNA_LISTEN="9301"
ANNA_LISTEN_GP="9401"
ANNA_VPORT="9001"
BERNARD_LISTEN="9302"
BERNARD_LISTEN_GP="9402"
BERNARD_VPORT="9002"
CORNELIS_LISTEN="9303"
CORNELIS_LISTEN_GP="9403"
CORNELIS_VPORT="9003"

# percentage of messages to drop in uFldNodeComms
DROP_PCT=0

SERVERHOST="localhost" #"localhost"
nsplug meta_shoreside.moos targ_shoreside.moos -f WARP=$TIME_WARP \
   VNAME="shoreside" USC_DATA_DIR="../../../data"        \
   SHARE_LISTEN=$SHORE_LISTEN VPORT=$SHORE_VPORT SERVER_HOST=$SERVERHOST       \
   LOCATION=$EXP_LOCATION  PLUG_DIR=$PLUGDIR  MSG_DIR=$MSGDIR   \
   PAINT_SEGLIST=$PAINTSEGLIST   SIMULATION=$RUN_SIMULATION     \
   DROP_PERCENTAGE=$DROP_PCT  USE_GUI=$GUI

# START HEADING same for all vehicles - can be customized (not needed here)
START_HEADING="230"

# The first vehicle community
VNAME1="anna"
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

# The third vehicle community
VNAME3="cornelis"
START_DEPTH3="5"
START_POS3="410,950"
WAYPOINTS3="455,980:455,965:430,965:430,980:455,980"
MODEMID3="3"
VTYPE3="UUV" # UUV, SHIP
PREDICTIONS_PREFIX3="${VNAME3}_predictions"
PSHARE_CORNELIS="./plugs/pShare_auv.moos"

if [ $NUM_VEHICLES -ge 3 ] ; then
SHAREGP3=$CORNELIS_LISTEN_GP
fi

if [ $NUM_VEHICLES -ge 2 ] ; then
PSHARE_ANNA="./plugs/pShare_auv.moos"
SHAREGP2=$BERNARD_LISTEN_GP
fi

nsplug meta_vehicle.moos targ_$VNAME1.moos -f WARP=$TIME_WARP  \
   VNAME=$VNAME1  START_POS=$START_POS1  START_HDG=$START_HEADING \
   VPORT=$ANNA_VPORT SHARE_LISTEN=$ANNA_LISTEN SHARE_LISTEN_GP=$ANNA_LISTEN_GP \
   SHARE_GP2=$SHAREGP2  SHARE_GP3=$SHAREGP3 SERVER_LISTEN=$SHORE_LISTEN \
   VTYPE=$VTYPE1      MODEMID=$MODEMID1 \
   SERVER_HOST=$SERVERHOST  LOCATION=$EXP_LOCATION             \
   PLUG_DIR=$PLUGDIR  MSG_DIR=$MSGDIR                          \
   LAWNMOWER_CONFIG=$LAWNMOWER  PREDICTIONS_PREFIX=$PREDICTIONS_PREFIX1 \
   NR_VEHICLES=$NUM_VEHICLES  MISSION_FILE_PSHARE=$PSHARE_ANNA  \
   ADAPTIVE_WPTS=$ADAPTIVE  USE_TDS=$TDS  USE_ACOMMS=$ACOMMS   \
   USE_VORONOI=$VORONOI_PARTITIONING  USE_GUI=$GUI  USE_CG=$CG
nsplug meta_vehicle.bhv targ_$VNAME1.bhv -f VNAME=$VNAME1      \
    START_POS=$START_POS1 WAYPOINTS=$WAYPOINTS1                \
    START_DEPTH=$START_DEPTH1 VTYPE=$VTYPE1                    \
    LAWNMOWER_NS=$LAWNMOWERNS LAWNMOWER_EW=$LAWNMOWEREW        \
    HP_LOITER=$HP_LOITER_CONFIG  ADAPTIVE_WPTS=$ADAPTIVE       \
    OTHER_VEHICLE=$VNAME2 OPREGION=$BHVOPREGION
# TODO fix OTHER_VEHICLE

if [ $NUM_VEHICLES -ge 2 ] ; then
SHAREGP2=$ANNA_LISTEN_GP

nsplug meta_vehicle.moos targ_$VNAME2.moos -f WARP=$TIME_WARP  \
   VNAME=$VNAME2  START_POS=$START_POS2  START_HDG=$START_HEADING \
   VPORT=$BERNARD_VPORT SHARE_LISTEN=$BERNARD_LISTEN SHARE_LISTEN_GP=$BERNARD_LISTEN_GP \
   SERVER_LISTEN=$SHORE_LISTEN    SHARE_GP2=$SHAREGP2   SHARE_GP3=$SHAREGP3 \
   VTYPE=$VTYPE2      MODEMID=$MODEMID2 \
   SERVER_HOST=$SERVERHOST  LOCATION=$EXP_LOCATION             \
   PLUG_DIR=$PLUGDIR  MSG_DIR=$MSGDIR                          \
   LAWNMOWER_CONFIG=$LAWNMOWER  PREDICTIONS_PREFIX=$PREDICTIONS_PREFIX2 \
   NR_VEHICLES=$NUM_VEHICLES  MISSION_FILE_PSHARE=$PSHARE_BERNARD  \
   ADAPTIVE_WPTS=$ADAPTIVE  USE_TDS=$TDS  USE_ACOMMS=$ACOMMS   \
   USE_VORONOI=$VORONOI_PARTITIONING  USE_GUI=$GUI  USE_CG=$CG
nsplug meta_vehicle.bhv targ_$VNAME2.bhv -f VNAME=$VNAME2      \
    START_POS=$START_POS2 WAYPOINTS=$WAYPOINTS2                \
    START_DEPTH=$START_DEPTH2 VTYPE=$VTYPE2                    \
    LAWNMOWER_NS=$LAWNMOWERNS2 LAWNMOWER_EW=$LAWNMOWEREW2        \
    HP_LOITER=$HP_LOITER_CONFIG2  ADAPTIVE_WPTS=$ADAPTIVE        \
    OTHER_VEHICLE=$VNAME1 OPREGION=$BHVOPREGION
fi
# TODO fix OTHER_VEHICLE

if [ $NUM_VEHICLES -ge 3 ] ; then
SHAREGP2=$ANNA_LISTEN_GP
SHAREGP3=$BERNARD_LISTEN_GP
nsplug meta_vehicle.moos targ_$VNAME3.moos -f WARP=$TIME_WARP  \
   VNAME=$VNAME3  START_POS=$START_POS3  START_HDG=$START_HEADING \
   VPORT=$CORNELIS_VPORT SHARE_LISTEN=$CORNELIS_LISTEN SHARE_LISTEN_GP=$CORNELIS_LISTEN_GP \
   SERVER_LISTEN=$SHORE_LISTEN    SHARE_GP2=$SHAREGP2   SHARE_GP3=$SHAREGP3  \
   VTYPE=$VTYPE3      MODEMID=$MODEMID3 \
   SERVER_HOST=$SERVERHOST  LOCATION=$EXP_LOCATION             \
   PLUG_DIR=$PLUGDIR  MSG_DIR=$MSGDIR                          \
   LAWNMOWER_CONFIG=$LAWNMOWER  PREDICTIONS_PREFIX=$PREDICTIONS_PREFIX3 \
   NR_VEHICLES=$NUM_VEHICLES  MISSION_FILE_PSHARE=$PSHARE_CORNELIS  \
   ADAPTIVE_WPTS=$ADAPTIVE  USE_TDS=$TDS  USE_ACOMMS=$ACOMMS   \
   USE_VORONOI=$VORONOI_PARTITIONING  USE_GUI=$GUI  USE_CG=$CG
nsplug meta_vehicle.bhv targ_$VNAME3.bhv -f VNAME=$VNAME3      \
    START_POS=$START_POS3 WAYPOINTS=$WAYPOINTS3                \
    START_DEPTH=$START_DEPTH3 VTYPE=$VTYPE3                    \
    LAWNMOWER_NS=$LAWNMOWERNS3 LAWNMOWER_EW=$LAWNMOWEREW3        \
    HP_LOITER=$HP_LOITER_CONFIG3  ADAPTIVE_WPTS=$ADAPTIVE        \
    OTHER_VEHICLE=$VNAME2 OPREGION=$BHVOPREGION
fi
# TODO fix OTHER_VEHICLE

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

if [ $NUM_VEHICLES -ge 3 ] ; then
printf "Launching $VNAME3 MOOS Community (WARP=%s) \n" $TIME_WARP
pAntler targ_$VNAME3.moos > log_$VNAME3.log &
sleep .25
fi

printf "Done \n"

uMAC targ_shoreside.moos

printf "Killing all processes ... \n"
kill %1 %2 %3 %4
printf "Done killing processes.   \n"
