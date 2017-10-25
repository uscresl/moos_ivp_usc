#!/bin/bash 
#-------------------------------------------------------
#  Part 1: Check for and handle command-line arguments
#-------------------------------------------------------
TIME_WARP=1
JUST_MAKE="no"
ADAPTIVE="yes"
TDS="no"
ACOMMS="no"
NUM_VEHICLES=1
RUN_SIMULATION="yes"
VORONOI_PARTITIONING="no"
AREA="old"
GUI="no"
ADP_START="cross"
BASE_PORT=9000

function printHelp()
{
printf "%s [OPTIONS]  \n" $0
      printf "  Options:  \n"
      printf "  -j: just make (not running)    \n" 
      printf "  -l: lawnmower (not adaptive)   \n"
      printf "  -t: timed data sharing         \n"
      printf "  -c: acomms data sharing        \n"
      printf "  -v: use voronoi partitioning   \n"
      printf "  -n: '1', '2' or '3' (nr of auvs)  \n"
      printf "  -b: 'bigger1' or 'bigger2' areas  \n"
      printf "  -g: use GUI                    \n"
      printf "  -r: use rprop (default: conj. gradient)  \n"
      printf "  -s: 'cross' or 'random' (adaptive start) \n"
      printf "  -w: time warp (int)            \n"
      printf "  -p: base port (default: 9000)  \n"
      printf "  --help, -h         \n" 
      exit 0;
}

while getopts jltcvn:b:gr:s:hw:p: option
do
  case "${option}"
  in
  j) JUST_MAKE="yes";;
  l) ADAPTIVE="no";; # lawnmower
  t) TDS="yes";;
  c) ACOMMS="yes";;
  v) VORONOI_PARTITIONING="yes";;
  n) NUM_VEHICLES=${OPTARG};;
  b) AREA=${OPTARG};;
  g) GUI="yes";;
  s) ADP_START=${OPTARG};;
  h) printHelp;;
  w) TIME_WARP=${OPTARG};;
  p) BASE_PORT=${OPTARG};;
  esac
done

# check settings
echo "ADAPTIVE: " ${ADAPTIVE}
echo "TDS: " ${TDS}
echo "ACOMMS: " ${ACOMMS}
echo "VORONOI_PARTITIONING: " ${VORONOI_PARTITIONING}
echo "NUM_VEHICLES: " ${NUM_VEHICLES}
echo "AREA: " ${AREA}
echo "GUI: " ${GUI}
echo "ADP_START: " ${ADP_START}
echo "WARP: " ${TIME_WARP}
echo "BASE_PORT: " ${BASE_PORT}

# check if sim data file present
if [ ! -f 'test.csv' ] ; then 
  cp ../../../data/fake_bio/two_depths.csv test.csv
  echo 'ERROR: No simulated data file presented. Copied two_depths.csv';
fi
# most scenario files have 3D data in them
DATA_NR_DIMENSIONS=3

#-------------------------------------------------------
#  Part 2: Create the .moos and .bhv files. 
#-------------------------------------------------------
# simulation set-up
EXP_LOCATION="puddingstone" # puddingstone, santafe, arrowhead
PLUGDIR="../../../plugs" # no leading slash
MSGDIR="${MOOSIVP_USC_HOME}/proto"

# paint survey area on pMarineViewer
if [ ${AREA} = "bigger1" ] ; then
# bigger 1
PAINTSEGLIST="pts={600,900:600,1300:1200,1300:1200,900:600,900},label=survey_area,label_color=white,edge_color=yellow,vertex_color=yellow,vertex_size=3,edge_size=3"
BHVOPREGION="label,OpRegion:375,875:375,1050:600,1320:1250,1320:1250,875"
elif [ ${AREA} = "bigger2" ] ; then
# bigger 2
PAINTSEGLIST="pts={700,700:700,1300:1200,1300:1200,700:700,700},label=survey_area,label_color=white,edge_color=yellow,vertex_color=yellow,vertex_size=3,edge_size=3"
BHVOPREGION="label,OpRegion:375,875:375,1050:600,1320:1250,1320:1250,650:600,650"
else
# old area
PAINTSEGLIST="pts={500,1200:500,1000:900,1000:900,1200:500,1200},label=survey_area,label_color=white,edge_color=yellow,vertex_color=yellow,vertex_size=3,edge_size=3"
BHVOPREGION="label,OpRegion:400,920:400,1045:480,1215:600,1300:1000,1300:1000,920"
fi

# config for lawnmower for actual GP model building
if [ ${AREA} = "bigger1" ] ; then
# bigger1
LX=900
LY=1100
LW=600
LH=400
elif [ ${AREA} = "bigger2" ] ; then
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
LX3=$[LX+LW/3]
  if [ ${AREA} = "bigger2" ] ; then
    LAWNMOWER1="format=lawnmower,x=${LX1},y=${LY},width=${LW3v},height=${LH},lane_width=40,degs=0,startx=0,starty=0"
    LAWNMOWER2="format=lawnmower,x=${LX},y=${LY},width=${LW3v},height=${LH},lane_width=40,degs=0,startx=0,starty=0"
    LAWNMOWER3="format=lawnmower,x=${LX3},y=${LY},width=${LW3v},height=${LH},lane_width=40,degs=0,startx=0,starty=0"
  else
    LAWNMOWER1="format=lawnmower,x=${LX1},y=${LY},width=${LW3v},height=${LH},lane_width=20,degs=0,startx=0,starty=0"
    LAWNMOWER2="format=lawnmower,x=${LX},y=${LY},width=${LW3v},height=${LH},lane_width=20,degs=0,startx=0,starty=0"
    LAWNMOWER3="format=lawnmower,x=${LX3},y=${LY},width=${LW3v},height=${LH},lane_width=20,degs=0,startx=0,starty=0"
  fi
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

##### specify for adaptive whether to use integrated cross or random pilot #####
if [ "${ADAPTIVE}" = "yes" ] && [ "${ADP_START}" = "cross" ] ; then
  if [ ${AREA} = "bigger1" ] ; then
    # 1auv cross
    PILOT_PTS1=600,900:1200,1300:600,1300:1200,900
    CROSS_END1=1200,900 # last wpt
    if [ $NUM_VEHICLES -ge 2 ] ; then
      # 2auv cross
      PILOT_PTS1=600,900:900,1300:600,1300:900,900
      CROSS_END1=900,900
      PILOT_PTS2=900,900:1200,1300:900,1300:1200,900
      CROSS_END2=1200,900
    fi
    if [ $NUM_VEHICLES -ge 3 ] ; then
      # 3auv cross
      PILOT_PTS1=600,900:800,1300:600,1300:800,900
      CROSS_END1=800,900
      PILOT_PTS2=800,900:1000,1300:800,1300:1000,900
      CROSS_END2=1000,900
      PILOT_PTS3=1000,900:1200,1300:1000,1300:1200,900  
      CROSS_END3=1200,900
    fi
  elif [ ${AREA} = "bigger2" ] ; then
    # 1auv cross
    PILOT_PTS1=700,700:1200,1300:700,1300:1200,700
    CROSS_END1=1200,700 # last wpt
    if [ $NUM_VEHICLES -ge 2 ] ; then
      # 2auv cross
      PILOT_PTS1=700,700:1200,1000:700,1000:1200,700
      CROSS_END1=1200,700
      PILOT_PTS2=700,1000:1200,1300:700,1300:1200,1000
      CROSS_END2=1200,1000
    fi
    if [ $NUM_VEHICLES -ge 3 ] ; then
      # 3auv cross
      PILOT_PTS1=700,700:1200,900:700,900:1200,700
      CROSS_END1=1200,700
      PILOT_PTS2=700,900:1200,1100:700,1100:1200,900
      CROSS_END2=1200,900
      PILOT_PTS3=700,1100:1200,1300:700,1300:1200,1100  
      CROSS_END3=1200,1100
    fi
  else
    # 1auv cross
    PILOT_PTS1=500,1000:900,1200:500,1200:900,1000
    CROSS_END1=900,1000
    if [ $NUM_VEHICLES -ge 2 ] ; then
      # 2auv cross
      PILOT_PTS1=500,1000:700,1200:500,1200:700,1000
      CROSS_END1=700,1000
      PILOT_PTS2=700,1000:900,1200:700,1200:900,1000
      CROSS_END2=900,1000
    fi
    if [ $NUM_VEHICLES -ge 3 ] ; then
      # 3auv cross
      PILOT_PTS1=500,1000:633,1200:500,1200:633,1000
      CROSS_END1=633,1000
      PILOT_PTS2=634,1000:767,1200:634,1200:767,1000
      CROSS_END2=767,1000
      PILOT_PTS3=768,1000:900,1200:768,1200:900,1000  
      CROSS_END3=900,1000
    fi
  fi
fi

if [ "${ADAPTIVE}" = "yes" ] && [ "${ADP_START}" = "random" ] ; then
  # 1auv random
  randpts=$(perl -le 'print map { 500+int(rand(400)), ",", 1000+int(rand(200)), ":" } 1..10 + ","')
  echo " 10 random points: " $randpts
  PILOT_PTS1=${randpts}
  if [ $NUM_VEHICLES -ge 2 ] ; then
  # 2auv random
  randpts=$(perl -le 'print map { 500+int(rand(400)), ",", 1000+int(rand(200)), ":" } 1..5 + ","')
  echo " 5 random points: " $randpts
  PILOT_PTS1=${randpts}
  randpts=$(perl -le 'print map { 500+int(rand(400)), ",", 1000+int(rand(200)), ":" } 1..5 + ","')
  echo " 5 random points: " $randpts
  PILOT_PTS2=${randpts}
  fi
  if [ $NUM_VEHICLES -ge 3 ] ; then
  # 3auv random
  randpts=$(perl -le 'print map { 500+int(rand(400)), ",", 1000+int(rand(200)), ":" } 1..3 + ","')
  echo " 3 random points: " $randpts
  PILOT_PTS1=${randpts}
  randpts=$(perl -le 'print map { 500+int(rand(400)), ",", 1000+int(rand(200)), ":" } 1..3 + ","')
  echo " 3 random points: " $randpts
  PILOT_PTS2=${randpts}
  randpts=$(perl -le 'print map { 500+int(rand(400)), ",", 1000+int(rand(200)), ":" } 1..3 + ","')
  echo " 3 random points: " $randpts
  PILOT_PTS3=${randpts}
  fi
fi
#####

# ports
SHORE_VPORT=$(($BASE_PORT))
SHORE_LISTEN=$(($BASE_PORT+300))

ANNA_VPORT=$(($BASE_PORT+1))
ANNA_LISTEN=$(($BASE_PORT+11))
ANNA_LISTEN_GP=$(($BASE_PORT+21))

BERNARD_VPORT=$(($BASE_PORT+2))
BERNARD_LISTEN=$(($BASE_PORT+12))
BERNARD_LISTEN_GP=$(($BASE_PORT+22))

CORNELIS_VPORT=$(($BASE_PORT+3))
CORNELIS_LISTEN=$(($BASE_PORT+13))
CORNELIS_LISTEN_GP=$(($BASE_PORT+23))

SHUB_VPORT=$(($BASE_PORT+100))
SHUB_LISTEN=$(($BASE_PORT+110))
SHUB_LISTEN_GP=$(($BASE_PORT+120))


if [ $NUM_VEHICLES -ge 2 ] ; then
PSHARE_MISSIONFILE="./plugs/pShare_auv.moos"
SHAREGP2=$BERNARD_LISTEN_GP
fi
if [ $NUM_VEHICLES -ge 3 ] ; then
SHAREGP3=$CORNELIS_LISTEN_GP
fi
SHAREGP_HUB=$SHUB_LISTEN_GP

# percentage of messages to drop in uFldNodeComms (acomms)
DROP_PCT=25

SERVERHOST="localhost" #"localhost"
nsplug meta_shoreside.moos targ_shoreside.moos -f WARP=$TIME_WARP \
   VNAME="shoreside" USC_DATA_DIR="../../../data"        \
   SHARE_LISTEN=$SHORE_LISTEN VPORT=$SHORE_VPORT SERVER_HOST=$SERVERHOST       \
   LOCATION=$EXP_LOCATION  PLUG_DIR=$PLUGDIR  MSG_DIR=$MSGDIR   \
   PAINT_SEGLIST=$PAINTSEGLIST   SIMULATION=$RUN_SIMULATION     \
   DROP_PERCENTAGE=$DROP_PCT  USE_GUI=$GUI

# START HEADING same for all vehicles - can be customized (not needed here)
START_HEADING="230"

if [ ${AREA} = "bigger2" ] ; then
  START_POS1="635,790"
  START_POS2="635,770"
  START_POS3="635,750"
  START_POS_SHUB="1000,1000"
  WAYPOINTS_SHUB="1000,1000"

  HP_LOITER_CONFIG="format=radial,x=655,y=720,radius=10,pts=4,snap=1,label=hp_optim_loiter"
  if [ $NUM_VEHICLES -ge 2 ] ; then
  HP_LOITER_CONFIG2="format=radial,x=655,y=750,radius=10,pts=4,snap=1,label=hp_optim_loiter"
  fi
  if [ $NUM_VEHICLES -ge 3 ] ; then
  HP_LOITER_CONFIG3="format=radial,x=655,y=780,radius=10,pts=4,snap=1,label=hp_optimloiter"
  fi
  HP_LOITER_CONFIG_SHUB="format=radial,x=1000,y=1000,radius=10,pts=4,snap=1,label=hp_optim_loiter"
else
  START_POS1="430,950"
  START_POS2="450,950"
  START_POS3="410,950"
  START_POS_SHUB="430,1010"
  WAYPOINTS_SHUB="700,1100"

  # loiter for during hyperparameter optimization
  HP_LOITER_CONFIG="format=radial,x=440,y=970,radius=10,pts=4,snap=1,label=hp_optim_loiter"
  if [ $NUM_VEHICLES -ge 2 ] ; then
  HP_LOITER_CONFIG2="format=radial,x=470,y=990,radius=10,pts=4,snap=1,label=hp_optim_loiter"
  fi
  if [ $NUM_VEHICLES -ge 3 ] ; then
  HP_LOITER_CONFIG3="format=radial,x=410,y=970,radius=10,pts=4,snap=1,label=hp_optimloiter"
  fi
  HP_LOITER_CONFIG_SHUB="format=radial,x=430,y=1010,radius=10,pts=4,snap=1,label=hp_optim_loiter"
fi

# The first vehicle community
VNAME1="anna"
START_DEPTH1="5"
WAYPOINTS1="455,980:455,965:430,965:430,980:455,980"
MODEMID1="1"
VTYPE1="UUV" # UUV, SHIP
PREDICTIONS_PREFIX1="${VNAME1}_predictions"

# The second vehicle community
VNAME2="bernard"
START_DEPTH2="5"
WAYPOINTS2="455,980:455,965:430,965:430,980:455,980"
MODEMID2="2"
VTYPE2="UUV" # UUV, SHIP
PREDICTIONS_PREFIX2="${VNAME2}_predictions"

# The third vehicle community
VNAME3="cornelis"
START_DEPTH3="5"
WAYPOINTS3="455,980:455,965:430,965:430,980:455,980"
MODEMID3="3"
VTYPE3="UUV" # UUV, SHIP
PREDICTIONS_PREFIX3="${VNAME3}_predictions"

# 4th vehicle community: surface hub
SHUB="surfacehub"
START_DEPTH_SHUB=0
MODEMID_SHUB="15"
VTYPE_SHUB="SHIP"
PREDICTIONS_PREFIX_SHUB="${SHUB}_predictions"

nsplug meta_vehicle.moos targ_$VNAME1.moos -f WARP=$TIME_WARP  \
   VNAME=$VNAME1  START_POS=$START_POS1  START_HDG=$START_HEADING \
   VPORT=$ANNA_VPORT SHARE_LISTEN=$ANNA_LISTEN SHARE_LISTEN_GP=$ANNA_LISTEN_GP \
   SHARE_GP2=$SHAREGP2  SHARE_GP3=$SHAREGP3  SHARE_GP_HUB=$SHAREGP_HUB  \
   SERVER_LISTEN=$SHORE_LISTEN \
   VTYPE=$VTYPE1      MODEMID=$MODEMID1 \
   SERVER_HOST=$SERVERHOST  LOCATION=$EXP_LOCATION             \
   PLUG_DIR=$PLUGDIR  MSG_DIR=$MSGDIR                          \
   LAWNMOWER_CONFIG=$LAWNMOWER  PREDICTIONS_PREFIX=$PREDICTIONS_PREFIX1 \
   NR_VEHICLES=$NUM_VEHICLES  MISSION_FILE_PSHARE=$PSHARE_MISSIONFILE  \
   ADAPTIVE_WPTS=$ADAPTIVE  USE_TDS=$TDS  USE_ACOMMS=$ACOMMS   \
   USE_VORONOI=$VORONOI_PARTITIONING  USE_GUI=$GUI  \
   ADP_START_PILOT=$ADP_START  SURVEY_AREA=$AREA               \
   DATA_NUM_DIMENSIONS=$DATA_NR_DIMENSIONS
nsplug meta_vehicle.bhv targ_$VNAME1.bhv -f VNAME=$VNAME1      \
    START_POS=$START_POS1 WAYPOINTS=$WAYPOINTS1                \
    START_DEPTH=$START_DEPTH1 VTYPE=$VTYPE1                    \
    LAWNMOWER_NS=$LAWNMOWERNS LAWNMOWER_EW=$LAWNMOWEREW        \
    HP_LOITER=$HP_LOITER_CONFIG_SHUB  ADAPTIVE_WPTS=$ADAPTIVE       \
    OTHER_VEHICLE=$VNAME2 OPREGION=$BHVOPREGION                \
    PILOT_PTS=$PILOT_PTS1  CROSS_END_WPT=$CROSS_END1
# TODO fix OTHER_VEHICLE

if [ $NUM_VEHICLES -ge 2 ] ; then
SHAREGP2=$ANNA_LISTEN_GP

nsplug meta_vehicle.moos targ_$VNAME2.moos -f WARP=$TIME_WARP  \
   VNAME=$VNAME2  START_POS=$START_POS2  START_HDG=$START_HEADING \
   VPORT=$BERNARD_VPORT SHARE_LISTEN=$BERNARD_LISTEN SHARE_LISTEN_GP=$BERNARD_LISTEN_GP \
   SHARE_GP2=$SHAREGP2   SHARE_GP3=$SHAREGP3   SHARE_GP_HUB=$SHAREGP_HUB  \
   SERVER_LISTEN=$SHORE_LISTEN  \
   VTYPE=$VTYPE2      MODEMID=$MODEMID2 \
   SERVER_HOST=$SERVERHOST  LOCATION=$EXP_LOCATION             \
   PLUG_DIR=$PLUGDIR  MSG_DIR=$MSGDIR                          \
   LAWNMOWER_CONFIG=$LAWNMOWER  PREDICTIONS_PREFIX=$PREDICTIONS_PREFIX2 \
   NR_VEHICLES=$NUM_VEHICLES  MISSION_FILE_PSHARE=$PSHARE_MISSIONFILE  \
   ADAPTIVE_WPTS=$ADAPTIVE  USE_TDS=$TDS  USE_ACOMMS=$ACOMMS   \
   USE_VORONOI=$VORONOI_PARTITIONING  USE_GUI=$GUI  \
   ADP_START_PILOT=$ADP_START  SURVEY_AREA=$AREA               \
   DATA_NUM_DIMENSIONS=$DATA_NR_DIMENSIONS
nsplug meta_vehicle.bhv targ_$VNAME2.bhv -f VNAME=$VNAME2      \
    START_POS=$START_POS2 WAYPOINTS=$WAYPOINTS2                \
    START_DEPTH=$START_DEPTH2 VTYPE=$VTYPE2                    \
    LAWNMOWER_NS=$LAWNMOWERNS2 LAWNMOWER_EW=$LAWNMOWEREW2        \
    HP_LOITER=$HP_LOITER_CONFIG2  ADAPTIVE_WPTS=$ADAPTIVE        \
    OTHER_VEHICLE=$VNAME1 OPREGION=$BHVOPREGION                \
    PILOT_PTS=$PILOT_PTS2  CROSS_END_WPT=$CROSS_END2
fi
# TODO fix OTHER_VEHICLE

if [ $NUM_VEHICLES -ge 3 ] ; then
SHAREGP2=$ANNA_LISTEN_GP
SHAREGP3=$BERNARD_LISTEN_GP
nsplug meta_vehicle.moos targ_$VNAME3.moos -f WARP=$TIME_WARP  \
   VNAME=$VNAME3  START_POS=$START_POS3  START_HDG=$START_HEADING \
   VPORT=$CORNELIS_VPORT SHARE_LISTEN=$CORNELIS_LISTEN SHARE_LISTEN_GP=$CORNELIS_LISTEN_GP \
   SHARE_GP2=$SHAREGP2   SHARE_GP3=$SHAREGP3   SHARE_GP_HUB=$SHAREGP_HUB  \
   SERVER_LISTEN=$SHORE_LISTEN  \
   VTYPE=$VTYPE3      MODEMID=$MODEMID3 \
   SERVER_HOST=$SERVERHOST  LOCATION=$EXP_LOCATION             \
   PLUG_DIR=$PLUGDIR  MSG_DIR=$MSGDIR                          \
   LAWNMOWER_CONFIG=$LAWNMOWER  PREDICTIONS_PREFIX=$PREDICTIONS_PREFIX3 \
   NR_VEHICLES=$NUM_VEHICLES  MISSION_FILE_PSHARE=$PSHARE_MISSIONFILE  \
   ADAPTIVE_WPTS=$ADAPTIVE  USE_TDS=$TDS  USE_ACOMMS=$ACOMMS   \
   USE_VORONOI=$VORONOI_PARTITIONING  USE_GUI=$GUI  \
   ADP_START_PILOT=$ADP_START  SURVEY_AREA=$AREA               \
   DATA_NUM_DIMENSIONS=$DATA_NR_DIMENSIONS
nsplug meta_vehicle.bhv targ_$VNAME3.bhv -f VNAME=$VNAME3      \
    START_POS=$START_POS3 WAYPOINTS=$WAYPOINTS3                \
    START_DEPTH=$START_DEPTH3 VTYPE=$VTYPE3                    \
    LAWNMOWER_NS=$LAWNMOWERNS3 LAWNMOWER_EW=$LAWNMOWEREW3        \
    HP_LOITER=$HP_LOITER_CONFIG3  ADAPTIVE_WPTS=$ADAPTIVE        \
    OTHER_VEHICLE=$VNAME2 OPREGION=$BHVOPREGION                \
    PILOT_PTS=$PILOT_PTS3  CROSS_END_WPT=$CROSS_END3
fi
# TODO fix OTHER_VEHICLE

# v4: surface hub
nsplug meta_surface_hub.moos targ_surface_hub.moos -f WARP=$TIME_WARP  \
   VNAME=$SHUB  START_POS=$START_POS_SHUB  START_HDG=$START_HEADING \
   VPORT=$SHUB_VPORT SHARE_LISTEN=$SHUB_LISTEN SHARE_LISTEN_GP=$SHUB_LISTEN_GP \
   SHARE_GP2=$SHAREGP2  SHARE_GP3=$SHAREGP3  SHARE_GP_HUB=$SHAREGP_HUB  \
   SERVER_LISTEN=$SHORE_LISTEN \
   VTYPE=$VTYPE_SHUB      MODEMID=$MODEMID_SHUB \
   SERVER_HOST=$SERVERHOST  LOCATION=$EXP_LOCATION             \
   PLUG_DIR=$PLUGDIR  MSG_DIR=$MSGDIR                          \
   LAWNMOWER_CONFIG=$LAWNMOWER  PREDICTIONS_PREFIX=$PREDICTIONS_PREFIX_SHUB \
   NR_VEHICLES=$NUM_VEHICLES  MISSION_FILE_PSHARE=$PSHARE_MISSIONFILE  \
   ADAPTIVE_WPTS=$ADAPTIVE  USE_TDS=$TDS  USE_ACOMMS=$ACOMMS   \
   USE_GUI=$GUI  SURVEY_AREA=$AREA  DATA_NUM_DIMENSIONS=$DATA_NR_DIMENSIONS
nsplug meta_surface_hub.bhv targ_surfacehub.bhv -f VNAME=$SHUB      \
    START_POS=$START_POS_SHUB WAYPOINTS=$WAYPOINTS_SHUB                \
    START_DEPTH=$START_DEPTH_SHUB VTYPE=$VTYPE_SHUB                    \
    HP_LOITER=$HP_LOITER_CONFIG  ADAPTIVE_WPTS=$ADAPTIVE       \
    OTHER_VEHICLE=$VNAME2 OPREGION=$BHVOPREGION


if [ ${JUST_MAKE} = "yes" ] ; then
    exit 0
fi

#-------------------------------------------------------
#  Part 3: Launch the processes
#-------------------------------------------------------
# shoreside (visualization, node comms)
printf "Launching shoreside MOOS Community (WARP=%s) \n"  $TIME_WARP
pAntler targ_shoreside.moos > log_shoreside.log &

# vehicles
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

# surface hub
printf "Launching $SHUB MOOS Community (WARP=%s) \n" $TIME_WARP
pAntler targ_surface_hub.moos > log_surface_hub.log &
sleep .25

printf "Done \n"

uMAC targ_shoreside.moos

printf "Killing all processes ... \n"
kill %1 %2 %3 %4 %5
printf "Done killing processes.   \n"
