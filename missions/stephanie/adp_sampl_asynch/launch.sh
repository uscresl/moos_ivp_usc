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
BASE_PORT=9000
WITH_SHUB="yes"

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
  printf "  -d: completely decentralized, no surface hub \n"
  printf "  -w: time warp (int)            \n"
  printf "  -p: base port (default: 9000)  \n"
  printf "  --help, -h         \n" 
  exit 0;
}

while getopts jltcvdn:b:gr:s:hw:p: option
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
  d) WITH_SHUB="no";;
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
echo "WITH_SHUB: " ${WITH_SHUB}
echo "WARP: " ${TIME_WARP}
echo "BASE_PORT: " ${BASE_PORT}

# check if sim data file present
if [ ! -f 'test.csv' ] ; then 
  cp ../../../data/fake_bio/two_depths.csv test.csv
  echo 'ERROR: No simulated data file presented. Copied two_depths.csv';
fi
# most scenario files have 3D data in them
if [ ${AREA} = "bigger2" ] ; then
  DATA_NR_DIMENSIONS=2
else
  DATA_NR_DIMENSIONS=3
fi

#-------------------------------------------------------
#  Part 2: Create the .moos and .bhv files. 
#-------------------------------------------------------
# simulation set-up
EXP_LOCATION="puddingstone" # puddingstone, santafe, arrowhead
PLUGDIR="../../../plugs" # no leading slash
MSGDIR="${MOOSIVP_USC_HOME}/proto"
# specify comms range
COMMUNICATION_RANGE=1000

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
LM_CTR_X=900
LM_CTR_Y=1100
AREA_WIDTH=600
AREA_HEIGHT=400
elif [ ${AREA} = "bigger2" ] ; then
# bigger2
LM_CTR_X=950
LM_CTR_Y=1000
AREA_WIDTH=500
AREA_HEIGHT=600
else
# old area
LM_CTR_X=700
LM_CTR_Y=1100
AREA_WIDTH=400
AREA_HEIGHT=200
fi

LAWNMOWER="format=lawnmower,x=${LM_CTR_X},y=${LM_CTR_Y},width=${AREA_WIDTH},height=${AREA_HEIGHT},lane_width=20,degs=0,startx=0,starty=0"
if [ $NUM_VEHICLES -eq 2 ] ; then
AREA_WIDTH2v=$[AREA_WIDTH/2]
LM_CTR_X1=$[LM_CTR_X-AREA_WIDTH/4]
LAWNMOWER1="format=lawnmower,x=${LM_CTR_X1},y=${LM_CTR_Y},width=${AREA_WIDTH2v},height=${AREA_HEIGHT},lane_width=20,degs=0,startx=0,starty=0"
LM_CTR_X2=$[LM_CTR_X+AREA_WIDTH/4]
LAWNMOWER2="format=lawnmower,x=${LM_CTR_X2},y=${LM_CTR_Y},width=${AREA_WIDTH2v},height=${AREA_HEIGHT},lane_width=20,degs=0,startx=0,starty=0"
elif [ $NUM_VEHICLES -ge 3 ] ; then
AREA_WIDTH3v=$[AREA_WIDTH/3]
LM_CTR_X1=$[LM_CTR_X-AREA_WIDTH/3]
LM_CTR_X3=$[LM_CTR_X+AREA_WIDTH/3]
  if [ ${AREA} = "bigger2" ] ; then
    LAWNMOWER1="format=lawnmower,x=${LM_CTR_X1},y=${LM_CTR_Y},width=${AREA_WIDTH3v},height=${AREA_HEIGHT},lane_width=40,degs=0,startx=0,starty=0"
    LAWNMOWER2="format=lawnmower,x=${LM_CTR_X},y=${LM_CTR_Y},width=${AREA_WIDTH3v},height=${AREA_HEIGHT},lane_width=40,degs=0,startx=0,starty=0"
    LAWNMOWER3="format=lawnmower,x=${LM_CTR_X3},y=${LM_CTR_Y},width=${AREA_WIDTH3v},height=${AREA_HEIGHT},lane_width=40,degs=0,startx=0,starty=0"
  else
    LAWNMOWER1="format=lawnmower,x=${LM_CTR_X1},y=${LM_CTR_Y},width=${AREA_WIDTH3v},height=${AREA_HEIGHT},lane_width=20,degs=0,startx=0,starty=0"
    LAWNMOWER2="format=lawnmower,x=${LM_CTR_X},y=${LM_CTR_Y},width=${AREA_WIDTH3v},height=${AREA_HEIGHT},lane_width=20,degs=0,startx=0,starty=0"
    LAWNMOWER3="format=lawnmower,x=${LM_CTR_X3},y=${LM_CTR_Y},width=${AREA_WIDTH3v},height=${AREA_HEIGHT},lane_width=20,degs=0,startx=0,starty=0"
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
if [ "${ADAPTIVE}" = "yes" ] ; then
  script_location=`locate create_sample_path.m`
  if [ -z $script_location ] ; # if string length zero
  then
    echo "Cannot find create_sample_path.m, exiting"
    exit 0
  fi

  # create_sample_path(temp, offset_x, offset_y, max_x, max_y, grid_resolution, 
  #                    do_plot, run_nr, max_wpts, max_length, output)
  SOFTMAX_TEMP=0.6 # 6 from paper, div by 10 for distance scaling

  echo "Calling Matlab to find initial waypts (this may take a while..)"

  if [ ${AREA} = "bigger1" ] ; then
    OFFSET_Y=900
    OFFSET_X1=600
  elif [ ${AREA} = "bigger2" ] ; then
    OFFSET_Y=700
    OFFSET_X1=700
  else
    OFFSET_Y=1000
    OFFSET_X1=500
  fi

  if [ $NUM_VEHICLES -eq 1 ] ; then
    waypts=$(matlab -nodesktop -nosplash -r "[stat,result] = system('locate create_sample_path.m'); addpath(result(1:length(result)-21)); answ=create_sample_path($SOFTMAX_TEMP,$OFFSET_X1,$OFFSET_Y,$AREA_WIDTH,$AREA_HEIGHT,10); disp(answ); quit" |  cut -d= -f2 | sed 's/ //g' | tail -n2 | head -n1)
    PILOT_PTS1=${waypts}
    echo " softmax waypoints auv1: " $PILOT_PTS1
    CROSS_END1=$(echo ${waypts##*:})

  elif [ $NUM_VEHICLES -eq 2 ] ; then
    #OFFSET_X2=$[OFFSET_X1+AREA_WIDTH2v]
    AREA_HEIGHT_V2=$[AREA_HEIGHT/2]
    OFFSET_Y2=$[OFFSET_Y+AREA_HEIGHT_V2]

    waypts=$(matlab -nodesktop -nosplash -r "[stat,result] = system('locate create_sample_path.m'); addpath(result(1:length(result)-21)); answ=create_sample_path($SOFTMAX_TEMP,$OFFSET_X1,$OFFSET_Y,$AREA_WIDTH,$AREA_HEIGHT_V2,10); disp(answ); quit" |  cut -d= -f2 | sed 's/ //g' | tail -n2 | head -n1)
    PILOT_PTS1=${waypts}
    echo " softmax waypoints auv1: " $PILOT_PTS1
    CROSS_END1=$(echo ${waypts##*:})
    
    sleep 1

    waypts=$(matlab -nodesktop -nosplash -r "[stat,result] = system('locate create_sample_path.m'); addpath(result(1:length(result)-21)); answ=create_sample_path($SOFTMAX_TEMP,$OFFSET_X1,$OFFSET_Y2,$AREA_WIDTH,$AREA_HEIGHT_V2,10); disp(answ); quit" |  cut -d= -f2 | sed 's/ //g' | tail -n2 | head -n1)
    PILOT_PTS2=${waypts}
    echo " softmax waypoints auv2: " $PILOT_PTS2
    CROSS_END2=$(echo ${waypts##*:})
    
  elif [ $NUM_VEHICLES -eq 3 ] ; then
    OFFSET_X2=$[OFFSET_X1+AREA_WIDTH3v]
    OFFSET_X3=$[OFFSET_X1+2*AREA_WIDTH3v]

    waypts=$(matlab -nodesktop -nosplash -r "[stat,result] = system('locate create_sample_path.m'); addpath(result(1:length(result)-21)); answ=create_sample_path($SOFTMAX_TEMP,$OFFSET_X1,$OFFSET_Y,$AREA_WIDTH3v,$AREA_HEIGHT,10); disp(answ); quit" |  cut -d= -f2 | sed 's/ //g' | tail -n2 | head -n1)
    PILOT_PTS1=${waypts}
    echo " softmax waypoints auv1: " $PILOT_PTS1
    CROSS_END1=$(echo ${waypts##*:})

    waypts=$(matlab -nodesktop -nosplash -r "[stat,result] = system('locate create_sample_path.m'); addpath(result(1:length(result)-21)); answ=create_sample_path(1,$OFFSET_X2,$OFFSET_Y,$AREA_WIDTH3v,$AREA_HEIGHT,10); disp(answ); quit" |  cut -d= -f2 | sed 's/ //g' | tail -n2 | head -n1)
    PILOT_PTS2=${waypts}
    echo " softmax waypoints auv2: " $PILOT_PTS2
    CROSS_END2=$(echo ${waypts##*:})
    
    waypts=$(matlab -nodesktop -nosplash -r "[stat,result] = system('locate create_sample_path.m'); addpath(result(1:length(result)-21)); answ=create_sample_path($SOFTMAX_TEMP,$OFFSET_X3,$OFFSET_Y,$AREA_WIDTH3v,$AREA_HEIGHT,10); disp(answ); quit" |  cut -d= -f2 | sed 's/ //g' | tail -n2 | head -n1)
    PILOT_PTS3=${waypts}
    echo " softmax waypoints auv3: " $PILOT_PTS3
    CROSS_END3=$(echo ${waypts##*:})
  fi
fi

sleep 10

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

SERVERHOST="localhost"
nsplug meta_shoreside.moos targ_shoreside.moos -f WARP=$TIME_WARP \
   VNAME="shoreside" USC_DATA_DIR="../../../data"        \
   SHARE_LISTEN=$SHORE_LISTEN VPORT=$SHORE_VPORT SERVER_HOST=$SERVERHOST       \
   LOCATION=$EXP_LOCATION  PLUG_DIR=$PLUGDIR  MSG_DIR=$MSGDIR   \
   PAINT_SEGLIST=$PAINTSEGLIST   SIMULATION=$RUN_SIMULATION     \
   DROP_PERCENTAGE=$DROP_PCT  USE_GUI=$GUI  COMMS_RANGE=$COMMUNICATION_RANGE

# START HEADING same for all vehicles - can be customized (not needed here)
START_HEADING="230"

if [ ${AREA} = "bigger2" ] ; then
  START_POS1="650,1020"
  START_POS2="650,980"
  START_POS3="650,960"
  START_POS_SHUB="1000,1000"
  WAYPOINTS_SHUB="1000,1000"

  HP_LOITER_CONFIG="format=radial,x=650,y=1030,radius=10,pts=4,snap=1,label=hp_optim_loiter"
  if [ $NUM_VEHICLES -ge 2 ] ; then
  HP_LOITER_CONFIG2="format=radial,x=650,y=970,radius=10,pts=4,snap=1,label=hp_optim_loiter"
  fi
  if [ $NUM_VEHICLES -ge 3 ] ; then
  HP_LOITER_CONFIG3="format=radial,x=650,y=950,radius=10,pts=4,snap=1,label=hp_optimloiter"
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

# for all
VEH_SPEED="1.5"
SAMPLE_DEPTH="5"

# The first vehicle community
VNAME1="anna"
WAYPOINTS1="455,980:455,965:430,965:430,980:455,980"
MODEMID1="1"
VTYPE1="UUV" # UUV, SHIP
PREDICTIONS_PREFIX1="${VNAME1}_predictions"

# The second vehicle community
VNAME2="bernard"
WAYPOINTS2="455,980:455,965:430,965:430,980:455,980"
MODEMID2="2"
VTYPE2="UUV" # UUV, SHIP
PREDICTIONS_PREFIX2="${VNAME2}_predictions"

# The third vehicle community
VNAME3="cornelis"
WAYPOINTS3="455,980:455,965:430,965:430,980:455,980"
MODEMID3="3"
VTYPE3="UUV" # UUV, SHIP
PREDICTIONS_PREFIX3="${VNAME3}_predictions"

# 4th vehicle community: surface hub
SHUB="shub"
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
   SURVEY_AREA=$AREA    SURVEY_DEPTH=$SAMPLE_DEPTH  \
   DATA_NUM_DIMENSIONS=$DATA_NR_DIMENSIONS          \
   USE_SHUB=$WITH_SHUB  BHV_SPEED=$VEH_SPEED
nsplug meta_vehicle.bhv targ_$VNAME1.bhv -f VNAME=$VNAME1      \
    START_POS=$START_POS1 WAYPOINTS=$WAYPOINTS1                \
    START_DEPTH=$SAMPLE_DEPTH VTYPE=$VTYPE1                    \
    LAWNMOWER_NS=$LAWNMOWERNS LAWNMOWER_EW=$LAWNMOWEREW        \
    HP_LOITER=$HP_LOITER_CONFIG  ADAPTIVE_WPTS=$ADAPTIVE       \
    OTHER_VEHICLE=$VNAME2 OPREGION=$BHVOPREGION                \
    PILOT_PTS=$PILOT_PTS1  CROSS_END_WPT=$CROSS_END1           \
    BHV_SPEED=$VEH_SPEED
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
   SURVEY_AREA=$AREA    SURVEY_DEPTH=$SAMPLE_DEPTH  \
   DATA_NUM_DIMENSIONS=$DATA_NR_DIMENSIONS          \
   USE_SHUB=$WITH_SHUB  BHV_SPEED=$VEH_SPEED
nsplug meta_vehicle.bhv targ_$VNAME2.bhv -f VNAME=$VNAME2      \
    START_POS=$START_POS2 WAYPOINTS=$WAYPOINTS2                \
    START_DEPTH=$SAMPLE_DEPTH VTYPE=$VTYPE2                    \
    LAWNMOWER_NS=$LAWNMOWERNS2 LAWNMOWER_EW=$LAWNMOWEREW2      \
    HP_LOITER=$HP_LOITER_CONFIG2  ADAPTIVE_WPTS=$ADAPTIVE      \
    OTHER_VEHICLE=$VNAME1 OPREGION=$BHVOPREGION                \
    PILOT_PTS=$PILOT_PTS2  CROSS_END_WPT=$CROSS_END2           \
    BHV_SPEED=$VEH_SPEED
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
   SURVEY_AREA=$AREA    SURVEY_DEPTH=$SAMPLE_DEPTH  \
   DATA_NUM_DIMENSIONS=$DATA_NR_DIMENSIONS          \
   USE_SHUB=$WITH_SHUB  BHV_SPEED=$VEH_SPEED
nsplug meta_vehicle.bhv targ_$VNAME3.bhv -f VNAME=$VNAME3      \
    START_POS=$START_POS3 WAYPOINTS=$WAYPOINTS3                \
    START_DEPTH=$SAMPLE_DEPTH VTYPE=$VTYPE3                    \
    LAWNMOWER_NS=$LAWNMOWERNS3 LAWNMOWER_EW=$LAWNMOWEREW3      \
    HP_LOITER=$HP_LOITER_CONFIG3  ADAPTIVE_WPTS=$ADAPTIVE      \
    OTHER_VEHICLE=$VNAME2 OPREGION=$BHVOPREGION                \
    PILOT_PTS=$PILOT_PTS3  CROSS_END_WPT=$CROSS_END3           \
    BHV_SPEED=$VEH_SPEED
fi
# TODO fix OTHER_VEHICLE

if [ ${WITH_SHUB} = "yes" ] ; then
# v4: surface hub
SHAREGP2=$ANNA_LISTEN_GP
SHAREGP3=$BERNARD_LISTEN_GP
nsplug meta_shub.moos targ_$SHUB.moos -f WARP=$TIME_WARP  \
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
   USE_GUI=$GUI  SURVEY_AREA=$AREA  DATA_NUM_DIMENSIONS=$DATA_NR_DIMENSIONS \
   USE_SHUB=$WITH_SHUB  USE_VORONOI=$VORONOI_PARTITIONING
nsplug meta_shub.bhv targ_shub.bhv -f VNAME=$SHUB      \
    START_POS=$START_POS_SHUB WAYPOINTS=$WAYPOINTS_SHUB                \
    START_DEPTH=$START_DEPTH_SHUB VTYPE=$VTYPE_SHUB                    \
    HP_LOITER=$HP_LOITER_CONFIG_SHUB  ADAPTIVE_WPTS=$ADAPTIVE       \
    OTHER_VEHICLE=$VNAME2 OPREGION=$BHVOPREGION
fi

if [ ${JUST_MAKE} = "yes" ] ; then
  echo "Using option JUST_MAKE (-j), exiting."
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

if [ ${WITH_SHUB} = "yes" ] ; then
# surface hub
printf "Launching $SHUB MOOS Community (WARP=%s) \n" $TIME_WARP
pAntler targ_$SHUB.moos > log_$SHUB.log &
sleep .25
fi

printf "Done \n"

uMAC targ_shoreside.moos

printf "Killing all processes ... \n"
kill %1 %2 %3 %4 %5
printf "Done killing processes.   \n"
