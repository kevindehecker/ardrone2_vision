if [ "$#" -ne 1 ]; then
    echo "Illegal number of parameters"
    exit
fi

../downloadvids.sh 
 set -ex
 mkdir ~/AfstudeerData/DroneCam/$1
 cp video_dsp.avi ~/AfstudeerData/DroneCam/$1/
 cp ~/paparazzi/var/logs/14_12_18__* ~/AfstudeerData/DroneCam/$1/

