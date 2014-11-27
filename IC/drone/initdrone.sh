set -ex
killall program.elf.respawner.sh
killall -9 program.elf

mkdir -p /opt/
#rm -f /opt/arm

#install magical working dsp files!


mkdir -p /data/video/stick
mount /dev/sda1 /data/video/stick/

ln -s /data/video/stick/arm_light/ /opt/arm
ln -s /data/video/stick/arm_light/ /opt/arm_light
ln -s /data/video/stick/drone/ drone

export PATH=/opt/arm/gst/bin:$PATH


cd data/video/drone/build
killall program.elf.respawner.sh
killall -9 program.elf
mount /dev/sda1 /data/video/stick/
export PATH=/opt/arm_light/gst/bin:$PATH

