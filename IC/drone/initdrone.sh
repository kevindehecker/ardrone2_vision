# install framework
killall program.elf.respawner.sh
killall -9 program.elf
killall -9 ap.elf
killall -9 IC

mkdir -p /data/video/stick
mount /dev/sda1 /data/video/stick/
mkdir /data/video/drone/build -p
mkdir -p /opt/
ln -s /data/video/stick/arm_light/ /opt/arm
ln -s /data/video/stick/arm_light/ /opt/arm_light
ln -s /data/video/stick/drone/ drone
export PATH=/opt/arm/gst/bin:$PATH

#install video frane work dsp files!
mount /dev/sda1 /data/video/stick/
cd /data/vide/stick/arm_full
tar -xzf  magicalworkingdspfiles.tar.gz
cd lib/dsp
mkdir -p /lib/dsp
mv * /lib/dsp/
reboot

#normal start:
killall program.elf.respawner.sh
killall -9 program.elf
mount /dev/sda1 /data/video/stick/
export PATH=/opt/arm_light/gst/bin:$PATH
cd /data/video/drone/build

killall -9 ap.elf && ./IC


