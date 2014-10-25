
ftp 192.168.1.1 <<script

cd drone
cd build
put IC
bye
script

telnet 192.168.1.1 <<script2

cd data
cd video
cd drone
cd build
chmod +x IC 
exit
script2
