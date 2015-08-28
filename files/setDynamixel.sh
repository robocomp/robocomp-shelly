echo "Dynamixel..."
dynamixel=`ls -lah /dev/dynamixel | awk '{print $11}'`
sed -idynamixel.back "s/Dynamixel.Device.*/Dynamixel.Device=\/dev\/$dynamixel/g" /home/robocomp/robocomp/components/robocomp-ursus/etc/dynamixel.conf
