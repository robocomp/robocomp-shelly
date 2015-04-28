echo "Faulhaber..."
faulhaber=`ls -lah /dev/faulhaber | awk '{print $11}'`
sed -ifaulhaber.back "s/Faulhaber.Device.*/Faulhaber.Device=\/dev\/$faulhaber/g" /home/robocomp/robocomp/components/robocomp-ursus-rockin/etc/faulhaber.conf

echo "Dynamixel..."
dynamixel=`ls -lah /dev/dynamixel | awk '{print $11}'`
sed -idynamixel.back "s/Dynamixel.Device.*/Dynamixel.Device=\/dev\/$dynamixel/g" /home/robocomp/robocomp/components/robocomp-ursus-rockin/etc/dynamixel.conf
