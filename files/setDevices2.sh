echo "Dunker..."
dunker=`ls -lah /dev/dunker | awk '{print $11}'`
sed -idunker.back "s/dunkermotoren.Device.*/dunkermotoren.Device=\/dev\/$dunker/g" /home/robocomp/robocomp/components/robocomp-ursus-rockin/etc/dunker.conf

