
#faulhaberComp
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession
sess=`qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.activeSessionId`
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'cd /home/robocomp/robocomp/components/robocomp-ursus/components/faulhaberComp/bin'
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'killall -9 faulhaberComp' 
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand './faulhaberComp --Ice.Config=/home/robocomp/robocomp/components/robocomp-ursus/etc/faulhaber.conf'
qdbus org.kde.yakuake /yakuake/tabs org.kde.yakuake.setTabTitle $sess 'Faulhaber'
sleep 2

#dynamixel
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession
sess=`qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.activeSessionId`
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'cd /home/robocomp/robocomp/components/robocomp-robolab/components/dynamixelComp/bin'
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'killall -9 dynamixelComp'
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand './dynamixelComp --Ice.Config=/home/robocomp/robocomp/components/robocomp-ursus/etc/dynamixel_head.conf'
qdbus org.kde.yakuake /yakuake/tabs org.kde.yakuake.setTabTitle $sess 'Dynamixel'
sleep 2

#joyArm
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession
sess=`qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.activeSessionId`
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'cd /home/robocomp/robocomp/components/robocomp-ursus/components/joystickArmComp'
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'killall -9 joystickarmcomp'
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'bin/joystickarmcomp --Ice.Config=/home/robocomp/robocomp/components/robocomp-ursus/etc/joystickArm.conf'
qdbus org.kde.yakuake /yakuake/tabs org.kde.yakuake.setTabTitle $sess 'joystickarmcomp'
sleep 2

#joystickUniversal
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession
sess=`qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.activeSessionId`
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'cd /home/robocomp/robocomp/components/robocomp-robolab/components/joystickuniversalComp'
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'killall -9 joystickuniversalComp'
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'bin/joystickuniversalComp --Ice.Config=/home/robocomp/robocomp/components/robocomp-ursus/etc/joystickUniversal.conf'
qdbus org.kde.yakuake /yakuake/tabs org.kde.yakuake.setTabTitle $sess 'joystickuniversalComp'
sleep 2

#ursusHead
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession
sess=`qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.activeSessionId`
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'cd /home/robocomp/robocomp/components/robocomp-ursus/components/ursusHeadComp'
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'killall -9 ursusheadcomp'
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'bin/ursusheadcomp --Ice.Config=/home/robocomp/robocomp/components/robocomp-ursus/etc/ursusHead.conf'
qdbus org.kde.yakuake /yakuake/tabs org.kde.yakuake.setTabTitle $sess 'ursusheadcomp'
sleep 2









