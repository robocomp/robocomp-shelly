
faulhaberComp
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession
sess=`qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.activeSessionId`
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'cd /home/robocomp/robocomp/Components/Ursus/faulhaberComp/bin'
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand './faulhaberComp --Ice.Config=/home/robocomp/robocomp/Components/Ursus/etc/faulhaber.conf'
qdbus org.kde.yakuake /yakuake/tabs org.kde.yakuake.setTabTitle $sess 'faulhaber'
sleep 10

#joystickArmComp
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession
sess=`qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.activeSessionId`
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'cd /home/robocomp/robocomp/Components/Ursus/joystickArmComp/bin'
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand './joystickarmcomp --Ice.Config=/home/robocomp/robocomp/Components/Ursus/etc/joystickArm.conf'
qdbus org.kde.yakuake /yakuake/tabs org.kde.yakuake.setTabTitle $sess 'joyARM'
sleep 1
#primeSense
# qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession
# sess=`qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.activeSessionId`
# qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'cd /home/robocomp/robocomp/Components/RoboLab/Experimental/PrimeSenseComp/bin/'
# qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand './primeSenseComp --Ice.Config=/home/robocomp/robocomp/Components/Ursus/etc/primeSense.conf'
# qdbus org.kde.yakuake /yakuake/tabs org.kde.yakuake.setTabTitle $sess 'primeSense'

# RGBD viewer
# qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession
# sess=`qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.activeSessionId`
# qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'cd /home/robocomp/robocomp/Components/RoboLab/Experimental/rgbdViewerComp/bin'
# qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand './rgbdviewercomp --Ice.Config=/home/robocomp/robocomp/Components/Ursus/etc/rgbdviewer.conf '
# qdbus org.kde.yakuake /yakuake/tabs org.kde.yakuake.setTabTitle $sess 'rgbdviewer'

#ursusHeadComp
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession
sess=`qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.activeSessionId`
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'cd /home/robocomp/robocomp/Components/Ursus/ursusHeadComp/bin'
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand './ursusheadcomp --Ice.Config=/home/robocomp/robocomp/Components/Ursus/etc/ursusHead.conf'
qdbus org.kde.yakuake /yakuake/tabs org.kde.yakuake.setTabTitle $sess 'Head&Pinza'
sleep 1
#Camera	
# qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession
# sess=`qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.activeSessionId`
# qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'cd /home/robocomp/robocomp/Components/HAL/cameraComp/bin'
# qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand './cameraComp --Ice.Config=../etc/config'
# qdbus org.kde.yakuake /yakuake/tabs org.kde.yakuake.setTabTitle $sess 'camera'
sleep 1

#joystickComp
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession
sess=`qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.activeSessionId`
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'cd /home/robocomp/robocomp/Components/RoboLab/Experimental/joystickuniversalComp/bin'
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand './joystickuniversalComp --Ice.Config=/home/robocomp/robocomp/Components/Ursus/etc/joystickUniversal.conf'
qdbus org.kde.yakuake /yakuake/tabs org.kde.yakuake.setTabTitle $sess 'joystick'
sleep 1
#dynamixel
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession
sess=`qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.activeSessionId`
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'cd /home/robocomp/robocomp/Components/HAL/dynamixelComp/bin'
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand './dynamixelComp --Ice.Config=/home/robocomp/robocomp/Components/Ursus/etc/dynamixel_head.conf'
qdbus org.kde.yakuake /yakuake/tabs org.kde.yakuake.setTabTitle $sess 'dynamixel'
sleep 1
# Camera Monitor
# qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession
# sess=`qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.activeSessionId`
# qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'cd /home/robocomp/'
# qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'cheese'
# qdbus org.kde.yakuake /yakuake/tabs org.kde.yakuake.setTabTitle $sess 'cheese'
# sleep 1
# Soft jose (debug mode jaja)
# qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession
# sess=`qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.activeSessionId`
# qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'cd /home/robolab/Software/BrazoUrsus3'
# qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'sudo ./canbus /dev/ttyUSB0 /dev/ttyUSB1'
# qdbus org.kde.yakuake /yakuake/tabs org.kde.yakuake.setTabTitle $sess 'joyArm'

