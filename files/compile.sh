
sudo sudo

#number of compiling threads
N=3


### Help
help()
{
	echo "./compile.sh 0 --> update and compile all components"
	echo "./compile.sh 1 --> compile NUC1 components"
	echo "./compile.sh 2 --> compile NUC2 components"
	echo "./compile.sh 3 --> compile NUC3 components"
	exit
}

compile()
{
	echo "make $1"
	cd $2
	if [ ! -f Makefile ]; then
		cmake .
	fi
	make -j$N
	if [ $? -ne 0 ]; then
		echo "error compiling $1"
		exit
	fi
}


if [ $# -eq 0 ]; then
	help
fi

if [ $1 -ne 1 ] && [ $1 -ne 2 ] && [ $1 -ne 3 ] && [ $1 -ne 0 ]; then
	help
fi




###
### Update repositories
###
# AGM
echo "update agm"
cd /home/robocomp/AGM
git pull

# robocomp
echo "update robocomp"
cd /home/robocomp/robocomp
git pull
# robocomp-robolab
echo "update robocomp-robolab"
cd /home/robocomp/robocomp/components/robocomp-robolab
git pull
# robocomp-shelly
echo "update robocomp-shelly"
cd /home/robocomp/robocomp/components/robocomp-shelly
git pull
# prp
echo "update prp"
cd /home/robocomp/robocomp/components/prp
git pull

sleep 4


###
### RoboComp
###
echo "make robocomp"
cd /home/robocomp/robocomp/build
make -j$N
if [ $? -ne 0 ]; then
	echo "error compiling robocomp"
	exit
fi
echo "make install robocomp"
sudo make install
if [ $? -ne 0 ]; then
	echo "error installing robocomp"
	exit
fi

###
### AGM
###
echo "make agm"
cd /home/robocomp/AGM
if [ ! -f Makefile ]; then
	cmake . -DROBOCOMP_SUPPORT=True
fi
make -j$N
if [ $? -ne 0 ]; then
	echo "error compiling agm"
	exit
fi
echo "make install agm"
sudo make install
if [ $? -ne 0 ]; then
	echo "error installing robocomp"
	exit
fi


###
### COMPONENTS 
###

# ALWAYS
# april localization
compile "april localization" "/home/robocomp/robocomp/components/robocomp-robolab/experimental/aprilBasedPublish/"
# manual localization 
compile "manual localization" "/home/robocomp/robocomp/components/robocomp-shelly/components/manualLocalization"
# joystickcomp
compile "joystickcomp" "/home/robocomp/robocomp/components/robocomp-robolab/components/joystickOmniComp/"



# NUC 1
if [ $1 -eq 1 ] || [ $1 -eq 0 ]; then
	echo "Compiling NUC1 components"
	# cgr
	compile "cgr" "/home/robocomp/robocomp/components/robocomp-robolab/experimental/CGR/"
	# stable odometry
	compile "stable odometry" "/home/robocomp/robocomp/components/robocomp-robolab/experimental/stableOdometry"
	# trajectory
	#compile "trajectory" "/home/robocomp/robocomp/components/robocomp-shelly/components/trajectoryrobot2d/"
	# laserRGBD
	compile "laserRGBD" "/home/robocomp/robocomp/components/robocomp-robolab/experimental/laserRGBDComp2/"
	# base
	compile "baseursus" "/home/robocomp/robocomp/components/robocomp-shelly/components/baseursus/"
	# ursuscommonjoint
	compile "ursuscommonjoint" "/home/robocomp/robocomp/components/robocomp-shelly/components/ursusCommonJoint/"
	# dunker
	compile "dunker" "/home/robocomp/robocomp/components/robocomp-robolab/components/dunkermotorenComp"
	# dynamixel
	compile "dynamixel" "/home/robocomp/robocomp/components/robocomp-robolab/components/dynamixelComp"
	# faulhaber
	compile "faulhaber" "/home/robocomp/robocomp/components/robocomp-shelly/components/faulhaberComp"
	# inversekinematics
	compile "ik" "/home/robocomp/robocomp/components/robocomp-shelly/components/inversekinematics"
	# gik visual
	compile "gik" "/home/robocomp/robocomp/components/robocomp-shelly/components/ikGraphGenerator/"
	# ik visual
	compile "ik visual" "/home/robocomp/robocomp/components/robocomp-shelly/components/visualik/"
	# dumb trajectory
	compile "dumb trajectory" "/home/robocomp/robocomp/components/robocomp-shelly/components/dumbtrajectoryrobot2d"
fi

# NUC 2
if [ $1 -eq 2 ] || [ $1 -eq 0 ]; then
	echo "Compiling NUC2 components"
	# navigationAgent
	compile "navigation agent" "/home/robocomp/robocomp/components/robocomp-shelly/components/navigationAgent/"
	# proprioceptionAgent
	compile "proprioceptionAgent agent" "/home/robocomp/robocomp/components/robocomp-shelly/components/proprioceptionAgent/"
	# graspingAgent
	compile "grasping agent" "/home/robocomp/robocomp/components/robocomp-shelly/components/graspingAgent/"
	# objectAgent
	compile "object agent" "/home/robocomp/robocomp/components/robocomp-shelly/components/objectagent/"
	# oracle
	compile "oracle agent" "/home/robocomp/robocomp/components/prp/components/objectOracle/"
	# human
# 	compile "human agent" "/home/robocomp/robocomp/components/robocomp-shelly/components/humanAgent/"
fi


# NUC 3
if [ $1 -eq 3 ] || [ $1 -eq 0 ]; then
	echo "Compiling NUC3 components"
	# primesense
	compile "primesense" "/home/robocomp/robocomp/components/robocomp-robolab/components/openni2RGBD/"
	# apriltags
	compile "apriltags" "/home/robocomp/robocomp/components/robocomp-robolab/components/apriltagsComp/"
	# hokuyo
	compile "hokuyo" "/home/robocomp/robocomp/components/robocomp-robolab/components/hokuyoComp"
fi

