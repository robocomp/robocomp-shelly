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

if [ $1 -ne 1 ] && [ $1 -ne 2 ] && [ $1 -ne 3 ] && [ $1 -ne 0 ] && [ $1 -ne -1 ]; then
	help
fi


sudo sudo 2> /dev/null

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
# joystickcomp
compile "joystickcomp" "/home/robocomp/robocomp/components/robocomp-robolab/components/joystickOmniComp/"



# NUC 1
if [ $1 -eq 1 ] || [ $1 -eq 0 ]; then
	echo "Compiling NUC1 components"
	sleep 1
fi


	
	
# gmapping
if [ $1 -eq 1 ] || [ $1 -eq 0 ] || [ $1 -eq -1 ]; then
	compile "gmapping" "/home/robocomp/robocomp/components/robocomp-robolab/experimental/gmappingComp/"
fi
# stable odometry
if [ $1 -eq 1 ] || [ $1 -eq 0 ] || [ $1 -eq -1 ]; then
	compile "stable odometry" "/home/robocomp/robocomp/components/robocomp-robolab/experimental/stableOdometry"
fi
# trajectory
# if [ $1 -eq 1 ] || [ $1 -eq 0 ]; then
	#compile "trajectory" "/home/robocomp/robocomp/components/robocomp-shelly/components/trajectoryrobot2d/"
# fi
# laserRGBD
if [ $1 -eq 1 ] || [ $1 -eq 0 ] || [ $1 -eq -1 ]; then
	compile "laserRGBD" "/home/robocomp/robocomp/components/robocomp-robolab/experimental/laserRGBDComp2/"
fi
# base
if [ $1 -eq 1 ] || [ $1 -eq 0 ]; then
	compile "baseursus" "/home/robocomp/robocomp/components/robocomp-shelly/components/baseursus/"
fi
# ursuscommonjoint
if [ $1 -eq 1 ] || [ $1 -eq 0 ] || [ $1 -eq -1 ]; then
	compile "ursuscommonjoint" "/home/robocomp/robocomp/components/robocomp-shelly/components/ursusCommonJoint/"
fi
# dunker
if [ $1 -eq 1 ] || [ $1 -eq 0 ]; then
	compile "dunker" "/home/robocomp/robocomp/components/robocomp-robolab/components/dunkermotorenComp"
fi
# dynamixel
if [ $1 -eq 1 ] || [ $1 -eq 0 ]; then
	compile "dynamixel" "/home/robocomp/robocomp/components/robocomp-robolab/components/dynamixelComp"
fi
# faulhaber
if [ $1 -eq 1 ] || [ $1 -eq 0 ]; then
	compile "faulhaber" "/home/robocomp/robocomp/components/robocomp-shelly/components/faulhaberComp"
fi
# inversekinematics
if [ $1 -eq 1 ] || [ $1 -eq 0 ] || [ $1 -eq -1 ]; then
	compile "ik" "/home/robocomp/robocomp/components/robocomp-shelly/components/inversekinematics"
fi
# gik visual
if [ $1 -eq 1 ] || [ $1 -eq 0 ] || [ $1 -eq -1 ]; then
	compile "gik" "/home/robocomp/robocomp/components/robocomp-shelly/components/ikGraphGenerator/"
fi
# ik visual
if [ $1 -eq 1 ] || [ $1 -eq 0 ] || [ $1 -eq -1 ]; then
	compile "ik visual" "/home/robocomp/robocomp/components/robocomp-shelly/components/visualik/"
fi
# dumb trajectory
if [ $1 -eq 1 ] || [ $1 -eq 0 ] || [ $1 -eq -1 ]; then
	compile "dumb trajectory" "/home/robocomp/robocomp/components/robocomp-shelly/components/dumbtrajectoryrobot2d"
fi



# NUC 2
if [ $1 -eq 2 ] || [ $1 -eq 0 ]; then
	echo "Compiling NUC2 components"
	sleep 1
fi

# navigationAgent
if [ $1 -eq 2 ] || [ $1 -eq 0 ] || [ $1 -eq -1 ]; then
	compile "navigation agent" "/home/robocomp/robocomp/components/robocomp-shelly/components/navigationAgent/"
fi
# proprioceptionAgent
if [ $1 -eq 2 ] || [ $1 -eq 0 ] || [ $1 -eq -1 ]; then
	compile "proprioceptionAgent agent" "/home/robocomp/robocomp/components/robocomp-shelly/components/proprioceptionAgent/"
fi
# graspingAgent
if [ $1 -eq 2 ] || [ $1 -eq 0 ] || [ $1 -eq -1 ]; then
	compile "grasping agent" "/home/robocomp/robocomp/components/robocomp-shelly/components/graspingAgent/"
fi
# objectAgent
if [ $1 -eq 2 ] || [ $1 -eq 0 ] || [ $1 -eq -1 ]; then
	compile "object agent" "/home/robocomp/robocomp/components/robocomp-shelly/components/objectagent/"
fi
# oracle
if [ $1 -eq 2 ] || [ $1 -eq 0 ] || [ $1 -eq -1 ]; then
	compile "oracle agent" "/home/robocomp/robocomp/components/prp/components/objectOracle/"
fi
# human
# if [ $1 -eq 2 ] || [ $1 -eq 0 ] || [ $1 -eq -1 ]; then
# 	compile "human agent" "/home/robocomp/robocomp/components/robocomp-shelly/components/humanAgent/"
# fi


# NUC 3
if [ $1 -eq 3 ] || [ $1 -eq 0 ]; then
	echo "Compiling NUC3 components"
	sleep 1
fi

# primesense
if [ $1 -eq 3 ] || [ $1 -eq 0 ]; then
	compile "primesense" "/home/robocomp/robocomp/components/robocomp-robolab/components/openni2RGBD/"
fi
# apriltags
if [ $1 -eq 3 ] || [ $1 -eq 0 ] || [ $1 -eq -1 ]; then
	compile "apriltags" "/home/robocomp/robocomp/components/robocomp-robolab/components/apriltagsComp/"
fi
# hokuyo
if [ $1 -eq 3 ] || [ $1 -eq 0 ]; then
	compile "hokuyo" "/home/robocomp/robocomp/components/robocomp-robolab/components/hokuyoComp"
fi

