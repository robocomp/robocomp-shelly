### 
### Help
###

help()
{
	echo "./compile.sh 0 --> update and compile all components"
	echo "./compile.sh 1 --> compile NUC1 components"
	echo "./compile.sh 2 --> compile NUC2 components"
	echo "./compile.sh 3 --> compile NUC3 components"
	exit
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

N=3

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
echo "make april localization"
cd /home/robocomp/robocomp/components/robocomp-robolab/experimental/aprilBasedPublish/
cmake .
make -j$N
if [ $? -ne 0 ]; then
        echo "error compiling april localization"
        exit
fi

# joystickcomp
echo "make joystickcomp"
cd /home/robocomp/robocomp/components/robocomp-robolab/components/joystickOmniComp/
cmake .
make -j$N
if [ $? -ne 0 ]; then
        echo "error compiling joystickOmni"
        exit
fi


echo 'a'

# NUC 1
if [ $1 -eq 1 ] || [ $1 -eq 0 ]; then
    echo "Compiling NUC1 components"

    # cgr
    echo "make cgr"
    cd /home/robocomp/robocomp/components/robocomp-robolab/experimental/CGR/
    cmake .
    make -j$N
    if [ $? -ne 0 ]; then
            echo "error compiling CGR"
            exit
    fi
    
    # stable odometry
    echo "make stable odometry"
    cd /home/robocomp/robocomp/components/robocomp-robolab/experimental/stableOdometry
    cmake .
    make -j$N
    if [ $? -ne 0 ]; then
            echo "error compiling stableOdometry"
            exit
    fi
        
    # trajectory
    echo "make trajectory"
    cd /home/robocomp/robocomp/components/robocomp-shelly/components/trajectoryrobot2d/
    cmake .
    make -j$N
    if [ $? -ne 0 ]; then
            echo "error compiling trajectory"
    #	exit
    fi 

    # laserRGBD
    echo "make laserRGBD"
    cd /home/robocomp/robocomp/components/robocomp-robolab/experimental/laserRGBDComp2/
    cmake .
    make -j$N
    if [ $? -ne 0 ]; then
            echo "error compiling laserrgbd"
            exit
    fi

    # base
    echo "make baseursus"
    cd /home/robocomp/robocomp/components/robocomp-shelly/components/baseursus/
    cmake .
    make -j$N
    if [ $? -ne 0 ]; then
            echo "error compiling baseursus"
            exit
    fi

    # ursuscommonjoint
    echo "make ursuscommonjoint"
    cd /home/robocomp/robocomp/components/robocomp-shelly/components/ursusCommonJoint/
    cmake .
    make -j$N
    if [ $? -ne 0 ]; then
            echo "error compiling ursuscommonjoint"
            exit
    fi

    # dunker
    echo "make dunker"
    cd /home/robocomp/robocomp/components/robocomp-robolab/components/dunkermotorenComp
    cmake .
    make -j$N
    if [ $? -ne 0 ]; then
            echo "error compiling dunker"
            exit
    fi



    # dynamixel
    echo "make dynamixel"
    cd /home/robocomp/robocomp/components/robocomp-robolab/components/dynamixelComp
    cmake .
    make -j$N
    if [ $? -ne 0 ]; then
            echo "error compiling dynamixel"
            exit
    fi

    # faulhaber
    echo "make faulhaber"
    cd /home/robocomp/robocomp/components/robocomp-shelly/components/faulhaberComp
    cmake .
    make -j$N
    if [ $? -ne 0 ]; then
            echo "error compiling faulhaber"
            exit
    fi

    # inversekinematics
    echo "make ik"
    cd /home/robocomp/robocomp/components/robocomp-shelly/components/inversekinematics
    cmake .
    make -j$N
    if [ $? -ne 0 ]; then
            echo "error compiling IK"
            exit
    fi

    # gik visual
    echo "make gik"
    cd /home/robocomp/robocomp/components/robocomp-shelly/components/ikGraphGenerator/
    cmake .
    make -j$N
    if [ $? -ne 0 ]; then
            echo "error compiling gik"
            exit
    fi

    # ik visual
    echo "make ik visual"
    cd /home/robocomp/robocomp/components/robocomp-shelly/components/visualik/
    cmake .
    make -j$N
    if [ $? -ne 0 ]; then
            echo "error compiling visualik"
            exit
    fi

fi

# NUC 2
if [ $1 -eq 2 ] || [ $1 -eq 0 ]; then
    echo "Compiling NUC2 components"

    # navigationAgent
    echo "make navigation agent"
    cd /home/robocomp/robocomp/components/robocomp-shelly/components/navigationAgent/
    cmake .
    make -j$N
    if [ $? -ne 0 ]; then
            echo "error compiling navigation agent"
            exit
    fi

    # proprioceptionAgent
    echo "make proprioceptionAgent agent"
    cd /home/robocomp/robocomp/components/robocomp-shelly/components/proprioceptionAgent/
    cmake .
    make -j$N
    if [ $? -ne 0 ]; then
            echo "error compiling proprioception agent"
            exit
    fi

    # graspingAgent
    echo "make grasping agent"
    cd /home/robocomp/robocomp/components/robocomp-shelly/components/graspingAgent/
    cmake .
    make -j$N
    if [ $? -ne 0 ]; then
            echo "error compiling grasping agent"
            exit
    fi

    # objectAgent
    echo "make object agent"
    cd /home/robocomp/robocomp/components/robocomp-shelly/components/objectagent/
    cmake .
    make -j$N
    if [ $? -ne 0 ]; then
            echo "error compiling object agent"
            exit
    fi

    # human
    echo "make human agent"
    cd /home/robocomp/robocomp/components/robocomp-shelly/components/humanAgent/
    cmake .
    make -j$N
    if [ $? -ne 0 ]; then
            echo "error compiling human agent"
            exit
    fi

fi


# NUC 3
if [ $1 -eq 3 ] || [ $1 -eq 0 ]; then
    echo "Compiling NUC3 components"

    # primesense
    echo "make primesense"
    cd /home/robocomp/robocomp/components/robocomp-robolab/components/openni2RGBD/
    cmake .
    make -j$N
    if [ $? -ne 0 ]; then
            echo "error compiling openni2RGBD"
            exit
    fi    

    # apriltags
    echo "make apriltags"
    cd /home/robocomp/robocomp/components/robocomp-robolab/components/apriltagsComp/
    cmake .
    make -j$N
    if [ $? -ne 0 ]; then
            echo "error compiling apriltags"
            exit
    fi

    # hokuyo
    echo "make hokuyo"
    cd /home/robocomp/robocomp/components/robocomp-robolab/components/hokuyoComp
    cmake .
    make -j$N
    if [ $? -ne 0 ]; then
            echo "error compiling hokuyo"
            exit
    fi
    
fi
