inversekinematics
===============================

##New component for inverse kinematics

This is the new version for the BIK component. This component includes new enhancements such as:

1) Executes more than once a target. The inverse kinematic result is not the same if the start point of the effector is the home or a pointB near tho the goal point.
2) Executes the traslations without the motors of the wrisht (only for Ursus).


##Configuration parameters
We can connect this component with other components through the port 10220.

Like all the components of Robocomp, inversekinematics needs a configuration file to start. You can see one example in

    etc/configDefinitivo

We can find there the following lines:

    BodyInverseKinematics.Endpoints=tcp -p 10220 									#The port of the component
    CommonBehavior.Endpoints=tcp -p 12207
    JoystickAdapterTopic=tcp -p 12226
    
    JointMotorProxy = jointmotor:tcp -p 20000 										#We need the ursuscommonjoint in order to move the motors of the robot.
    DifferentialRobotProxy = differentialrobot:tcp  -p 10004 -h localhost			#This is the base of the robot
    OmniRobotProxy = omnirobot:tcp  -p 12238 -h localhost							
    InnerModelManagerProxy = innermodelmanager:tcp  -p 11175 -h localhost			#To manage the InnerModel
    
    InnerModel=/home/robocomp/robocomp/components/robocomp-ursus/etc/ursus.xml		#A model of the robot Ursus

    #Motors of the robot:
	RIGHTARM=rightShoulder1;rightShoulder2;rightShoulder3;rightElbow;rightForeArm;rightWrist1;rightWrist2
	RIGHTTIP=grabPositionHandR
	LEFTARM=leftShoulder1;leftShoulder2;leftShoulder3;leftElbow;leftForeArm;leftWrist1;leftWrist2
	LETFTIP=grabPositionHandL
	HEAD=head_yaw_joint;head_pitch_joint
	HEADTIP=rgbd_transform
	
	TopicManager.Proxy=IceStorm/TopicManager:default -p 9999
	
	Ice.Warn.Connections=0
	Ice.Trace.Network=0
	Ice.Trace.Protocol=0
	Ice.ACM.Client=10
	Ice.ACM.Server=10

    
##Starting the component
To avoid changing the *config* file in the repository, we can copy it to the component's home directory, so changes will remain untouched by future git pulls:

    cd

``` <inversekinematics 's path> ```

    cp etc/config config
    
After editing the new config file we can run the component:

    bin/

```inversekinematics ```

    --Ice.Config=config
