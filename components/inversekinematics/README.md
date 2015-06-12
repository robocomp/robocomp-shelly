inversekinematics
===============================
##What is Inverse Kinematic (IK)?

The inverse kinematic (IK) is responsible for calculating the angle of each motor (joint) of a kinematic chain, in order to move the end effector from a start point A to an endpoint B.

This IK component uses the Levenberg-Marquardt algorithm to solve this problem.

##New component for inverse kinematics

This is the new version for the BIK component. Its operation is simple, you only have to tell the component somo important things:

1) the robot whicht will calculate the IK, for example, Ursus. You must put the path of the robot.xml file

    InnerModel=/home/robocomp/robocomp/components/robocomp-ursus/etc/ursus.xml

2) the chains that the robot have, for example, 

    LEFTARM
    RIGHTARM
    HEAD

3) the motors that make up the chains, for example,

    LEFTARM=leftShoulder1;leftShoulder2;leftShoulder3;leftElbow;leftForeArm;leftWrist1;leftWrist2

4) And, finally, the tip or end effector of the chain, for example, 
    
    LETFTIP=grabPositionHandL.

When you send a target position (with his traslation and rotation), you must indicates the chain that will run the target, and the target traslations and rotations weights. 

This revision of the component includes some new enhancements such as:

1) Executes more than once a target. The inverse kinematic result is not the same if the start point of the effector is the robot's home or a point B near tho the goal point.

2) Executes the traslations without the motors of the wrisht (only for Ursus). This makes possible to move the arm with stiff wrist, and then we can rotate easely the wrist when the end effectos is near the target.


##Configuration parameters
We can connect this component with other components through the port 10220.

Like all the components of Robocomp, inversekinematics needs a configuration file to start. You can see one example in

    etc/configDefinitivo

We can find there the following lines:

    BodyInverseKinematics.Endpoints=tcp -p 10220 								#The port of the component
    CommonBehavior.Endpoints=tcp -p 12207
    JoystickAdapterTopic=tcp -p 12226
    
    JointMotorProxy = jointmotor:tcp -p 20000 									#We need the ursuscommonjoint in
                                                                                #order to move the motors of the robot.
    DifferentialRobotProxy = differentialrobot:tcp  -p 10004 -h localhost		#This is the base of the robot
    OmniRobotProxy = omnirobot:tcp  -p 12238 -h localhost							
    InnerModelManagerProxy = innermodelmanager:tcp  -p 11175 -h localhost		#To manage the InnerModel
    
    InnerModel=/home/robocomp/robocomp/components/robocomp-ursus/etc/ursus.xml	#A model of the robot Ursus

    #Motors of the robot Ursus:
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

    cd /robocomp/components/robocomp-ursus/components/inversekinematics
    cp etc/config ourConfig
    
After editing the new config file (ourConfig) we can run the component:

    ./bin/inversekinematics ourConfig
    
It's not necessary put --Ice.Config=.