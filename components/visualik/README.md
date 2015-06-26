visualik
===============================
##What is Visual Inverse Kinematic (visualIK)?
In the [`inversekinematic` component](https://github.com/robocomp/robocomp-ursus/blob/master/components/inversekinematics/README.md) we explained the principles on which the inverse kinematics of a robot is based and the operation of it. But one of the problems that the inverse kinematics can not solve by itself is the problem of gaps and imperfections of the robot. These gaps and inaccuracies make the robot move its arm toward the target position improperly, so that the robot “thinks” that the end effector has reached the target but in reality has fallen far short of the target pose.

In order to solve this last problem, we need visual feedback to correct the errors and mistakes introduced for the gaps and inaccuracies in the kinematic chain. The visual inverse kinematics (`visualik`) component is responsible for solve this visual feedback and correct the inverse kinematic. The operation of the algorithm is very simple and takes as its starting point the investigations of Seth Hutchinson, Greg Hager and Peter Corke, collected in [A Tutorial on Visual Servo Control](http://www-cvr.ai.uiuc.edu/~seth/ResPages/pdfs/HutHagCor96.pdf)

Our component implements a simple state machine where waits the reception of a target position (a vector with traslations and rotations: `[tx, ty, tz, rx, ry, rz]`) through its interface. When a target is received, the `visualik` send it to the `inversekinematics` component like a POSE6D target, and waits for him in order to receive the final information of the target (when the `inversekinematics`component ends the target execution, it returns some information about this target: traslation and rotation errors, final angles, elapsed times...) and to place the arm in the first position gived by the `inversekinematics`component. On the end efffector it is an apriltag that send us information about the position and the rotation of the robot hand. As the end effector will be a little out of the target position (due to inaccuracies), the `visualik` will be prepared to correct this error:

1. It calculates the visual pose of the end effector (through `apriltags`, `visualik` receives the position of the end effector mark that the camera head sees).
2. After, it compute the error vector between the visual pose and the target pose.
3. With this error vector, `visualik` corrects the target pose and sends the new position to the `inversekinematics` component.
4. This process is repeated until the error achieved in translation and rotation is less than a predetermined threshold.

In this way we can correct the errors introduced by the inaccuracies of the joints.

##Configuration parameters
In the `visualik` configuration whe must put the next data:
1. We run the component in the port 10240 (because the `inversekinematic`component is executed in the port 10220)
2. We need to connect our component with the [`apriltags`](https://github.com/robocomp/robocomp-robolab/blob/master/components/apriltagsComp/README.md) component in order to receive the position of the apriltag.
3. We need to connect the `visualik` with the `inversekinematics` component in order to send the target and receive the final angles.
4. We need the controller of the joints, in order to move the kinematic chain.
5. And finally, we need the file .xml with the description of the robot.

So we need a config like this:

    InverseKinematics.Endpoints=tcp -p 10240
    CommonBehavior.Endpoints=tcp -p 10000
    
    AprilTagsTopic.Endpoints=tcp -p 12938
    InverseKinematicsProxy =inversekinematics:tcp -h localhost -p 10220
    JointMotorProxy = jointmotor:tcp -p 20000 
    OmniRobotProxy = omnirobot:tcp -p 12238
    
    InnerModel=robotfile.xml
    
    # This property is used by the clients to connect to IceStorm.
    TopicManager.Proxy=IceStorm/TopicManager:default -p 9999
    Ice.Warn.Connections=0
    Ice.Trace.Network=0
    Ice.Trace.Protocol=0
    Ice.ACM.Client=10
    Ice.ACM.Server=10
    
##Starting the component
To avoid changing the *config* file in the repository, we can copy it to the component's home directory, so changes will remain untouched by future git pulls:

    cd /robocomp/components/robocomp-ursus/components/visualik
    cp etc/config ourConfig
    
After editing the new config file (ourConfig) we can run the component:

    ./bin/visualik ourConfig
    
It's not necessary put the command `--Ice.Config=`.
