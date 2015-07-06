/*
 *    Copyright (C) 2015 by YOUR NAME HERE
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "specificworker.h"

#define STEP_DISTANCE 50
// #define CLOSE_DISTANCE (STEP_DISTANCE*2.5)
#define CLOSE_DISTANCE (STEP_DISTANCE*1.8)
/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx)
{
	state = GIK_NoTarget;

	show();
	initBox->show();
	commandBox->hide();
#ifdef USE_QTGUI
	innerViewer = NULL;
	osgView = new OsgView(widget);
	osgGA::TrackballManipulator *tb = new osgGA::TrackballManipulator;
	osg::Vec3d eye(osg::Vec3(4000.,4000.,-1000.));
	osg::Vec3d center(osg::Vec3(0.,0.,-0.));
	osg::Vec3d up(osg::Vec3(0.,1.,0.));
	tb->setHomePosition(eye, center, up, true);
	tb->setByMatrix(osg::Matrixf::lookAt(eye,center,up));
 	osgView->setCameraManipulator(tb);
#endif
	connect(&timer, SIGNAL(timeout()), this, SLOT(compute()));
	mutex = new QMutex(QMutex::Recursive);

}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{

}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	printf("params: %ld\n", params.size());
	for (auto p : params)
	{
		std::cout << p.first.c_str() << std::endl;
	}
	if (params.size() == 0)
		return true;

	RoboCompCommonBehavior::Parameter par;
	try
	{
		par = params.at("InnerModel");
	}
	catch(std::exception e)
	{
		qFatal("Error reading config param InnerModel (%s)", e.what());
	}
	if( QFile::exists(QString::fromStdString(par.value)) )
	{
#ifdef USE_QTGUI
		innerModel  = new InnerModel(par.value);
		innerVisual = new InnerModel(par.value);
		innerViewer = new InnerModelViewer(innerVisual, "root", osgView->getRootGroup(), true);
		osgView->getRootGroup()->addChild(innerViewer);
		show();
#endif
	}
	else
	{
		std::cout << "Innermodel path " << par.value << " not found. ";
		qFatal("Abort");
	}



	InnerModelDraw::addTransform_ignoreExisting(innerViewer, "init", "root");
	InnerModelDraw::addPlane_ignoreExisting(innerViewer, "init_p", "init", QVec::vec3(0,0,0), QVec::vec3(1,0,0), "#ff7777", QVec::vec3(15,15,15));

	InnerModelDraw::addTransform_ignoreExisting(innerViewer, "end", "root");
	InnerModelDraw::addPlane_ignoreExisting(innerViewer, "end_p", "end", QVec::vec3(0,0,0), QVec::vec3(1,0,0), "#77ff77", QVec::vec3(15,15,15));

	InnerModelDraw::addTransform_ignoreExisting(innerViewer, "target", "root");
	InnerModelDraw::addPlane_ignoreExisting(innerViewer, "target_p", "target", QVec::vec3(0,0,0), QVec::vec3(1,0,0), "#7777ff", QVec::vec3(15,15,15));

	connect(fromFileButton, SIGNAL(clicked()), this, SLOT(initFile()));
	connect(generateButton, SIGNAL(clicked()), this, SLOT(initGenerate()));

	timer.start(10);

	return true;
}


void SpecificWorker::initFile()
{
	initBox->hide();

	try
	{
		graph = new ConnectivityGraph("ursus.ikg");
	}
	catch(...)
	{
		printf("Can't create graph from file\n");
		return;
	}
	for (uint i=0; i<graph->vertices.size(); i++)
	{
		QString id = QString("node_") + QString::number(i);
		QString color;
		if (graph->vertices[i].configurations.size()>0)
			color = "#22cc22";
		else
			color = "#cc2222";

		InnerModelDraw::addPlane_ignoreExisting(innerViewer, id, "root", QVec::vec3(graph->vertices[i].pose[0], graph->vertices[i].pose[1], graph->vertices[i].pose[2]),
		    QVec::vec3(1,0,0), color, QVec::vec3(2,2,2));
	}
	for (uint i=0; i<graph->edges.size(); i++)
	{
		for (uint j=0; j<graph->edges[i].size(); j++)
		{
			if (graph->edges[i][j] < DJ_INFINITY)
			{
				QString id = QString("edge_") + QString::number(i) + QString("_") + QString::number(j);
// 				float *p1 = graph->vertices[i].pose;
// 				float *p2 = graph->vertices[j].pose;
// 				InnerModelDraw::drawLine2Points(innerViewer, id, "root", QVec::vec3(p1[0], p1[1], p1[2]), QVec::vec3(p2[0], p2[1], p2[2]), "#88ff88", 0.05);
			}
		}
	}
	commandBox->show();
	connect(goIKButton,  SIGNAL(clicked()), this, SLOT(goIK()));
	connect(goVIKButton, SIGNAL(clicked()), this, SLOT(goVIK()));
	connect(homeButton, SIGNAL(clicked()), this, SLOT(goHome()));

}


void SpecificWorker::initGenerate()
{
	initBox->hide();

	xrange = std::pair<float, float>( -110, 400);
	yrange = std::pair<float, float>( 580, 1200);
	zrange = std::pair<float, float>( 140, 570);

	QVec center = QVec::vec3((xrange.second+xrange.first)/2, (yrange.second+yrange.first)/2, (zrange.second+zrange.first)/2);

	float XR = abs(xrange.second - xrange.first);
	float YR = abs(yrange.second - yrange.first);
	float ZR = abs(zrange.second - zrange.first);
	float max = MAX(MAX(XR, YR), ZR);
	XR = XR/max;
	YR = YR/max;
	ZR = ZR/max;

// 	float step = 0.5001*sqrt((CLOSE_DISTANCE*CLOSE_DISTANCE)/3.);
	float step = STEP_DISTANCE;


	uint32_t included=0;
	graph = new ConnectivityGraph(0);
	for (float xpos = xrange.first; xpos<xrange.second; xpos+=step)
	{
		for (float ypos = yrange.first; ypos<yrange.second; ypos+=step)
		{
			for (float zpos = zrange.first; zpos<zrange.second; zpos+=step)
			{

				QVec pos = QVec::vec3(xpos, ypos, zpos);
				QVec diff = center-pos;
				diff(0) = abs(diff(0))/XR;
				diff(1) = abs(diff(1))/YR;
				diff(2) = abs(diff(2))/ZR;

				if (diff.norm2() < 320)
				{
					graph->addVertex(ConnectivityGraph::VertexData());
					QString id = QString("node_") + QString::number(included);
					graph->vertices[included].setPose(xpos, ypos, zpos);
					graph->vertices[included].configurations.clear();
					graph->vertices[included].id = included;

					{
						QMutexLocker l(mutex);
						InnerModelDraw::addPlane_ignoreExisting(innerViewer, id, "root",
							QVec::vec3(graph->vertices[included].pose[0],
							graph->vertices[included].pose[1], graph->vertices[included].pose[2]),
							QVec::vec3(1,0,0), "#666666", QVec::vec3(3.5,3.5,3.5)
						);
					}
					included++;
				}
			}
		}
	}

	printf("inc %d\n", included);

	int rec = 0;
	if (not goAndWait(180+(rand()%40), 780+(rand()%40), 300+(rand()%40), -1, centerConfiguration, rec))
		qFatal("Couldn't get initial position");

	workerThread = new WorkerThread(this);
	workerThread->start();

}

void SpecificWorker::updateFrame(uint wait_usecs)
{
	QMutexLocker l(mutex);
#ifdef USE_QTGUI
	if (innerViewer)
		innerViewer->update();
	osgView->autoResize();
	osgView->frame();
#endif
	usleep(wait_usecs);
}

void SpecificWorker::goAndWaitDirect(const MotorGoalPositionList &mpl)
{
	static MotorGoalPositionList last;

	if (mpl == last)
	{
		printf("skipping same config\n");
		return;
	}
	last = mpl;

	jointmotor_proxy->setSyncPosition(mpl);
	{
		QMutexLocker l(mutex);
		for (auto g : mpl)
		{
			innerVisual->updateJointValue(QString::fromStdString(g.name), g.position);
// 			printf("%s: %f\n", g.name.c_str(), g.position);
		}
	}
	usleep(20000);
}

bool SpecificWorker::goAndWait(int nodeId, MotorGoalPositionList &mpl, int &recursive)
{
	return goAndWait(graph->vertices[nodeId].pose[0], graph->vertices[nodeId].pose[1], graph->vertices[nodeId].pose[2], nodeId, mpl, recursive);
}

bool SpecificWorker::goAndWait(float x, float y, float z, int node, MotorGoalPositionList &mpl, int &recursive)
{
	RoboCompInverseKinematics::Pose6D target;
	target.x = x;
	target.y = y;
	target.z = z;

	float rely = (y - yrange.first) / (yrange.second - yrange.first);
	if (rely < 0.33)
		target.rx = 0.3;
	else if (rely < 0.66)
		target.rx = 0;
	else
		target.rx = -0.3;
	target.rx = 0; ////////////////////////////// WARNING

	float relx = (x - xrange.first) / (xrange.second - xrange.first);
	if (relx < 0.33)
		target.ry = -1.8;
	else if (relx < 0.66)
		target.ry = -1.47;
	else
		target.ry = -0.47;

	float relz = (z - zrange.first) / (zrange.second - zrange.first);
	if (relz < 0.33)
		target.ry += -0.8;
	else if (relz < 0.5)
		target.ry += -0.4;
	else
		target.ry += 0.3;

	if (target.ry < -1.57)
		target.ry = -1.57;

	target.rz = -3.14;

	//NOTE: CAMBIO DE MERCEDES 
	//RoboCompInverseKinematics::WeightVector weights;
	weights.x = weights.y = weights.z = 1;
	if (recursive==0) weights.rx = weights.ry = weights.rz = 0.1;
	else weights.rx = weights.ry = weights.rz = 0;

	int targetId = inversekinematics_proxy->setTargetPose6D("RIGHTARM", target, weights);

	TargetState stt;
	QTime initialTime = QTime::currentTime();
	do
	{
		usleep(10000);
		stt = inversekinematics_proxy->getTargetState("RIGHTARM", targetId);
		if (stt.finish == true)
			break;
	} while (initialTime.elapsed()<15000);

	{
		QMutexLocker l(mutex);
		innerVisual->updateTransformValues("target", target.x, target.y, target.z, target.rx, target.ry, target.rz);
	}

	mpl.resize(0);
	for (auto gp : stt.motors)
	{
		MotorGoalPosition mgp;
		mgp.position = gp.angle;
		mgp.maxSpeed = 2.;
		mgp.name = gp.name;
		mpl.push_back(mgp);
	}

	goAndWaitDirect(mpl);

	float err = innerVisual->transform("grabPositionHandR", "target").norm2();

	printf("ERROR segun IM %f\n", err);
	printf("ERROR segun IK %f\n", stt.errorT);
	printf("IK message %s\n", stt.state.c_str());

	if (stt.errorT > MAX_ERROR_IK or initialTime.elapsed()>15000 or not stt.finish)
	{
		printf("cant\'t go\n");

		if (recursive>0)
		{
			printf("recursive call\n");
			bool ret = false;
			printf("reseteando al mas parecido\n");
			float mustBeBiggerThan = -1;
			MotorGoalPositionList minConfig;
			for (int jj=0; jj<10; jj++)
			{
				if (jj == 3 and node != -1)
				{
					QVec xm = QVec::uniformVector(graph->size(), xrange.first, xrange.second);
					QVec ym = QVec::uniformVector(graph->size(), yrange.first, yrange.second);
					QVec zm = QVec::uniformVector(graph->size(), zrange.first, zrange.second);
					graph->vertices[node].pose[0] = xm(node);
					graph->vertices[node].pose[1] = ym(node);
					graph->vertices[node].pose[2] = zm(node);
				}
				float minDist = 99999999;
				int minIndex = -1;
				printf("%f %f\n", mustBeBiggerThan, minDist);
				printf("(iter:%d) graph size %d\n", jj, graph->size());
				for (int j=0; j<graph->size(); j++)
				{
					if (graph->vertices[j].configurations.size() > 0)
					{
						float p[3];
						p[0] = x;
						p[1] = y;
						p[2] = z;
						float dist = graph->vertices[j].distTo(p);
						if ((dist > mustBeBiggerThan and dist < minDist) or minDist == -1)
						{
							minDist = dist;
							minIndex = j;
						}
					}
				}
				if (minIndex != -1)
				{
					minConfig = graph->vertices[minIndex].configurations[0];
					mustBeBiggerThan = minDist;
					printf("patras con %d\n", minIndex);
					goAndWaitDirect(minConfig);
					printf("realizando la llamada recursiva\n");
					recursive++;
					ret = goAndWait(x, y, z, -1, mpl, recursive);
					printf("got %d\n", ret);
					if (ret)
						return true;
				}
				else
				{
					return false;
				}
			}
			return false;
		}
		else
		{
			return false;
		}
	}
	return true;
}

void SpecificWorker::computeHard()
{
	static bool stop=false;
	static int nodeSrc=-1;
	static int nodeDst=-1;
	static MotorGoalPositionList currentConfiguration;

	if (stop) return;

	printf("=======================================\n");

	if (nodeDst == graph->size() or nodeDst == -1)
	{
		nodeSrc++;
		QString id = QString("node_") + QString::number(graph->vertices[nodeSrc].id);
		if (nodeSrc == graph->size()-1)
		{
			commandBox->show();
			connect(goIKButton,  SIGNAL(clicked()), this, SLOT(goIK()));
			connect(goVIKButton, SIGNAL(clicked()), this, SLOT(goVIK()));
			connect(homeButton, SIGNAL(clicked()), this, SLOT(goHome()));
			graph->save("aqui.ikg");
			stop = true;
			return;
		}
		MotorGoalPositionList configuration;
		int de;
		int recursive = 0;
		for (de=0; de < 5 and not goAndWait(nodeSrc, configuration, recursive); de++)
		{
			int d=0;
			goAndWait(180+(rand()%40), 780+(rand()%40), 300+(rand()%40), -1, centerConfiguration, d);
		}
		if (de == 5)
		{
			qDebug() << "Couldn't reach target: skipping node" << nodeSrc;
			nodeDst = -1;
			InnerModelDraw::setPlaneTexture(innerViewer, id, "#ff0000");
			return;
		}

		InnerModelDraw::setPlaneTexture(innerViewer, id, "#00ff00");

		QVec elbowPose = innerVisual->transform("robot", "rightElbow");
		graph->vertices[nodeSrc].setElbowPose(elbowPose(0), elbowPose(1), elbowPose(2));
		graph->vertices[nodeSrc].configurations.push_back(configuration);
		currentConfiguration = configuration;
		nodeDst=0;

		return;
	}

	if (graph->vertices[nodeSrc].configurations[0] != currentConfiguration)
	{
		currentConfiguration = graph->vertices[nodeSrc].configurations[0];
	}
	goAndWaitDirect(currentConfiguration);

	printf("nodeSrc: %d\n", nodeSrc);
	printf("nodeDst: %d\n", nodeDst);



	float dist = graph->vertices[nodeSrc].distTo(graph->vertices[nodeDst].pose);
	if (dist>0 and dist<CLOSE_DISTANCE and graph->edges[nodeSrc][nodeDst] == DJ_INFINITY)
	{
		printf("***  nodeSrc: %d\n", nodeSrc);
		printf("***  nodeDst: %d\n", nodeDst);
		MotorGoalPositionList configuration;
		int rec=0;
		if (goAndWait(nodeDst, configuration, rec))
		{
			printf("goAndWait done\n");
			printf("add %d %d %f\n", nodeSrc, nodeDst, dist);
			graph->add_edge(nodeSrc, nodeDst, dist);
			graph->add_configurationToNode(nodeDst, configuration);

			{
				static int edId = 0;
				const QString id = QString("edge_") + QString::number(edId++);
				float *p1 = graph->vertices[nodeSrc].pose;
				float *p2 = graph->vertices[nodeDst].pose;
				QMutexLocker l(mutex);
				InnerModelDraw::drawLine2Points(innerViewer, id, "root", QVec::vec3(p1[0], p1[1], p1[2]), QVec::vec3(p2[0], p2[1], p2[2]), "#88ff88", 1);
			}
		}
		else
		{
			qDebug() << "Can't move to last target...";
// 			stop = true;
		}
	}

	nodeDst++;

}


void SpecificWorker::compute()
{
	updateFrame(10);

	updateInnerModel();

	static int tick = 0;
	
	if (tick++ % 10 != 0) return;

	static uint32_t pathIndex = 0;

	switch(state)
	{
		case GIK_NoTarget:
			return;
		case GIK_GoToInit:
			goAndWaitDirect(graph->vertices[closestToInit].configurations[0]);
			pathIndex = 0;
			state = GIK_GoToEnd;
			updateFrame(500000);
			break;
		case GIK_GoToEnd:
			goAndWaitDirect(graph->vertices[path[pathIndex]].configurations[0]);
			printf("%d %f %f %f\n", pathIndex, graph->vertices[path[pathIndex]].pose[0], graph->vertices[path[pathIndex]].pose[1], graph->vertices[path[pathIndex]].pose[2]);
			pathIndex++;
			updateFrame(500000);
			if (pathIndex>=path.size())
			{
				pathIndex = 0;
				state = GIK_GoToActualTargetSend;
			}
			break;
		case GIK_GoToActualTargetSend:
			updateFrame(500000);
			state = GIK_GoToActualTargetSent;
			
			//NOTE: CAMBIOS DE MERCEDES
			weights.x = weights.y = weights.z = weights.rx = weights.ry = weights.rz = 1;
			targetId = inversekinematics_proxy->setTargetPose6D("RIGHTARM", finalTarget, weights);
			
			printf("FINAL TARGET %f %f %f - %f %f %f\n", finalTarget.x, finalTarget.y, finalTarget.z, finalTarget.rx, finalTarget.ry, finalTarget.rz);
			printf("WITH WEIGHTS %f %f %f - %f %f %f\n", weights.x, weights.y, weights.z, weights.rx, weights.ry, weights.rz);
			break;
		case GIK_GoToActualTargetSent:
			updateFrame(500000);
			TargetState stt = inversekinematics_proxy->getTargetState("RIGHTARM", targetId);
			if (stt.finish == true)
			{
				printf("IK message %s\n", stt.state.c_str());
				//NOTE:CAMBIOS DE MERCEDES he cambiado err por stt.errorT
				//float err = innerVisual->transform("grabPositionHandR", "target").norm2();
				if (stt.errorT > MAX_ERROR_IK)
				{
					QMessageBox::information(this, "finished ERR", QString("can't go: error=")+QString::number(stt.errorT)+QString("\n")+QString::fromStdString(stt.state));
				}
				else
				{
					MotorGoalPositionList mpl;
					for (auto gp : stt.motors)
					{
						MotorGoalPosition mgp;
						mgp.position = gp.angle;
						mgp.maxSpeed = 0.8;
						mgp.name = gp.name;
						mpl.push_back(mgp);
					}
					goAndWaitDirect(mpl);
					updateFrame(500000);
					QMessageBox::information(this, "finished OK", QString("target reached: error=")+QString::number(stt.errorT)+QString("\n")+QString::fromStdString(stt.state));
				}
				state = GIK_NoTarget;
			}
			break;
	}
}

void WorkerThread::run()
{
	while(true)
	{
		((SpecificWorker*)data)->computeHard();
		usleep(10000);
	}
}

void SpecificWorker::updateInnerModel()
{
	try
	{
		RoboCompJointMotor::MotorStateMap mMap;
		jointmotor_proxy->getAllMotorState(mMap);

		for (auto j : mMap)
		{
			innerModel->updateJointValue(QString::fromStdString(j.first), j.second.pos);
			innerVisual->updateJointValue(QString::fromStdString(j.first), j.second.pos);
		}
	}
	catch (const Ice::Exception &ex)
	{
		cout<<"--> Exception updating InnerModel\n";
	}
}

void SpecificWorker::goIK()
{
	/// Get target and update it in IMV
	float vtx = tx->value();
	float vty = ty->value();
	float vtz = tz->value();
	float vrx = rx->value();
	float vry = ry->value();
	float vrz = rz->value();

	finalTarget.x = vtx;
	finalTarget.y = vty;
	finalTarget.z = vtz;
	finalTarget.rx = vrx;
	finalTarget.ry = vry;
	finalTarget.rz = vrz;
	
	innerVisual->updateTransformValues("target", vtx, vty, vtz, vrx, vry, vrz);

	// Get closest node to initial position and update it in IMV
	updateInnerModel();
	QVec position = innerModel->transform("robot", "grabPositionHandR");
	closestToInit = graph->getCloserTo(&position(0));
	const float *poseInit = graph->vertices[closestToInit].pose;
	innerVisual->updateTransformValues("init", poseInit[0], poseInit[1], poseInit[2], 0,0,0);

	// Get closest node to target and update it in IMV
	closestToEnd = graph->getCloserTo(vtx, vty, vtz);
	const float *poseEnd = graph->vertices[closestToEnd].pose;
	innerVisual->updateTransformValues("end", poseEnd[0], poseEnd[1], poseEnd[2], 0,0,0);

	// Compute path and update state
	Dijkstra d = Dijkstra(&(graph->edges));
	d.calculateDistance(closestToInit);
	path.clear();
	d.go(closestToEnd, path);
	state = GIK_GoToInit;
}


void SpecificWorker::goVIK()
{
}

void SpecificWorker::goHome()
{
	printf("going home\n");


	MotorGoalPositionList listGoals;
	listGoals.resize(7);
	listGoals[0].name     = "rightShoulder1";
	listGoals[0].position = -2.7;
	listGoals[1].name     = "rightShoulder2";
	listGoals[1].position = -0.2;
	listGoals[2].name     = "rightShoulder3";
	listGoals[2].position = 1.5;
	listGoals[3].name     = "rightElbow";
	listGoals[3].position = 0.4;
	listGoals[4].name   = "rightForeArm";
	listGoals[4].position = -1.;
	listGoals[5].name = "rightWrist1";
	listGoals[5].position = 0.;
	listGoals[6].name = "rightWrist2";
	listGoals[6].position = 0.;

	for (int i=0; i<7; i++)
	{
		listGoals[i].maxSpeed = 1.;
	}

	jointmotor_proxy->setSyncPosition(listGoals);
}













