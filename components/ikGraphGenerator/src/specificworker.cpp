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

#define MAX_SPEED 0.7

#define STEP_DISTANCE 50
// #define CLOSE_DISTANCE (STEP_DISTANCE*2.5)
#define CLOSE_DISTANCE (STEP_DISTANCE*1.8)
/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////
/** ------------------------------------------------------
* \brief Default constructor
* @param mprx 
*------------------------------------------------------*/
SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx)
{
	READY         = false;
	state         = GIK_NoTarget;
	targetCounter = 0;
	mutexSolved   = new QMutex(QMutex::Recursive);

#ifdef USE_QTGUI
	show();
	initBox->show();
	ikCommandWidget->hide();
	innerViewer   = NULL;
	osgView       = new OsgView(widget);
	osgGA::TrackballManipulator *tb = new osgGA::TrackballManipulator;
	osg::Vec3d eye(osg::Vec3(4000.,4000.,-1000.));
	osg::Vec3d center(osg::Vec3(0.,0.,-0.));
	osg::Vec3d up(osg::Vec3(0.,1.,0.));
	tb->setHomePosition(eye, center, up, true);
	tb->setByMatrix(osg::Matrixf::lookAt(eye,center,up));
 	osgView->setCameraManipulator(tb);
#endif
	connect(&timer, SIGNAL(timeout()), this, SLOT(compute()));
	
	timer.start(10);
}



/**------------------------------------------------------
* \brief Default destructor
* ------------------------------------------------------*/
SpecificWorker::~SpecificWorker()
{

}



/** ------------------------------------------------------
 * \brief setParams
 * @param params
 * ------------------------------------------------------ */ 
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
		innerModel  = new InnerModel(par.value);
#ifdef USE_QTGUI
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
#ifdef USE_QTGUI
        //posicion inicial
	InnerModelDraw::addTransform_ignoreExisting(innerViewer, "init", "root");
	InnerModelDraw::addPlane_ignoreExisting(innerViewer, "init_p", "init", QVec::vec3(0,0,0), QVec::vec3(1,0,0), "#ff7777", QVec::vec3(15,15,15));

        //final 
	InnerModelDraw::addTransform_ignoreExisting(innerViewer, "end", "root");
	InnerModelDraw::addPlane_ignoreExisting(innerViewer, "end_p", "end", QVec::vec3(0,0,0), QVec::vec3(1,0,0), "#77ff77", QVec::vec3(15,15,15));

        //target
	InnerModelDraw::addTransform_ignoreExisting(innerViewer, "target", "root");
	InnerModelDraw::addPlane_ignoreExisting(innerViewer, "target_p", "target", QVec::vec3(0,0,0), QVec::vec3(1,0,0), "#7777ff", QVec::vec3(15,15,15));

	connect(fromFileButton, SIGNAL(clicked()), this, SLOT(initFile()));
	connect(generateButton, SIGNAL(clicked()), this, SLOT(initGenerate()));
#endif


	try
	{
		InnerModelNode *parent = innerModel->getNode("root");
		if (innerModel->getNode("target") == NULL)
		{
			InnerModelTransform *tr;
			try
			{
				tr = innerModel->newTransform("target", "static", parent, 0,0,0, 0,0,0);
				parent->addChild(tr);
			}
			catch (QString err)
			{
				printf("%s:%s:%d: Exception: %s\n", __FILE__, __FUNCTION__, __LINE__, err.toStdString().c_str());
				throw;
			}
		}
	}
	catch (QString err)
	{
		printf("%s:%s:%d: Exception: %s\n", __FILE__, __FUNCTION__, __LINE__, err.toStdString().c_str());
		throw;
	}


	/*timer.start(10);*/	
 	initFile();
	qDebug()<<"READY CONFIG PARAMS";
	return true;
}

/** ------------------------------------------------------
 * \brief initFile
 * ------------------------------------------------------*/
void SpecificWorker::initFile()
{
	printf("%s: %d\n", __FILE__, __LINE__);
#ifdef USE_QTGUI
	initBox->hide();
#endif
	try
	{
		//graph = new ConnectivityGraph("ursus.ikg");
		//graph = new ConnectivityGraph("/home/robocomp/robocomp/components/robocomp-ursus/components/ikGraphGenerator/ursus.ikg");
		graph = new ConnectivityGraph("/home/robocomp/robocomp/components/robocomp-ursus/components/ikGraphGenerator/ursusRt.ikg");
		printf("Read graph: size=%d\n", graph->size());
	}
	catch(...)
	{
		qFatal("Can't create graph from file\n");
	}
#ifdef USE_QTGUI
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
//				QString id = QString("edge_") + QString::number(i) + QString("_") + QString::number(j);
// 				float *p1 = graph->vertices[i].pose;
// 				float *p2 = graph->vertices[j].pose;
// 				InnerModelDraw::drawLine2Points(innerViewer, id, "root", QVec::vec3(p1[0], p1[1], p1[2]), QVec::vec3(p2[0], p2[1], p2[2]), "#88ff88", 0.05);
			}
		}
	}
	ikCommandWidget->show();
	connect(goIKButton, SIGNAL(clicked()), this, SLOT(goIK()));
	connect(homeButton, SIGNAL(clicked()), this, SLOT(goHome()));
#endif
	READY = true;
}



/** ------------------------------------------------------
 * \brief initGenerate
 * ------------------------------------------------------*/
void SpecificWorker::initGenerate()
{
	printf("%s: %d\n", __FILE__, __LINE__);
#ifdef USE_QTGUI
	initBox->hide();
#endif

	xrange = std::pair<float, float>( 50, 400);
	yrange = std::pair<float, float>( 750, 1100);
	zrange = std::pair<float, float>( 180, 450);
	QVec center = QVec::vec3((xrange.second+xrange.first)/2, (yrange.second+yrange.first)/2, (zrange.second+zrange.first)/2);

// 	float XR = abs(xrange.second - xrange.first);
// 	float YR = abs(yrange.second - yrange.first);
// 	float ZR = abs(zrange.second - zrange.first);
// 	float max = MAX(MAX(XR, YR), ZR);
// 	XR = XR/max;
// 	YR = YR/max;
// 	ZR = ZR/max;

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
// 				QVec diff = center-pos;
// 				diff(0) = abs(diff(0))/XR;
// 				diff(1) = abs(diff(1))/YR;
// 				diff(2) = abs(diff(2))/ZR;

// 				if (diff.norm2() < 400)
				{
					graph->addVertex(ConnectivityGraph::VertexData());
					QString id = QString("node_") + QString::number(included);
					graph->vertices[included].setPose(xpos, ypos, zpos);
					graph->vertices[included].configurations.clear();
					graph->vertices[included].id = included;
#ifdef USE_QTGUI
					{
						QMutexLocker l(mutex);
						InnerModelDraw::addPlane_ignoreExisting(innerViewer, id, "root",
							QVec::vec3(graph->vertices[included].pose[0],
							graph->vertices[included].pose[1], graph->vertices[included].pose[2]),
							QVec::vec3(1,0,0), "#666666", QVec::vec3(3.5,3.5,3.5)
						);
					}
#endif
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
	READY = true;
}



/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////
/**------------------------------------------------------
 * \brief updateFrame
 * @param wait_usecs
 * ------------------------------------------------------*/
void SpecificWorker::updateFrame(uint wait_usecs)
{
#ifdef USE_QTGUI
	QMutexLocker l(mutex);
	if (innerViewer)
		innerViewer->update();
	osgView->autoResize();
	osgView->frame();
	usleep(wait_usecs);
#endif
}



/** ------------------------------------------------------
 * \brief updateInnerModel
 * ------------------------------------------------------*/ 
void SpecificWorker::updateInnerModel()
{
	try
	{
		RoboCompJointMotor::MotorStateMap mMap;
		jointmotor_proxy->getAllMotorState(mMap);

		for (auto j : mMap)
		{
			innerModel->updateJointValue(QString::fromStdString(j.first), j.second.pos);
#ifdef USE_QTGUI
			innerVisual->updateJointValue(QString::fromStdString(j.first), j.second.pos);
#endif
		}
	}
	catch (const Ice::Exception &ex)
	{
		cout<<"--> Exception updating InnerModel\n";
	}
}


/** -------------------------------------------------------------------
 * \brief goAndWaitDirect. Mueve el brazo en base a la posicion de los
 * motores almacenada en mpl.
 * @param mpl motores y posiciones de los motores.
 * ------------------------------------------------------*/
void SpecificWorker::goAndWaitDirect(const MotorGoalPositionList &mpl)
{
	printf("%s: %d\n", __FILE__, __LINE__);
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
#ifdef USE_QTGUI
			innerVisual->updateJointValue(QString::fromStdString(g.name), g.position);
#endif
		}
	}
	usleep(20000);
	//sleep(1);
}


/** ------------------------------------------------------
 * \brief goAndWait
 * @param mpl
 * @param recursive
 * @return bool
 * ------------------------------------------------------*/
bool SpecificWorker::goAndWait(int nodeId, MotorGoalPositionList &mpl, int &recursive)
{
	printf("%s: %d\n", __FILE__, __LINE__);
	return goAndWait(graph->vertices[nodeId].pose[0], graph->vertices[nodeId].pose[1], graph->vertices[nodeId].pose[2], nodeId, mpl, recursive);
}


/** ------------------------------------------------------
 * \brief goAndWait
 * @param x
 * @param y
 * @param z
 * @param node
 * @param mpl
 * @param recursive
 * @return bool
 * ------------------------------------------------------*/
bool SpecificWorker::goAndWait(float x, float y, float z, int node, MotorGoalPositionList &mpl, int &recursive)
{
	printf("%s: %d\n", __FILE__, __LINE__);
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
	target.rx = 0;

// 	float relx = (x - xrange.first) / (xrange.second - xrange.first);
// 	if (relx < 0.33)
// 		target.ry = -1.8;
// 	else if (relx < 0.66)
// 		target.ry = -1.47;
// 	else
// 		target.ry = -0.47;


	if (x < 50)
		target.ry = -1.5;
	else if (x > 400)
		target.ry = 0;
	else
		target.ry = -0.785398163397448;
	

// 	float relz = (z - zrange.first) / (zrange.second - zrange.first);
// 	if (relz < 0.33)
// 		target.ry += -0.8;
// 	else if (relz < 0.5)
// 		target.ry += -0.4;
// 	else
// 		target.ry += 0.3;
// 
// 	if (target.ry < -1.57)
// 		target.ry = -1.57;

	target.rz = 0;

	WeightVector weights;
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

#ifdef USE_QTGUI
	{
		QMutexLocker l(mutex);
		innerVisual->updateTransformValues("target", target.x, target.y, target.z, target.rx, target.ry, target.rz);
	}
#endif

	mpl.resize(0);
	for (auto gp : stt.motors)
	{
		MotorGoalPosition mgp;
		mgp.position = gp.angle;
		mgp.maxSpeed = MAX_SPEED;
		mgp.name = gp.name;
		mpl.push_back(mgp);
	}

	goAndWaitDirect(mpl);

	printf("ERROR segun IK %f\n", stt.errorT);
	printf("IK message %s\n", stt.state.c_str());

	if (stt.errorT > MAX_ERROR_IK or initialTime.elapsed()>15000 or not stt.finish)
	{
		printf("cant\'t go\n");

		if (recursive>0)
		{
			printf("recursive call\n");
			bool ret = false;
			printf("reset to the closest node\n");
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





/** ------------------------------------------------------
 * \brief computeHard
 * ------------------------------------------------------*/
void SpecificWorker::computeHard()
{
	printf("%s: %d\n", __FILE__, __LINE__);
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
#ifdef USE_QTGUI
			ikCommandWidget->show();
#endif
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
#ifdef USE_QTGUI
			InnerModelDraw::setPlaneTexture(innerViewer, id, "#ff0000");
#endif
			return;
		}
#ifdef USE_QTGUI
		InnerModelDraw::setPlaneTexture(innerViewer, id, "#00ff00");
#endif
		QVec elbowPose = innerModel->transform("robot", "rightElbow");
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
#ifdef USE_QTGUI
			{
				static int edId = 0;
				const QString id = QString("edge_") + QString::number(edId++);
				float *p1 = graph->vertices[nodeSrc].pose;
				float *p2 = graph->vertices[nodeDst].pose;
				QMutexLocker l(mutex);
				InnerModelDraw::drawLine2Points(innerViewer, id, "root", QVec::vec3(p1[0], p1[1], p1[2]), QVec::vec3(p2[0], p2[1], p2[2]), "#88ff88", 1);
			}
#endif
		}
		else
		{
			qDebug() << "Can't move to last target...";
// 			stop = true;
		}
	}
	nodeDst++;
}



/** ------------------------------------------------------
 * \brief compute
 * this method executes each 10 iterations
 * ------------------------------------------------------*/
void SpecificWorker::compute()
{
	if (not READY) return;
	static int tick = 0;
	if (tick++ % 10 != 0) return;

	QMutexLocker l(mutex);
	updateFrame(10);
	updateInnerModel();

	static uint32_t pathIndex = 0;

	switch(state)
	{
	case GIK_NoTarget:
		printf(".");
		fflush(stdout);
		break;
	case GIK_GoToInit:
		printf("GIK_GoToInit\n");
		break;
	case GIK_GoToEnd:
		printf("GIK_GoToEnd\n");
		break;
	case GIK_GoToActualTargetSend:
// 		printf("GIK_GoToActualTargetSend\n");
		break;
	case GIK_GoToActualTargetSent:
// 		printf("GIK_GoToActualTargetSent\n");
		break;
	default:
		printf("%s: %d\n", __FILE__, __LINE__);
	}
	
	
	switch(state)
	{
	case GIK_NoTarget:
		return;
	//--------------------------------------------------------------------------------------------------//
	case GIK_GoToInit :
		goAndWaitDirect(graph->vertices[closestToInit].configurations[0]);
		pathIndex = 0;
		if (path.size() > 1) pathIndex = 1;
		state = GIK_GoToEnd;
	break;
	//--------------------------------------------------------------------------------------------------//
	case GIK_GoToEnd:
		goAndWaitDirect(graph->vertices[path[pathIndex]].configurations[0]);
		printf("%d %f %f %f\n", pathIndex, graph->vertices[path[pathIndex]].pose[0], graph->vertices[path[pathIndex]].pose[1], graph->vertices[path[pathIndex]].pose[2]);
		pathIndex++;
		if (pathIndex>=path.size())
		{
			pathIndex = 0;
			state = GIK_GoToActualTargetSend;
			waitForMotorsToStop();
			qDebug()<<"\n\nPASAMOS A LA IK!!!!!!!!!\n";
			sleep(2);
		}
	break;
	//--------------------------------------------------------------------------------------------------//
	case GIK_GoToActualTargetSend:
	
		qDebug()<<"SEND---->("<<currentTarget.pose.x<<", "<<currentTarget.pose.y<<", "<<currentTarget.pose.z<<")";
		try {
			currentTarget.id_IK = inversekinematics_proxy->setTargetPose6D("RIGHTARM", currentTarget.pose, currentTarget.weights);
			state = GIK_GoToActualTargetSent;
		}
		catch (Ice::Exception e){ qDebug()<<"cannot connect with inversekinematics_proxy"<<e.what();}
	break;
	//--------------------------------------------------------------------------------------------------//
	case GIK_GoToActualTargetSent:
		TargetState stt = inversekinematics_proxy->getTargetState("RIGHTARM", currentTarget.id_IK);
		if (stt.finish == true)
		{
			qDebug()<<"HE TERMINADO!!: "<<currentTarget.id_IK<<"..."<<currentTarget.id_IKG;
			QMutexLocker mm(mutexSolved);
			currentTarget.state = stt;
			if (stt.errorT > MAX_ERROR_IK)
			{
				lastFinish = "ERROR";
// #ifdef USE_QTGUI
// 		QMessageBox::information(this, "finished ERR", QString("can't go: error=")+QString::number(stt.errorT)+QString("\n")+QString::fromStdString(stt.state));
// #endif
			}
			else
			{
				MotorGoalPositionList mpl;
				for (auto gp : stt.motors)
				{
					MotorGoalPosition mgp;
					mgp.position = gp.angle;
					mgp.maxSpeed = MAX_SPEED;
					mgp.name = gp.name;
					mpl.push_back(mgp);
				}
				goAndWaitDirect(mpl);
// 				qFatal("hecho!");
				waitForMotorsToStop();
				lastMotorGoalPositionList = mpl;
				lastFinish = "OK";
// #ifdef USE_QTGUI
// 						updateFrame(500000);
// 						QMessageBox::information(this, "finished OK", QString("target reached: error=")+QString::number(stt.errorT)+QString("\n")+QString::fromStdString(stt.state));
// #endif
				usleep(500000);
			}
			qDebug()<<"finish: "<<QString::fromStdString(lastFinish);
			updateInnerModel();
			solvedList.enqueue(currentTarget); //guardamos el target
			printf("(%f %f %f)\n", currentTarget.pose.x, currentTarget.pose.y, currentTarget.pose.z);
			qDebug()<<"ERROR T: "<<currentTarget.state.errorT;
			state = GIK_NoTarget;
		}
		break;
	}
}
/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////
void WorkerThread::run()
{
	printf("%s: %d\n", __FILE__, __LINE__);
	while(true)
	{
		((SpecificWorker*)data)->computeHard();
		usleep(10000);
	}
}
/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////
/** --------------------------------------------------
 * \brief goIK
 * --------------------------------------------------- */ 
void SpecificWorker::goIK()
{
	printf("%s: %d\n", __FILE__, __LINE__);
#ifdef USE_QTGUI
	/// Get target and update it in IMV
	float vtx = tx->value();
	float vty = ty->value();
	float vtz = tz->value();
	float vrx = rx->value();
	float vry = ry->value();
	float vrz = rz->value();

	currentTarget.pose.x = vtx;
	currentTarget.pose.y = vty;
	currentTarget.pose.z = vtz;
	currentTarget.pose.rx = vrx;
	currentTarget.pose.ry = vry;
	currentTarget.pose.rz = vrz;

	currentTarget.weights.x  = currentTarget. weights.y = currentTarget.weights.z  = 1;
	currentTarget.weights.rx = currentTarget.weights.ry = currentTarget.weights.rz = 0.1;

	setTargetPose6D("RIGHTARM", currentTarget.pose, currentTarget.weights);
#endif
}



/**-----------------------------------------------
 * \brief goHome
 * ----------------------------------------------*/
void SpecificWorker::goHome()
{
	printf("%s: %d\n", __FILE__, __LINE__);
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
		listGoals[i].maxSpeed = MAX_SPEED;
	}

	jointmotor_proxy->setSyncPosition(listGoals);
// 	printf("%s: %d\n", __FILE__, __LINE__);
// 	qFatal("home?\n");
	usleep(20000);
}


/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////
/**-------------------------------------------
 * \brief setTargetPose6D Metodo de la interfaz de IK, que guarda el target en la variable
 * currentTarget. TODO Por ahora admitimos que el visualIK se espera hasta que el currentTarget
 * termine de ejecutarse, pero hay que ampliar, usando una lista de targets pendientes.
 * @param bodyPart name of the body part.
 * @param target   pose of the goal point.
 * @param weights  weights of each traslation and rotation components.
 * @return the identifier of the target (an int)
 * ------------------------------------------*/ 
int SpecificWorker::setTargetPose6D(const string &bodyPart, const Pose6D &target, const WeightVector &weights)
{
	printf("%s: %d\n", __FILE__, __LINE__);
	QMutexLocker l(mutex);
	currentTarget.part    = QString::fromStdString(bodyPart);
	currentTarget.id_IKG  = targetCounter;
	currentTarget.pose    = target;
	currentTarget.weights = weights;

#ifdef USE_QTGUI
	innerVisual->updateTransformValues("target", target.x, target.y, target.z, target.rx, target.ry, target.rz);
#endif
	updateInnerModel();
		
	// Si la distancia entre el target y la mano es poca, pasamos del grafo:
	innerModel->updateTransformValues("target", target.x, target.y, target.z, target.rx, target.ry, target.rz);
	float distancia = innerModel->transform("target", "grabPositionHandR").norm2();
 	printf("ERROR AL TARGET: %f\n", distancia);
	if (distancia<100)
	{
 		qDebug()<<"DIRECTO";
 		qDebug()<<"DIRECTO";
 		qDebug()<<"DIRECTO";
 		qDebug()<<"DIRECTO";
 		qDebug()<<"DIRECTO";
#ifdef USE_QTGUI
		innerVisual->updateTransformValues("init", target.x, target.y, target.z, 0,0,0);
		innerVisual->updateTransformValues("end", target.x, target.y, target.z, 0,0,0);
#endif
		state = GIK_GoToActualTargetSend;
	}
	else
	{	
		// Get closest node to initial position and update it in IMV
		QVec position = innerModel->transform("robot", "grabPositionHandR");
		closestToInit = graph->getCloserTo(position(0), position(1), position(2));
#ifdef USE_QTGUI
		const float *poseInit = graph->vertices[closestToInit].pose;
		innerVisual->updateTransformValues("init", poseInit[0], poseInit[1], poseInit[2], 0,0,0);
#endif

		// Get closest node to target and update it in IMV
		closestToEnd = graph->getCloserTo(target.x, target.y, target.z);
#ifdef USE_QTGUI
		const float *poseEnd = graph->vertices[closestToEnd].pose;
		innerVisual->updateTransformValues("end", poseEnd[0], poseEnd[1], poseEnd[2], 0,0,0);
#endif
		
		// Compute path and update state
		Dijkstra d = Dijkstra(&(graph->edges));
		d.calculateDistance(closestToInit);
		std::vector<int> tempPath;
		d.go(closestToEnd, tempPath);
		
 		path.clear();

		if (tempPath.size() > 0)
		{
			printf("path A: ");
			for (uint i=0; i<tempPath.size(); i++) printf("%d", tempPath[i]);
			printf("\n");

			// Should we skip the first node?
			int first = 0;
			QVec firstNodePosition = QVec::vec3(graph->vertices[first].pose[0], graph->vertices[first].pose[1], graph->vertices[first].pose[2]);
			float distF = innerModel->transform("grabPositionHandR", firstNodePosition, "root").norm2();
			if (distF < 75)
			{
				printf("skipping first node!\n");
				first += 1;
			}
			// Should we skip the last node?
			int last = tempPath.size()>0?tempPath.size()-1:0;
			QVec lastNodePosition = QVec::vec3(graph->vertices[last].pose[0], graph->vertices[last].pose[1], graph->vertices[last].pose[2]);
			float distL = innerModel->transform("target", lastNodePosition, "root").norm2();
			if (last > first and last > 0 and distL < 75)
			{
				printf("skipping last node!\n");
				last -= 1;
			}
			// Generate resulting path vector
			for (int i = first; i<=last; i++)
			{
				path.push_back(tempPath[i]);
			}
			printf("path B: ");
			for (uint i=0; i<path.size(); i++) printf("%d", path[i]);
			printf("\n");
		}

		state = GIK_GoToInit;
	}
	targetCounter++;

	return currentTarget.id_IKG;
}


/**
 * \brief este metodo esta pensado para mover la cabeza del robot, principalmente.
 * Se queda esperando hasta que la IK devuelve el estado del target. Si el estado es TRUE
 * mueve la parte del robot hasta el target.
 * @param bodyPart  name of the body part.
 * @param ax the direction
 * @param dist step to advance un milimeters
 * @return the identifier of the target (an int)
 */ 
int SpecificWorker::setTargetAlignaxis(const string &bodyPart, const Pose6D &target, const Axis &ax)
{
	printf("%s: %d\n", __FILE__, __LINE__);
	int id = inversekinematics_proxy->setTargetAlignaxis(bodyPart, target, ax);
	while (inversekinematics_proxy->getTargetState(bodyPart, id).finish == false);
	
	for(auto motor : inversekinematics_proxy->getTargetState(bodyPart, id).motors)
	{
		try
		{
			RoboCompJointMotor::MotorGoalPosition nodo;
			nodo.name = motor.name;
			nodo.position = motor.angle; // position in radians
			nodo.maxSpeed = MAX_SPEED;   // rads per second
			jointmotor_proxy->setPosition(nodo);
		} catch (const Ice::Exception &ex) {
			cout<<"EXCEPTION IN UPDATE MOTORS [TARGET ALING AXIS]: "<<ex<<endl;
		}
	}
	return id;
}


/**
 * \brief este metodo  mueve directamente la parte del robot sobre el eje indicado una distancia determinada.
 * @param bodyPart  name of the body part.
 * @param ax the direction
 * @param dist step to advance un milimeters
 * @return the identifier of the target (an int)
 */ 
int SpecificWorker::setTargetAdvanceAxis(const string &bodyPart, const Axis &ax, const float dist)
{
	printf("%s: %d\n", __FILE__, __LINE__);

	int id = inversekinematics_proxy->setTargetAdvanceAxis(bodyPart, ax, dist);
	
	if (id==-1)
	{
		qDebug()<<"Error from IK: "<<id;
		return id;
	}
	
	while(inversekinematics_proxy->getTargetState(bodyPart, id).finish == false);
	
	for(auto motor : inversekinematics_proxy->getTargetState(bodyPart, id).motors)
	{
		try
		{
			RoboCompJointMotor::MotorGoalPosition nodo;
			nodo.name = motor.name;
			nodo.position = motor.angle;
			nodo.maxSpeed = MAX_SPEED;
			jointmotor_proxy->setPosition(nodo);
		} catch (const Ice::Exception &ex) {
			cout<<"EXCEPTION IN UPDATE MOTORS [TARGET ADVANCE AXIS]: "<<ex<<endl;
		}
	}
	return id;
}



/**
 * TODO
 */ 
void SpecificWorker::setFingers(const float d)
{
	printf("%s: %d\n", __FILE__, __LINE__);
	try
	{
		inversekinematics_proxy->setFingers(d);
	}
	catch (Ice::Exception e)
	{
		qDebug()<<"SpecificWorker::setFingers cannot connect with inversekinematics_proxy"<<e.what();
	}
}



/**
 * \brief Este metodo mira si el target de identificador senialado esta dentro de la lista de
 * targets resueltos por el ikGraph. Si esta devuelve true en el campo finish del target.
 * Si no esta, devuelve false.
 * @param bodypart part of the robot that resolves the target
 * @param targetID target identifier
 * @return TargetState
 */ 
TargetState SpecificWorker::getTargetState(const string &bodyPart, const int targetID)
{
	TargetState s;
	s.finish = false;
	
	QMutexLocker mm(mutexSolved);
	for(int i=0; i<solvedList.size(); i++)
	{
		if (targetID == solvedList[i].id_IKG)
		{
			qDebug()<<"PEDIDO TARGET "<<targetID<<". ENCONTRADO.";
			return solvedList[i].state;
		}
	}
	return s;
}



/**
 * \brief this method moves the motors to their home value
 * @param bodyPart the part of the robot that we want to move to the home
 */
void SpecificWorker::goHome(const string &bodyPart)
{
	printf("%s: %d\n", __FILE__, __LINE__);
	goHome();
}


void SpecificWorker::stop(const string &bodyPart)
{
	printf("%s: %d\n", __FILE__, __LINE__);
	state = GIK_NoTarget;
}



bool SpecificWorker::getPartState(const string &bodyPart)
{
	printf("%s: %d\n", __FILE__, __LINE__);
	if (state == GIK_NoTarget)
		return true;
	return false;
}



/**
 * TODO
 */ 
void SpecificWorker::setJoint(const string &joint, const float angle, const float maxSpeed)
{
	printf("%s: %d\n", __FILE__, __LINE__);
	try
	{
		inversekinematics_proxy->setJoint(joint, angle, maxSpeed);
	}
	catch (Ice::Exception e)
	{
		qDebug()<<"SpecificWorker::setFingers cannot connect with inversekinematics_proxy"<<e.what();
	}
}



/**
 * \brief Waits for the motors to stop
 */ 
void SpecificWorker::waitForMotorsToStop()
{
	MotorStateMap allMotorsCurr, allMotorsBack;
	jointmotor_proxy->getAllMotorState(allMotorsBack);
	usleep(500000);

	while(true)
	{
		printf("%s: %d\n", __FILE__, __LINE__);
		jointmotor_proxy->getAllMotorState(allMotorsCurr); //valor actual		
		for (auto v : allMotorsCurr)
		{
			if (abs(v.second.pos - allMotorsBack[v.first].pos) > 0.05)
			{
				allMotorsBack = allMotorsCurr;
				usleep(500000);
				break;
			}
		}
		return;
	}
}








