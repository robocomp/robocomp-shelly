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

#define MAX_SPEED 1.5
// #define MAX_SPEED 0.7

#define STEP_DISTANCE 40
// #define CLOSE_DISTANCE (STEP_DISTANCE*2.5)
#define CLOSE_DISTANCE (STEP_DISTANCE*1.8)
/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////
//      http://pointclouds.org/documentation/tutorials/kdtree_search.php
//      http://robotica.unileon.es/mediawiki/index.php/PCL/OpenNI_tutorial_3:_Cloud_processing_%28advanced%29
//--->        pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////    
/** ------------------------------------------------------
* \brief Default constructor
* @param mprx 
*------------------------------------------------------*/
SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx)
{ 
	READY         = false;
	printf("STATE: %d\n", state);
	state         = GIK_NoTarget;
	printf("STATE CHANGED TO: %d\n", state);
	targetCounter = 0;
	mutexSolved   = new QMutex(QMutex::Recursive);
	// NOTE Inicializamos las nubes de puntos
//	full_cloud     = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>);
//	cloud_filtered = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>);

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
//------------------------------------------------------------------------------------------------------------------------------------------------
		//my mesh para ver si chocamos con el brazo.
		//my_mesh = innerModel->newMesh("my_mesh", innerModel->getNode("root"), "/home/robocomp/robocomp/files/osgModels/basics/cube.3ds", 
		//			       25,25,25,
		//			       0,
		//		               0,0,0,0,0,0, 
		//		               true);
		//recursiveIncludeMeshes(innerModel->getNode("arm_pose"), meshes);
		//std::sort(meshes.begin(), meshes.end()); 
//------------------------------------------------------------------------------------------------------------------------------------------------
	
#ifdef USE_QTGUI
		innerVisual = new InnerModel(par.value);
		innerViewer = new InnerModelViewer(innerVisual, "root", osgView->getRootGroup(), true);
		osgView->getRootGroup()->addChild(innerViewer);
		show();
// 		//InnerModelDraw::addMesh_ignoreExisting(innerViewer, "my_mesh", "root", QVec::zeros(3), QVec::zeros(3),"/home/robocomp/robocomp/files/osgModels/basics/cube.3ds", QVec::vec3(25,25,25));

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

	timer.start(10);
 	initFile();
	//qDebug()<<"[ikGraphGenerator]: READY CONFIG PARAMS";
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
		graph = new ConnectivityGraph("/home/robocomp/robocomp/components/robocomp-shelly/components/ikGraphGenerator/shelly.ikg");
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

	QVec center = QVec::vec3(0, 900, 640);
	xrange = std::pair<float, float>( center(0)-100, center(0)+100 + 1);
	yrange = std::pair<float, float>( center(1)-200, center(1)+200 + 1);
	zrange = std::pair<float, float>( center(2)-200, center(2)+200 + 1);
// 	QVec center = QVec::vec3((xrange.second+xrange.first)/2, (yrange.second+yrange.first)/2, (zrange.second+zrange.first)/2);

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
				if ((pos-center).norm2() < 150)
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
	int rec = 0;
// 	if (not goAndWait(0, 900, 560, -1, centerConfiguration, rec))
	if (not goAndWait(center(0), center(1), center(2), -1, centerConfiguration, rec))
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
 * \brief goAndWaitDirect. Mueve el brazo en base a la posicion de los motores almacenada en mpl.
 * @param mpl motores y posiciones de los motores.
 * ------------------------------------------------------*/
void SpecificWorker::goAndWaitDirect(const MotorGoalPositionList &mpl)
{
	static MotorGoalPositionList last;
	if (mpl == last)
	{
		//printf("skipping same config\n");
		return;
	}
	last = mpl;
	try
	{
		QTime t = QTime::currentTime();
		jointmotor_proxy->setSyncPosition(mpl);
		printf("elapsed: %d\n", t.elapsed());
	}
	catch (const RoboCompJointMotor::CollisionException &ex)
	{
		cout<<"--> Collision in commonjoint" << ex << "\n";
		lastFinish = "ERROR";
		printf("STATE: %d\n", state);
		state = GIK_NoTarget;
		printf("STATE CHANGED TO: %d\n", state);
		return;
	}
	catch (const Ice::Exception &ex)
	{
		cout<<"--> Ice::Exception:" << ex.what() <<"\n";
		lastFinish = "ERROR";
		printf("STATE: %d\n", state);
		state = GIK_NoTarget;
		printf("STATE CHANGED TO: %d\n", state);
		return;
	}
	{
		QMutexLocker l(mutex);
		for (auto g : mpl)
		{
#ifdef USE_QTGUI
			innerVisual->updateJointValue(QString::fromStdString(g.name), g.position);
#endif
		}
	}
	usleep(200);
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
	RoboCompInverseKinematics::Pose6D target;
	target.x = x;
	target.y = y;
	target.z = z;

	
	float rely = (y - yrange.first) / (yrange.second - yrange.first);
	if (rely < 0.33)
		target.rx = 0.0;
	else if (rely < 0.66)
		target.rx = 0;
	else
		target.rx = -0.0;
	target.rx = 0;
	
	
// 	float relx = (x - xrange.first) / (xrange.second - xrange.first);
// 	if (relx < 0.33)
// 		target.ry = -1.8;
// 	else if (relx < 0.66)
// 		target.ry = -1.47;
// 	else
// 		target.ry = -0.47;


// 	if (x < 50)
// 		target.ry = -1.5;
// 	else if (x > 400)
// 		target.ry = 0;
// 	else
// 		target.ry = -0.785398163397448;

	target.rx = 0;
	target.ry = 0;
	target.rz = 0;

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
	weights.rx = weights.ry = weights.rz = 0;
 	if (recursive==0)
	{
		weights.rx = 0.6;
	}
 	

	int targetId;
	try
	{
		targetId = inversekinematics_proxy->setTargetPose6D("ARM", target, weights);
	}
	catch (...)
	{
		printf("err 0\n");
	}

	TargetState stt;
	QTime initialTime = QTime::currentTime();
	do
	{
		usleep(10000);
		try
		{
			stt = inversekinematics_proxy->getTargetState("ARM", targetId);
		}
		catch (...)
		{
			printf("err\n");
		}
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
	printf("time %d\n", initialTime.elapsed());
	printf("finished: %d\n", stt.finish);
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
	static bool stop=false;
	static int nodeSrc=-1;
	static int nodeDst=-1;
	static MotorGoalPositionList currentConfiguration;

	if (stop) return;
	printf("======================================= %d\n", nodeDst);
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
			printf("laaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa\n");
			printf("laaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa\n");
			printf("laaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa\n");
			printf("laaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa\n");
			exit(0);
			return;
		}
		MotorGoalPositionList configuration;
		int de;
		int recursive = 0;
		for (de=0; de < 2 and not goAndWait(nodeSrc, configuration, recursive); de++)
		{
			int d=0;
			goAndWait(0, 850, 650, -1, centerConfiguration, d);
		}
		if (de == 2)
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
		QVec elbowPose = innerModel->transform("robot", "arm_elbow");
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
 * ------------------------------------------------------*/
void SpecificWorker::compute()
{
	QMutexLocker l(mutex);
	if (not READY) return;

	

#ifdef USE_QTGUI
	ui_state->setText(QString::number(state));
	QVec rrr = innerModel->transform6D("robot", "grabPositionHandR");
	ui_x->setText( (fabs(rrr(0))<0.0001)?QString("0"):QString::number(rrr(0)));
	ui_y->setText( (fabs(rrr(1))<0.0001)?QString("0"):QString::number(rrr(1)));
	ui_z->setText( (fabs(rrr(2))<0.0001)?QString("0"):QString::number(rrr(2)));
	ui_rx->setText((fabs(rrr(3))<0.0001)?QString("0"):QString::number(rrr(3)));
	ui_ry->setText((fabs(rrr(4))<0.0001)?QString("0"):QString::number(rrr(4)));
	ui_rz->setText((fabs(rrr(5))<0.0001)?QString("0"):QString::number(rrr(5)));
#endif

	updateFrame(10);
	updateInnerModel();

	static uint32_t pathIndex = 0;

	switch(state)
	{
	case GIK_NoTarget:
		static int tick = 0;
		if (tick++ % 3 == 0)
		{
			printf(".");
			fflush(stdout);
		}
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
	static QTime t;
	case GIK_GoToInit :
		t = QTime::currentTime();
		printf("GIK_GoToInit\n");
		goAndWaitDirect(graph->vertices[closestToInit].configurations[0]);
		pathIndex = 0;
		if (path.size() > 1) pathIndex = 1;
		printf("STATE: %d\n", state);
		state = GIK_GoToEnd;
		printf("STATE CHANGED TO: %d\n", state);
		printf("elapsed1: %d\n", t.elapsed());
		break;
	//--------------------------------------------------------------------------------------------------//
	case GIK_GoToEnd:
		printf("GIK_GoToEnd\n");
		printf("elapsed1: %d\n", t.elapsed());

		goAndWaitDirect(graph->vertices[path[pathIndex]].configurations[0]);
		printf("%d %f %f %f\n", pathIndex, graph->vertices[path[pathIndex]].pose[0], graph->vertices[path[pathIndex]].pose[1], graph->vertices[path[pathIndex]].pose[2]);
		pathIndex++;
		waitForMotorsToStop();
		if (pathIndex>=path.size())
		{
			pathIndex = 0;
			printf("STATE: %d\n", state);
			state = GIK_GoToActualTargetSend;
			printf("STATE CHANGED TO: %d\n", state);
			qDebug()<<"\n\nPASAMOS A LA IK!!!!!!!!!\n";
			usleep(2000);
		}
		printf("elapsed2: %d\n", t.elapsed());
		break;
	//--------------------------------------------------------------------------------------------------//
	case GIK_GoToActualTargetSend:
		printf("GIK_GoToActualTargetSend\n");
		printf("elapsed1: %d\n", t.elapsed());

		qDebug()<<"SEND---->("<<currentTarget.pose.x<<", "<<currentTarget.pose.y<<", "<<currentTarget.pose.z<<")";
		try {
			currentTarget.id_IK = inversekinematics_proxy->setTargetPose6D("ARM", currentTarget.pose, currentTarget.weights);
			printf("STATE: %d\n", state);
			state = GIK_GoToActualTargetSent;
			printf("STATE CHANGED TO: %d\n", state);
		}
		catch (Ice::Exception e){ qDebug()<<"cannot connect with inversekinematics_proxy"<<e.what();}
		printf("elapsed2: %d\n", t.elapsed());

		break;
	//--------------------------------------------------------------------------------------------------//
	case GIK_GoToActualTargetSent:
		{
			TargetState stt = inversekinematics_proxy->getTargetState("ARM", currentTarget.id_IK);
// 			printf("GIK_GoToActualTargetSent\n");
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
					usleep(5000);
				}
				qDebug()<<"finish: "<<QString::fromStdString(lastFinish);
				updateInnerModel();
				solvedList.enqueue(currentTarget); //guardamos el target
				printf("(%f %f %f)\n", currentTarget.pose.x, currentTarget.pose.y, currentTarget.pose.z);
				qDebug()<<"ERROR T: "<<currentTarget.state.errorT;
				printf("STATE: %d\n", state);
				state = GIK_NoTarget;
				printf("STATE CHANGED TO: %d\n", state);
			}
		break;
	}
	default:
		printf("GIK_ DEFAULT\n");

	}
}



/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////
bool SpecificWorker::delete_collision_points()
{ 
#ifdef USE_PCL
	// Actualizamos el mesh auxiliar para ponerlo donde no estorbe.
	innerModel->updateTransformValues("my_mesh", 0,0,0,0,0,0, "root");

	float collision_threshold = 40;                  // Los centímetros entre el punto y los nodos del grafo 8 cm
	RoboCompRGBD::PointSeq point_cloud;              // Los puntos detectados por la RGBD
	RoboCompJointMotor::MotorStateMap hState;        // Para sacar los puntos de la RGBD
	RoboCompDifferentialRobot::TBaseState bState;    // Para sacar los puntos de la RGBD
	
	// Obtenemos la nube de puntos
	try{
		rgbd_proxy->getXYZ(point_cloud, hState, bState); 
	}catch(...)
	{
		std::cout<<"Error point cloud\n";
	}

	//Pasamos puntos a pcl: full_cloud es un array que contiene estructuras del tipo PointXYZ
	full_cloud->points.resize(point_cloud.size()); // Numero de puntos en la nube PCL

	// ALERT Valores puestos a pelo desde el fichero del grafo
	// TODO Método que te devuelva el punto más cercano y el más lejano para hacer el cubo espacial
	// Guardamos los puntos dentro del array si estan dentro del volumen de trabajo del robot.
	float minx= -120.0, miny= 820.0, minz=560.0;
	float maxx=  120.0, maxy= 1020.0, maxz=760.0;
	
	QMat mat = innerModel->getTransformationMatrix("robot", "rgbd"); //matriz de transformacion
	int32_t usedPoints = 0;
	
	//cargamos el numero de hilos que lanzaran los procesos
	int num_hilos = 3;
	omp_set_num_threads(num_hilos);
	
	#pragma omp parallel for shared(usedPoints) ordered schedule(static,1)
	for (uint32_t i=0; i<point_cloud.size(); /*i++*/i=i+10) //casi mismo resultado que si computamos con todos los puntos
	{
		QVec v = (mat * QVec::vec4(point_cloud[i].x, point_cloud[i].y, point_cloud[i].z, 
		                           point_cloud[i].w)).fromHomogeneousCoordinates(); //transformamos al robot
		
		if (v(0)>=minx and v(0)<=maxx and v(1)>=miny and v(1)<=maxy and v(2)>=minz and v(2)<=maxz)
		{
			#pragma omp ordered
			{

				full_cloud->points[usedPoints].x =  v(0);
				full_cloud->points[usedPoints].y =  v(1);
				full_cloud->points[usedPoints].z =  v(2);
				usedPoints++;
			}//fin del pragma
		}//fin del if
	}
	
	if (usedPoints>0)
	{
		// AHora guardamos los puntos restantes en la nube
		full_cloud->width = usedPoints;
		full_cloud->height = 1;
		full_cloud->points.resize(usedPoints);
		qDebug()<<"Used Points: "<<usedPoints;
		qDebug()<<"Points before: "<<point_cloud.size()<<" Points after: "<<full_cloud->points.size()<<" --> "<<full_cloud->points.size()/float(point_cloud.size())*100<<" %";

		//Quitamos densidad --> Create the filtering object
	// 	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
		pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
		sor.setInputCloud (full_cloud);
		sor.setMeanK (50);
		sor.setStddevMulThresh (1.0);
		sor.filter (*cloud_filtered);
		qDebug()<<"Points before: "<<full_cloud->points.size()<<" Points after: "<<cloud_filtered->points.size()<<" --> "<<cloud_filtered->points.size()/float(full_cloud->points.size())*100<<" %";
		// Pasamos a KDTREE para agilizar búsquedas
		pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
		kdtree.setInputCloud (cloud_filtered);
		
		// Comparamos los nodos del grafo con el KDTREE:
// 		pcl::PointXYZ searchPoint;                     //estructura para comparar
// 		std::vector<int> pointIdxRadiusSearch;         //auxiliar, no se utilizara
// 		std::vector<float> pointRadiusSquaredDistance; //auxiliar, no se utilizara
		//recorremos grafo
		int free_nodes=0;

		#pragma omp parallel for shared(free_nodes) ordered schedule(static,1)
		for (uint i=0; i<graph->vertices.size(); i++)
		{
			pcl::PointXYZ searchPoint;                     //estructura para comparar
			std::vector<int> pointIdxRadiusSearch;         //auxiliar, no se utilizara
			std::vector<float> pointRadiusSquaredDistance; //auxiliar, no se utilizara
		
			if (graph->vertices[i].valid)
			{
				// Pasamos el nodo a PointXYZ
				searchPoint.x = graph->vertices[i].pose[0];
				searchPoint.y = graph->vertices[i].pose[1];
				searchPoint.z = graph->vertices[i].pose[2];
				// Si hay puntos que chocan contra el vertice dle grafo, hay que invalidarlo
				#pragma omp ordered
				{
					if (kdtree.radiusSearch (searchPoint, collision_threshold, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )
					{
						// De esos puntos quitamos los que se correspondan con el brazo
						innerModel->updateTransformValues("my_mesh", searchPoint.x, searchPoint.y, searchPoint.z,0,0,0, "root");
						for (auto a: meshes)
						{
							if (innerModel->collide(a, "my_mesh")==false)
							{
								graph->vertices[i].state=ConnectivityGraph::VertexState::LOCKED_NODE;
#ifdef USE_QTGUI
								InnerModelDraw::addPlane_ignoreExisting(innerViewer, (QString("node_") + QString::number(i)), "root",
													QVec::vec3(graph->vertices[i].pose[0], 
														graph->vertices[i].pose[1], 
														graph->vertices[i].pose[2]),
													QVec::vec3(1,0,0), "#cc2222", QVec::vec3(2,2,2));
#endif
								break;
							}// fin del if
							else
							{
								graph->vertices[i].state=ConnectivityGraph::VertexState::FREE_NODE;
								free_nodes++;
#ifdef USE_QTGUI
								InnerModelDraw::addPlane_ignoreExisting(innerViewer, (QString("node_") + QString::number(i)), "root",
													QVec::vec3(graph->vertices[i].pose[0], 
														graph->vertices[i].pose[1], 
														graph->vertices[i].pose[2]),
													QVec::vec3(1,0,0), "#22cc22", QVec::vec3(2,2,2));
#endif
							}
						}//fin del for
					}
					else
					{
						graph->vertices[i].state=ConnectivityGraph::VertexState::FREE_NODE;
						free_nodes++;
#ifdef USE_QTGUI
						InnerModelDraw::addPlane_ignoreExisting(innerViewer, (QString("node_") + QString::number(i)), "root",
											QVec::vec3(graph->vertices[i].pose[0], 
												graph->vertices[i].pose[1], 
												graph->vertices[i].pose[2]),
											QVec::vec3(1,0,0), "#22cc22", QVec::vec3(2,2,2));
#endif
					}
				}
			}
		}
		// http://pointclouds.org/documentation/tutorials/kdtree_search.php
		qDebug()<<"NODOS LIBRES: "<<free_nodes<<" DE "<<graph->vertices.size()<<" NODOS EN TOTAL";
		return true;
	}
#endif
	return false;
}
/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////
void SpecificWorker::recursiveIncludeMeshes(InnerModelNode *node, std::vector<QString> &in)
{
#ifdef USE_PCL
	bool niapa = true;
	if (niapa)
	{
		in.push_back("shellyArm_BASE_mesh");
		in.push_back("shellyArm_BASE2_mesh");
		in.push_back("shellyArm_HUMERO_mesh");
		in.push_back("shellyArm_CODO_mesh");
		in.push_back("shellyArm_ANTEBRAZO_mesh");
		in.push_back("finger_wrist_mesh");
		in.push_back("handMeshBase");
		in.push_back("handMesh2");
		in.push_back("finger_wrist_1_mesh2");
		in.push_back("finger_wrist_1_mesh3");
		in.push_back("finger_wrist_1_mesh4");
		in.push_back("finger_wrist_1_mesh5");
		in.push_back("finger_wrist_1_mesh6");
		in.push_back("finger_wrist_1_mesh7");
		in.push_back("finger_wrist_2_mesh2");
		in.push_back("finger_wrist_2_mesh3");
		in.push_back("finger_wrist_2_mesh4");
	}
	else
	{
		QMutexLocker locker(mutex);
	
		InnerModelMesh *mesh;
		InnerModelPlane *plane;
		InnerModelTransform *transformation;

		if ((mesh = dynamic_cast<InnerModelMesh *>(node)) or (plane = dynamic_cast<InnerModelPlane *>(node)))
		{
			in.push_back(node->id);
		}

		else if ((transformation = dynamic_cast<InnerModelTransform *>(node)))
		{
			for (int i=0; i<node->children.size(); i++)
			{
				recursiveIncludeMeshes(node->children[i], in);
			}
		}
// 		qFatal("dd");
	}
#endif
}


/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////
void WorkerThread::run()
{
	printf("%s: %d\n", __FILE__, __LINE__);
	while (true)
	{
		((SpecificWorker*)data)->computeHard();
		usleep(1000);
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

	setTargetPose6D("ARM", currentTarget.pose, currentTarget.weights);
#endif
}



/**-----------------------------------------------
 * \brief goHome
 * ----------------------------------------------*/
void SpecificWorker::goHome()
{
// 	printf("%s: %d\n", __FILE__, __LINE__);
// 	MotorGoalPositionList listGoals;
// 	listGoals.resize(7);
// 	listGoals[0].name     = "rightShoulder1";
// 	listGoals[0].position = -2.7;
// 	listGoals[1].name     = "rightShoulder2";
// 	listGoals[1].position = -0.2;
// 	listGoals[2].name     = "rightShoulder3";
// 	listGoals[2].position = 1.5;
// 	listGoals[3].name     = "rightElbow";
// 	listGoals[3].position = 0.4;
// 	listGoals[4].name   = "rightForeArm";
// 	listGoals[4].position = -1.;
// 	listGoals[5].name = "rightWrist1";
// 	listGoals[5].position = 0.;
// 	listGoals[6].name = "rightWrist2";
// 	listGoals[6].position = 0.;
// 
// 	for (int i=0; i<7; i++)
// 	{
// 		listGoals[i].maxSpeed = MAX_SPEED;
// 	}
// 
// 	jointmotor_proxy->setSyncPosition(listGoals);
// 	usleep(20000);
	printf("%s: %d\n", __FILE__, __LINE__);
	qFatal("home?\n");
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
		printf("STATE: %d\n", state);
		state = GIK_GoToActualTargetSend;
		printf("STATE CHANGED TO: %d\n", state);
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
			for (uint i=0; i<tempPath.size(); i++) printf(" %d ", tempPath[i]);
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
			for (uint i=0; i<path.size(); i++) printf(" %d ", path[i]);
			printf("\n");
		}

		state = GIK_GoToInit;
		printf("state switched to %d\n", state);
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
	
// 	qDebug()<<"PEDIDO TARGET "<<targetID;
	QMutexLocker mm(mutexSolved);
	for(int i=0; i<solvedList.size(); i++)
	{
		if (targetID == solvedList[i].id_IKG)
		{
// 			qDebug()<<"PEDIDO TARGET "<<targetID<<". ENCONTRADO.";
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
	printf("STATE: %d\n", state);
	state = GIK_NoTarget;
	printf("STATE CHANGED TO: %d\n", state);
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
	MotorStateMap allMotorsCurr/*, allMotorsBack*/;

	bool moveInitialized = false;
	bool moveFinished = false;
	QTime waitTime = QTime::currentTime();

	while (waitTime.elapsed() < 1200)
	{
		try
		{
			jointmotor_proxy->getAllMotorState(allMotorsCurr);
		}
		catch(...)
		{
			std::cout<<"Error retrieving all motors state\n";
		}
		bool someMotorMoving = false;
		for (auto v : allMotorsCurr)
		{
			if (fabs(v.second.vel)>0.8)
			{
				someMotorMoving = true;
				usleep(50000);
				break;
			}
		}
		
		if (someMotorMoving==true   ) moveInitialized = true;
		if (someMotorMoving==false and moveInitialized == true) moveFinished = true;

		if (moveInitialized == true and moveFinished == true)
			return;
	}
}



int SpecificWorker::mapBasedTarget(const string &bodyPart, const StringMap &strings, const ScalarMap &scalars)
{
	return 0;
}






