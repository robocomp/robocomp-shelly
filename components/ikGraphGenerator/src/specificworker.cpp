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

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx)
{
	maxDist = 100;



	show();
#ifdef USE_QTGUI
	innerViewer = NULL;
	osgView = new OsgView(this);
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

	uint32_t included=0;
	uint32_t size_f = 200;


	InnerModelDraw::addTransform_ignoreExisting(innerViewer, "target", "root");

	InnerModelDraw::addPlane_ignoreExisting(innerViewer, "target_p", "target", QVec::vec3(0,0,0), QVec::vec3(1,0,0), "#7777ff", QVec::vec3(18,18,18));


	xrange = std::pair<float, float>(-50,  400);
	yrange = std::pair<float, float>( 650, 950);
	zrange = std::pair<float, float>( 175,  400);

	graph = new ConnectivityGraph(size_f);
	while (included<size_f)
	{
		QVec xm = QVec::uniformVector(size_f, xrange.first, xrange.second);
		QVec ym = QVec::uniformVector(size_f, yrange.first, yrange.second);
		QVec zm = QVec::uniformVector(size_f, zrange.first, zrange.second);

		QString id = QString("node_") + QString::number(included);
		if (true)
		{
			graph->vertices[included].pose[0] = xm(included);
			graph->vertices[included].pose[1] = ym(included);
			graph->vertices[included].pose[2] = zm(included);
			graph->vertices[included].configurations.clear();
			graph->vertices[included].id = included;

			{
				QMutexLocker l(mutex);
				InnerModelDraw::addPlane_ignoreExisting(innerViewer, id, "root",
				  QVec::vec3(graph->vertices[included].pose[0],
				  graph->vertices[included].pose[1], graph->vertices[included].pose[2]),
				  QVec::vec3(1,0,0), "#990000", QVec::vec3(8,8,8)
				);
			}
			included++;
		}
	}


	if (not goAndWait(200, 800, 300, centerConfiguration, true))
		qFatal("Couldn't get initial position");


	timer.start(10);

	return true;
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
	jointmotor_proxy->setSyncPosition(mpl);
	usleep(500000);

	{
		QMutexLocker l(mutex);
		for (auto g : mpl)
		{
			innerVisual->updateJointValue(QString::fromStdString(g.name), g.position);
// 			printf("%s: %f\n", g.name.c_str(), g.position);
		}
	}
	return;
}

bool SpecificWorker::goAndWait(int nodeId, MotorGoalPositionList &mpl, bool recursive)
{
	return goAndWait(graph->vertices[nodeId].pose[0], graph->vertices[nodeId].pose[1], graph->vertices[nodeId].pose[2], mpl, recursive);
}

bool SpecificWorker::goAndWait(float x, float y, float z, MotorGoalPositionList &mpl, bool recursive)
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
		target.ry = -1.57;
	else if (relx < 0.66)
		target.ry = -0.8;
	else
		target.ry = 0.;

// 	float relz = (z - zrange.first) / (zrange.second - zrange.first);
// 	if (relz < 0.33)
// 		target.ry -= -0.8;
// 	else if (relz < 0.66)
// 		target.ry -= -0.4;
// 	else
// 		target.ry -= 0.;
// 	
// 	if (target.ry < -1.57)
// 		target.ry = -1.57;

// 	target.ry = -1.57;
	target.rz = -3.14;

	RoboCompInverseKinematics::WeightVector weights;
	weights.x  = weights.y  = weights.z  = 1;
	weights.rx = 0.01;
	weights.ry = 0.01;
	weights.rz = 0.1;

	int targetId = inversekinematics_proxy->setTargetPose6D("RIGHTARM", target, weights);

	TargetState stt;
	QTime initialTime = QTime::currentTime();
	do
	{
		usleep(500000);
		stt = inversekinematics_proxy->getTargetState("RIGHTARM", targetId);
		if (stt.finish == true)
			break;
	} while (initialTime.elapsed()<10000);

	if (initialTime.elapsed()>=10000)
		qFatal("10 seconds!");

	if (not stt.finish)
	{
		printf("cant\'t go\n");
		return false;
	}

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

	if (stt.errorT > MAX_ERROR_IK)
	{
		if (not recursive)
		{
			printf("reseteando al mas parecido\n");
			float minDist = -1;
			MotorGoalPositionList minConfig;
			for (int j=0; j<graph->size(); j++)
			{
				if (graph->vertices[j].configurations.size() > 0)
				{
					float p[3];
					p[0] = x;
					p[1] = y;
					p[2] = z;
					float dist = graph->vertices[j].distTo(p);
					if (minDist < 0 or minDist > dist)
					{
						minDist = dist;
						minConfig = graph->vertices[j].configurations[0];
					}
				}
			}
			goAndWaitDirect(minConfig);
			printf("realizando la llamada recursiva\n");
			return goAndWait(x, y, z, mpl);
		}
		else
		{
			return false;
		}
	}
	return true;
}

int SpecificWorker::getRandomNodeClose(int &current, float &dist)
{
	int some;
	for (int iter=0; iter<100; iter++)
	{
		printf("get random\n");
		// Select random number
		some = rand() % graph->size();
		// If the selected number is the current node's index iterate again
		if (some == current) continue;

		// Compute direct distance
		dist = graph->vertices[current].distTo(graph->vertices[some].pose);
		// If the distance is small, we use such index
		if (dist < maxDist)
			return some;

		if (rand()%100 < 30)
		{
			printf("get random Dijkstra\n");

			// Otherwhise, check graph-based distance
			Dijkstra d = Dijkstra(&(graph->edges));
			d.calculateDistance(current);
			for (int i=0; i<graph->size(); i++)
			{
				std::vector<int> path;
				if (d.go(i, path) != -1)
				{
					dist = graph->vertices[i].distTo(graph->vertices[some].pose);
					if (dist < maxDist and graph->vertices[i].configurations.size()>0)
					{
						printf("goAndWaitDirect Djk 1\n");
						goAndWaitDirect(graph->vertices[i].configurations[0]);
						printf("goAndWaitDirect Djk 2\n");
						current = i;
						return some;
					}
				}
			}
		}
	}
	return -1;
}

void SpecificWorker::computeHard()
{
	bool stop=false;
	static int nodeSrc=0;
	static int nodeDst=-1;
	static MotorGoalPositionList currentConfiguration;
	MotorGoalPositionList configuration;

	if (stop) return;

	if (nodeDst == -1)
	{
		qDebug() << nodeSrc;
		if (not goAndWait(nodeSrc, configuration))
		{
			qFatal("Couln't reach target");
		}
		currentConfiguration = configuration;
		nodeDst=0;
		return;
	}

	if (nodeDst == nodeSrc)
	{
		nodeDst++;
		if (nodeDst == graph->size())
		{
			nodeDst = -1;
			nodeSrc++;
			if (nodeSrc == graph->size())
			{
				qDebug() << "fin";
				stop = true;
			}
		}
	}

// 	printf("%d %d\n", nodeSrc, nodeDst);
/*
	dist = graph->vertices[nodeSrc].distTo(graph->vertices[nodeDst].pose);
	if (dist < 100)
	{
		if (goAndWait(nodeDst, configuration))
		{
			printf("goAndWait done\n");
			printf("add %d %d %f\n", nodeSrc, nodeDst, dist);
			graph->add_edge(nodeSrc, nodeDst, dist*dist);
			graph->add_configurationToNode(nodeDst, configuration);

			{
				static int edId = 0;
				const QString id = QString("edge_") + QString::number(edId++);
				float *p1 = graph->vertices[nodeSrc].pose;
				float *p2 = graph->vertices[nodeDst].pose;
				QMutexLocker l(mutex);
				InnerModelDraw::drawLine2Points(innerViewer, id, "root", QVec::vec3(p1[0], p1[1], p1[2]), QVec::vec3(p2[0], p2[1], p2[2]), "#88ff88");
			}

			goAndWaitDirect(currentConfiguration);
			nodeSrc = nodeDst;
		}
		else
		{
			qDebug() << "Can't move to last target. Reinitializing...";
			stop = true;
		}
	}
*/
	nodeDst++;
	if (nodeDst == graph->size())
	{
		nodeDst = -1;
		nodeSrc++;
		if (nodeSrc == graph->size())
		{
				qDebug() << "fin2";
				stop = true;
		}
	}

}

/*
void SpecificWorker::computeHard()
{
	static bool first = true;
	static int currentNode;
	static MotorGoalPositionList currentConfiguration;

	MotorGoalPositionList configuration;

	if (first or (rand()%100 < 5) )
	{
		while (true)
		{
			printf("random pose!\n");
			currentNode = rand() % graph->size();
			if (goAndWait(currentNode, configuration))
			{
				if (first)
					printf("initialized!\n");
				else
					printf("re-initialized!\n");
				first = false;
				currentConfiguration = configuration;
				printf("si\n");
				break;
			}
			printf("no\n");
		}
	}

	float dist;
	printf("aaa\n");
	int nextNode = getRandomNodeClose(currentNode, dist);
	if (nextNode == -1)
	{
		first = true;
		return;
	}
	printf("bbb\n");

	printf("goAndWait\n");
	if (goAndWait(currentNode, configuration))
	{
		printf("goAndWait done\n");
		printf("add %d %d %f\n", currentNode, nextNode, dist);
		graph->add_edge(currentNode, nextNode, dist);
		graph->add_configurationToNode(nextNode, configuration);

		{
			static int edId = 0;
			const QString id = QString("edge_") + QString::number(edId++);
			float *p1 = graph->vertices[currentNode].pose;
			float *p2 = graph->vertices[nextNode].pose;
			QMutexLocker l(mutex);
			InnerModelDraw::drawLine2Points(innerViewer, id, "root", QVec::vec3(p1[0], p1[1], p1[2]), QVec::vec3(p2[0], p2[1], p2[2]), "#88ff88");
		}

		currentConfiguration = configuration;
		currentNode = nextNode;
	}
	else
	{
		printf("Can't move to last target. Reinitializing...\n");
		first = true;
// 		goAndWaitDirect(currentConfiguration);
	}
}
*/

void SpecificWorker::compute()
{
	static bool first = true;
	if (first)
	{
		first = false;
		workerThread = new WorkerThread(this);
		workerThread->start();
	}
	updateFrame(10);
}




void WorkerThread::run()
{
	while(true)
	{
		((SpecificWorker*)data)->computeHard();
		usleep(100);
	}
}
