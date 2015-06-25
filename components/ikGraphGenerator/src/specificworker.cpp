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

	xrange = std::pair<float, float>(-100,  500);
	yrange = std::pair<float, float>( 500, 1300);
	zrange = std::pair<float, float>( 150,  450);
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

bool SpecificWorker::goAndWaitDirect(const MotorGoalPositionList &mpl)
{
	{
		QMutexLocker l(mutex);
		for (auto g : mpl)
		{
			innerVisual->updateJointValue(QString::fromStdString(g.name), g.position);
		}
	}
	float err = innerVisual->transform("target", "grabPositionHandR").norm2();
	printf("ERROR (%d)  %f\n", err>MAX_ERROR_IK, err);

	jointmotor_proxy->setSyncPosition(mpl);
	usleep(500000);

	return (err < MAX_ERROR_IK);
}

bool SpecificWorker::goAndWait(int nodeId, MotorGoalPositionList &mpl)
{
	RoboCompInverseKinematics::Pose6D target;
	target.x = graph->vertices[nodeId].pose[0];
	target.y = graph->vertices[nodeId].pose[1];
	target.z = graph->vertices[nodeId].pose[2];
	
	float relx = (graph->vertices[nodeId].pose[0] - xrange.first) / (xrange.second - xrange.first);
	printf("relx: %f\n", relx);
	
	if (relx < 0.33)
		target.ry = -0.8;
	else if (relx < 0.66)
		target.ry = -0.8;
	else
		target.ry = 0.;

	target.rx = 0;
	target.rz = -3.14;
	
	RoboCompInverseKinematics::WeightVector weights;
	weights.x  = weights.y  = weights.z  = 1;
	weights.rx = weights.ry = weights.rz = 1;

	int targetId = inversekinematics_proxy->setTargetPose6D("RIGHTARM", target, weights);

	TargetState stt;
	QTime initialTime = QTime::currentTime();
	do
	{
		stt = inversekinematics_proxy->getTargetState("RIGHTARM", targetId);
		usleep(50000);
		if (stt.finish == true)
			break;
	} while (initialTime.elapsed()<10000);

	if (not stt.finish)
	{
		return false;
	}

	{
		QMutexLocker l(mutex);
		innerVisual->updateTransformValues("target", target.x, target.y, target.z, 0,0,0);
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

	return goAndWaitDirect(mpl);
}

int SpecificWorker::getRandomNodeClose(int &current, float &dist)
{
	int some;
	while (true)
	{
		printf("get random\n");
		// Select random number
		some = rand() % graph->size();
		// If the selected number is the current node's index iterate again
		if (some == current) continue;

		// Compute direct distance
		dist = graph->vertices[current].distTo(graph->vertices[some].pose);
		// If the distance is small, we use such index
		if (dist < maxDist) return some;

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
	return some;
}

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

void SpecificWorker::compute()
{
	static bool first = true;
	if (first)
	{
		first = false;
		workerThread = new WorkerThread(this);
		workerThread->start();
	}
	updateFrame(1000);
}




void WorkerThread::run()
{
	while(true)
	{
		((SpecificWorker*)data)->computeHard();
		usleep(10000);
	}
}
