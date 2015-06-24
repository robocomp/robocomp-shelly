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
		printf("ddd\n");
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

	graph = new ConnectivityGraph(size_f);
	while (included<size_f)
	{
		printf("inc: %d\n", included);
		QVec xm = QVec::uniformVector(size_f, -100, 500);
		QVec ym = QVec::uniformVector(size_f, 500, 1300);
		QVec zm = QVec::uniformVector(size_f, 150, 450);

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
	jointmotor_proxy->setSyncPosition(mpl);
	{
		QMutexLocker l(mutex);
		for (auto g : mpl)
		{
			innerVisual->updateJointValue(QString::fromStdString(g.name), g.position);
		}
	}
	usleep(500000);
	return true;
}

bool SpecificWorker::goAndWait(int nodeId, MotorGoalPositionList &mpl)
{
	printf("goAndWait 1\n");
	RoboCompInverseKinematics::Pose6D target;
	target.x = graph->vertices[nodeId].pose[0];
	target.y = graph->vertices[nodeId].pose[1];
	target.z = graph->vertices[nodeId].pose[2];
	target.rx = target.ry = 0;
	target.rz = -3.14;
	RoboCompInverseKinematics::WeightVector weights;
	weights.x  = weights.y  = weights.z  = 1;
	weights.rx = weights.ry = weights.rz = 0;

	int targetId = inversekinematics_proxy->setTargetPose6D("RIGHTARM", target, weights);

	TargetState stt;
	QTime initialTime = QTime::currentTime();
	do
	{
		stt = inversekinematics_proxy->getTargetState("RIGHTARM", targetId);
		usleep(500000);
		if (stt.finish == true)
			break;
	} while (initialTime.elapsed()<10000);

	if (not stt.finish)
	{
		printf("goAndWait fails\n");
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
	goAndWaitDirect(mpl);

	return true;
}

int SpecificWorker::getRandomNodeClose(int &current, float &dist)
{
	printf("getRandomNodeClose A\n");
	int some;
	while (true)
	{
		// Select random number
		some = rand() % graph->size();
		// If the selected number is the current node's index iterate again
		if (some == current) continue;

		// Compute direct distance
		dist = graph->vertices[current].distTo(graph->vertices[some].pose);
		// If the distance is small, we use such index
		if (dist < maxDist) return some;

		if (false)
		{
			// Otherwhise, check graph-based distance
			Dijkstra d = Dijkstra(&(graph->edges));
			d.calculateDistance(current);
			for (int i=0; i<graph->size(); i++)
			{
				if (d.distance[i] < DJ_INFINITY)
				{
// 					printf("distancia %f\n", d.distance[i]);
					dist = graph->vertices[i].distTo(graph->vertices[some].pose);
					if (dist < maxDist and graph->vertices[i].configurations.size()>0)
					{
// 						printf("configs pa ese punto %d\n", int(graph->vertices[i].configurations.size()));
						goAndWaitDirect(graph->vertices[i].configurations[0]);
						usleep(500000);
						current = i;
						return some;
					}
				}
			}
		}
	}
	printf("getRandomNodeClose Z\n");
	return some;
}

void SpecificWorker::computeHard()
{
	static int currentNode = 0;
	static bool first = true;
	static MotorGoalPositionList currentConfiguration;

	MotorGoalPositionList configuration;

	if (first)
	{
		first = false;
		while (currentNode < graph->size())
		{
			if (goAndWait(currentNode, configuration))
			{
				printf("initialized!\n");
				currentConfiguration = configuration;
				break;
			}
			printf("minitialized!\n");
			currentNode++;
		}
		printf("dddeee\n");
	}

	float dist;
	int nextNode = getRandomNodeClose(currentNode, dist);
	printf("compute 5\n");

	if (goAndWait(currentNode, configuration))
	{
		printf("add %d %d %f\n", currentNode, nextNode, dist);
		printf("add %d %d %f\n", currentNode, nextNode, dist);
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
		printf("noooo00000000000000000000000000oooo");
		goAndWaitDirect(currentConfiguration);
	}

	usleep(1000);
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
