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

#define CLOSE_DISTANCE 120
/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx)
{
// 	maxDist = 180;



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


	InnerModelDraw::addTransform_ignoreExisting(innerViewer, "target", "root");

	InnerModelDraw::addPlane_ignoreExisting(innerViewer, "target_p", "target", QVec::vec3(0,0,0), QVec::vec3(1,0,0), "#7777ff", QVec::vec3(18,18,18));


	xrange = std::pair<float, float>( -120, 360);
	yrange = std::pair<float, float>( 600, 1200);
	zrange = std::pair<float, float>( 160, 500);

	QVec center = QVec::vec3((xrange.second+xrange.first)/2, (yrange.second+yrange.first)/2, (zrange.second+zrange.first)/2);

	InnerModelDraw::addPlane_ignoreExisting(innerViewer, "centro", "root", center, QVec::vec3(1,0,0), "#1111cc", QVec::vec3(10,10,10));


	float XR = abs(xrange.second - xrange.first);
	float YR = abs(yrange.second - yrange.first);
	float ZR = abs(zrange.second - zrange.first);
	float max = MAX(MAX(XR, YR), ZR);
	XR = XR/max;
	YR = YR/max;
	ZR = ZR/max;
	float step = sqrt((CLOSE_DISTANCE*CLOSE_DISTANCE)/3.);


	uint32_t included=0;
//#define RANDOMGENERATION
#ifdef RANDOMGENERATION
	uint32_t size_f = 200;
	graph = new ConnectivityGraph(size_f);
	while (included<size_f)
	{
		float xpos = QVec::uniformVector(size_f, xrange.first, xrange.second)(included);
		float ypos = QVec::uniformVector(size_f, yrange.first, yrange.second)(included);
		float zpos = QVec::uniformVector(size_f, zrange.first, zrange.second)(included);
#else
	graph = new ConnectivityGraph(0);
	for (float xpos = xrange.first; xpos<xrange.second; xpos+=step)
	{
		for (float ypos = yrange.first; ypos<yrange.second; ypos+=step)
		{
			for (float zpos = zrange.first; zpos<zrange.second; zpos+=step)
			{
#endif

				QVec pos = QVec::vec3(xpos, ypos, zpos);
				QVec diff = center-pos;
				diff(0) = abs(diff(0))/XR;
				diff(1) = abs(diff(1))/YR;
				diff(2) = abs(diff(2))/ZR;

#ifndef RANDOMGENERATION
				graph->addVertex(ConnectivityGraph::VertexData());
#endif
				if (diff.norm2() < 300)
				{
					QString id = QString("node_") + QString::number(included);
					graph->vertices[included].setPose(xpos, ypos, zpos);
					graph->vertices[included].configurations.clear();
					graph->vertices[included].id = included;

					{
						QMutexLocker l(mutex);
						InnerModelDraw::addPlane_ignoreExisting(innerViewer, id, "root",
							QVec::vec3(graph->vertices[included].pose[0],
							graph->vertices[included].pose[1], graph->vertices[included].pose[2]),
							QVec::vec3(1,0,0), "#666666", QVec::vec3(6,6,6)
						);
					}
					included++;
				}
#ifdef RANDOMGENERATION
	}
#else
			}
		}
	}
#endif


	printf("inc %d\n", included);

	int rec = 0;
	if (not goAndWait(180+(rand()%40), 780+(rand()%40), 300+(rand()%40), -1, centerConfiguration, rec))
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

	RoboCompInverseKinematics::WeightVector weights;
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
						printf("j:%d siiiiiiiiii d %f md %f mbb %f\n", j, dist, minDist, mustBeBiggerThan);
						if ((dist > mustBeBiggerThan and dist < minDist) or minDist == -1)
						{
							printf("pillo (%d) con dist %f (que es menor que %f)\n", j, dist, minDist);
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

/*
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
*/
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
				qDebug() << "fin feliz";
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
		}
		else
		{
			qDebug() << "Can't move to last target...";
// 			stop = true;
		}
	}

	nodeDst++;

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
		usleep(1000);
	}
}
