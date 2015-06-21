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
	show();
	printf("ss\n");
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
		innerModel = new InnerModel(par.value);

// #ifdef USE_QTGUI
		printf("ddd\n");
		innerVisual = new InnerModel(par.value);
		innerViewer = new InnerModelViewer(innerVisual, "root", osgView->getRootGroup(), true);
		osgView->getRootGroup()->addChild(innerViewer);
		show();
// #endif
	}
	else
	{
		std::cout << "Innermodel path " << par.value << " not found. ";
		qFatal("Abort");
	}


	uint32_t included=0;
	while (included<400)
	{
		printf("inc: %d\n", included);
		QVec xm = QVec::uniformVector(1, -100, 500);
		QVec ym = QVec::uniformVector(1, 500, 1500);
		QVec zm = QVec::uniformVector(1, 100, 700);

		QString id = QString("node_") + QString::number(included);
		if (true)
		{
			Vertex vertex = boost::add_vertex(graph);
			graph[vertex].pose = QVec::vec3(xm(0), ym(0), zm(0));
			graph[vertex].configurations.clear();
			InnerModelDraw::addPlane_ignoreExisting(innerViewer, id, "root", QVec::vec3(xm(0),ym(0),zm(0)), QVec::vec3(1,0,0), "#990000", QVec::vec3(5,5,5));
			included++;
		}
	}

	timer.start(10);

// 	bodyinversekinematics_proxy 

	return true;
}

void SpecificWorker::compute()
{
	static VertexIterator current;
	static bool first = true;
	
	if (first)
	{
		VertexIterator last;
		tie(current, last) = vertices(graph);
		
		while (first and current != last)
		{
			RoboCompInverseKinematics::Pose6D target;
// 			target.x = current->pose.x();
// 			target.y = current->pose.y();
// 			target.z = current->pose.z();
			target.rx = target.ry = target.rz = 0;
			RoboCompInverseKinematics::WeightVector weights;
			weights.x  = weights.y  = weights.z  = 1;
			weights.rx = weights.ry = weights.rz = 1;
			inversekinematics_proxy->setTargetPose6D("RIGHTARM", target, weights);
			
			first = false;
			sleep(2);

#ifdef USE_QTGUI
			if (innerViewer)
				innerViewer->update();
			osgView->autoResize();
			osgView->frame();
			break;
#endif
		}
		return;
	}
	

// 	mt19937 gen;
// 	auto v = graph_traits<Graph>::vertex_descriptor(graph, gen);

	
	typedef property_map<Graph, vertex_index_t>::type IndexMap;
	IndexMap index = get(vertex_index, graph);

	std::cout << "vertices(g) = ";
	typedef boost::graph_traits<Graph>::vertex_iterator VertexIterator;
	std::pair<vertex_iter, VertexIterator> vp;
	for (vp = vertices(g); vp.first != vp.second; ++vp.first)
	{
		std::cout << index[*vp.first] <<  " ";
	}
	std::cout << std::endl;

	
#ifdef USE_QTGUI
	if (innerViewer) innerViewer->update();
	osgView->autoResize();
	osgView->frame();
#endif


}




