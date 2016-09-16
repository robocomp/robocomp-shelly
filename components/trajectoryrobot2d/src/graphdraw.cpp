/*
 * Copyright 2016 pbustos <email>
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 * 
 */

#include "graphdraw.h"

/**
 * @brief Draws the graph on a InnerModelViewer instance
 * 
 * @param innerViewer ...
 * @return bool
 */

GraphDraw::GraphDraw()
{}

GraphDraw::~GraphDraw()
{}

bool GraphDraw::draw(const PlannerPRM& planner, InnerViewer* viewer)
{
	QMutexLocker ml(viewer->innerViewer->mutex);
	cleanGraph(viewer);
	InnerModelDraw::addTransform(viewer->innerViewer, "graph", "world");

	//compute connected components
	ComponentMap componentMap;
	ConnectedComponents compList;
	planner.connectedComponents(componentMap, compList);

	QString item;
	int i=0;
	QStringList color;
	color << "#AA0000" << "#00AA00" << "#0000AA" << "#AA00AA" << "#AAAA00";
	int j=0;
	for(auto comp : compList)
	{
		QString c = color.at(j++%color.size());
		qDebug() << __FUNCTION__<< "color C" << c << comp.second.size();
		for(auto elem : comp.second)
		{
			item = "g_" + QString::number(i);
			QString  parentT = QString("g_") + QString::number(i);
			InnerModelDraw::addTransform(viewer->innerViewer, parentT, "graph");
			viewer->innerViewer->innerModel->updateTransformValues(parentT, planner.graph[elem].pose.x(), 10, planner.graph[elem].pose.z(), 0,0,0);
			InnerModelDraw::addPlane_ignoreExisting(viewer->innerViewer, item + "_plane", parentT, QVec::vec3(0,0,0), QVec::vec3(0,1,0),c, QVec::vec3(60,60,10));
			i++;
		}
	}

	i=0;
	BGL_FORALL_EDGES(e, planner.graph, Graph)
    {
		item = "ge_" + QString::number(i);
		QVec p1 = planner.graph[boost::source(e,planner.graph)].pose;
		QVec p2 = planner.graph[boost::target(e,planner.graph)].pose;
		QVec center = (p2-p1)/(T)2.f;

		QString  parentTE = QString("ge_") + QString::number(i);
		InnerModelDraw::addTransform(viewer->innerViewer, parentTE, "graph");
		viewer->innerViewer->innerModel->updateTransformValues(parentTE, p1.x()+center.x(), p1.y()+center.y(), p1.z()+center.z(), 0, QLine2D(p1,p2).getAngleWithZAxis()+M_PI/2, 0 );
		InnerModelDraw::addPlane_ignoreExisting(viewer->innerViewer, QString("ge_")+QString::number(i)+"_plane", parentTE, QVec::vec3(0,0,0), QVec::vec3(0,1,0), "#00A0A0", QVec::vec3((p1-p2).norm2(), 15 , 15));
		i++;
	}
	return true;
}

void GraphDraw::cleanGraph(InnerViewer *viewer)
{
  if (viewer->innerModel->getNode("graph"))
	 InnerModelDraw::removeNode(viewer->innerViewer, "graph");
}
