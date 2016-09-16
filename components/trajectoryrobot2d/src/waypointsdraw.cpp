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

#include "waypointsdraw.h"

WaypointsDraw::WaypointsDraw()
{}

WaypointsDraw::~WaypointsDraw()
{}

bool WaypointsDraw::draw(WayPoints &road, InnerViewer *viewer, const CurrentTarget &currentTarget)
{
	QMutexLocker ml(viewer->innerViewer->mutex);
	clearDraw(viewer);
	if (road.size() == 0) return false;
	InnerModelDraw::addTransform_ignoreExisting(viewer->innerViewer, "road", "world");

	///////////////////
	//Draw all points
	//////////////////
	for (int i = 1; i < road.size(); i++)
	{
		WayPoint &w = road[i];
		WayPoint &wAnt = road[i - 1];
		QLine2D l(wAnt.pos, w.pos);
		QLine2D lp = l.getPerpendicularLineThroughPoint(QVec::vec2(w.pos.x(), w.pos.z()));
		QVec normal = lp.getNormalForOSGLineDraw();  //3D vector
		QVec tangent = road.getTangentAtClosestPoint().getNormalForOSGLineDraw();    
		QString item = "p_" + QString::number(i);
		InnerModelDraw::addTransform_ignoreExisting(viewer->innerViewer, item, "road");
		
		viewer->innerViewer->innerModel->updateTransformValues(item, w.pos.x(), 10, w.pos.z(), 0, 0, 0);

		if ((int) i == (int) road.getIndexOfCurrentPoint() + 1) 		{
			InnerModelDraw::drawLine(viewer->innerViewer, item + "_line", item, tangent, 600, 30, "#000055");
		}
		if (w.isVisible)
			InnerModelDraw::drawLine(viewer->innerViewer, item + "_point", item, normal, 500, 50, "#005500"); //TAKE NUMBER FROM ROBOT!!!
		else
			InnerModelDraw::drawLine(viewer->innerViewer, item + "_point", item, normal, 500, 50, "#550099");  //Morado
	}
	if (currentTarget.hasRotation() == true)    //Draws an arrow indicating final desired orientation
	{
		float rot = currentTarget.getRotation().y();
		WayPoint &w = road.last();
		QLine2D l(w.pos, w.pos + QVec::vec3((T) (500 * sin(rot)), 0, (T) (500 * cos(rot))));
		QVec ln = l.getNormalForOSGLineDraw();
		QString item = "p_" + QString::number(road.size() - 1);
		InnerModelDraw::drawLine(viewer->innerViewer, item + "_line", item, ln, 600, 30, "#400055");
	}
	return true;
}


void WaypointsDraw::clearDraw(InnerViewer *viewer)
{
	if (viewer->innerViewer->innerModel->getNode("road"))
		InnerModelDraw::removeNode(viewer->innerViewer, "road");
}
