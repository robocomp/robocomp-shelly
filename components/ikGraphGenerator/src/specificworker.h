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

/**
       \brief
       @author authorname
*/

#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>
#include <innermodel/innermodel.h>

#ifdef USE_QTGUI
	#include <osgviewer/osgview.h>
	#include <innermodel/innermodelviewer.h>
	#include <innermodeldraw.h>
#endif

#include <nabo/nabo.h>
#include <innermodeldraw.h>

using namespace boost;


class ConnectivityGraph
{
public:
	
	ConnectivityGraph(int32_t size)
	{
		for (int32_t i=0;i<size; i++)
		{
			vertices.push_back(VertexData());
			std::vector<float> eds;
			for (int32_t j=0;j<size; j++)
			{
				eds.push_back(-1);
			}
			edges.push_back(eds);
		}
	}
	
	struct VertexData
	{
		float pose[3];
		std::vector < std::map < std::string, float > > configurations;
		std::size_t id;
		VertexData()
		{
			id = -1;
		}
		VertexData(std::size_t i, const float *p)
		{
			id = i;
			for (int j=0; j<3; j++)
				pose[j] = p[0];
		}
		void setPose(const float *p)
		{
			for (int j=0; j<3; j++)
				pose[j] = p[0];
		}
	};


	std::vector<VertexData> vertices;
	std::vector< std::vector< float > > edges;
}




class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(MapPrx& mprx);
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);


public slots:
	void compute();

private:
	Graph graph;
	InnerModel *innerModel;


#ifdef USE_QTGUI
	OsgView *osgView;
	InnerModelViewer *innerViewer;
	InnerModel *innerVisual;
#endif


};

#endif

