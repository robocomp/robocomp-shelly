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

#define MAX_ERROR_IK 5.
#include <djk.h>

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
				eds.push_back(DJ_INFINITY);
			}
			edges.push_back(eds);
		}
	}

	int size()
	{
		return vertices.size();
	}


	struct VertexData
	{
		float pose[3];
		std::vector < MotorGoalPositionList > configurations;
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
		float distTo(const float *p)
		{
			return sqrt( (p[0]-pose[0])*(p[0]-pose[0]) + (p[1]-pose[1])*(p[1]-pose[1]) + (p[2]-pose[2])*(p[2]-pose[2]) );
		}
	};


	std::vector<VertexData> vertices;
	std::vector< std::vector< float > > edges;

	void add_edge(int a, int b, float dist)
	{
		edges[a][b] = dist;
		edges[b][a] = dist;
	}

	void add_configurationToNode(int node, MotorGoalPositionList gpl)
	{
		vertices[node].configurations.push_back(gpl);
	}

	int path(int source, int dest, std::vector<int> &path)
	{
		Dijkstra d = Dijkstra(&edges);
		d.calculateDistance(source);
		return d.go(dest, path);
	}
};

class WorkerThread : public QThread
{
Q_OBJECT
public:
	WorkerThread(void *data_)
	{
		data = data_;
	}

	void *data;

	void run();
};

class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(MapPrx& mprx);
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);


public slots:
	void computeHard();
	void compute();

private:
	void updateFrame(uint wait_usecs=0);
	bool goAndWait(int nodeId, MotorGoalPositionList &mpl, bool recursive=false);
	bool goAndWait(float x, float y, float z, int node, MotorGoalPositionList &mpl, bool recursive=false);
	void goAndWaitDirect(const MotorGoalPositionList &mpl);

	std::pair<float, float> xrange, yrange, zrange;

	int getRandomNodeClose(int &current, float &dist);

	MotorGoalPositionList centerConfiguration;


	float maxDist;
	ConnectivityGraph *graph;
	WorkerThread *workerThread;

#ifdef USE_QTGUI
	OsgView *osgView;
	InnerModelViewer *innerViewer;
	InnerModel *innerVisual;
#endif


};

#endif

