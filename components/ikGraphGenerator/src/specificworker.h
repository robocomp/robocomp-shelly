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


#define MIN(X,Y) (((X) < (Y)) ? (X) : (Y))
#define MAX(X,Y) (((X) > (Y)) ? (X) : (Y))

class ConnectivityGraph
{
public:
	struct VertexData
	{
		bool valid;
		float pose[3];
		float poseElbow[3];
		std::vector < MotorGoalPositionList > configurations;
		std::size_t id;
		VertexData()
		{
			id = -1;
			valid = false;
			pose[0] = pose[1] = pose[2] = 0;
			poseElbow[0] = poseElbow[1] = poseElbow[2] = 0;
		}
		VertexData(std::size_t i, const float *p)
		{
			id = i;
			for (int j=0; j<3; j++)
				pose[j] = p[0];
		}
		void setPose(const float *p)
		{
			setPose(p[0], p[1], p[2]);
		}
		void setElbowPose(const float *p)
		{
			setElbowPose(p[0], p[1], p[2]);
		}
		void setPose(const float x, const float y, const float z)
		{
			pose[0] = x;
			pose[1] = y;
			pose[2] = z;
			valid = true;
		}
		void setElbowPose(const float x, const float y, const float z)
		{
			poseElbow[0] = x;
			poseElbow[1] = y;
			poseElbow[2] = z;
		}
		float distTo(const float *p)
		{
			if (valid)
				return sqrt( (p[0]-pose[0])*(p[0]-pose[0]) + (p[1]-pose[1])*(p[1]-pose[1]) + (p[2]-pose[2])*(p[2]-pose[2]) );
			return 1000000000000;
		}
	};
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

	ConnectivityGraph(char *path)
	{
		QFile(path);
		if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
			throw 1;

		QTextStream in(&file);
		while (!in.atEnd())
		{
			QString line = in.readLine();
			if (line[0] == 'V')
			{
				graph->addVertex(ConnectivityGraph::VertexData());
				QStringList parts = line.split("_")
				graph->vertices[graph->size()-1].id = parts[1].toInt();
				graph->vertices[graph->size()-1].setPose(     parts[2].toFloat(), parts[3].toFloat(), parts[4].toFloat());
				graph->vertices[graph->size()-1].setElbowPose(parts[5].toFloat(), parts[6].toFloat(), parts[7].toFloat());

				graph->vertices[graph->size()-1].configurations.resize(1);
				for (uint m=8; m<parts.size(); m++)
				{
					QStringList goal = parts[m].split(":");
					MotorGoalPosition mgp;
					mgp.position = goal[1].toFloat();
					mgp.maxSpeed = 2.;
					mgp.name = goal[0].toStdString();
					graph->vertices[graph->size()-1].configurations[0].push_back(mgp);
				}
			}
			else if (line[0] == 'E')
			{
				QStringList parts = line.split("_");
				int a = parts[0].toInt();
				int b = parts[1].toInt();
				float peso = parts[2].toFloat();
				graph->edges[a][b] = peso;
			}
			else
			{
				throw 2;
			}
		}
	}

	bool save(char *path)
	{
		QFile file(path);
		if (!file.open(QIODevice::WriteOnly | QIODevice::Text))
			return false;

		QTextStream out(&file);
		for (uint vid=0; vid<vertices.size(); vid++)
		{
			out << "V_";
			out << vertices[vid].id << "_";
			out << vertices[vid].pose[0]      << "_" << vertices[vid].pose[1]      << "_" << vertices[vid].pose[2] << "_";
			out << vertices[vid].poseElbow[0] << "_" << vertices[vid].poseElbow[1] << "_" << vertices[vid].poseElbow[2];
			if (vertices[vid].configurations.size()>0)
			{
				for (auto motor : vertices[vid].configurations[0])
				{
					out << "_" << QString::fromStdString(motor.name) << ":" << motor.position;
				}
			}
			out << "\n";
		}

		for (uint v1=0; v1<edges.size(); v1++)
		{
			for (uint v2=0; v2<edges[v1].size(); v2++)
			{
				out << v1 << "_" << v2 << "_" << edges[v1][v2] << "\n";
			}
		}
		return true;
	}

	void addVertex(const VertexData &v)
	{
		// Add vertex
		vertices.push_back(v);
		// Add edges for existing nodes
		for (uint32_t j=0;j<vertices.size()-1; j++)
		{
			edges[j].push_back(DJ_INFINITY);
		}
		// Add edges for the new node
		std::vector<float> eds;
		for (uint32_t j=0;j<vertices.size(); j++)
		{
			eds.push_back(DJ_INFINITY);
		}
		edges.push_back(eds);
	}

	int size()
	{
		return vertices.size();
	}





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
	bool goAndWait(int nodeId, MotorGoalPositionList &mpl, int &recursive);
	bool goAndWait(float x, float y, float z, int node, MotorGoalPositionList &mpl, int &recursive);
	void goAndWaitDirect(const MotorGoalPositionList &mpl);

	std::pair<float, float> xrange, yrange, zrange;

// 	int getRandomNodeClose(int &current, float &dist);
	MotorGoalPositionList centerConfiguration;

// 	float maxDist;
	ConnectivityGraph *graph;
	WorkerThread *workerThread;

#ifdef USE_QTGUI
	OsgView *osgView;
	InnerModelViewer *innerViewer;
	InnerModel *innerVisual;
#endif


};

#endif

