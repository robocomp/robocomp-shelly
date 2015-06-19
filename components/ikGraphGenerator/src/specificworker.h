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

#include <iostream>
#include <deque>
#include <iterator>
#include <iostream>
#include <ostream>
#include <vector>
#include "boost/graph/adjacency_list.hpp"
#include "boost/graph/topological_sort.hpp"
#include <boost/property_map/property_map.hpp>
#include <boost/foreach.hpp>
#include <boost/graph/graph_utility.hpp>
#include <boost/graph/incremental_components.hpp>
#include <boost/graph/connected_components.hpp>
#include <boost/pending/disjoint_sets.hpp>
#include <boost/graph/graphviz.hpp>
#include <boost/property_map/dynamic_property_map.hpp> 
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/iteration_macros.hpp>
#include <nabo/nabo.h>
#include <innermodeldraw.h>


struct VertexPayload
{	
	QVec pose; // 3D
	std::map<std::string, float> configuration;
	std::size_t vertex_id;
	VertexPayload()
	{
	}
	VertexPayload(std::size_t i, const QVec &p, const std::map<std::string, float> &cfg)
	{
		vertex_id = i;
		configuration = cfg;
		pose = p;
	}
};	
struct EdgePayload
{
	float dist;
	EdgePayload()
	{
		dist = -1;
	}
	EdgePayload(float d)
	{
		dist = d;
	}
};

typedef boost::adjacency_list<boost::listS,boost::listS, boost::undirectedS, VertexPayload, EdgePayload, boost::listS> Graph;
typedef boost::graph_traits<Graph>::vertex_descriptor Vertex;
typedef boost::graph_traits<Graph>::edge_descriptor Edge;
typedef boost::graph_traits<Graph>::vertices_size_type VertexIndex;

typedef boost::graph_traits<Graph>::vertex_iterator VertexIterator;
typedef boost::component_index<VertexIndex> Components;
typedef std::vector<Graph::edge_descriptor> PathType;
typedef boost::graph_traits<Graph>::edge_iterator EdgeIterator;
typedef std::pair<EdgeIterator, EdgeIterator> EdgePair;
typedef std::pair<int, std::vector<Vertex> > CComponent;
typedef std::map<Vertex, int32_t> ComponentMap; 
typedef std::vector<CComponent> ConnectedComponents;



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

