/*
 * Copyright 2014 <copyright holder> <email>
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

#include "plannerprm.h"

/**
 * @brief Probabilistic Road Map planner constructor. Computes and updates a free-path graph and trajectories on it. If no path can be found on graph, it calls RRT
 * 
 * @param innerModel_ ...
 * @param nPoints ...
 * @param neigh ...
 * @param parent ...
 */
PlannerPRM::PlannerPRM()
{	
}

void PlannerPRM::initialize(Sampler* sampler_, const RoboCompCommonBehavior::ParameterList &params)
{	
	sampler = sampler_;
	
	////////////////////////
	/// Initialize RRTplaner
	////////////////////////
	plannerRRT.initialize(sampler); 
	 
	
	////////////////////////
	/// Check if graph already exists
	////////////////////////
	if( QFile(graphFileName).exists())
	{
		qDebug() << __FUNCTION__ << "Graph file exits. Loading";
		readGraphFromFile(graphFileName);
	}
	else
	{
		try
		{
			nPointsInGraph = std::stoi(params.at("PlannerGraphPoints").value);
			nNeighboursInGraph = std::stoi(params.at("PlannerGraphNeighbours").value);
			maxDistToSearchmm = std::stof(params.at("PlannerGraphMaxDistanceToSearch").value);
			robotRadiusmm = std::stof(params.at("RobotRadius").value);
		}
		catch(...)
		{ qFatal("Planner-Initialize. Aborting. Some Planner graph parameters not found in config file"); }
		
		qDebug() << __FUNCTION__ << "No graph file found. Creating with " << nPointsInGraph << "nodes and " << nNeighboursInGraph << "neighboors";
		QList<QVec> pointList = sampler->sampleFreeSpaceR2(nPointsInGraph);
		
		if( pointList.size() < nNeighboursInGraph )
			qFatal("Planner-Initialize. Aborting. Could not find enough free points to build de graph"); 
		
		qDebug() << __FUNCTION__ << "Creating with " << nPointsInGraph << "nodes and " << nNeighboursInGraph << "neighboors";
    constructGraph(pointList, nNeighboursInGraph, maxDistToSearchmm, robotRadiusmm);  ///GET From IM ----------------------------------
		qDebug() << __FUNCTION__ << "Graph constructed with " << pointList.size() << "points";
		writeGraphToFile(graphFileName);
	}
	graphDirtyBit = true;

}
/**
 * @brief Main callable method. Computes a path for the robot given a target pose and a point to Innermodel. inner is used to synchronize the current copy of Innermodel.
 * This is done so the planner can ran as a thread in the future.
 * Should de extended to return the best path found given a certain time limit
 * @param target QVec for pose translation. Should be within world limits
 * @param inner Pointer to Innermodel
 * @return true if a path of at least two waypoints have been found. Path is written to class variable currentPath.
 * Additional chekcs on max and min lenght for the path should be applied
 */
bool PlannerPRM::computePath(QVec& target, InnerModel* innerModel)
{
	QVec robot = innerModel->transform("world","robot");
	QVec robotRotation = innerModel->getRotationMatrixTo("world", "robot").extractAnglesR_min();
	
 	qDebug() << __FUNCTION__ << "Starting planning with robot at:" << robot <<  "and target at:" << target;

	/////////////////////////////////////////////
	//If robot on obstacle we can¡t proceed
	/////////////////////////////////////////////
	if( std::get<bool>(sampler->checkRobotValidStateAtTarget(robot, robotRotation)) == false)
	{
		qDebug() << __FUNCTION__ << "Robot collides in origin. Aborting planner";  
		return false;
	}
	
	/////////////////////////////////////////////
	//If target on obstacle find a point close to it on the robot-target line. For now returns
	/////////////////////////////////////////////
	if( sampler->searchRobotValidStateCloseToTarget(target) == false )
	{
		qDebug() << __FUNCTION__ << "Robot collides in target. Aborting planner";  //Should search a next obs-free target
		return false;
	}
	
	////////////////////////////////////////////
	// PLanner uses another instance of InnerModel to plan so we resynchronize both before initiate planning
	////////////////////////////////////////////
	//innerModel->updateTransformValues("robot", robot.x(), robot.y(), robot.z(), robotRotation.x(), robotRotation.y(), robotRotation.z());

	//Clean former path
	currentPath.clear();
	
	/////////////////////////////////////////////
	//Check if the target is in "plain sight"
	/////////////////////////////////////////////
	if ( sampler->checkRobotValidDirectionToTargetOneShot( robot, target) )
	{
 		qDebug() << __FUNCTION__ << "Target on sight. Proceeding";
		currentPath << robot << target;
		return true;
	}
	qDebug() << __FUNCTION__ << "Target not on sight. Proceeding";
	
	/////////////////////////////////////////////
	//If not, search in KD-tree for closest points to origin and target
	/////////////////////////////////////////////
	Vertex robotVertex, targetVertex;
	searchClosestPoints(robot, target, robotVertex, targetVertex);
	qDebug() << __FUNCTION__ << "Computing closest point to robot in graph";
	qDebug() << __FUNCTION__ << "Closest point to robot:" << graph[robotVertex].pose;
	qDebug() << __FUNCTION__ << "Closest point to targett:" << graph[targetVertex].pose;
	
	/////////////////////////////////////////////
	// Check if the closest point in the graph is in "plain sight"
	// Otherwise call the planner
	/////////////////////////////////////////////
	if ( sampler->checkRobotValidDirectionToTargetOneShot( robot, graph[robotVertex].pose) )
	{
 		qDebug() << __FUNCTION__ << "Closest point to robot in graph is ON sight";
		currentPath << robot << graph[robotVertex].pose;
	}
	else
	{
		qDebug() << __FUNCTION__ << "Closest point to robot in graph is NOT on sight. Proceeding to search";
		/////////////////////////////////////////////
		//Obtain a free path from [robot] to [robotVertex] using RRTConnect. Return if fail.
		/////////////////////////////////////////////
		qDebug() << __FUNCTION__ << "Searching with RRT from ROBOT to closest point in graph (CPG)";
		QList<QVec> path;
		if (planWithRRT(robot, graph[robotVertex].pose, path) )
		{
			path.removeLast();
			currentPath << path;
			qDebug() << __FUNCTION__ << "RRTConnect succeeded finding a path from ROBOT to CPG with a length of " << currentPath.size() << "steps.";
		}
		else
		{
			qDebug() << __FUNCTION__ << "ERROR: Path from ROBOT to CPG NOT FOUND:";
			return false;
		}
	}

	/////////////////////////////////////////////
	//Now we are in the graph
	//Search in graph a minimun path using Dijkstra algorithm
	/////////////////////////////////////////////
	if( robotVertex != targetVertex )  //Same node for both. We should skip searchGraph
	{
		std::vector<Vertex> vertexPath;
		if ( searchGraph(robotVertex, targetVertex, vertexPath) == true)
			for( Vertex v : vertexPath )
				currentPath.append(graph[v].pose);
		else //No path found. The path does not connect to target's closest point in graph
		{
			qDebug() << __FUNCTION__ << "No path through graph. Starting RRTConnect from " << graph[robotVertex].pose << " to" <<  target;
		  QList<QVec> path;	
		  if ( planWithRRT(graph[robotVertex].pose, target, path) )
			{
				path.removeFirst();
				currentPath += path;
	  		qDebug() << __FUNCTION__ << "RRTConnect succeeded finding a path from CPG to CPT with a length of " << currentPath.size() << "steps.";
				qDebug() << __FUNCTION__ << "RRTConnect path form CPG to CPT:" <<  path;
			}
			else
				return false;
		}
	}
	else	//add the only node
		currentPath += graph[robotVertex].pose;

	
	///////////////////////////////////////////////////////////
	// Check if the closest point in the graph to the TARGET is in "plain sight".
	// Otherwise call the planner
	////////////////////////////////////////////////////////////7
	if ( sampler->checkRobotValidDirectionToTargetOneShot( graph[targetVertex].pose, target ))
	{
 		qDebug() << __FUNCTION__ << "Closest point to target in graph ON sight";
		currentPath << target << graph[targetVertex].pose;
	}
	else
	{
		qDebug() << __FUNCTION__ << "Closest point to target in graph NOT on sight. Proceeding to search";
		
		/////////////////////////////////////////////
		//Obtain a free path from target to targetVertex using RRTConnect.
		/////////////////////////////////////////////
		qDebug() << __FUNCTION__ << "Searching with RRT from closest point in graph (CPG) to TARGET";
		QList<QVec> path;
		if (planWithRRT(graph[robotVertex].pose, target, path) == true)
		{
				path.removeLast();
				currentPath << path;
		}
		else
		{
				qDebug() << __FUNCTION__ << "ERROR: Path from TARGET to CPG NOT FOUND using RRT planner. Can't do better";
				return false;
		}
	}

	/////////////////////////////////////
	// Smoothing
	/////////////////////////////////////
	pathSmoother(currentPath);
	return true;
}

////////////////////////////////////////////////////////////////////////
/// One query planner to be used when no path found un PRM
////////////////////////////////////////////////////////////////////////


bool PlannerPRM::planWithRRT(const QVec &origin, const QVec &target, QList<QVec> &path)
{
 	qDebug() << __FUNCTION__ << "RRTConnect start...";
	const float diffV = (origin-target).norm2();
	
	if (diffV < 200) // HALF ROBOT RADIOUS:::::::::::::::::::::::::::::::::::::
	{
		qDebug() << __FUNCTION__ << "Origin and target too close. Returning trivial path";
		path << origin << target;
		return true;
	}

	QVec point;
	if (sampler->checkRobotValidDirectionToTargetOneShot( origin, target ))
	{
		qDebug() << __FUNCTION__ << "Found target directly in line of sight. Returning trivial path";
		path << origin << target;
		return true;
	}

	qDebug() << __FUNCTION__ << "Calling Full Power of RRTConnect OMPL planner. This may take a while";
	
	if (plannerRRT.computePath(origin, target, 600) == true)  //max time but it would be better an asynchronous versin
	{
		path += plannerRRT.getPath();
		return true;
	}
	else
		return false;
}

//////////////////////////////////////////////////////////////////////////////////////////////////
/// Graph operations
/////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * @brief Searches in the graph closest points to origin and target
 * 
 * @param origin ...
 * @param target ...
 * @param originVertex to return selected vertex
 * @param targetVertex ...
 * @return void
 */
void PlannerPRM::searchClosestPoints(const QVec& origin, const QVec& target, Vertex& originVertex, Vertex& targetVertex)
{
	qDebug() << __FUNCTION__ << "Searching from " << origin << "and " << target;

	//prepare the query
	Eigen::MatrixXi indices;
	Eigen::MatrixXf distsTo;
	Eigen::MatrixXf query(3,2);
	indices.resize(1, query.cols());
	distsTo.resize(1, query.cols());
	query(0,0) = origin.x();query(1,0) = origin.y();query(2,0) = origin.z();
	query(0,1) = target.x();query(1,1) = target.y();query(2,1) = target.z();

	nabo->knn(query, indices, distsTo, 1);

	originVertex = vertexMap.value(indices(0,0));
	targetVertex = vertexMap.value(indices(0,1));

 	qDebug() << __FUNCTION__ << "Closest point to origin is at" << data(0,indices(0,0)) << data(1,indices(0,0)) << data(2,indices(0,0)) << " and corresponds to " << graph[originVertex].pose;
 	qDebug() << __FUNCTION__ << "Closest point to target is at" << data(0,indices(0,1)) << data(1,indices(0,1)) << data(2,indices(0,1)) << " and corresponds to " << graph[targetVertex].pose;

}

/**
 * @brief Search the graph for a minimun path between originVertex and targetVertex. Returns the path
 *
 * @param originVertex ...
 * @param targetVertex ...
 * @param vertexPath std::vector of Vertex with the path
 * @return bool
 */
bool PlannerPRM::searchGraph(const Vertex &originVertex, const Vertex &targetVertex, std::vector<Vertex> &vertexPath)
{
	
	qDebug() << __FUNCTION__ << "Searching the graph between " << graph[originVertex].pose << "and " << graph[targetVertex].pose;
	
	// Create things for Dijkstra
	std::vector<Vertex> predecessors(boost::num_vertices(graph)); // To store parents
	std::vector<float> distances(boost::num_vertices(graph));     // To store distances

	//Create a vertex_index property map, since VertexList is listS
	typedef std::map<Vertex, size_t>IndexMap;
	IndexMap indexMap;
	boost::associative_property_map<IndexMap> propmapIndex(indexMap);
	
  //indexing the vertices
	int i=0;
	BGL_FORALL_VERTICES(v, graph, Graph)
		boost::put(propmapIndex, v, i++);

	auto predecessorMap = boost::make_iterator_property_map(&predecessors[0], propmapIndex);
	auto distanceMap = boost::make_iterator_property_map(&distances[0], propmapIndex);

	boost::dijkstra_shortest_paths(graph, originVertex, boost::weight_map(boost::get(&EdgePayload::dist, graph))
															.vertex_index_map(propmapIndex)
															.predecessor_map(predecessorMap)
 															.distance_map(distanceMap));

	//////////////////////////
	// Extract a shortest path
	//////////////////////////
	PathType path;
	Vertex v = targetVertex;

	//////////////////////////
	// Start by setting 'u' to the destintaion node's predecessor   |||// Keep tracking the path until we get to the source
	/////////////////////////
	for( Vertex u = predecessorMap[v]; u != v; v = u, u = predecessorMap[v]) // Set the current vertex to the current predecessor, and the predecessor to one level up
	{
		std::pair<Graph::edge_descriptor, bool> edgePair = boost::edge(u, v, graph);
		Graph::edge_descriptor edge = edgePair.first;
		path.push_back( edge );
	}

 	qDebug() << __FUNCTION__ << " Path found with length: " << path.size() << "steps and length " << 	distanceMap[targetVertex];
;
	Vertex lastVertex;
	if(path.size() > 0)
	{
		vertexPath.clear();
		for(PathType::reverse_iterator pathIterator = path.rbegin(); pathIterator != path.rend(); ++pathIterator)
		{
			vertexPath.push_back(boost::source(*pathIterator, graph));
			lastVertex = boost::target(*pathIterator, graph);
		}
		vertexPath.push_back(lastVertex);
		return true;
	}
	else
	{
		qDebug() << "Path no found between nodes";
		return false;
	}
}

//ConnectedComponents PlannerPRM::connectedComponents(ComponentMap &componentMap, bool print)
void PlannerPRM::connectedComponents(ComponentMap &componentMap, ConnectedComponents &compList, bool print) const
{
	//qDebug() << __FUNCTION__;

	//create a vertex_index property map, since VertexList is listS and the graph does not have a "natural" index
 	typedef std::map<Vertex, size_t> IMap;
 	typedef boost::associative_property_map<IMap> IndexMap;
	IMap indexMap;
	IndexMap index( indexMap );

 	//indexing the vertices
 	int i=0;
 	BGL_FORALL_VERTICES(v, graph, Graph)
 		boost::put(index, v, i++);

	std::vector<int> components(boost::num_vertices(graph));
	boost::iterator_property_map< std::vector<int>::iterator, IndexMap> compIterMap( components.begin(), index );

  int numComps = boost::connected_components(graph, compIterMap, vertex_index_map(index));
	
	//Create a nice data structure to return info
	compList.resize(numComps);
	for(auto it : compList)
		it = std::make_pair(0, std::vector<Vertex>());

	BGL_FORALL_VERTICES(v, graph, Graph)
	{
		compList[get(compIterMap, v)].second.push_back( v );
		compList[get(compIterMap, v)].first++;
		componentMap.insert( std::pair<Vertex,int32_t>( v, get(compIterMap, v )));
	}

	if( print )
		for(auto it : compList)
		{
			qDebug() << __FUNCTION__ << "Num comps: " << it.first ;
			for(auto itt : it.second)
				std::cout <<  graph[itt].vertex_id << " " ;
			std::cout << std::endl;
		}
}

int32_t PlannerPRM::constructGraph(const QList< QVec >& pointList, uint neighboors, float max_distance_to_check, uint RobotRadius)
{

	qDebug() << __FUNCTION__ << "Constructing graph with " << pointList.size() << " new points and " <<  neighboors << "neighboors";

	int ROBOT_SIZE_SQR = (int)pow(RobotRadius,2); //mm  OBTAIN FROM ROBOT'S BOUNDING BOX!!!!!
	float MAX_DISTANTE_TO_CHECK_SQR = max_distance_to_check * max_distance_to_check;

	/////////////////////////////////////////////////////////////////////////////////////////
	//Expand the matrix with new points, insert new vertices and update de <int,vertex> map
	////////////////////////////////////////////////////////////////////////////////////////
	int lastCol = data.cols();
	data.conservativeResize(3, data.cols() + pointList.size() );

	//////////////////////////
	//Create the query matrix
	//////////////////////////
	Eigen::MatrixXf query(data.rows(), pointList.size());
	//qDebug() << __FUNCTION__ << "Data matrix resized" << data.rows() << data.cols() << "lastCol" << lastCol;

	//////////////////////////
	//Add points to data matrix and to graph
	//////////////////////////
	for(int i=lastCol, k=0; i< data.cols(); i++, k++)
	{
		data(0,i) = pointList[k].x();
		data(1,i) = pointList[k].y();
		data(2,i) = pointList[k].z();
		query(0,k) = pointList[k].x();
		query(1,k) = pointList[k].y();
		query(2,k) = pointList[k].z();
		Vertex vertex = boost::add_vertex(graph);
		vertexMap.insert(i,vertex);   			//Hash<row,Vertex> to go back and forth between data matrix and graph
		graph[vertex].pose = QVec::vec3(data(0,i), data(1,i), data(2,i));
		graph[vertex].vertex_id = i;
		//qDebug() << __FUNCTION__ << "Inserted vertex " << i;
	}

	qDebug() << __FUNCTION__ << "Computing KdTree...";
	/////////////////////
	//Compute KdTree
	/////////////////////
	try{ nabo = Nabo::NNSearchF::createKDTreeTreeHeap(data); }
	catch(std::exception &ex){ std::cout << ex.what() << std::endl; qFatal("fary en Nabo");}

	////////////////////////////////////////////////////
	//Query KDTree for sorted distances to NEIGHBOORS neighbours
	//////////////////////////////////////////////////////
	Eigen::MatrixXi indices(neighboors, pointList.size() );
	Eigen::MatrixXf distsTo(neighboors, pointList.size() );
	if( neighboors >= data.cols())
		neighboors = data.cols()-1;
	nabo->knn(query, indices, distsTo, neighboors, 0, Nabo::NNSearchF::SORT_RESULTS);

	/////////////////////////
	//Connect the new points
	//////////////////////////
	qDebug() << __FUNCTION__ << "Connecting points...";
	bool reachEnd;
	int32_t numExit = 0;
	
	// First go through new points
	for(int i=lastCol, j=0; i<lastCol+pointList.size(); i++, j++)
	{
		Vertex vertexNew = vertexMap.value(i);
		//for each new point locate a set of neighbours sorted by increasing distance
		for(uint k=0; k<neighboors; k++)
		{
			int indKI = indices(k,j);
			Vertex vertexOld = vertexMap.value(indKI);
			//qDebug() << __FUNCTION__ << "Trying" << i << j <<k << "dist" << distsTo(k,j) << "indKI" << indKI;
			//check distance to be lower that max limit and greater than robot size and try to connect both dots
			if( (distsTo(k,j) < MAX_DISTANTE_TO_CHECK_SQR) and (distsTo(k,j) > ROBOT_SIZE_SQR )) 
			{
				QVec lastPoint;
				const QVec iiv = QVec::vec3(data(0,i), data(1,i), data(2,i));
				const QVec ijdie = QVec::vec3(data(0,indKI), data(1,indKI), data(2,indKI));
				reachEnd = sampler->checkRobotValidDirectionToTargetOneShot(iiv, ijdie);
				//qDebug() << __FUNCTION__ << "i" << i << "to k" << k << indKI << "at dist " << distsTo(k,j) << "reaches the end:" << reachEnd;

				// If there is a free path to the neighboor, insert it in the graph
				if( reachEnd == true)
				{
					//Compute connected comps to check it tne vertices belong to the same comp.
					//ComponentMap componentMap;
					//ConnectedComponents components = connectedComponents(componentMap);
					ComponentMap componentMap;
					ConnectedComponents compList;
					connectedComponents(componentMap, compList);

					//Reject connecting elements belonging to the same connected component and already existing edges between vertices
					if( (boost::edge(vertexOld,vertexNew,graph).second == false )
							and (componentMap.at(vertexOld) != componentMap.at(vertexNew)) )
					{
						EdgePayload edge;
						edge.dist = (QVec(graph[vertexNew].pose) - QVec(graph[vertexOld].pose)).norm2();
						boost::add_edge(vertexOld, vertexNew, edge, graph);
						numExit++;
					}
				}
			}
		}
	}
	
	qDebug() << __FUNCTION__ << "Cleaning up...";
	if( numExit > 0 ) 
		graphDirtyBit = true;
	removeSmallComponents(3); //This will rebuild data hash and indices
	if( graphDirtyBit == true)
		rebuildExternalData();
	return numExit;
}

/**
 * @brief Selects the two biggest connected comps and try to connect their closest points. The number of vertices does not change
 * @return void
 */
bool PlannerPRM::connectIsolatedComponents(int32_t &numConnections)
{
	//compute connected components
	ComponentMap componentMap;
	ConnectedComponents compList;
	connectedComponents(componentMap, compList);

	//If only one component, ntohing to do here
	if( compList.size() < 2)
		return false;

	//Sort the compList by size
	std::sort( compList.begin(), compList.end(),
			   []( CComponent a, CComponent b){ return a.first > b.first; });

	//Recover the two first indexes
	uint32_t largestSize  = compList.front().first;
	uint32_t largest2Size = (*(compList.cbegin() + 1)).first;

	// Compute closest points between both comps using KdTree.
	// Fist, build a Matrix with the elements of the largest to create a KdTree

	// qDebug() << __FUNCTION__ << "Sizes: " << largestSize << largest2Size;
	const uint32_t neighboors = 1;							//K=1 Only the closest one
	Eigen::MatrixXf dataL( 3 , largestSize );   //Watch R3 dependant
	Eigen::MatrixXf queryL(3 , largest2Size );
	Eigen::MatrixXi indicesL(neighboors, largest2Size );
	Eigen::MatrixXf distsToL(neighboors, largest2Size );

 	QHash<uint32_t,Vertex> vertexMapL, vertexMapLL;
	uint32_t i = 0;
	for( auto it : compList[0].second)
	{
		dataL(0,i) = graph[it].pose.x();
		dataL(1,i) = graph[it].pose.y();
		dataL(2,i) = graph[it].pose.z();
		vertexMapL.insert(i,it);
		i++;
		//qDebug() << graph[it].pose;
	}

	//build the query matrix with the second comp
	i=0;
	for( auto it : compList[1].second)
	{
		queryL(0,i) = graph[it].pose.x();
		queryL(1,i) = graph[it].pose.y();
		queryL(2,i) = graph[it].pose.z();
		vertexMapLL.insert(i,it);
		i++;
	}

	// Build the KdTree is valid here.
	Nabo::NNSearchF *naboL = Nabo::NNSearchF::createKDTreeTreeHeap(dataL);

	// Query for sorted distances from one elements in C2  to elements in C1
	naboL->knn(queryL, indicesL, distsToL, neighboors, 0, 0);

	/////////////////////////////////
	// Try to connect elements from both components
	/////////////////////////////////
	int nExit = 0;
	for (uint i=0; i<largest2Size; i++)
	{
		Vertex vv = vertexMapLL.value(i);
		Vertex v = vertexMapL.value(indicesL(0,i));
		
		if( sampler->checkRobotValidDirectionToTargetOneShot( graph[v].pose , graph[vv].pose) )
		{
			EdgePayload edge;
			edge.dist = distsToL(0,i);
			boost::add_edge(v, vv, edge, graph);
			nExit++;
			break;			//connected!
		}
	}
	
	numConnections = nExit;
	if( numConnections > 0)
	{ 
		rebuildExternalData();
	}

	return true;
}

/**
 * @brief Remove all small components in the graph with a size smaller thatn minSize.
 *
 * @param minSize min number of vertices in a conn. compo to be removed.
 * @return number of componets removed
 */
int32_t PlannerPRM::removeSmallComponents(int32_t minSize)
{
	//qDebug() << __FUNCTION__;
	
	ComponentMap componentMap;
	ConnectedComponents compList;
  connectedComponents(componentMap, compList);

	uint32_t nc = 0;
 	for( auto p : compList )
	{
		if( p.first < minSize )
		{
			for( auto q : p.second )
			{
				boost::clear_vertex( q, graph);
				boost::remove_vertex(q, graph);
			}
			nc++;
		}
	}
	qDebug() << __FUNCTION__  << "Graph after removing has " << boost::num_vertices(graph) << " vertices and" << compList.size() << "components" ;

	if( nc > 0 )
	{
		rebuildExternalData();
	}
	return nc;
}

/**
 * @brief Connect nearby components to facilitate the creation of loops
 *
 * @return number of edges created
 */
int32_t PlannerPRM::connectCloseElements( float thoseCloserThan)
{
	qDebug() << __FUNCTION__;
		
	//libNabo uses squared dist
	thoseCloserThan = thoseCloserThan*thoseCloserThan;

	// We use the KDTree structure to compute distances
	Eigen::MatrixXi indices(1, data.cols() );
	Eigen::MatrixXf distsTo(1, data.cols() );

	nabo->knn(data, indices, distsTo, 1, 0, Nabo::NNSearchF::SORT_RESULTS);
	
	int nExit = 0;
	
	for(int i=0; i<data.cols(); i++)
		if( distsTo(0,i) < thoseCloserThan )
		{
			Vertex v = vertexMap.value(indices(0,i));
			Vertex vv = vertexMap.value(i);
			if( sampler->checkRobotValidDirectionToTargetOneShot( graph[v].pose, graph[vv].pose) )
			 {
					EdgePayload edge;
					edge.dist = distsTo(0,i);
					boost::add_edge(v, vv, edge, graph);
					nExit++;
			 }
		}

	if( nExit > 0 )
	{
		rebuildExternalData();
	}
	qDebug() << __FUNCTION__ << "Connected " << nExit << "pairs of nodes";
	return nExit;
}

////////////////////////////////////////////////////////////////////////
/// GRaph UTILITIES
////////////////////////////////////////////////////////////////////////

/**
 * @brief Rebuilds de data matrix used for KDtree and the QHash that links Vertex to data indices
 *
 * @return bool
 */
bool PlannerPRM::rebuildExternalData()
{
	//get size of graph
	int nPoints = boost::num_vertices(graph);
	graphDirtyBit = true;

	if( nPoints == 0 )
	{
		// qDebug() << __FUNCTION__ << "Empty graph. Returning";
		return false;
	}

	//Expand the matrix
	data.resize( 3, nPoints );

	//Clean the vertex-to-data hash
	vertexMap.clear();

	int i=0;
	BGL_FORALL_VERTICES(v, graph, Graph)
	{
		data(0,i) = graph[v].pose.x();
		data(1,i) = graph[v].pose.y();
		data(2,i) = graph[v].pose.z();
		vertexMap.insert(i,v);
		i++;
	}

	//Compute KdTre
	if(nabo != nullptr)
		delete nabo;
	nabo = Nabo::NNSearchF::createKDTreeTreeHeap(data);

	//Ready to go!!
	graphDirtyBit = false;
	return true;
}

void PlannerPRM::writeGraphToFile(const QString &fileName)
{
	std::filebuf fb;
	//Check if empty default name is used. If so use the class variable
	if( fileName == "")
		fb.open (graphFileName.toStdString(),std::ios::out);
	else
		fb.open(fileName.toStdString(),std::ios::out);		
	std::ostream stream(&fb);
	
	////////////////////////////////
	// Create a vertex_index property map, since VertexList is listS and the graph does not have a "natural" index
	////////////////////////////////
 	typedef std::map<Vertex, size_t> IMap;
 	typedef boost::associative_property_map<IMap> IndexMap;
	IMap indexMap;
	IndexMap index( indexMap );

	///////////////////////
 	//Indexing the vertices
	////////////////////////
 	BGL_FORALL_VERTICES(v, graph, Graph)
 		boost::put(index, v, graph[v].vertex_id);

 	std::vector<int> components(boost::num_vertices(graph));
 	boost::iterator_property_map< std::vector<int>::iterator, IndexMap> compIterMap( components.begin(), index );

  	boost::write_graphviz(stream, graph, make_vertex_writer(boost::get(&VertexPayload::vertex_id, graph),boost::get(&VertexPayload::pose, graph)),
  										 make_edge_writer(boost::get(&EdgePayload::dist, graph)), boost::default_writer(), index);
}

/**
 * @brief Reads and existing graph from file
 * 
 * @param name ...
 * @return void
 */
void PlannerPRM::readGraphFromFile(QString name)
{
 	std::ifstream fin(name.toStdString().c_str());

	boost::dynamic_properties dynamicProperties;
	dynamicProperties.property("Index", boost::get(&VertexPayload::vertex_id, graph));
  dynamicProperties.property("Pose", boost::get(&VertexPayload::pose, graph));
  dynamicProperties.property("Distance", boost::get(&EdgePayload::dist, graph));
	
 	try{	boost::read_graphviz(fin, graph, dynamicProperties, "Index" );   }
  catch (std::exception& e){ std::cout << e.what() << std::endl; }
  
	data.resize(3,boost::num_vertices(graph));		//ONLY 3D POINTS SO FAR
 	int i=0;

	BGL_FORALL_VERTICES(v, graph, Graph)
    {
		data(0,i) = graph[v].pose.x();
		data(1,i) = graph[v].pose.y();
		data(2,i) = graph[v].pose.z();
		vertexMap.insert(i,v);
		i++;
	}
	nabo = Nabo::NNSearchF::createKDTreeTreeHeap(data);

	ComponentMap componentMap;
	ConnectedComponents compList;
  connectedComponents(componentMap, compList);

	qDebug() << __FUNCTION__ << "Graph size" << boost::num_vertices(graph) << "Nº of connected components" << compList.size();
}


////////////////////////////////////////////////////////////////////////
/// LEARNERS
////////////////////////////////////////////////////////////////////////

bool PlannerPRM::learnForAWhile(uint maxGraphNodes, bool print, bool save)
{
	static int verticesAnt=0, edgesAnt=0, compsAnt=0;
	static bool firstTime = true;
	
	/////////////////////////
	// Check Graph health
	//////////////////////////
	ComponentMap componentMap;
	ConnectedComponents compList;
	connectedComponents(componentMap, compList);
	int nComps = compList.size();
	
	if( print )
	{
		qDebug() << __FUNCTION__ << "-------------------------Graph Health----------------------------------------";
		qDebug() << "		Vertices:" << boost::num_vertices(graph);
		qDebug() << "		Vertices variation:" << boost::num_vertices(graph) - verticesAnt;
		qDebug() <<	"		Edges:" << boost::num_edges(graph);
		qDebug() << "		Edges variation:" << boost::num_edges(graph) - edgesAnt;
		qDebug() << "		Components:" << nComps;
		qDebug() << "		Components variation:" << nComps - compsAnt;
		QString cls;
		for(auto c : compList)
			cls += QString::number(c.second.size()) + " ";
		qDebug() << "		Components sizes:" << cls;	
		qDebug() << __FUNCTION__ << "-------------------------END of Graph Health-----------------------------------------";
	}
	compsAnt = nComps;
	verticesAnt = boost::num_vertices(graph);
	edgesAnt = boost::num_edges(graph);
	
	if (nComps == 1 and boost::num_vertices(graph) >= maxGraphNodes )
	{
		if( firstTime )
		{
			if( connectCloseElements(300) > 0 )
				if( save ) writeGraphToFile();
			firstTime = false;		
			return true;
		}
		return false;
	}
	
	rInfo("");
	
	/////////////////////////
	// First try to connect everything
	//////////////////////////
	if (nComps > 1)
	{
		int connected = connectIsolatedComponents(connected);
		qDebug() << __FUNCTION__ << "After CONNECTING comps there are" << connected << "components";
		if( save ) writeGraphToFile();
		return true;
	}
	
	////////////////////////////////////////////
	// Otherwise expand the Graph to remote zones
	///////////////////////////////////////////
	QList<QVec> points;
	int32_t pointsAdded;
	points = sampler->sampleFreeSpaceR2( 5 );  			//THE sampler could be more "smart" to bias zones around less visited nodes in the graph
	pointsAdded = constructGraph(points, nNeighboursInGraph, maxDistToSearchmm, robotRadiusmm);
	qDebug() << __FUNCTION__ << "After EXPANDING the graph has " << pointsAdded << "new nodes";
	if( save ) writeGraphToFile();
	return true;
}



/**
 * @brief Gets a fresh new path driven by the robot and adds a subsample of the points to the graph
 *
 * @param path ...
 * @return void
 */
bool PlannerPRM::learnPath(const QList< QVec >& path)
{
 	qDebug() << __FUNCTION__ << "Learning an intial path of" << path.size() << " points";

//	const int nWays=10;   //Max number of point to be inserted
	const int MIN_STEP = 350;

	if( path.size() < 2)
		return false;

	QList<QVec> sList;
	float dist =0;

	//Trim the path to a fixed number of waypoints
	for(int i=0; i<path.size()-1; i++)
	{
		dist += (path[i]-path[i+1]).norm2();
		if( dist > MIN_STEP)
		{
			sList << path[i];
			dist = 0;
		}
	}

 	qDebug() << __FUNCTION__ << "Learning with a shortened path of" << sList.size() << " points";
	constructGraph( sList, 10, 100, 10);
	return true;
}

////////////////////////////////////////////////////////////////////////
/// SMOOTHERS
////////////////////////////////////////////////////////////////////////
bool PlannerPRM::pathSmoother(QList<QVec> & pointList)
{
	currentSmoothedPath.clear();
	qDebug() << __FUNCTION__ << "number of points " << pointList.size();
	smoothPath(pointList);
	qDebug() << __FUNCTION__ << "number of points " << currentSmoothedPath.size();
	
	pointList = currentSmoothedPath;
	return true;
}

/**
 * @brief Fast recursive smoother that takes a list of poses and returns a safe shorter path free of collisions.
 *
 * @param list List of poses comprising the path
 * @return void
 */
void PlannerPRM::smoothPath( const QList<QVec> & list)
{
	if ( sampler->checkRobotValidDirectionToTargetOneShot( list.first(), list.last()) )
	{
		if(currentSmoothedPath.contains(list.first()) == false)
		  currentSmoothedPath.append(list.first());
		if(currentSmoothedPath.contains(list.last()) == false)
		  currentSmoothedPath.append(list.last());
		return;
	}
	else		//call again with the first half first and the second half later
	{
		if(list.size()>2)
		{
			smoothPath( list.mid(0,list.size()/2 +1));
		    smoothPath( list.mid( list.size()/2 , -1 ));
		}
	}
}

void PlannerPRM::smoothPathIter(QList<QVec> & list)
{
	//pick two ordered points and check free path between them
	int tam = list.size()*3;
	int i=0;
	while( i< tam and list.size() > 4)
	{
		int first = (int)QVec::uniformVector(1,0, list.size()-2)[0];
		int second = (int)QVec::uniformVector(1,first+2,list.size()-1)[0];

		QList<QVec>::iterator it = list.begin()+first;
		QList<QVec>::iterator itt = list.begin()+second;

		qDebug() << __LINE__;
		if ( sampler->checkRobotValidDirectionToTargetOneShot( *it, *itt ))
		{
			for(QList<QVec>::iterator ir = it + 1; ir != itt-1; ++ir)
			{
				list.erase(ir);
// 				qDebug() << __FUNCTION__ << "list size" << list.size();
			}
		}
		i++;
	}
}

void PlannerPRM::removeGraph(InnerModelViewer* innerViewer)
{
  data.resize(0,0);
  qDebug() << __FUNCTION__	<< "graph size" << boost::num_vertices(graph);
}

// /**
//  * @brief Creates a grpah if it does not exist
//  * 
//  * @return void
//  */
// void PlannerPRM::createGraph()
// {
// 	//remove this, sampler must be up  and running
// 	//sampler->initialize(innerModel, outerRegion, innerRegions);
// 	
// 	QList<QVec> pointList = sampler->sampleFreeSpaceR2(graphNumPoints);
// 	qDebug() << __FUNCTION__ << "constructing new graph";
// 	constructGraph(pointList,graphNeighPoints, 2500.f, 400);
// 	std::ofstream fout(fileName);
// 	writeGraphToStream(fout);
// }
// std::tuple<std::vector<Vertex>, QMap<u_int32_t, VertexIndex> > PlannerPRM::connectedComponents()
// {
// 	qDebug() << __FUNCTION__;
//
// 	//create a vertex_index property map, since VertexList is listS and the graph does not have a "natural" index
// 	typedef std::map<Vertex, size_t>IndexMap;
// 	IndexMap indexMap;
// 	boost::associative_property_map<IndexMap> propmapIndex(indexMap);
//     //indexing the vertices
// 	int i=0;
// 	BGL_FORALL_VERTICES(v, graph, Graph)
// 		boost::put(propmapIndex, v, i++);
//
// 	std::vector<VertexIndex> rank(boost::num_vertices(graph));
// 	std::vector<Vertex> parent(boost::num_vertices(graph));
// 	boost::disjoint_sets<VertexIndex*, Vertex*> ds(&rank[0], &parent[0]);
//
// 	boost::initialize_incremental_components(graph, ds);
// 	boost::incremental_components(graph, ds);
//
// 	QMap<u_int32_t, VertexIndex> sizeComps;
//  	Components components(parent.begin(),parent.end(), propmapIndex);
// //	Components components(propmapIndex);
//
//
// 	// get the property map for vertex indices
// //     typedef boost::property_map<Graph, boost::vertex_index_t>::type IndexMap;
// //     IndexMap index = boost::get(boost::vertex_index, graph);
//
// 	std::cout << "Number of connected components " << components.size() << std::endl;
// 	for(auto p : components)
// 	{
// 		std::cout << "Component " << p << " contains: " << std::endl;
// 		auto pair = components[p];
// 		uint32_t sizeC = 0;
// 		for( auto it = pair.first; it != pair.second; ++it)
// 		{	sizeC++;
// 		//	std::cout << graph[*it].index << " ";
// 		}
// 		//sizeComps.push_back(sizeC);
// 		sizeComps.insert(sizeC, p);
// 		std::cout << std::endl << "Total: " << sizeC << " nodes" << std::endl;
// 	}
// 	std::cout  << "--------------------------------" << std::endl;
//
// 	return std::make_tuple(parent, sizeComps);
//}
