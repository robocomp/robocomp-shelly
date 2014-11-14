/*
    <one line to give the program's name and a brief idea of what it does.>
    Copyright (C) <year>  <name of author>

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/

#include "planner.h"
#include <boost/concept_check.hpp>

Planner::Planner(InnerModel *innerModel_, QObject *parent)
{
	innerModel = innerModel_;
	InnerModelTransform *t = innerModel->newTransform("baseT", "static", innerModel->getNode("floor"), 0, 0, 0, 0, 0, 0);
	innerModel->getNode("floor")->addChild(t);
	InnerModelMesh *m = innerModel->newMesh ("baseFake", t, "/home/robocomp/robocomp/files/osgModels/robex/robex.ive", 1000, 0, 0, 0, -181, 0, 0, 0);
	t->addChild(m);
  
	//Trees and planning
	arbol = new tree<QVec>;
	arbolGoal = new tree<QVec>;
	//p2Ant = innerModel->robotToWorld(QVec::vec3(0,0,-500));	
	PATH_FOUND = false;
	MAX_ITER = 5000;

}

/**
* \brief computes a path to a target from current position of robot. Is activated asynchronously
* @param target point in 3D space
*/
bool Planner::computePath(const QVec & target)
{
	//p2Ant = innerModel->robotToWorld(QVec::vec3(0,0,-500));	
	this->finalTarget = target;
	QVec currentTarget = finalTarget;
	//QVec robot = innerModel->robotInWorld();
	QVec robot = innerModel->getBaseCoordinates();
	QVec currentTargetGoal = robot;

	if(collisionDetector( this->finalTarget, innerModel ) == true)
	{
		qDebug() << "Planner::computePath - collides in target";
		PATH_FOUND = false;
		return false;
	}
	
	computeRandomSequence(this->finalTarget);	
 	arbol->clear();
	arbolGoal->clear();

	tree<QVec>::iterator top, nodeCurrentPos, auxNode, auxNodeGoal;
	tree<QVec>::iterator topGoal, nodeCurrentPosGoal;

	top = arbol->begin();
	topGoal =arbolGoal->begin();

	nodeCurrentPos = arbol->insert(top, robot);
	nodeCurrentPosGoal = arbolGoal->insert(topGoal, finalTarget);
   	CURRENT_LEAF_GOAL = nodeCurrentPosGoal;
   	CURRENT_LEAF = nodeCurrentPos;
   
	QVec res(3), resGoal(3);
	int i;
	bool reachEnd, success=false;
	int swapCounter = 0;

	for(i=0; i< MAX_ITER; i++)
	{
		//get new point form random sequence. First point is always the target
		currentTarget = chooseRandomPointInFreeSpace();
		// find closest point from random point to tree
		nodeCurrentPos = findClosestPointInTree( arbol, currentTarget);	
		
		//copy nodeCurrentPos to aux so nodeCurrentPos is not modified when calling trySegmentToTarget
		auxNode = nodeCurrentPos;
		// Local navigation. Find Bezier trajectory from closespoint in tree to random point from sequence. res is the closest point to currentTarget through Bezier route
		//res = tryBezierToTarget( *nodeCurrentPos, currentTarget, reachEnd, arbol, auxNode);		
		res = trySegmentToTarget( *nodeCurrentPos, currentTarget, reachEnd, arbol, auxNode);		
		// Can  only be equal if random point accidentally falls on tree
		if (equal(res, *nodeCurrentPos) == false) //Not achieved goal
			//appends res to tree at auxNode position
			CURRENT_LEAF = arbol->append_child( auxNode, res);			
		else
			CURRENT_LEAF=nodeCurrentPos;
		
		//search for closest point to res un goal tree
		nodeCurrentPosGoal = findClosestPointInTree( arbolGoal, res);		
		//Local navigation from nodeCurrentPosGoal to res using Bezier trajectories. Returns closest point in trajectory
		//resGoal = tryBezierToTarget( *nodeCurrentPosGoal, res, reachEnd, arbolGoal, nodeCurrentPosGoal);
		//copy nodeCurrentPos to aux so nodeCurrentPos is not modified when calling trySegmentToTarget
		auxNodeGoal = nodeCurrentPosGoal;
		resGoal = trySegmentToTarget( *nodeCurrentPosGoal, res, reachEnd, arbolGoal, auxNodeGoal);		

		// if origin and goal are not the same point, add resGoal to tree
 		if (equal(resGoal, *nodeCurrentPosGoal) == false)
			CURRENT_LEAF_GOAL = arbolGoal->append_child( auxNodeGoal, resGoal);		
		else
			CURRENT_LEAF_GOAL=nodeCurrentPosGoal;

			
		// if the result on localnavigation in both trees meet then we have found a way from robot to target
		if (equal(res, resGoal))
		{
			success = true;
			break;	
		}
		
		//Balance trees by swaping pointers when sizes differ.
		if ( (not success) and (arbol->size() > arbolGoal->size()) )
		{
			aux = arbolGoal;
			arbolGoal = arbol;
			arbol = aux;
			tree<QVec>::pre_order_iterator auxLeaf;
  			auxLeaf = CURRENT_LEAF_GOAL;
  			CURRENT_LEAF_GOAL = CURRENT_LEAF;
  			CURRENT_LEAF = auxLeaf;			//if even (par) then trees are in original position
			swapCounter++;
		}
	}
	
	if ( i < MAX_ITER)
	{
		
		PATH_FOUND = true;
		//If not even we do one more swap to restore trees to original position
		if( swapCounter%2 > 0)
		{
			aux = arbolGoal;
			arbolGoal = arbol;
			arbol = aux;
			tree<QVec>::pre_order_iterator auxLeaf;
			auxLeaf = CURRENT_LEAF_GOAL;
			CURRENT_LEAF_GOAL = CURRENT_LEAF;
			CURRENT_LEAF = auxLeaf;
		}
		currentPath = recoverPath(arbol,CURRENT_LEAF, arbolGoal,CURRENT_LEAF_GOAL);

		if (currentPath.size()>0)
		{
		  adaptiveSmoother(currentPath);
		}
		return true;
	}
	else
	{
		PATH_FOUND = false;
		return false;
	}
}

	tree<QVec>::iterator Planner::findClosestPointInTree( tree<QVec> * arb, const QVec & currentTarget){
	tree<QVec>::pre_order_iterator ini = arb->begin();
	tree<QVec>::pre_order_iterator end = arb->end();
	tree<QVec>::iterator nodeCurrentPos;
	float min = std::numeric_limits<float>::max();
	float dist;
	while(ini!=end)
	{
		dist = (currentTarget-(*ini)).norm2();
		if(  dist < min )
		{
			min = dist;
			nodeCurrentPos = ini;
		}
		++ini;
	}
   return nodeCurrentPos;
}

QVec Planner::chooseRandomPointInFreeSpace()
{
	QVec p(3),x(3);
	bool collision=true;

	//Inject goal position to bias the distribution
	if (ind > 0 and (float)qrand()*100/RAND_MAX > 80)
		return(this->finalTarget);
 
	do
	{
		p = QVec::vec3(fsX[ind], 0 , fsZ[ind]);
		//ind = (ind +1 )%(MAX_ITER*10);
		ind++;
		collision = collisionDetector( p, innerModel);	
	}
	while (collision and ind == fsX.size());
	if(ind > fsX.size()) qDebug()<<"no se encuentra posición libre";
		return p;	
}

void Planner::computeRandomSequence(const QVec & target)
{
 	QRect floorRect(-2500,2500,5000,-5000);
	
	this->fsX = QVec::uniformVector(MAX_ITER , floorRect.left()+1, floorRect.right()-1);
 	this->fsZ = QVec::uniformVector(MAX_ITER , floorRect.bottom()+1, floorRect.top()-1);
	this->fsX(0) = 	target.x();
 	this->fsZ(0) = 	target.z();
 	this->ind = 0;
}

bool Planner::equal(const QVec & p1, const QVec & p2)
{
	if( (p1-p2).norm2() < 50)
		return true;
	else
		return false;
}

bool Planner::collisionDetector( const QVec &point,  InnerModel *innerModel)
{
	//Check if the virtual robot collides with any obstacle
	innerModel->updateTransformValues("baseT", point(0), 0, point(2), 0, 0, 0);
	bool hit = false;
	for(int i=1; i<=16; i++)
	{
		QString caja("cajaMesh" + QString::number(i));
		if (innerModel->collide("baseFake", caja) )
		{
			hit = true;
			break;
		}
	}
	
// 	bool hit = innerModel->collide("baseFake", "cajaMesh1") or innerModel->collide("baseFake", "cajaMesh2") or 
// 						 innerModel->collide("baseFake", "cajaMesh3") or innerModel->collide("baseFake", "cajaMesh4") or
// 						 innerModel->collide("baseFake", "cajaMesh5") or innerModel->collide("baseFake", "cajaMesh6") or
// 						 innerModel->collide("baseFake", "cajaMesh7") or innerModel->collide("baseFake", "cajaMesh8") or
// 						 innerModel->collide("baseFake", "cajaMesh9") or innerModel->collide("baseFake", "cajaMesh10"); 
						 
	return hit;
	
}

bool Planner::isThereAnObstacleAtPosition(const QVec & pCenter, float rX, float rZ)
{
// 	//Construct robot contour from "point. Assume a square of side 2*radius + security offset
// 	QVec upLeft = cube->fromMetricToCell( pCenter - QVec::vec3(rX,0,rZ) );
// 	QVec downRight = cube->fromMetricToCell( pCenter + QVec::vec3(rX,0,rZ) );
// 	QVec cellPos;
// 	int x, z;
// 	
// 	float acumOccupancy=0.;
// 	
// 	//if inside limits
// 	if (upLeft.isEmpty()==false and downRight.isEmpty()==false)
// 	{
// 		//walk all inside cells checking occupancy
// 		for(int v=upLeft.x() ; v<=downRight.x() ; v++)
// 			for(int w=upLeft.z() ; w<=downRight.z() ; w++)
// 			{
// 				if(cube->getCellOccupancy(v,w)>cube->OCC_THRESHOLD)
// 					  acumOccupancy+=cube->getCellOccupancy(v,w);
// 				if(acumOccupancy > GLOBAL_OCCUPANCY_THRESHOLD)
// 					  return true;
// 			}
// 	}
// 	else
// 		return true;
// 	
	return false;  
}

QVec Planner::trySegmentToTarget(const QVec & origin , const QVec & target, bool & reachEnd, tree<QVec> * arbol , tree<QVec>::iterator & nodeCurrentPos)
{
	float stepSize = 100.f; //100 mms chunks
	uint nSteps = (uint)rint((origin - target).norm2() / stepSize);  
	float step;
	
	//if too close return target
	if (nSteps == 0) 
	{
		reachEnd = true;
		return target;
	}
	step = 1./nSteps;
	
	//go along visual ray connecting robot pose and target pos in world coordinates
	// l*robot + (1-r)*roiPos = 0
	
	QVec point(3), pointAnt(3);
	float landa = step;
	QVec pos(3), front(3);
	
	pointAnt=origin;
	for(uint i=1 ; i<=nSteps; i++)
	{
		// center of robot position
		point = (origin * (1-landa)) + (target * landa);
		
		//Collision detector
		if (collisionDetector( point, innerModel) == true)
		{
		  reachEnd = false;
		  return pointAnt;
		}
		else
		  if( arbol != NULL )
			nodeCurrentPos = arbol->append_child( nodeCurrentPos, point);			
		 
		landa = landa + step;
		pointAnt = point;
	}
	reachEnd= true;
	return target;
}


QList<QVec> Planner::recoverPath(tree<QVec> *arbol , const tree<QVec>::pre_order_iterator & _current, tree<QVec> *arbolGoal, const tree<QVec>::pre_order_iterator & _currentGoal)
{
	QList<QVec> path;
	tree<QVec>::pre_order_iterator begin, current;
	
	if ( arbol == NULL or arbol->is_valid(_current) == false )
		  return path; 
	
	begin = arbol->begin();
	current = _current;
	
	path.append(*current);
	while(current!=begin) 
	{
	  current = arbol->parent(current);
	  path.prepend(*current);

	}
	//Second tree
	if ( arbolGoal == NULL or arbolGoal->is_valid(_currentGoal) == false)
	  return path;
	begin = arbolGoal->begin();
	tree<QVec>::pre_order_iterator currentGoal = _currentGoal;

	path.append(*currentGoal);

	while(currentGoal!=begin) 
	{
		currentGoal = arbolGoal->parent(currentGoal);
		path.append(*currentGoal);

	}
	return path;
}

void Planner::adaptiveSmoother( const QList<QVec> & list)
{
	float sum=0.f, sumAnt=0.f;
	QList<QVec> localList = list;
	
// 	qDebug()<<"adaptiveSmoother";
  	for(int i=0; i< 10; i++)
	{
	  currentSmoothedPath.clear();

	  smoothPath(localList);
	  sum = 0.f;
	  for(int j=0 ; j< currentSmoothedPath.size()-1;j++)
		sum += (currentSmoothedPath[j] - currentSmoothedPath[j+1]).norm2();
/*	  qDebug() << "Sum " << sum << currentSmoothedPath.size();
	  qDebug()<<"----------------currentSmoothedPath-----------------";
	  for(int j=0 ; j< currentSmoothedPath.size();j++)
	    qDebug()<<"point"<<j<<":"<<currentSmoothedPath[j];*/
	  if( fabs(sumAnt-sum) == 0.f )
		break;
	  else
	  {
		sumAnt = sum;
		localList = adaptiveSmootherInsertPoints( currentSmoothedPath );
	  }
//   	  qDebug()<<"----------------localList-----------------";
// 	  for(int j=0 ; j< localList.size();j++)
// 	    qDebug()<<"point"<<j<<":"<<localList[j];

	}
	
}

void Planner::smoothPath( const QList<QVec> & list)
{
	bool reachEnd;
	tree<QVec>::iterator it;

	trySegmentToTarget( list.first(), list.last(), reachEnd, NULL, it);

// 	if(!reachEnd && list.size()<=2)
// 	  qDebug()<<"algo raro está pasando.......";

	if (reachEnd == true) 
	{
		if(!this->currentSmoothedPath.contains(list.first()))
		  this->currentSmoothedPath.append(list.first());
		if(!this->currentSmoothedPath.contains(list.last()))
		  this->currentSmoothedPath.append(list.last());

		return;
	}
	else
	{
		if(list.size()>2)	   
		{
		      smoothPath( list.mid(0,list.size()/2 +1));
		      smoothPath( list.mid( list.size()/2 , -1 ));
		}
	}
}

QList<QVec> Planner::adaptiveSmootherInsertPoints( const QList<QVec> & list)
{
	bool reachEnd, rEl1, rEl2;
	tree<QVec>::iterator it;
	QVec vl(3),vr(3);
	int NUM_ITER = 20;
	QList<QVec> localList;
	  
	localList.append(list.first());
	for(int i=1; i<list.size()-1; i++)
	{
	  reachEnd=false;
	  float kp2 = 2.f;
	  for(int k=1; k<NUM_ITER; k++)
	  {
		  vl = (list[i-1]*(1.f/kp2)) + (list[i]*((kp2-1)/kp2));
		  vr = (list[i+1]*(1.f/kp2)) + (list[i]*((kp2-1)/kp2));
		  kp2 = kp2 * 2.f;
		  trySegmentToTarget( vl, vr, reachEnd, NULL, it);
		  trySegmentToTarget( list[i-1], vl, rEl1, NULL, it);
		  trySegmentToTarget( vr, list[i+1], rEl2, NULL, it);
		  if (reachEnd && rEl1 && rEl2) 
		  {
			localList.append(vl);
// 			localList.append(list[i]);	
			localList.append(vr);
// 			qDebug() << "breaking " << k;
			break;
		  }
		  else
		    reachEnd=false;
	  }
	  if(!reachEnd)
		localList.append(list[i]);
	  else
	  {
		i++;
		if(i<list.size()-1)
			localList.append(list[i]);
	  }
	}
  	localList.append(currentSmoothedPath.last());
	return localList;
}

////////////
////Drawing
///////////

// void Planner::drawTree(InnerModelManagerPrx innermodelmanager_proxy)
// {
// 	tree<QVec>::pre_order_iterator begin = arbol->begin();
// 	tree<QVec>::pre_order_iterator end = arbol->end();
// 	tree<QVec>::pre_order_iterator padre;
// 		
// 	/*if (finalTarget.isEmpty() == false)
// 		world->drawEllipse(QPointF(this->finalTarget.x(), this->finalTarget.z()), 40, 40 , Qt::green, true);
// 	*/	
// 	QString name;
// 	int i=0;
// 	RoboCompInnerModelManager::Plane3D p;
// 	while(begin!=end) 
// 	{
// 		padre = arbol->parent(begin);
// 		if(arbol->is_valid(padre))
// 		{
// 			//world->drawLine(QLine((*padre).x(), (*padre).z(),(*begin).x(),(*begin).z()), Qt::blue, 20);
// 			name = "t_" + QString::number(i++);
// 			float len = (QVec::vec3((*padre).x(), 600, (*padre).z()) -  QVec::vec3((*begin).x(), 600, (*begin).z())).norm2() ;
// 			QVec midPoint = (QVec::vec3((*padre).x(), 600, (*padre).z()) + QVec::vec3((*begin).x(), 600, (*begin).z()) )*(T)0.5; ;
// 			p.px = midPoint.x(); p.py = midPoint.y();	p.pz = midPoint.z();
// 			QLine2D line(QVec::vec3((*padre).x(), 600, (*padre).z()),QVec::vec3((*begin).x(), 600, (*begin).z()));
// 			QVec normal = line.getNormalForOSGLineDraw();
// 			p.nx = normal.x();	p.ny = normal.y();	p.nz = normal.z();
// 			p.height = 20;	p.width = len;	p.thickness = 20;
// 			p.texture = "#0000FF";
// 			RcisDraw::addPlane_ignoreExisting(innermodelmanager_proxy, name, "floor", p);
// 		}
// 		++begin;
// 	}



//////////////////////////////////////////////////////////////////




// 	tree<QVec>::pre_order_iterator beginGoal = arbolGoal->begin();
// 	tree<QVec>::pre_order_iterator endGoal = arbolGoal->end();
// 	tree<QVec>::pre_order_iterator padreGoal;
	
// 	if (finalTarget.isEmpty() == false)
// 		world->drawEllipse(QPointF(this->finalTarget.x(), this->finalTarget.z()), 40, 40 , Qt::green, true);

// 	while(beginGoal!=endGoal) 
// 	{
// 		padreGoal = arbolGoal->parent(beginGoal);
// 		if(arbolGoal->is_valid(padreGoal))
// 		{
// 			name = "t_" + QString::number(i++);
// 			QVec midPoint = (QVec::vec3((*padreGoal).x(), 600, (*padreGoal).z()) + QVec::vec3((*beginGoal).x(), 600, (*beginGoal).z()) )*(T)0.5; 
// 			p.px = midPoint.x(); p.py = midPoint.y();	p.pz = midPoint.z();
// 			QLine2D line(QVec::vec3((*padre).x(), 600, (*padre).z()),QVec::vec3((*begin).x(), 600, (*begin).z()));
// 			QVec normal = line.getNormalForOSGLineDraw();
// 			p.nx = normal.x();	p.ny = normal.y();	p.nz = normal.z();
// 			p.height = 20;	p.width = midPoint.norm2();	p.thickness = 20;
// 			p.texture = "#'0000FF";
// 			RcisDraw::addPlane_ignoreExisting(innermodelmanager_proxy, name, "floor", p);
// 			//world->drawLine(QLine((*padreGoal).x(), (*padreGoal).z(),(*beginGoal).x(),(*beginGoal).z()), Qt::magenta, 20);
// 		}
// 		++beginGoal;
// 	}
//}

// void Planner::drawPath(qWorld *world)
// {
//  // drawPath(world, currentPath, Qt::yellow);
// }

// void Planner::drawSmoothedPath(qWorld *world)
// {
//   drawPath(world, currentSmoothedPath, Qt::green);
//   
//   QList<QVec> currentSmoothedPath2=currentSmoothedPath;
//   currentSmoothedPath.clear();
//   if(currentPath.size()>0)
//   {
// 	smoothPath(currentPath);
// 	drawPath(world, currentSmoothedPath, Qt::red);
//   }
// 
//   currentSmoothedPath=currentSmoothedPath2;
//}

// void Planner::drawPath(qWorld *world, const QList<QVec> & path, const QColor & color)
// {
//   if( path.size() > 1)
//   {	
// 	QVec pant = path[0];
// 	for(int i=1; i< path.size(); i++)
// 	{
// 	  world->drawLine(QLine(pant.x(), pant.z(), path[i].x(), path[i].z()), color, 40);
// 	  pant = path[i];
// 	}	
//   }
//}

// Only for Bezier
// 		this->world->drawEllipse(QPointF( this->origin.x(),this->origin.z()), 60, 60, Qt::blue,  true);
// 		this->world->drawEllipse(QPointF( this->p1.x(),this->p1.z()), 60, 60, Qt::black,true);
// 		this->world->drawEllipse(QPointF( this->p2.x(),this->p2.z()), 60, 60, Qt::red, true);
// 		this->world->drawEllipse(QPointF( this->target.x(),this->target.z()), 60, 60, Qt::magenta, true);	
