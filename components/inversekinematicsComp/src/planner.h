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

#ifndef PLANNER_H
#define PLANNER_H

#include <QtCore>
#include <qmat/QMatAll>
#include "tree.hh"
#include <innermodel/innermodel.h>
//#include <InnerModelManager.h>


#define GLOBAL_OCCUPANCY_THRESHOLD 18.f

//using namespace RoboCompInnerModelManager;

class Planner : public QObject
{
Q_OBJECT
public:
	Planner(InnerModel *innerModel_, QObject *parent=0);
	~Planner(){};
  
	 void setMaxIter( int v) 		{ MAX_ITER = v;}
	 bool computePath(const QVec &target);
// 	 void drawTree( InnerModelManagerPrx innermodelmanager_proxy );
// 	 void drawPath( qWorld *world, const QList<QVec> & path , const QColor & color);
// 	 void drawPath(qWorld *world);
// 	 void drawSmoothedPath(qWorld *world);

	 QList<QVec> getPath() const	{ return currentSmoothedPath;}; 
	 QVec trySegmentToTarget(const QVec & origin, const QVec & target, bool & reachEnd, tree<QVec>  *arbol, tree<QVec>::iterator & nodeCurrentPos);
	 //QVec tryBezierToTarget(const QVec & origin , const QVec & target, bool & reachEnd, tree<QVec>  *arbol , tree<QVec>::iterator & nodeCurrentPos);
	 QList<QVec> recoverPath(tree<QVec> *arbol , const tree<QVec>::pre_order_iterator & _current, tree<QVec> *arbolGoal, const tree<QVec>::pre_order_iterator & _currentGoal);
	 bool isThereAnObstacleAtPosition(const QVec & pCenter, float rX, float rZ);
	 
  private:
		InnerModel *innerModel;
		tree<QVec> *arbol, *arbolGoal, *aux;
		tree<QVec>::pre_order_iterator CURRENT_LEAF, CURRENT_LEAF_GOAL;
		void cellOccupancyAlongLine( const QVec & roiPos);
		QVec chooseRandomPointInFreeSpace();
		bool collisionDetector( const QVec &point,  InnerModel *innerModel);
		bool equal(const QVec & p1, const QVec & p2);
		void smoothPath( const QList<QVec> & list);
		void adaptiveSmoother( const QList<QVec> & list);
		QList<QVec> adaptiveSmootherInsertPoints( const QList<QVec> & list);
		tree<QVec>::iterator findClosestPointInTree( tree<QVec> *arb , const QVec & currentTarget);
		QVec fsX, fsZ;
		void computeRandomSequence(const QVec & target);
		int ind;
		//QVec p2Ant; //init point
		QVec finalTarget;
		bool  PATH_FOUND;
		QVec p1,p2,origin,target;
		int MAX_ITER;
		QList<QVec> currentPath, currentSmoothedPath, currentAdaptiveSmoothedPath;
		
};

#endif // PLANNER_H











