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

#include "plannerompl.h"



PlannerOMPL::PlannerOMPL(InnerModel *innerModel_, QObject *parent)
{
// 	xMin = 0.;
// 	xMax = 10000.;
// 	zMin = -10000.;
// 	zMax = 0.;
// 	
// 	innerModel = new InnerModel(*innerModel_);
// 	
// 	//Create state space as R2
// 	ob::RealVectorStateSpace *space = new ob::RealVectorStateSpace();
// 	space->addDimension(xMin, xMax);
// 	space->addDimension(zMin, zMax);
// 	
// 	//Setup class
// 	simpleSetUp.reset(new og::SimpleSetup(ob::StateSpacePtr(space)));
// 	
// 	//set Sampler
// 	simpleSetUp->getSpaceInformation()->setValidStateSamplerAllocator(allocOBValidStateSampler);
// 	
// 	// set state validity checking for this space
// 	//simpleSetUp->setStateValidityChecker(boost::bind(&PlannerOMPL::isStateValid, this, _1));
// 	simpleSetUp->setStateValidityChecker(boost::bind(&Sampler::isStateValid, sampler, _1));
// 	space->setup();
// 	simpleSetUp->getSpaceInformation()->setStateValidityCheckingResolution(0.01);
// 	//simpleSetUp->getSpaceInformation()->setStateValidityCheckingResolution(100 / space->getMaximumExtent());
// 	simpleSetUp->setPlanner(ob::PlannerPtr(new og::RRTConnect(simpleSetUp->getSpaceInformation())));
// 	simpleSetUp->getPlanner()->as<og::RRTConnect>()->setRange(2000);
// 	//simpleSetUp->setPlanner(ob::PlannerPtr(new og::RRT(simpleSetUp->getSpaceInformation())));
// 	//simpleSetUp->setPlanner(ob::PlannerPtr(new og::RRTstar(simpleSetUp->getSpaceInformation())));
// 	//simpleSetUp->setPlanner(ob::PlannerPtr(new og::PRMstar(simpleSetUp->getSpaceInformation())));
// 	//simpleSetUp->setPlanner(ob::PlannerPtr(new og::LBTRRT(simpleSetUp->getSpaceInformation())));
}

ob::ValidStateSamplerPtr PlannerOMPL::allocOBValidStateSampler(const ob::SpaceInformation *si)
{
	return ob::ValidStateSamplerPtr(new ob::ObstacleBasedValidStateSampler(si));
}

//void PlannerOMPL::initialize(const Sampler &sampler )
void PlannerOMPL::initialize(Sampler *sampler )
{
	QRectF outerRegion = sampler->getOuterRegion();
	xMin = outerRegion.left();      
	xMax = outerRegion.right();
	zMin = outerRegion.bottom();
	zMax = outerRegion.top();
	
	//Create state space as R2
	ob::RealVectorStateSpace *space = new ob::RealVectorStateSpace();
	space->addDimension(xMin, xMax);
	space->addDimension(zMin, zMax);
	
	//Setup class
	simpleSetUp.reset(new og::SimpleSetup(ob::StateSpacePtr(space)));

	//set Sampler
	simpleSetUp->getSpaceInformation()->setValidStateSamplerAllocator(allocOBValidStateSampler);
	
	// set state validity checking for this space

	simpleSetUp->setStateValidityChecker(boost::bind(&Sampler::isStateValid, sampler, _1));
	space->setup();
	
	// Set the resolution at which state validity needs to be verified in order for a motion between two states to be considered valid. 
	// This value is specified as a fraction of the space's extent.
	simpleSetUp->getSpaceInformation()->setStateValidityCheckingResolution(0.01);  
	
	simpleSetUp->setPlanner(ob::PlannerPtr(new og::RRTConnect(simpleSetUp->getSpaceInformation())));
	
	//represents the maximum length of a motion to be added in the tree of motions.
	simpleSetUp->getPlanner()->as<og::RRTConnect>()->setRange(2000);  
}

bool PlannerOMPL::setPath(const QVec& origin, const QVec &target)
{
	//Planning proper
	if (simpleSetUp == NULL)
		return false;

	simpleSetUp->clear();
	
	ob::ScopedState<> start(simpleSetUp->getStateSpace());
	start[0] = origin.x();	start[1] = origin.z();
	ob::ScopedState<> goal(simpleSetUp->getStateSpace());
	goal[0] = target.x();	goal[1] = target.z();
	simpleSetUp->setStartAndGoalStates(start, goal);
	simpleSetUp->getProblemDefinition()->print(std::cout);
	
	return true;
}

bool PlannerOMPL::computePath(const QVec& origin, const QVec &target, int maxTime)  //maxTime in seconds
{	

	simpleSetUp->clear();
	ob::ScopedState<> start(simpleSetUp->getStateSpace());
	start[0] = origin.x();	start[1] = origin.z();
	ob::ScopedState<> goal(simpleSetUp->getStateSpace());
	goal[0] = target.x();	goal[1] = target.z();
	simpleSetUp->setStartAndGoalStates(start, goal);
	simpleSetUp->getProblemDefinition()->print(std::cout);
	//Call the planner
	printf("starting RRT search with maxTime %d seconds\n",maxTime);
	ob::PlannerStatus solved = simpleSetUp->solve(maxTime);
	printf("FINISH SEARCH\n");
	if (solved)
	{
		std::cout << __FILE__ << __FUNCTION__ << "RRT, found solution with " << simpleSetUp->getSolutionPath().getStateCount() << " waypoints" << std::endl;;
	
		//if (simpleSetUp->haveSolutionPath())	
		//simpleSetUp->simplifySolution();
		og::PathGeometric &p = simpleSetUp->getSolutionPath();
		std::cout << __FILE__ << __FUNCTION__ << "Solution after simplify: " << p. getStateCount() << ". Path length: " << p.length() << std::endl;
		p.print(std::cout);

 		simpleSetUp->getPathSimplifier()->simplify(p,5);
 		//simpleSetUp->getPathSimplifier()->smoothBSpline(p);
//		p.interpolate();
		
		for (std::size_t i = 0; i < p.getStateCount(); ++i)
		{
			currentPath.append( QVec::vec3( p.getState(i)->as<ob::RealVectorStateSpace::StateType>()->values[0], 
											0, 
											p.getState(i)->as<ob::RealVectorStateSpace::StateType>()->values[1]));
		}
		return true;
	}
	else
		return false;
}

ob::PlannerStatus PlannerOMPL::getPlanState()
{
	return simpleSetUp->getLastPlannerStatus();
}
