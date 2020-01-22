/*
 *    Copyright (C) 2016 by YOUR NAME HERE
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
#include "specificworker.h"


class TimedList
{
	class TimedDatum
	{
	public:
		TimedDatum(float d)
		{
			datum = d;
			datum_time = QTime::currentTime();
		}
		float datum;
		QTime datum_time;
	};

public:
	TimedList(float msecs)
	{
		maxMSec = msecs;
	}
	void add(float datum)
	{
		data.push_back(TimedDatum(datum));
	}
	float getSum()
	{
		while (data.size()>0)
		{
			if (data[0].datum_time.elapsed() > maxMSec)
				data.pop_front();
			else
				break;
		}
		float acc = 0.;
		for (int i=0; i<data.size(); i++)
			acc += data[i].datum;
		return acc;
	}
private:
	float maxMSec;
	QList<TimedDatum> data;
};

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx)
{
	active = false;
	worldModel = AGMModel::SPtr(new AGMModel());
	worldModel->name = "worldModel";
	innerModel = std::make_shared<InnerModel>();
	newCGR = false;
	newApril = false;
	useCGR = false;
	useApril = false;

	worker_params_mutex = new QMutex();
	RoboCompCommonBehavior::Parameter aux;
	aux.editable = false;
	aux.type = "float";
	aux.value = "0";
	worker_params["frameRate"] = aux;
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{

}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	// Config params
	useCGR = QString::fromStdString(params["UseCGR"].value).contains("true");
	CGRWeight = QString::fromStdString(params["CGRWeight"].value).toFloat();
	useApril = QString::fromStdString(params["UseApril"].value).contains("false");
	aprilWeight = QString::fromStdString(params["AprilWeight"].value).toFloat();


	try
	{
		RoboCompAGMWorldModel::World w = agmexecutive_proxy->getModel();
		AGMExecutiveTopic_structuralChange(w);
	}
	catch(...)
	{
		printf("The executive is probably not running, waiting for first AGM model publication...");
	}

	return true;
}

void SpecificWorker::initialize(int period)
{
	std::cout << "Initialize worker" << std::endl;
	this->Period = period;
//	timer.start(Period);
	timer.start(1000);

}

void SpecificWorker::AprilBasedLocalization_newAprilBasedPose(const float x, const float z, const float alpha)
{
	QMutexLocker l(mutex);
	newApril = true;
	aprilState.correctedX = x;
	aprilState.correctedZ = z;
	aprilState.correctedAlpha = alpha;
}

void SpecificWorker::newCGRPose(const float poseUncertainty, const float x, const float z, const float alpha)
{
	QMutexLocker l(mutex);
	newCGR = true;
	cgrState.correctedX = x;
	cgrState.correctedZ = z;
	cgrState.correctedAlpha = alpha;
}

void SpecificWorker::newCGRCorrection(float poseUncertainty, float x1, float z1, float alpha1, float x2, float z2, float alpha2)
{
	QMutexLocker l(mutex);

	static InnerModel *innermodel=NULL;
	static InnerModelTransform *corrSLAMBack;
	static InnerModelTransform *corrSLAMNew;
	static InnerModelTransform *corrREALBack;
	static InnerModelTransform *corrREALNew;

	if (innermodel == NULL)
	{
		innerModel = std::make_shared<InnerModel>();

		corrSLAMBack = innermodel->newTransform("corrSLAMBack", "static", innermodel->getRoot(), 0,0,0, 0,0,0, 0);
		innermodel->getRoot()->addChild(corrSLAMBack);
		corrSLAMNew = innermodel->newTransform("corrSLAMNew", "static", innermodel->getRoot(), 0,0,0, 0,0,0, 0);
		innermodel->getRoot()->addChild(corrSLAMNew);

		corrREALBack = innermodel->newTransform("corrREALBack", "static", innermodel->getRoot(), 0,0,0, 0,0,0, 0);
		innermodel->getRoot()->addChild(corrREALBack);
		corrREALNew = innermodel->newTransform("corrREALNew", "static", corrREALBack, 0,0,0, 0,0,0, 0);
		corrREALBack->addChild(corrREALNew);
	}

	innermodel->updateTransformValues("corrSLAMBack",   x1, 0, z1,      0, alpha1, 0  );
	innermodel->updateTransformValues("corrSLAMNew",    x2, 0, z2,      0, alpha2, 0  );
	QVec inc = innermodel->transform6D("corrSLAMBack", "corrSLAMNew");

	inc.print("Correction: ");

	RoboCompGenericBase::TBaseState bState;
	try { omnirobot_proxy->getBaseState(bState); }
	catch(Ice::Exception &ex) { std::cout<<ex.what()<<std::endl; }
	innermodel->updateTransformValues("corrREALBack", bState.correctedX,0.,bState.correctedZ,   0,bState.correctedAlpha,0);
	innermodel->updateTransformValues("corrREALNew",   inc(0),0,inc(2),    0,inc(4),0);

	QVec result = innermodel->transform6D("root", "corrREALNew");
	newCGR = true;
	cgrState.correctedX = result(0);
	cgrState.correctedZ = result(2);
	cgrState.correctedAlpha = result(4);
}

void SpecificWorker::CGR_resetPose(const float x, const float z, const float alpha)
{
//subscribesToCODE

}


void SpecificWorker::compute()
{
	static QTime reloj = QTime::currentTime();
	QMutexLocker l(mutex);

	RoboCompGenericBase::TBaseState newState;
	static bool first=true;
	if (first)
	{
		qLog::getInstance()->setProxy("both", logger_pubproxy);
		rDebug2(("localizationAgent started"));
		first = false;
	}

	// retrieve model
	if (worldModel->getIdentifierByType("robot") < 0)
	{
		try
		{
			RoboCompAGMWorldModel::World w = agmexecutive_proxy->getModel();
			AGMExecutiveTopic_structuralChange(w);
		}
		catch(...)
		{
			printf("The executive is probably not running, waiting for first AGM model publication...");
		}
	}
	// retrieve different position values

	// odometry
	try
	{
		omnirobot_proxy->getBaseState(omniState);
		newState = omniState;
	}
	catch (...)
	{
		printf("Can't connect to the robot!!\n");
	}

	// cgr
	if (newCGR)
	{
// 		printf("ODOM: %f %f %f\n", newState.correctedX, newState.correctedZ, newState.correctedAlpha);
		newCGR = false;
		newState.correctedX = cgrState.correctedX;
		newState.correctedZ = cgrState.correctedZ;
		newState.correctedAlpha = cgrState.correctedAlpha;
		std::cout<<"CGR correction pose"<<std::endl;
		printf("CGR: %f %f %f\n", newState.correctedX, newState.correctedZ, newState.correctedAlpha);
	}

	// april
	if (newApril)
	{
		newApril = false;
	}

	// join bState position



	// Check if base needs correction
	if (enoughDifference(omniState, newState))
	{
		setCorrectedPosition(newState);
		lastState = newState;
	}

	odometryAndLocationIssues();

	worker_params_mutex->lock();
		//save framerate in params
		worker_params["frameRate"].value = std::to_string(reloj.restart()/1000.f);
	worker_params_mutex->unlock();
}

// compute difference between actual and last value to determine if it should be sent
bool SpecificWorker::enoughDifference(const RoboCompGenericBase::TBaseState &lastState, const RoboCompGenericBase::TBaseState &newState)
{
	if (fabs(newState.correctedX - lastState.correctedX) > 5 or fabs(newState.correctedZ - lastState.correctedZ) > 5 or fabs(newState.correctedAlpha - lastState.correctedAlpha) > 0.02)
	{
		return true;
	}
	return false;
}

// send corrected position
void SpecificWorker::setCorrectedPosition(const RoboCompGenericBase::TBaseState &bState)
{
	std::cout<<"correct odometer position"<<std::endl;
	try
	{
		omnirobot_proxy->correctOdometer(bState.correctedX, bState.correctedZ, bState.correctedAlpha);
	}
	catch(Ice::Exception &ex)
	{
		std::cout<<ex.what()<<std::endl;
	}
}

//update robot position in model

bool SpecificWorker::odometryAndLocationIssues(bool force)
{
	static QTime lastSent = QTime::currentTime();
	if (lastSent.elapsed() > 2000)
	{
		force = true;
	}
	// Get robot's odometry
	RoboCompGenericBase::TBaseState bState;
	try
	{
		omnirobot_proxy->getBaseState(bState);
	}
	catch (...)
	{
		printf("Can't connect to the robot!!\n");
		return false;
	}

	// Get robot's symbol and its identifier
	int32_t robotId=-1;
	robotId = worldModel->getIdentifierByType("robot");
	if (robotId < 0)
	{
		printf("Robot symbol not found, Waiting for the executive...\n");
		usleep(1000000);
		return false;
	}
	AGMModelSymbol::SPtr robot = worldModel->getSymbol(robotId);

	// Update odometry in the cognitive model
	includeMovementInRobotSymbol(robot);

	// Get current roomId
	int roomId=-1;
	for (auto edge = robot->edgesBegin(worldModel); edge != robot->edgesEnd(worldModel); edge++)
	{
		const std::pair<int32_t, int32_t> symbolPair = edge->getSymbolPair();

		if (edge->getLabel() == "RT")
		{
			const string secondType = worldModel->getSymbol(symbolPair.first)->symbolType;
			if (symbolPair.second == robotId and secondType == "room")
			{
				roomId = symbolPair.first;
				break;
			}
		}
	}
	if (roomId < 0)
	{
		printf("roomId not found, Waiting for Insert innerModel...\n");
		usleep(1000000);
		return false;
	}

	// Query which should actually be the current room based on the corrected odometry odometry
	int32_t robotIsActuallyInRoom;
	float schmittTriggerLikeThreshold = 80;
    robotIsActuallyInRoom = roomId;

//    if (bState.correctedZ < -schmittTriggerLikeThreshold)
//	{
//		robotIsActuallyInRoom = 5;
//	}
//	else if (bState.correctedZ > schmittTriggerLikeThreshold)
//	{
//		robotIsActuallyInRoom = 3;
//	}
//	else
//	{
//		robotIsActuallyInRoom = roomId;
//	}


	if (roomId != robotIsActuallyInRoom)
	{
		try
		{
			AGMModel::SPtr newModel(new AGMModel(worldModel));

			// Modify IN edge
			newModel->removeEdgeByIdentifiers(robotId, roomId, "in");
			newModel->addEdgeByIdentifiers(robotId, robotIsActuallyInRoom, "in");

			// Move "in" edges for every object IN the robot
			AGMModelSymbol::SPtr newRobot = newModel->getSymbol(robotId);
			for (auto edgeIn = robot->edgesBegin(newModel); edgeIn != robot->edgesEnd(newModel); edgeIn++)
			{
				const std::pair<int32_t, int32_t> symbolPair = edgeIn->getSymbolPair();
				if ( edgeIn->linking=="in" and symbolPair.second==robot->identifier )
				{
					AGMModelSymbol::SPtr objectToMove = newModel->getSymbol(symbolPair.first);
					newModel->removeEdgeByIdentifiers(objectToMove->identifier, roomId, "in");
					newModel->addEdgeByIdentifiers(objectToMove->identifier, robotIsActuallyInRoom, "in");
				}
			}


			// Modify RT edge
			AGMModelEdge edgeRT = newModel->getEdgeByIdentifiers(roomId, robotId, "RT");
			newModel->removeEdgeByIdentifiers(roomId, robotId, "RT");
			try
			{
				float bStatex = str2float(edgeRT->getAttribute("tx"));
				float bStatez = str2float(edgeRT->getAttribute("tz"));
				float bStatealpha = str2float(edgeRT->getAttribute("ry"));

				// to reduce the publication frequency
				if (fabs(bStatex - bState.correctedX)>5 or fabs(bStatez - bState.correctedZ)>5 or fabs(bStatealpha - bState.correctedAlpha)>0.02 or force)
				{
					//Publish update edge
					printf("\nUpdate odometry...\n");
					qDebug()<<"bState local --> "<<bStatex<<bStatez<<bStatealpha;
					qDebug()<<"bState corrected --> "<<bState.correctedX<<bState.correctedZ<<bState.correctedAlpha;

					edgeRT->setAttribute("tx", float2str(bState.correctedX));
					edgeRT->setAttribute("tz", float2str(bState.correctedZ));
					edgeRT->setAttribute("ry", float2str(bState.correctedAlpha));
				}
				newModel->addEdgeByIdentifiers(robotIsActuallyInRoom, robotId, "RT", edgeRT->attributes);
				AGMMisc::publishModification(newModel, agmexecutive_proxy, "navigationAgent");
				rDebug2(("navigationAgent moved robot from room"));
			}
			catch (...)
			{
				printf("Can't update odometry in RT, edge exists but we encountered other problem!!\n");
				return false;
			}
		}
		catch (...)
		{
			printf("Can't update room... do edges exist? !!!\n");
			return false;
		}
	}
	else
	{
		try
		{
			AGMModelEdge edge  = worldModel->getEdgeByIdentifiers(roomId, robotId, "RT");
			try
			{
				float bStatex = str2float(edge->getAttribute("tx"));
				float bStatez = str2float(edge->getAttribute("tz"));
				float bStatealpha = str2float(edge->getAttribute("ry"));
				// to reduce the publication frequency
				if (fabs(bStatex - bState.correctedX)>5 or fabs(bStatez - bState.correctedZ)>5 or fabs(bStatealpha - bState.correctedAlpha)>0.02 or force)
				{
					//Publish update edge
 					printf("\nUpdate odometry...\n");
 					qDebug()<<"bState local --> "<<bStatex<<bStatez<<bStatealpha;
 					qDebug()<<"bState corrected --> "<<bState.correctedX<<bState.correctedZ<<bState.correctedAlpha;
					edge->setAttribute("tx", float2str(bState.correctedX));
					edge->setAttribute("tz", float2str(bState.correctedZ));
					edge->setAttribute("ry", float2str(bState.correctedAlpha));
					lastSent = QTime::currentTime();
					AGMMisc::publishEdgeUpdate(edge, agmexecutive_proxy);
				}
			}
			catch (...)
			{
				printf("Can't update odometry in RT, edge exists but we encountered other problem!!\n");
				return false;
			}
		}
		catch (...)
		{
			printf("Can't update odometry in RT, edge does not exist? !!!\n");
			return false;
		}
	}

	return true;

}


void SpecificWorker::includeMovementInRobotSymbol(AGMModelSymbol::SPtr robot)
{
	static TimedList list(3000);
	static RoboCompGenericBase::TBaseState lastBaseState = lastState;

	const float movX = lastState.x - lastBaseState.x;
	const float movZ = lastState.z - lastBaseState.z;
	const float movA = abs(lastState.alpha - lastBaseState.alpha);
	const float mov = sqrt(movX*movX+movZ*movZ) + 20.*movA;
// 	printf("add mov: %f\n", mov);
	list.add(mov);
	lastBaseState = lastState;


	const float currentValue = list.getSum();
// 	printf("sum: %f\n", currentValue);
	bool setValue = true;

	static QTime lastSent = QTime::currentTime();
	try
	{
		const float availableValue = str2float(robot->getAttribute("movedInLastSecond"));
		const float ddiff = abs(currentValue-availableValue);
		if (ddiff < 5 and lastSent.elapsed()<1000)
		{
			setValue = false;
		}
	}
	catch(...)
	{
	}

	if (setValue)
	{
		lastSent = QTime::currentTime();
		const std::string attrValue = float2str(currentValue);
		robot->setAttribute("movedInLastSecond", attrValue);
		try
		{
			AGMMisc::publishNodeUpdate(robot, agmexecutive_proxy);
		}
		catch (...)
		{
			printf("Executive not running?\n");
		}
	}
}



//AGENT RELATED
bool SpecificWorker::AGMCommonBehavior_reloadConfigAgent()
{
	return true;
}

bool SpecificWorker::AGMCommonBehavior_activateAgent(const ParameterMap &prs)
{
	bool activated = false;
	printf("<<activateAgent\n");
	if (setParametersAndPossibleActivation(prs, activated))
	{
		if (not activated)
		{
			printf("activateAgent 0 >>\n");
			return activate(p);
		}
	}
	else
	{
		return false;
	}
	return true;
}

bool SpecificWorker::AGMCommonBehavior_setAgentParameters(const ParameterMap &prs)
{
	bool activated = false;
	return setParametersAndPossibleActivation(prs, activated);
}

ParameterMap SpecificWorker::AGMCommonBehavior_getAgentParameters()
{
	return params;
}

void SpecificWorker::AGMCommonBehavior_killAgent()
{

}

int SpecificWorker::AGMCommonBehavior_uptimeAgent()
{
	return 0;
}

bool SpecificWorker::AGMCommonBehavior_deactivateAgent()
{
	return deactivate();
}

StateStruct SpecificWorker::AGMCommonBehavior_getAgentState()
{
	StateStruct s;
	if (isActive())
	{
		s.state = Running;
	}
	else
	{
		s.state = Stopped;
	}
	s.info = p.action.name;
	return s;
}

void SpecificWorker::AGMExecutiveTopic_selfEdgeAdded(const int nodeid, const string &edgeType, const RoboCompAGMWorldModel::StringDictionary &attributes)
{
//subscribesToCODE
    QMutexLocker lockIM(mutex);
    try { worldModel->addEdgeByIdentifiers(nodeid, nodeid, edgeType, attributes); } catch(...){ printf("Couldn't add an edge. Duplicate?\n"); }

    try { innerModel.reset(AGMInner::extractInnerModel(worldModel)); } catch(...) { printf("Can't extract an InnerModel from the current model.\n"); }
}

void SpecificWorker::AGMExecutiveTopic_selfEdgeDeleted(const int nodeid, const string &edgeType)
{
//subscribesToCODE
    QMutexLocker lockIM(mutex);
    try { worldModel->removeEdgeByIdentifiers(nodeid, nodeid, edgeType); } catch(...) { printf("Couldn't remove an edge\n"); }

    try { innerModel.reset(AGMInner::extractInnerModel(worldModel)); } catch(...) { printf("Can't extract an InnerModel from the current model.\n"); }
}

void SpecificWorker::AGMExecutiveTopic_structuralChange(const RoboCompAGMWorldModel::World &w)
{
	mutex->lock();
 	AGMModelConverter::fromIceToInternal(w, worldModel);

//	if (innerModel) delete innerModel;
	innerModel.reset(AGMInner::extractInnerModel(worldModel));
	mutex->unlock();
}

void SpecificWorker::AGMExecutiveTopic_edgesUpdated(const RoboCompAGMWorldModel::EdgeSequence &modification)
{
	QMutexLocker locker(mutex);
	AGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);

//	delete innerModel;
	innerModel.reset(AGMInner::extractInnerModel(worldModel));
}

void SpecificWorker::AGMExecutiveTopic_edgeUpdated(const RoboCompAGMWorldModel::Edge &modification)
{
	QMutexLocker locker(mutex);
	AGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);

//	delete innerModel;
	innerModel.reset(AGMInner::extractInnerModel(worldModel));
}

void SpecificWorker::AGMExecutiveTopic_symbolUpdated(const RoboCompAGMWorldModel::Node &modification)
{
	QMutexLocker locker(mutex);
	AGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);

//	delete innerModel;
	innerModel.reset(AGMInner::extractInnerModel(worldModel));
}

void SpecificWorker::AGMExecutiveTopic_symbolsUpdated(const RoboCompAGMWorldModel::NodeSequence &modification)
{
	QMutexLocker locker(mutex);
	AGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);

//	delete innerModel;
	innerModel.reset(AGMInner::extractInnerModel(worldModel));
}




bool SpecificWorker::setParametersAndPossibleActivation(const ParameterMap &prs, bool &reactivated)
{
	printf("<<< setParametersAndPossibleActivation\n");
	// We didn't reactivate the component
	reactivated = false;

	// Update parameters
	params.clear();
	for (ParameterMap::const_iterator it=prs.begin(); it!=prs.end(); it++)
	{
		params[it->first] = it->second;
	}

	try
	{
		action = params["action"].value;
		std::transform(action.begin(), action.end(), action.begin(), ::tolower);
		//TYPE YOUR ACTION NAME
		if (action == "actionname")
		{
			active = true;
		}
		else
		{
			active = true;
		}
	}
	catch (...)
	{
		printf("exception in setParametersAndPossibleActivation %d\n", __LINE__);
		return false;
	}

	// Check if we should reactivate the component
	if (active)
	{
		active = true;
		reactivated = true;
	}

	printf("setParametersAndPossibleActivation >>>\n");

	return true;
}
void SpecificWorker::sendModificationProposal(AGMModel::SPtr &worldModel, AGMModel::SPtr &newModel)
{
	try
	{
//		AGMModelPrinter::printWorld(newModel);
		AGMMisc::publishModification(newModel, agmexecutive_proxy, "localizationAgentAgent");
	}
	catch(const RoboCompAGMExecutive::Locked &e)
	{
		printf("error when sending modification proposal (Locked)\n");
	}
	catch(const RoboCompAGMExecutive::OldModel &e)
	{
		printf("error when sending modification proposal (OldModel)\n");
	}
	catch(const RoboCompAGMExecutive::InvalidChange &e)
	{
		printf("error when sending modification proposal (InvalidChange)\n");
	}
	catch(const Ice::Exception& e)
	{
		printf("error when sending modification proposal\n");
		exit(1);
	}
}
RoboCompCommonBehavior::ParameterList SpecificWorker::getWorkerParams()
{
	QMutexLocker locker(worker_params_mutex);
	return worker_params;
}
