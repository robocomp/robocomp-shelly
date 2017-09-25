/*
 *    Copyright (C)2017 by YOUR NAME HERE
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
#include "genericworker.h"
/**
* \brief Default constructor
*/
GenericWorker::GenericWorker(MapPrx& mprx) :
#ifdef USE_QTGUI
Ui_guiDlg()
#else
QObject()
#endif

{
	jointmotor_proxy = (*(JointMotorPrx*)mprx["JointMotorProxy"]);
	inversekinematics_proxy = (*(InverseKinematicsPrx*)mprx["InverseKinematicsProxy"]);
	agmexecutive_proxy = (*(AGMExecutivePrx*)mprx["AGMExecutiveProxy"]);
	logger_proxy = (*(LoggerPrx*)mprx["LoggerPub"]);

	mutex = new QMutex(QMutex::Recursive);
	worldModel = NULL;
	#ifdef USE_QTGUI
		setupUi(this);
		show();
	#endif
	Period = BASIC_PERIOD;
	connect(&timer, SIGNAL(timeout()), this, SLOT(compute()));
// 	timer.start(Period);
}

/**
* \brief Default destructor
*/
GenericWorker::~GenericWorker()
{

}
void GenericWorker::killYourSelf()
{
	rDebug("Killing myself");
	emit kill();
}
/**
* \brief Change compute period
* @param per Period in ms
*/
void GenericWorker::setPeriod(int p)
{
	rDebug("Period changed"+QString::number(p));
	Period = p;
	timer.start(Period);
}

RoboCompPlanning::Action GenericWorker::createAction(std::string s)
{
	// Remove useless characters
	char chars[]="()";
	for (unsigned int i=0; i<strlen(chars); ++i)
	{
		s.erase(std::remove(s.begin(), s.end(), chars[i]), s.end());
	}

		// Initialize string parsing
	RoboCompPlanning::Action ret;
	istringstream iss(s);

	// Get action (first segment)
	if (not iss)
	{
		printf("agent %s: received invalid action (%s) -> (%d)\n", PROGRAM_NAME, __FILE__, __LINE__);
		exit(-1);
	}
	else
	{
		iss >> ret.name;
	}

	do
	{
		std::string ss;
		iss >> ss;
		ret.symbols.push_back(ss);
	} while (iss);

	return ret;
}


bool GenericWorker::activate(const BehaviorParameters &prs)
{
	printf("Worker::activate\n");
	mutex->lock();
	p = prs;
	active = true;
	iter = 0;
	mutex->unlock();
	return active;
}

bool GenericWorker::deactivate()
{
	printf("Worker::deactivate\n");
	mutex->lock();
	active = false;
	iter = 0;
	mutex->unlock();
	return active;
}


bool GenericWorker::setParametersAndPossibleActivation(const ParameterMap &prs, bool &reactivated)
{
	params.clear();
	for (ParameterMap::const_iterator it=prs.begin(); it!=prs.end(); it++)
	{
		params[it->first] = it->second;
	}

	// Types
	agm_types.clear();
	std::string ssss = params["types"].value;
	std::vector<std::string> types_split;
 	boost::split(types_split, ssss, [](char c){return c == '#';});
	for (auto sub : types_split)
	{
		std::vector<std::string> aType;
		std::vector<std::string> types_split_elements;
	 	boost::split(types_split_elements, sub, [](char c){return c == ' ';});
		for (auto sub : types_split_elements)
		{
			aType.push_back(sub);
		}

		agm_types[types_split_elements[types_split_elements.size()-1]] = aType;
	}


	QMutexLocker locker(mutex);
	if ( worldModel == NULL)
	{
		printf("******No model yet %p", worldModel.get());
		return true;
	}

	// if (prs.find("modelversion") != prs.end() and stoi(prs.find("modelversion")->second.value) != worldModel->version)
	// {
	// 	previousParams.clear();
	// 	for (ParameterMap::const_iterator it=prs.begin(); it!=prs.end(); it++)
	// 	{
	// 		previousParams[it->first] = it->second;
	// 	}
	// 	reactivated = false;
	// 	qDebug()<<"******Plan is more updated than model, we have to wait newModel";
	// 	return true;
	// }

	// We didn't reactivate the component
	reactivated = false;
	// Update parameters
	previousParams.clear();


	try
	{
		backAction = action;
		action = "";
		for (uint i=0; i<params["action"].value.size(); i++)
		{
			action += params["action"].value[i];
			if (i+2<params["action"].value.size())
			{
				if (params["action"].value[i+1] == '_' and params["action"].value[i+2] == '_')
					break;
			}
		}
		std::transform(action.begin(), action.end(), action.begin(), ::tolower);


		if (action == "graspobject") { active = true; }
		else { active = true; }
	}
	catch (...)
	{
		printf("exception in setParametersAndPossibleActivation %d\n", __LINE__);
		return false;
	}

	// Fill received plan
	p.plan.clear();
	QStringList actionList = QString::fromStdString(params["plan"].value).split(QRegExp("[()]+"), QString::SkipEmptyParts);
	for (int32_t actionString=0; actionString<actionList.size(); actionString++)
	{
		std::vector<string> elementsVec;
		QStringList elements = actionList[actionString].remove(QChar('\n')).split(QRegExp("\\s+"), QString::SkipEmptyParts);
		for (int32_t elem=0; elem<elements.size(); elem++)
		{
			elementsVec.push_back(elements[elem].toStdString());
		}
		p.plan.push_back(elementsVec);
	}

	// Check if we should reactivate the component
	if (active)
	{
		active = true;
		reactivated = true;
	}
	return true;
}
