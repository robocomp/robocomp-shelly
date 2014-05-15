/*
 *    Copyright (C) 2006-2010 by RoboLab - University of Extremadura
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

/**
* \brief Default constructor
*/

#define BUUU printf("%s: %d\n", __FILE__, __LINE__);

SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx)	
{
	refresh = false;
	worldModel = AGMModel::SPtr(new AGMModel());
	targetModel = AGMModel::SPtr(new AGMModel());

	QPalette palette1 = modelWidget->palette();
	palette1.setColor( backgroundRole(), QColor( 255, 255, 255 ) );
	rcdraw1 = new RCDraw(modelWidget);
	modelWidget->setPalette(palette1);
	modelDrawer = new AGMModelDrawer(rcdraw1, tableWidget);
	
	QPalette palette2 = targetWidget->palette();
	palette2.setColor( backgroundRole(), QColor( 255, 255, 255 ) );
	rcdraw2 = new RCDraw(targetWidget);
	targetWidget->setPalette(palette2);
	targetDrawer = new AGMModelDrawer(rcdraw2);

	connect(quitButton,             SIGNAL(clicked()), this, SLOT(quitButtonClicked()));
	connect(broadcastButton,        SIGNAL(clicked()), this, SLOT(broadcastButtonClicked()));

	connect(activateButton,         SIGNAL(clicked()), this, SLOT(activateClicked()));
	connect(deactivateButton,       SIGNAL(clicked()), this, SLOT(deactivateClicked()));
	connect(resetButton,            SIGNAL(clicked()), this, SLOT(resetClicked()));

	connect( findMugButton,         SIGNAL(clicked()), this, SLOT(setMissionFindMug()));
	connect(graspMugButton,         SIGNAL(clicked()), this, SLOT(setMissionGraspMug()));

	timer.start(90);
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{

}

void SpecificWorker::compute( )
{
	static QTime taim = QTime::currentTime();

	if (refresh)
	{
		refresh = false;
		/// Print PLAN
		QString planString;
		planMutex.lock();
		printf("%d\n", (int)plan.actions.size());
		printf("%d\n", (int)plan.actions.size());
		printf("%d\n", (int)plan.actions.size());
		printf("%d\n", (int)plan.actions.size());
		printf("%d\n", (int)plan.actions.size());
		printf("%d\n", (int)plan.actions.size());
		printf("%d\n", (int)plan.actions.size());
		printf("%d\n", (int)plan.actions.size());
		printf("%d\n", (int)plan.actions.size());
		printf("%d\n", (int)plan.actions.size());
		printf("%d\n", (int)plan.actions.size());
		printf("%d\n", (int)plan.actions.size());
		printf("%d\n", (int)plan.actions.size());
		printf("%d\n", (int)plan.actions.size());
		printf("%d\n", (int)plan.actions.size());
		for (uint i=0; i<plan.actions.size(); ++i)
		{
			planString += "<p><b>" + QString::number(i+1) + "</b> ";
			planString += QString::fromStdString(plan.actions[i].name);
			planString += " ( ";
			if (plan.actions[i].symbols.size() > 0)
				planString += QString::fromStdString(plan.actions[i].symbols[0]);
			for (uint s=1; s<plan.actions[i].symbols.size(); ++s)
			{
				planString += ", ";
				planString += QString::fromStdString(plan.actions[i].symbols[s]);
			}
			planString += " ) </p>\n";
		}
		planMutex.unlock();
		planText->clear();
		planText->setText(planString);
	}

	modelMutex.lock();
	modelDrawer->update(worldModel);
	targetDrawer->update(targetModel);
	modelMutex.unlock();

	taim = QTime::currentTime();
}



bool SpecificWorker::setAgentParameters(const ParameterMap& params)
{
	return true;
}

void SpecificWorker::modelModified(const RoboCompAGMWorldModel::Event& modification)
{
	refresh = true;
	printf("MODEL MODIFIED (%s)\n", modification.sender.c_str());
	modelMutex.lock();
	AGMModelConverter::fromIceToInternal(modification.newModel, worldModel);
	AGMModelPrinter::printWorld(worldModel);
	modelDrawer->update(worldModel);
	modelMutex.unlock();
}

void SpecificWorker::modelUpdated(const RoboCompAGMWorldModel::Node& modification)
{
	refresh = true;
	modelMutex.lock();
	AGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);
	modelMutex.unlock();
}

void SpecificWorker::update(const RoboCompAGMWorldModel::World &a, const RoboCompAGMWorldModel::World &b, const RoboCompPlanning::Plan &pl)
{
	printf("SpecificWorker::update\n");
	planMutex.lock();
	plan = pl;
	planMutex.unlock();
	refresh = true;
}

void SpecificWorker::quitButtonClicked()
{
	printf("quit button\n");
	exit(0);
}

void SpecificWorker::broadcastButtonClicked()
{
	printf("broadcast button\n");
	agmexecutive_proxy->broadcastModel();
}


void SpecificWorker::setMissionFindMug()
{
	printf("find mug button\n");
	planText->clear();
	agmexecutive_proxy->broadcastModel();
	targetModel->clear();

	AGMModelSymbol::SPtr robot   = targetModel->newSymbol("robot");
	AGMModelSymbol::SPtr object  = targetModel->newSymbol("object");
	AGMModelSymbol::SPtr classS  = targetModel->newSymbol("class");
	AGMModelSymbol::SPtr mugType = targetModel->newSymbol("mug");
	
	robot->identifier   = 1000;
	object->identifier  = 1001;
	classS->identifier  = 1002;
	mugType->identifier = 1003;
	
	targetModel->addEdgeByIdentifiers( robot->identifier,  object->identifier, "know");
	targetModel->addEdgeByIdentifiers(object->identifier,  classS->identifier, "prop");
	targetModel->addEdgeByIdentifiers(classS->identifier, mugType->identifier, "prop");


	AGMModelConverter::fromInternalToIce(targetModel, targetModelICE);
	try { agmexecutive_proxy->deactivate(); } catch(const Ice::Exception & e) { cout << e << "agmexecutive_proxy->deactivate"<< endl; }
	try { agmexecutive_proxy->setMission(targetModelICE); } catch(const Ice::Exception & e) { cout << e << "agmexecutive_proxy->setMission. Check ExecutiveComp" << endl; }
	try { agmexecutive_proxy->activate(); } catch(const Ice::Exception & e) { cout << e << "agmexecutive_proxy->activate"<< endl; }
}


void SpecificWorker::setMissionGraspMug()
{
	printf("grasp mug button\n");
	planText->clear();
	agmexecutive_proxy->broadcastModel();
	targetModel->clear();

	AGMModelSymbol::SPtr robot   = targetModel->newSymbol("robot");
	AGMModelSymbol::SPtr object  = targetModel->newSymbol("object");
	AGMModelSymbol::SPtr grasp   = targetModel->newSymbol("grasp");
	AGMModelSymbol::SPtr classS  = targetModel->newSymbol("class");
	AGMModelSymbol::SPtr mugType = targetModel->newSymbol("mug");

	robot->identifier   = 1000;
	object->identifier  = 1001;
	classS->identifier  = 1002;
	mugType->identifier = 1003;
	grasp->identifier   = 1004;

	targetModel->addEdgeByIdentifiers( robot->identifier,  object->identifier, "know");
	targetModel->addEdgeByIdentifiers(object->identifier,   grasp->identifier, "prop");
	targetModel->addEdgeByIdentifiers(object->identifier,  classS->identifier, "prop");
	targetModel->addEdgeByIdentifiers(classS->identifier, mugType->identifier, "prop");


	AGMModelConverter::fromInternalToIce(targetModel, targetModelICE);
	try { agmexecutive_proxy->deactivate(); } catch(const Ice::Exception & e) { cout << e << "agmexecutive_proxy->deactivate"<< endl; }
	try { agmexecutive_proxy->setMission(targetModelICE); } catch(const Ice::Exception & e) { cout << e << "agmexecutive_proxy->setMission. Check ExecutiveComp" << endl; }
	try { agmexecutive_proxy->activate(); } catch(const Ice::Exception & e) { cout << e << "agmexecutive_proxy->activate"<< endl; }
}


void SpecificWorker::activateClicked()
{
	try
	{
		agmexecutive_proxy->activate();
	}
	catch(const Ice::Exception & e)
	{
		cout << e << "agmexecutive_proxy->activate" << endl;
	}
}

void SpecificWorker::deactivateClicked()
{
	try
	{
		agmexecutive_proxy->deactivate();
	}
	catch(const Ice::Exception & e)
	{
		cout << e << "agmexecutive_proxy->deactivate"<< endl;
	}
}


void SpecificWorker::resetClicked()
{
	try
	{
		agmexecutive_proxy->reset();
	}
	catch(const Ice::Exception & e)
	{
		cout << e << "agmexecutive_proxy->reset"<< endl;
	}
}


