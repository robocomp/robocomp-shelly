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

	QTimer *secondTimer = new QTimer();
	secondTimer->start(1000);
	connect(secondTimer, SIGNAL(timeout()), this, SLOT(setGeometry()));

	set3DViewer();

	connect(quitButton,           SIGNAL(clicked()), this, SLOT(quitButtonClicked()));
	connect(broadcastModelButton, SIGNAL(clicked()), this, SLOT(broadcastModelButtonClicked()));
	connect(broadcastPlanButton,  SIGNAL(clicked()), this, SLOT(broadcastPlanButtonClicked()));

	connect(activateButton,       SIGNAL(clicked()), this, SLOT(activateClicked()));
	connect(deactivateButton,     SIGNAL(clicked()), this, SLOT(deactivateClicked()));
	connect(resetButton,          SIGNAL(clicked()), this, SLOT(resetClicked()));

	connect(setMissionButton,     SIGNAL(clicked()), this, SLOT(setMission()));
	connect(imCheck,           SIGNAL(clicked()), this, SLOT(imShow()));
		
	innerModelVacio = new InnerModel();	
	osgView = new OsgView( inner3D );
	show();
	
	innerViewer = new InnerModelViewer(innerModelVacio, "root", osgView->getRootGroup(), true);
	manipulator = new osgGA::TrackballManipulator;
	osgView->setCameraManipulator(manipulator, true);
	innerViewer->setMainCamera(manipulator, InnerModelViewer::TOP_POV);
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
// 	printf("compute 1\n");
	static QTime taim = QTime::currentTime();

// 	graphViewer->animateStep();
	
	if (refresh)
	{
		refresh = false;
		/// Print PLAN
		QString planString;
		QMutexLocker dd(&planMutex);
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
		planText->clear();
		planText->setText(planString);
	}
	
	{
		
			
		QMutexLocker dd(&modelMutex);
		
		
		if (tabWidget->currentIndex()==0)
		{
			modelDrawer->update(worldModel);
			targetDrawer->update(targetModel);
		}
		else if (tabWidget->currentIndex()==1 )
		{
			
			graphViewer->update(worldModel);
			graphViewer->animateStep();
		}
		else
		{
			innerViewer->update();
			osgView->autoResize();		
			osgView->frame();			
		}
	}

	taim = QTime::currentTime();
// 	printf("compute>\n");
}


void SpecificWorker::changeInner (InnerModel *inner)
{
	inner->save("inner.xml");
	if (innerViewer)
	{
// 		QList<InnerModelNode*> l;l.clear();
// 		innerViewer->innerModel->getSubTree(innerViewer->innerModel->getRoot() ,&l);
// 		foreach(InnerModelNode* node, l) 
// 		{
// 			//remove node in InnerModel
// 			innerViewer->innerModel->removeNode(node->id);
// 		}

		osgView->getRootGroup()->removeChild(innerViewer);				
		qDebug()<<"delete innerViewer";
		
		innerViewer = NULL;
	}
	
	innerViewer = new InnerModelViewer(inner, "root", osgView->getRootGroup(), true);
	innerViewer->setMainCamera(manipulator, InnerModelViewer::TOP_POV);
}

bool SpecificWorker::setAgentParameters(const ParameterMap& params)
{
	return true;
}

void SpecificWorker::structuralChange(const RoboCompAGMWorldModel::Event& modification)
{
// 	printf("MODEL MODIFIED (%s)\n", modification.sender.c_str());
	{
		QMutexLocker dd(&modelMutex);
		AGMModelConverter::fromIceToInternal(modification.newModel, worldModel);
		//AGMModelPrinter::printWorld(worldModel);
		agmInner.setWorld(worldModel);		
		changeInner(agmInner.extractInnerModel("room"));		
		refresh = true;
		
	}
}

void SpecificWorker::symbolUpdated(const RoboCompAGMWorldModel::Node& modification)
{
	{
		QMutexLocker dd(&modelMutex);
		AGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);
		agmInner.setWorld(worldModel);				
		refresh = true;
	}
}
void SpecificWorker::edgeUpdated(const RoboCompAGMWorldModel::Edge& modification)
{
	{
		QMutexLocker dd(&modelMutex);
		AGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);
		agmInner.setWorld(worldModel);
// 		changeInner(agmInner.extractInnerModel("room"));		
		refresh = true;
	}
}


void SpecificWorker::update(const RoboCompAGMWorldModel::World &a, const RoboCompAGMWorldModel::World &b, const RoboCompPlanning::Plan &pl)
{
	printf("SpecificWorker::update\n");
	{
		QMutexLocker dd(&planMutex);
		plan = pl;
		refresh = true;
	}
}

void SpecificWorker::imShow()
{
	qDebug()<<"imCheck->isChecked()"<<imCheck->isChecked();
	modelDrawer->setShowInnerModel(imCheck->isChecked());
	
}



void SpecificWorker::quitButtonClicked()
{
	printf("quit button\n");
	exit(0);
}

void SpecificWorker::broadcastModelButtonClicked()
{
	printf("broadcast model button\n");
	try
	{
		agmexecutive_proxy->broadcastModel();
	}
	catch(const Ice::Exception &e)
	{
		QMessageBox::critical(this, "Can't connect to the executive", "Can't connect to the executive. Please, make sure the executive is running properly");
	}
}

void SpecificWorker::broadcastPlanButtonClicked()
{
	printf("broadcast plan button\n");
	try
	{
		agmexecutive_proxy->broadcastPlan();
	}
	catch(const Ice::Exception &e)
	{
		QMessageBox::critical(this, "Can't connect to the executive", "Can't connect to the executive. Please, make sure the executive is running properly");
	}
}


void SpecificWorker::setMission()
{
	printf("mission #%d\n", missions->currentIndex());
	try
	{
		agmexecutive_proxy->broadcastModel();
	}
	catch(const Ice::Exception & e)
	{
		QMessageBox::critical(this, "Can't connect to the executive", "Can't connect to the executive. Please, make sure the executive is running properly");
	}
	planText->clear();
	AGMModelConverter::fromXMLToInternal(missionPaths[missions->currentIndex()], targetModel);
	AGMModelConverter::fromInternalToIce(targetModel, targetModelICE);

	try
	{
		agmexecutive_proxy->deactivate();
		agmexecutive_proxy->setMission(targetModelICE);
		agmexecutive_proxy->activate(); 
	}
	catch(const Ice::Exception & e)
	{
		QMessageBox::critical(this, "Can't connect to the executive", "Can't connect to the executive. Please, make sure the executive is running properly");
	}
}

void SpecificWorker::activateClicked()
{
	try
	{
		agmexecutive_proxy->activate();
	}
	catch(const Ice::Exception & e)
	{
		QMessageBox::critical(this, "Can't connect to the executive", "Can't connect to the executive. Please, make sure the executive is running properly");
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
		QMessageBox::critical(this, "Can't connect to the executive", "Can't connect to the executive. Please, make sure the executive is running properly");
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
		QMessageBox::critical(this, "Can't connect to the executive", "Can't connect to the executive. Please, make sure the executive is running properly");
	}
}


void SpecificWorker::set3DViewer()
{
// #if QT_VERSION >= 0x050000
// 	// Qt5 is currently crashing and reporting "Cannot make QOpenGLContext current in a different thread" when the viewer is run multi-threaded, this is regression from Qt4
	osgViewer::ViewerBase::ThreadingModel threadingModel = osgViewer::ViewerBase::SingleThreaded;
// #else
// 	osgViewer::ViewerBase::ThreadingModel threadingModel = osgViewer::ViewerBase::CullDrawThreadPerContext;
// #endif

	graphViewer = new GraphModelViewer(threadingModel, widget3D, false);

	setGeometry();
	graphViewer->show();
}

void SpecificWorker::setGeometry()
{
	graphViewer->setGeometry(0, 0, widget3D->width(), widget3D->height());
}



