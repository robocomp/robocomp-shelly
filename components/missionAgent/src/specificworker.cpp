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
		
	innerModel = new InnerModel();
	innerViewer = NULL;
	osgView = new OsgView( inner3D );
	show();
	changeInner(innerModel);
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
// 	printf("compute\n");
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
		osgView->getRootGroup()->removeChild(innerViewer);
		
		//delete innerViewer;
		innerViewer = NULL;
	}
	innerViewer = new InnerModelViewer(inner, "root", osgView->getRootGroup(), true);

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
		AGMModelPrinter::printWorld(worldModel);
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
// 		changeInner(agmInner.extractInnerModel("room"));
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


//  /**
//  * @brief This method updates the simulator
//  * 
//  * @param idPerson Identifier of the person who will be updated in the simulator
//  * @return void
//  */
 void SpecificWorker::updateInner3D()
{
	
	QList<InnerModelNode *>	l;
	
	innerModel->getSubTree(innerModel->getNode("root"),&l);
	
	QList<InnerModelNode*>::iterator it;
	for (it=l.begin();it!=l.end();it++)
	{
		qDebug()<<"\tinsertNodeInnerModel "<<(*it)->id;
		insertNodeInnerModel((*it));
	}
}
void SpecificWorker::insertNodeInnerModel( InnerModelNode* node)
{
// 	qDebug()<<node->id;
// 	qDebug()<<"\t parent"<<node->parent->id;
	
	
	InnerModelNode * parent = node->parent;
// 	if (node->parent->id=="root")
// 	{			
// 		parent = im->getRoot();							
// 	}
// 	else
// 	{
// 		parent = im->getNode(pre+node->parent->id);
// 	}
// 	if (parent==NULL)
// 		qFatal("parent null stop");

	if(  dynamic_cast<InnerModelTransform *>( node )  != NULL )
	{
		
		InnerModelTransform * tf=dynamic_cast<InnerModelTransform *>( node );
				
		if(  dynamic_cast<InnerModelJoint *>( tf )  != NULL )
		{
			qDebug()<<"insert Joint"<<node->id;
			InnerModelJoint * joint=dynamic_cast<InnerModelJoint *>( tf );
			
			/*InnerModelJoint * newJoint = im->newJoint (pre+joint->id,dynamic_cast<InnerModelTransform *>( parent),
								joint->backlX,joint->backlY,joint->backlZ,joint->backhX,joint->backhY,joint->backhZ,
								joint->backtX,joint->backtY,joint->backtZ,joint->backrX,joint->backrY,joint->backrZ,
								joint->min, joint->max,joint->port,joint->axis,joint->home);
			parent->addChild(newJoint);*/		
			QVec r = QVec::vec3(joint->getRxValue(),joint->getRyValue(),joint->getRzValue());
			InnerModelDraw::addJoint(innerViewer,node->id,parent->id, joint->getTr(),r,QString::fromStdString(joint->axis));
		}
		else
		{
			qDebug()<<"insert transform"<<node->id;
			InnerModelDraw::addTransform_ignoreExisting(innerViewer,node->id,parent->id);
			
			/*InnerModelTransform * newTf = im->newTransform(pre+tf->id,tf->engine,parent,tf->backtX,tf->backtY,tf->backtZ,tf->backrX,tf->backrY,tf->backrZ,tf->mass);
			parent->addChild(newTf)*/;
			
			
		}
	}
	else if(  dynamic_cast<InnerModelMesh *>( node )  != NULL )
	{
		qDebug()<<"insert Mesh"<<node->id;
		InnerModelMesh * m=dynamic_cast<InnerModelMesh *>( node );
		QVec r = QVec::vec3(m->getRxValue(),m->getRyValue(),m->getRzValue());
		QVec scale = QVec::vec3(m->scalex,m->scaley,m->scalez);
		InnerModelDraw::addMesh_ignoreExisting(innerViewer,node->id,parent->id,m->getTr(),r,m->meshPath,scale);
		
// 		InnerModelMesh * newMesh = im->newMesh(pre+m->id,parent, m->meshPath,m->scalex,m->scaley,m->scalez,m->render,m->tx,m->ty,m->tz,m->rx,m->ry,m->rz,m->collidable);
// 		parent->addChild(newMesh);		
	}
	else if(  dynamic_cast<InnerModelPlane *>( node )  != NULL )
	{
		qDebug()<<"insert Plane"<<node->id;
		InnerModelPlane * p=dynamic_cast<InnerModelPlane *>( node );
		
		QVec size = QVec::vec3(p->width,p->height,p->depth);

		InnerModelDraw::addPlane_ignoreExisting (innerViewer,node->id,parent->id,p->point,p->normal, p->texture,size);
		
// 		InnerModelPlane * newPlane = im->newPlane(pre+p->id,parent,p->texture,
// 							p->width,p->height,p->depth,p->repeat,
// 							p->normal(0),p->normal(1),p->normal(2),
// 							p->point(0),p->point(1),p->point(2),
// 							p->collidable);
// 		parent->addChild(newPlane);		
	}
	else
	{
		qDebug()<<"type not implemented, node-id: asumo transform "<<node->id<<"\n";
		try
		{
			InnerModelDraw::addTransform_ignoreExisting(innerViewer,node->id,parent->id);
		}
		catch(...)
		{
			qDebug()<<"la recogo";
		}
		
		
	}
	
}
