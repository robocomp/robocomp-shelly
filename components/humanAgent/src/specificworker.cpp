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
#include "specificworker.h"
#include <stdio.h>
#include <iostream>
#include <fstream>
#define SINGLE_MODE 

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx)
{
	active = false;
	worldModel = AGMModel::SPtr(new AGMModel());
	worldModel->name = "worldModel";
	innerModelVacio = new InnerModel();
	innerModelsLocals = new InnerModel();
	innerModelAGM = new InnerModel();
	imHumanGeneric = new InnerModel("/home/robocomp/robocomp/components/robocomp-ursus-rockin/etc/person.xml");
	newBodyEvent = false;
	number=0;
	idSingle=3000;
	
	innerModelMap.clear();
	initDictionary();
	personList.clear();
	
#ifdef SINGLE_MODE	
	qDebug()<<__LINE__;
	innerModelMap[idSingle]=innerModelVacio;	
	qDebug()<<__LINE__;
	QString pre =QString::fromStdString(int2str(idSingle));
	qDebug()<<__LINE__;
	newInnerModel(imHumanGeneric, innerModelMap.at(idSingle),pre);
	qDebug()<<__LINE__;
	
	QList<InnerModelNode *>	l;
	l.clear();
	innerModelMap.at(idSingle)->getSubTree(innerModelMap.at(idSingle)->getNode("root"),&l);		
	QList<InnerModelNode*>::iterator it;
	for (it=l.begin();it!=l.end();it++)
	{
		insertNodeInnerModel(innerModelsLocals,(*it));
	}
// 	innerViewer = new InnerModelViewer(innerModelsLocals, "root", osgView->getRootGroup(), true);	
	qDebug()<<__LINE__<<innerModelMap.size();
#endif
	
#ifdef USE_QTGUI
	osgView = new OsgView(this);
	
	innerViewer = new InnerModelViewer(innerModelsLocals, "root", osgView->getRootGroup(), true);

	manipulator = new osgGA::TrackballManipulator;
	osgView->setCameraManipulator(manipulator, true);
	innerViewer->setMainCamera(manipulator, InnerModelViewer::FRONT_POV);
	show();
	
#endif
	
	
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
}

void SpecificWorker::newMSKBodyEvent(const PersonList &people, const long &timestamp)
{
	QMutexLocker m (mutex);
	this->personList = people;
	this->timeStamp = timestamp;
	
	///esto es pq cuando hay 0 personas no me envia nada
// 	timerTimeStamp.setSingleShot(true);
// 	timerTimeStamp.start(500);
	newBodyEvent = true;	
}

//Dado  un innerModel creo otro, añadiendo el prefijo a cada id del nodo
void SpecificWorker::newInnerModel(InnerModel * imSrc, InnerModel *imDst, QString pre)
{
	//recorrer innermodel generico añadiendo el sufijo del nuevo symbolo e insertandolo en el map
	
	QList<InnerModelNode *>	l;

	imSrc->getSubTree(imSrc->getNode("root"),&l);

	QList<InnerModelNode*>::iterator it;
	for (it=l.begin();it!=l.end();it++)
	{
		insertNodeInnerModel(imDst,(*it),pre);
	}
	//copia perfecta con su prefijo
	//imDst->save("imDst.xml");

}
void SpecificWorker::insertNodeInnerModel(InnerModel* im, InnerModelNode* node, QString pre)
{
// 	qDebug()<<"pre:"<<pre;
// 	qDebug()<<node->id;
// 	qDebug()<<"\t parent"<<node->parent->id;
	
	
	InnerModelNode * parent = NULL;
	if (node->parent->id=="root")
	{			
		parent = im->getRoot();							
	}
	else
	{
		parent = im->getNode(pre+node->parent->id);
	}
	if (parent==NULL)
		qFatal("parent null stop");

	if (dynamic_cast<InnerModelTransform *>( node )  != NULL )
	{
		
		InnerModelTransform * tf=dynamic_cast<InnerModelTransform *>( node );
				
		if(  dynamic_cast<InnerModelJoint *>( tf )  != NULL )
		{
// 			qDebug()<<"insert Joint"<<node->id;
			InnerModelJoint * joint=dynamic_cast<InnerModelJoint *>( tf );
			
			InnerModelJoint * newJoint = im->newJoint (pre+joint->id,dynamic_cast<InnerModelTransform *>( parent),								   
								joint->backlX,joint->backlY,joint->backlZ,joint->backhX,joint->backhY,joint->backhZ,
								joint->getTr().x(),joint->getTr().y(),joint->getTr().z(),joint->getRxValue(),joint->getRyValue(),joint->getRzValue(),
								joint->min, joint->max,joint->port,joint->axis,joint->home);
			parent->addChild(newJoint);		
		}
		else
		{
// 			qDebug()<<"insert transform"<<node->id;
			InnerModelTransform * newTf = im->newTransform(pre+tf->id,tf->engine,parent,tf->getTr().x(),tf->getTr().y(),tf->getTr().z(),
								       tf->getRxValue(),tf->getRyValue(),tf->getRzValue(),tf->mass);
			parent->addChild(newTf);	
		}
	}
	else if(  dynamic_cast<InnerModelMesh *>( node )  != NULL )
	{
// 		qDebug()<<"insert Mesh"<<node->id;
		InnerModelMesh * m=dynamic_cast<InnerModelMesh *>( node );
		try
		{
			InnerModelMesh * newMesh = im->newMesh(pre+m->id,parent, m->meshPath,m->scalex,m->scaley,m->scalez,m->render,m->tx,m->ty,m->tz,m->rx,m->ry,m->rz,m->collidable);
			parent->addChild(newMesh);		
		}
		catch (QString e )
		{
			qDebug()<<"error mesh"<<e;
		}

	}
	else if(  dynamic_cast<InnerModelPlane *>( node )  != NULL )
	{
// 		qDebug()<<"insert Plane"<<node->id;
		InnerModelPlane * p=dynamic_cast<InnerModelPlane *>( node );
		
		InnerModelPlane * newPlane = im->newPlane(pre+p->id,parent,p->texture,
							p->width,p->height,p->depth,p->repeat,
							p->normal(0),p->normal(1),p->normal(2),
							p->point(0),p->point(1),p->point(2),
							p->collidable);
		parent->addChild(newPlane);		
	}
	else
	{
		qDebug()<<"type not implemented, node-id: "<<node->id<<"\n";
	}
	
}

void SpecificWorker::compute()
{
  QMutexLocker m (mutex);
  if (worldModel->numberOfSymbols()==0)
  {
	  qDebug()<<"waiting for executive";
	  return;
  }
	
/********************* TEST CODE***********************************************	
	if (innerModelMap.empty())
	{
		printf("Created: %d\n", __LINE__);
		innerModelMap[0] =new InnerModel();
		newInnerModel(imHumanGeneric, innerModelMap.at(0),"0");
		innerModelMap[1] =new InnerModel();
		newInnerModel(imHumanGeneric, innerModelMap.at(0),"1");
	}	
	updateViewerLocalInnerModels();
*******************************************************************/	

	if (newBodyEvent==false)
	{
		//qDebug()<<"newBodyEvent"<<newBodyEvent;	
		return;
	}
// 	if (timerTimeStamp.isActive()==false)
// 	{
		newBodyEvent=false;
// 	}
	
	//single
	TPerson p =(*personList.begin()).second;
	float zTorso = p.joints[dictionaryEnum["Spine"]].Position.Z;
	qDebug()<<"ZZZZZZZZZZ"<<zTorso;
	
	if (zTorso < 1.5 or zTorso > 5.0)
	{
		qDebug()<<"distance wrong to update"<<zTorso<<"meters";
		return;
	}
	updateViewerLocalInnerModelSingle();		
	updateHumanInnerFull(); 

	//multi
//	updateViewerLocalInnerModels();		
//	updatePeopleInnerFullB(); 

	
	//clear personList after a while without to recive any event
// 	if (timerTimeStamp.isActive() == false and !personList.empty())		
// 	{		
// 		std::cout<<"\t clear list \n";
// 		personList.clear();		

// 		/*updatePeopleInnerFull();
// 		if (!innerModelMap.empty())
// 		{
// 			saveInnerModels("666666");
// 			innerModelsLocals->save("innerModelsLocals.xml");
// 			qDebug()<<"innerModelMap.size()"<<innerModelMap.size();			
// 			qFatal("fary innerModelMap not empty");
// 		}
//	}

#ifdef USE_QTGUI
	innerViewer->update();
	osgView->autoResize();
	osgView->frame();
#endif	
	
}




void SpecificWorker::updatePeopleInnerFullB()
{
	QMutexLocker m (mutex);
	int32_t robotID = worldModel->getIdentifierByType("robot");
	if (robotID < 0)
	{
		printf("Robot symbol not found, Waiting for the executive...\n");
		return;
	}
	bool modification = false;

	///CAUTION CHAPUZA PA PROBAR A COLGAR DLE MUNDO 
	int32_t roomID = AgmInner::findSymbolIDWithInnerModelName(worldModel,"room");
	if (roomID < 0)
	{
		printf("ROOOM symbol not found, \n");
		qFatal("abort");
		return;
	}
	
	//extrae en una lista con los ID de los symbolos "person" que son hijos del symbolo room enlazados mediante "RT"
	//Qlist<int32_t> l = listaSymbolos(int symbolID, string symbolType=person,string linkType=RT);
	const AGMModelSymbol::SPtr &symbol = worldModel->getSymbol(roomID);
	QList<int32_t> lSymbolsPersons;
	
	for (AGMModelSymbol::iterator edge_itr=symbol->edgesBegin(worldModel); edge_itr!=symbol->edgesEnd(worldModel); edge_itr++)
	{
		//std::cout<<(*edge_itr).toString(worldModel)<<"\n";
		//comprobamos el id del simbolo para evitar los arcos que le llegan y seguir solo los que salen del nodo
		if ((*edge_itr)->getLabel() == "RT" && (*edge_itr)->getSymbolPair().first==roomID )
		{
			int second = (*edge_itr)->getSymbolPair().second;
			const AGMModelSymbol::SPtr &symbolSecond=  worldModel->getSymbolByIdentifier(second);
			if(symbolSecond->symbolType=="person")
			{
				try
				{
					string trackingId = symbolSecond->getAttribute("TrackingId");
					lSymbolsPersons.append(str2int(trackingId));
					
				}
				catch(...)
				{
					std::cout<<__FUNCTION__<<" es una persona Sin TrackinID"<<symbolSecond->toString()<<"\n";
				}
			}
		}
	}
	
	qDebug()<<"lsymbols person:"<<lSymbolsPersons;
	//Contienen el id de las personas. un entero.
	QList<int32_t> lInsertions;
	QList<int32_t> lUpdates;
	for( auto mapIt : innerModelMap )
	{		
		if (lSymbolsPersons.contains(mapIt.first))
		{
			qDebug()<<__FILE__<<__FUNCTION__<<__LINE__<<"mappIt.first"<<mapIt.first;						
			lUpdates.append(mapIt.first);
			lSymbolsPersons.removeOne(mapIt.first);
		}
		else
		{
			lInsertions.append(mapIt.first);
			
		}
	}
	//remove
	qDebug()<<"lsymbols person to remove:"<<lSymbolsPersons;
	for (int i=0; i<lSymbolsPersons.size(); i++)
	{	
	  
		int symbolID = AgmInner::findSymbolIDWithInnerModelName(worldModel, QString::fromStdString(int2str(lSymbolsPersons.at(i)))+"XN_SKEL_TORSO");
		qDebug()<<__FUNCTION__<<__LINE__<<innerModelMap.size()<<"lSymbolsPersons.at(i)"<<lSymbolsPersons.at(i)<<"symbol"<<symbolID;
		//CAUTION el vector innermodelMap no contiene nada en esa posición
		//agmInner.remove_Im(innerModelMap.at(lSymbolsPersons.at(i)));
		QList <int> listaDescendientes;
		bool loop=false;	
		AgmInner::checkLoop(worldModel, symbolID,listaDescendientes,"RT",loop);
// 		
		//qDebug()<<"listaDescendientes"<<listaDescendientes;
		for (int j=0; j<listaDescendientes.size();j++)
		    worldModel->removeSymbol(listaDescendientes.at(j));
		modification = true;
		
		
	}
	//add, 
	qDebug()<<"lInsertions"<<lInsertions;
	for (int i=0; i<lInsertions.size(); i++)
	{
		AGMModelSymbol::SPtr newSymbolPerson =worldModel->newSymbol("person");
		AGMModelSymbol::SPtr typeSymbolPerson =worldModel->newSymbol("unknownPerson");
		int personID = newSymbolPerson->identifier;
		
		int TrackingId = lInsertions.at(i);
		newSymbolPerson->setAttribute("TrackingId",int2str(TrackingId));
		std::cout<<" añado un nuevo symbolo persona "<<newSymbolPerson->toString()<<" TrackingID "<<TrackingId<<"\n";
		
		//state está en personList
		try
		{
			int state = personList.at(TrackingId).state;
			newSymbolPerson->setAttribute("State",int2str(state));
// 			newSymbolPerson->setAttribute("Red",int2str(personList.at(TrackingId).spineJointColor.R));
// 			newSymbolPerson->setAttribute("Green",int2str(personList.at(TrackingId).spineJointColor.G));
// 			newSymbolPerson->setAttribute("Blue",int2str(personList.at(TrackingId).spineJointColor.B));
			
		}

		catch (const std::out_of_range& oor)
		{
			qDebug()<<"PersonList at exception";
			std::cerr << "Out of Range error: " << oor.what() << '\n';
			continue;
		}
		
		
		//añado su arco calculado para innerModel, de la matzi kinect a la persona
		QString pre =QString::number(TrackingId);
		std::map<string,string>att;
		try
		{
			try
			{
				att["tx"]=float2str(innerModelMap.at(TrackingId)->getTransform(pre+"XN_SKEL_TORSO")->getTr().x());				
				att["ty"]=float2str(innerModelMap.at(TrackingId)->getTransform(pre+"XN_SKEL_TORSO")->getTr().y());
				att["tz"]=float2str(innerModelMap.at(TrackingId)->getTransform(pre+"XN_SKEL_TORSO")->getTr().z());
				att["rx"]=float2str(innerModelMap.at(TrackingId)->getTransform(pre+"XN_SKEL_TORSO")->getRxValue());
				att["ry"]=float2str(innerModelMap.at(TrackingId)->getTransform(pre+"XN_SKEL_TORSO")->getRyValue());
				att["rz"]=float2str(innerModelMap.at(TrackingId)->getTransform(pre+"XN_SKEL_TORSO")->getRzValue());
				worldModel->addEdgeByIdentifiers(roomID,personID,"RT",att);
				worldModel->addEdgeByIdentifiers(personID,roomID,"in");
				worldModel->addEdgeByIdentifiers(personID,typeSymbolPerson->identifier,"personIs");
			}
			catch (const std::out_of_range& oor)
			{	
				qDebug()<<"at exception InnerModelMap";
				std::cerr << "Out of Range error: " << oor.what() << '\n';			
				continue;
			}
			
		}
		catch (...)
		{
			qDebug()<<"si no existe el torso q qFatal"<<__FUNCTION__<<__LINE__;
			qFatal("abort fary");
		}
		//el prefijo es el ID del tracking, pero cuelga/inicia en la persona. Debo crear el arco entre room--RT-->person
		QHash<QString, int32_t>  match;
		
		//match.insert(pre+"XN_SKEL_TORSO",personID);
		//agmInner.include_im(match, innerModelMap.at(TrackingId));
		AgmInner::includeInnerModel(worldModel,personID,innerModelMap.at(TrackingId));
		modification =true;
	}
	qDebug()<<"lUpdates:"<<lUpdates;
// 	//update, si no hay nada que borrar ni insertar. puedo usar el que contiene todos
	for (int i=0; i<lUpdates.size(); i++)
	{
		try
		{
			InnerModel* imTmp =innerModelMap.at(lUpdates.at(i));
			//agmInner.updateAgmWithInnerModelAndPublish(imTmp,agmexecutive_proxy);
			AgmInner::updateAgmWithInnerModelAndPublish(worldModel, imTmp, agmexecutive_proxy);
		}
		catch (const std::out_of_range& oor)
		{	
			qDebug()<<"at exception InnerModelMap"<<__FUNCTION__<<__LINE__;
			std::cerr << "Out of Range error: " << oor.what() << '\n';			
			continue;
		}
		
		int TrackingId = lUpdates.at(i);
		int symbolID = agmInner.findSymbolIDWithInnerModelName(worldModel,QString::fromStdString(int2str(TrackingId))+"XN_SKEL_TORSO");
		
		AGMModelSymbol::SPtr symbolPerson = worldModel->getSymbol(symbolID);
		//rgb color
// 		symbolPerson->setAttribute("Red",int2str(personList.at(TrackingId).spineJointColor.R));
// 		symbolPerson->setAttribute("Green",int2str(personList.at(TrackingId).spineJointColor.G));
// 		symbolPerson->setAttribute("Blue",int2str(personList.at(TrackingId).spineJointColor.B));
		
// 		========================== MARIO =======================
		//int b = personList.at(TrackingId).spineJointColor.B;
// 		QVec colorRGB = QVec::vec3(personList.at(TrackingId).spineJointColor.R,
// 					 personList.at(TrackingId).spineJointColor.G,
// 					 personList.at(TrackingId).spineJointColor.B);

		RgbColor colorRGB;
// 		colorRGB.r = personList.at(TrackingId).spineJointColor.R;
// 		colorRGB.g = personList.at(TrackingId).spineJointColor.G;
// 		colorRGB.b = personList.at(TrackingId).spineJointColor.B;
		HsvColor colorHSV = rgb2hsv(colorRGB);
		
                
// 		cv::Mat hsv_image;
// 		cv::cvtColor(bgr_image, hsv_image, cv::COLOR_BGR2HSV);
// 		// Threshold the HSV image, keep only the red pixels
// 		cv::Mat lower_red_hue_range;
// 		cv::Mat upper_red_hue_range;
// 		cv::inRange(hsv_image, cv::Scalar(0, 100, 100), cv::Scalar(10, 255, 255), lower_red_hue_range);
// 		cv::inRange(hsv_image, cv::Scalar(160, 100, 100), cv::Scalar(179, 255, 255), upper_red_hue_range);
                
                
                
                
                
                
		std::cout<<"---------------------------"<<std::endl;
// 		std::cout<<"rgb("<<int2str(personList.at(TrackingId).spineJointColor.R)<<","<<int2str(personList.at(TrackingId).spineJointColor.G)<<","<<int2str(personList.at(TrackingId).spineJointColor.B)<<")"<<std::endl;
		std::cout<<"hsv("<<int2str(colorHSV.h)<<","<<int2str(colorHSV.s)<<","<<int2str(colorHSV.v)<<")"<<std::endl;
		std::cout<<"---------------------------"<<std::endl;
		
		string colorName = getColorName(colorHSV);
		colorName="Blue";
		cout << colorName << endl;
		symbolPerson->setAttribute("Color",colorName);
		
		//state
		int state = personList.at(TrackingId).state;
		symbolPerson->setAttribute("State",int2str(state));
		
		
		//funcion para cambiar typeSymbolPerson
		int idTypeSymbolPerson=-1;
		for (AGMModelSymbol::iterator  edge_itr=symbolPerson->edgesBegin(worldModel); edge_itr!=symbolPerson->edgesEnd(worldModel); edge_itr++)
		{
			
			//comprobamos el id del simbolo para evitar los arcos que le llegan y seguir solo los que salen del nodo
			if ((*edge_itr)->getLabel() == "personIs"  && ( (*edge_itr)->symbolPair.first==symbolPerson->identifier) )
			{				
				std::cout<<"\n\t"<<(*edge_itr).toString(worldModel)<<"\n"	;
				idTypeSymbolPerson = (*edge_itr)->symbolPair.second;				
				break;
			}
		}
		if (idTypeSymbolPerson != -1)
		{
			AGMModelSymbol::SPtr typeSymbolPerson = worldModel->getSymbol(idTypeSymbolPerson);
			
			//AQUI hacer operaciones en base al filtro colorName						
		
			if (colorName == "Blue" && typeSymbolPerson->typeString() !="extranger")
			{
				qDebug()<<"publish typeSymbolPerson->setType( extranger )"<< "idTypeSymbolPerson" <<idTypeSymbolPerson;	
				typeSymbolPerson->setType("extranger");							
				AGMMisc::publishNodeUpdate(typeSymbolPerson,agmexecutive_proxy);
				usleep(500);
			}	
// 			else
// 			{
// 				typeSymbolPerson->setType("unknownPerson");
// 				AGMMisc::publishNodeUpdate(typeSymbolPerson,agmexecutive_proxy);
// 				usleep(500);
// 			}
			
		}
		AGMMisc::publishNodeUpdate(symbolPerson,agmexecutive_proxy);
		
	}
	qDebug()<<"\n ********** \n";
	if (modification)
	{
		qDebug()<<"----------- MODIFICATION -----------------------";
 		AGMModel::SPtr newModel(new AGMModel(worldModel));	
 		sendModificationProposal(worldModel, newModel);					
		//saveInnerModels(QString::number(number));
		number++;
		//if (number>1)
			//qFatal("fary");
	}
}



string SpecificWorker::getColorName(HsvColor hsv)
{
	//FILTRO CON EL STRUCT HSV
	return "INSERTAR EL FILTRO";
}

/**
 * Converts an RGB color value to HSV. Conversion formula
 * adapted from http://en.wikipedia.org/wiki/HSV_color_space.
 * Assumes r, g, and b are contained in the set [0, 255] and
 * returns h, s, and v in the set [0, 1].
 *
 * @param   Number  r       The red color value
 * @param   Number  g       The green color value
 * @param   Number  b       The blue color value
 * @return  Array           The HSV representation
 */
HsvColor SpecificWorker::rgb2hsv(RgbColor rgb)
{
	HsvColor hsv;
	double rd = (double) rgb.r/255;
	double gd = (double) rgb.g/255;
	double bd = (double) rgb.b/255;
	double maxN = max(rd, max(gd, bd));
	double minN = min(rd, min(gd, bd));
	double h, s, v = maxN;

	double d = maxN - minN;
	s = maxN == 0 ? 0 : d / maxN;

	if (maxN == minN) { 
	h = 0; // achromatic
	} else {
	if (maxN == rd) {
		h = (gd - bd) / d + (gd < bd ? 6 : 0);
	} else if (maxN == gd) {
		h = (bd - rd) / d + 2;
	} else if (maxN == bd) {
		h = (rd - gd) / d + 4;
	}
	h /= 6;
	}

	hsv.h = h*360;
	hsv.s = s*100;
	hsv.v = v*100;
	return hsv;
}


bool SpecificWorker::reloadConfigAgent()
{
	return true;
}

bool SpecificWorker::activateAgent(const ParameterMap &prs)
{
	bool activated = false;
	if (setParametersAndPossibleActivation(prs, activated))
	{
		if (not activated)
		{
			return activate(p);
		}
	}
	else
	{
		return false;
	}
	return true;
}

bool SpecificWorker::setAgentParameters(const ParameterMap &prs)
{
	bool activated = false;
	return setParametersAndPossibleActivation(prs, activated);
}

ParameterMap SpecificWorker::getAgentParameters()
{
	return params;
}

void SpecificWorker::killAgent()
{

}

int SpecificWorker::uptimeAgent()
{
	return 0;
}

bool SpecificWorker::deactivateAgent()
{
	return deactivate();
}

StateStruct SpecificWorker::getAgentState()
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

void SpecificWorker::structuralChange(const RoboCompAGMWorldModel::World &modification)
{
	QMutexLocker m (mutex);	
 	AGMModelConverter::fromIceToInternal(modification, worldModel);
	
	if (innerModelAGM) 
		delete innerModelAGM;
	innerModelAGM = AgmInner::extractInnerModel(worldModel, "world");
	innerModelAGM->save("innerModelAGM.xml");
	///TEST to check extractAGM
	//AGMModel::SPtr worldClean = AgmInner::extractSymbolicGraph(worldModel);
	//worldClean->save("worldClean.xml");
	//AGMModelPrinter::printWorld(worldClean);
}

void SpecificWorker::edgesUpdated(const RoboCompAGMWorldModel::EdgeSequence &modifications)
{
	QMutexLocker lockIM(mutex);
	
	for (auto modification : modifications)
	{
		AGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);
		AGMModelEdge dst;
		AGMModelConverter::fromIceToInternal(modification,dst);
		AgmInner::updateImNodeFromEdge(worldModel, dst, innerModelAGM);
	}
}

void SpecificWorker::edgeUpdated(const RoboCompAGMWorldModel::Edge &modification)
{
	QMutexLocker m (mutex);	
  	AGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);
 	AgmInner::updateImNodeFromEdge(worldModel,modification,innerModelAGM);
 
}

void SpecificWorker::symbolUpdated(const RoboCompAGMWorldModel::Node &modification)
{
	QMutexLocker m (mutex);	
 	AGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);		
}

void SpecificWorker::symbolsUpdated(const RoboCompAGMWorldModel::NodeSequence &modifications)
{
	QMutexLocker m (mutex);
	for (auto modification : modifications)
		AGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);		
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
	QMutexLocker m (mutex);	
	try
	{
// 		AGMModelPrinter::printWorld(newModel);
		AGMMisc::publishModification(newModel, agmexecutive_proxy, "humanCompAgent");
	}
	catch(Ice::Exception e)
	{
		std::cout<<"que pasa amigo" << e.what()<<"\n";
		exit(1);
	}
	catch(...)
	{
		qDebug()<<"que pasa amigo";
		exit(1);
	}
  
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{


//       THE FOLLOWING IS JUST AN EXAMPLE for AGENTS
// 	try
// 	{
// 		RoboCompCommonBehavior::Parameter par = params.at("NameAgent.InnerModel") ;
// 		if( QFile(QString::fromStdString(par.value)).exists() == true)
// 		{
// 			qDebug() << __FILE__ << __FUNCTION__ << __LINE__ << "Reading Innermodel file " << QString::fromStdString(par.value);
// 			innerModel = new InnerModel(par.value);
// 			qDebug() << __FILE__ << __FUNCTION__ << __LINE__ << "Innermodel file read OK!" ;
// 		}
// 		else
// 		{
// 			qDebug() << __FILE__ << __FUNCTION__ << __LINE__ << "Innermodel file " << QString::fromStdString(par.value) << " does not exists";
// 			qFatal("Exiting now.");
// 		}
// 	}
// 	catch(std::exception e)
// 	{
// 		qFatal("Error reading config params");
// 	}

	
	timer.start(Period);

	return true;
}


void SpecificWorker::initDictionary()
{
	QMutexLocker m (mutex);	
	/*Cargamos el mapa de  nombres*/
	dictionaryNames[ "Spine" ] = "XN_SKEL_TORSO";
	dictionaryNames[ "Head" ] = "XN_SKEL_NECK";
	dictionaryNames[ "ShoulderLeft" ] = "XN_SKEL_LEFT_SHOULDER";
	dictionaryNames[ "ShoulderRight" ] = "XN_SKEL_RIGHT_SHOULDER";
	dictionaryNames[ "ElbowLeft" ] = "XN_SKEL_LEFT_ELBOW";
	dictionaryNames[ "ElbowRight" ] = "XN_SKEL_RIGHT_ELBOW";
// 	dictionaryNames[ "HipCenter" ] = "XN_SKEL_WAIST";
// 	dictionaryNames[ "HipLeft" ] = "XN_SKEL_LEFT_HIP";
// 	dictionaryNames[ "HipRight" ] = "XN_SKEL_RIGHT_HIP";
// 	dictionaryNames[ "KneeLeft" ] = "XN_SKEL_LEFT_KNEE";
// 	dictionaryNames[ "KneeRight" ] = "XN_SKEL_RIGHT_KNEE";
	dictionaryNames[ "HandRight" ] = "XN_SKEL_RIGHT_HAND";
// 	dictionaryNames[ "FootLeft" ] = "XN_SKEL_LEFT_FOOT";
// 	dictionaryNames[ "FootRight" ] = "XN_SKEL_RIGHT_FOOT";
	dictionaryNames[ "HandLeft" ] = "XN_SKEL_LEFT_HAND";
// 	dictionaryNames[ "WallPose" ] = "WALL_POSE";
// 	dictionaryNames[ "Wall" ] = "XN_SUCCESS_WALL";

	dictionaryEnum[ "Spine" ] = Spine;
	dictionaryEnum[ "Head" ] = Head;
	dictionaryEnum[ "ShoulderLeft" ] = ShoulderLeft;
	dictionaryEnum[ "ShoulderRight" ] = ShoulderRight;
	dictionaryEnum[ "ElbowLeft" ] = ElbowLeft;
	dictionaryEnum[ "ElbowRight" ] = ElbowRight;
	dictionaryEnum[ "HipCenter" ] = HipCenter;
	dictionaryEnum[ "HipLeft" ] = HipLeft;
	dictionaryEnum[ "HipRight" ] = HipRight;
	dictionaryEnum[ "KneeLeft" ] = KneeLeft;
	dictionaryEnum[ "HandRight" ] = HandRight;
	dictionaryEnum[ "KneeRight" ] = KneeRight;
	dictionaryEnum[ "FootLeft" ] = FootLeft;
	dictionaryEnum[ "FootRight" ] = FootRight;
	dictionaryEnum[ "HandLeft" ] = HandLeft;
}


////************** INNERMODEL ******************++
 void SpecificWorker::updateInnerModel( TPerson &person, int idPerson )
{
	QMutexLocker m (mutex);	
	float x,y,z,rx,ry,rz;
	string idJoint;
	
// 	qDebug()<<"RGB("<<person.spineJointColor.R<<person.spineJointColor.G<<person.spineJointColor.B<<") (X,Y"<<person.spineJointColor.X<<person.spineJointColor.Y<<")";
	
	calculateJointRotations(person);
	
	for( auto dictionaryNamesIt : dictionaryNames )
	{
		try
		{
			idJoint = dictionaryNamesIt.first;
			x=y=z=rx=ry=rz=0.0;
			
			x = 1000*mapJointRotations[ idJoint ].getTr().x();
			y = 1000*mapJointRotations[ idJoint ].getTr().y();
			z = 1000*mapJointRotations[ idJoint ].getTr().z();		
			
			QVec rot =mapJointRotations[ idJoint ].extractAnglesR_min();
			rx = rot(0);
			ry = rot(1);
			rz = rot(2);
				
			
			//nan check			
			if ( (x!=x) or (y!=y) or (z!=z) or (rx!=rx) or (ry!=ry) or (rz!=rz) )
				continue;
			string idNode = int2str(idPerson) + dictionaryNamesIt.second.toStdString();			
			
			
			
			
			if (idJoint=="Spine")
			{
				innerModelMap[ idPerson ]->updateTransformValues( QString::fromStdString(idNode),x,y,z,rx,ry,rz );
				qDebug()<<"idNode"<<QString::fromStdString(idNode)<<"idjoint"<<QString::fromStdString( idJoint)<<"[" <<x<<y<<z<<"( "<<rx<<ry<<rz<<" )";
				try
				{
					innerModelsLocals->updateTransformValues(QString::fromStdString(idNode),x,y,z,rx,ry,rz);
				}
				catch (...)
				{
					qDebug()<<"Exception, probably innerModelLocals, has not been created yet";
				}
			}
			else
			{
				innerModelMap[ idPerson ]->updateRotationValues( QString::fromStdString(idNode),rx,ry,rz );						
				try
				{
					innerModelsLocals->updateRotationValues( QString::fromStdString(idNode),rx,ry,rz );
				}
				catch (...)
				{
					qDebug()<<"Exception, probably innerModelLocals, has not been created yet";
				}
			}
		}
		catch ( Ice::Exception e ) 
		{
			qDebug( )<<"error updateInnerModel"<<e.what( );
		}
	}
	

	const float fScale = 0.83;
	QString torsoName= QString::fromStdString(int2str(idPerson)) +"XN_SKEL_TORSO";
	QVec v = innerModelMap[ idPerson ]->getTransform(torsoName)->getTr();
	float rxValue= innerModelMap[ idPerson ]->getTransform(torsoName)->extractAnglesR_min().x();
	float ryValue= innerModelMap[ idPerson ]->getTransform(torsoName)->extractAnglesR_min().y();
	float rzValue= innerModelMap[ idPerson ]->getTransform(torsoName)->extractAnglesR_min().z();
	
	v.print("v");
	v(0)*=fScale;v(1)*=fScale;v(2)*=fScale;
	QVec ff = innerModelAGM->transform6D("room",QVec::vec6 ( v.x(),v.y(),v.z(), rxValue,ryValue,rzValue),"rgbdHumanPose");
	
	ff.print("ff");
	innerModelMap[ idPerson ]->updateTransformValues( torsoName,ff(0),ff(1),ff(2),ff(3),ff(4),ff(5) );
	innerModelsLocals->updateTransformValues( torsoName,ff.x(),ff.y(),ff.z(),ff.rx(),ff.ry(),ff.rz() );

}
 /**
 * @brief Given a person, this method calculates the Rotation Matrix of all the Joints. person is in metres
 * 
 * @param person Person who rotation matrix is calculated
 * @return void
 */
 void SpecificWorker::calculateJointRotations( TPerson &person )
{
	RTMat kinect;
	RoboCompMSKBody::JointList jointList;
	
	jointList = person.joints;
// 	qDebug()<<"person.TrackingId"<<person.TrackingId;
// 	qDebug()<<"person.state"<<person.state;
	//no se
	for (auto iter : jointList )
	{
// 		qDebug()<<iter.first<<iter.second.Position.X;
		jointList[iter.first].Position.X = -1.*jointList[iter.first].Position.X;
// 		qDebug()<<jointList[iter.first].Position.X;
	}

	///*********************** TODO transform ***************************	
// 	try
// 	{		
// 		//kinect=innerModelAGM->getTransformationMatrix("rgbdHumanPose","room");
// 		kinect=innerModelAGM->getTransformationMatrix("room","rgbdHumanPose");				
// 		kinect.getTr().print("Kinect: TR");
// 		kinect.extractAnglesR_min().print("extractAnglesR_min");
// 		//qDebug()<<"["<< kinect.getRxValue()<<kinect.getRyValue()<<kinect.getRzValue()<<"]";
// 		kinect.print("kinect matrix");
// 		kinect.setTr(kinect.getTr().x()/1000.0, kinect.getTr().y()/1000.0,kinect.getTr().z()/1000.0);
// 
// 	}
// 	catch (...)
// 	{
// 		qDebug()<<"not found kinect using identity RTMat kinect";
// 	}
	
	/// apunta el torso (inclinación alante/atrás y lateral del torso)
	mapJointRotations[ "Spine" ]=
	rtMatFromJointPosition( kinect,
			      jointList[ dictionaryEnum[ "Spine" ] ].Position,
			      jointList[ dictionaryEnum[ "Head" ] ].Position, 
			      jointList[ dictionaryEnum[ "Spine" ] ].Position, 2 );			

	/// alineación de hombros (rotación en Z del torso), previa al cálculo de la transformacion final de los hombros. 
	RTMat LEFT_SHOULDER_PRE_Z=
	rtMatFromJointPosition( mapJointRotations[ "Spine" ],
			      jointList[ dictionaryEnum[ "ShoulderLeft" ] ].Position,
			      jointList[ dictionaryEnum[ "ElbowLeft" ] ].Position, 
			      jointList[ dictionaryEnum[ "ShoulderLeft" ] ].Position, 2 );
	RTMat RIGHT_SHOULDER_PRE_Z=
	rtMatFromJointPosition( mapJointRotations[ "Spine" ],
			      jointList[ dictionaryEnum[ "ShoulderRight" ] ].Position,
			      jointList[ dictionaryEnum[ "ElbowRight" ] ].Position, 
			      jointList[ dictionaryEnum[ "ShoulderRight" ] ].Position, 2 );
	
	rotarTorso ( LEFT_SHOULDER_PRE_Z.getTr( ), RIGHT_SHOULDER_PRE_Z.getTr( ) );
// 	rotarTorso ( RIGHT_SHOULDER_PRE_Z.getTr( ), LEFT_SHOULDER_PRE_Z.getTr( ) );
	
	

	///Cintura
// 	mapJointRotations[ "HipCenter" ]= 
// 	rtMatFromJointPosition( mapJointRotations[ "Spine" ],
// 			      jointList[ dictionaryEnum[ "HipCenter" ] ].Position,
// 			      jointList[ dictionaryEnum[ "Spine" ] ].Position,
// 			      jointList[ dictionaryEnum[ "HipCenter" ] ].Position, 2 );

	mapJointRotations[ "HipRight" ]=
	rtMatFromJointPosition( mapJointRotations[ "Spine" ],
			      jointList[ dictionaryEnum[ "HipLeft" ] ].Position,
			      jointList[ dictionaryEnum[ "KneeLeft" ] ].Position,
			      jointList[ dictionaryEnum[ "HipLeft" ] ].Position, 2);
	
	mapJointRotations[ "HipLeft" ]=
	rtMatFromJointPosition( mapJointRotations[ "Spine" ],
			      jointList[ dictionaryEnum[ "HipRight" ] ].Position,
			      jointList[ dictionaryEnum[ "KneeRight" ] ].Position,
			      jointList[ dictionaryEnum[ "HipRight" ] ].Position, 2);
	
	///Knee			
	mapJointRotations[ "KneeRight" ]=
	rtMatFromJointPosition( mapJointRotations[ "Spine" ]*mapJointRotations[ "HipRight" ],
			      jointList[ dictionaryEnum[ "KneeLeft" ] ].Position,
			      jointList[ dictionaryEnum[ "FootLeft" ] ].Position,
      			      jointList[ dictionaryEnum[ "KneeLeft" ] ].Position, 2);
				
	mapJointRotations[ "KneeLeft" ]=
	rtMatFromJointPosition( mapJointRotations[ "Spine" ]*mapJointRotations[ "HipLeft" ],
			      jointList[ dictionaryEnum[ "KneeRight" ] ].Position,
			      jointList[ dictionaryEnum[ "FootRight" ] ].Position,
      			      jointList[ dictionaryEnum[ "KneeRight" ] ].Position, 2);


//     ///cabeza
// 	mapJointRotations[ "Head" ]=
// 	rtMatFromJointPosition( mapJointRotations[ "Spine" ],
// 				jointList[ dictionaryEnum[ "Head" ] ].Position,
// 				joints[ "JOINT_HEAD" ],
// 				joints[ "JOINT_NECK" ], 2);


	///brazo izquierdo
	mapJointRotations[ "ShoulderRight" ]=
	rtMatFromJointPosition( mapJointRotations[ "Spine" ],
			      jointList[ dictionaryEnum[ "ShoulderLeft" ] ].Position,
			      jointList[ dictionaryEnum[ "ElbowLeft" ] ].Position,
			      jointList[ dictionaryEnum[ "ShoulderLeft" ] ].Position, 2);		
			    
	mapJointRotations[ "ElbowRight" ]=
	rtMatFromJointPosition( mapJointRotations[ "Spine" ]*mapJointRotations[ "ShoulderRight" ],
			      jointList[ dictionaryEnum[ "ElbowLeft" ] ].Position,
			      jointList[ dictionaryEnum[ "HandLeft" ] ].Position,
			      jointList[ dictionaryEnum[ "ElbowLeft" ] ].Position, 2);		


	///brazo derecho
	///codo al hombro (p2 inicio p1 final)
	mapJointRotations[ "ShoulderLeft" ]=
	rtMatFromJointPosition( mapJointRotations[ "Spine" ],
			      jointList[ dictionaryEnum[ "ShoulderRight" ] ].Position,
			      jointList[ dictionaryEnum[ "ElbowRight" ] ].Position,
			      jointList[ dictionaryEnum[ "ShoulderRight" ] ].Position, 2);		
			    
	mapJointRotations[ "ElbowLeft" ]=
	rtMatFromJointPosition( mapJointRotations[ "Spine" ]*mapJointRotations[ "ShoulderLeft" ],
			      jointList[ dictionaryEnum[ "ElbowRight" ] ].Position,
			      jointList[ dictionaryEnum[ "HandRight" ] ].Position,
			      jointList[ dictionaryEnum[ "ElbowRight" ] ].Position, 2);		
 
	///Manos derecha e izquierda
	mapJointRotations[ "HandLeft" ]=
	rtMatFromJointPosition( mapJointRotations[ "Spine" ]*mapJointRotations[ "ShoulderLeft" ]*mapJointRotations[ "ElbowLeft" ],
			      jointList[ dictionaryEnum[ "HandRight" ] ].Position,
			      jointList[ dictionaryEnum[ "ElbowRight" ] ].Position,
			      jointList[ dictionaryEnum[ "HandRight" ] ].Position, 2);
	
	mapJointRotations[ "HandRight" ]=
	rtMatFromJointPosition( mapJointRotations[ "Spine" ]*mapJointRotations[ "ShoulderRight" ]*mapJointRotations[ "ElbowRight" ],
			      jointList[ dictionaryEnum[ "HandLeft" ] ].Position,
			      jointList[ dictionaryEnum[ "ElbowLeft" ] ].Position,
			      jointList[ dictionaryEnum[ "HandLeft" ] ].Position, 2);

	///Manos derecha e izquierda
	mapJointRotations[ "FootLeft" ]=
	rtMatFromJointPosition( mapJointRotations[ "Spine" ]*mapJointRotations[ "HipLeft" ]*mapJointRotations[ "KneeLeft" ],
			      jointList[ dictionaryEnum[ "FootRight" ] ].Position,
			      jointList[ dictionaryEnum[ "KneeRight" ] ].Position,
			      jointList[ dictionaryEnum[ "FootRight" ] ].Position, 2);
	
     	mapJointRotations[ "FootRight" ]=
	rtMatFromJointPosition( mapJointRotations[ "Spine" ]*mapJointRotations[ "HipRight" ]*mapJointRotations[ "KneeRight" ],
			      jointList[ dictionaryEnum[ "FootLeft" ] ].Position,
			      jointList[ dictionaryEnum[ "KneeLeft" ] ].Position,
			      jointList[ dictionaryEnum[ "FootLeft" ] ].Position, 2);
}
 
 /**
  * @brief This method calculates the rotation of a Joint given some points.
  * 
  * @param rS Indicates the rotations of the above joints. (Example: To calculate the rotation of the ElbowLeft we must know the rotation of the ShoulderLeft).
  * @param p1 Starting point of the Joint (Example: ElbowLeft)
  * @param p2 Ending point of the Joint (Example: WristLeft)
  * @param translation Traslation of the current Joint (Example: ElbowLeft)
  * @param axis Always axis Z (2)
  * @return RMat::RTMat This is the Rotation Matrix
  */
 RTMat SpecificWorker::rtMatFromJointPosition( RTMat rS, RoboCompMSKBody::SkeletonPoint p1, RoboCompMSKBody::SkeletonPoint p2, RoboCompMSKBody::SkeletonPoint translation, int axis )
{
   	bool XClockWise=true, YClockWise=true, ZClockWise=true;
	float alpha, beta, gamma;
	
	RTMat rt(XClockWise,YClockWise, ZClockWise);
	QVec p1h = QVec::vec4(p1.X, p1.Y, p1.Z, 1);
	QVec p2h = QVec::vec4(p2.X, p2.Y, p2.Z,1);
	QVec translationH = QVec::vec4(translation.X, translation.Y, translation.Z,1);
	
	QMat aux =rS;
	aux = aux.invert();
	QVec translationT = aux *translationH;
	QVec p1t = aux * p1h;
	QVec p2t = aux * p2h;
	QVec v = p2t - p1t;
	
	v= v.normalize();
	
	  ///por filas
	switch(axis){
	case 0:
		alpha = 0;
		
		if(YClockWise) beta = atan2(-v.z(),v.x());
		else beta = atan2(v.z(),v.x());
		
		if(ZClockWise) gamma = asin(-v.y());
		else gamma = asin(v.y());
		
		break;
	case 1:
		if(XClockWise) alpha = atan2(v.z(),v.y());
		else alpha = atan2(-v.z(),v.y());
		
		beta = 0;
		
		if(ZClockWise) gamma = asin(v.x());
		else gamma = asin(-v.x());
		
		break;
	case 2:
		if(XClockWise) alpha =  atan2(-v.y(),v.z());
		else alpha =  atan2(v.y(),v.z());
		
		if(YClockWise) beta = asin(v.x());
		else beta = asin(-v.x());
		
		gamma = 0;
		
		break;
	}
	
	rt.setRT(alpha,beta,gamma,translationT);
	
	return rt;
}
 
 
 /**
  * @brief This method allows to rotate the torso from the position and rotation of the shoulders
  * 
  * @param hombroizq Rotation Matrix from LeftShoulder
  * @param hombroder Rotation Matrix from RightShoulder
  * @return true if all the operations went correctly
  */
 bool SpecificWorker::rotarTorso( const QVec & hombroizq, const QVec & hombroder )
{
	QVec eje=hombroizq - hombroder; //Calculamos el eje que va de un hombro a otro
	
	eje.normalize ();

	if( eje.x( ) == 0 ) return false;

	float angulo = atan2( eje.y( ),eje.x( ) );	//Calculamos el giro necesario para alinear los hombros con el eje
	
// 	QVec rrobot = QVec::vec3(0,0,0);
// 	try
// 	{
// 		rrobot = innerModelAGM->getRotationMatrixTo("rgbdHumanPose","room").extractAnglesR_min();
// 		
// 		
// 	}
// 	catch (...)
// 	{
// 	}
// 	rrobot.print("\nrotacion del robot \n");
	//mapJointRotations[ "Spine" ].setRZ( angulo - rrobot(1) ); 
	
	mapJointRotations[ "Spine" ].setRZ( angulo ); // Aplicamos dicho giro al eje Z del torso
	
	
	return true;
}

void SpecificWorker::saveInnerModels(QString number)
{	
	worldModel->save(number.toStdString()+"_agmWorldModelLocal.xml");
// 	agmInner.getWorld()->save(number.toStdString()+"_agmWorldModel_agmInner.xml");	
	for( auto m : innerModelMap )			
	{		
		QString pre =QString::fromStdString(int2str(m.first));
		qDebug()<<"Saving innermodels : "<<pre+"innerHuman.xml";
		m.second->save(number+"_"+pre+"innerHuman.xml");			
	}		
	AgmInner::extractInnerModel(worldModel,"room", true)->save(number+"_extractInnerModelFromRoom.xml");
	
}

void SpecificWorker::updateViewer()
{	
// 	for( auto m : innerModelMap )			
// 	{	
// 		std::map<int,IMV>::iterator it;			
// 		it = InnerModelViewerMap.find(m.first);
// 		//si está actualizo
// 		if (it != InnerModelViewerMap.end())
// 		{						
// 			InnerModelViewerMap.at(m.first).innerViewer->update();
// 			InnerModelViewerMap.at(m.first).osgView->autoResize();						
// 			InnerModelViewerMap.at(m.first).osgView->frame();
// 			InnerModelViewerMap.at(m.first).widget->update();
// 			printf("Updated: %d\n", __LINE__);
// 			
// 		}
// 		//sino inserto
// 		else 
// 		{			
// 			printf("No Exist: %d\n", __LINE__);
// 			IMV myIMV;			
// 			InnerModelViewerMap.insert ( std::pair<int,IMV>(m.first,myIMV) );
// // 			myIMV.widget = new QWidget();
// // 			myIMV.widget->show();
// // 			myIMV.widget->resize(1024,768);
// // 			myIMV.widget->setWindowTitle("innerModel:"+QString::number(m.first));
// // 			myIMV.osgView = new OsgView(myIMV.widget);
// // 			myIMV.innerViewer = new InnerModelViewer(innerModelMap.at(m.first), "root", myIMV.osgView->getRootGroup(), true);
// // 			
// // 			myIMV.manipulator = new osgGA::TrackballManipulator;
// // 			printf("No Exist: %d\n", __LINE__);
// // 			myIMV.osgView->setCameraManipulator(myIMV.manipulator, true);
// // 			printf("No Exist: %d\n", __LINE__);
// // 			myIMV.innerViewer->setMainCamera(myIMV.manipulator, InnerModelViewer::TOP_POV);
// 			
// 			InnerModelViewerMap.at(m.first).widget = new QWidget();
// 			InnerModelViewerMap.at(m.first).widget->show();
// 			InnerModelViewerMap.at(m.first).widget->resize(1024,768);
// 			InnerModelViewerMap.at(m.first).widget->setWindowTitle("innerModel:"+QString::number(m.first));
// 			InnerModelViewerMap.at(m.first).osgView = new OsgView(InnerModelViewerMap.at(m.first).widget);
// 			InnerModelViewerMap.at(m.first).innerViewer = new InnerModelViewer(innerModelMap.at(m.first), "root", 
// 											   InnerModelViewerMap.at(m.first).osgView->getRootGroup(), true);
// 			
// 			InnerModelViewerMap.at(m.first).manipulator = new osgGA::TrackballManipulator;
// 			printf("No Exist: %d\n", __LINE__);
// 			InnerModelViewerMap.at(m.first).osgView->setCameraManipulator(InnerModelViewerMap.at(m.first).manipulator, true);
// 			printf("No Exist: %d\n", __LINE__);
// 			InnerModelViewerMap.at(m.first).innerViewer->setMainCamera(InnerModelViewerMap.at(m.first).manipulator, InnerModelViewer::TOP_POV);			
// 			
// 		}
// 	}		
}

void SpecificWorker::updateViewerLocalInnerModels()
{	
	QMutexLocker m (mutex);	
	qDebug()<<"-- init --";
	//borro si alguno en el map no esta en list		
	std::map<int,InnerModel*>::iterator it;
	std::map<int,TPerson>::iterator itPersonList;
	bool creatViewer=false;
	
	for( auto mapIt : innerModelMap )
	{
		itPersonList = personList.find(mapIt.first);
		//alguno en el mapa no está en la lista, tengo que borrar el viewer y crearlo;
		if (itPersonList == personList.end())
		{
			qDebug()<<__FILE__<<__FUNCTION__<<__LINE__<<"mappIt.first"<<mapIt.first;
			innerModelMap.erase (mapIt.first);			
			creatViewer = true;			
		}
	}
	
	//actgualizar y añadir	
	for( auto personIt : personList )
	{
		qDebug()<<__FILE__<<__FUNCTION__<<__LINE__<<"TPerson.TrackingId"<<personIt.second.TrackingId;
		
		int id = personIt.first;
		
		//lo busco 
		it = innerModelMap.find(id);
		//lo encuentro lo actualizo
		if (it != innerModelMap.end())
		{
			//qDebug()<<__FILE__<<__FUNCTION__<<__LINE__<<"id"<<id;
			updateInnerModel(personIt.second,id);			
		}
		//no lo encuentrr lo añado y tngo que crear el viewer
		else
		{
			//qDebug()<<__FILE__<<__FUNCTION__<<__LINE__<<"id"<<id;
			innerModelMap[id] =new InnerModel();
			QString pre =QString::fromStdString(int2str(id));
			newInnerModel(imHumanGeneric, innerModelMap.at(id),pre);		
			updateInnerModel(personIt.second,id);
			creatViewer = true;			
		}
		
		//to verify distance
// 		try
// 		{
// 			QVec v=	innerModelAGM->transform("rgbdHumanPose",QString::fromStdString(int2str(id))+"XN_SKEL_TORSO");
// 			qDebug()<<"$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$";
// 			qDebug()<<"\t"<<id<<"XN_SKEL_TORSO";
// 			qDebug()<<v.norm2();
// 			qDebug()<<"$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$";
// 		}
// 		catch (...)
// 		{
// 			qDebug()<<"not found node";
// 		}
	}

	if (creatViewer )
	{
#ifdef USE_QTGUI
		qDebug()<<__FUNCTION__<<__LINE__;
		osgView->getRootGroup()->removeChild(innerViewer);
		innerModelsLocals = new InnerModel();
#else
		delete innerModelsLocals;
		innerModelsLocals = new InnerModel();
#endif
		//añado al viewer innerModelMap.at(id)
		for( auto mapIt : innerModelMap )
		{
			QList<InnerModelNode *>	l;
			l.clear();
			mapIt.second->getSubTree(mapIt.second->getNode("root"),&l);		
			QList<InnerModelNode*>::iterator it;
			for (it=l.begin();it!=l.end();it++)
			{
				insertNodeInnerModel(innerModelsLocals,(*it));
			}
		}
#ifdef USE_QTGUI
		innerViewer = new InnerModelViewer(innerModelsLocals, "root", osgView->getRootGroup(), true);	
		innerViewer->setMainCamera(manipulator, InnerModelViewer::FRONT_POV);
#endif
	}
	qDebug()<<"creatViewer"<<creatViewer<<"-- end --";
}



///****** code for check innermodel memory
// 	std::ofstream myfile;
// 	myfile.open ("innerTestMesh.xml");
// 	myfile <<  "<innerModel>\n\n";
// 	for (int i=0;i<500;i++)
// 	{
// 		myfile <<"<transform id=\""<<QString::number(i).toStdString()<<"transform\">";	
// 		myfile <<"\n\t<mesh id=\"XN_SKEL_TORSO_mesh"<<QString::number(i).toStdString()
// 			<<"\" file=\"/home/robocomp/robocomp/files/osgModels/humanColor/pecho.osg\" scale=\"10\" rx=\"1.57\" />\n";		
// 		myfile <<"</transform>\n";
// 	}
// 	myfile <<  "\n</innerModel>";
// 	myfile.close();
// 	qFatal("fary");
	
// 	qDebug()<<"worldModel->numberOfSymbols()"<<worldModel->numberOfSymbols()<<"newBodyEvent"<<newBodyEvent;	
// 	InnerModel *innerModelTmp = new InnerModel("/home/luiky/robocomp/components/robocomp-ursus-rockin/components/humanAgent/etc/innerTestMesh.xml");
// 	InnerModel *innerModelTmp = new InnerModel("/home/robocomp/robocomp/components/robocomp-ursus-rockin/files/autonomyLab/autonomyLabModel.xml");
// 		usleep(10000);		
// 		qDebug()<<"11";
// 		delete innerModelTmp;
// 		qDebug()<<"22";
// 		usleep(20000);
// 		return;
// 	
// 	return;


void SpecificWorker::updateViewerLocalInnerModelSingle()
{
	QMutexLocker m (mutex);	
// 	qDebug()<<"-- init SINGLE --"<<"personList.empty()"<<personList.empty();
// 	qDebug()<<"innerModelMap.size()" <<innerModelMap.size();
// 	innerModelMap.at(idSingle)->save("idSingle");

/*	if (personList.empty())
	{
		return;
	}*/
	
	//update	
	//itPersonList = personList.cbegin();
		//we need to obtain the imName of the torso node. TrackingId+"XN_SKEL_TORSO"
	updateInnerModel((*personList.begin()).second,idSingle);


}

void SpecificWorker::updateHumanInnerFull()
{
	QMutexLocker m (mutex);
	int32_t robotID = worldModel->getIdentifierByType("robot");
	if (robotID < 0)
	{
		printf("Robot symbol not found, Waiting for the executive...\n");
		return;
	}
	bool modification = false;

	///Human hangs on room symbol
	int32_t roomID = AgmInner::findSymbolIDWithInnerModelName(worldModel,"room");
	if (roomID < 0)
	{
		printf("ROOOM symbol not found, \n");
		qFatal("abort");
		return;
	}
	
	if (personList.empty() )
		return;
	
	const AGMModelSymbol::SPtr &symbol = worldModel->getSymbol(roomID);
	
	//encontrar a la persona
	int symbolPersonID=worldModel->getIdentifierByType("person");
	
// 	//borrar
// 	qDebug()<<__FUNCTION__<<__LINE__<<"symbolID"<<symbolID<<"personList.empty()"<<personList.empty();	
// 	if (personList.empty() and symbolID!=-1)
// 	{
// // 		int symbolID = AgmInner::findSymbolIDWithInnerModelName(worldModel, QString::fromStdString(int2str(idSingle)+"XN_SKEL_TORSO"));		
// 		qDebug()<<__FUNCTION__<<__LINE__<<innerModelMap.size()<<"idSingle"<<idSingle<<"symbol"<<symbolID;
// 		
// 		//CAUTION el vector innermodelMap no contiene nada en esa posición
// 		//agmInner.remove_Im(innerModelMap.at(lSymbolsPersons.at(i)));
// 		QList <int> listaDescendientes;
// 		bool loop=false;	
// 		AgmInner::checkLoop(worldModel, symbolID,listaDescendientes,"RT",loop);
// 		for (int j=0; j<listaDescendientes.size();j++)
// 			worldModel->removeSymbol(listaDescendientes.at(j));
// 		modification = true;
// 	}
	
	
	qDebug()<<__FUNCTION__<<__LINE__<<"symbolID"<<symbolPersonID<<"personList.empty()"<<personList.empty();	
	//añadir
	if ( symbolPersonID == -1 )
	{
		AGMModelSymbol::SPtr newSymbolPerson =worldModel->newSymbol("person");		
		
		int personID = newSymbolPerson->identifier;
	
		newSymbolPerson->setAttribute("TrackingId",int2str(idSingle));
		std::cout<<" añado un nuevo symbolo persona "<<newSymbolPerson->toString()<<" TrackingID "<<idSingle<<"\n";
		
		//state está en personList
		try
		{
			int state = (*personList.begin()).second.state;
			newSymbolPerson->setAttribute("State",int2str(state));
		}

		catch (const std::out_of_range& oor)
		{
			qDebug()<<"PersonList at exception";
			std::cerr << "Out of Range error: " << oor.what() << '\n';			
		}
		
		try
		{
			try
			{
				worldModel->addEdgeByIdentifiers(roomID,personID,"RT");
				worldModel->addEdgeByIdentifiers(personID,roomID,"in");
				AgmInner::includeInnerModel(worldModel,personID,innerModelMap.at(idSingle));
				modification =true;
			}
			catch (const std::out_of_range& oor)
			{	
				qDebug()<<__FILE__<<__LINE__<<"at exception InnerModelMap";
				std::cerr << "Out of Range error: " << oor.what() << '\n';							
			}
		}
		catch (...)
		{
			qDebug()<<"Exception adding edges"<<__FUNCTION__<<__LINE__;
			qFatal("abort fary");
		}		
	}
	//update	
	else 
	{
		
		//update
		qDebug()<<"update Human in mission";
		try
		{
			InnerModel* imTmp =innerModelMap.at(idSingle);		
			AgmInner::updateAgmWithInnerModelAndPublish(worldModel, imTmp, agmexecutive_proxy);
		}
		catch (const std::out_of_range& oor)
		{	
			qDebug()<<"at exception InnerModelMap"<<__FUNCTION__<<__LINE__;
			std::cerr << "Out of Range error: " << oor.what() << '\n';				
		}
		
	}
	
	if (modification)
	{
		qDebug()<<"----------- MODIFICATION -----------------------";
 		AGMModel::SPtr newModel(new AGMModel(worldModel));	
 		sendModificationProposal(worldModel, newModel);					
		//saveInnerModels(QString::number(number));
		number++;
// 		if (number>2)
// 			qFatal("fary");
	}
}
