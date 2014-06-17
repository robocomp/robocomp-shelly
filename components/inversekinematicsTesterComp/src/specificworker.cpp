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
SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx)	
{	
	// Inicializamos las banderas de los targets a FALSE:
	banderaTrayectoria = banderaRCIS= false;
	
	// ÑAPA PARA PODER ENVIAR UN MISMO TARGET A DOS O TRES PARTES DEL CUERPO A LA VEZ
	// Por defecto siempre empezamos con una única parte del cuerpo.
	partesActivadas = QVec(3); //vector de partes
	partesActivadas[0] = 1; //activada
	partesActivadas[1] = 0; //desactivada
	partesActivadas[2] = 0; //desactivada.
	
	///////////// HACEMOS LAS CONEXIONES /////////////////
	//Conectamos botones de EJECUCIÓN:
	connect(rcisButton, SIGNAL(clicked()), this, SLOT(enviarRCIS()));
	connect(robotButton, SIGNAL(clicked()), this, SLOT(enviarROBOT()));
	connect(homePushButton, SIGNAL(clicked()), this, SLOT(enviarHome()));
	connect(stopButton, SIGNAL(clicked()), this, SLOT(stop()));
	
	//Esta señal la emite el QTabWidget cuando el usuario cambia el tab activo
	
	//Conectamos botones de la primera pestaña POSE6D:
	connect(camareroZurdoButton, SIGNAL(clicked()), this, SLOT(camareroZurdo()));
	connect(camareroDiestroButton, SIGNAL(clicked()), this, SLOT(camareroDiestro()));
	connect(camareroCentroButton, SIGNAL(clicked()), this, SLOT(camareroCentro()));
	connect(camareroCentro2Button, SIGNAL(clicked()), this, SLOT(camareroCentro2()));

	connect(Part1_pose6D, SIGNAL(clicked()), this, SLOT(activarDesactivar()));
	connect(Part2_pose6D, SIGNAL(clicked()), this, SLOT(activarDesactivar()));
	connect(Part3_pose6D, SIGNAL(clicked()), this, SLOT(activarDesactivar()));
	
	//TODO Conectamos los botones de la pestaña ALIGNAXIS
	connect(Part1_AxisAlign, SIGNAL(clicked()), this, SLOT(activarDesactivar()));
	connect(Part2_AxisAlign, SIGNAL(clicked()), this, SLOT(activarDesactivar()));
	connect(Part3_AxisAlign, SIGNAL(clicked()), this, SLOT(activarDesactivar()));
	

	//TODO Conectamos los botones de la pestaña MOVE ALONG AXIS
	connect(Part1_AlongAxis, SIGNAL(clicked()), this, SLOT(activarDesactivar()));
	connect(Part2_AlongAxis, SIGNAL(clicked()), this, SLOT(activarDesactivar()));
	connect(Part3_AlongAxis, SIGNAL(clicked()), this, SLOT(activarDesactivar()));
		
	osgView = new OsgView (frame);
	osgView->setCameraManipulator(new osgGA::TrackballManipulator); 	
	osgView->getCameraManipulator()->setHomePosition(osg::Vec3(0.,0.,-2.),osg::Vec3(0.,0.,4.),osg::Vec3(0.,1,0.));
	
// 	frameOsg->show();

}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{

}


bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	// ¡CUIDADO CON EL INNERMODEL! Debe ser el mismo que LOKIARM!!!
	try
	{
		RoboCompCommonBehavior::Parameter par = params.at("BIK.InnerModel") ;
		if( QFile(QString::fromStdString(par.value)).exists() == true)
		{
			qDebug() << __FILE__ << __FUNCTION__ << __LINE__ << "Reading Innermodel file " << QString::fromStdString(par.value);
			innerModel = new InnerModel(par.value);
			qDebug() << __FILE__ << __FUNCTION__ << __LINE__ << "Innermodel file read OK!" ;		
		}
		else
		{	qDebug() << __FILE__ << __FUNCTION__ << __LINE__ << "Innermodel file " << QString::fromStdString(par.value) << " does not exists";
			qFatal("Exiting now.");
		}
	}
	catch(std::exception e)
	{
		qFatal("Error reading config params");
	}

	try
	{
		motorparamList = jointmotor_proxy->getAllMotorParams();
		
		// Ponemos los datos de los límites min y max a TODOS los motores. (por eso lo apiñurgo un poco, porque quedaría
		// un tochaco de código insufrible --y aún así lo es--)
		foreach(RoboCompJointMotor::MotorParams motorParam, motorparamList)
		{
			
			motorList.push_back(motorParam.name);
			
			// BRAZO IZQUIERDO:
			if(motorParam.name == shoulder1Left->text().toStdString()){	angleMaxSL1->display(motorParam.maxPos);	angleMinSL1->display(motorParam.minPos);	velocityNewSL1->setValue(motorParam.maxVelocity);}
			if(motorParam.name == shoulder2Left->text().toStdString()){	angleMaxSL2->display(motorParam.maxPos);	angleMinSL2->display(motorParam.minPos);	velocityNewSL2->setValue(motorParam.maxVelocity);}
			if(motorParam.name == shoulder3Left->text().toStdString()){	angleMaxSL3->display(motorParam.maxPos);	angleMinSL3->display(motorParam.minPos);	velocityNewSL3->setValue(motorParam.maxVelocity);}
			if(motorParam.name == elbowLeft->text().toStdString()){		angleMaxEL->display(motorParam.maxPos);		angleMinEL->display(motorParam.minPos);		velocityNewEL->setValue(motorParam.maxVelocity);}
			if(motorParam.name == foreArmLeft->text().toStdString()){	angleMaxFAL->display(motorParam.maxPos);	angleMinFAL->display(motorParam.minPos);	velocityNewFAL->setValue(motorParam.maxVelocity);}
			if(motorParam.name == wristLeft1->text().toStdString()){	angleMaxWL1->display(motorParam.maxPos);	angleMinWL1->display(motorParam.minPos);	velocityNewWL1->setValue(motorParam.maxVelocity);}
			if(motorParam.name == wristLeft2->text().toStdString()){	angleMaxWL2->display(motorParam.maxPos);	angleMinWL2->display(motorParam.minPos);	velocityNewWL2->setValue(motorParam.maxVelocity);}
			
			// BRAZO DERECHO:	
			if(motorParam.name == shoulder1Right->text().toStdString()){angleMaxSR1->display(motorParam.maxPos);	angleMinSR1->display(motorParam.minPos);	velocityNewSR1->setValue(motorParam.maxVelocity);}
			if(motorParam.name == shoulder2Right->text().toStdString()){angleMaxSR2->display(motorParam.maxPos);	angleMinSR2->display(motorParam.minPos);	velocityNewSR2->setValue(motorParam.maxVelocity);}
			if(motorParam.name == shoulder3Right->text().toStdString()){angleMaxSR3->display(motorParam.maxPos);	angleMinSR3->display(motorParam.minPos);	velocityNewSR3->setValue(motorParam.maxVelocity);}
			if(motorParam.name == elbowRight->text().toStdString()){	angleMaxER->display(motorParam.maxPos);		angleMinER->display(motorParam.minPos);		velocityNewER->setValue(motorParam.maxVelocity);}
			if(motorParam.name == foreArmRight->text().toStdString()){	angleMaxFAR->display(motorParam.maxPos);	angleMinFAR->display(motorParam.minPos);	velocityNewFAR->setValue(motorParam.maxVelocity);}
			if(motorParam.name == wristRight1->text().toStdString()){	angleMaxWR1->display(motorParam.maxPos);	angleMinWR1->display(motorParam.minPos);	velocityNewWR1->setValue(motorParam.maxVelocity);}
			if(motorParam.name == wristLeft2->text().toStdString()){	angleMaxWR2->display(motorParam.maxPos);	angleMinWR2->display(motorParam.minPos);	velocityNewWR2->setValue(motorParam.maxVelocity);}
			
			// CABEZA:
			if(motorParam.name == head1->text().toStdString()){	angleMaxH1->display(motorParam.maxPos);	angleMinH1->display(motorParam.minPos);	velocityNewH1->setValue(motorParam.maxVelocity);}
			if(motorParam.name == head2->text().toStdString()){	angleMaxH2->display(motorParam.maxPos);	angleMinH2->display(motorParam.minPos);	velocityNewH2->setValue(motorParam.maxVelocity);}
			if(motorParam.name == head3->text().toStdString()){	angleMaxH3->display(motorParam.maxPos);	angleMinH3->display(motorParam.minPos);	velocityNewH3->setValue(motorParam.maxVelocity);}
			
			// BASE:
			// TODO
		}
		
	} catch(const Ice::Exception &ex) {cout<<"--> Excepción en SETPARAMS al tomar datos del robot: "<<ex<<endl;}
	
	imv = new InnerModelViewer (innerModel, "root", osgView->getRootGroup());
	timer.start(Period);
	return true;
};



/*------------------------------------------------------------------------------------------------------*
 * 												SLOTS													*
 *------------------------------------------------------------------------------------------------------*/ 
void SpecificWorker::compute( )
{
	static QTime reloj = QTime::currentTime();
	// Si estamos mirando la pestaña número 5 mostramos los límites de los motores.
	if(pestanias->currentIndex()==5)
		mostrarDatos();
	
	if((trayectoria.isEmpty() == false)  and banderaRCIS)
	{
		enviarPose6D( trayectoria.dequeue() );
	}
	else
		banderaRCIS = false;
	
	actualizarInnerModel();
	imv->update();
	osgView->frame();
	
// 	try
// 	{ 
// 		TargetState state = bodyinversekinematics_proxy->getState("RIGHTARM"); 
// 		if(state.finish == false)
// 			qDebug() << "Target elapsed time" << state.elapsedTime;
// 	}
// 	catch(const Ice::Exception &ex)
// 	{ std::cout << "Warning! BIK not responding" << std::endl; };
// 	
// 	
	if ( reloj.elapsed() > 3000)
	{
		try{ bodyinversekinematics_proxy->ice_ping(); }
		catch(const Ice::Exception &ex)
		{ std::cout << "Warning! BIK not responding" << std::endl; };
		reloj.restart();
	}
	
}

/*------------------------------------------------------------------------------------------------------*
 * 									SLOTS DE LOS BOTONES DE ENVÍO										*
 *------------------------------------------------------------------------------------------------------*/
/**
 * @brief SLOT ENVIAR RCIS. Envía una pose suelta o una trayectoria completa, dependiendo de si
 * está levantada la banderaTrayectoria, al componente inverseKinematicsComp. El tipo del target
 * dependerá de la pestaña que esté abierta en ese momento.
 * 
 * @return void
 */ 
void SpecificWorker::enviarRCIS()
{
	try
	{
		bodyinversekinematics_proxy->setRobot(0);
		enviar();
	}
	catch(const Ice::Exception &ex){std::cout << ex << std::endl;};
}

/**
 * @brief SLOT ENVIAR ROBOT. TODO Por hacer
 * @return void
 */ 
void SpecificWorker::enviarROBOT()
{
	try
	{
		bodyinversekinematics_proxy->setRobot(1);
		enviar();
	}
	catch(const Ice::Exception &ex){std::cout << ex << std::endl;};
}


void SpecificWorker::enviar()
{
	// Si estamos en la pestaña 0 (la de Pose6D), entonces podemos enviar una trayectoria o una
	// pose suelta de tipo Pose6D. Para enviar la pose suelta leemos de la interfaz del usuario
	banderaRCIS = true;
	if(pestanias->tabText(pestanias->currentIndex()) == "Pose6D")
	{
		QVec p = QVec(6);
		p[0] = poseTX->value();		p[1] = poseTY->value();		p[2] = poseTZ->value(); //TRASLACIONES
		p[3] = poseRX->value();		p[4] = poseRY->value();		p[5] = poseRZ->value(); //ROTACIONES
		enviarPose6D(p);
	}

	// Estamos en la segunda pestaña: Axis Align. Enviamos target
	// del tipo AXISALIGN.
	if(pestanias->tabText(pestanias->currentIndex()) == "Axis Align" )
		enviarAxisAlign();
	if(pestanias->tabText(pestanias->currentIndex()) == "Move Along Axis" )
		moveAlongAxis();
	if(pestanias->tabText(pestanias->currentIndex()) == "Fingers" )
		closeFingers();
	if(pestanias->tabText(pestanias->currentIndex()) == "Home" )
		goHome(part_home->currentText());
}


void SpecificWorker::enviarHome()
{
	goHome("All");
}


/*
 * Método actualizarInnermodel
 * Actualiza el InnerModel con las nuevas posiciones de los motores del robot.  
 * FUNCIONA.
 */ 
void SpecificWorker::actualizarInnerModel()
{
	try 
	{
			RoboCompJointMotor::MotorStateMap mMap = jointmotor_proxy->getMotorStateMap( this->motorList);
		
			for(uint j=0; j<motorList.size(); j++)
				innerModel->updateJointValue(QString::fromStdString(motorList[j]), mMap.at(motorList[j]).pos);

	} catch (const Ice::Exception &ex) {
		cout<<"--> Excepción en actualizar InnerModel: "<<ex<<endl;
	}
}

void SpecificWorker::stop()
{
	try 
	{
		std::string part;
		if(partesActivadas[0] == 1)	
			bodyinversekinematics_proxy->stop( partBox1_pose6D->currentText().toStdString() );
			
		if(partesActivadas[1] == 1)
			bodyinversekinematics_proxy->stop( partBox2_pose6D->currentText().toStdString() );
		
		if(partesActivadas[2] == 1) 
			bodyinversekinematics_proxy->stop( partBox3_pose6D->currentText().toStdString() );		
	} 
	catch (const Ice::Exception &ex) 
	{
		cout<<"--> Excepción en actualizar InnerModel: "<<ex<<endl;
	}
}


/*------------------------------------------------------------------------------------------------------*
 * 									SLOTS PARA LA PESTAÑA POSE6D										*
 *------------------------------------------------------------------------------------------------------*/
/**
 * @brief SLOT ACTIVAR DESACTIVAR. Activa o desactiva en todas las pestañas de la interfaz las cajas 
 * donde estan las distintas partes del cuerpo. Sirve para poder enviar un mismo target a dos o más 
 * partes al mismo tiempo. Para ello, pone un 1 en el vector de partesActivadas en aquellas cajas que
 * esten seleccionadas o un 0 si no están activadas. Yo se que me explico muy mal, pero el caso es que
 * funciona.
 * TODO Se puede añadir el control para eliminar las partes ya seleccionadas en las cajas anteriores.
 * 
 * @return void
 */ 
void SpecificWorker::activarDesactivar()
{
	// Si están activadas alguna de las tres primeras cajas de partes del robot ACTIVAMOS TODAS.
	// Si no lo está, DESACTIVAMOS TODAS. Así tenemos la misma configuración cuando cambiemos de página	
	if((Part1_pose6D->isChecked() and pestanias->currentIndex()==0) or 
	   (Part1_AxisAlign->isChecked() and pestanias->currentIndex()==1) or 
	   (Part1_AlongAxis->isChecked() and pestanias->currentIndex()==2))
	{
		//Activamos las cajas y marcamos los checkBox
		partBox1_pose6D->setEnabled(true);		partBox1_pose6D->repaint();		Part1_pose6D->setCheckState(Qt::Checked);
		partBox1_AxisAlign->setEnabled(true);	partBox1_AxisAlign->repaint();	Part1_AxisAlign->setCheckState(Qt::Checked);
		partBox1_AlongAxis->setEnabled(true);	partBox1_AlongAxis->repaint();	Part1_AlongAxis->setCheckState(Qt::Checked);
		partesActivadas[0] = 1;
	}
	else
	{
		// Desactivamos las cajas y desmarcamos los chekbox
		partBox1_pose6D->setEnabled(false);		partBox1_pose6D->repaint();		Part1_pose6D->setCheckState(Qt::Unchecked);
		partBox1_AxisAlign->setEnabled(false);	partBox1_AxisAlign->repaint();	Part1_AxisAlign->setCheckState(Qt::Unchecked);
		partBox1_AlongAxis->setEnabled(false);	partBox1_AlongAxis->repaint(); 	Part1_AlongAxis->setCheckState(Qt::Unchecked);
		partesActivadas[0] = 0;
	}
	
	//----------------------------------------------------------------------------//
	if((Part2_pose6D->isChecked() and pestanias->currentIndex()==0) or 
	   (Part2_AxisAlign->isChecked() and pestanias->currentIndex()==1) or 
	   (Part2_AlongAxis->isChecked() and pestanias->currentIndex()==2))
	{
		partBox2_pose6D->setEnabled(true);		partBox2_pose6D->repaint();		Part2_pose6D->setCheckState(Qt::Checked);
		partBox2_AxisAlign->setEnabled(true);	partBox2_AxisAlign->repaint();	Part2_AxisAlign->setCheckState(Qt::Checked);
		partBox2_AlongAxis->setEnabled(true);	partBox2_AlongAxis->repaint();	Part2_AlongAxis->setCheckState(Qt::Checked);
		partesActivadas[1] = 1;
	}
	
	else
	{
		partBox2_pose6D->setEnabled(false);		partBox2_pose6D->repaint();		Part2_pose6D->setCheckState(Qt::Unchecked);
		partBox2_AxisAlign->setEnabled(false);	partBox2_AxisAlign->repaint();	Part2_AxisAlign->setCheckState(Qt::Unchecked);
		partBox2_AlongAxis->setEnabled(false);	partBox2_AlongAxis->repaint();	Part2_AlongAxis->setCheckState(Qt::Unchecked);
		partesActivadas[1] = 0;
	}
	
	//----------------------------------------------------------------------------//
	if((Part3_pose6D->isChecked() and pestanias->currentIndex()==0) or 
	   (Part3_AxisAlign->isChecked() and pestanias->currentIndex()==1) or 
	   (Part3_AlongAxis->isChecked() and pestanias->currentIndex()==2))
	{
		partBox3_pose6D->setEnabled(true);		partBox3_pose6D->repaint();		Part3_pose6D->setCheckState(Qt::Checked);	
		partBox3_AxisAlign->setEnabled(true);	partBox3_AxisAlign->repaint();	Part3_AxisAlign->setCheckState(Qt::Checked);	
		partBox3_AlongAxis->setEnabled(true);	partBox3_AlongAxis->repaint();	Part3_AlongAxis->setCheckState(Qt::Checked);
		partesActivadas[2] = 1;
	}
	else
	{
		partBox3_pose6D->setEnabled(false);		partBox3_pose6D->repaint();		Part3_pose6D->setCheckState(Qt::Unchecked);
		partBox3_AxisAlign->setEnabled(false);	partBox3_AxisAlign->repaint();	Part3_AxisAlign->setCheckState(Qt::Unchecked);
		partBox3_AlongAxis->setEnabled(false);	partBox3_AlongAxis->repaint();	Part3_AlongAxis->setCheckState(Qt::Unchecked);
		partesActivadas[2] = 0;
	}
}


/**
 * @brief Metodo CAMARERO ZURDO. Crea una trayectoria de poses para un camarero con una bandeja en
 * la mano izquierda. Guarda esa trayectoria en un atributo de la clase, trayectoria, para luego 
 * enviarla a que la ejecute el innerModel o el robot real. Debemos limpiar la trayectoria siempre
 * para que no se acumulen las trayectorias y levanta la banderaTrayectoria, indicando que existe 
 * una trayectoria lista para enviar al RCIS o al ROBOT.
 * -------------------------> CREA LA TRAYECTORIA EN MILIMETROS <--------------------------------
 * Desactiva el resto de banderas para el resto de targets.
 * 
 * @return void
 */ 
void SpecificWorker::camareroZurdo()
{
	trayectoria.clear(); //Limpiamos trayectoria para que NO SE ACUMULEN.
	
	//DEFINIMOS VARIABLES:
	//		- POSE: vector de 6 elementos donde se guardan las TRASLACIONES y ROTACIONES. Aunque las 
	//				rotaciones se dejan a CERO.
	//		- SALTO: salto en X e Y para pasar de una pose a otra. Fijado a 10mm.
	//		- TRAYECTORIA: atributo de la clase donde se almacenan las POSES.
	//		- xAux e yAux: auxiliares para crear el marco donde se mueve la mano del camarero.
    QVec pose = QVec::zeros(6);
	float salto = 10;
    float xAux, yAux;
	
	 // Trasladamos hacia la derecha en X:
	for(float i=-150; i<=150; i=i+salto)
	{
		pose[0] = i; pose[1] = 900; pose[2] = 350;
		trayectoria.enqueue(pose);
		xAux = i;
	}
	// Subimos en Y:
	for(float j=900; j<1100; j=j+salto)
	{
		pose[0] = xAux; pose[1] = j; pose[2] = 350;
		trayectoria.enqueue(pose);
		yAux = j;
	}
	// Trasladamos hacia la izquierda en X:
	for(float i=xAux; i>=-150; i=i-salto)
	{
		pose[0] = i; pose[1] = yAux; pose[2] = 350;
		trayectoria.enqueue(pose);
		xAux=i;
	}
	// Bajamos en Y:
	for(float j=yAux; j>=900; j=j-salto)
	{
		pose[0] = xAux; pose[1] = j; pose[2] = 350;
		trayectoria.enqueue(pose);
		yAux = j;
	}
	
	banderaTrayectoria = true; //Indicamos que hay una trayectoria lista para enviar.
}

/**
 * @brief Metodo CAMARERO DIESTRO. Crea una trayectoria de poses para un camarero con una bandeja 
 * en la mano derecha. Guarda esa trayectoria en un atributo de la clase, trayectoria, para luego 
 * enviarla a que la ejecute el innerModel o el robot real. Debemos limpiar la trayectoria siempre
 * para que no se acumulen las trayectorias y levanta la banderaTrayectoria, indicando que existe 
 * una trayectoria lista para enviar al RCIS o al ROBOT.
 * -------------------------> CREA LA TRAYECTORIA EN MILIMETROS <--------------------------------
 * Desactiva el resto de banderas para el resto de targets
 * 
 * @return void
 */ 
void SpecificWorker::camareroDiestro()
{
	trayectoria.clear(); //Limpiamos trayectoria para que NO SE ACUMULEN.
	
	//DEFINIMOS VARIABLES:
	//		- POSE: vector de 6 elementos donde se guardan las TRASLACIONES y ROTACIONES. Aunque las 
	//				rotaciones se dejan a CERO.
	//		- SALTO: salto en X e Y para pasar de una pose a otra. Fijado a 10mm.
	//		- TRAYECTORIA: atributo de la clase donde se almacenan las POSES.
	//		- xAux e yAux: auxiliares para crear el marco donde se mueve la mano del camarero.
    QVec pose = QVec::zeros(6);
	float salto = 10;
    float xAux, yAux;
	
	//guardamos la posicion de partida
 	QVec home(6);
 	home.inject(innerModel->transform("world", QVec::zeros(3),"grabPositionHandR"),0);
 	home.inject(innerModel->getRotationMatrixTo("world","grabPositionHandR").extractAnglesR_min(),3);
 	
	// Trasladamos en X hacia la izquierda: 
	for(float i=-100; i<300; i=i+salto)
	{
		pose[0] = i; pose[1] = 900; pose[2] = 350;
		pose[3] = poseRX->value(); pose[4] = poseRY->value(); pose[5] = poseRZ->value();
		trayectoria.append(pose);
		xAux = i;
	}
	// Subimos en Y:
	for(float j=900; j<1100; j=j+salto)
	{
		pose[0] = xAux; pose[1] = j; pose[2] = 350;
		pose[3] = poseRX->value(); pose[4] = poseRY->value(); pose[5] = poseRZ->value();
		trayectoria.append(pose);
		yAux = j;
	}
	// Trasladamos en X hacia la derecha:
	for(float i=xAux; i>=-100; i=i-salto)
	{
		pose[0] = i; pose[1] = yAux; pose[2] = 350;
		pose[3] = poseRX->value(); pose[4] = poseRY->value(); pose[5] = poseRZ->value();
		trayectoria.append(pose);
		xAux = i;
	}
	// Bajamos en Y:
	for(float j=yAux; j>=900; j=j-salto)
	{
		pose[0] = xAux; pose[1] = j; pose[2] = 350;
		pose[3] = poseRX->value(); pose[4] = poseRY->value(); pose[5] = poseRZ->value();
		trayectoria.append(pose);
		yAux = j;
	}
	
	//go back to initial grabPositionHandR
	trayectoria.append(home);
	
	banderaTrayectoria = true; //Indicamos que hay una trayectoria lista para enviar.
}

/**
 * @brief Metodo CAMARERO CENTRO. Crea una trayectoria de poses para un camarero con una bandeja 
 * en moviendola por el centro del pecho. Guarda esa trayectoria en un atributo de la clase, 
 * trayectoria, para luego enviarla a que la ejecute el innerModel o el robot real. Debemos limpiar
 * la trayectoria siempre para que no se acumulen las trayectorias y levanta la banderaTrayectoria, 
 * indicando que existe una trayectoria lista para enviar al RCIS o al ROBOT.
 * -------------------------> CREA LA TRAYECTORIA EN MILIMETROS <--------------------------------
 * Desactiva el resto de banderas para el resto de targets.
 * 
 * @return void
 */ 
void SpecificWorker::camareroCentro()
{
	trayectoria.clear(); //Limpiamos trayectoria para que NO SE ACUMULEN.
	
	//DEFINIMOS VARIABLES:
	//		- POSE: vector de 6 elementos donde se guardan las TRASLACIONES y ROTACIONES. Aunque las 
	//				rotaciones se dejan a CERO.
	//		- SALTO: salto en X e Y para pasar de una pose a otra. Fijado a 10mm.
	//		- TRAYECTORIA: atributo de la clase donde se almacenan las POSES.
	//		- xAux e yAux: auxiliares para crear el marco donde se mueve la mano del camarero.
  QVec pose = QVec::zeros(6);
	
	pose[3] = poseRX->value(); pose[4] = poseRY->value(); pose[5] = poseRZ->value();
		
	pose[0] = -150; pose[1] = 900; pose[2] = 300;
	trayectoria.append(pose);
	
	pose[0] = 150; pose[1] = 900; pose[2] = 300;
	trayectoria.append(pose);

	pose[0] = 150; pose[1] = 1100; pose[2] = 300;
	trayectoria.append(pose);

	pose[0] = -150; pose[1] = 1100; pose[2] = 300;
	trayectoria.append(pose);
	
	pose[0] = -150; pose[1] = 900; pose[2] = 300;
	trayectoria.append(pose);
	
	banderaTrayectoria = true; //Indicamos que hay una trayectoria lista para enviar.
}




void SpecificWorker::camareroCentro2()
{
	trayectoria.clear(); //Limpiamos trayectoria para que NO SE ACUMULEN.
	
	//DEFINIMOS VARIABLES:
	//		- POSE: vector de 6 elementos donde se guardan las TRASLACIONES y ROTACIONES. Aunque las 
	//				rotaciones se dejan a CERO.
	//		- SALTO: salto en X e Y para pasar de una pose a otra. Fijado a 10mm.
	//		- TRAYECTORIA: atributo de la clase donde se almacenan las POSES.
	//		- xAux e yAux: auxiliares para crear el marco donde se mueve la mano del camarero.
  QVec pose = QVec::zeros(6);
	
	pose[3] = poseRX->value(); pose[4] = poseRY->value(); pose[5] = poseRZ->value();
		
	pose[0] = -150; pose[1] = 900; pose[2] = 300;
	trayectoria.append(pose);
	
	pose[0] = 150; pose[1] = 900; pose[2] = 300;
	trayectoria.append(pose);

	pose[0] = 150; pose[1] = 1100; pose[2] = 300;
	trayectoria.append(pose);

	pose[0] = -150; pose[1] = 1100; pose[2] = 300;
	trayectoria.append(pose);
	
	pose[0] = -150; pose[1] = 900; pose[2] = 300;
	trayectoria.append(pose);
	
	banderaTrayectoria = true; //Indicamos que hay una trayectoria lista para enviar.
}

/*--------------------------------------------------------------------------------------------------*
 * 							SLOTS PARA LA PESTAÑA DE HOME											*
 *--------------------------------------------------------------------------------------------------*/
/**
 * @brief Metodo GO HOME. Envia a la posicion de HOME la parte del cuerpo del robot que este 
 * seleccionada en la pestaña "Home". Existe la opción "All" dentro de esta pestaña, en la que
 * hay que enviar TODAS las partes del cuerpo al HOME.
 * 
 * @return void
 */
void SpecificWorker::goHome(QString partName)
{
	std::string part = partName.toStdString();
	qDebug() << "Go gome" << partName;
	try 
	{	
		
		if(partName=="All")
		{
				bodyinversekinematics_proxy->goHome("HEAD");
				bodyinversekinematics_proxy->goHome("LEFTARM");
				bodyinversekinematics_proxy->goHome("RIGHTARM");
		}
		else
			bodyinversekinematics_proxy->goHome(part);
	} 
	catch (Ice::Exception ex) 
	{
		cout << ex << endl;
	}
}

/*--------------------------------------------------------------------------------------------------*
 * 							SLOTS PARA LA PESTAÑA SET FINGERS								*
 *--------------------------------------------------------------------------------------------------*/
/**
 * @brief ..
 * @return void
 */ 
void SpecificWorker::closeFingers()
{
	try 
	{	
		qDebug() << __FUNCTION__ << "Set fingers";
		bodyinversekinematics_proxy->setFingers((T)fingersDistanceSB->value());
	} 
	catch (Ice::Exception ex) 
	{
		cout << ex << endl;
	}
	
}

/*--------------------------------------------------------------------------------------------------*
 * 							SLOTS PARA LA PESTAÑA ADVANCE ALONG AXIS								*
 *--------------------------------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------------------------*
 * 								MÉTODOS PRIVADOS DE LA CLASE									*
 *----------------------------------------------------------------------------------------------*/
/**
 * @brief Metodo MOVER TARGET. Mueve el target dentro del innerModel a una posicion que se le pasa 
 * como parametro de entrada. Crea una pose3D a cero y actualiza sus traslaciones tx, ty y tz y sus 
 * rotaciones rx, ry y rz con los datos del parámetro de entrada. 
 * Sirve para colocar el target en el innerModel. Para nada más.
 * @param QVec pose es la posicion en la que se debe pintar el target en el innerModel. 
 * 
 * @return void
 */ 
void SpecificWorker::moverTargetEnRCIS(const QVec &pose)
{
	try
	{
		RoboCompInnerModelManager::Pose3D p;
		p.x=p.y=p.z=p.rx=p.ry=p.rz=0.0; //Primero inicializamos a cero.

		p.x = pose[0]; p.y = pose[1]; p.z = pose[2];
		p.rx = pose[3]; p.ry = pose[4]; p.rz = pose[5];
			
		innermodelmanager_proxy->setPoseFromParent("target",p);
		innerModel->updateTransformValues("target",p.x,p.y,p.z,p.rx,p.ry,p.rz);        ////CREO QUE SE PUEDE QUITAR
		
	}catch (const Ice::Exception &ex){ cout<<"Excepción en moverTarget: "<<ex<<endl; }
}

/**
 * @brief Metodo privado ENVIAR POSE6D. Es llamado por el SLOT enviarRCIS cuando la banderaTrayectoria
 * no esta levantada y se está trabajando en la primera pestaña (Pose6D). Tiene capacidad para enviar 
 * un mismo target a las tres partes del cuerpo con las que se estan trabajando ahora: el brazo derecho,
 * el izquierdo y la cabeza, con las opciones de la interfaz.
 * @param QVec p es el vector de traslaciones y posiciones que tiene que enviar al inverseKinematicsComp
 * 
 * @return void
 */ 
void SpecificWorker::enviarPose6D(QVec p)
{
	// DEFINIMOS VARIABLES:
	//		- poseMetros: auxiliar para pasar las traslaciones de las poses de milímetros a metros
	//					  (para pintar el cubo o target en el RCIS de METROS)
	//		- pesos: vector de pesos de las traslaciones y las rotaciones.
	//		- part1, par2 y par3: partes del cuerpo del robot al que va dirigido el/los target/s
	//		- partes: cola donde colocar las partes del cuerpo e ir consultándolas.
	//		- type: tipo de target que se le está enviando.
	//		- pose6D: tipo de variable para enviar traslaciones y rotaciones al inverseKinematicsComp
	//		- weights: tipo de variable para enviar los pesos al inverseKinematicsComp
	QVec poseMetros = QVec::zeros(6);
	QVec pesos(6);
	std::string part1, part2, part3;
	QQueue <std::string> partes;
	QString type;	
	RoboCompBodyInverseKinematics::Pose6D pose6D;
	RoboCompBodyInverseKinematics::WeightVector weights;
		
	try
	{
		//colocamos el target en el RCIS. HAY QUE PASAR A METROS LAS TRASLACIONES
		//poseMetros[0] = p[0]/1000; 	poseMetros[1]=p[1]/1000; 	poseMetros[2]=p[2]/1000;
		//poseMetros[0] = p[0]; 	poseMetros[1]=p[1]; 	poseMetros[2]=p[2];   //POSE A RCIS EN mm
		moverTargetEnRCIS(p); 
			
		//Creamos la pose6D para pasárselo al componente lokiArm (pasamos MILÍMETROS)
		pose6D.x = p[0];     pose6D.y = p[1];     pose6D.z = p[2];
		//antes de pasar las rotaciones las "normalizamos"
		QVec aux (3); 
		aux[0] = p[3]; 	     aux[1] = p[4];	  aux[2] = p[5];
		//calcularModuloFloat(aux, 2*M_PI);
		pose6D.rx = aux[0];    pose6D.ry = aux[1];    pose6D.rz = aux[2];
		
		//Ponemos los pesos mirando la interfaz.
		pesos.set((T)0);
		if(wTX->isChecked()) pesos[0] = 1;	if(wTY->isChecked()) pesos[1] = 1;	if(wTZ->isChecked()) pesos[2] = 1; //TRASLACIONES
		if(wRX->isChecked()) pesos[3] = 1;	if(wRY->isChecked()) pesos[4] = 1;	if(wRZ->isChecked()) pesos[5] = 1; //ROTACIONES
			
		//Creamos la variable del tipo weightvector para pasarselo al componente lokiArm
		weights.x = pesos[0];		weights.y = pesos[1];		weights.z = pesos[2];    
		weights.rx = pesos[3];		weights.ry = pesos[4];		weights.rz = pesos[5];
   	
		//Sacamos la/s parte/s del cuerpo a la que va destinado el target y las encolamos para
		//luego ir sacándolas en un bucle. También obtenemos el tipo de target
		part1 = part2 = part3 = "NO_NAME";
		if(partesActivadas[0] == 1)	part1 = partBox1_pose6D->currentText().toStdString();
		if(partesActivadas[1] == 1) part2 = partBox2_pose6D->currentText().toStdString();
		if(partesActivadas[2] == 1) part3 = partBox3_pose6D->currentText().toStdString();
		partes.enqueue(part1); partes.enqueue(part2); partes.enqueue(part3);
		type = "POSE6D"; //Fijamos el tipo
		
		for(int i=0; i<3; i++)
		{
			std::string part = partes.dequeue();
			if (part != "NO_NAME")
			{
				if(part == "HEAD")
				{
					RoboCompBodyInverseKinematics::Axis axis;
					axis.x = 0; axis.y = -1; axis.z = 0;
					bodyinversekinematics_proxy->pointAxisTowardsTarget(part, pose6D, axis, false, 0);	
				}				
				else
				{ 	
					qDebug()<<"---> TARGET ENVIADO con traslaciones ("<<pose6D.x<<pose6D.y<<pose6D.z<<") y rotaciones ("<<pose6D.rx<<pose6D.ry<<pose6D.rz<<")";
					bodyinversekinematics_proxy->setTargetPose6D(part, pose6D, weights, 250);
				}
				usleep(50000);
			}
		}
	}catch(Ice::Exception ex){ std::cout<<"Error al pasar TARGET POSE6D: "<<ex<<endl;}
}


/**
 * @brief Metodo ENVIAR AXIS ALIGN. Saca de la segunda pestaña de la interfaz del usuario los datos para 
 * componer un target del tipo AXISALIGN y enviarselo a una parte o a varias partes del cuerpo del robot.
 * 
 * @return void
 */
void SpecificWorker::enviarAxisAlign()
{
	QQueue <QString> partes;
	
	if(partesActivadas[0] == 1)	
		partes.enqueue(partBox1_AxisAlign->currentText());
	if(partesActivadas[1] == 1) 
		partes.enqueue(partBox2_AxisAlign->currentText());
	if(partesActivadas[2] == 1) 
		partes.enqueue(partBox3_AxisAlign->currentText());
	
	foreach(QString parte, partes)
	{
		try
		{
			RoboCompBodyInverseKinematics::Axis axis;
			axis.x = TipAxis_X->value();
			axis.y = TipAxis_Y->value();
			axis.z = TipAxis_Z->value();		
			RoboCompBodyInverseKinematics::Pose6D pose6D;
			pose6D.x = axisAlignXSB->value();
			pose6D.y = axisAlignYSB->value();
			pose6D.z = axisAlignZSB->value();
			pose6D.rx = axisAlignRXSB->value();
			pose6D.ry = axisAlignRYSB->value();
			pose6D.rz = axisAlignRZSB->value();	
			
			QVec pose = QVec::zeros(6);
			pose[0] = pose6D.x/1000; pose[1] = pose6D.y/1000; pose[2] = pose6D.z/1000;
			moverTargetEnRCIS(pose);
	
			bodyinversekinematics_proxy->pointAxisTowardsTarget(parte.toStdString(), pose6D, axis, false, 0 );
 		}
 		catch(Ice::Exception ex){ std::cout<< __FUNCTION__ << __LINE__ << "Error al pasar el target tipo ALIGNAXIS: "<<ex<<endl;}
	}
}

/**
 * @brief Metodo MOVE ALONG AXIS. Saca de la tercera pestaña de la interfaz del usuario los datos para
 * componer un target del tipo ALONGAXIS y se lo envia a una o a varias partes del cuerpo del robot.
 * 
 * @return void
 */
void SpecificWorker::moveAlongAxis()
{
	QQueue <QString> partes;
	
	
	qDebug() << __FUNCTION__ << "Processing command";
	if(partesActivadas[0] == 1)	
		partes.enqueue(partBox1_AlongAxis->currentText());
	if(partesActivadas[1] == 1) 
		partes.enqueue(partBox2_AlongAxis->currentText());
	if(partesActivadas[2] == 1) 
		partes.enqueue(partBox3_AlongAxis->currentText());
	
	foreach(QString parte, partes)
	{
		try
		{
			RoboCompBodyInverseKinematics::Axis axis;
			axis.x = TipAxis_X2->value();
			axis.y = TipAxis_Y2->value();
			axis.z = TipAxis_Z2->value();
			float dist = distanceSpinBox->value();
			
			bodyinversekinematics_proxy->advanceAlongAxis(parte.toStdString(), axis, dist);
		} 
		catch(Ice::Exception ex){ std::cout<<"Error al pasar el target tipo ADVANCEALONGAXIS: "<<ex<<endl;}
	}
}


/**
 * @brief Metodo MOSTRAR DATOS. Muestra los datos de los motores por la interfaz: el nombre del motor, los
 * limites que tiene, la posicion angular en radianes actual y la velocidad en radianes por segundo.
 * 
 * @return void
 */ 
void SpecificWorker::mostrarDatos()
{
	try
	{
		// Recuerda: motorparamList ya lo tenemos relleno de datos desde el setParams.
		foreach(RoboCompJointMotor::MotorParams motorParam, motorparamList)
		{
			// Sacamos parámetros dinámicos del motor.
			RoboCompJointMotor::MotorState motorState = jointmotor_proxy->getMotorState(motorParam.name);
			
			// BRAZO IZQUIERDO:
			if(motorParam.name == shoulder1Left->text().toStdString()){	angleCurrentSL1->display(motorState.pos);	velocityNewSL1->setValue(motorState.vel); }
			if(motorParam.name == shoulder2Left->text().toStdString()){	angleCurrentSL2->display(motorState.pos);	velocityNewSL2->setValue(motorState.vel); }
			if(motorParam.name == shoulder3Left->text().toStdString()){	angleCurrentSL3->display(motorState.pos);	velocityNewSL3->setValue(motorState.vel); }
			if(motorParam.name == elbowLeft->text().toStdString()){		angleCurrentEL->display(motorState.pos);	velocityNewEL->setValue(motorState.vel); }
			if(motorParam.name == foreArmLeft->text().toStdString()){	angleCurrentFAL->display(motorState.pos);	velocityNewFAL->setValue(motorState.vel); }
			if(motorParam.name == wristLeft1->text().toStdString()){	angleCurrentWL1->display(motorState.pos);	velocityNewWL1->setValue(motorState.vel); }
			if(motorParam.name == wristLeft2->text().toStdString()){	angleCurrentWL2->display(motorState.pos);	velocityNewWL2->setValue(motorState.vel); }

			// BRAZO DERECHO
			if(motorParam.name == shoulder1Right->text().toStdString()){angleCurrentSR1->display(motorState.pos);	velocityNewSR1->setValue(motorState.vel); }
			if(motorParam.name == shoulder2Right->text().toStdString()){angleCurrentSR2->display(motorState.pos);	velocityNewSR2->setValue(motorState.vel); }
			if(motorParam.name == shoulder3Right->text().toStdString()){angleCurrentSR3->display(motorState.pos);	velocityNewSR3->setValue(motorState.vel); }
			if(motorParam.name == elbowRight->text().toStdString()){	angleCurrentER->display(motorState.pos);	velocityNewER->setValue(motorState.vel); }
			if(motorParam.name == foreArmRight->text().toStdString()){	angleCurrentFAR->display(motorState.pos);	velocityNewFAR->setValue(motorState.vel); }
			if(motorParam.name == wristRight1->text().toStdString()){	angleCurrentWR1->display(motorState.pos);	velocityNewWR1->setValue(motorState.vel); }
			if(motorParam.name == wristRight2->text().toStdString()){	angleCurrentWR2->display(motorState.pos);	velocityNewWR2->setValue(motorState.vel); }

			// CABEZA:
			if(motorParam.name == head1->text().toStdString()){	angleCurrentH1->display(motorState.pos);	velocityNewH1->setValue(motorState.vel); }
			if(motorParam.name == head2->text().toStdString()){	angleCurrentH2->display(motorState.pos);	velocityNewH2->setValue(motorState.vel); }
			if(motorParam.name == head3->text().toStdString()){	angleCurrentH3->display(motorState.pos);	velocityNewH3->setValue(motorState.vel); }
		}				
	} catch(const Ice::Exception &ex) {cout<<"--> Excepción en MOSTRAR DATOS: "<<ex<<endl;}
}


/*
* AÑADIDO DESDE CINEMÁTICA INVERSA.
*  Metodo moduloFloat
* Devuelve el m��dulo entre dos n��meros reales.   ///HAS PROBADO FMOD? NO, NO TENGO TANTO CONOCIMIENTO DE C++
* FUNCIONA.
*/
void SpecificWorker::calcularModuloFloat(QVec &angles, float mod)
{
        for(int i=0; i<angles.size(); i++)
        {
                int cociente = (int)(angles[i] / mod);
                angles[i] = angles[i] -(cociente*mod);

                if(angles[i] > M_PI)
                        angles[i] = angles[i]- M_PI;
                else
                        if(angles[i] < -M_PI)
                                angles[i] = angles[i] + M_PI;
        }
}



/////CODIGO DE LOKIARM PARA INICIALIZAR DESDE EL GENERADOR

//CREA LA LISTA DE TARGETS PARA CADA PARTE DEL CUERPO: necesita innerModel y el bodypart
	/*listaTargetsBrazoDerecho = generador.generarListaTargets(innerModel, bodyParts.value("RIGHTARM"));
	bodyParts["RIGHTARM"].addListaTarget(listaTargetsBrazoDerecho);
	
	listaTargetsBrazoIzquierdo = generador.generarListaTargets(innerModel, bodyParts.value("LEFTARM")); //añadido
	bodyParts["LEFTARM"].addListaTarget(listaTargetsBrazoIzquierdo);*/
	
	
	//USAMOS LA INTERFAZ DE ICE
// 	QVec w(6);
// 	w.set(1.f);
	
// 	listaTargetsBrazoDerecho = generador.generarListaTargets(innerModel, bodyParts.value("RIGHTARM"), w);
// 	bodyParts["RIGHTARM"].addListaTarget(listaTargetsBrazoDerecho);
// 	
//  	listaTargetsBrazoIzquierdo = generador.generarListaTargets(innerModel, bodyParts.value("LEFTARM"), w); 
// 	foreach(Target t, listaTargetsBrazoIzquierdo)
// 	{
// 		RoboCompBodyInverseKinematics::Pose6D pose;
// 		RoboCompBodyInverseKinematics::WeightVector weights;
// 		pose.x = t.getPose().x();pose.y = t.getPose().y();pose.z = t.getPose().z();
// 		pose.rx = t.getPose().rx();pose.ry = t.getPose().ry();pose.rz = t.getPose().rz();
// 		weights.x = t.getWeights().x();	weights.y = t.getWeights().y();	weights.z = t.getWeights().z();
// 		weights.rx = t.getWeights().rx();	weights.ry = t.getWeights().ry(); weights.rz = t.getWeights().rz();
// 		this->setTargetPose6D("LEFTARM", pose, weights);
// 	}
// 	qDebug() << "Size lista  " << bodyParts.value("LEFTARM").getListaTargets().size();
	
  //bodyParts["LEFTARM"].addListaTarget(listaTargetsBrazoIzquierdo);
// 	listaTargetsCabeza = generador.generarListaTargets(innerModel, bodyParts.value("HEAD"),w); //añadido
// 	foreach(Target t, listaTargetsCabeza)
// 	{
// 		RoboCompBodyInverseKinematics::Pose6D pose;
// 		pose.x = t.getPose().x();pose.y = t.getPose().y();pose.z = t.getPose().z();
// 		pose.rx = t.getPose().rx();pose.ry = t.getPose().ry();pose.rz = t.getPose().rz();
// 		this->pointAxisTowardsTarget("HEAD", pose, "z", true, 0.f);		
// 	}
	
	//bodyParts["HEAD"].addListaTarget(listaTargetsCabeza);
