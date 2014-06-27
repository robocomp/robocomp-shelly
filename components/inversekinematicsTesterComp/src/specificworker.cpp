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
	//	- NO hay trayectoria asignada
	//	- NO hay target para RCIS
	banderaTrayectoria = banderaNiapa = banderaRCIS= false;

	// ÑAPA PARA PODER ENVIAR UN MISMO TARGET A DOS O TRES PARTES DEL CUERPO A LA VEZ
	// Por defecto siempre empezamos con una única parte del cuerpo.
	//	- 1: está activada.
	//	- 0: está desactivada.
	partesActivadas = QVec(3); //vector de partes
	partesActivadas[0] = 1;		partesActivadas[1] = 0;		partesActivadas[2] = 0; 

	///////////// HACEMOS LAS CONEXIONES /////////////////
	//Conectamos botones de EJECUCIÓN:
	connect(rcisButton, SIGNAL(clicked()), this, SLOT(enviarRCIS()));
	connect(robotButton, SIGNAL(clicked()), this, SLOT(enviarROBOT()));
	connect(homePushButton, SIGNAL(clicked()), this, SLOT(enviarHome()));
	connect(stopButton, SIGNAL(clicked()), this, SLOT(stop()));
	
	connect(aprilSendButton, SIGNAL(clicked()), this, SLOT(ballisticPartToAprilTarget()));
	connect(aprilFineButton, SIGNAL(clicked()), this, SLOT(finePartToAprilTarget()));
	//Esta señal la emite el QTabWidget cuando el usuario cambia el tab activo

	//Conectamos botones de la primera pestaña POSE6D:
	connect(camareroZurdoButton, SIGNAL(clicked()), this, SLOT(camareroZurdo()));
	connect(camareroDiestroButton, SIGNAL(clicked()), this, SLOT(camareroDiestro()));
	connect(camareroCentroButton, SIGNAL(clicked()), this, SLOT(camareroCentro()));
	connect(esfera, SIGNAL(clicked()), this, SLOT(puntosEsfera()));
	connect(cubo, SIGNAL(clicked()), this, SLOT(puntosCubo()));

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

	// BOTONERA AÑADIDA
	connect(test1Button, SIGNAL(clicked()), this, SLOT(abrirPinza()));
	connect(test2Button, SIGNAL(clicked()), this, SLOT(posicionInicial()));
	connect(test3Button, SIGNAL(clicked()), this, SLOT(posicionCoger()));
	connect(test4Button, SIGNAL(clicked()), this, SLOT(cerrarPinza()));
	connect(test5Button, SIGNAL(clicked()), this, SLOT(posicionSoltar()));
	connect(test6Button, SIGNAL(clicked()), this, SLOT(izquierdoRecoger()));
	connect(test7Button, SIGNAL(clicked()), this, SLOT(abrirPinza()));
	connect(test8Button, SIGNAL(clicked()), this, SLOT(retroceder()));
	connect(test9Button, SIGNAL(clicked()), this, SLOT(goHomeR()));	
	connect(test10Button, SIGNAL(clicked()), this, SLOT(izquierdoOfrecer()));
	connect(test11Button, SIGNAL(clicked()), this, SLOT(enviarHome()));

}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{

}

/**
 * @brief Method SET PARAMS. It's called for the MONITOR thread, which initialize the component with the parameters
 * of the correspondig config file.
 * 
 * @return bool
 */ 
bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	// Guardamos el innerModel que se nos pasa como parámetro de inicialización.
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
	catch(std::exception e)	{ qFatal("Error reading config params"); }

	try
	{
		// Sacamos todos los parámetros de los motores del robot. Esto nos servirá para inicializar la pestaña de direct Kinematics
		// de la interfaz de usuario.
		motorparamList = jointmotor_proxy->getAllMotorParams();

		foreach(RoboCompJointMotor::MotorParams motorParam, motorparamList)
		{
			// Ponemos los datos de los límites min y max a TODOS los motores. (por eso lo apiñurgo un poco, porque quedaría
			// un tochaco de código insufrible --y aún así lo es--)
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

	// Ponemos también una ventana del innerModel en esta pestaña:
	imv = new InnerModelViewer (innerModel, "root", osgView->getRootGroup());
	timer.start(Period);
	return true;
};



/*------------------------------------------------------------------------------------------------------*
 * 												SLOTS													*
 *------------------------------------------------------------------------------------------------------*/ 
/**
 * @brief Compute Method.
 * 
 * @return void
 */
void SpecificWorker::compute( )
{
	static QTime reloj = QTime::currentTime();

	// ÑAPA: Si estamos mirando la pestaña número 5 mostramos los límites de los motores
	// para que no se nos quede colgada la interfaz.
	if(pestanias->currentIndex()==5)
		mostrarDatos();

	// Desde el compute sólo se envían trayectorias. Para targets sueltos, se usan los demás
	// métodos.
	// ÑAPA: Si la trayectoriano está vacía y la bandera RCIS está levantada, enviamos la
	// trayectoria de targetes al RCIS. De esta forma evitamos que el tester se quede
	// colgado hasta que termine de enviar todos los targets de la trayectoria.
	if((trayectoria.isEmpty() == false)  and banderaRCIS)
	{
		enviarPose6D( trayectoria.dequeue() );
		sleep(1);
	}
	else
		banderaRCIS = false;

	// Actualizamos el innerModel y la ventada del viewer
	actualizarInnerModel();
	imv->update();
	osgView->frame();

	// Hacemos un ping al inverseKinematicsComp cada cierto tiempo. Si no responde
	// es que el inverseKinematicsComp no está levantado.
	if ( reloj.elapsed() > 3000)
	{
		try{ 
			bodyinversekinematics_proxy->ice_ping(); 

		}catch(const Ice::Exception &ex){ std::cout << "Warning! BIK not responding" << std::endl; };
		reloj.restart();
	}

}

/*------------------------------------------------------------------------------------------------------*
 * 									SLOTS DE LOS BOTONES DE ENVÍO										*
 *------------------------------------------------------------------------------------------------------*/
/**
 * @brief SLOT STOP. Para la ejecución del componente inverseKinematicsComp. Aborta el target o la
 * trayectoria de targets enviada al componente inverseKinematicsComp.
 * 
 * @return void
 */ 
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
	catch (const Ice::Exception &ex) {	cout<<"Excepción en STOP: "<<ex<<endl;	}
}

/**
 * @brief SLOT ENVIAR HOME. Se invoca al pulsar el botón HOME de los botones de control.
 * Es un envoltorio que llama al método goHome indicando que vayan a la posición de home
 * TODAS las partes del robot.
 */
void SpecificWorker::enviarHome()
{
	goHome("All");
}

/**
 * @brief SLOT ENVIAR RCIS. Marca como proxy objetivo el del RCIS y llama a la función enviar, que se 
 * encarga de ver en qué pestaña está el usuario de la GUI y enviar el target del tipo que sea, al proxy 
 * indicado.
 * 
 * @return void
 */ 
void SpecificWorker::enviarRCIS()
{
	try
	{
		bodyinversekinematics_proxy->setRobot(0);
		banderaRCIS = true;
		enviar();
	}
	catch(const Ice::Exception &ex){std::cout <<"Enviar RCIS:"<< ex << std::endl;};
}

/**
 * @brief SLOT ENVIAR ROBOT. Marca como proxy objetivo el del ROBOT REAL  y llama a la función enviar, 
 * que se encarga de ver en qué pestaña está el usuario de la GUI y enviar el target del tipo que sea, 
 * al proxy indicado.
 * 
 * @return void
 */ 
void SpecificWorker::enviarROBOT()
{
	try
	{
		bodyinversekinematics_proxy->setRobot(1);
		banderaRCIS = false;
		enviar();
	}
	catch(const Ice::Exception &ex){std::cout <<"Enviar ROBOT: "<< ex << std::endl;};
}

/*------------------------------------------------------------------------------------------------------*
 * 									SLOTS COMUNES A TODAS LAS PESTAÑAS									*
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


/*------------------------------------------------------------------------------------------------------*
 * 									SLOTS PARA LA PESTAÑA POSE6D										*
 *------------------------------------------------------------------------------------------------------*/
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

/**
 * @brief Método PUNTOS ESFERA. Crea una trayectoria, formando una esfera cerca del efector final
 * del robot.Está pensada para que la parte del robot a la que se le envíe llegue sin problemas, 
 * relativamente, se guarden los errores y el tiempo de ejecución de cada target resuleto y se pinte
 * una gráfica del error y del tiempo de ejecución en MATLAB.
 *  -------------------------> CREA LA TRAYECTORIA EN MILIMETROS <--------------------------------
 * Calcula la esfera de radio 100mm para el efector seleccionado en el primer recuadro!!!
 * 
 * @return void
 */
void SpecificWorker::puntosEsfera()
{
	trayectoria.clear(); //Limpiamos trayectoria para que NO SE ACUMULEN.

	//DEFINIMOS VARIABLES:
	//		- LISTAPUNTOS: lista auxiliar donde se guardan las traslaciones de los puntos calculados. 
	//		  Se utiliza para no repetir puntos con las mismas traslaciones.
	//		- POSE: vector de 6 elementos donde se guardan las TRASLACIONES y ROTACIONES. Le guardamos 
	//		  las rotaciones como la interfaz indica
	//		- PAUX: vector auxiliar para calcular traslaciones alrededor de la esfera.
	//		- TRAYECTORIA: atributo de la clase donde se almacenan las POSES.
	//		- TOTAL: entero auxiliar para calcular una pose de la esfera.
	//		- RADIO: float que indica el radio de la esfera.
	//		- PART: parte del robot seleccionada.
	//		- EFECTOR:	nombre del efector final de la parte del robot. Sirve como centro de la esfera
	QList<QVec> listaPuntos;
	QVec pose = QVec::zeros(6);	pose[3] = poseRX->value(); pose[4] = poseRY->value(); pose[5] = poseRZ->value();
	QVec paux = QVec::zeros(3);
	int total = 0;
	float radio = 100.0; //10cm

	// Sacamos la parte del robot de la primera caja y obtenemos su efector final para situar
	// el centro de la esfera
	std::string part = partBox1_pose6D->currentText().toStdString();
    QString efector;
	if(part=="LEFTARM") 	efector="grabPositionHandL";
	if(part=="RIGHTARM")	efector="grabPositionHandR";
	if(part=="HEAD")	 	efector="head3";

	// Mientras que no rellenemos la lista con todos los targets que queremos calcular:
	// 	- Por una parte calculamos las traslaciones, que serán alrededor de una esfera.
	// 	- Por otra parte dejamos las rotaciones como están.
	while(trayectoria.size()<100)
	{
		// Preparamos las traslaciones:
		while(total < 1)
		{
			QVec d1 = QVec::uniformVector(1, -1, 1);
			QVec d2 = QVec::uniformVector(1, -1, 1);

			if (QVec::vec2(d1[0],d2[0]).norm2() < 1.f)
			{
				paux[0]	=	2*d1[0] - sqrt(1.f-d1[0]*d1[0] - d2[0]*d2[0]);	
				paux[1]	=	2*d2[0] - sqrt(1.f-d1[0]*d1[0] - d2[0]*d2[0]);	
				paux[2]	=	1.f -2.f*(d1[0]*d1[0] + d2[0]*d2[0]);
				paux = (paux * (T)radio) + innerModel->transform("world", QVec::zeros(3),efector);
				total++;
			}
		}
		total = 0;

		if(!listaPuntos.contains(paux))
		{
			listaPuntos.append(paux);
			pose[0] = paux[0];	pose[1] = paux[1];	pose[2] = paux[2];
			trayectoria.enqueue(pose);
		}
	}
	banderaTrayectoria = true; //Indicamos que hay una trayectoria lista para enviar.
// 	banderaNiapa = true;
}

/**
 * @brief Método PUNTOS CUBO.Crea una trayectoria de puntos aleatorios pertenecientes al cubo
 * tridimensional formado por los vértices:
 * 			x    y    z
 * 		- (400, 800, 200)	(400,1200, 200)		(400, 800, 600) 	(400, 1200, 600)
 * 		- (-400, 800, 200)	(-400,1200, 200)	(-400, 800, 600) 	(-400, 1200, 600)
 * 
 * Está pensado para que la parte del robot a la que se le envíe llegue sin muchos problemas, 
 * relativamente.
 *  -------------------------> CREA LA TRAYECTORIA EN MILIMETROS <--------------------------------
 * 
 * @return void
 */ 
void SpecificWorker::puntosCubo()
{
	trayectoria.clear(); //Limpiamos trayectoria para que NO SE ACUMULEN.

	//DEFINIMOS VARIABLES:
	//		- LISTAPUNTOS: lista auxiliar donde se guardan las traslaciones de los puntos calculados. 
	//		  Se utiliza para no repetir puntos con las mismas traslaciones.
	//		- POSE: vector de 6 elementos donde se guardan las TRASLACIONES y ROTACIONES. Le guardamos 
	//		  las rotaciones como la interfaz indica
	//		- PAUX: vector auxiliar para calcular traslaciones alrededor de la esfera.
	//		- TRAYECTORIA: atributo de la clase donde se almacenan las POSES.
	//		- TOTAL: entero auxiliar para calcular una pose de la esfera.
	//		- RADIO: float que indica el radio de la esfera.
	//		- PART: parte del robot seleccionada.
	//		- EFECTOR:	nombre del efector final de la parte del robot. Sirve como centro de la esfera
	QList<QVec> listaPuntos;
	QVec pose = QVec::zeros(6);	pose[3] = poseRX->value(); pose[4] = poseRY->value(); pose[5] = poseRZ->value();
	QVec paux = QVec::zeros(3);
	
	srand((unsigned) time(NULL)); 
	
	// Mientras que no rellenemos la lista con todos los targets que queremos calcular:
	// calculamos las traslaciones, que serán alrededor de una esfera.
	while(trayectoria.size()<100)
	{
		paux[0]= (rand()%801)-400; 			// X entre -400 y 400
		paux[1]= (rand()%(1200-800))+800; 	// Y entre 800 y 1200
		paux[2]= (rand()%(600-200))+200;  	// Z entre 200 y 600

		if(!listaPuntos.contains(paux))
		{
			listaPuntos.append(paux);
			pose[0] = paux[0];	pose[1] = paux[1];	pose[2] = paux[2];
			trayectoria.enqueue(pose);
		}
	}
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
 * @brief MÉTODO ENVIAR. Mira la pestaña en la que se encuentra el usuario de la GUI:
 * 	- Si está en la primera pestaña envía targets de tipo POSE6D sueltos, leyendo los 
 *    parámetros de traslación y rotación de la GUI. Las trayectorias las envía 
 * 	  directamente el compute para que el tester no se quede bloqueado.
 * 	- Si está en la segunda pestaña envía targets de tipo AXISALIGN.
 * 	- Si está en la tercera pestaña envía targets de tipo ALONGAXIS.
 * 	- Si está en la cuarta pestaña envía al robot a la posición de HOME.
 * 	- Si está en la quinta pestaña envía al robot la apertura de los dedos del robot.
 * 
 * @return void
 */ 
void SpecificWorker::enviar()
{
	// Levnatamos bandera para enviar también al RCIS:
	banderaRCIS = true;
	// Si estamos en la pestaña 0 (la de Pose6D), entonces podemos enviar una pose suelta 
	// de tipo Pose6D. Para enviar la pose suelta leemos de la interfaz del usuario
	if(pestanias->tabText(pestanias->currentIndex()) == "Pose6D")
	{
		QVec p = QVec(6);
		p[0] = poseTX->value();		p[1] = poseTY->value();		p[2] = poseTZ->value(); //TRASLACIONES
		p[3] = poseRX->value();		p[4] = poseRY->value();		p[5] = poseRZ->value(); //ROTACIONES
		enviarPose6D(p);
	}

	// Estamos en la segunda pestaña: Axis Align. Enviamos target del tipo AXISALIGN.
	if(pestanias->tabText(pestanias->currentIndex()) == "Axis Align" )
		enviarAxisAlign();
	if(pestanias->tabText(pestanias->currentIndex()) == "Move Along Axis" )
		moveAlongAxis();
	if(pestanias->tabText(pestanias->currentIndex()) == "Home" )
		goHome(part_home->currentText());
	if(pestanias->tabText(pestanias->currentIndex()) == "Fingers" )
		closeFingers();
}



/**
 * @brief Método ACTUALIZAR INNERMODEL. Actualiza el InnerModel con las nuevas posiciones 
 * de los motores del robot.  
 * 
 * @return void
 */ 
void SpecificWorker::actualizarInnerModel()
{
	try 
	{
		printf("%s %d\n", __FILE__, __LINE__);
		RoboCompJointMotor::MotorStateMap mMap = jointmotor_proxy->getMotorStateMap( this->motorList);
		printf("%s %d\n", __FILE__, __LINE__);

		for(uint j=0; j<motorList.size(); j++)
		{
			//printf("%s\n", motorList[j].c_str());
			innerModel->updateJointValue(QString::fromStdString(motorList[j]), mMap.at(motorList[j]).pos);
		}
		printf("%s %d\n", __FILE__, __LINE__);
	} catch (const Ice::Exception &ex) {cout<<"--> Excepción en actualizar InnerModel: "<<ex<<endl;	}
}

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
		try
		{
			innermodelmanager_proxy->setPoseFromParent("target",p);
		}
		catch (const Ice::Exception &ex){ cout<<"RCIS connection problem "<<ex<<endl; }
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

		//moverTargetEnRCIS(p); 
			
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
   	//weights.rx = 1;		weights.ry = 0;		weights.rz = 1;
   	
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

// 				// ÑAPA: Cuando se le envía una trayectoria pruebaMatlab después de cada target se le lleva al home.
// 				if(banderaNiapa==true)
// 				{
// 					sleep(1);
// 					qDebug()<<"banderaNiapa"<<banderaNiapa;
// 					goHome(QString::fromStdString(part));
// 					sleep(1);
// 				}
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



/*----------------------------------------------------------------------------------*/

void SpecificWorker::abrirPinza()
{	
	try
	{	
		qDebug() << __FUNCTION__ << "Open fingers";
		bodyinversekinematics_proxy->setRobot(1);
		bodyinversekinematics_proxy->setFingers((T)abrirPinzaValor->value());

// 		bodyinversekinematics_proxy->setRobot(0);
// 		bodyinversekinematics_proxy->setFingers((T)abrirPinzaValor->value());
	} catch (Ice::Exception ex) {cout <<"ERROR EN ABRIR PINZA: "<< ex << endl;}
}


void SpecificWorker::cerrarPinza()
{
	try
	{	
		qDebug() << __FUNCTION__ << "Close fingers";
		bodyinversekinematics_proxy->setRobot(1);
		bodyinversekinematics_proxy->setFingers((T)cerrarPinzaValor->value());

// 		bodyinversekinematics_proxy->setRobot(0);
// 		bodyinversekinematics_proxy->setFingers((T)cerrarPinzaValor->value());
	} catch (Ice::Exception ex) { cout << "ERROR EN CERRAR PINZA: "<<ex << endl;}
}


void SpecificWorker::posicionInicial()
{
	try
	{
		RoboCompBodyInverseKinematics::Pose6D pose6D;
		pose6D.x = PX->value();
		pose6D.y = PY->value();
		pose6D.z = PZ->value();
		pose6D.rx = RX->value();
		pose6D.ry = RY->value();
		pose6D.rz = RZ->value();	

		QVec pose = QVec::zeros(6);
		pose[0] = pose6D.x/1000; pose[1] = pose6D.y/1000; pose[2] = pose6D.z/1000;

		moverTargetEnRCIS(pose);

		RoboCompBodyInverseKinematics::WeightVector weights;
		weights.x = 1; 		weights.y = 1; 		weights.z = 1;
		weights.rx = 1; 	weights.ry = 1; 	weights.rz = 1; 

		std::string part = "RIGHTARM";
		bodyinversekinematics_proxy->setRobot(1);
		bodyinversekinematics_proxy->setTargetPose6D(part, pose6D, weights, 250);

// 		bodyinversekinematics_proxy->setRobot(0);
// 		bodyinversekinematics_proxy->setTargetPose6D(part, pose6D, weights, 250);

		part = "HEAD";
		RoboCompBodyInverseKinematics::Axis axis;
		axis.x = 0; axis.y = -1; axis.z = 0;
// 		bodyinversekinematics_proxy->setRobot(1);
		bodyinversekinematics_proxy->pointAxisTowardsTarget(part, pose6D, axis, false, 0);	

// 		bodyinversekinematics_proxy->setRobot(0);
// 		bodyinversekinematics_proxy->pointAxisTowardsTarget(part, pose6D, axis, false, 0);	
	}
 	catch(Ice::Exception ex){ std::cout<< __FUNCTION__ << __LINE__ << "ERROR POSICION INICIAL: "<<ex<<endl;}

}

void SpecificWorker::posicionCoger()
{
	try
	{
		RoboCompBodyInverseKinematics::Pose6D pose6D;
		pose6D.x = PX_2->value();
		pose6D.y = PY_2->value();
		pose6D.z = PZ_2->value();
		pose6D.rx = RX_2->value();
		pose6D.ry = RY_2->value();
		pose6D.rz = RZ_2->value();	

		QVec pose = QVec::zeros(6);
		pose[0] = pose6D.x/1000; pose[1] = pose6D.y/1000; pose[2] = pose6D.z/1000;

		moverTargetEnRCIS(pose);

		RoboCompBodyInverseKinematics::WeightVector weights;
		weights.x = 1; 		weights.y = 1; 		weights.z = 1;
		weights.rx = 1; 	weights.ry = 1; 	weights.rz = 1; 

		std::string part = "RIGHTARM";
		bodyinversekinematics_proxy->setRobot(1);
		bodyinversekinematics_proxy->setTargetPose6D(part, pose6D, weights, 250);

// 		bodyinversekinematics_proxy->setRobot(0);
// 		bodyinversekinematics_proxy->setTargetPose6D(part, pose6D, weights, 250);

		part = "HEAD";
		RoboCompBodyInverseKinematics::Axis axis;
		axis.x = 0; axis.y = -1; axis.z = 0;
// 		bodyinversekinematics_proxy->setRobot(1);
		bodyinversekinematics_proxy->pointAxisTowardsTarget(part, pose6D, axis, false, 0);	

// 		bodyinversekinematics_proxy->setRobot(0);
// 		bodyinversekinematics_proxy->pointAxisTowardsTarget(part, pose6D, axis, false, 0);
	}
 	catch(Ice::Exception ex){ std::cout<< __FUNCTION__ << __LINE__ << "ERROR EN POSICION COGER: "<<ex<<endl;}
}


void SpecificWorker::posicionSoltar()
{

	try
	{
		RoboCompBodyInverseKinematics::Pose6D pose6D;
		pose6D.x = PX_3->value();
		pose6D.y = PY_3->value();
		pose6D.z = PZ_3->value();
		pose6D.rx = RX_3->value();
		pose6D.ry = RY_3->value();
		pose6D.rz = RZ_3->value();	

		QVec pose = QVec::zeros(6);
		pose[0] = pose6D.x/1000; pose[1] = pose6D.y/1000; pose[2] = pose6D.z/1000;

		moverTargetEnRCIS(pose);

		RoboCompBodyInverseKinematics::WeightVector weights;
		weights.x = 1; 		weights.y = 1; 		weights.z = 1;
		weights.rx = 1; 	weights.ry = 1; 	weights.rz = 1; 

		std::string part = "RIGHTARM";
		bodyinversekinematics_proxy->setRobot(1);
		bodyinversekinematics_proxy->setTargetPose6D(part, pose6D, weights, 250);
// 		bodyinversekinematics_proxy->setRobot(0);
// 		bodyinversekinematics_proxy->setTargetPose6D(part, pose6D, weights, 250);

		part = "HEAD";
		RoboCompBodyInverseKinematics::Axis axis;
		axis.x = 0; axis.y = -1; axis.z = 0;
// 		bodyinversekinematics_proxy->setRobot(1);
		bodyinversekinematics_proxy->pointAxisTowardsTarget(part, pose6D, axis, false, 0);	
// 		bodyinversekinematics_proxy->setRobot(0);
// 		bodyinversekinematics_proxy->pointAxisTowardsTarget(part, pose6D, axis, false, 0);	
	}
 	catch(Ice::Exception ex){ std::cout<< __FUNCTION__ << __LINE__ << "ERROR EN POSICION SOLTAR: "<<ex<<endl;}

}



void SpecificWorker::izquierdoRecoger()
{
	try
	{
		RoboCompBodyInverseKinematics::Pose6D pose6D;
		pose6D.x = PX_4->value();
		pose6D.y = PY_4->value();
		pose6D.z = PZ_4->value();
		pose6D.rx = RX_4->value();
		pose6D.ry = RY_4->value();
		pose6D.rz = RZ_4->value();	

		QVec pose = QVec::zeros(6);
		pose[0] = pose6D.x/1000; pose[1] = pose6D.y/1000; pose[2] = pose6D.z/1000;

		moverTargetEnRCIS(pose);

		RoboCompBodyInverseKinematics::WeightVector weights;
		weights.x = 1; 		weights.y = 1; 		weights.z = 1;
		weights.rx = 1; 	weights.ry = 1; 	weights.rz = 1; 


		std::string part = "LEFTARM";

		/* AL ROBOT :*/
		bodyinversekinematics_proxy->setRobot(1);
		bodyinversekinematics_proxy->setTargetPose6D(part, pose6D, weights, 250);

		/* AL RCIS*/
// 		bodyinversekinematics_proxy->setRobot(0);
// 		bodyinversekinematics_proxy->setTargetPose6D(part, pose6D, weights, 250);
	}
 	catch(Ice::Exception ex){ std::cout<< __FUNCTION__ << __LINE__ << "ERROR EN IZQUIERDO RECOGER: "<<ex<<endl;}

}


void SpecificWorker::retroceder()
{
	try
	{
		RoboCompBodyInverseKinematics::Pose6D pose6D;
		pose6D.x = PX_5->value();
		pose6D.y = PY_5->value();
		pose6D.z = PZ_5->value();
		pose6D.rx = RX_5->value();
		pose6D.ry = RY_5->value();
		pose6D.rz = RZ_5->value();	

		QVec pose = QVec::zeros(6);
		pose[0] = pose6D.x/1000; pose[1] = pose6D.y/1000; pose[2] = pose6D.z/1000;

		moverTargetEnRCIS(pose);

		RoboCompBodyInverseKinematics::WeightVector weights;
		weights.x = 1; 		weights.y = 1; 		weights.z = 1;
		weights.rx = 1; 	weights.ry = 1; 	weights.rz = 1; 

		std::string part = "RIGHTARM";

		/* AL ROBOT :*/
		bodyinversekinematics_proxy->setRobot(1);
		bodyinversekinematics_proxy->setTargetPose6D(part, pose6D, weights, 250);

		/* AL RCIS */
// 		bodyinversekinematics_proxy->setRobot(0);
// 		bodyinversekinematics_proxy->setTargetPose6D(part, pose6D, weights, 250);
	}
 	catch(Ice::Exception ex){ std::cout<< __FUNCTION__ << __LINE__ << "ERROR EN RETROCEDER: "<<ex<<endl;}

}

void SpecificWorker::goHomeR()
{
	goHome("RIGHTARM");
}


void SpecificWorker::izquierdoOfrecer()
{
	try
	{
		RoboCompBodyInverseKinematics::Pose6D pose6D;
		pose6D.x = PX_6->value();
		pose6D.y = PY_6->value();
		pose6D.z = PZ_6->value();
		pose6D.rx = RX_6->value();
		pose6D.ry = RY_6->value();
		pose6D.rz = RZ_6->value();	

		QVec pose = QVec::zeros(6);
		pose[0] = pose6D.x/1000; pose[1] = pose6D.y/1000; pose[2] = pose6D.z/1000;

		moverTargetEnRCIS(pose);

		RoboCompBodyInverseKinematics::WeightVector weights;
		weights.x = 1; 		weights.y = 1; 		weights.z = 1;
		weights.rx = 1; 	weights.ry = 1; 	weights.rz = 1; 

		std::string part = "LEFTARM";
		bodyinversekinematics_proxy->setRobot(1);
		bodyinversekinematics_proxy->setTargetPose6D(part, pose6D, weights, 250);

// 		bodyinversekinematics_proxy->setRobot(0);
// 		bodyinversekinematics_proxy->setTargetPose6D(part, pose6D, weights, 250);

		part = "HEAD";
		RoboCompBodyInverseKinematics::Axis axis;
		axis.x = 0; axis.y = -1; axis.z = 0;
// 		bodyinversekinematics_proxy->setRobot(1);
		bodyinversekinematics_proxy->pointAxisTowardsTarget(part, pose6D, axis, false, 0);	

// 		bodyinversekinematics_proxy->setRobot(0);
// 		bodyinversekinematics_proxy->pointAxisTowardsTarget(part, pose6D, axis, false, 0);	
	}
 	catch(Ice::Exception ex){ std::cout<< __FUNCTION__ << __LINE__ << "ERROR EN IZQUIERDO OFRECER: "<<ex<<endl;}

}

/// SUBSCRIBED METHOD FROM APRILTAGS

void SpecificWorker::newAprilTag(const tagsList& tags)
{
	marcaApril = QVec::zeros(6);
 	for(uint i=0; i<tags.size(); i++)
 	{
 		//qDebug() << tags[i].id << tags[i].tx << tags[i].ty << tags[i].tz << tags[i].rx << tags[i].ry << tags[i].rz;
		if( tags[i].id == 2 )  //Taza
		{
			mutex->lock();
				marcaApril[0] = tags[0].tx; marcaApril[1] = tags[0].ty; marcaApril[2] = tags[0].tz; marcaApril[3] = tags[0].rx; marcaApril[4] = tags[0].ry; marcaApril[5] = tags[0].rz;    
			mutex->unlock();
		}
		if( tags[i].id == 10 )  //Mano
		{
			mutex->lock();
				manoApril[0] = tags[0].tx; manoApril[1] = tags[0].ty; manoApril[2] = tags[0].tz; manoApril[3] = tags[0].rx; manoApril[4] = tags[0].ry; manoApril[5] = tags[0].rz;    
			mutex->unlock();
		}
 	}
}


void SpecificWorker::ballisticPartToAprilTarget(int xoffset)
{
	InnerModelNode *nodeParent = innerModel->getNode("rgbd");
	InnerModelTransform *node = innerModel->newTransform("marca", "static", nodeParent, 0, 0, 0, 0, 0, 0, 0);
	nodeParent->addChild(node);
	
	mutex->lock();
		innerModel->updateTransformValues("marca",marcaApril.x(), marcaApril.y(), marcaApril.z(), marcaApril.rx(), marcaApril.ry(), marcaApril.rz(), "rgbd");	
	mutex->unlock();
	
	QVec marcaTInWorld = innerModel->transform("world", QVec::zeros(3), "marca");
	
	marcaTInWorld[0] += xoffset;   ///OJO ESTO SOLO VALE PARA LA MANO DERECHA
	
	//QVec marcaRInWorld = innerModel->getRotationMatrixTo("world","marca").extractAnglesR_min();
	QVec marcaRInWorld = QVec::vec3(3.15,-1.57,0);
	QVec marcaInWorld(6);

	marcaInWorld.inject(marcaTInWorld,0);
	marcaInWorld.inject(marcaRInWorld,3);
	enviarPose6D( marcaInWorld );
	qDebug() << "Sent to target" << marcaInWorld;
	
	innerModel->removeNode("marca");
}


void SpecificWorker::finePartToAprilTarget()
{
	//Here we should have both apriltags on sight
	// The target and the grabPositionHandL
	
	//Now we want to tell BIK to change its endEffector to manoApril once transformed to the current endEffector reference system	
	InnerModelNode *nodeParent = innerModel->getNode("rgbd");
	InnerModelTransform *node = innerModel->newTransform("handInCamera", "static", nodeParent, 0, 0, 0, 0, 0, 0, 0);
	nodeParent->addChild(node);
	
	mutex->lock();
		innerModel->updateTransformValues("marca",manoApril.x(), manoApril.y(), manoApril.z(), manoApril.rx(), manoApril.ry(), manoApril.rz(), "rgbd");	
	mutex->unlock();
	
	QVec manoTInEndEffector = innerModel->transform("grabPositionHandR", QVec::zeros(3), "handInCamera");
	QVec manoRInEndEffector = innerModel->getRotationMatrixTo("grabPositionHandR","handInCamera").extractAnglesR_min();
	QVec manoInEndEffector(6);
	manoInEndEffector.inject(manoTInEndEffector,0);
	manoInEndEffector.inject(manoRInEndEffector,3);  //This is the 6D pose from current endEffector to mano apriltags
	//We send now to BIK the new endEffector pose
	try 
	{
		//bodyinversekinematics_proxy->setNewEndEffectorRelativeToCurrentEndEffector(part, "visualGrabPositionHandR", Pose6D);
	} catch (exception) 
	{
		
	}
	
	//And finally, we tell BIK to move the new endEffector to marcaApril, already in rgbd reference frame
	// ballisticPartToAprilTarget(0);
}

