#include "inversedkinematic.h"
///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
/**
 * \brief default constructor
 */
InversedKinematic::InversedKinematic()
{
	repetitions = 0;
}
/**
 * \brief default destructor
 */
InversedKinematic::~InversedKinematic()
{

}
///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
/**
 * \brief This method prepares the target in order to execute correctly the method that
 * calculates the inverse kinematic, the levenbergMarquardt method.
 * @param bodypart_
 * @param immermodel_
 */
void InversedKinematic::solveTarget(BodyPart *bodypart_, InnerModel *innermodel_)
{
	bodypart       = bodypart_;
	innermodel     = innermodel_;
	Target target  = bodypart->getTargetList().head();

	if(target.getTargetType()==Target::ADVANCEAXIS and target.getTargetState()==Target::IN_PROCESS)
	{
		QVec axis = target.getTargetAxis() * target.getTargetStep(); 				//Scale the unitary vector along direction by distance
		QVec pose = innermodel->transform("root", axis, bodypart->getTipName());	//We go from tip to root

		QMat matrix_from_TIP_to_ROOT = innermodel->getRotationMatrixTo("root", bodypart->getTipName());
		QVec angles_rot = matrix_from_TIP_to_ROOT.extractAnglesR_min();

		innermodel->updateTransformValues(target.getTargetNameInInnerModel(),pose.x(), pose.y(), pose.z(), angles_rot.x(), angles_rot.y(), angles_rot.z(), "root");

		levenbergMarquardt(target);
		repetitions++;
		return;
	}
	if((target.getTargetType()== Target::ALIGNAXIS or target.getTargetType()==Target::POSE6D) and target.getTargetState()==Target::IN_PROCESS)
	{
		levenbergMarquardt(target);
		repetitions++;
		return;
	}
}
/**
 * \brief This method indicates if a target could be deleted of the bodypart list of target
 * @return bool true (the target can be deleted if we have repeated his execution more than 8 times or when the error is very small) or
 * false (the target couldn't be deleted if we haven't repated yet his execution more than 8 times or when the error is bigger than the
 * threshold.
 */
bool InversedKinematic::deleteTarget()
{
	static QVec angles  = QVec::zeros(checkMotors().size());
	QMat We             = QMat::makeDiagonal(bodypart->getTargetList().head().getTargetWeight());
	QVec error          = We*computeErrorVector(bodypart->getTargetList().head());
	QVec errorT         = QVec::vec3(error.x(), error.y(), error.z());
	QVec errorR         = QVec::vec3(error.rx(), error.ry(), error.rz());

// 	qDebug()<<"-----------------------------------------------------------------";
// 	qDebug()<<"Milliseconds computed: "<<bodypart->getTargetList().head().getTargetTimeExecution()*1000;
// 	qDebug()<<"Repeticiones: "<<repetitions;
// 	qDebug()<<"Error T: "<<errorT.norm2();
// 	qDebug()<<"Error R: "<<errorR.norm2();
// 	qDebug()<<"-----------------------------------------------------------------";

	QVec restaAngles (checkMotors().size());
	for(int i=0; i<bodypart->getTargetList().head().getTargetFinalAngles().size(); i++)
		restaAngles[i] = fabs(angles[i]-bodypart->getTargetList().head().getTargetFinalAngles()[i]);
	angles = bodypart->getTargetList().head().getTargetFinalAngles();

	if(
	   (bodypart->getTargetList().head().getTargetTimeExecution()>1 or (errorT.norm2()<0.0001 and errorR.norm2()<0.001)/* or restaAngles.norm2()<0.0001*/)
	   and
	   bodypart->getTargetList().head().getTargetTimeExecution()>0.1)

	{
		repetitions = 0;
		return true;
	}
	else
		return false;
}
///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
/**
 * \brief this method Calculates the error vector resultant of two operations :
 * 1 ) The operation to calculate translations : puntoObjetivo - endEffector .
 * 2 ) The operation to calculate the error of rotations.
 * @param Target
 * @return QVec error vector .
 */
QVec InversedKinematic::computeErrorVector(Target& target)
{
	QVec finalError = QVec::zeros(6); //Vector error final
	QString frameBase = checkMotors().last(); // Frame where the errors will be referred

	if(target.getTargetType()==Target::POSE6D or target.getTargetType()==Target::ADVANCEAXIS)
	{
		QVec target_Traslations_in_FrameBase = innermodel->transform(frameBase, QVec::zeros(3), target.getTargetNameInInnerModel());
		QVec tip_Traslations_in_FrameBase 	 = innermodel->transform(frameBase, QVec::zeros(3), bodypart->getTipName());
		QVec error_Traslations_in_FrameBase	 = target_Traslations_in_FrameBase - tip_Traslations_in_FrameBase;

		// Calculamos el error de rotación: ¿Cúanto debe girar last Joint para que el tip quede orientado como el target?
		// 1) Calculamos la matriz de rotación que nos devuelve los ángulos que debe girar el tip para orientarse como el target:
		QMat matTargetInTip = innermodel->getRotationMatrixTo(bodypart->getTipName(), target.getTargetNameInInnerModel());
		// 2) Calculamos la matriz de rotación que nos devuelve los ángulos que debe girar el lastJoint para orientarse como el tip:
		QMat matTipInFrameBase = innermodel->getRotationMatrixTo(frameBase, bodypart->getTipName());
		// 3) Calculamos el error de rotación entre el target y el tip:
		QVec targetRInTip = matTargetInTip.extractAnglesR_min();
		// Calculamos el equivalente de rotar en X el Tip pero en el sistema de referencia del lastJoint (frameBase)
		QVec firstRot = matTipInFrameBase * QVec::vec3(targetRInTip[0],0,0);
		Rot3D matFirtRot(firstRot[0], firstRot[1], firstRot[2]);
		// Hacemos lo mismo con la rotación en Y
		QVec secondRot = matTipInFrameBase * QVec::vec3(0,targetRInTip[1],0);
		Rot3D matSecondRot(secondRot[0], secondRot[1], secondRot[2]);
		// Hacemos lo mismo con la rotación en Z
		QVec thirdRot = matTipInFrameBase * QVec::vec3(0,0,targetRInTip[2]);
		Rot3D matThirdRot(thirdRot[0], thirdRot[1], thirdRot[2]);
		// Creamos la matríz resultante de las rotaciones calculadas multipiclando las matrices calculasdas anteriormente (ojo al orden correcto)
		QMat matResulInFrameBase =  (matFirtRot * matSecondRot) * matThirdRot;
		// Extraemos los ángulos de la matriz calculada que ya equivalen a las rotaciones del tip vistas desde el frameBase
		QVec error_Rotations_in_FrameBase = matResulInFrameBase.extractAnglesR_min();

		finalError.inject(error_Traslations_in_FrameBase,0);
		finalError.inject(error_Rotations_in_FrameBase, 3);
	}
	if(target.getTargetType()==Target::ALIGNAXIS)
	{
		// compute a vector going from tip to target
		QVec targetInTip = innermodel->transform(bodypart->getTipName(),QVec::zeros(3),target.getTargetNameInInnerModel()).normalize();

		QVec a = target.getTargetAxis();
		QVec o = a^targetInTip; // axis to rotate

		float co = (a * targetInTip);
		float si = (o.norm2());  //Angle to rotate
		float ang = atan2(si,co);
		QMat c = o.crossProductMatrix();
		QMat r = QMat::identity(3) + (c * (T)sin(ang)) + (c*c)*(T)(1.f-cos(ang));  ///Rodrigues formula to compute R from <vector-angle>

		QVec erroRotaciones = r.extractAnglesR_min();

		// 4) Pasamos los errores de rotación del tip al last joint.
		QVec errorRInTip = erroRotaciones;
		QMat matTipInFrameBase = innermodel->getRotationMatrixTo(frameBase, bodypart->getTipName());
		// Calculamos el equivalente de rotar en X el Tip pero en el sistema de referencia del lastJoint (frameBase)
		QVec firstRot = matTipInFrameBase * QVec::vec3(errorRInTip[0],0,0);
		Rot3D matFirtRot(firstRot[0], firstRot[1], firstRot[2]);
		// Hacemos lo mismo con la rotación en Y
		QVec secondRot = matTipInFrameBase * QVec::vec3(0,errorRInTip[1],0);
		Rot3D matSecondRot(secondRot[0], secondRot[1], secondRot[2]);
		// Hacemos lo mismo con la rotación en Z
		QVec thirdRot = matTipInFrameBase * QVec::vec3(0,0,errorRInTip[2]);
		Rot3D matThirdRot(thirdRot[0], thirdRot[1], thirdRot[2]);
		// Creamos la matríz resultante de las rotaciones calculadas multipiclando las matrices calculasdas anteriormente (ojo al orden correcto)
		QMat matResulInFrameBase =  (matFirtRot * matSecondRot) * matThirdRot;
		// Extraemos los ángulos de la matriz calculada que ya equivalen a las rotaciones del tip vistas desde el frameBase
		QVec errorRInFrameBase = matResulInFrameBase.extractAnglesR_min();

		finalError.inject(errorRInFrameBase,3);
	}
	return finalError;
}
/**
 * \brief Creates the Jacobian matrix of the direct kinematic function to the data out of the innerModel .
 * The filling goes by columns , with the columns and rows motors tx , ty, tz , rx, ry and rz .
 * You will pass a table of as many elements as you work with motors . If there are zero position
 * motors calculates translations and rotations . If there is a 1 in the motor position fills the column to 0
 * @param motors is a vector of 0 and 1 , with as many items as motors that are working with the
 * Reverse . 0 means that the motor is not locked (calculated column associated Jacobian
 * motors) and 1 means that the motor if blocked , Jacobian column filled with 0 .
 * @return The Jacobian matrix QMat
 */
QMat InversedKinematic::jacobian(QVec motors)
{
	// Initialize the Jacobian matrix of the size of the target point having 6 ELEMENTS [ tx , ty, tz , rx , ry, rz ]
	// The number of motors from the list of joints : 6 rows by n columns. We also initialize a vector of zeros
	QStringList considermotors = checkMotors();
	QMat jacob(6, considermotors.size(), 0.f); //QMat jacob(6, bodypart->getMotorList().size(), 0.f);  //6 output variables
 	QVec zero = QVec::zeros(3);
 	int j=0; //índice de columnas de la matriz: MOTORES

 	foreach(QString linkName, considermotors) //foreach(QString linkName, bodypart->getMotorList())
 	{
		if(motors[j] == 0)
		{
			QString frameBase = considermotors.last(); //QString frameBase = bodypart->getMotorList().last();

			// TRASLACIONES: con respecto al último NO traslada
			QVec axisTip    = innermodel->getJoint(linkName)->unitaryAxis(); //vector de ejes unitarios
			axisTip         = innermodel->transform(frameBase, axisTip, linkName);
			QVec axisBase   = innermodel->transform(frameBase, zero, linkName);
			QVec axis       = axisBase - axisTip;
			QVec toEffector = (axisBase -innermodel->transform(frameBase, zero, bodypart->getTipName()) );
			QVec res        = toEffector.crossProduct(axis);

			jacob(0,j) = res.x();
			jacob(1,j) = res.y();
			jacob(2,j) = res.z();

			// ROTACIONES
			QVec axisTip2   = innermodel->getJoint(linkName)->unitaryAxis(); //vector de ejes unitarios en el que gira
			axisTip2        = innermodel->transform(frameBase, axisTip2, linkName); //vector de giro pasado al hombro.
			QVec axisBase2  = innermodel->transform(frameBase, zero, linkName); //motor al hombro
			QVec axis2      = axisBase2 - axisTip2; //vector desde el eje de giro en el sist. hombro, hasta la punta del eje de giro en el sist. hombro.

			jacob(3,j) = axis2.x();
			jacob(4,j) = axis2.y();
			jacob(5,j) = axis2.z();
		}
 		j++;
 	}
 	return jacob;
}
/**
 * \brief Method levenbergMarquardt. Performs the Levenberg -Marquardt extended for cases
 * That the matrix must be inverted out singular. Run until a threshold of acceptance
 * of the solution is reached by dialing
 * User. Returns the values ​​of the angles in order to be changed.
 * IT WORKS
 * - Jt * J + nu * I * Increases = Jt * e
 * A + nu * I * Increases Ep = g *
 *
 * @param Target ...
 * @return Void
 */
void InversedKinematic::levenbergMarquardt(Target& target)
{
	const int  kMax = 100;
	const QMat Identidad = QMat::identity(checkMotors().size()); //matriz identidad

	int k=0, v=2, auxInt;                                        //iterador, variable para descenso y un entero auxiliar
	QVec incrementos, aux;                                       //vector de incrementos y vector auxiliar para guardar cambios
	QVec motors (checkMotors().size());                          // lista de motores para rellenar el jacobiano.
	QVec angles = computeAngles();                               // ángulos iniciales de los motores.
	QMat We     = QMat::makeDiagonal(target.getTargetWeight());  //matriz de pesos para compensar milímietros con radianes.
	QVec error  = We * computeErrorVector(target);               //error de la posición actual con la deseada.
	QMat J      = jacobian(motors);                              //JACOBIANO
	QMat H      = J.transpose()*(We*J);                          // HESSIANO -->ERROR
	QVec g      = J.transpose()*(error);
	bool stop   = false;
	bool smallInc = false;
	bool nanInc   = false;
	float ro      = 0;
	float n       = pow(10, -3)*H.getDiagonal().max(auxInt);	//¿por que 0,01?
	
// 	We.print("WE");
// 	error.print("error");
// 	J.print("J");
// 	H.print("H");
// 	g.print("g");
	
	

	while((stop==false) and (k<kMax) and (smallInc == false) and (nanInc == false))
	{
		k++;
		do{
			try
			{
				incrementos = (H + (Identidad*n)).invert() * g;
				for(int i=0; i<incrementos.size(); i++)
					if(isnan(incrementos[i])) 						///NAN increments
					{
						nanInc = true;
						bodypart->getTargetList()[0].setTargetFinalState(Target::NAN_INCS);
						break;
					}
				if(nanInc == true) break;
			}
			catch(QString str){ qDebug()<< __FUNCTION__ << __LINE__ << "SINGULAR MATRIX EXCEPTION";	}

			if(incrementos.norm2() <= 0.0001)   ///Too small increments
			{
				//stop = true;
				smallInc = true;
				bodypart->getTargetList()[0].setTargetFinalState(Target::LOW_INCS);
				break;
			}
			else
			{
				aux = angles-incrementos;
				computeFloatModule(aux, 2*M_PI); // NORMALIZAMOS

				if(outLimits(aux, motors) == true)		///COMPROBAR SI QUEDAN MOTORES LIBRES, SINO SALIR!!!!!!!!!!!
				{
					// Recalculamos el Jacobiano, el Hessiano y el vector g. El error es el mismo que antes
					// puesto que NO aplicamos los cambios (los ángulos nuevos).
					J = jacobian(motors);
					H = J.transpose()*(We*J);
					g = J.transpose()*(error);
				}
				updateAngles(aux); // Metemos los nuevos angles LUEGO HAY QUE DESHACER EL CAMBIO.
				ro = ((error).norm2() - (We*computeErrorVector(target)).norm2()); //Check for sign of error derivative
				if(ro > 0) //if sign is positive
				{
					motors.set((T)0);
					// Estamos descendiendo correctamente --> errorAntiguo > errorNuevo.
					stop = (We*computeErrorVector(target)).norm2() <= 0.001; //1 milimetro de error
					angles = aux;
					// Recalculamos con nuevos datos.
					error = We*computeErrorVector(target);
					J = jacobian(motors);
					H = J.transpose()*(We*J);
					g = J.transpose()*(error);
					//n = n * std::max(1.f/3.f, (float)(1.f-pow(2*ro - 1,3)));
					n = n * 0.9;
					v=2;
				}
				else
				{
					updateAngles(angles); //volvemos a los ángulos viejos.
					//increase landa values in regularization matrix
					n = n*v;
					v = 2*v;
				}
			}//fin else incrementos no despreciables.
		}while(ro<=0 and stop==false);
		stop = error.norm2() <= 0.001; //1 milimetro de error
	}
	bodypart->getTargetList()[0].setTargetState(Target::FINISH);

	if (stop == true)           bodypart->getTargetList()[0].setTargetFinalState(Target::LOW_ERROR);
	else if ( k>=kMax)          bodypart->getTargetList()[0].setTargetFinalState(Target::KMAX);
	else if ( smallInc == true) bodypart->getTargetList()[0].setTargetFinalState(Target::LOW_INCS);
	else if ( nanInc == true)   bodypart->getTargetList()[0].setTargetFinalState(Target::NAN_INCS);

	bodypart->getTargetList()[0].setTargetError(error);
	bodypart->getTargetList()[0].setTargetFinalAngles(angles);
}
///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
/**
 * \brief This method gets the angles of all the cinematic motors.
 * @return QVec motors angles
 */
QVec InversedKinematic::computeAngles()
{
	QVec angles;

	for(int i=0; i<checkMotors().size(); i++)//for(int i=0; i<bodypart->getMotorList().size(); i++)
	{
		float angle = innermodel->getJoint(bodypart->getMotorList()[i])->getAngle();
		//qDebug()<<"Obteniedo angulo "<<angle<<" del motor "<<bodypart->getMotorList()[i];
		angles.push_back(angle);
	}
	return angles;
}
/**
 * \brief This method normalizes the angles in a range.
 * @param angles
 * @param mod
 */
void InversedKinematic::computeFloatModule(QVec& angles, float mod)
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
/**
 * \brief This method prove if some motors are in their limit angles
 * @param angles
 * @param motors
 */
bool InversedKinematic::outLimits(QVec& angles, QVec& motors)
{
	bool noSupera = false;
	float limiteMin, limiteMax;

	for(int i=0; i<checkMotors().size(); i++)//for(int i=0; i<bodypart->getMotorList().size(); i++)
	{
		limiteMin = innermodel->getJoint(bodypart->getMotorList()[i])->min;
		limiteMax = innermodel->getJoint(bodypart->getMotorList()[i])->max;

		if(angles[i]<limiteMin or angles[i]>limiteMax)
		{
			noSupera = true;
			motors[i] = 1;

			if(angles[i]<limiteMin) angles[i] = limiteMin;
			if(angles[i]>limiteMax) angles[i] = limiteMax;
			//qDebug()<< __FUNCTION__ << "MIN: "<<limiteMin<<" MAX: "<<limiteMax<<" ANGLE: "<<angles[i]<<" MOTORES: "<<bodypart->getMotorList()[i];
		}
	}
	return noSupera;
}
/**
 * \brief This method update the angles values of the motors.
 * @param new_angles
 */
void InversedKinematic::updateAngles(QVec new_angles)
{
	for(int i=0; i<checkMotors().size(); i++)//for(int i=0; i<bodypart->getMotorList().size(); i++)
		innermodel->updateJointValue(bodypart->getMotorList()[i], new_angles[i]);
}
/**
 * \brief this method looks the target weights. If the target is only for traslation, we delete the
 * wright motors of de bodyPart (the tree last)
 * @return the motors list
 */
QStringList InversedKinematic::checkMotors()
{
	QStringList motors;
	QVec weights = bodypart->getTargetList().head().getTargetWeight();
	if(weights.rx()==0 and weights.ry()==0 and weights.rz()==0)
	{
		for(int i=0; i<bodypart->getMotorList().size()-3; i++)
			motors<<bodypart->getMotorList()[i];
	}
	else
		motors = bodypart->getMotorList();

	return motors;
}
