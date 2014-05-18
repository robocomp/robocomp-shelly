/*
    Copyright (c) 2014 <copyright holder> <email>

    Permission is hereby granted, free of charge, to any person
    obtaining a copy of this software and associated documentation
    files (the "Software"), to deal in the Software without
    restriction, including without limitation the rights to use,
    copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the
    Software is furnished to do so, subject to the following
    conditions:

    The above copyright notice and this permission notice shall be
    included in all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
    EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
    OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
    NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
    HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
    WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
    FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
    OTHER DEALINGS IN THE SOFTWARE.
*/

#include "cinematica_inversa.h"
#include <qt4/QtCore/qstringlist.h>
#include <boost/graph/graph_concepts.hpp>

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*
 * 								CONSTRUCTORES Y DESTRUCTORES												   *
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/ 
Cinematica_Inversa::Cinematica_Inversa(InnerModel *inner_, QStringList joints_, QString endEffector_) 
		:	inner(inner_), listaJoints(joints_), endEffector(endEffector_)
{
	// Inicializa los atributos de la clase a partir de los : //
	this->ERROR = 0;
}

Cinematica_Inversa::~Cinematica_Inversa()
{
}

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*
 * 										MÉTODOS PÚBLICOS													   *
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/ 
/*
 * Metodo ponerTarget
 * Asigna el valor de la variable de entrada (target) al atributo puntoObjetivo.
 * Es el punto al que queremos llevar el nodo effector o nodo final.
 * Hay que llamar a este metodo cada vez que queramos llevar la mano del robot a 
 * un objetivo distinto: cada vez que llamemos al Levenberg-Marquardt.
 * FUNCIONA.
 */ 
QVec Cinematica_Inversa::resolverTarget(const Target& target)
{
	this->target = target;
	this->puntoObjetivo = target.getPose();
	this->weights = target.getWeights();
	QVec ang;
	
	if(target.getType() == Target::ALIGNAXIS)  //Change current tip to tip of virtual nose
	{
		float len = inner->transform(this->endEffector,QVec::zeros(3),"target").norm2();   //distance from tip to target
		InnerModelNode *nodeTip = inner->getNode(this->endEffector);
		InnerModelTransform *nodeAppex = inner->newTransform("appex", "static", nodeTip, 0, 0, 0, 0, 0, 0, 0);
		nodeTip->addChild(nodeAppex);
		QString axisName = target.getAxisName();
		if(axisName == "x" or axisName == "X")
			inner->updateTransformValues("appex", len, 0, 0, 0, 0, 0, this->endEffector);
		if(axisName == "y" or axisName == "Y")
			inner->updateTransformValues("appex", 0, len, 0, 0, 0, 0, this->endEffector);
		if(axisName == "z" or axisName == "Z")
			inner->updateTransformValues("appex", 0, 0, len, 0, 0, 0, this->endEffector);
		
		ang = levenbergMarquardt();
		inner->removeNode("appex");
	}
	else
		ang = levenbergMarquardt();
		
	return ang;
}

float Cinematica_Inversa::devolverError()
{
	return ERROR;
}

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*
 * 										MÉTODOS PRIVADOS													   *
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/ 

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*
 * 										MÉTODOS DE TRASLACIÓN Y ROTACIÓN									   *
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/ 
/*
 * Metodo jacobian.
 * Crea la matriz jacobiana de la función de cinematica directa con los datos que saca del innerModel. 
 * La va rellenando por columnas, siendo las columnas los motores y las filas la tx, ty, tz, rx, ry y rz.
 * Se le pasa un vector de tantos elementos como motores con los que trabaja. Si hay cero en la posición
 * del motor calcula traslaciones y rotaciones. Si hay un 1 en la posición del motor rellena la columna a 0
 */ 
QMat Cinematica_Inversa::jacobian(QVec motores)
{
	// Inicializamos la matriz jacobiana de tamaño el del punto objetivo que tiene 6 ELEMENTOS [tx, ty, tz, rx, ry, rz]
	// por el número de motores de la lista de articulaciones: 6 filas por n columnas. También inicializamos un vector de ceros
	QMat jacob(this->puntoObjetivo.size(), this->listaJoints.size(), 0.f);
 	QVec zero = QVec::zeros(3);
 	int j=0; //índice de columnas de la matriz: MOTORES
	
 	foreach(QString linkName, this->listaJoints)
 	{
		if(motores[j] == 0)
		{
			// TRASLACIONES: con respecto al último NO traslada
			QVec axisTip = this->inner->getJoint(linkName)->unitaryAxis(); //vector de ejes unitarios
			axisTip = this->inner->transform(this->listaJoints[0], axisTip, linkName);
			QVec axisBase = this->inner->transform(this->listaJoints[0], zero, linkName);
			QVec axis = axisBase - axisTip;
			QVec toEffector = (axisBase - this->inner->transform(this->listaJoints[0], zero, this->endEffector) );		
			QVec res = toEffector.crossProduct(axis);

			jacob(0,j) = res.x();
			jacob(1,j) = res.y();
			jacob(2,j) = res.z();
			
			// ROTACIONES
			QVec axisTip2 = this->inner->getJoint(linkName)->unitaryAxis(); //vector de ejes unitarios en el que gira
			axisTip2 = this->inner->transform(this->listaJoints.last(), axisTip2, linkName); //vector de giro pasado al hombro.
			QVec axisBase2 = this->inner->transform(this->listaJoints.last(), zero, linkName); //motor al hombro
			QVec axis2 = axisBase2 - axisTip2; //vector desde el eje de giro en el sist. hombro, hasta la punta del eje de giro en el sist. hombro. 
			
			jacob(3,j) = axis2.x(); 
			jacob(4,j) = axis2.y();
			jacob(5,j) = axis2.z();	
		}
 		j++;
 	}
//	jacob.print("JACOBIANO TRASLACIONES y ROTACIONES");
 	return jacob;
}

/*
 * Metodo calcularVectorError.
 * Calcula el vector de error resultante de dos operaciones:
 *		1) la operacion para calcular traslaciones: puntoObjetivo-endEffector.
 * 		2) la operacion para calcular el error de rotaciones.
 * FUNCIONA
 */ 
QVec Cinematica_Inversa::calcularVectorError()
{
	QVec errorTotal = QVec::zeros(6); //Vector error final
	
	if(target.getType() == Target::POSE6D)
	{
		// ---> TRASLACIONES: Al punto objetivo le restamos las coordenadas del nodo final endEffector
		QVec errorTraslaciones = QVec::zeros(3);
		QVec auxTraslaciones = QVec::vec3(puntoObjetivo[0], puntoObjetivo[1], puntoObjetivo[2]);
		QVec targetInRoot = inner->transform(this->listaJoints[0], auxTraslaciones ,"world");
		QVec tip = inner->transform(this->listaJoints[0], QVec::zeros(3), this->endEffector);
		errorTraslaciones = targetInRoot - tip;
		//ROTATIONS
		QMat matriz = inner->getRotationMatrixTo(listaJoints.last(), "target") /** Rot3DOX(1.57)*/;
		QVec TARGETenMANO = inner->getTransformationMatrix(listaJoints.last(), "target").extractAnglesR3(matriz);
		QVec angulos1 = QVec::vec3(TARGETenMANO[0], TARGETenMANO[1], TARGETenMANO[2]);
		QVec angulos2 = QVec::vec3(TARGETenMANO[3], TARGETenMANO[4], TARGETenMANO[5]);
		QVec errorRotaciones;
		if(angulos1.norm2() < angulos2.norm2())
			errorRotaciones = angulos1;
		else
			errorRotaciones = angulos2;
				
		// Montamos el ERROR FINAL:
		for(int i=0; i<3; i++)
		{
			errorTotal[i] = errorTraslaciones[i];
			errorTotal[i+3] = errorRotaciones[i];
		}
	}

	//AXIS ALIGN BY INSERTING A VIRTUAL APPEX
	if(target.getType() == Target::ALIGNAXIS)
	{
		QVec errorTraslaciones = QVec::zeros(3);
		QVec auxTraslaciones = QVec::vec3(puntoObjetivo[0], puntoObjetivo[1], puntoObjetivo[2]);
		QVec targetInRoot = inner->transform(this->listaJoints[0], auxTraslaciones ,"world");
		QVec tip = inner->transform(this->listaJoints[0], QVec::zeros(3), "appex");
		errorTraslaciones = targetInRoot - tip;	
		errorTotal.inject(errorTraslaciones,0);
		//Compute rotation error at the original tip, not at the tip of the virtual appex
		if( target.getAxisConstraint() == true )
		{
			QMat matriz = inner->getRotationMatrixTo(listaJoints.last(), "target");  //ESTO NO ESTA DEL TODO BIEN. La restriccón debe ser sobre el axisName directamente
			QVec ang = matriz.extractAnglesR3(matriz);		
			QString axisName = target.getAxisName();
			if(axisName == "x" or axisName == "X")
				errorTotal[3] = ang[0];
			if(axisName == "y" or axisName == "Y")
				errorTotal[4] = ang[1];
			if(axisName == "z" or axisName == "Z")	
				errorTotal[5] = ang[2];
		}
	}
	
	if(target.getType() == Target::ADVANCEALONGAXIS)
	{
		QVec errorTraslaciones = QVec::zeros(3);
		//Get the axis
		QVec axis = target.getPose().subVector(0,2);
		//Compute the target form the axis
		QVec targetInRoot = inner->transform(this->listaJoints[0], axis, this->endEffector);
		QVec tip = inner->transform(this->listaJoints[0], QVec::zeros(3), this->endEffector);
		errorTraslaciones = targetInRoot - tip;	
		errorTotal.inject(errorTraslaciones,0);
	}
	
	//errorTotal.print("errortotal");
	return errorTotal;
}

/*
 * Metodo levenbergMarquardt.
 * Realiza el algoritmo de Levenberg-Marquardt extendido para aquellos casos en
 * los que la matriz que ha de ser invertida salga singular. No para de ejecutar
 * hasta que se alcanza un umbral de aceptacion de la solucion marcado por el
 * usuario. Devuelve los valores de los angulos en orden que han de cambiar.
 * SI FUNCIONA
 * 			- Jt*J + nu*I*Incrementos = Jt*e
 * 				A  + nu*I*Incrementos = g*Ep
 */ 
QVec Cinematica_Inversa::levenbergMarquardt()
{
	//qDebug()<<"\n--ALGORITMO DE LEVENBERG-MARQUARDT --\n";
	//e3 = 10
	const float e1 = 0.0001, e2 = 0.00000001, e3 = 0.0004, e4 = 0.f, t = pow(10, -3);
	const int kMax = 100;
	const QMat Identidad = QMat::identity(this->listaJoints.size());
	
	// VARIABLES:
	int k=0, v=2, auxInt; //iterador, variable para descenso y un entero auxiliar
	
	QVec incrementos, aux; //vector de incrementos y vector auxiliar para guardar cambios
	QVec motores (this->listaJoints.size()); // lista de motores para rellenar el jacobiano.
	QVec angulos = calcularAngulos(); // ángulos iniciales de los motores.
	QVec error = calcularVectorError(); //error de la posición actual con la deseada.
	
	QMat We = QMat::makeDiagonal(this->weights);  //matriz de pesos para compensar milímietros con radianes.
	
	QMat J = jacobian(motores);
	QMat H = J.transpose()*(We*J);
	QVec g = J.transpose()*(We*error);		
	
	bool stop = (g.maxAbs(auxInt) <= e1);
	float ro = 0; 
	float n = t*H.getDiagonal().max(auxInt); 
	
	while(!stop and k<kMax)
	{
		k++;
		do{
			try
			{
				incrementos = (H + (Identidad*n)).invert() * g;
				
				for(int i=0; i<incrementos.size(); i++)
					if(isnan(incrementos[i])) 
					{
						k = kMax;
						break;
					}
			
			}catch(QString str){ qDebug()<<"\n----> ERROR EN LA MATRIZ INVERSA\n"; }
			
			if(incrementos.norm2() <= (e2*(angulos.norm2()+e2)))
			{
				stop = true;
// 				qDebug()<<"Me atasco en iteracion: "<<k;
			}
			else
			{
				aux = angulos-incrementos; 
				calcularModuloFloat(aux, 2*M_PI); // NORMALIZAMOS

				if(dentroLimites(aux, motores) == false)
				{
					//qDebug()<<"FUERA DE LOS LIMITES";
					// Recalculamos el Jacobiano, el Hessiano y el vector g. El error es el mismo que antes
					// puesto que NO aplicamos los cambios (los ángulos nuevos).
					J = jacobian(motores);
					H = J.transpose()*(We*J);
					g = J.transpose()*(We*error);
				}
				
				else
				{
					for(int i=0; i<motores.size(); i++)  //Cambiar por .set((T)0);
						motores[i] = 0;
					
					actualizarAngulos(aux); // Metemos los nuevos angulos LUEGO HAY QUE DESHACER EL CAMBIO.
					ro = ((We*error).norm2() - (We*calcularVectorError()).norm2()) /*/ (incrementos3*(incrementos3*n3 + g3))*/;

					if(ro > 0)
					{
						// Estamos descendiendo correctamente --> errorAntiguo > errorNuevo. 
						stop = ((We*error).norm2() - (We*calcularVectorError()).norm2()) < e4*(We*error).norm2();
						//qDebug()<<"HAY MEJORA ";
						angulos = aux;
						// Recalculamos con nuevos datos.
						error = calcularVectorError();						
						J = jacobian(motores);
						H = J.transpose()*(We*J);
						g = J.transpose()*(We*error);
		
						stop = (stop) or (g.maxAbs(auxInt)<=e1);
						n = n * std::max(1.f/3.f, (float)(1.f-pow(2*ro - 1,3)));		
						v=2;
					}
					else
					{
						//qDebug() << __FILE__ << __FUNCTION__ << __LINE__ << "NO IMPROVEMENT";
						actualizarAngulos(angulos); //volvemos a los ángulos viejos.
						n = n*v;
						v= 2*v;
					}
				}//fin else dentro límites
			}//fin else incrementos no despreciables.
		}while(ro<=0 and stop==false);
		stop = (We*error).norm2() <= e3;
	}
	//qDebug()<<"Error vector: "<<error<<" norm: "<<error.norm2();
	ERROR = ERROR + error.norm2();
	//qDebug()<<"K: "<<k;
		
	return angulos;
}
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*
 * 										MÉTODOS DE CÁLCULO													   *
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
/*
 * Metodo calcularAngulos
 * Devuelve en un vector todos los angulos de los motores del robot.
 * FUNCIONA
 */ 
QVec Cinematica_Inversa::calcularAngulos()
{
	QVec angulos;
	
	for(int i=0; i<this->listaJoints.size(); i++)
	{
		float angle = inner->getJoint(listaJoints[i])->getAngle();
		angulos.push_back(angle);
	}
	return angulos;
}

/*
* Metodo moduloFloat
* Devuelve el m��dulo entre dos n��meros reales.
* FUNCIONA.
*/ 
void Cinematica_Inversa::calcularModuloFloat(QVec &angles, float mod)
{
	for(int i=0; i<angles.size(); i++)
	{
		int cociente = (int)(angles[i] / mod);
		angles[i] = angles[i] -(cociente*mod);
		
		if(angles[i] > M_PI)
			angles[i] = angles[i]- M_PI;
	}
}


/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*
 * 										MÉTODOS DE ACTUALIZACIÓN											   *
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
/*
 * Metodo actualizarAngulos.
 * Actualiza la lista de motores del brazo con los nuevos angulos
 * que recibe como parametro (un vector) de entrada.
 */ 
void Cinematica_Inversa::actualizarAngulos(QVec angulos_nuevos)
{
	for(int i=0; i<this->listaJoints.size(); i++)
	{
		this->inner->updateJointValue(this->listaJoints[i], angulos_nuevos[i]);
	}
}

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*
 * 										MÉTODOS DE CONSULTA													   *
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
/*
 * Método dentroLimites
 * Devuelve TRUE si los ángulos que se le pasan para mover los motores están correctos y no superan
 * los límites de cada motor, o FALSE si alguno de los ángulos para algún motor ha sobrepasado el
 * límite. Si se supera el límite pone un 1 en el vector de motores, para que cuando calcule la matriz
 * jacobiana, ponga ceros en la columna del motor.
 */ 
bool Cinematica_Inversa::dentroLimites(QVec angulos, QVec &motores)
{
	bool noSupera = true;
	float limiteMin, limiteMax;
	
	for(int i=0; i<listaJoints.size(); i++)
	{
		// Obtenemos los límites mínimo y máximo de cada motor y lo comparamos con el ángulo obtenido
		// por el algoritmo de Levenberg-Marquardt.
		limiteMin = inner->getJoint(listaJoints[i])->min;
		limiteMax = inner->getJoint(listaJoints[i])->max;
		
		if(angulos[i]<limiteMin or angulos[i]>limiteMax)
		{
			noSupera = false;
			motores[i] = 1;
			//qDebug()<<"MIN: "<<limiteMin<<" MAX: "<<limiteMax<<" ANGLE: "<<angulos[i]<<" MOTORES: "<<listaJoints[i];
		}
	}
	
	return noSupera;
}


///  CODE TO ALIGN A BODYPART WITHOUT CREATING A VIRTUAL APPEX (NOT WORKING)

// 	if(target.getType() == Target::ALIGNAXIS)
// 	{
// 		// compute a vector going from tip to target
// 		qDebug() << target.getTipName();
// 		QVec targetInTip = inner->transform(target.getTipName(),QVec::zeros(3),"target").normalize();
// 		//targetInTip.print("targetInTip");
// 		
// 		QString axis = target.getAxisName();
// 		QVec a(3);
// 		if(axis == "x" or axis == "X")
// 			a = QVec::vec3(1,0,0);
// 		else if (axis == "y" or axis == "Y")
// 			a = QVec::vec3(0,1,0);
// 		else if (axis == "z" or axis == "Z")
// 			a = QVec::vec3(0,0,1);
// 		else
// 		{
// 			qDebug() << __FILE__ << __FUNCTION__ << __LINE__ << "Target axis not recognized";
// 			return QVec();
// 		}
// 		QVec o = a^targetInTip; // axis to rotate
// 		float ang = asin(o.norm2());  //Angle to rotate
// 		//o.print("o");
// 		//qDebug()<< "ang " << ang;
// 		QMat c = o.crossProductMatrix();
// 		//c.print("c");
// 		QMat r = QMat::identity(3) + (c * (T)sin(ang)) + (c*c)*(T)(1.f-cos(ang));
// 		//r.print("r");
// 		QVec rotaciones = r.extractAnglesR3(r);
// 		QVec errorRotaciones(3);
// 		//rotaciones.print("rotaciones");
// 		//rotaciones.subVector(0,2).print("sb");
// 		//rotaciones.subVector(3,5).print("sb2");
// 		if(rotaciones.subVector(0,2).norm2() < rotaciones.subVector(3,5).norm2())
// 			errorRotaciones = rotaciones.subVector(0,2);
// 		else
// 			errorRotaciones = rotaciones.subVector(3,5);
// 		//errorRotaciones.print("rotaciones");
// 	
// 		errorTotal[3] = errorRotaciones[0];
// 		errorTotal[4] = errorRotaciones[1];
// 		errorTotal[5] = errorRotaciones[2];
// 	}

