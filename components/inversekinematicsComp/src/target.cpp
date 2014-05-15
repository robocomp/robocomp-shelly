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

#include "target.h"

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*
 * 								CONSTRUCTORES Y DESTRUCTORES												   *
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/ 
/**
 * \brief Constructor por defecto
 */
Target::Target()
{
	this->activo = false;
}

/**
 * \brief Constructor parametrizado
 * Inicializa las estructuras que componen sus atributos de clase. 
 * Le pasamos: los límites de las traslaciones en los que queremos poner el target.
 * 			   las coordenadas del effector final TODO pasarle la parte del cuerpo a la que pertenece el target.
 * 			   la distancia máxima del target al tip por si debe trocear.
 */ 
Target::Target(InnerModel* innerModel, QVec pose, QMap<QString,QPair<QStringList,QString> > bodyParts, QString bodyPart)
{
	this->activo = true;
	this->inner = innerModel;
	this->pose = pose;
	this->bodyPart = bodyPart;
	this->robotBodyParts = bodyParts;
	
	trocearTarget();
}

Target::~Target()
{

}

/*--------------------------------------------------------------------------*
 *						MÉTODOS PÚBLICOS									*
 *--------------------------------------------------------------------------*/
void Target::setInnerModel(InnerModel *newInner)
{
	this->inner = newInner;
}

void Target::setPose(QVec newPose)
{
	this->pose = newPose;
}

void Target::setBodyPart(QString newBodyPart)
{
	this->bodyPart = newBodyPart;
}

void Target::setActivo(bool newActivo)
{
	this->activo = newActivo;
}

void Target::setStartTime(QTime newStart)
{
	this->start = newStart;
}


/*--------------------------------------------------------------------------*
 *						MÉTODOS PRIVADOS									*
 *--------------------------------------------------------------------------*/
/*
 * Método TROCEAR TARGET
 * Crea una recta entre el tip y el target colocando subtargets cada distanciaMax
 * permitida. Troceamos con la ecuación  R= (1-Landa)*P + Landa*Q
 */ 
void Target::trocearTarget()
{
	// Sacamos el tip:
	QString endEffector = robotBodyParts.value(bodyPart).second; //DA PROBLEMAS AL LEER
	
	//TRASLACIÓN: 
	QVec traslacionPose = QVec::vec3(pose[0], pose[1], pose[2]);
	QVec traslaciones = (traslacionPose - inner->transform("world", QVec::zeros(3), endEffector));
	
	//ROTACIÓN:
	// Hay que calcular las rotaciones de otra forma. No tenemos actualizado el innerModel para saber cúanto
	// debe girar el endEffector para alcanzar la rotación del target.
	// Sacamos las rotaciones del endEffector y restamos rotaciones. Si son iguales la resta da 0.
	QMat matriz = inner->getRotationMatrixTo("world", endEffector);
	QVec endEffectorEnMundo = inner->getTransformationMatrix("world", endEffector).extractAnglesR3(matriz);
	QVec angulos1 = QVec::vec3(endEffectorEnMundo[0], endEffectorEnMundo[1], endEffectorEnMundo[2]);
	QVec angulos2 = QVec::vec3(endEffectorEnMundo[3], endEffectorEnMundo[4], endEffectorEnMundo[5]);
	QVec rot;
	if(angulos1.norm2() < angulos2.norm2())
		rot = angulos1;
	else
		rot = angulos2;
	
	QVec rotacionPose = QVec::vec3(pose[3], pose[4], pose[5]); 
	QVec rotaciones = rotacionPose - rot;
		
	QVec ambos(6);
	for(int i=0; i<3; i++)
	{
		ambos[i] = traslaciones[i];
		ambos[i+3] = rotaciones[i];
	}
	
	float dist = ambos.norm2();
	// Si la distancia es mayor que 1cm troceamos la trayectoria:
	if(dist >0.01)
	{
		float Npuntos = dist / 0.01;
		
		for(float i=0; i<Npuntos; i++)
		{
			float landa = 1/(Npuntos) * i;
			
			QVec R = inner->transform("world", QVec::zeros(3), endEffector)*(1-landa) + pose*landa;
			subtargets.append(R);
		}
	}
	// R(6) = P*(1-landa) + Q*landa
	// P = tip
	// Q = target
}





