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


#ifndef TARGET_H
#define TARGET_H

/*
 * CLASE AÑADIDA: TARGET.
 * Sirve para marcar puntos objetivos donde llevar diferentes partes del robot, como el brazo
 * derecho, el brazo izquierdo o la cabeza...
 */ 
#include <innermodel/innermodel.h>
#include <qt4/QtCore/qstring.h>
#include <qt4/QtCore/QTime>
#include <qt4/QtCore/qmap.h>
#include <qt4/QtCore/qpair.h>

using namespace std;


class Target
{
	
public:
	
	Target();
	Target(InnerModel* innerModel, QVec pose, QMap<QString, QPair<QStringList,QString> > bodyParts, QString bodyPart);
	~Target();

	// MÉTODOS GET:
	QVec getPose() { return pose; };
	QQueue <QVec> getSubtargets() { return subtargets; };
	QTime getStartTime() { return start; };
	bool getActivo() { return activo; };
	QString getBodyPart() {return bodyPart;};
	QStringList getMotorList(){ return robotBodyParts.value(bodyPart).first;};
	QString getTip(){ return robotBodyParts.value(bodyPart).second;};
	
	// MÉTODOS SET:
	void setInnerModel (InnerModel *newInner);
	void setPose(QVec newPose);
	void setBodyPart(QString newBodyPart);
	
	void setStartTime (QTime newStart);
	void setActivo (bool newActivo);	
	
private:
	
	// ATRIBUTOS DE LA CLASE
	InnerModel *inner;											//copia del innermodel que usa el specificworker.
	QString bodyPart; 											//Nombre de la parte del robot al que está asociado el target
	QMap<QString,QPair<QStringList,QString> > robotBodyParts;	//Mapa con motores y endEffector del robot.
	QVec pose; 													//vector de 6 elementos, 3 traslaciones y 3 rotaciones: tx, ty, tz, rx, ry, rz
	QQueue <QVec> subtargets; 									//lista de subtargets desde el body part hasta el target original.
	QTime start;												//tiempo en que comenzó a trabajar el robot con el target original.
	bool activo;												//bandera para indicar si el target es válido y el robot debe trabajar con él o no.

	
	// OTROS MÉTODOS
	void trocearTarget();
};


#endif // TARGET_H