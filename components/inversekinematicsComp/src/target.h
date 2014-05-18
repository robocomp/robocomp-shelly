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
 * derecho, el brazo izquierdo o la cabeza... Necesita varias cosas:
 * 		- Un vector POSE que le indique cuál es su traslación y su rotación.
 * 		- El tip o endEffector al que está asignado.
 * 		- El innerModel parapoder trocear la trayectoria desde el endEffector hasta el target.
 */ 
#include <innermodel/innermodel.h>
#include <qt4/QtCore/qstring.h>
#include <qt4/QtCore/QTime>
#include <qt4/QtCore/qmap.h>
#include <qt4/QtCore/qqueue.h>

using namespace std;

class Target
{
	
public:
	
	Target();
	Target(InnerModel *inner, QVec pose, QString bodyPart);
	~Target();
	
	// MÉTODOS GET:
	QString getTipName() const { return this->tip; }; 	//Devuelve el nombre del TIP.
	QVec getPose() const { return this->pose; }; 				// Devuelve el vector pose del target
	QTime getStartTime() const { return this->start; }; // Devuelve el tiempo del target.
	bool getActivo() const { return this->activo; };		// Devuelve el estado del target
	
	// MÉTODOS SET:
	void setInnerModel (InnerModel *newInner);
	void setPose(QVec newPose);
	void setStartTime (QTime newStart);
	void setActivo (bool newActivo);	
	
	// OTROS MÉTODOS
	void trocearTarget();
	
private:
	
	// ATRIBUTOS DE LA CLASE
	QString tip; 							// Nombre del efector final al que está asociado el target
	QTime start;							// Tiempo en que comenzó a trabajar el robot con el target original.
	bool activo;							// Bandera para indicar si el target es válido y el robot debe trabajar con él o no.
	QVec pose; 								// Vector de 6 elementos, 3 traslaciones y 3 rotaciones: tx, ty, tz, rx, ry, rz
	QQueue<QVec> subtargets;	// Cola de subtargets (trayectorias troceadas) para cada target.
	InnerModel *inner;				// Innermodel para calcular cosas.
	
};


#endif // TARGET_H