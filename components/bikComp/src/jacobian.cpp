/*
 * Copyright 2015 <copyright holder> <email>
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 * 
 */

#include "jacobian.h"
/*
Jacobian::Jacobian()
{
}

Jacobian::~Jacobian()
{
}

static QMat Jacobian::jac(const QVec &motores)
{
	// La lista de motores define una secuencia contigua de joints, desde la base hasta el extremo.
	// Inicializamos las filas del Jacobiano al tamaño del punto objetivo que tiene 6 ELEMENTOS [tx, ty, tz, rx, ry, rz]
	// y las columnas al número de motores (Joints): 6 filas por n columnas. También inicializamos un vector de ceros
	
	QMat jacob(6, this->listaJoints.size(), 0.f);  //6 output variables
 	QVec zero = QVec::zeros(3);
 	int j=0; //índice de columnas de la matriz: MOTORES
	
 	foreach(QString linkName, this->listaJoints)
 	{
		if(motores[j] == 0)
		{
			QString frameBase = this->listaJoints.last();
						
			// TRASLACIONES: con respecto al último NO traslada
			QVec axisTip = this->inner->getJoint(linkName)->unitaryAxis(); 		//vector de ejes unitarios
			axisTip = this->inner->transform(frameBase, axisTip, linkName);
			QVec axisBase = this->inner->transform(frameBase, zero, linkName);
			QVec axis = axisBase - axisTip;
			QVec toEffector = (axisBase - this->inner->transform(frameBase, zero, this->endEffector) );		
			QVec res = toEffector.crossProduct(axis);

			jacob(0,j) = res.x();
			jacob(1,j) = res.y();
			jacob(2,j) = res.z();
			
			// ROTACIONES
			QVec axisTip2 = this->inner->getJoint(linkName)->unitaryAxis(); 		//vector de ejes unitarios en el que gira
			axisTip2 = this->inner->transform(frameBase, axisTip2, linkName); 		//vector de giro pasado al hombro.
			QVec axisBase2 = this->inner->transform(frameBase, zero, linkName); 	//motor al hombro
			QVec axis2 = axisBase2 - axisTip2; 										//vector desde el eje de giro en el sist. hombro, hasta la punta del eje de giro en el sist. hombro. 
			
			jacob(3,j) = axis2.x(); 
			jacob(4,j) = axis2.y();
			jacob(5,j) = axis2.z();
				
		}
 		j++;
 	}
 	return jacob;
}*/
