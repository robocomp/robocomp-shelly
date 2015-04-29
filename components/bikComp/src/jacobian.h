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

#ifndef JACOBIAN_H
#define JACOBIAN_H

#include <qmat/QMatAll>

	/**
 * @brief Metodo jacobian.
 * Crea la matriz jacobiana de la función de cinematica directa con los datos que saca del innerModel. 
 * La va rellenando por columnas, siendo las columnas los motores y las filas la tx, ty, tz, rx, ry y rz.
 * Se le pasa un vector de tantos elementos como motores con los que trabaja. Si hay cero en la posición
 * del motor calcula traslaciones y rotaciones. Si hay un 1 en la posición del motor rellena la columna a 0
 * 
 * @param motores es un vector de 0 y 1, con tantos elementos como motores con los que esté trabajando la 
 * 		  inversa. El 0 significa que el motor NO está bloqueado (calcula la columna del jacobiano asociado
 * 		  al motor) y el 1 significa que el motor SI está bloqueado, rellenado la columna del jacobiano con 0.
 * 
 * @return QMat la matriz jacobiana
 */ 

class Jacobian
{
	public:
		Jacobian(){};
		~Jacobian(){};
		static QMat jac(InnerModel *inner,  QStringList &listaJoints, const QVec &motores, const QString &endEffector)
		{
			// La lista de motores define una secuencia contigua de joints, desde la base hasta el extremo.
			// Inicializamos las filas del Jacobiano al tamaño del punto objetivo que tiene 6 ELEMENTOS [tx, ty, tz, rx, ry, rz]
			// y las columnas al número de motores (Joints): 6 filas por n columnas. También inicializamos un vector de ceros
			
			QMat jacob(6, listaJoints.size(), 0.f);  //6 output variables
			QVec zero = QVec::zeros(3);
			int j=0; //índice de columnas de la matriz: MOTORES
			
			foreach(QString linkName, listaJoints)
			{
				if(motores[j] == 0)
				{
					QString frameBase = listaJoints.last();
								
					// TRASLACIONES: con respecto al último NO traslada
					QVec axisTip = inner->getJoint(linkName)->unitaryAxis(); 		//vector de ejes unitarios
					axisTip = inner->transform(frameBase, axisTip, linkName);
					QVec axisBase = inner->transform(frameBase, zero, linkName);
					QVec axis = axisBase - axisTip;
					QVec toEffector = (axisBase - inner->transform(frameBase, zero, endEffector) );		
					QVec res = toEffector.crossProduct(axis);

					jacob(0,j) = res.x();
					jacob(1,j) = res.y();
					jacob(2,j) = res.z();
					
					// ROTACIONES
					QVec axisTip2 = inner->getJoint(linkName)->unitaryAxis(); 		//vector de ejes unitarios en el que gira
					axisTip2 = inner->transform(frameBase, axisTip2, linkName); 		//vector de giro pasado al hombro.
					QVec axisBase2 = inner->transform(frameBase, zero, linkName); 	//motor al hombro
					QVec axis2 = axisBase2 - axisTip2; 				//vector desde el eje de giro en el sist. hombro, hasta la punta del eje de giro en el sist. hombro. 
					
					jacob(3,j) = axis2.x(); 
					jacob(4,j) = axis2.y();
					jacob(5,j) = axis2.z();
				}
				j++;
			}
			return jacob;
		}
};

#endif // JACOBIAN_H
