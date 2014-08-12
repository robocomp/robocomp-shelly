/**
 * @brief CLASE AÑADIDA:  TRAYECTORIA
 * Se encarga de crear las trayectorias de poses6D ofrecidas por la interfaz.
 */

#ifndef TRAYECTORIA_H
#define TRAYECTORIA_H

#include <qt4/Qt/qqueue.h>
#include <qt4/Qt/qlist.h>
#include <qt4/Qt/qstring.h>
#include <innermodel/innermodel.h>

class Trayectoria
{
public:
	//// ATRIBUTOS PÚBLICOS ////
	QVec 			centroEsfera;
	
	// Constructores y destructores:
					Trayectoria			();	
					~Trayectoria		();
	
	//// MÉTODOS PÚBLICOS ////
	QQueue<QVec> 	crearTrayectoria	(int tipo, QVec rot);		// Método principal: llama a los métodos privados para crear una trayectoria.
	
private:
	//// ATRIBUTOS PRIVADOS ////
	QQueue<QVec>	trayectoria;
	QVec 			rotaciones;
	
	//// MÉTODOS PRIVADOS ////
	void 			camareroCentro		();			// Trayectoria cuadrada para ambos brazos 
	void 			camareroDiestro		();			// Trayectoria cuadrada para el brazo derecho
	void			camareroZurdo		();			// Trayectoria cuadrada para el brazo izquierdo
	void 			puntosEsfera		();			// Trayectoria cuadrada para ambos brazos 
	void 			puntosCubo			();			// Trayectoria cuadrada para ambos brazos

};


#endif
