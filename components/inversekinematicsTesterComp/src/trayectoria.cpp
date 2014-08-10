
#include "trayectoria.h"

//**********************************************************//
//				CONSTRUCTORES Y DESTRUCTORES				//
//**********************************************************//

/**
 * @brief Constructor por defecto.
 * Esta clase no necesita constructores parametrizados.
 */ 
Trayectoria::Trayectoria()
{
	qDebug()<<"Inicializada la clase auxiliar TRAYECTORIA";
	this->rotaciones = QVec::vec3(0,0,0);
}

/**
 * @brief Destructor por defecto.
 * Esta clase no necesita limpiar ninguna estructura. 
 */ 
Trayectoria::~Trayectoria()
{
	qDebug()<<"Destruyendo la clase TRAYECTORIA";
}



//**********************************************************//
//						MÉTODOS PÚBLICOS					//
//**********************************************************//
/**
 * @brief Método público: CREAR TRAYECTORIA
 * Recibe como parámetro de entrada un entero que indica el tipo de trayectoria a generar
 * y que devolverá como parámetro de salida así como las rotaciones que debe tener cada
 * pose o target que forman la trayectoria.
 * 
 * @param tipo entero que indica qué trayectoria se debe crear
 * @param rot vector con las rotaciones que aparecen en la interfaz gráfica de usuario.
 * 
 * @return cola de targets o trayectoria.
 */ 
QQueue< QVec > Trayectoria::crearTrayectoria(int tipo, QVec rot)
{
	this->rotaciones = rot;
	
	switch(tipo)
	{
		case 1:		camareroCentro	();		break;
			
		case 2:		camareroDiestro	();		break;
			
		case 3:		camareroZurdo	();		break;
			
		case 4:		puntosCubo		();		break;
			
		case 5:		puntosEsfera	();		break;
			
		default:	qDebug()<<"ESTA TRAYECTORIA NO EXISTE: "<<tipo;		break;
	}
	return this->trayectoria;
}



//**********************************************************//
//						MÉTODOS PRIVADOS					//
//**********************************************************//
/**
 * @brief Metodo CAMARERO CENTRO. 
 * Crea una trayectoria de poses para un camarero con una bandeja moviendola por el 
 * centro del pecho. Guarda esa trayectoria en el atributo de la clase, trayectoria
 * Limpia nada más empezar la trayectoria para que no se acumulen las poses.
 * -------------------------> CREA LA TRAYECTORIA EN MILIMETROS <------------------
 * 
 * @return void
 */
void Trayectoria::camareroCentro()
{
	trayectoria.clear(); //Limpiamos trayectoria para que NO SE ACUMULEN.

	QVec pose = QVec::zeros(6);
	
	//Fijamos las mismas rotaciones
	pose[3] = this->rotaciones[0];	pose[4] = this->rotaciones[1]; 	pose[5] = this->rotaciones[2];
	
	pose[0] = -150; 				pose[1] = 900; 					pose[2] = 300;		trayectoria.append(pose);
	pose[0] = 150; 					pose[1] = 900; 					pose[2] = 300;		trayectoria.append(pose);
	pose[0] = 150; 					pose[1] = 1100; 				pose[2] = 300;		trayectoria.append(pose);
	pose[0] = -150; 				pose[1] = 1100; 				pose[2] = 300;		trayectoria.append(pose);
	pose[0] = -150; 				pose[1] = 900; 					pose[2] = 300;		trayectoria.append(pose);
}

/**
 * @brief Metodo CAMARERO DIESTRO. 
 * Crea una trayectoria de poses para un camarero con una bandeja en la mano derecha. Guarda la 
 * trayectoria en el atributo de la clase, trayectoria. Debemos limpiar la trayectoria siempre
 * para que no se acumulen las poses 
 * -------------------------> CREA LA TRAYECTORIA EN MILIMETROS <--------------------------------
 * 
 * @return void
 */ 
void Trayectoria::camareroDiestro()
{
	trayectoria.clear();
	
	float salto = 10, xAux, yAux;
    QVec pose = QVec::zeros(6);
	
	//Fijamos las mismas rotaciones
	pose[3] = this->rotaciones[0];	pose[4] = this->rotaciones[1]; 	pose[5] = this->rotaciones[2];
	
	// Trasladamos en X hacia la izquierda: 
	for(float i=-100; i<300; i=i+salto)
	{
		pose[0] = i; 				pose[1] = 900; 					pose[2] = 350;
		trayectoria.append(pose);
		xAux = i;
	}
	// Subimos en Y:
	for(float j=900; j<1100; j=j+salto)
	{
		pose[0] = xAux; 			pose[1] = j; 					pose[2] = 350;
		trayectoria.append(pose);
		yAux = j;
	}
	// Trasladamos en X hacia la derecha:
	for(float i=xAux; i>=-100; i=i-salto)
	{
		pose[0] = i; 				pose[1] = yAux; 				pose[2] = 350;
		trayectoria.append(pose);
		xAux = i;
	}
	// Bajamos en Y:
	for(float j=yAux; j>=900; j=j-salto)
	{
		pose[0] = xAux; 			pose[1] = j; 					pose[2] = 350;
		trayectoria.append(pose);
		yAux = j;
	}
}

/**
 * @brief Metodo CAMARERO ZURDO. 
 * Crea una trayectoria de poses para un camarero con una bandeja en la mano izquierda. Guarda 
 * la trayectoria en el atributo de la clase. Debemos limpiar la trayectoria siempre para que 
 * no se acumulen las poses 
 * -------------------------> CREA LA TRAYECTORIA EN MILIMETROS <--------------------------------
 * 
 * @return void
 */
void Trayectoria::camareroZurdo()
{
	trayectoria.clear(); //Limpiamos trayectoria para que NO SE ACUMULEN.
	
	float salto = 10, xAux, yAux;
	QVec pose = QVec::zeros(6);
	
	//Fijamos las mismas rotaciones
	pose[3] = this->rotaciones[0];	pose[4] = this->rotaciones[1]; 	pose[5] = this->rotaciones[2];

	 // Trasladamos hacia la derecha en X:
	for(float i=-150; i<=150; i=i+salto)
	{
		pose[0] = i; 				pose[1] = 900; 				pose[2] = 350;
		trayectoria.enqueue(pose);
		xAux = i;
	}
	// Subimos en Y:
	for(float j=900; j<1100; j=j+salto)
	{
		pose[0] = xAux;				pose[1] = j; 				pose[2] = 350;
		trayectoria.enqueue(pose);
		yAux = j;
	}
	// Trasladamos hacia la izquierda en X:
	for(float i=xAux; i>=-150; i=i-salto)
	{
		pose[0] = i; 				pose[1] = yAux; 			pose[2] = 350;
		trayectoria.enqueue(pose);
		xAux=i;
	}
	// Bajamos en Y:
	for(float j=yAux; j>=900; j=j-salto)
	{
		pose[0] = xAux; 			pose[1] = j; 				pose[2] = 350;
		trayectoria.enqueue(pose);
		yAux = j;
	}
}

/**
 * @brief Metodo PUNTOS CUB0. 
 * Crea una lista de poses colocadas dentro de un cubo 3D.
 * -------------------------> CREA LA TRAYECTORIA EN MILIMETROS <--------------------------------
 * 
 * @return void
 */
void Trayectoria::puntosCubo()
{

}

void Trayectoria::puntosEsfera()
{

}
