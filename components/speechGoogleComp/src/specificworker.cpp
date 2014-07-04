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

SpecificWorker::SpecificWorker(MapPrx& mprx, QObject *parent) : GenericWorker(mprx, parent)	
{

}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{

}

void SpecificWorker::compute( )
{

  waitForRecognition();
  
}


void SpecificWorker::waitForRecognition()
{
	QStringList listaArgumentos;
	listaArgumentos << "";

	QString nombrePrograma = "sh googleScript/record.sh";
	QString pathPrograma = "/googleScript/";
	
	QProcess processASR;	
	processASR.setWorkingDirectory(pathPrograma);
	processASR.execute(nombrePrograma);
	
	processASR.waitForFinished();
	
	QString output(processASR.readAllStandardOutput());

	std::string outString = output.toStdString();

	std::stringstream ss(outString);
	std::string line;
	std::string phraseRecognized;
	float confidenceValue = 0.0;
	bool firstLine = true;
	while(std::getline(ss, line))
	{
	   if( line != "")
	   {
			 if(firstLine)
			 {
				 phraseRecognized = line;
				 std::cout << "newSentence: " << phraseRecognized;

			 }
			 else
			 {
				 istringstream(line) >> confidenceValue;
				 std::cout << " with confidence " <<  line << endl;

			 }

			 firstLine = false;
	   }
	}

	std::cout << "newSentence: " << phraseRecognized << " with confidence " <<  confidenceValue << endl;

}


bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	timer.start(Period);
	/**
	 * @brief PUblish method
	 * 
	 * @param text ...
	 * @return void
	 */
	return true;
};


bool SpecificWorker::isBusy()
{
	return false;

}
bool SpecificWorker::say(const string& text, bool owerwrite)
{ 
  
  QString TextTTS = QString::fromStdString(text);
  qDebug()<< "TEXT"<<TextTTS<<"def";  

  QString audioPath = "googleScript/audio.mp3";
  QString SpeechCommand = "wget -q -U Mozilla -O " + audioPath + " \"http://translate.google.com/translate_tts?ie=ISO-8859-1&tl=es-ES&q=" + TextTTS + "\"";
  QString MplayerCommand = "mplayer -really-quiet " + audioPath;
  
  	QProcess processSpeech;	
	processSpeech.execute(SpeechCommand);
	
  	QProcess processMplayer;	
	processMplayer.execute(MplayerCommand);
 
	return true;
}
