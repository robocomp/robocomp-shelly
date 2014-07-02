#!/bin/sh
LANGUAGE="es-es"
#LANGUAGE="en-en"


#echo "1 SoX Sound .flac with 44100"
sox -t alsa default ./out.flac silence -l 1 0.01 0.5% 2 1.0 0.5%
 
#echo "1 SoX Sound .flac with 16000"
sox out.flac -r 16000 -b 16 -c 1 $1.flac 

#echo "2 Submit to Google Voice Recognition $1.flac"
wget -U 'Mozilla/5.0' --post-file $1.flac --header="Content-Type: audio/x-flac; rate=16000" -O - 'http://www.google.com/speech-api/v2/recognize?output=json&lang=es_ES&key=AIzaSyB2kDQAMLo3BOCyzL0A77G4PRJnf22p5es' > $speech.json

#echo "3 SED Extract recognized text" 
cat $speech.json > $1.base

cat $1.base | sed 's/.*transcript":"//' | sed 's/","confidence.*//' > $1.txt
cat $1.base | sed 's/.*confidence"://' | sed 's/}.*//' > $1.conf
#echo "4 Remove Temporary Files"
rm $1.flac

#echo "5 Show Text "
cat $1.txt $1.conf > $1.combined
cat $1.combined


