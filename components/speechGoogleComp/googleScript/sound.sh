#!/bin/sh
#LANGUAGE="es-es"
#LANGUAGE="en-en"




echo "1 Submit to Google Voice Generation $text"

wget -q -U Mozilla -O $audio.mp3 "http://translate.google.com/translate_tts?ie=UTF-8&tl=es-CL&q=Marco+se+come+el+marron+del+Rockin"

#wget -q -U Mozilla -O $audio.mp3 "http://translate.google.com/translate_tts?ie=UTF-8&tl=en-US&q=I+need+you+help+Marco"

echo "2 Mplayer  text" 
mplayer $audio.mp3

echo "3 Remove Temporary Files"
rm $audio.mp3







