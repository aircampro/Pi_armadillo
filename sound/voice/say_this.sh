#!/bin/bash
# to install the jtalk engine
# sudo apt install -y open-jtalk open-jtalk-mecab-naist-jdic hts-voice-nitech-jp-atr503-m001

# to install another voice e.g. mei chan
# wget https://sourceforge.net/projects/mmdagent/files/MMDAgent_Example/MMDAgent_Example-1.7/MMDAgent_Example-1.7.zip --no-check-certificate
# unzip MMDAgent_Example-1.7.zip
#
tempfile=$(mktemp)
if "$1" == "mai" 
then
   option="-m /usr/share/hts-voice/mei/mei_normal.htsvoice \
   -x /var/lib/mecab/dic/open-jtalk/naist-jdic \
   -ow $tempfile"
   echo "$2" | open_jtalk $option
else   
   option="-m /usr/share/hts-voice/nitech-jp-atr503-m001/nitech_jp_atr503_m001.htsvoice \
   -x /var/lib/mecab/dic/open-jtalk/naist-jdic \
   -ow $tempfile"
   echo "$1" | open_jtalk $option
fi

aplay -q $tempfile
rm $tempfile