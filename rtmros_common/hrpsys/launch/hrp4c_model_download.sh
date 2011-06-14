#!/bin/bash

HRP4C_DOWNLOAD_FILE=$HOME/Downloads/HRP-4C.zip
HRP4C_MODEL_DIR=`rospack find hrpsys`/share/hrpsys/samples/
HRP4C_MODEL_PATH=${HRP4C_MODEL_DIR}/HRP-4C/HRP4Cmain.wrl

if [ ! -f ${HRP4C_MODEL_PATH} ]; then
    if [ -f ${HRP4C_DOWNLOAD_FILE} ]; then
	unzip ${HRP4C_DOWNLOAD_FILE} -d ${HRP4C_MODEL_DIR}
    else
	echo "======================================================"
	echo "======================================================"
	echo "======================================================"
	echo "Download HRP4 model and extract to ${HRP4C_MODEL_PATH}"
	echo "======================================================"
	echo "======================================================"
	echo "======================================================"
	firefox http://unit.aist.go.jp/is/humanoid/hrp-4c/hrp-4c.html
	firefox http://unit.aist.go.jp/is/humanoid/hrp-4c/agreement.html
	exit -1
    fi
fi