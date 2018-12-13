#!/usr/bin/env bash

ROS_WORKSPACE=`dirname $(dirname $(dirname $(realpath $_)))`
if [[ ! -f ${ROS_WORKSPACE}/devel/setup.sh ]]; then
    ROS_WORKSPACE=`echo ${ROS_PACKAGE_PATH} | sed -r "s,.*($HOME[^:]*)/src.*,\1,"`
    if [[ -z ${ROS_WORKSPACE} ]]; then
        echo "No ROS workspace found. Is it sourced?"
        exit 1
    fi
fi

if [[ -z ${ROS_ROOT} ]]; then
    echo "No ROS installation found. Is it sourced?"
    exit 1
fi

ROS_INSTALLATION=`echo ${ROS_ROOT} | sed "s/\/share\/ros//"`

echo "ROS Installation: ${ROS_INSTALLATION}"
echo "ROS Workspace:    ${ROS_WORKSPACE}"

LIB_DIRS=( ${ROS_WORKSPACE}/devel/lib/python2.7/dist-packages ${ROS_WORKSPACE}/devel/lib/python3/dist-packages )

INIT_FILES=`find -L ${ROS_WORKSPACE}/src -name '*.pyx' | sed -r 's/([a-zA-Z_\-]*\.pyx)$/__init__.py/'`
PARAM_CONFIGS=`find -L ${ROS_WORKSPACE}/src -name '*_paramsConfig.py*'`

for DIR in ${LIB_DIRS[@]}; do
    echo "Touching files for `basename $(dirname ${DIR})`"
    for FILE in ${INIT_FILES}; do
        NEW_FILE=`echo ${FILE} | sed -r "s,^.*?/src/,${DIR}/,g"`
        if [[ -d `dirname ${NEW_FILE}` && ! -f ${NEW_FILE} ]]; then
            touch "${NEW_FILE}"
            echo "  Touched ${NEW_FILE}"
        fi
    done

    echo "Copying config files for `basename $(dirname ${DIR})`"
    for CONFIG in ${PARAM_CONFIGS}; do
        NEW_CONFIG=`echo ${CONFIG} | sed -r "s,^.*?/src/,${DIR}/,"`
        if [[ -d `dirname ${NEW_CONFIG}` && ! -f ${NEW_CONFIG} ]]; then
            cp "${CONFIG}" "${NEW_CONFIG}"
            echo "  Copied ${CONFIG}"
        fi
    done
done

SETUP_FILE=${ROS_WORKSPACE}/devel/setup.sh
if [[ -L ${SETUP_FILE} ]]; then
    SETUP_FILE=`readlink ${SETUP_FILE}`
fi

PYTHON3_LOCAL="export PYTHONPATH=\$PYTHONPATH:${ROS_WORKSPACE}/devel/lib/python3/dist-packages"
if [[ `tail -1 ${SETUP_FILE}` != ${PYTHON3_LOCAL} ]]; then
    echo ${PYTHON3_LOCAL} >> ${SETUP_FILE}
    echo "Please source ROS again!"
fi

