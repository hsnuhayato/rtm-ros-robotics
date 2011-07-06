#!/bin/bash

function start {
    term=xterm

    $term -e python ArmPlanComp.py &
    $term -e python TaskSchedComp.py &

    pushd externals/GraspPlan
    $term -e ./GraspPlannerComp &
    popd

    # pushd ../GraspConsumerSample
    # $term -e ./${GC}Comp &
    # popd
}

function connect_grasp {
    NSLOC=/telesto:9876/
    rtcwd $NSLOC

    LOC1=telesto.host_cxt
    LOC2=ubuntu.host_cxt

    rtcon $LOC1/ArmPlan0.rtc:ServicePort $LOC1/TaskSchedComp0.rtc:ArmPlanServicePort
    rtact $LOC1/ArmPlan0.rtc
    rtact $LOC1/TaskSchedComp0.rtc

    rtcon $LOC2/GraspPlanner0.rtc:PlannStartPort $LOC1/TaskSchedComp0.rtc:PlannStartPort
    rtcon $LOC2/GraspPlanner0.rtc:ResultPort $LOC1/TaskSchedComp0.rtc:ResultPort
}

function connect {
    NSLOC=/hiro014:2809
    rtcon $NSLOC/RobotHardware0.rtc:jointStt $NSLOC/JointStatePublisher0.rtc:in
    rtact $NSLOC/JointStatePublisher0.rtc $NSLOC/leye_capture.rtc $NSLOC/rhand_capture.rtc
}

function kill {
    for pid in `pgrep -f ${GP}Comp`; do
	kill -9 $pid
    done
    for pid in `pgrep -f ${GC}Comp`; do
	kill -9 $pid
    done
}

function setup {
    start
    sleep 3
    connect
}

$1
