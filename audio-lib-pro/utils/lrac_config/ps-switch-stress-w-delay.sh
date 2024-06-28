#!/bin/sh

if [ $# -ne 4 ]; then
    echo "Usage:"
    echo "  $0 dev0 dev1 test_time wait_sec"
    echo "  Example:"
    echo "      $0 /dev/ttyS29 /dev/ttyS19 10 20"
    exit 1
fi

FIRST_DEV="$1"
SECOND_DEV="$2"
TEST_TIME=$3
WAIT_SEC=$4

cnt=0
cur_dev=0

while [ $cnt -lt $TEST_TIME ]; do
    switch_dev=$FIRST_DEV
    conn_dev=$SECOND_DEV
    if [ $cur_dev -ne 0 ]; then
        switch_dev=$SECOND_DEV
        conn_dev=$FIRST_DEV
    fi

    echo "($cnt)"

    # Do PS-SWITCH
    echo " - Do PS-SWITCH: $switch_dev"
    ./ps-switch.py -d $switch_dev -w >/dev/null 2>&1

    # Wait and check connection
    Sleep 2
    echo " - Check connection: $conn_dev"
    ./check_conn.py -d $conn_dev >/dev/null 2>&1
    if [ $? -ne 0 ]; then
        echo "Connection Drop"
        exit 1
    fi

    # Wait sniff mode and check connection
    echo " - Wait get into sniff mode"
    Sleep $WAIT_SEC;
    echo " - Check connection: $conn_dev"
    ./check_conn.py -d $conn_dev >/dev/null 2>&1
    if [ $? -ne 0 ]; then
        echo "Connection Drop"
        exit 1
    fi

    # Next one
    let cur_dev=!$cur_dev
    let cnt=cnt+1
done

exit 0
