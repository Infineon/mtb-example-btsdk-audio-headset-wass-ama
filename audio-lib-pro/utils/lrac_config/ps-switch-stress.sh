#!/bin/sh

if [ "$#" -lt 1 ]; then
    echo "Usage: $0 port [loop] [prevent_glitch]" >&2
    exit 1
fi

echo Port:$1

if [ "$#" -ge 2 ]; then
    loop=$2
else
    loop=100
fi

if [ "$#" -ge 3 ]; then
    prevent_glitch=$3
else
    prevent_glitch=0
fi

echo "loop:$loop prevent_glitch:$prevent_glitch"


i=0;
pass=0;
fail=0;
reject=0;
ready_too_long=0;
force_abort=0;

while [ $i -lt $loop ]; do
    echo;
    echo --------------- ;
    echo Iteration $i;
    if [ "$prevent_glitch" != 0 ]; then
        ./ps-switch.py -d $1 -w -p
    else
        ./ps-switch.py -d $1 -w
    fi
    ret=$?
    if [ $ret -eq 0 ]; then
        let pass=pass+1;
    elif [ $ret -eq 1 ]; then
        let reject=reject+1;
    elif [ $ret -eq 2 ]; then
        let ready_too_long=ready_too_long+1;
    elif [ $ret -eq 3 ]; then
        let force_abort=force_abort+1;
    else
        let fail=fail+1;
    fi
    sleep 2;
    let i=i+1;
    echo --------------- ;
done

echo
echo
echo ">> total: $loop, pass: $pass, fail: $fail"

if [ $reject -ne 0 -o $ready_too_long -ne 0 ]; then
    echo "(reject: $reject, ready_too_long: $ready_too_long, force_abort: $force_abort)"
fi
