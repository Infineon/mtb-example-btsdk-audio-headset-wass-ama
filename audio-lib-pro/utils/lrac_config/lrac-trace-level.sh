#!/bin/sh

if [ "$#" -lt 3 ]; then
    echo "Usage: $0 port1 port2 level" >&2
    exit 1
fi

echo Set LRAC Trace Level for the first board
echo ./lrac_config.exe -d $1 -v 1 -b 3000000 --lrac_trace $3
./lrac_config.exe -d $1 -v 1 -b 3000000 --lrac_trace $3

echo

echo Set LRAC Trace Level for the second board
echo ./lrac_config.exe -d $2 -v 1 -b 3000000 --lrac_trace $3
./lrac_config.exe -d $2 -v 1 -b 3000000 --lrac_trace $3

echo
