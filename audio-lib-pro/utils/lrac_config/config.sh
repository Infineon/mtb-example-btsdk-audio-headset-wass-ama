#!/bin/sh

if [ "$#" -lt 2 ]; then
    echo "Usage: $0 port1 port2 [addr-seed]" >&2
    echo "addr-seed: [10.98]"
    exit 1
fi

BDADDR_BASE="20721b2000"

RAND=0
MIN=10
MAX=98

if [ "$#" -eq 3 ]; then
if [ "$3" -eq "$3" ] 2>/dev/null; then
if [ "$3" -ge 10 ] 2>/dev/null; then
if [ "$3" -le 98 ] 2>/dev/null; then
    RAND=$3
fi # <= 98
fi # >= 10
fi # Integer
else
# Choose a random number between [10..98]
echo "Take Random value"
RAND=$(($RANDOM % 89 + 10 ))
fi

if [ "$RAND" -eq 0 ] 2>/dev/null; then
    echo "ERROR: 3rd, optional, parameter must be an integer [10..98]."
    exit 1
fi

# Build the Primary BdAddr
PRIMARY_PORT=$1
PRIMARY_BDADDR=$BDADDR_BASE"$RAND"
echo PRIMARY_BDADDR:$PRIMARY_BDADDR

# Build the Secondary BdAddr (Primary + 1)
RAND=$((RAND + 1))
SECONDARY_PORT=$2
SECONDARY_BDADDR=$BDADDR_BASE"$RAND"
echo SECONDARY_BDADDR:$SECONDARY_BDADDR

echo Configuring first board as Primary Left
echo ./lrac_config.exe -d $PRIMARY_PORT -v 1 -b 3000000 -l $PRIMARY_BDADDR -c PL -p $SECONDARY_BDADDR
./lrac_config.exe -d $PRIMARY_PORT -v 1 -b 3000000 -l $PRIMARY_BDADDR -c PL -p $SECONDARY_BDADDR

echo

echo Configuring second board as Secondary Right
echo ./lrac_config.exe -d $SECONDARY_PORT -v 1 -b 3000000 -l $SECONDARY_BDADDR -c SR -p $PRIMARY_BDADDR
./lrac_config.exe -d $SECONDARY_PORT -v 1 -b 3000000 -l $SECONDARY_BDADDR -c SR -p $PRIMARY_BDADDR

echo
