#!/bin/sh

if [ "$#" -lt 3 ]; then
    echo "Download Voice Prompt File System"
    echo "Usage: $0 port file offset" >&2
    exit 1
fi

echo Download $2 to $1 at offset $3

FILENAME=$2
FILESIZE=$(stat -c%s "$2")
echo File:$FILENAME contains $FILESIZE bytes
VPFS_NVRAM_ID=205
OFFSET=$3
DATA=

PADDING=0

# The NVRAM Data must be 8 bytes (4 bytes for Offset and 4 bytes for Length)

# calculate the length of the offset string
OFFSET_LENGTH=${#OFFSET}
DATA_BE=
# Add 0 as prefix padding
for ((i=1 ; i<=(8-OFFSET_LENGTH) ; i++)); do
    DATA_BE=$DATA_BE$PADDING
done
DATA_BE=$DATA_BE$OFFSET
#echo DATA_BE:$DATA_BE

# Convert the Offset string to Little Endian
DATA_LE=
for ((i=0 ; i<=3 ; i++)); do
    DATA_LE=$DATA_LE${DATA_BE:6-i*2:1}
    DATA_LE=$DATA_LE${DATA_BE:7-i*2:1}
#    echo DATA_LE:$DATA_LE
done
DATA=$DATA$DATA_LE

# calculate the length of the FileSize string (converted in Hex)
FILESIZE_HEX=$( printf "%x" $FILESIZE )
#echo FILESIZE_HEX:$FILESIZE_HEX
FILESIZE_HEX_LENGTH=${#FILESIZE_HEX}
DATA_BE=
# Add 0 as prefix padding
for ((i=1 ; i<=(8-FILESIZE_HEX_LENGTH) ; i++)); do
    DATA_BE=$DATA_BE$PADDING
done
DATA_BE=$DATA_BE$FILESIZE_HEX

# Convert the Length string to Little Endian
DATA_LE=
for ((i=0 ; i<=3 ; i++)); do
    DATA_LE=$DATA_LE${DATA_BE:6-i*2:1}
    DATA_LE=$DATA_LE${DATA_BE:7-i*2:1}
#    echo DATA_LE:$DATA_LE
done
DATA=$DATA$DATA_LE

#echo DATA:$DATA

echo File size: $FILESIZE
echo NVRAM-ID: $VPFS_NVRAM_ID

echo Downloading the Voice Prompt File System
echo ./lrac_config.exe -d $1 -v 1 -b 3000000 --foffset $OFFSET --wbftf $FILENAME
./lrac_config.exe -d $1 -v 1 -b 3000000 --foffset $OFFSET --wbftf $FILENAME

sleep 1

echo Write the Voice Prompt Config entry in NVRAM
echo ./lrac_config.exe -d $1 -v 1 -b 3000000 --nvwrite $VPFS_NVRAM_ID --data $DATA
./lrac_config.exe -d $1 -v 1 -b 3000000 --nvwrite $VPFS_NVRAM_ID --data $DATA

echo
