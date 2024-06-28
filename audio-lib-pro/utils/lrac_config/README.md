
This tool (to be compiled under Cygwin or Linux) is used to configure an LRAC Device.
It use used to configure:

 - The local BdAddr of the device
 - The LRAC Primary/Secondary and Left/Right configuration.
 - The peer LRAC BdAddr to connect

To configure a device (for example, connected on COM18) as Primary, Left, local BdAddr = 20719b100070,
Peer LRAC BdAddr=20719b100071:<br/>
$./lrac\_config.exe -d COM18 -v 1 -b 3000000 -l 20719b100070 -c PL -p 20719b100071


 The config.sh shows how to use it to configure two boards (e.g. connected to COM18 and COM2):<br>
$./config.sh COM18 COM20 70

The command above 'generates' the following two commands:<br/>
$./lrac\_config.exe -d COM18 -v 1 -b 3000000 -l 20719b100070 -c PL -p 20719b100071<br/>
$./lrac\_config.exe -d COM20 -v 1 -b 3000000 -l 20719b100071 -c SR -p 20719b100070

Note, the last parameter (70 here) is optional. If present, the script uses this value
for the last byte of the BdAddr of the Primary.

This tool can also be used for debug/test (audio insert, PS Switch, etc).
