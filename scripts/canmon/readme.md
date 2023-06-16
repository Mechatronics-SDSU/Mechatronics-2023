## CanMon - SDSU Mechatronics CAN tool

CanMon is a terminal-based tool for formatting and displaying communication between the Embedded Node and NVIDIA Orin computer. It also allows for simple frame injection and automation via. shell scripting. It is designed to be a low-overhead solution to monitoring the submarine without sacrificing ease-of-use.

### Configuration

CanMon is only one file. This means that configuration requires the developer to edit the source code. Given that,I have tried my best to format it in an easy to understand manner. Please read the comments at the top of the file, all values that you would need to change (without directly altering core program functionality) are found in the CONFIG section (top of file, below header inclusions).

### Bulding

Building CanMon is simple. It requires no fancy libraries, and is only one file long. To build, simply:
```
gcc canmon.c -o canmon
```

### Troubleshooting

One thing that CanMon lacks is proper error handling. That being said, the big ones are covered. If there is a problem with connecting to CAN, it will tell you. If this occurs, make sure that you are configured to use the correct bus (see bottom right/middle of TUI). Similarly, if CanMon cannot find the scripts directory, it will fail to start. To remedy this, either configure the program to use a different scripts directory, or create `.canmon` in your home (`~/`) directory. All shell scripts present in this directory will be accessable to CanMon (don't even ask about the security concerns, the Orin isn't supposed to have WIFI underwater anyway!). Other possible solutions to various problems include:
 - make sure you are running CanMon on the Orin, it requires access to the CAN bus to work.
 - is the TUI distorted? try increasing terminal size (or zooming out) and restarting the program.
 - Something else not working? leave a note in the software channel and someone will probably look into it.
