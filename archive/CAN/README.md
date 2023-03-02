## create file enable_CAN.sh
```
touch /enable_CAN.sh
chmod 755 /enable_CAN.sh
```
## check if /etc/rc.local exists else
```
printf '%s\n' '#!/bin/bash' 'exit 0' | sudo tee -a /etc/rc.local
sudo chmod +x /etc/rc.local
```
## add line to rc.local
```
sh /enable_CAN.sh &
```
