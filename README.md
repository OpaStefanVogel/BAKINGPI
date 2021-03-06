Das ist mein [bare metal](https://www.raspberrypi.org/forums/viewforum.php?f=72&sid=0828289b3d1e532207d78b72567f63c6) Versuch für meinen ausrangierten [Raspberry Pi 1 Mod. B](https://de.wikipedia.org/wiki/Raspberry_Pi#Raspberry_Pi) .

- Im Verzeichnis `BAKINGPI/ok02/` mit `make` die Datei `kernel.img` erzeugen und diese ins `/boot/`-Verzeichnis vom Raspberry Pi B kopieren.

- eine [serielle Verbindung](https://elinux.org/RPi_Serial_Connection) herstellen.

- Raspberry Pi B starten.

Wenn alles gut geht, sollte eine Simulation von diesem [Forth-Interpreter](http://opastefanvogel.github.io/FORTY-FORTH/) zum laufen kommen, vorerst nur ansatzweise:

`77 88 M* M. <Enter> 3F38 ok`

Bisherige Schritte waren:
1. blinkende LED aus https://www.cl.cam.ac.uk/projects/raspberrypi/tutorials/os/ok02.html
2. serielle Verbindung aus https://github.com/s-matyukevich/raspberry-pi-os/blob/master/docs/lesson02/rpi-os.md
3. daß man kernel.img mit `hexdump` über die serielle Verbindung zum Raspberry Pi B schicken kann
4. den Forth-Interpreter irgendwie zum Laufen bringen
