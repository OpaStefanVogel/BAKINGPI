Das ist ein [BareMetal](https://www.raspberrypi.org/forums/viewforum.php?f=72&sid=0828289b3d1e532207d78b72567f63c6) Versuch für meinen ausrangierten [Raspberry Pi 1 Mod. B](https://de.wikipedia.org/wiki/Raspberry_Pi#Raspberry_Pi) .

- `BAKINGPI/ok02/kernel.img` ins Verzeichnis `/boot/` vom Raspberry Pi kopieren.

- eine [serielle Verbindung](https://elinux.org/RPi_Serial_Connection) herstellen.

- Raspberry Pi B starten.

Wenn alles gut geht, sollte eine Simulation von diesem [Forth-Interpreter](http://opastefanvogel.github.io/FORTY-FORTH/) zum laufen kommen, vorerst nur ansatzweise:

`77 88 M* M. <Enter> 3F38 ok`

