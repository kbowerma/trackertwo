
## Trackertwo

This is a device tracking implementation of Particle.io 's  [Asset Tracker](https://github.com/spark/AssetTracker) for tracking drone movement.  It uses the physical [Asset Tracker](https://www.particle.io/products/hardware/asset-tracker) hardware when using the particle electron and uses a standalone [Adafruit Ultimate GPS](https://www.adafruit.com/product/746) when run on the Photon.   The goal of this project is to be able to use the same code base for both hardware platforms.  This code is written to run the Particle hardware and can be compiled on a new project at [build.particle.io](build.particle.io).  You do not need to own any hardware to compile this code on the particle ide but you will not be able to run it.



## Usage

The include section in the trackertwo.cpp are setup for local compiling which is looking for a lib folder in the root.
If you are compiling on [Particle the web IDE](build.particle.io)
you should comment out this whole include block and add: AssetTracker, Steaming, and HTTPClient when using the web IDE.  Click on the Bookmark icon in the left panel to add libraries.

## Wish list
1.  The http request.body in line 72 - 76 is repeated in the gpsPublish() function on line 129-134,  break this out into a separate  function
2.  The function create above should take in key value pairs and generate the json payload as a string.  We will frequently be adding more data here so we will need an easy way to build this payload.
3. The current code base updates the location (request.put) every __delayMinutes__ which is fine for the photon but will get expensive for the electron.  The goal would be to store the previous lat/lon every time we call the __gpsPublish()__, compare with the new lat/lon, calcualte the distance.   If the distance is greater than distThreshold then do the update put.


## Photon wiring

Photon pin | GPS Pin |Photon pin | GPS Pin
----| ------- | ----| -------
VIN| - | 3v3 | VIN
GND| - | RST | -
Tx| Rx | VBAT | -
Rx| Tx | GND | -
WKP| - | D7 | -
DAC| - | D6 | -
A5| - | D5 | -
A4| - | D4 | -
A3| - | D3 | -
A2| - | D2 | -
A1| - | D1 | -
A0| - | D0 | -

## Electron wiring

Uses the Asset shield.  Not this shield has the LIS3DH 3 axis accelerometer built in.   This is not built into the GPS itself so  when using the Photon  the accelerometer is not available.

![](http://kbowerma.github.io/images/AssetTrackers.jpg)


### Links
* [parse nmea messages ](http://freenmea.net/decoder)
* [Adafruit GPS Library](https://github.com/adafruit/Adafruit_GPS)
* [Adafruit Resources for Ultimate GPS](https://learn.adafruit.com/adafruit-ultimate-gps)


### Contributors
[kbowerma](https://github.com/kbowerma)
[billsedison](https://www.topcoder.com/members/billsedison/)
