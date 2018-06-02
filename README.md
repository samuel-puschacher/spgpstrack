# spGPStrack - LoRaWAN GPS Mapping

This program is a LoRaWAN GPS Tracker for mapping purposes
It is developed for Arduino Uno and Dragino Lora/GPS Shield
  - configured as ABP so that you can power up the device at locations where you do not have coverage
  - will transmit the location all x meters
  - you can choose the payload format between CayenneLPP and a manual Payload Decoder Function
  - the software supports confirmed uplink and the Arduino will beep
  - You can read the GPS over Hardware Serial or Software Serial, so it's easier for development if you see what is going on during real operation
  - In case of Hardware Serial usage you can enable a Software Serial for debugging
 ## To Do
  - Timed transmission, which also executes when the tracker is not moving. For example every hour, to see if the tracker is still alive.
  - low power option
  - ESP32 support [State: in work, see branch develop. The SPI Config produces backtraces] Hardware: ESP32 Uno, branded as Wemos

 ## Quick Start
  - In the TTN Console, create an Application, register a new device (as ABP)
  - Copy the LoRaWAN Device Information and Keys into the sketch
  - Stack the boards and upload the sketch to board. To upload the sketch, you need to keep the Reset button on the Dragino pressed.
  (If you can upload without pressing that button, then this could be a sign that you have something wrong in the Dragino Jumpers or the Serial Settings of your sketch)
  - When you see the traffic in the TTN Console, you can proceed in configuring the Payload Decoder Function.
  - (if you do not see traffic in the TTN Console although you are sure that you are in good distance to a gateway, check the Frame Counter and the Dragino Jumpers)
  - Configure the TTNMapper-Integration, specify an Experiment Name for the time of the first try outs 
  (or you will disclose your home location with a cloud of successfull connect dots)
  - You should find your new Experiment at the end of [this list](https://ttnmapper.org/experiments/list_all.php).

 ## Dependencies
  - [Arduino LMIC](https://github.com/matthijskooijman/arduino-lmic)
  - [TinyGPS++](http://arduiniana.org/libraries/tinygpsplus/)
  - [CayenneLPP (Optional)](https://www.thethingsnetwork.org/docs/devices/arduino/api/cayennelpp.html) Install from Arduino Library Manager

 ## Configuration:
  - TXdist - defines, the distance [in meters], after which the position is sent
  - SF - Define The LoRaWAN Spreading Factor (DR_SF7 - DR_SF12) 7 and 8 recommended for mapping
  - SINGLE_CHANNEL - Only Use LoRaWAN Channel 0 for Single Channel Gateways
  - CONFIRMED - enables confirmed uplinks, ONLY Enable, if you conncect a Buzzer to Pin D5! Otherwise this feature is useless
  - SOFT_SERIAL - Uncomment to use Hardware Serial, otherwise Software Serial is used. In that case connect the GPS Module to RXpin and TXpin
  - DEBUG - If you use Hardware Serial you can enable DEBUG, to get Debug output on a Software Serial. Leave disabled to not use Software Serial at all
  - CAYENNELPP - If you want to use CayenneLPP as Payload Format otherwise use following Decoder Payload Function

 ## Payload Function
 ```C
 function Decoder(bytes, port) {
  var decoded = {};
  // if (port === 1) decoded.led = bytes[0];
  decoded.latitude = ((bytes[0]<<16)>>>0) + ((bytes[1]<<8)>>>0) + bytes[2];
  decoded.latitude = (decoded.latitude / 16777215.0 * 180) - 90;
  decoded.longitude = ((bytes[3]<<16)>>>0) + ((bytes[4]<<8)>>>0) + bytes[5];
  decoded.longitude= (decoded.longitude / 16777215.0 * 360) - 180;
  var altValue = ((bytes[6]<<8)>>>0) + bytes[7];
  var sign = bytes[6] & (1 << 7);
  if(sign)
  {
    decoded.altitude = 0xFFFF0000 | altValue;
  }
  else
  {
    decoded.altitude = altValue;
  }
  decoded.hdop = bytes[8] / 10.0;
  return decoded;
}
 ```
