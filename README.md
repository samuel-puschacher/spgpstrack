# spGPStrack - LoRaWAN GPS Mapping

This Program is a LoRaWAN GPS Tracker for mapping purposes
It is developed for Arduino Uno and Dragino Lora/GPs Shield
  - will transmit the location all x Meters
  - you can Choose the Payload Format between CayenneLPP and a Payload decoder Function
  - the Software supports confirmed uplink and the Arduino will beep
  - You can read The GPS over Hardware Serial or Software Serial, so it's easier for development if you see what is going on during real operation
  - In case of Hardware Serial usage you can enable a Software Serial for debugging
To get started just replace the LoRaWAN Keys!!!
 ## Dependencies
   - [Arduino LMIC](https://github.com/matthijskooijman/arduino-lmic)
   - [TinyGPS++](http://arduiniana.org/libraries/tinygpsplus/)
   - [CayenneLPP (Optional)](https://www.thethingsnetwork.org/docs/devices/arduino/api/cayennelpp.html) Install from Arduino Library Manager

 ## Configuration:
  - TXdist - defines, the distance [in Meter], after which the position is sent
  - SF - Define The LoRaWAN Spreading Factor (DR_SF7 - DR_SF12) 7 and 8 recommended for Mapping
  - SINGLE_CHANNEL - Only Use LoRaWAN Channel 0 for Single Channel Gateways
  - CONFIRMED - enables Confirmed uplinks, ONLY Enable, if you conncect a Buzzer to Pin D5! Otherwise this Feature is useless
  - SOFT_SERIAL - Uncomment to use Hardware Serial, Otherwise Software Serial is used. In that case connect the GPS Module to RXpin and TXpin
  - DEBUG - If you use Hardware Serial you can enable DEBBUG, to get Debug outputon a Software Serial. Leave disabled to not use Software Serial at all
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