#include <TimeLib.h> 
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <SPI.h>
#include <Wire.h>

const int relayPin = D1;
const long interval = 1000;  // pause for two seconds
int sensorValue = 0;


const char ssid[] = "";  //  your network SSID (name)
const char pass[] = "";       // your network password

IPAddress timeServer(62, 237, 86, 238);

const int timeZone = 2;

WiFiUDP Udp;
unsigned int localPort = 8888;  // local port to listen for UDP packets

void setup()
{
	Serial.begin(9600);

	pinMode(relayPin, OUTPUT);

	WiFi.begin(ssid, pass);

	while (WiFi.status() != WL_CONNECTED) {
		delay(100);
		Serial.print(".");
	}
	Serial.println();

	Udp.begin(localPort);

	setSyncProvider(getNtpTime);

	delay(2000);

}

void loop()
{
	sensorValue = analogRead(A0); // read analog input pin 0
	if (sensorValue < 150 && ((hour() >= 16 && hour() <= 23) || (hour() >= 7 && hour() <= 10)))
	{
		digitalWrite(relayPin, HIGH); // turn on relay with voltage HIGH
	}
	else
	{
		digitalWrite(relayPin, LOW);  // turn off relay with voltage LOW
	}
	digitalClockDisplaySerial();
	Serial.println(sensorValue);
	delay(interval);              // pause
}



void digitalClockDisplaySerial() {
	// digital clock display of the time
	Serial.print(hour());
	printDigitsSerial(minute());
	printDigitsSerial(second());
	Serial.print(" ");
	Serial.print(day());
	Serial.print(".");
	Serial.print(month());
	Serial.print(".");
	Serial.print(year());
	Serial.println();
}

void printDigitsSerial(int digits) {
	// utility for digital clock display: prints preceding colon and leading 0
	Serial.print(":");
	if (digits < 10)
		Serial.print('0');
	Serial.print(digits);
}

/*-------- NTP code ----------*/

const int NTP_PACKET_SIZE = 48; // NTP time is in the first 48 bytes of message
byte packetBuffer[NTP_PACKET_SIZE]; //buffer to hold incoming & outgoing packets

time_t getNtpTime()
{
	while (Udp.parsePacket() > 0); // discard any previously received packets
	Serial.println("Transmit NTP Request");
	sendNTPpacket(timeServer);
	uint32_t beginWait = millis();
	while (millis() - beginWait < 1500) {
		int size = Udp.parsePacket();
		if (size >= NTP_PACKET_SIZE) {
			Serial.println("Receive NTP Response");
			Udp.read(packetBuffer, NTP_PACKET_SIZE);  // read packet into the buffer
			unsigned long secsSince1900;
			// convert four bytes starting at location 40 to a long integer
			secsSince1900 = (unsigned long)packetBuffer[40] << 24;
			secsSince1900 |= (unsigned long)packetBuffer[41] << 16;
			secsSince1900 |= (unsigned long)packetBuffer[42] << 8;
			secsSince1900 |= (unsigned long)packetBuffer[43];
			return secsSince1900 - 2208988800UL + timeZone * SECS_PER_HOUR;
		}
	}
	Serial.println("No NTP Response :-(");
	return 0; // return 0 if unable to get the time
}

// send an NTP request to the time server at the given address
void sendNTPpacket(IPAddress &address)
{
	// set all bytes in the buffer to 0
	memset(packetBuffer, 0, NTP_PACKET_SIZE);
	// Initialize values needed to form NTP request
	// (see URL above for details on the packets)
	packetBuffer[0] = 0b11100011;   // LI, Version, Mode
	packetBuffer[1] = 0;     // Stratum, or type of clock
	packetBuffer[2] = 6;     // Polling Interval
	packetBuffer[3] = 0xEC;  // Peer Clock Precision
							 // 8 bytes of zero for Root Delay & Root Dispersion
	packetBuffer[12] = 49;
	packetBuffer[13] = 0x4E;
	packetBuffer[14] = 49;
	packetBuffer[15] = 52;
	// all NTP fields have been given values, now
	// you can send a packet requesting a timestamp:                 
	Udp.beginPacket(address, 123); //NTP requests are to port 123
	Udp.write(packetBuffer, NTP_PACKET_SIZE);
	Udp.endPacket();
}
