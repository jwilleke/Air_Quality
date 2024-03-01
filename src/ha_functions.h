#pragma once
#include <Arduino.h>
#include <ha_config.h>


/**
 * @brief Print the MAC address of the given byte array
 * 
 * @param mac The byte array to print
 * @param len The length of the byte array
 
*/
void printByetArray(byte mac[], int len)
{
  for (int i = len; i > 0; i--)
  {
    if (mac[i] < 16)
    {
      Serial.print("0");
    }
    Serial.print(mac[i], HEX);
    if (i > 1)
    {
      Serial.print(":");
    }
  }
  Serial.println();
}

/**
 * @brief Callback function for the switch state change
*/
int getPercentage(int value)
{
  // Determine soil moisture percentage value
  value = map(value, AIR_VALUE, WATER_VALUE, 0, 100);
  // Keep values between 0 and 100
  value = constrain(value, 0, 100);
  return value;
}


/**
 * @brief Print the MAC address of the given byte array
 * 
*/
void printCurrentNet(WiFiClass WiFi, byte myId[])
{
  if (WiFi.status() == WL_CONNECTED)
  {
    DEBUG_PRINT("[*] Network information for ");
    DEBUG_PRINTLN(WiFi.SSID());

    DEBUG_PRINTLN("[+] BSSID : " + WiFi.BSSIDstr());
    DEBUG_PRINT("[+] Gateway IP : ");
    DEBUG_PRINTLN(WiFi.gatewayIP());
    DEBUG_PRINT("[+] Subnet Mask : ");
    DEBUG_PRINTLN(WiFi.subnetMask());
    DEBUG_PRINTLN((String) "[+] RSSI : " + WiFi.RSSI() + " dB");
    DEBUG_PRINT("[+] ESP32 IP : ");
    DEBUG_PRINTLN(WiFi.localIP());
  }
}