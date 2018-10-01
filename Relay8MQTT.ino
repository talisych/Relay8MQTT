/*------------------------- Pre-Configuration ----------------------------*/
#define W5500                       5500
#define W5100                       5100
#define ETHER_TYPE                  W5500
#define ENABLE_DHCP                 false
#define ENABLE_EXTERNAL_WATCHDOG    true
#define _DEBUG_LEVEL                1

#define BROKER_IP                   192,168,1,50

#define HOST_IP                     192,168,1,163
#define HOST_NETMASK                255,255,255,0
#define HOST_DNS                    192,168,1,1
#define HOST_GATEWAY                192,168,1,1

#define MAC_REFRESH                 false
#define MAC_0                       0xDA
#define MAC_1                       0x00
#define MAC_2                       0x00
#define MAC_3                       0x00
#define MAC_4                       0x00
#define MAC_5                       0x00

/*------------------------------- Start ----------------------------------*/
#include <SPI.h>
#include <EEPROM.h>
#include <PubSubClient.h>
#if (ETHER_TYPE == W5500)
#include "Ethernet2.h"
#else
#include "Ethernet.h"
#endif
#include "Wire.h"
/*----------------------------- Watch dog --------------------------------*/
#define WATCHDOG_PIN                    12
#define WATCHDOG_PULSE_LENGTH           50        // Milliseconds
#define WATCHDOG_RESET_INTERVAL         5000      // Milliseconds. Also the period for sensor reports.
long watchdogLastResetTime = 0;

/*-------------------------- Get MAC address -----------------------------*/
/* CHANGE THIS TO YOUR OWN UNIQUE VALUE.  The MAC number should be
   different from any other devices on your network or you'll have
   problems receiving packets. Can be replaced automatically below
   using a MAC address ROM. */
#define MAC_PREFIX   '#'
#define MAC_START    (1)
#define MAC(idx)     (MAC_START + idx)
#define MAC_LEN      (6)
static uint8_t mac[MAC_LEN] = { MAC_0, MAC_1, MAC_2, MAC_3, MAC_4, MAC_5 };
char get_MAC(uint8_t *mac_buf, bool mac_refresh);

/*----------------------- Define IP information --------------------------*/
#define DEF_IP           192,168,1,196
#define DEF_NETMASK      255,255,255,0
#define DEF_DNS          8,8,8,8
#define DEF_GATEWAY      192,168,1,1

IPAddress broker(BROKER_IP);

#if (defined HOST_IP) && defined (HOST_NETMASK)
IPAddress ip(HOST_IP);
IPAddress submask(HOST_NETMASK);
#else
IPAddress ip(DEF_IP);
IPAddress submask(DEF_NETMASK);
#endif

#if (defined HOST_DNS) && defined (HOST_GATEWAY)
IPAddress _dns(HOST_DNS);
IPAddress gateway(HOST_GATEWAY);
#else
IPAddress _dns(DEF_DNS);
IPAddress gateway(DEF_GATEWAY);
#endif

/*---------------------- Define Relay8 information -----------------------*/
#define INTERVAL_TIME           50   /* Interval time(ms) for switch relay. */
#define MAX_CHANNEL_NUM          8
#define MAX_SHIELD_NUM           8

/* Define MCP23017 register */
#define MCP23017_GPIOA 0x12

typedef enum {
    RELAY8_I2C_ADDR_1 = 0x20,   // 0x20 is the address with all jumpers removed
    RELAY8_I2C_ADDR_2 = 0x21,   // 0x21 is the address with a jumper on position A0
    RELAY8_I2C_ADDR_3 = 0x22,   // 0x22 is the address with a jumper on position A1
    RELAY8_I2C_ADDR_4 = 0x23,   // 0x23 is the address with a jumper on position A1 and A2
    RELAY8_I2C_ADDR_5 = 0x24,   // 0x24 is the address with a jumper on position A3
    RELAY8_I2C_ADDR_6 = 0x25,   // 0x25 is the address with a jumper on position A0 and A3
    RELAY8_I2C_ADDR_7 = 0x26,   // 0x26 is the address with a jumper on position A1 and A3
    RELAY8_I2C_ADDR_8 = 0x27    // 0x27 is the address with a jumper on position A1, A2 and A3
} RELAY8_I2C_ADDR;

typedef enum {
  SWITCH_NONE = 0,
  SWITCH_ON,
  SWITCH_OFF
} SWITCH_OPTION;

const uint8_t relay_shield_address[MAX_SHIELD_NUM] =
                               {RELAY8_I2C_ADDR_1, RELAY8_I2C_ADDR_2,
                                RELAY8_I2C_ADDR_3, RELAY8_I2C_ADDR_4,
                                RELAY8_I2C_ADDR_5, RELAY8_I2C_ADDR_6,
                                RELAY8_I2C_ADDR_7, RELAY8_I2C_ADDR_8};
uint8_t relay_shield_valid[MAX_SHIELD_NUM] =
                               {0, 0, 0, 0, 0, 0, 0, 0};

/*----------------------- Define MQTT information -----------------------*/
#define CID_BUF_LEN 3   /* Buf size for parse channel ID from MQTT message. */

/* MQTT define */
const char * const eventsTopic  = "events";    // MQTT topic to publish status reports
const char * const cmd_topic PROGMEM = "/device/%02x%02x%02x/ch/+/command";
const char * const state_topic PROGMEM = "/device/%02x%02x%02x/ch/%d/state";
const char * const client_str PROGMEM = "Relay8-%02x%02x%02x%02x%02x%02x";
const char * const mqtt_msg_str PROGMEM = "%s is connected.";
char str_buf[64] = {0};

/**
   MQTT callback
*/
void callback(char* topic, byte* payload, unsigned int length);

/*------------------------------- Setup ---------------------------------*/
EthernetClient ethclient;
PubSubClient client(ethclient);

void setup()
{
  Wire.begin(); // Wake up I2C bus
  Serial.begin( 9600 );
  
  /* Set up the watchdog timer */
  if(ENABLE_EXTERNAL_WATCHDOG == true)
  {
    pinMode(WATCHDOG_PIN, OUTPUT);
    digitalWrite(WATCHDOG_PIN, LOW);
  }

  while (!Serial && millis() < 2000) {
    ;
  }
  Serial.print(F("Getting MAC address from ROM: "));
  get_MAC(mac, MAC_REFRESH);
  char tmpBuf[17];
  sprintf(tmpBuf, "%02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  Serial.println(tmpBuf);

  // Set up the Ethernet library to talk to the Wiznet board
  if ( ENABLE_DHCP == true )
  {
    Ethernet.begin(mac);      // Use DHCP
  } else {
    Ethernet.begin(mac, ip);  // Use static address defined above
    Ethernet.begin(mac, ip, _dns, gateway, submask);  // Use static address defined above
  }

  // Print IP address:
  Serial.print(F("Local IP: "));
  for (byte thisByte = 0; thisByte < 4; thisByte++) {
    // print the value of each byte of the IP address:
    Serial.print(Ethernet.localIP()[thisByte], DEC);
    if ( thisByte < 3 )
    {
      Serial.print(F("."));
    }
  }
  Serial.println();

  /* Set up the Relay8 shields */
  Serial.print(F("Detect Relay8 shields: "));
  for (byte idx = 0; idx < 8; idx++) {
    byte error;
    error = initialiseRelay8Shield(relay_shield_address[idx]);
    if (error == 0) {
      /* Detect shield board. */
      relay_shield_valid[idx] = 1;
      Serial.print(F("0x"));
      Serial.print(relay_shield_address[idx], HEX);
      Serial.print(F(" "));
    }
    else {
      /* Non-detected shield board. */
      relay_shield_valid[idx] = 0;
    }
  }
  Serial.println();

  client.setServer(broker, 1883);
  client.setCallback(callback);

  Serial.println(F("Ready."));
}

/*-------------------------------- Loop ---------------------------------*/
void loop()
{
  if (!client.connected()) {
    reconnect();
  }

  runHeartbeat();
  
  client.loop();
}

void callback(char* topic, byte* payload, unsigned int length)
{
  int str_start = 0, str_end = 0, str_len;
  int ch_id = 0;
  char buf[CID_BUF_LEN] = {0};
  byte value = 0;
  int rv = 0;
  SWITCH_OPTION option = SWITCH_NONE;

#if (_DEBUG_LEVEL > 0)
  Serial.println();
  Serial.print(F("Message arrived ["));
  Serial.print(topic);
  Serial.print(F("] "));
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println("");
#endif

  /* Parse the channel ID from MQTT message. */
  str_start = (int)strstr(topic, "ch/") - (int)topic;
  str_end = (int)strstr(topic, "/command") - (int)topic;
  str_len = (str_end - (str_start + 3));
  if (str_len > CID_BUF_LEN) {
    Serial.println("F( Can not to parse channel ID.)");
    return;
  }

  strncpy(buf, (strstr(topic, "ch/") + 3), str_len );
#if (_DEBUG_LEVEL > 0)
  Serial.print(F("Parse channel ID: "));
  for (int i = 0; i < str_len; i++) {
    Serial.print((char)buf[i]);
  }
  Serial.println("");
#endif

  ch_id = atoi(buf);
  if (ch_id >= MAX_CHANNEL_NUM * MAX_SHIELD_NUM) {
    Serial.println(F(" Invalid channel ID, the channel range is 1 from 64."));
    return;
  }
  else {
    Serial.print(F("  Channel ID: "));
    Serial.print(ch_id, DEC);
  }

  if (relay_shield_valid[(ch_id - 1) / MAX_SHIELD_NUM] == 0) {
    Serial.print(F(", Invalid ID on Relay8 shield: 0x"));
    Serial.println(relay_shield_address[(ch_id - 1) / MAX_SHIELD_NUM], HEX);
    return;
  }

  if (strncmp((char *)payload, "on", length) == 0) {
    option = SWITCH_ON;
  }
  if (strncmp((char *)payload, "off", length) == 0) {
    option = SWITCH_OFF;
  }
  if (option != SWITCH_NONE) {
    rv = switch_relay(ch_id, option);
    if (rv == 0) {
      Serial.println( option == SWITCH_ON ?
                                  F(", trun on the channel.") :
                                  F(", trun off the channel."));
    }
    else {
      Serial.print(option == SWITCH_ON ?
                               F(", turn on error in the channel. (rv=") :
                               F(", turn off error in the channel. (rv="));
      Serial.print(rv);
      Serial.println(F(")"));
    }
  }
  else {
    Serial.println(F(", only support \"on\" and \"off\" payload."));
  }
}

void reconnect() {
  // Loop until we're reconnected
  sprintf(str_buf, client_str, mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  Serial.print("Client Id: ");
  Serial.println(str_buf);

  while (!client.connected()) {
    Serial.print(F("Attempting MQTT connection..."));
    // Attempt to connect
    if (client.connect(str_buf)) {
      Serial.println(F("connected"));
      // Once connected, publish an announcement...
      sprintf(str_buf, mqtt_msg_str, str_buf);
      client.publish(eventsTopic, str_buf);
      // ... and resubscribe
      sprintf(str_buf, cmd_topic, mac[3], mac[4], mac[5]);
      client.subscribe(str_buf);
    } else {
      Serial.print(F("failed, rc="));
      Serial.print(client.state());
      Serial.println(F(" try again in 5 seconds"));
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

/*-------------------------- Relay8 function ----------------------------*/
byte initialiseRelay8Shield(byte shieldAddress)
{
  // Set I/O bank A(0x00) to outputs(0x00)
  return write_byte_to_shield(shieldAddress, 0x00, 0x00);
}

byte write_byte_to_shield(byte address, byte offset, byte value)
{
  Wire.beginTransmission(address);
  Wire.write(offset);        // Select GPIOA
  Wire.write(value);         // Send value to bank A
  return Wire.endTransmission();
}

byte read_byte_to_shield(byte address, byte offset, byte *value)
{
  int rv = 0;
  Wire.beginTransmission(address);
  Wire.write(offset);
  rv = Wire.endTransmission();
  if ( rv != 0) {
    return rv;
  }
  Wire.requestFrom((int)address, 0x1);
  if (!Wire.available()) {
    // Wait;
  }
  else {
    *value = Wire.read();
  }
  return rv;
}

int switch_relay(byte ch_id, SWITCH_OPTION option) {
    int rv = -1;
    byte value = 0;
    rv = read_byte_to_shield(relay_shield_address[(ch_id - 1) / MAX_SHIELD_NUM], MCP23017_GPIOA, &value);
    if ( rv == 0) {
      delay(INTERVAL_TIME);
      if(option == SWITCH_ON)
        rv = write_byte_to_shield(relay_shield_address[(ch_id - 1) / MAX_SHIELD_NUM],
                                  MCP23017_GPIOA,
                                  value | (0x1 << ((ch_id - 1) % MAX_CHANNEL_NUM)));
      else
        rv = write_byte_to_shield(relay_shield_address[(ch_id - 1) / MAX_SHIELD_NUM],
                                  MCP23017_GPIOA,
                                  value & ~(0x1 << ((ch_id - 1) % MAX_CHANNEL_NUM)));

      if (rv == 0) {
        sprintf(str_buf, state_topic, mac[3], mac[4], mac[5], ch_id);
        client.publish(str_buf, option == SWITCH_ON ? "on" : "off");
      }
      return rv;
    }

    return rv;
}

/*---------------------- Get MAC address function -----------------------*/
char get_MAC(uint8_t *mac_buf, bool mac_refresh) {
  // Random MAC address stored in EEPROM
  if (mac_refresh == false && EEPROM.read(MAC_START) == MAC_PREFIX) {
    for (int ee_idx = MAC(1), buf_idx = 0; ee_idx <= (MAC_START + MAC_LEN); ee_idx++, buf_idx++) {
      mac_buf[buf_idx] = EEPROM.read(ee_idx);
    }
  } else {
    randomSeed(analogRead(0));
    EEPROM.write(MAC(1), mac_buf[0] & 0xFE);
    for (int ee_idx = MAC(2), buf_idx = 1; ee_idx <= (MAC_START + MAC_LEN); ee_idx++, buf_idx++) {
      mac_buf[buf_idx] = random(0, 255);
      EEPROM.write(ee_idx, mac_buf[buf_idx]);
    }
    // Write prefix
    EEPROM.write(MAC_START, MAC_PREFIX);
  }
  return 0;
}

/**
 * The heartbeat takes care of both patting the watchdog and reporting sensor values
 */
void runHeartbeat()
{
  if((millis() - watchdogLastResetTime) > WATCHDOG_RESET_INTERVAL)  // Is it time to run yet?
  {
      patWatchdog();  // Only pat the watchdog if we successfully published to MQTT
    // The interval timer is updated inside patWatchdog()
  }
}

/**
 * Pulse the hardware watchdog timer pin to reset it
 */
void patWatchdog()
{
  if( ENABLE_EXTERNAL_WATCHDOG )
  {
    digitalWrite(WATCHDOG_PIN, HIGH);
    delay(WATCHDOG_PULSE_LENGTH);
    digitalWrite(WATCHDOG_PIN, LOW);
  }
  watchdogLastResetTime = millis();
}
