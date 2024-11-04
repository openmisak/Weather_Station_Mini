#ifndef CONFIG_H
#define CONFIG_H

/* Definujte typ desky
3.2 stará deska
3.5 novější deska
podle nastavení desky se chová zbytek kódu, posílá se to na správná čidla atd.
*/

#define ESP32_DEV_v32 1
#define ESP32_DEV_v35 2

//#define BOARD_TYPE ESP32_DEV_v32
#define BOARD_TYPE ESP32_DEV_v35

// vypln tve GUID cidla
String GUID = "tempV";
String luxGUID = "lux";

//konfigurace ESP32 desky
#define SLEEP_SEC 15 * 60          // Measurement interval 15 minutes (X * 60 in seconds) 
#define ADC_PIN 0                  // ADC pin on LaskaKit Meteo mini
#define deviderRatio 1.7693877551  // Voltage devider ratio on ADC pin 1M + 1.3MOhm (odpovídá schémetu desky)
#define SDA 19
#define SCL 18
#define PIN_ON 3

#if defined(BOARD_TYPE) && BOARD_TYPE == ESP32_DEV_v32
  #define BME280_ADDRESS (0x76) // adresa čidla deska v3.2 0x76
  #define TEMP_SENSOR_NAME "http://4t6jv7-ne224p.tmep.cz/index.php?"
  #define LUX_SENSOR_NAME "http://my9dhv-56c3u9.tmep.cz/index.php?"
  #define REF_VOLTAGE 3.3 // Float for Reference Voltage, old board 3.3, new version 1

#elif defined(BOARD_TYPE) && BOARD_TYPE == ESP32_DEV_v35
  #define BME280_ADDRESS (0x77) // adresa čidla deska v3.5 0x77
  #define  TEMP_SENSOR_NAME "http://apnfk5-v3yn9r.tmep.cz/index.php?"
  #define LUX_SENSOR_NAME "http://my9dhv-56c3u9.tmep.cz/index.php?"
  #define REF_VOLTAGE 1 // Float for Reference Voltage, old board 3.3, new version 1

#else
    #error "Unknown BOARD_TYPE selected"
#endif

#endif // CONFIG_H