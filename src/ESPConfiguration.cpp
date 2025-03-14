#include "ESPConfiguration.h"
#include "classes/IoTGpio.h"
//#include "classes/IoTDiscovery.h"

extern IoTGpio IoTgpio;

std::list<IoTItem*> IoTItems;
void* getAPI(String subtype, String params);

void configure(String path) {
    File file = seekFile(path);
    if (!file) {
        SerialPrint(F("E"), F("FS"), F("configure file open error"));
        return;
    }
    file.find("[");
    while (file.available()) {
        String jsonArrayElement = file.readStringUntil('}') + "}";
        if (jsonArrayElement.startsWith(",")) {
            jsonArrayElement = jsonArrayElement.substring(1, jsonArrayElement.length());  //это нужно оптимизировать в последствии
        }
        if (jsonArrayElement == "]}") {
            jsonArrayElement = "";
        }
        if (jsonArrayElement != "") {
            IoTItem* myIoTItem;
            String subtype;
            if (!jsonRead(jsonArrayElement, F("subtype"), subtype)) {  //если нет такого ключа в представленном json или он не валидный
                SerialPrint(F("E"), F("Config"), "json error " + subtype);
                continue;
            } else {
                //(IoTItem*) - getAPI вернула ссылку, что бы ее привести к классу IoTItem используем
                myIoTItem = (IoTItem*)getAPI(subtype, jsonArrayElement);
                if (myIoTItem) {
                    void* driver;
                    // пробуем спросить драйвер GPIO
                    if (driver = myIoTItem->getGpioDriver()) IoTgpio.regDriver((IoTGpio*)driver);
                    // пробуем спросить драйвер RTC
                    if (driver = myIoTItem->getRtcDriver()) rtcItem = (IoTItem*)driver;
                    // пробуем спросить драйвер CAM
                    //if (driver = myIoTItem->getCAMDriver()) camItem = (IoTItem*)driver;
                    // пробуем спросить драйвер Benchmark
                    if (driver = myIoTItem->getBenchmarkTask()) benchTaskItem = ((IoTBench*)driver);
                    if (driver = myIoTItem->getBenchmarkLoad()) benchLoadItem = ((IoTBench*)driver);
                    // пробуем спросить драйвер для интеграций
                    if (driver = myIoTItem->getHOMEdDiscovery()) HOMEdDiscovery = ((IoTDiscovery*)driver);
                    if (driver = myIoTItem->getHADiscovery()) HADiscovery = ((IoTDiscovery*)driver);
                    // пробуем спросить драйвер Telegram_v2
                    if (driver = myIoTItem->getTlgrmDriver()) tlgrmItem = (IoTItem*)driver;
                    IoTItems.push_back(myIoTItem);
                }
            }
        }
    }
    file.close();
    SerialPrint("i", "Config", "Configured");
/*      
#ifdef ESP32
  if(HOMEdDiscovery)
        HOMEdDiscovery->mqttSubscribeDiscovery();
    if(HADiscovery)
        HADiscovery->mqttSubscribeDiscovery();
        // оттправляем все статусы
    if(HOMEdDiscovery || HADiscovery)
    {
        for (std::list<IoTItem *>::iterator it = IoTItems.begin(); it != IoTItems.end(); ++it)
        {
            if ((*it)->iAmLocal)
            {
                publishStatusMqtt((*it)->getID(), (*it)->getValue());
                (*it)->onMqttWsAppConnectEvent();
            }
        }
    }
#endif
*/
}

void clearConfigure() {
    Serial.printf("Start clearing config\n");
    rtcItem = nullptr;
    //camItem = nullptr;
    tlgrmItem = nullptr;
    IoTgpio.clearDrivers();
    
    for (std::list<IoTItem*>::iterator it = IoTItems.begin(); it != IoTItems.end(); ++it) {
        Serial.printf("Start delete iotitem %s \n", (*it)->getID().c_str());
        if (*it) delete *it;
    }
    IoTItems.clear();
#ifdef LIBRETINY
    valuesFlashJson.remove(0, valuesFlashJson.length());
#else
    valuesFlashJson.clear();
#endif
    benchTaskItem = nullptr; 
    benchLoadItem = nullptr; 
    HOMEdDiscovery = nullptr;
    HADiscovery = nullptr;
}