#include "Global.h"
#include "classes/IoTItem.h"

#include "AM2302-Sensor.h"
#include <map>

std::map<int, AM2302::AM2302_Sensor*> am2302s;

class Am2302t : public IoTItem {
   private:
    AM2302::AM2302_Sensor* _am2302;

   public:
    Am2302t(AM2302::AM2302_Sensor* am2302, String parameters) : IoTItem(parameters) {
        _am2302 = am2302;
        _am2302 -> begin(); // оно тут вообще надо ?
    }

    void doByInterval() {
        value.valD = _am2302->get_Temperature();
        if (String(value.valD) != "nan")
            regEvent(value.valD, "Am2302t");
        else
            SerialPrint("E", "Sensor AM2302t", "Error", _id);
    }

    ~Am2302t(){};
};

class Am2302h : public IoTItem {
   private:
    AM2302::AM2302_Sensor* _am2302;

   public:
    Am2302h(AM2302::AM2302_Sensor* am2302, String parameters) : IoTItem(parameters) {
        _am2302 = am2302;
        _am2302 -> begin(); // оно тут вообще надо ?
    }

    void doByInterval() {
        value.valD = _am2302->get_Humidity();
        if (String(value.valD) != "nan")
            regEvent(value.valD, "Am2302h");
        else
            SerialPrint("E", "Sensor Am2302h", "Error", _id);
    }

    ~Am2302h(){};
};

void* getAPI_Am2302(String subtype, String param) {
    if (subtype == F("Am2302t") || subtype == F("Am2302h")) {
        int pin;
        String senstype;
        jsonRead(param, "pin", pin);
        jsonRead(param, "senstype", senstype);
    

        if (am2302s.find(pin) == am2302s.end()) {
           am2302s[pin] = new AM2302::AM2302_Sensor(pin);  // так и не понял что тут надо было написать

       }
    
       if (subtype == F("Am2302t")) {
           return new Am2302t(am2302s[pin], param); //тут что-то пошло не так...
       } else if (subtype == F("Am2302h")) {
           return new Am2302h(am2302s[pin], param); //тут что-то пошло не так...
       }
    }
    
    return nullptr;
}
