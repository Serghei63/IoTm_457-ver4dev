#include "Global.h"
#include "classes/IoTItem.h"
#include "ESPConfiguration.h"
#include "NTP.h"

void *getAPI_Date2(String params);

class Loging2 : public IoTItem {
   private:
    String logid1;
    String logid2;
    String id;
    String tmpValue;
    String filesList = "";

    int _publishType = -2;
    int _wsNum = -1;

    int points;
    //int keepdays;

    IoTItem *dateIoTItem;

    String prevDate = "";
    bool firstTimeInit = true;

    //long interval;

   public:
    Loging2(String parameters) : IoTItem(parameters) {
        jsonRead(parameters, F("logid1"), logid1);
        jsonRead(parameters, F("logid2"), logid2);
        jsonRead(parameters, F("id"), id);
        jsonRead(parameters, F("points"), points);
        if (points > 300) {
            points = 300;
            SerialPrint("E", F("Loging2"), "'" + id + "' user set more points than allowed, value reset to 300");
        }

        long interval;

        jsonRead(parameters, F("int"), interval);
        interval = interval * 1000 * 60;  // приводим к милисекундам
        //jsonRead(parameters, F("keepdays"), keepdays, false);

        // создадим экземпляр класса даты
        dateIoTItem = (IoTItem *)getAPI_Date2("{\"id\": \"" + id + "-date\",\"int\":\"20\",\"subtype\":\"date\"}");
        IoTItems.push_back(dateIoTItem);
        SerialPrint("I", F("Loging2"), "created date instance " + id);
    }

    void doByInterval() {
        // если объект логгирования не был создан
        if (!isItemExist(logid1)) {
            SerialPrint("E", F("Loging2"), "'" + id + "' loging object not exist, return");
            return;
        }

        String value = getItemValue(logid1);
        // если значение логгирования пустое
        if (value == "") {
            SerialPrint("E", F("Loging2"), "'" + id + "' loging value is empty, return");
            return;
        }
                String value2 = getItemValue(logid2);
        // если значение логгирования пустое
        if (value == "") {
            SerialPrint("E", F("Loging2"), "'" + id + "' loging value is empty, return");
            return;
        }

        // если время не было получено из интернета
        if (!isTimeSynch) {
            SerialPrint("E", F("Loging2"), "'" + id + "' Сant loging - time not synchronized, return");
            return;
        }

        regEvent(value, F("Loging2"));

        String logData2;

        jsonWriteInt(logData2, "x", unixTime, false);
        jsonWriteFloat(logData2, "y1", value.toFloat(), false);
        jsonWriteFloat(logData2, "y2", value2.toFloat(), false);

        // прочитаем путь к файлу последнего сохранения
        String filePath = readDataDB(id);

        // если данные о файле отсутствуют, создадим новый
        if (filePath == "failed" || filePath == "") {
            SerialPrint("E", F("Loging2"), "'" + id + "' file path not found, start create new file");
            createNewFileWithData(logData2);
            return;
        } else {
            // если файл все же есть но был создан не сегодня, то создаем сегодняшний
            if (getTodayDateDotFormated() != getDateDotFormatedFromUnix(getFileUnixLocalTime(filePath))) {
                SerialPrint("E", F("Loging2"), "'" + id + "' file too old, start create new file");
                createNewFileWithData(logData2);
                return;
            }
        }

        // считаем количество строк и определяем размер файла
        size_t size = 0;
        int lines = countJsonObj(filePath, size);
        SerialPrint("i", F("Loging2"), "'" + id + "' " + "lines = " + String(lines) + ", size = " + String(size));

        // если количество строк до заданной величины и дата не менялась
        if (lines <= points && !hasDayChanged()) {
            // просто добавим в существующий файл новые данные
            addNewDataToExistingFile(filePath, logData2);
            // если больше или поменялась дата то создадим следующий файл
        } else {
            createNewFileWithData(logData2);
        }
        // запускаем процедуру удаления старых файлов если память переполняется
        deleteLastFile();
    }

    void SetDoByInterval(String valse) {
        String value = valse;
        // если значение логгирования пустое
        if (value == "") {
            SerialPrint("E", F("Loging2Event"), "'" + id + "' loging value is empty, return");
            return;
        }
        // если время не было получено из интернета
        if (!isTimeSynch) {
            SerialPrint("E", F("Loging2Event"), "'" + id + "' Сant loging - time not synchronized, return");
            return;
        }
        regEvent(value, F("Loging2Event"));
        String logData2;
        jsonWriteInt(logData2, "x", unixTime, false);
        jsonWriteFloat(logData2, "y1", value.toFloat(), false);
        jsonWriteFloat(logData2, "y2", value.toFloat(), false);
        // прочитаем путь к файлу последнего сохранения
        String filePath = readDataDB(id);

        // если данные о файле отсутствуют, создадим новый
        if (filePath == "failed" || filePath == "") {
            SerialPrint("E", F("Loging2Event"), "'" + id + "' file path not found, start create new file");
            createNewFileWithData(logData2);
            return;
        } else {
            // если файл все же есть но был создан не сегодня, то создаем сегодняшний
            if (getTodayDateDotFormated() != getDateDotFormatedFromUnix(getFileUnixLocalTime(filePath))) {
                SerialPrint("E", F("Loging2Event"), "'" + id + "' file too old, start create new file");
                createNewFileWithData(logData2);
                return;
            }
        }

        // считаем количество строк и определяем размер файла
        size_t size = 0;
        int lines = countJsonObj(filePath, size);
        SerialPrint("i", F("Loging2Event"), "'" + id + "' " + "lines = " + String(lines) + ", size = " + String(size));

        // если количество строк до заданной величины и дата не менялась
        if (lines <= points && !hasDayChanged()) {
            // просто добавим в существующий файл новые данные
            addNewDataToExistingFile(filePath, logData2);
            // если больше или поменялась дата то создадим следующий файл
        } else {
            createNewFileWithData(logData2);
        }
        // запускаем процедуру удаления старых файлов если память переполняется
        deleteLastFile();
    }
    void createNewFileWithData(String &logData) {
        logData = logData + ",";
        String path = "/lg2/" + id + "/" + String(unixTimeShort) + ".txt";  // создадим путь вида /lg/id/133256622333.txt
        // создадим пустой файл
        if (writeEmptyFile(path) != "success") {
            SerialPrint("E", F("Loging2"), "'" + id + "' file writing error, return");
            return;
        }

        // запишем в него данные
        if (addFile(path, logData) != "success") {
            SerialPrint("E", F("Loging2"), "'" + id + "' data writing error, return");
            return;
        }
        // запишем путь к нему в базу данных
        if (saveDataDB(id, path) != "success") {
            SerialPrint("E", F("Loging2"), "'" + id + "' db file writing error, return");
            return;
        }
        SerialPrint("i", F("Loging2"), "'" + id + "' file created http://" + WiFi.localIP().toString() + path);
    }

    void addNewDataToExistingFile(String &path, String &logData) {
        logData = logData + ",";
        if (addFile(path, logData) != "success") {
            SerialPrint("i", F("Loging2"), "'" + id + "' file writing error, return");
            return;
        };
        SerialPrint("i", F("Loging2"), "'" + id + "' loging in file http://" + WiFi.localIP().toString() + path);
    }

    // данная функция уже перенесена в ядро и будет удалена в последствии
    bool hasDayChanged() {
        bool changed = false;
        String currentDate = getTodayDateDotFormated();
        if (!firstTimeInit) {
            if (prevDate != currentDate) {
                changed = true;
                SerialPrint("i", F("NTP"), F("Change day event"));
#if defined(ESP8266)
                FileFS.gc();
#endif
#if defined(ESP32)
#endif
            }
        }
        firstTimeInit = false;
        prevDate = currentDate;
        return changed;
    }

    void publishValue() {
        String dir = "/lg2/" + id;
        filesList = getFilesList(dir);

        SerialPrint("i", F("Loging2"), "file list: " + filesList);

        int f = 0;

        bool noData = true;

        while (filesList.length()) {
            String path = selectToMarker(filesList, ";");

            path = "/lg2/" + id + path;

            f++;

            unsigned long fileUnixTimeLocal = getFileUnixLocalTime(path);

            unsigned long reqUnixTime = strDateToUnix(getItemValue(id + "-date"));
            if (fileUnixTimeLocal > reqUnixTime && fileUnixTimeLocal < reqUnixTime + 86400) {
                noData = false;
                String json = getAdditionalJson();
                if (_publishType == TO_MQTT) {
                    publishChartFileToMqtt(path, id, calculateMaxCount());
                } else if (_publishType == TO_WS) {
                    sendFileToWsByFrames(path, "charta", json, _wsNum, WEB_SOCKETS_FRAME_SIZE);
                } else if (_publishType == TO_MQTT_WS) {
                    sendFileToWsByFrames(path, "charta", json, _wsNum, WEB_SOCKETS_FRAME_SIZE);
                    publishChartFileToMqtt(path, id, calculateMaxCount());
                }
                SerialPrint("i", F("Loging2"), String(f) + ") " + path + ", " + getDateTimeDotFormatedFromUnix(fileUnixTimeLocal) + ", sent");
            } else {
                SerialPrint("i", F("Loging2"), String(f) + ") " + path + ", " + getDateTimeDotFormatedFromUnix(fileUnixTimeLocal) + ", skipped");
            }

            filesList = deleteBeforeDelimiter(filesList, ";");
        }
        // если данных нет отправляем пустой грфик
        if (noData) {
            clearValue();
        }
    }

    String getAdditionalJson() {
        String topic = mqttRootDevice + "/" + id;
        String json = "{\"maxCount\":" + String(calculateMaxCount()) + ",\"topic\":\"" + topic + "\"}";
        return json;
    }

    void publishChartToWsSinglePoint(String value) {
        String topic = mqttRootDevice + "/" + id;
        String json = "{\"maxCount\":" + String(calculateMaxCount()) + ",\"topic\":\"" + topic + "\",\"status\":[{\"x\":" + String(unixTime) + ",\"y1\":" + value + ",\"y2\":" + value + "}]}";
        sendStringToWs("chartb", json, -1);
    }

    void clearValue() {
        String topic = mqttRootDevice + "/" + id;
        String json = "{\"maxCount\":0,\"topic\":\"" + topic + "\",\"status\":[]}";
        sendStringToWs("chartb", json, -1);
    }

    void clearHistory() {
        String dir = "/lg2/" + id;
        cleanDirectory(dir);
    }

    void deleteLastFile() {
        IoTFSInfo tmp = getFSInfo();
        SerialPrint("i", "Loging2", String(tmp.freePer) + " % free flash remaining");
        if (tmp.freePer <= 20.00) {
            String dir = "/lg2/" + id;
            filesList = getFilesList(dir);
            int i = 0;
            while (filesList.length()) {
                String path = selectToMarker(filesList, ";");
                path = dir + path;
                i++;
                if (i == 1) {
                    removeFile(path);
                    SerialPrint("!", "Loging2", String(i) + ") " + path + " => oldest files been deleted");
                    return;
                }

                filesList = deleteBeforeDelimiter(filesList, ";");
            }
        }
    }

    void setPublishDestination(int publishType, int wsNum) {
        _publishType = publishType;
        _wsNum = wsNum;
    }

    String getValue() {
        return "";
    }
/*
    void loop() {
        if (enableDoByInt) {
            currentMillis = millis();
            difference = currentMillis - prevMillis;
            if (difference >= interval) {
                prevMillis = millis();
                if (interval != 0) {
                    this->doByInterval();
                }
            }
        }
    }
*/
    void regEvent(const String &value, const String &consoleInfo, bool error = false, bool genEvent = true) {
        String userDate = getItemValue(id + "-date");
        String currentDate = getTodayDateDotFormated();
        // отправляем в график данные только когда выбран сегодняшний день
        if (userDate == currentDate) {
            // generateEvent(_id, value);
            // publishStatusMqtt(_id, value);

            publishChartToWsSinglePoint(value);
            // SerialPrint("i", "Sensor " + consoleInfo, "'" + _id + "' data: " + value + "'");
        }
    }

    // просто максимальное количество точек
    int calculateMaxCount() {
        return 86400;
    }

    // путь вида: /lg/log/1231231.txt
    unsigned long getFileUnixLocalTime(String path) {
        return gmtTimeToLocal(selectToMarkerLast(deleteToMarkerLast(path, "."), "/").toInt() + START_DATETIME);
    }
    void setValue(const IoTValue &Value, bool genEvent = true) {
        value = Value;
        this->SetDoByInterval(String(value.valD));
        SerialPrint("i", "Loging2", "setValue:" + String(value.valD));
        regEvent(value.valS, "Loging2", false, genEvent);
    }
};

void *getAPI_Loging2(String subtype, String param) {
    if (subtype == F("Loging2")) {
        return new Loging2(param);
    } else {
        return nullptr;
    }
}

class Date2 : public IoTItem {
   private:
    bool firstTime = true;

   public:
    String id;
    Date2(String parameters) : IoTItem(parameters) {
        jsonRead(parameters, F("id2"), id);
        value.isDecimal = false;
    }

    void setValue(const String &valStr, bool genEvent = true) {
        value.valS = valStr;
        setValue(value, genEvent);
    }

    void setValue(const IoTValue &Value, bool genEvent = true) {
        value = Value;
        regEvent(value.valS, "", false, genEvent);
        // отправка данных при изменении даты
        for (std::list<IoTItem *>::iterator it = IoTItems.begin(); it != IoTItems.end(); ++it) {
            if ((*it)->getSubtype() == "Loging") {
                if ((*it)->getID() == selectToMarker(id, "-")) {
                    (*it)->setPublishDestination(TO_MQTT_WS, -1);
                    (*it)->publishValue();
                }
            }
        }
    }

    void setTodayDate() {
        setValue(getTodayDateDotFormated());
        SerialPrint("E", F("Loging2"), "today date set " + getTodayDateDotFormated());
    }

    void doByInterval() {
        if (isTimeSynch) {
            if (firstTime) {
                setTodayDate();
                firstTime = false;
            }
        }
    }
};

void *getAPI_Date2(String param) {
    return new Date2(param);
}
