#include "Global.h"
#include "classes/IoTItem.h"
#include "UpgradeFirm.h"
// #define FB_NO_UNICODE
// #define FB_NO_URLENCODE
// #define FB_NO_OTA
// #define FB_DYNAMIC
// #include <GyverGFX.h>
// #include <CharPlot.h>
// #include "esp_camera.h"
#ifdef ESP8266
#define FB_DYNAMIC
#endif

#include <FastBot.h>
#include <map>

// FastBot _myBot;
FastBot *_myBot = nullptr;
FastBot *instanceBot()
{
    if (!_myBot)
    {
        _myBot = new FastBot();
        // ot->begin();
    }
    return _myBot;
}

String _token;
String _chatID;
bool _autos;
bool _initSD;
bool _resolveOTA;
bool fl_rollback;
bool fl_reboot;
int8_t _OTAstate = -1;
struct ButtonMenu
{
    String message = "";
    String getId = "";
    String setId = "";
    String value = "";
};
/*
struct updateFirm {
    String settingsFlashJson;
    String configJson;
    String layoutJson;
    String scenarioTxt;
    String chartsData;
};
*/
std::map<String, ButtonMenu *> mapBtnMenu;   // <btnName, ID>
std::map<String, ButtonMenu *> mapBtnInline; // <btnName, ID>

class Telegram_v2 : public IoTItem
{
private:
    bool _receiveMsg;
    String _prevMsg = "";
    bool _useLed = false;
    uint8_t _textMode = 0;

public:
    Telegram_v2(String parameters) : IoTItem(parameters)
    {
        jsonRead(parameters, "token", _token);
        jsonRead(parameters, "autos", _autos);
        if (!jsonRead(parameters, "OTA", _resolveOTA))
            _resolveOTA = 0;
        jsonRead(parameters, "receiveMsg", _receiveMsg);
        jsonRead(parameters, "chatID", _chatID);
        _textMode = jsonReadInt(parameters, "textMode");
        fl_rollback = false;
        fl_reboot = false;
        instanceBot();
        _myBot->setTextMode(_textMode);
        _myBot->attach(telegramMsgParse);

#ifdef ESP32
        //          _myBot->useDNS(true);
#endif

        _myBot->setToken(_token);
        //  _myBot->enableUTF8Encoding(true);
        _myBot->setChatID(_chatID);
        //        _myBot->showMenuText("help","help",false);
    }

    void loop()
    {
        if (_receiveMsg && isNetworkActive())
        {
            _myBot->tick();
            if (fl_rollback)
            {
#ifdef ESP32
                _myBot->tickManual(); // Чтобы отметить сообщение прочитанным
                if (Update.rollBack())
                {
                    SerialPrint("I", F("Update"), F("Откат OTA успешно выполнен"));
                    _myBot->sendMessage("Откат OTA запущен, плата будет перезагружена", _chatID);
                    ESP.restart();
                }
                else
                {
                    SerialPrint("E", F("Update"), F("Откат OTA не выполнен!"));
                    _myBot->sendMessage("Откат OTA не выполнен!", _chatID);
                }
#else
                SerialPrint("I", F("Update"), F("Откат OTA только в ESP32"));
                _myBot->sendMessage("Откат OTA поддерживается только в ESP32", _chatID);
#endif
            }
            // была попытка OTA обновления. Обновляемся после ответа серверу!
            if (_OTAstate >= 0)
            {
                _myBot->tickManual(); // Чтобы отметить сообщение прочитанным
                String ota;
                if (_OTAstate == 0)
                    ota = F("Error");
                else if (_OTAstate == 1)
                    ota = F("No updates");
                else if (_OTAstate == 2)
                    ota = F("OK");
                _myBot->sendMessage(ota, _chatID);
                if (_OTAstate == 2)
                    ESP.restart();
                _OTAstate = -1;
            }
            if (fl_reboot)
            {
                _myBot->tickManual();
                ESP.restart();
            }
        }
        // Далее вызов doByInterval для обработки комманд
        IoTItem::loop();
    }

    void doByInterval()
    {
    }

    //=============================================================================
    //=============++====== Обработка команд сценария =============================
    //=============================================================================
    IoTValue execute(String command, std::vector<IoTValue> &param)
    {
        if (!isNetworkActive())
            return {};
        if (command == "sendMsg")
        {
            if (param.size())
            {
                String strTmp;
                if (param[0].isDecimal)
                    strTmp = param[0].valD;
                else
                    strTmp = param[0].valS;
                sendTelegramMsg(false, strTmp);
            }
        }
        else if (command == "sendOftenMsg")
        {
            if (param.size())
            {
                String strTmp;
                if (param[0].isDecimal)
                    strTmp = param[0].valD;
                else
                    strTmp = param[0].valS;
                sendTelegramMsg(true, strTmp);
            }
        }
        else if (command == "sendPinMsg")
        {
            if (param.size())
            {
                String strTmp;
                if (param[0].isDecimal)
                    strTmp = param[0].valD;
                else
                    strTmp = param[0].valS;
                _myBot->sendMessage(strTmp, _chatID);
                _myBot->pinMessage(_myBot->lastBotMsg());

                SerialPrint("i", F("Telegram"), "chat ID: " + _chatID + ", pin msg: " + strTmp);
            }
        }
        else if (command == "unpinAllMsg")
        {
            _myBot->unpinAll(_chatID);
            SerialPrint("i", F("Telegram"), "chat ID: " + _chatID + ", unpin all message");
        }
        else if (command == "editMsg")
        {
            if (param.size())
            {
                String strTmp;
                if (param[0].isDecimal)
                    strTmp = param[0].valD;
                else
                    strTmp = param[0].valS;
                _myBot->editMessage(_myBot->lastBotMsg(), strTmp);
                SerialPrint("i", F("Telegram"), "chat ID: " + _chatID + ", edit msg: " + strTmp);
            }
        }
        else if (command == "sendFile")
        {
            if (param.size() && !param[0].isDecimal)
            {
                //     String path = filepath(filename);
                auto file = FileFS.open(param[0].valS, FILE_READ);
                if (!file)
                {
                    SerialPrint("E", F("Telegram"), "Fail send file: " + param[0].valS);
                    return {};
                }
                //  File file = LittleFS.open(param[0].valS, "r"); // /test.png
                // selectToMarkerLast(msg.text, "_")
                uint8_t res = _myBot->sendFile(file, (FB_FileType)param[1].valD, selectToMarkerLast(param[0].valS, "/"), _chatID);
                file.close();
                SerialPrint("i", F("Telegram"), "chat ID: " + _chatID + ", sendFile: " + param[0].valS + " res: " + String(res));
            }
        }
        else if (command == "editFile")
        {
            if (param.size() && !param[0].isDecimal)
            {
                //     String path = filepath(filename);
                auto file = FileFS.open(param[0].valS, FILE_READ);
                if (!file)
                {
                    SerialPrint("E", F("Telegram"), "Fail edit file: " + param[0].valS);
                    return {};
                }
                //  File file = LittleFS.open(param[0].valS, "r"); // /test.png
                // selectToMarkerLast(msg.text, "_")
                uint8_t res = _myBot->editFile(file, (FB_FileType)param[1].valD, selectToMarkerLast(param[0].valS, "/"), _myBot->lastBotMsg(), _chatID);
                file.close();
                SerialPrint("i", F("Telegram"), "chat ID: " + _chatID + ", editFile: " + param[0].valS + " res: " + String(res));
            }
        }
        else if (command == "btnMenu")
        {
            mapBtnMenu[param[0].valS] = new ButtonMenu;
            if (param.size() == 3) // btnMenu("Name", message, getId);
            {
                // if (IoTItems.find(param[2].valS) != IoTItems.end())
                //  {
                mapBtnMenu[param[0].valS]->message = param[1].valS;
                mapBtnMenu[param[0].valS]->getId = param[2].valS;
                // mapBtnMenu[param[0].valS] = btn;
                SerialPrint("i", F("Telegram"), "add button menu: " + param[0].valS + ", get id: " + param[2].valS);
                // }
            }
            else if (param.size() == 5) // btnMenu("Name", message, getId, setId, value);
            {
                //  if (IoTItems.find(param[2].valS) != IoTItems.end())
                //  {
                mapBtnMenu[param[0].valS]->message = param[1].valS;
                mapBtnMenu[param[0].valS]->getId = param[2].valS;
                mapBtnMenu[param[0].valS]->setId = param[3].valS;
                mapBtnMenu[param[0].valS]->value = param[4].valS;
                // mapBtnMenu[param[0].valS] = btn;
                SerialPrint("i", F("Telegram"), "add button menu: " + param[0].valS + ",get id: " + param[2].valS + ",set id: " + param[3].valS + "=" + param[4].valS);
                //  }
            }
        }
        else if (command == "showMenu")
        {
            String out;
            // перебирвем весь мап и строим меню
            uint8_t cnt = 0;
            for (auto it = mapBtnMenu.begin(); it != mapBtnMenu.end(); it++)
            {
                cnt++;
                if (cnt % 2 == 0)
                {
                    out += " \t " + it->first;
                }
                else
                {
                    if (it == mapBtnMenu.begin())
                        out = it->first;
                    else
                        out += " \n " + it->first;
                }
            }
            _myBot->showMenuText("Menu", out, true);
            SerialPrint("i", F("Telegram"), "chat ID: " + _chatID + ", show menu: \n" + out);
        }
        else if (command == "btnInline")
        {
            mapBtnInline[param[0].valS] = new ButtonMenu;
            if (param.size() == 3) // btnMenu("Name", message, getId);
            {
                // if (IoTItems.find(param[2].valS) != IoTItems.end())
                //  {
                mapBtnInline[param[0].valS]->message = param[1].valS;
                mapBtnInline[param[0].valS]->getId = param[2].valS;
                // mapBtnMenu[param[0].valS] = btn;
                SerialPrint("i", "Telegram", "add button inline: " + param[0].valS + ", get id: " + param[2].valS);
                // }
            }
            else if (param.size() == 5) // btnMenu("Name", message, getId, setId, value);
            {
                //  if (IoTItems.find(param[2].valS) != IoTItems.end())
                //  {
                mapBtnInline[param[0].valS]->message = param[1].valS;
                mapBtnInline[param[0].valS]->getId = param[2].valS;
                mapBtnInline[param[0].valS]->setId = param[3].valS;
                mapBtnInline[param[0].valS]->value = param[4].valS;
                // mapBtnMenu[param[0].valS] = btn;
                SerialPrint("i", "Telegram", "add button inline: " + param[0].valS + ",get id: " + param[2].valS + ",set id: " + param[3].valS + "=" + param[4].valS);
                //  }
            }
        }
        else if (command == "showInline")
        {
            String out;
            // перебирвем весь мап и строим меню
            uint8_t cnt = 0;
            for (auto it = mapBtnInline.begin(); it != mapBtnInline.end(); it++)
            {
                cnt++;
                if (cnt % 2 == 0)
                {
                    out += " \t " + it->first;
                }
                else
                {
                    if (it == mapBtnInline.begin())
                        out = it->first;
                    else
                        out += " \n " + it->first;
                }
            }
            _myBot->inlineMenu("inline_menu", out);
            SerialPrint("i", F("Telegram"), "chat ID: " + _chatID + ", show inline: \n" + out);
        }
        else if (command == "clearInline")
        {
            clearMapMenuInline();
            SerialPrint("i", F("Telegram"), "chat ID: " + _chatID + ", clear inline menu ");
        }
        else if (command == "clearMenu")
        {
            clearMapMenu();
            SerialPrint("i", F("Telegram"), "chat ID: " + _chatID + ", clear menu ");
        }
        else if (command == "closeMenu")
        {
            _myBot->closeMenu();
            SerialPrint("i", F("Telegram"), "chat ID: " + _chatID + ", closeMenu ");
        }
        return {};
    }

    //=============================================================================
    //=================== Обработка сообщений из чата =============================
    //=============================================================================
    void static telegramMsgParse(FB_msg &msg)
    {
        static String OTAfilepath = "";
        static uint8_t typeOTA = 0;
        //     FB_msg msg;
        SerialPrint("i", F("Telegram"), "chat ID: " + msg.chatID + ", msg: " + msg.text);
        //  _myBot->setChatID(_chatID);
        if (_autos)
        {
            _chatID = msg.chatID;
        }
        // -------------- Обработка сообщения об откате --------------
        // -------------------------------------------------------------------------
        if (msg.text.indexOf("/rollback") != -1 && msg.chatID == _chatID)
        {
#ifdef ESP32
            _myBot->inlineMenu("Вы уверены, что хотите откатить прошивку? " + jsonReadStr(settingsFlashJson, F("name")) + " \n OTA_roll", F("Rollback \t Cancel"));
#elif ESP8266
            SerialPrint("E", F("Update"), F("Откат OTA не поддерживается на esp8266!"));
            _myBot->sendMessage("Откат OTA не поддерживается на esp8266!", _chatID);
#endif
        }
        else if (msg.text.indexOf("OTA_roll") != -1)
        {
#ifdef ESP32
            // удаляем последнее сообщение от бота
            _myBot->deleteMessage(_myBot->lastBotMsg());
            if (msg.data.indexOf("Rollback") != -1)
            {
                if (Update.canRollBack())
                {
                    fl_rollback = true;
                }
                else
                {
                    SerialPrint("E", F("Update"), F("Откат OTA не возможен!"));
                    _myBot->sendMessage("Откат OTA не возможен!", _chatID);
                }
            }
#endif
        }
        // -------------- Обработка файлов *.bin для прошивки по OTA --------------
        // -------------------------------------------------------------------------
        else if (msg.OTA && _resolveOTA && msg.chatID == _chatID)
        {
            // _myBot->editMessage(_myBot->lastUsrMsg(), "firmware...", _chatID);
            OTAfilepath = msg.fileUrl;
            if (msg.fileName.indexOf("littlefs") != -1)
                typeOTA = FB_SPIFFS;
            else if (msg.fileName.indexOf("firmware") != -1)
                typeOTA = FB_FIRMWARE;
            else
                SerialPrint("E", F("Update"), "Unknown file: " + msg.fileName);
            _myBot->inlineMenu("Вы уверены, что хотите прошить плату? " + jsonReadStr(settingsFlashJson, F("name")) + "\n OTA_firmware", F("Firmware \t Cancel"));
        }
        else if (msg.text.indexOf("OTA_firmware") != -1)
        {
            // удаляем последнее сообщение от бота
            _myBot->deleteMessage(_myBot->lastBotMsg());
            if (msg.data.indexOf("Firmware") != -1)
            {
                putUserDataToRam();
                if (typeOTA == FB_SPIFFS)
                {
                    if (_updateFS((String *)&OTAfilepath) == 1)
                    {
                        saveUserDataToFlash();
                        SerialPrint("!!!", F("Update"), "Start upgrade FS... ");
                    }
                    else
                    {
                        SerialPrint("E", F("Update"), F("FS Path error"));
                        _myBot->sendMessage("FS Path error!", _chatID);
                    }
                }
                else if (typeOTA == FB_FIRMWARE)
                {
                    if (_update((String *)&OTAfilepath) == 1)
                    {
                        saveUserDataToFlash();
                        SerialPrint("!!!", F("Update"), "Start upgrade BUILD... ");
                    }
                    else
                    {
                        SerialPrint("E", F("Update"), F("Build Path error"));
                        _myBot->sendMessage("Build Path error!", _chatID);
                    }
                }
            }
            OTAfilepath = "";
            typeOTA = 0;
        }
        //  -------------- обработка файлов загруженных пользователем --------------
        // -------------------------------------------------------------------------
        else if (msg.isFile)
        {
            if (msg.fileName.endsWith(F(".tft")) && msg.chatID == _chatID)
            {
                OTAfilepath = msg.fileUrl;
                _myBot->inlineMenu("Хотите прошить экран Nextion? На esp " + jsonReadStr(settingsFlashJson, F("name")) + "\n Next_firmware", F("Firmware \t Cancel"));
            }
            else if (msg.text.indexOf("download") != -1 && msg.chatID == _chatID)
            {
                downloadFile(msg);
            }
        }
        else if (msg.text.indexOf("Next_firmware") != -1)
        {
            // удаляем последнее сообщение от бота
            _myBot->deleteMessage(_myBot->lastBotMsg());
            if (msg.data.indexOf("Firmware") != -1 && OTAfilepath != "")
            {
                for (std::list<IoTItem *>::iterator it = IoTItems.begin(); it != IoTItems.end(); ++it)
                {
                    if ((*it)->getSubtype() == "NextionUpload" || (*it)->getSubtype() == "Nextion")
                    {
                        _myBot->sendMessage("Nextion firmware ...", _chatID);
                        (*it)->uploadNextionTlgrm(OTAfilepath);
                    }
                }
                OTAfilepath = "";
            }
        }
        // -------------- Обработка кнопок меню созданного в сценарии --------------
        // -------------------------------------------------------------------------
        else if (auto search = mapBtnMenu.find(msg.text); search != mapBtnMenu.end())
        {
            String outMsg;
            outMsg = mapBtnMenu[msg.text]->message;
            if (mapBtnMenu[msg.text]->getId != "")
            {
                IoTItem *item = findIoTItem(mapBtnMenu[msg.text]->getId);
                if (item)
                {
                    outMsg += ": " + item->getValue();
                }
            }
            if (mapBtnMenu[msg.text]->setId != "")
            {
                // outMsg += ", " + mapBtnMenu[msg.text]->setId + "=" + mapBtnMenu[msg.text]->value;
                generateOrder(mapBtnMenu[msg.text]->setId, mapBtnMenu[msg.text]->value);
            }
            SerialPrint("i", F("Telegram"), "chat ID: " + _chatID + ", msg: " + String(outMsg));
            _myBot->sendMessage(outMsg, _chatID);
        }

        // --------------  Обработка нажатия на пользовательском инлайн меню --------------
        // -------------------------------------------------------------------------
        else if (msg.text.indexOf("inline_menu") != -1)
        {
            SerialPrint("i", F("Telegram"), "chat ID: " + _chatID + ", data: " + String(msg.data));
            if (auto search = mapBtnInline.find(msg.data); search != mapBtnInline.end())
            {

                String outMsg;
                outMsg = mapBtnInline[msg.data]->message;
                if (mapBtnInline[msg.data]->getId != "")
                {
                    IoTItem *item = findIoTItem(mapBtnInline[msg.data]->getId);
                    if (item)
                    {
                        outMsg += ": " + item->getValue();
                        // SerialPrint("i", F("Telegram"), "chat ID: " + _chatID + ", msg: " + String(msg.data));
                    }
                }
                if (mapBtnInline[msg.data]->setId != "")
                {
                    // outMsg += ", " + mapBtnInline[msg.data]->setId + "=" + mapBtnInline[msg.data]->value;
                    generateOrder(mapBtnInline[msg.data]->setId, mapBtnInline[msg.data]->value);
                }
                SerialPrint("i", F("Telegram"), "chat ID: " + _chatID + ", msg: " + String(outMsg));
                _myBot->sendMessage(outMsg, _chatID);
            }
        }
        // -------------- Вызов (повторный вызов) меню созданного в сценарии --------------
        // -------------------------------------------------------------------------
        else if (msg.text.indexOf("menu") != -1)
        {
            String out;
            // перебирвем весь мап и строим меню
            uint8_t cnt = 0;
            for (auto it = mapBtnMenu.begin(); it != mapBtnMenu.end(); it++)
            {
                cnt++;
                if (cnt % 2 == 0)
                {
                    out += " \t " + it->first;
                }
                else
                {
                    if (it == mapBtnMenu.begin())
                        out = it->first;
                    else
                        out += " \n " + it->first;
                }
            }
            _myBot->showMenuText("Menu", out, true);
            SerialPrint("i", F("Telegram"), "chat ID: " + _chatID + ", show menu: \n" + out);
        }
        // --------------  вывод нижнего меню всех юнитов --------------
        // -------------------------------------------------------------------------
        else if (msg.text.indexOf("allMenu") != -1)
        {
            // String list = returnListOfParams();
            String out;
            // std::vector<float> vctr;
            uint8_t cnt = 0;
            for (std::list<IoTItem *>::iterator it = IoTItems.begin(); it != IoTItems.end(); ++it)
            {
                if ((*it)->iAmLocal)
                {
                    cnt++;
                    if (cnt % 2 == 0)
                    {
                        out += " \t get_" + (*it)->getID();
                    }
                    else
                    {
                        if (it == IoTItems.begin())
                            out = "get_" + (*it)->getID();
                        else
                            out += " \n get_" + (*it)->getID();
                    }
                }
            }
            _myBot->showMenuText("select Id", out, true);
            SerialPrint("i", F("Telegram"), "chat ID: " + _chatID + "\n" + out);
            //    _myBot->sendMessage(CharPlot<LINE_X2>(&vctr[0], vctr.size(), 5),  _chatID);
            //    SerialPrint("<-", F("Telegram"), CharPlot<LINE_X2>(&vctr[0], vctr.size(), 10));
        }

        //  -------------- Обработка нажатия в инлайн меню all (всех юнитов) --------------
        // -------------------------------------------------------------------------
        else if (msg.text.indexOf("all_inline") != -1 && msg.data.indexOf("get") != -1)
        {
            String out = deleteBeforeDelimiter(msg.data, "_");
            IoTItem *item = findIoTItem(out);
            if (item)
            {
                _myBot->sendMessage(item->getID() + ": " + item->getValue(), _chatID);
                SerialPrint("i", F("Telegram"), "chat ID: " + _chatID + ", msg: " + out);
            }
        }
        // --------------  вывод инлайн меню всех юнитов --------------
        // -------------------------------------------------------------------------
        else if (msg.text.indexOf("/all") != -1)
        {
            // String list = returnListOfParams();
            String out;
            // std::vector<float> vctr;
            uint8_t cnt = 0;
            for (std::list<IoTItem *>::iterator it = IoTItems.begin(); it != IoTItems.end(); ++it)
            {
                if ((*it)->iAmLocal)
                {
                    cnt++;
                    if (cnt % 2 == 0)
                    {
                        out += " \t get_" + (*it)->getID();
                    }
                    else
                    {
                        if (it == IoTItems.begin())
                            out = "get_" + (*it)->getID();
                        else
                            out += " \n get_" + (*it)->getID();
                    }
                }
            }
            _myBot->inlineMenu("all_inline", out);
            SerialPrint("i", F("Telegram"), "chat ID: " + _chatID + "\n" + out);
        }
        //  -------------- обработка команды /set_ID_VALUE --------------
        // -------------------------------------------------------------------------
        else if (msg.text.indexOf("/set") != -1)
        {
            msg.text = deleteBeforeDelimiter(msg.text, "_");
            generateOrder(selectToMarker(msg.text, "_"), selectToMarkerLast(msg.text, "_"));
            _myBot->replyMessage("order done", msg.messageID, _chatID);
            SerialPrint("i", F("Telegram"), "chat ID: " + _chatID + ", msg: " + String(msg.text));
        }
        // --------------  обработка команды /get_ID --------------
        // -------------------------------------------------------------------------
        else if (msg.text.indexOf("/get") != -1)
        {
            msg.text = deleteBeforeDelimiter(msg.text, "_");
            IoTItem *item = findIoTItem(msg.text);
            if (item)
            {
                _myBot->replyMessage(item->getValue(), msg.messageID, _chatID);
                SerialPrint("i", F("Telegram"), "chat ID: " + _chatID + ", msg: " + String(msg.text));
            }
        }
        //  -------------- обработка запроса пользователя на скачивание файла --------------
        // -------------------------------------------------------------------------
        else if (msg.text.indexOf("/file") != -1 && msg.chatID == _chatID)
        {
            if (msg.text.indexOf("file_type") != -1)
            {
                _myBot->sendMessage("Support file type download: \n 0-foto \n 1-audio \n 2-doc \n 3-video \n 4-gif \n 5-voice", _chatID);
            }
            else
            {
                msg.text = deleteBeforeDelimiter(msg.text, "_");
                SerialPrint("i", F("Telegram"), "chat ID: " + _chatID + ", get file: " + String(msg.text));
                auto file = FileFS.open(selectToMarker(msg.text, "_"), FILE_READ); // /test.png
                if (!file)
                {
                    SerialPrint("E", F("Telegram"), "Fail send file: " + selectToMarker(msg.text, "_"));
                    return;
                }
                int type = atoi(selectToMarkerLast(msg.text, "_").c_str());
                _myBot->sendFile(file, (FB_FileType)type, selectToMarker(msg.text, "_"), _chatID);
                file.close();
            }
        }
        //  -------------- обработка остальных команд --------------
        // -------------------------------------------------------------------------
        else if (msg.text.indexOf("/help") != -1)
        {
            if (msg.text.indexOf("/helpDebug") != -1)
            {
                _myBot->sendMessage("В папке toolchchain с которым собирались (Для esp32 например %%USERPROFILE%/.platformio/packages/toolchain-xtensa-esp32@8.4.0+2021r2-patch5/bin) из командной строки Windows (cmd) запустить файл xtensa-esp32-elf-addr2line.exe \nС параметрами -pfiaC -e Путь_к_файлу/firmware.elf Стэк_адресов", _chatID);
            }
            else
            {
                _myBot->sendMessage("ID: " + chipId, _chatID);
                _myBot->sendMessage("chatID: " + _chatID, _chatID);
                _myBot->sendMessage("Command: /help - this text \n /all - inline menu get all values \n /allMenu - bottom menu get all values \n /menu - bottom USER menu from scenario \n /get_id - get value by ID \n /set_id_value - set value in ID \n /file_/path/name_type - take file from esp \n /file_type - support file type \n /reboot - reboot esp \n\n send file and write download - \"download\" file to esp \n\n send *.tft file - flash Nextion \n\n send firmware.bin or littltfs.bin - firmware ESP ", _chatID);
            }
        }
        else if (msg.text.indexOf("/reboot") != -1)
        {
            _myBot->inlineMenu("Перезагрузить esp " + jsonReadStr(settingsFlashJson, F("name")) + "\n esp_reboot", F("Reboot \t Cancel"));
        }
        else if (msg.text.indexOf("esp_reboot") != -1)
        {
            // удаляем последнее сообщение от бота
            _myBot->deleteMessage(_myBot->lastBotMsg());
            if (msg.data.indexOf("Reboot") != -1)
            {
                fl_reboot = true;
            }
        }
        else if (msg.text.indexOf("/") != -1)
        {
            _myBot->sendMessage("Wrong order, use /help", _chatID);
        }
    }

    void sendTelegramMsg(bool often, String msg)
    {
        if (!isNetworkActive())
            return;
        if (often)
        {
            _myBot->sendMessage(msg, _chatID);
            SerialPrint("i", F("Telegram"), "chat ID: " + _chatID + ", msg: " + msg);
        }
        else
        {
            if (_prevMsg != msg)
            {
                _prevMsg = msg;
                _myBot->sendMessage(msg, _chatID);
                SerialPrint("i", F("Telegram"), "chat ID: " + _chatID + ", msg: " + msg);
            }
        }
    }

    void sendFoto(uint8_t *buf, uint32_t length, const String &name)
    {
        if (!isNetworkActive())
            return;
        _myBot->sendFile(buf, length, FB_PHOTO, name, _chatID);
        SerialPrint("i", F("Telegram"), "chat ID: " + _chatID + ", send foto from esp-cam");
    }

    void editFoto(uint8_t *buf, uint32_t length, const String &name)
    {
        if (!isNetworkActive())
            return;
        _myBot->editFile(buf, length, FB_PHOTO, name, _myBot->lastBotMsg(), _chatID);
        SerialPrint("i", F("Telegram"), "chat ID: " + _chatID + ", edit foto from esp-cam");
    }

    int static downloadFile(FB_msg &msg)
    {
        int _size = 0;
        String path = '/' + msg.fileName;          // вида /filename.xxx
        auto file = FileFS.open(path, FILE_WRITE); // открываем для записи
                                                   //        _myBot->sendMessage("Downloading from: " + _chatID + ", file: " + String(msg.fileName), _chatID);
        if (file)
        { // файл открылся/создался
            HTTPClient http;

#ifdef ESP8266 // esp8266 требует SSl
            BearSSL::WiFiClientSecure client;
            client.setInsecure();
            http.begin(client, msg.fileUrl); // пингуем файл
#else                                        // esp32 сама умеет SSL
            http.begin(msg.fileUrl); // пингуем файл
#endif

            if (http.GET() == HTTP_CODE_OK)
            { // файл доступен
                // загружаем в память. Результат > 0 - успешно
                _size = http.writeToStream(&file);
            }
            http.end();   // закрываем соединение
            file.close(); // закрываем файл

            if (_size == 0)
            {
                SerialPrint("E", F("Telegram"), "download error file url: " + msg.fileUrl);
                _myBot->sendMessage(F("Download Fail"), _chatID);
            }
            else
            {
                SerialPrint("i", F("Telegram"), "download from: " + _chatID + ", file: " + msg.fileName + " size = " + String(_size) + " byte");
                _myBot->sendMessage("Download Ok, size = " + String(_size) + " byte", _chatID);
            }
        }
        else
        {
            SerialPrint("E", F("Telegram"), F("file write error"));
            _myBot->sendMessage(F("file write error"), _chatID);
        }
        return _size;
    }

    IoTItem *getTlgrmDriver()
    {
        return this;
    }
    void clearMapMenu()
    {
        for (auto it = mapBtnMenu.begin(); it != mapBtnMenu.end(); it++)
        {
            delete it->second;
        }
        mapBtnMenu.clear();
        for (auto it = mapBtnInline.begin(); it != mapBtnInline.end(); it++)
        {
            delete it->second;
        }
        mapBtnInline.clear();
    }
    void clearMapMenuInline()
    {
        for (auto it = mapBtnInline.begin(); it != mapBtnInline.end(); it++)
        {
            delete it->second;
        }
        mapBtnInline.clear();
    }

    // ===================== OTA =====================
    // ОТА обновление, вызывать внутри обработчика сообщения по флагу OTA
    uint8_t static _update(String *_file_ptr, __attribute__((unused)) uint8_t type = FB_FIRMWARE)
    {
#ifndef FB_NO_OTA
        if (!_file_ptr)
            return 8;
        uint8_t OTAflag = type;
        _myBot->sendMessage((type == FB_FIRMWARE) ? F("OTA firmware...") : F("OTA spiffs..."), _chatID);

#ifdef ESP8266
        ESPhttpUpdate.rebootOnUpdate(false);
#ifdef FB_DYNAMIC
        BearSSL::WiFiClientSecure client;
        client.setInsecure();
#endif
        if (OTAflag == FB_FIRMWARE)
            _OTAstate = ESPhttpUpdate.update(client, *_file_ptr);
        else if (OTAflag == FB_SPIFFS)
            _OTAstate = ESPhttpUpdate.updateFS(client, *_file_ptr);
#else
        WiFiClientSecure client;
        client.setInsecure();
        httpUpdate.rebootOnUpdate(false);
        if (OTAflag == FB_FIRMWARE)
            _OTAstate = httpUpdate.update(client, *_file_ptr);
        else if (OTAflag == FB_SPIFFS)
            _OTAstate = httpUpdate.updateSpiffs(client, *_file_ptr);
#endif
#endif
        return 1;
    }

    // ОТА обновление SPIFFS, вызывать внутри обработчика сообщения по флагу OTA
    uint8_t static _updateFS(String *_file_ptr)
    {
        return _update(_file_ptr, FB_SPIFFS);
    }

    ~Telegram_v2()
    {
        clearMapMenu();
        clearMapMenuInline();
        tlgrmItem = nullptr;
    };
};

void *getAPI_Telegram_v2(String subtype, String param)
{
    if (subtype == F("Telegram_v2"))
    {
        return new Telegram_v2(param);
    }
    else
    {
        return nullptr;
    }
}
