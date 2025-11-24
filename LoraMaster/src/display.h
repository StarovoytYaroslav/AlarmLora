#pragma once

#include <heltec-eink-modules.h>

class Display{
private:
    EInkDisplay_WirelessPaperV1_1 display;
public:
    void setupDisplay();
    void displayTest();
    void printWiFiOnDisplay(const Printable&);
};
