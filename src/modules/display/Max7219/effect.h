#pragma once
#include <MD_Parola.h>

textEffect_t effect[] =
    {
        PA_NO_EFFECT,         // 0 Используется в качестве заполнителя места, не выполняет никаких операций.
        PA_PRINT,             // 1 Просто появляется текст (напечатанный)
        PA_SCROLL_UP,         // 2 Текст прокручивается вверх по дисплею.
        PA_SCROLL_DOWN,       // 3 Текст прокручивается вниз по дисплею.
        PA_SCROLL_LEFT,       // 4 Текст на дисплее прокручивается справа налево.
        PA_SCROLL_RIGHT,      // 5 Текст на дисплее прокручивается слева направо.
        PA_SPRITE,            // 6 Текст вводится и завершается с помощью пользовательского спрайта.
        PA_SLICE,             // 7 Текст вводится и выходит из фрагмента (столбца) по очереди справа.
        PA_MESH,              // 8 Текст вводится и выводится столбцами, перемещающимися в альтернативном направлении (U / D)
        PA_FADE,              // 9 Ввод текста и выход из него осуществляется путем затухания от / до 0 и настройки интенсивности
        PA_DISSOLVE,          // 10 Текст перемещается с одного дисплея на другой.
        PA_BLINDS,            // 11 Заменен текст за вертикальными жалюзи.
        PA_RANDOM,            // 12 Текст вводится и выводится в виде случайных точек.
        PA_WIPE,              // 13 Текст появляется / исчезает по одному столбцу за раз, выглядит так, будто его то стирают, то выключают.
        PA_WIPE_CURSOR,       // 14 Перед внесением изменений протрите их световой полосой.
        PA_SCAN_HORIZ,        // 15 Сканируйте светодиодные колонки по одной, затем они появляются / исчезают в конце.
        PA_SCAN_HORIZX,       // 16 Сканируйте пустой столбец текста по одному столбцу за раз, затем появляется / исчезает в конце.
        PA_SCAN_VERT,         // 17 Сканируйте ряд светодиодов по одному, затем появляйтесь / исчезайте в конце.
        PA_SCAN_VERTX,        // 18 Сканируйте пустую строку текста по одной строке за раз, затем появляется / исчезает в конце.
        PA_OPENING,           // 19 Появляются и исчезают от центра дисплея к концам.
        PA_OPENING_CURSOR,    // 20 ОТКРЫТИЕ со световыми полосами перед изменением.
        PA_CLOSING,           // 21 Появляются и исчезают с краев дисплея, ближе к середине.
        PA_CLOSING_CURSOR,    // 22 ЗАКРЫВАЕТСЯ световыми полосами перед изменением.
        PA_SCROLL_UP_LEFT,    // 23 Текст перемещается по диагонали вверх и влево (северо-восток)
        PA_SCROLL_UP_RIGHT,   // 24 Текст перемещается по диагонали вверх и вправо (северо-запад)
        PA_SCROLL_DOWN_LEFT,  // 25 Текст перемещается по диагонали вниз и влево (юго-восток)
        PA_SCROLL_DOWN_RIGHT, // 26	Текст перемещается по диагонали вниз и вправо (северо-запад)
        PA_GROW_UP,           // 27 Текст увеличивается снизу вверх и сжимается сверху вниз.
        PA_GROW_DOWN,         // 28 Текст увеличивается сверху вниз и сжимается снизу вверх.
};

textPosition_t setTextAlignment [] =
{
       PA_LEFT,             // выравнивание слева
       PA_CENTER,           // выравнивание по центу
       PA_RIGHT,            // выравнивание справа
};