/*
 * fs.c
 *
 *  Created on: 20 jun. 2018
 *      Author: VersD
 */

#include "bestanden.h"

static _i8 wachtwoord[64]; // Buffer om een wifi wachtwoord in op te slaan
static bool nieuwWachtwoord = false;

static _u32 g_tokens[4] = {0xAB34CD78, 0, 0, 0};
static _u32 g_accessMode = SL_FS_CREATE_SECURE | SL_FS_CREATE_NOSIGNATURE |
                           SL_FS_CREATE_VENDOR_TOKEN | SL_FS_CREATE_STATIC_TOKEN;

static int getFileInfo();

int leesWachtwoord(char *ww, bool vraagOmWachtwoord)
{
    // Wachtwoord ophalen of inlezen
    int Status=0;

    if ((getFileInfo() == SL_ERROR_FS_FILE_NOT_EXISTS) || vraagOmWachtwoord == true) // Bestand bestaat niet, aanmaken, dit is eenmalig
    {
        UART_PRINT("\r\nVoer jouw wifi-wachtwoord in:\r\n");

        char c;
        int i = 0;
        while ((c = getch()) != '\n' && c != '\r' && i < 63) // Zolang geen enter binnenkomt
        {
            if (c == '\b')
            {
                if (i > 0)
                {
                    i--;
                    UART_PRINT("\b");
                }
            }
            else
            {
                ww[i++] = c;
                UART_PRINT("*");
            }
        }
        ww[i] = '\0';
        UART_PRINT("\r\n");
        nieuwWachtwoord = true;

    }
    else // Bestand bestaat wel, uitlezen.
    {
        _i32 bestand = sl_FsOpen(BESTANDSNAAM, SL_FS_READ, &g_tokens[SL_FS_TOKEN_READ_ONLY]);
        if (bestand < 0)
        {
            UART_PRINT("Fout bij het openen van het wachtwoordbestand: %d\r\n",bestand);
        }
        else
        {
            sl_FsRead(bestand, 0, (unsigned char *)ww, 64);
            Status = sl_FsClose(bestand, 0, 0, 0);
        }
    }
    strcpy((char *)wachtwoord, ww);
    return Status;
}

int schrijfWachtwoord()
{
    int Status=0;
    // Als wachtwoord nog niet is opgeslagen alsnog doen
    if ((getFileInfo() == SL_ERROR_FS_FILE_NOT_EXISTS) || nieuwWachtwoord == true)
    {
        _i32 bestand = sl_FsOpen(BESTANDSNAAM,
                                 (SL_FS_CREATE | SL_FS_CREATE_FAILSAFE | SL_FS_OVERWRITE | SL_FS_CREATE_MAX_SIZE(65) | g_accessMode),
                                 &g_tokens[SL_FS_TOKEN_MASTER]);

        if (bestand < 0)
        {
            UART_PRINT("Fout bij het aanmaken van het wachtwoordbestand: %d\r\n",bestand);
        }
        else
        {
            sl_FsWrite(bestand, 0, (unsigned char *)wachtwoord, strlen((char *)wachtwoord));
            Status += sl_FsClose(bestand, 0, 0, 0);
            Status += getFileInfo(); //update tokens
            if (Status == 0)
            {
                UART_PRINT("Wachtwoord opgeslagen.\r\n");
            }
        }
    }
    return Status;
}

// Call this function after the secure file is created
static int getFileInfo()
{
    SlFsFileInfo_t fileInfo;
    int RetVal;

    RetVal = sl_FsGetInfo(BESTANDSNAAM, g_tokens[SL_FS_TOKEN_MASTER], &fileInfo);
    if (0 == RetVal)
    {
        g_tokens[SL_FS_TOKEN_READ_ONLY] = fileInfo.Token[SL_FS_TOKEN_READ_ONLY];
        g_tokens[SL_FS_TOKEN_WRITE_ONLY] = fileInfo.Token[SL_FS_TOKEN_WRITE_ONLY];
        g_tokens[SL_FS_TOKEN_WRITE_READ] = fileInfo.Token[SL_FS_TOKEN_WRITE_READ];
    }
    return RetVal;
}
