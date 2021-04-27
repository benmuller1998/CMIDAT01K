/*
 * fs.h
 *
 *  Created on: 20 jun. 2018
 *      Author: VersD
 */
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <stdbool.h>

#include <ti/drivers/net/wifi/slnetifwifi.h>
#include "uart_term.h"

#ifndef FS_H_
#define FS_H_

int    leesWachtwoord(char * ww, bool vraagOmWachtwoord);
int schrijfWachtwoord();

#define BESTANDSNAAM "wifibestand.bin"


#endif /* FS_H_ */
