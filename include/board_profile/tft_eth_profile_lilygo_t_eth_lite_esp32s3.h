#pragma once

// TFT/ETH-Profil fuer LilyGO T-ETH-Lite-S3
// Diese Defines sind bewusst generisch benannt und koennen spaeter
// mit Setup-Defines (z. B. aus Setup216) abgeglichen/erweitert werden.

#define BOARD_PROFILE_NAME "lilygo_t_eth_lite_esp32s3"

// W5500 (SPI3_HOST) - verifiziert im aktuellen Projektstand
#define BOARD_ETH_SCK   10
#define BOARD_ETH_MISO  11
#define BOARD_ETH_MOSI  12
#define BOARD_ETH_CS     9
#define BOARD_ETH_INT   13
#define BOARD_ETH_RST   14
