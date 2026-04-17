#pragma once

// Boardprofil-Auswahl fuer TFT/ETH-bezogene Defines.
// Aktivierung erfolgt ueber PlatformIO-Buildflags, z.B.:
//   -DLILYGO_T_ETH_LITE_ESP32S3
//   -DLILYGO_T_ETH_LITE_ESP32

#if defined(LILYGO_T_ETH_LITE_ESP32S3)
  #include "tft_eth_profile_lilygo_t_eth_lite_esp32s3.h"
#elif defined(LILYGO_T_ETH_LITE_ESP32)
  #include "tft_eth_profile_lilygo_t_eth_lite_esp32.h"
#else
  #error "Kein unterstuetztes Boardprofil gesetzt. Definiere LILYGO_T_ETH_LITE_ESP32S3 oder LILYGO_T_ETH_LITE_ESP32."
#endif
