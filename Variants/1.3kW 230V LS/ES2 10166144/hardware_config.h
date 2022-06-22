/**
  ********************************************************************************************************************************
  * @file    hardware_config.h 
  * @author  Roel Pantonial
  * @brief   Header of port and pin assignments
  * @details This file is used to align board with electromechanical parameters
  ********************************************************************************************************************************
  */

/* definition for the installation of EEPROM */
#define eepromInstall 

#define HARDWARE_VERSION_BULLRUNNER 0
#define HARDWARE_VERSION_1p3KW 1
#define HARDWARE_VERSION_4p5KW 2
#define HARDWARE_VERSION_8KW 3
#define HARDWARE_VERSION_1p3KW_REVE_AND_BELOW 4
#define HARDWARE_VERSION_1p3KW_REVE_AND_BELOW_EXT_CRYSTAL 5 // 1.3kW HW rev E and below with an external crystal

#define HARDWARE_VERSION HARDWARE_VERSION_1p3KW // set this to choose the HW version

// GPIO Input Ports/Pins
#if ((HARDWARE_VERSION == HARDWARE_VERSION_1p3KW) || (HARDWARE_VERSION == HARDWARE_VERSION_1p3KW_REVE_AND_BELOW))
#define CLOCK_CHECK 64
#endif

#if ((HARDWARE_VERSION == HARDWARE_VERSION_4p5KW) || (HARDWARE_VERSION == HARDWARE_VERSION_1p3KW_REVE_AND_BELOW_EXT_CRYSTAL) )
#define CLOCK_CHECK 72
#endif

#if HARDWARE_VERSION == HARDWARE_VERSION_8KW
#define CLOCK_CHECK 72
#endif
