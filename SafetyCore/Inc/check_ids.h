/**
  *******************************************************************************************************************************************
  * @file    check_ids.h 
  * @author  Justin Moon
  * @version V1.0
  * @date    22-Jun-2021
  * @brief   Header containing Check IDs for each test

  *******************************************************************************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef CHECK_IDS_H
    #define CHECK_IDS_H

    #ifdef __cplusplus
extern "C" {
    #endif
 
    #define ADC_CHECK_ID        (0)
    #define REGISTER_CHECK_ID   (1)
    #define CLOCK_CHECK_ID      (2)
    #define RAM_CHECK_ID        (3)  
    #define ROM_CHECK_ID        (4)

    #ifdef __cplusplus
}
    #endif

#endif // CHECK_IDS_H