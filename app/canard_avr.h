/*
 * Copyright (c) 2016 UAVCAN Team
 *
 * Distributed under the MIT License, available in the file LICENSE.
 *
 */

#ifndef CANARD_AVR_H
#define CANARD_AVR_H

#include "canard.h"

#ifdef __cplusplus
extern "C"
{
#endif

    // typedef struct
    // {

    //     uint32_t id; //!< ID der Nachricht (11 oder 29 Bit)
    //     struct
    //     {
    //         int rtr : 1;      //!< Remote-Transmit-Request-Frame?
    //         int extended : 1; //!< extended ID?
    //     } flags;

    //     uint8_t length;  //!< Anzahl der Datenbytes
    //     uint8_t data[8]; //!< Die Daten der CAN Nachricht

    //     //uint16_t timestamp;

    // } can_t;

    /**
 * \ingroup   communication
 * \defgroup  canard_avr_interface Libcanard CAN Interface for AVR microcontrollers
 * \brief     Interface for Libcanard CAN interaction with AVR microcontrollers
 *
 * \author    Matthias Renner <rennerm@ethz.ch>
 * \author    ETH Zuerich Robotics Systems Lab (http://http://www.rsl.ethz.ch/)
 *
 * \version   0.1
 */

    /**
 * @ingroup canard_avr_interface
 * @brief Initialize CAN interface on AVR microcontroller.
 * @warning Enables interrupts!
 *
 * @param [in] bitrate  Set CAN bitrate (bits/sec.)
 *
 * @retval     0        Successfully initialized.
 */
    int canardAVRInit(uint32_t bitrate);

    /**
 * @ingroup canard_avr_interface
 * @brief Deinitialize CAN interface on AVR microcontroller.
 * @warning Not implemented
 *
 * @retval 1     Initialisation successful
 * @retval -1    Error, bitrate not supported
 */
    int canardAVRClose(void);

    /**
 * @ingroup canard_avr_interface
 * @brief Transmits a CanardFrame to the CAN device.
 *
 * @param [in] frame  Canard CAN frame which contains the data to send
 *
 * @retval     0      No CAN send buffer free
 * @retval     -1     Error, data could not be sent
 * @retval     1      Data sent successful
 */
    int canardAVRTransmit(const CanardFrame *frame);

    /**
 * @ingroup canard_avr_interface
 * @brief Receives a CanardFrame from the CAN device.
 *
 * @param [out] out_frame  Canard CAN frame which contains data received
 *
 * @retval      0          No new CAN data to be read
 * @retval      -1         Error, data could not be read
 * @retval      1          Data read successful
 */
    int canardAVRReceive(CanardFrame *out_frame);

    /**
 * @ingroup canard_avr_interface
 * @brief Set hardware acceptance filters for specific node ID
 *
 * @param [in] id  node ID for hardware filter
 *
 * @retval      -1         Error, filters could no be set
 * @retval      1          Set filter successful
 */
    int canardAVRConfigureAcceptanceFilters(uint8_t node_id);

#ifdef __cplusplus
}
#endif

#endif
