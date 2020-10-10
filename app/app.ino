#include <string.h>
#include <time.h>
#include <stdio.h>
#include <SPI.h>


#include "canard.h"
// #include "libcanard/canard_avr.h"

#include <mcp_can_dfs.h>
#include <mcp_can.h>

/////////////////////////////
#define CANARD_CAN_EXT_ID_MASK 0x1FFFFFFFU
#define CANARD_CAN_FRAME_EFF (1UL << 31U) ///< Extended frame format

typedef struct
{

    uint32_t id; //!< ID der Nachricht (11 oder 29 Bit)
    struct
    {
        int rtr : 1;      //!< Remote-Transmit-Request-Frame?
        int extended : 1; //!< extended ID?
    } flags;

    uint8_t length;  //!< Anzahl der Datenbytes
    uint8_t data[8]; //!< Die Daten der CAN Nachricht

    //uint16_t timestamp;

} can_t;

////////////////////////////////

static const uint16_t HeartbeatSubjectID = 32085;

CanardInstance canard;
const time_t boot_ts = 0;
time_t next_1hz_at = boot_ts;

const int led_pin = PB7;
const uint16_t t4_load = 0;
const uint16_t t4_comp = 15625;

boolean pacemaker = false;

const int SPI_CS_PIN = 53;
MCP_CAN CAN(SPI_CS_PIN);

void setup()
{
    Serial.begin(115200);

    while (CAN_OK != CAN.begin(CAN_500KBPS)) // init can bus : baudrate = 500k
    {
        Serial.println("CAN BUS Shield init fail");
        Serial.println(" Init CAN BUS Shield again");
        delay(100);
    }
    Serial.println("CAN BUS Shield init ok!");

    // Initialize the node with a static node-ID as specified in the command-line arguments.
    canard = canardInit(&canardAllocate, &canardFree);
    canard.mtu_bytes = CANARD_MTU_CAN_CLASSIC; // Do not use CAN FD to enhance compatibility.
    canard.node_id = (CanardNodeID)42;

    setup_pacemaker();
}

void loop()
{
    // The main loop: publish messages and process service requests.

    if (pacemaker)
    {
        next_1hz_at = millis();
        publishHeartbeat(&canard, next_1hz_at);
        pacemaker = false;
    }

    // Transmit pending frames.
    const CanardFrame *txf = canardTxPeek(&canard);
    if (txf != NULL)
    {
        Serial.print("Transmit\n");
        Serial.flush();
    }
    while (txf != NULL)
    {
        //(void) socketcanPush(sock, txf, 0);  // Error handling not implemented
        canardAVRTransmit(txf);
        canardTxPop(&canard);
        free((void *)txf);
        txf = canardTxPeek(&canard);
    }
}

void setup_pacemaker()
{
    // Disable global interrupts
    cli();

    //Set LED pin to be output
    DDRB |= (1 << led_pin);

    // Reset Timer1 control Reg A and B
    TCCR4A = 0;
    TCCR4B = 0;

    // Set CTC mode
    TCCR4B &= ~(1 << WGM12);
    TCCR4B |= (1 << WGM12);

    // Set the prescaler of 1024
    TCCR4B |= (1 << CS12) | (1 << CS10);

    //Reset Timer4 and set compare value
    TCNT4 = t4_load;
    OCR4A = t4_comp;

    // Enable Timer4 compare interrupt
    TIMSK4 |= (1 << OCIE4A);

    //Enable global interrupts
    sei();
}

ISR(TIMER4_COMPA_vect)
{
    PORTB ^= (1 << led_pin);
    pacemaker = true;
}

static void publishHeartbeat(CanardInstance *const canard, const uint32_t uptime)
{
    static CanardTransferID transfer_id;
    const uint8_t payload[7] = {
        (uint8_t)(uptime >> 0U),
        (uint8_t)(uptime >> 8U),
        (uint8_t)(uptime >> 16U),
        (uint8_t)(uptime >> 24U),
        0,
        0,
        0,
    };
    const CanardTransfer transfer = {
        timestamp_usec : 0,
        priority : CanardPriorityNominal,
        transfer_kind : CanardTransferKindMessage,
        port_id : HeartbeatSubjectID,
        remote_node_id : CANARD_NODE_ID_UNSET,
        transfer_id : transfer_id,
        payload_size : sizeof(payload),
        payload : &payload[0]
    };
    ++transfer_id;
    (void)canardTxPush(canard, &transfer);
}

static void *canardAllocate(CanardInstance *const ins, const size_t amount)
{
    (void)ins;
    return malloc(amount);
}

static void canardFree(CanardInstance *const ins, void *const pointer)
{
    (void)ins;
    free(pointer);
}

int canardAVRTransmit(const CanardFrame *frame)
{
    // Not clear if this function is implemented //
    // const int poll_result = can_check_free_buffer();
    // if (poll_result <= 0)
    // {
    //     return 0;
    // }

    // can_t transmit_msg;
    // memset(&transmit_msg, 0, sizeof(transmit_msg));
    // transmit_msg.id = frame->extended_can_id & CANARD_CAN_EXT_ID_MASK;
    // transmit_msg.length = frame->payload_size;
    // transmit_msg.flags.extended = (frame->extended_can_id & CANARD_CAN_FRAME_EFF) != 0;
    // memcpy(transmit_msg.data, frame->payload, frame->payload_size);

    // const uint8_t res = can_send_message(&transmit_msg);
    // if (res <= 0)
    // {
    //     return -1;
    // }

    CAN.sendMsgBuf(frame->extended_can_id, 1, frame->payload_size, frame->payload); 
    //send out the message 'stmp' to the bus and tell other devices this is a standard frame from 0x00.

    // unsigned char stmp[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    // CAN.sendMsgBuf(0x00, 0, 8, stmp);
    return 1;
}
