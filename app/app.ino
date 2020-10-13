#include <string.h>
#include <time.h>
#include <stdio.h>
#include <SPI.h>

#include "canard.h"

#include <mcp_can_dfs.h>
#include <mcp_can.h>

/////////////////////////////
#define CANARD_CAN_EXT_ID_MASK 0x1FFFFFFFU
#define CANARD_CAN_FRAME_EFF (1UL << 31U) ///< Extended frame format
////////////////////////////////

static const uint16_t HeartbeatSubjectID = 32085;
static const uint16_t UltrasoundMessageSubjectID = 1610;

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

    canInit();

    // Initialize the node with a static node-ID as specified in the command-line arguments.
    canard = canardInit(&canardAllocate, &canardFree);
    canard.mtu_bytes = CANARD_MTU_CAN_CLASSIC; // Do not use CAN FD to enhance compatibility.
    canard.node_id = (CanardNodeID)42;

        // Heatbeat subscription
    CanardRxSubscription heartbeat_subscription;
    (void)canardRxSubscribe(&canard, // Subscribe to messages uavcan.node.Heartbeat.
                            CanardTransferKindMessage,
                            HeartbeatSubjectID, // The fixed Subject-ID of the Heartbeat message type (see DSDL definition).
                            7,                  // The maximum payload size (max DSDL object size) from the DSDL definition.
                            CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC,
                            &heartbeat_subscription);
    
    // Ultrasound subscription
    CanardRxSubscription ultrasound_subscription;
    (void)canardRxSubscribe(&canard, // Subscribe to messages uavcan.node.Heartbeat.
                            CanardTransferKindMessage,
                            UltrasoundMessageSubjectID,
                            7,                  
                            CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC,
                            &ultrasound_subscription);

    setup_pacemaker();
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

// Making sure it lives
static void handleHeartbeat()
{
    Serial.println("Thump thump");
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

// CAN Interface based on code written by the UAVCAN Team

int canInit()
{
    while (CAN_OK != CAN.begin(CAN_500KBPS)) // init can bus : baudrate = 500k
    {
        Serial.println("CAN BUS Shield init fail");
        Serial.println(" Init CAN BUS Shield again");
        delay(100);
    }
    Serial.println("CAN BUS Shield init ok!");
    return CAN_OK;
}

int canClose(void)
{
    return -1;
}

int canTransmit(const CanardFrame *frame)
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

int canRecieve(CanardFrame *out_frame)
{

    unsigned char len = 0;
    unsigned char buf[64];
    int receiver_state = CAN.checkReceive();

    if (CAN_NOMSG == receiver_state)
    {
        return 0;
    }

    if (CAN_MSGAVAIL == receiver_state)
    {

        // check if data coming
        CAN.readMsgBuf(&len, buf); // read data,  len: data length, buf: data buf

        out_frame->extended_can_id = CAN.getCanId();
        out_frame->payload_size = len;
        out_frame->payload = buf;
        // out_frame->id = receive_msg.id;
        // out_frame->data_len = receive_msg.length;
        // memcpy(out_frame->data, receive_msg.data, receive_msg.length);

        // if (receive_msg.flags.extended != 0)
        // {
        //     out_frame->id |= CANARD_CAN_FRAME_EFF;
        // }

        unsigned long canId = CAN.getCanId();

        Serial.println("-----------------------------");
        Serial.print("Get data from ID: 0x");
        Serial.println(canId, HEX);

        for (int i = 0; i < len; i++)
        { // print the data
            Serial.print(buf[i], HEX);
            Serial.print("\t");
        }
        Serial.println();

        return 1;
    }

    return -1;

    // const int poll_result = can_check_message();
    // if (poll_result <= 0)
    // {
    //     return 0;
    // }

    // can_t receive_msg;
    // const uint8_t res = can_get_message(&receive_msg);
    // if (res <= 0)
    // {
    //     return -1;
    // }
}

int canConfigureAcceptanceFilters()
{
    //     static const uint32_t DefaultFilterMsgMask = 0x80;
    //     static const uint32_t DefaultFilterMsgID = 0x0;
    //     static const uint32_t DefaultFilterSrvMask = 0x7F80;
    //     uint8_t res = 1;

    //     // create a new filter for receiving messages
    //     can_filter_t filter_Msg = {
    //         .id = DefaultFilterMsgID,
    //         .mask = DefaultFilterMsgMask,
    //         .flags = {
    //             .rtr = 0,
    //             .extended = 3
    //         }
    //     };

    //     // create a new filter for receiving services
    //     can_filter_t filter_Srv = {
    //         .id = ((uint32_t)node_id << 8) | 0x80,
    //         .mask = DefaultFilterSrvMask,
    //         .flags = {
    //             .rtr = 0,
    //             .extended = 3
    //         }
    //     };

    //     // setup 2 MOb's to receive, 12 MOb's are used as send buffer
    //     if (!can_set_filter(0, &filter_Msg))
    //     {
    //         res = -1;
    //     }

    //     if (!can_set_filter(1, &filter_Srv))
    //     {
    //         res = -1;
    //     }

    //     return res;
    return -1;
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
        canTransmit(txf);
        canardTxPop(&canard);
        free((void *)txf);
        txf = canardTxPeek(&canard);
    }

    // Process received frames, if any.
    CanardFrame rxf;
    uint8_t buffer[64];
    //while (socketcanPop(sock, &rxf, sizeof(buffer), buffer, 1000) > 0)  // Error handling not implemented
    if (canRecieve(&rxf))
    {

Serial.println("************");
        Serial.println(rxf.extended_can_id);
        Serial.println(rxf.payload_size);


        CanardTransfer transfer;
        int accepted = 2;
        if (( accepted = canardRxAccept(&canard, &rxf, 0, &transfer)))
        {
            Serial.print("CAN RECEIVE\n");
            if ((transfer.transfer_kind == CanardTransferKindMessage) &&
                (transfer.port_id == HeartbeatSubjectID))
            {
                Serial.print("Hearbeat Recieved\n");
            }
            free((void *)transfer.payload);
        }
        Serial.println(accepted);
        Serial.println("************");
    }
}