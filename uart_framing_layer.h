


#ifndef __UART_FRAMING_LAYER_H
#define __UART_FRAMING_LAYER_H

#include <stdint.h>
#include <stdbool.h>


#define UART_FRAME_PREAMBLE_LENGTH 4 // bytes
#define UART_FRAME_MAX_DATA_LEN 1600 // bytes
#define UART_FRAME_PREAMBLE 0xA0B1C2D3 // little endian transmission of bytes

typedef enum {
    UART_FRAMING_LAYER_OPERATION_STATUS_OK = 0,
    UART_FRAMING_LAYER_OPERATION_STATUS_TIMEOUT,
    UART_FRAMING_LAYER_OPERATION_STATUS_BUSY,
    UART_FRAMING_LAYER_OPERATION_STATUS_INVALID_ARGUMENT,
    UART_FRAMING_LAYER_OPERATION_STATUS_ERROR,
    UART_FRAMING_LAYER_OPERATION_STATUS_OUT_OF_MEMORY,
} uart_framing_layer_operation_status_t;


typedef enum {
    UART_FRAMING_LAYER_CTX_RX_STATE_WAITING_FOR_PREAMBLE, // the first state, or if the receiver has lost sync
    UART_FRAMING_LAYER_CTX_RX_STATE_RECEIVING_HEADER, // receiving the header
    UART_FRAMING_LAYER_CTX_RX_STATE_RECEIVING_DATA, // receiving the data
} uart_framing_layer_ctx_rx_state_t;

typedef enum {
    UART_FRAMING_LAYER_CTX_TX_STATE_IDLE, // waiting for data to send
    UART_FRAMING_LAYER_CTX_TX_STATE_SENDING_FRAME, // sending the frame
} uart_framing_layer_ctx_tx_state_t;

typedef void (*uart_frame_receive_callback_t)(void* user_data, uint8_t* data, uint32_t length);
typedef void (*uart_data_chunk_send_callback_t)(void* user_data, uint8_t* data, uint32_t length);
typedef void (*uart_action_callback_t)(void* user_data);


typedef volatile struct {
    volatile uint16_t length; // little endian
    volatile uint16_t checksum; // little endian
    // ... data follows, length as specified
} __attribute__((packed)) uart_frame_header_t;

// if the receiver loses sync (i.e checksum/preamble errors), it can scan for the next preamble to resync

typedef struct {
    uart_frame_receive_callback_t frame_receive_callback;
    uart_data_chunk_send_callback_t send_callback;
    uart_action_callback_t start_receive_callback; // called by the layer to start receiving data (a byte)

    uart_action_callback_t enable_RXNE_interrupt_callback; // called by the layer to enable the RXNE interrupt
    uart_action_callback_t disable_RXNE_interrupt_callback; // called by the layer to disable the RXNE interrupt
    uart_action_callback_t enable_TC_interrupt_callback; // called by the layer to enable the TC interrupt
    uart_action_callback_t disable_TC_interrupt_callback; // called by the layer to disable the TC interrupt

    void* user_data;
    uint8_t* rx_buffer;
    uint32_t rx_buffer_capacity;
    uint8_t* tx_buffer;
    uint32_t tx_buffer_capacity;
} uart_framing_layer_ctx_init_t;

typedef volatile struct {
    volatile struct {
        volatile uart_framing_layer_ctx_rx_state_t state;
        volatile uint32_t frame_offset; // 0 = first byte of preamble, 4=first byte of header (length field), 6=first byte of checksum field, 8=first byte of data
        volatile uint32_t frame_length; // length of the frame, including the preamble. used in state UART_FRAMING_LAYER_CTX_RX_STATE_RECEIVING_DATA
        volatile uart_frame_header_t header; // the header of the frame. set in UART_FRAMING_LAYER_CTX_RX_STATE_RECEIVING_HEADER, used in UART_FRAMING_LAYER_CTX_RX_STATE_RECEIVING_DATA

        volatile uint8_t received_byte;

        volatile uint8_t receive_buffer[UART_FRAME_PREAMBLE_LENGTH + sizeof(uart_frame_header_t) + UART_FRAME_MAX_DATA_LEN]; // receive buffer for the frame
        volatile uint32_t receive_buffer_offset;

        volatile uint8_t* volatile fifo_buffer; // the fifo buffer, containing the received frames ready to be dequeued
        volatile uint32_t fifo_buffer_capacity; // the capacity of the fifo buffer in bytes
        volatile uint32_t fifo_buffer_offset; // the current offset in the fifo buffer
        volatile uint32_t fifo_buffer_length; // the current length of the fifo buffer

        volatile uint8_t frame_callback_data_buffer[UART_FRAME_MAX_DATA_LEN]; // copied from rx fifo to this before calling the frame receive callback. ensures no race conditions when allowing receiving interrupts during the callback's operation
    } rx;

    volatile struct {
        volatile uart_framing_layer_ctx_tx_state_t state;
        volatile uint32_t frame_offset; // 0 = first byte of preamble, 4=first byte of header (length field), 6=first byte of checksum field, 8=first byte of data
        volatile uint32_t frame_length; // length of the frame being sent

        volatile uint32_t pending_frame_data_offset; // the fifo relative offset pointing to the next frame data's length word in the fifo buffer
        volatile uint8_t* volatile fifo_buffer; // the fifo buffer, containing the data of each pending frame to be sent, prefixed with uart_frame_header_t header that includes an int field so we know how much data to send and to free up when we're done
        volatile uint32_t fifo_buffer_capacity; // the capacity of the fifo buffer in bytes
        volatile uint32_t fifo_buffer_offset; // the current offset in the fifo buffer
        volatile uint32_t fifo_buffer_length; // the current length of the fifo buffer

        volatile uint8_t transmit_buffer[UART_FRAME_PREAMBLE_LENGTH + sizeof(uart_frame_header_t) + UART_FRAME_MAX_DATA_LEN]; // the buffer containing the preamble and header of the frame being sent
    } tx;

    uart_frame_receive_callback_t frame_receive_callback; // called when a frame is received
    uart_data_chunk_send_callback_t send_callback; // called by the layer to send a data chunk. user code should call uart_framing_layer_mark_data_chunk_sent() when the data is sent
    uart_action_callback_t start_receive_callback; // called by the layer to start receiving data (a byte)

    uart_action_callback_t enable_RXNE_interrupt_callback; // called by the layer to enable the RXNE (receive data register not empty) interrupt
    uart_action_callback_t disable_RXNE_interrupt_callback; // called by the layer to disable the RXNE (receive data register not empty) interrupt
    uart_action_callback_t enable_TC_interrupt_callback; // called by the layer to enable the TC (transmission complete) interrupt
    uart_action_callback_t disable_TC_interrupt_callback; // called by the layer to disable the TC (transmission complete) interrupt

    void* user_data; //e.g the UART peripheral handle
} uart_framing_layer_ctx_t;


void uart_framing_layer_mark_frame_sent(uart_framing_layer_ctx_t* ctx);
void uart_framing_layer_process_byte(uart_framing_layer_ctx_t* ctx);

uart_framing_layer_operation_status_t uart_framing_layer_send_frame(uart_framing_layer_ctx_t* ctx, uint8_t* data, uint32_t length);
uart_framing_layer_operation_status_t uart_framing_layer_init(uart_framing_layer_ctx_t* ctx, uart_framing_layer_ctx_init_t* init);
void uart_framing_layer_tick(uart_framing_layer_ctx_t* ctx); // call periodically to ensure "stuck" frames get sent (skipped interrupts)


#endif // __UART_FRAMING_LAYER_H