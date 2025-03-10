/*

    UART Framing Layer
    ------------------
    Provides a reliable mechanism for sending and receiving asynchronous frames 
    of data over UART or similar serial interfaces.


    Copyright (c) 2025 Thomas Granig

*/

#include "uart_framing_layer.h"

#include <string.h>

static void clear_tx_fifo(uart_framing_layer_ctx_t* ctx) {
    ctx->tx.fifo_buffer_offset = 0;
    ctx->tx.fifo_buffer_length = 0;
}

static uart_framing_layer_operation_status_t tx_fifo_read_bytes(uart_framing_layer_ctx_t* ctx, uint8_t* dest, uint32_t length) {
    if (ctx->tx.fifo_buffer_length < length) {
        return UART_FRAMING_LAYER_OPERATION_STATUS_ERROR;
    }

    for (uint32_t i = 0; i < length; i++) {
        dest[i] = ctx->tx.fifo_buffer[ctx->tx.fifo_buffer_offset];
        ctx->tx.fifo_buffer_offset = (ctx->tx.fifo_buffer_offset + 1) % ctx->tx.fifo_buffer_capacity;
    }

    ctx->tx.fifo_buffer_length -= length;

    return UART_FRAMING_LAYER_OPERATION_STATUS_OK;
}

static uart_framing_layer_operation_status_t tx_fifo_write_bytes(uart_framing_layer_ctx_t* ctx, const uint8_t* src, uint32_t length) {
    if (ctx->tx.fifo_buffer_length + length > ctx->tx.fifo_buffer_capacity) {
        return UART_FRAMING_LAYER_OPERATION_STATUS_ERROR;
    }

    for (uint32_t i = 0; i < length; i++) {
        ctx->tx.fifo_buffer[(ctx->tx.fifo_buffer_offset + ctx->tx.fifo_buffer_length + i) % ctx->tx.fifo_buffer_capacity] = src[i];
    }

    ctx->tx.fifo_buffer_length += length;

    return UART_FRAMING_LAYER_OPERATION_STATUS_OK;
}

static void clear_rx_fifo(uart_framing_layer_ctx_t* ctx) {
    ctx->rx.fifo_buffer_offset = 0;
    ctx->rx.fifo_buffer_length = 0;
}

static uart_framing_layer_operation_status_t rx_fifo_read_bytes(uart_framing_layer_ctx_t* ctx, uint8_t* dest, uint32_t length) {
    if (ctx->rx.fifo_buffer_length < length) {
        return UART_FRAMING_LAYER_OPERATION_STATUS_ERROR;
    }

    for (uint32_t i = 0; i < length; i++) {
        dest[i] = ctx->rx.fifo_buffer[ctx->rx.fifo_buffer_offset];
        ctx->rx.fifo_buffer_offset = (ctx->rx.fifo_buffer_offset + 1) % ctx->rx.fifo_buffer_capacity;
    }

    ctx->rx.fifo_buffer_length -= length;

    return UART_FRAMING_LAYER_OPERATION_STATUS_OK;
}

static uart_framing_layer_operation_status_t rx_fifo_write_bytes(uart_framing_layer_ctx_t* ctx, const uint8_t* src, uint32_t length) {
    if (ctx->rx.fifo_buffer_length + length > ctx->rx.fifo_buffer_capacity) {
        return UART_FRAMING_LAYER_OPERATION_STATUS_ERROR;
    }

    for (uint32_t i = 0; i < length; i++) {
        ctx->rx.fifo_buffer[(ctx->rx.fifo_buffer_offset + ctx->rx.fifo_buffer_length + i) % ctx->rx.fifo_buffer_capacity] = src[i];
    }

    ctx->rx.fifo_buffer_length += length;

    return UART_FRAMING_LAYER_OPERATION_STATUS_OK;
}

static uint16_t calculate_checksum(uint8_t* data, uint32_t length) {
    uint16_t checksum = 0;
    for (uint32_t i = 0; i < length; i++) {
        checksum += data[i] + (data[i] >> (i % 6));
    }
    return checksum;
}

static void send_check(uart_framing_layer_ctx_t* ctx) {
    switch (ctx->tx.state) {
    case UART_FRAMING_LAYER_CTX_TX_STATE_IDLE: { // nothing being sent at the moment
        if (ctx->tx.fifo_buffer_length > 0) { // we have a frame to send
            // start sending the oldest frame
            ctx->tx.state = UART_FRAMING_LAYER_CTX_TX_STATE_SENDING_FRAME;
            ctx->tx.frame_offset = 0;

            // write the preamble
            *(uint32_t*)&ctx->tx.transmit_buffer[0] = UART_FRAME_PREAMBLE;
            ctx->tx.frame_offset += 4;

            // write the header
            tx_fifo_read_bytes(ctx, (uint8_t*)(ctx->tx.transmit_buffer + ctx->tx.frame_offset), sizeof(uart_frame_header_t));
            uart_frame_header_t* header = (uart_frame_header_t*)(ctx->tx.transmit_buffer + ctx->tx.frame_offset);

            ctx->tx.frame_offset += sizeof(uart_frame_header_t);
            ctx->tx.frame_length = UART_FRAME_PREAMBLE_LENGTH + sizeof(uart_frame_header_t) + header->length;

            if (ctx->tx.frame_length > sizeof(ctx->tx.transmit_buffer)) {
                // frame too large somehow
                ctx->tx.state = UART_FRAMING_LAYER_CTX_TX_STATE_IDLE;
                clear_tx_fifo(ctx);
                return;
            }

            // write the data
            tx_fifo_read_bytes(ctx, (uint8_t*)(ctx->tx.transmit_buffer + ctx->tx.frame_offset), header->length);
            ctx->tx.frame_offset += header->length;

            // send the frame
            ctx->send_callback(ctx->user_data, ctx->tx.transmit_buffer, ctx->tx.frame_length);
        }

        break;
    }
    case UART_FRAMING_LAYER_CTX_TX_STATE_SENDING_FRAME: { // we are in the middle of sending a frame
        // can't do anything here, we have to wait for the user to call uart_framing_layer_mark_frame_sent(), which will call this function again
        break;
    }

    }
}

void uart_framing_layer_mark_frame_sent(uart_framing_layer_ctx_t* ctx) {
    // called from ISR context
    ctx->tx.state = UART_FRAMING_LAYER_CTX_TX_STATE_IDLE;
    send_check(ctx);
}

// Call this from the UART receive interrupt callback
void uart_framing_layer_process_byte(uart_framing_layer_ctx_t* ctx) {
    ctx->disable_RXNE_interrupt_callback(ctx->user_data);

    switch (ctx->rx.state) {
    case UART_FRAMING_LAYER_CTX_RX_STATE_WAITING_FOR_PREAMBLE: {
        if (ctx->rx.received_byte == ((UART_FRAME_PREAMBLE >> (ctx->rx.frame_offset * 8)) & 0xff)) {
            ctx->rx.frame_offset++;
            if (ctx->rx.frame_offset == UART_FRAME_PREAMBLE_LENGTH) {
                ctx->rx.state = UART_FRAMING_LAYER_CTX_RX_STATE_RECEIVING_HEADER;
                ctx->rx.frame_offset = 0;
            }
        }
        else {
            ctx->rx.frame_offset = 0;
        }

        break;
    }

    case UART_FRAMING_LAYER_CTX_RX_STATE_RECEIVING_HEADER: {
        ((uint8_t*)&ctx->rx.header)[ctx->rx.frame_offset] = ctx->rx.received_byte;
        ctx->rx.frame_offset++;
        if (ctx->rx.frame_offset == sizeof(uart_frame_header_t)) {
            ctx->rx.state = UART_FRAMING_LAYER_CTX_RX_STATE_RECEIVING_DATA;
            ctx->rx.receive_buffer_offset = 0;
        }

        break;
    }

    case UART_FRAMING_LAYER_CTX_RX_STATE_RECEIVING_DATA: {
        ctx->rx.receive_buffer[ctx->rx.receive_buffer_offset] = ctx->rx.received_byte;
        ctx->rx.receive_buffer_offset++;

        if (ctx->rx.receive_buffer_offset > sizeof(ctx->rx.receive_buffer)) {
            ctx->rx.state = UART_FRAMING_LAYER_CTX_RX_STATE_WAITING_FOR_PREAMBLE;
            ctx->rx.frame_offset = 0;
            ctx->rx.receive_buffer_offset = 0;
            break;
        }

        ctx->rx.frame_offset++;
        if (ctx->rx.receive_buffer_offset == ctx->rx.header.length) {
            if (ctx->rx.header.checksum == calculate_checksum(ctx->rx.receive_buffer, ctx->rx.header.length)) {
                // frame received successfully

                //write the header to the receive fifo buffer, for processing later in the main execution context which calls uart_framing_layer_tick()
                if (rx_fifo_write_bytes(ctx, (uint8_t*)&ctx->rx.header, sizeof(uart_frame_header_t)) != UART_FRAMING_LAYER_OPERATION_STATUS_OK) {
                    clear_rx_fifo(ctx);
                }

                // write the data to the receive fifo buffer
                if (rx_fifo_write_bytes(ctx, ctx->rx.receive_buffer, ctx->rx.header.length) != UART_FRAMING_LAYER_OPERATION_STATUS_OK) {
                    clear_rx_fifo(ctx);
                }
            }

            ctx->rx.state = UART_FRAMING_LAYER_CTX_RX_STATE_WAITING_FOR_PREAMBLE;
            ctx->rx.frame_offset = 0;
            ctx->rx.receive_buffer_offset = 0;
        }

        break;
    }
    }

    ctx->enable_RXNE_interrupt_callback(ctx->user_data);
    ctx->start_receive_callback(ctx->user_data);
}


uart_framing_layer_operation_status_t uart_framing_layer_init(uart_framing_layer_ctx_t* ctx, uart_framing_layer_ctx_init_t* init) {
    memset(ctx, 0, sizeof(uart_framing_layer_ctx_t));
    ctx->frame_receive_callback = init->frame_receive_callback;
    ctx->send_callback = init->send_callback;
    ctx->start_receive_callback = init->start_receive_callback;
    ctx->enable_RXNE_interrupt_callback = init->enable_RXNE_interrupt_callback;
    ctx->disable_RXNE_interrupt_callback = init->disable_RXNE_interrupt_callback;
    ctx->enable_TC_interrupt_callback = init->enable_TC_interrupt_callback;
    ctx->disable_TC_interrupt_callback = init->disable_TC_interrupt_callback;

    if (init->rx_buffer == NULL || init->rx_buffer_capacity == 0 || init->tx_buffer == NULL || init->tx_buffer_capacity == 0) {
        return UART_FRAMING_LAYER_OPERATION_STATUS_INVALID_ARGUMENT;
    }

    if (init->rx_buffer_capacity < sizeof(uart_frame_header_t) + UART_FRAME_MAX_DATA_LEN) {
        return UART_FRAMING_LAYER_OPERATION_STATUS_INVALID_ARGUMENT;
    }

    if (init->tx_buffer_capacity < (sizeof(uart_frame_header_t) + UART_FRAME_MAX_DATA_LEN)) {
        return UART_FRAMING_LAYER_OPERATION_STATUS_INVALID_ARGUMENT;
    }

    if (init->send_callback == NULL || init->start_receive_callback == NULL || init->enable_RXNE_interrupt_callback == NULL || init->disable_RXNE_interrupt_callback == NULL || init->enable_TC_interrupt_callback == NULL || init->disable_TC_interrupt_callback == NULL) {
        return UART_FRAMING_LAYER_OPERATION_STATUS_INVALID_ARGUMENT;
    }


    ctx->user_data = init->user_data;

    ctx->rx.state = UART_FRAMING_LAYER_CTX_RX_STATE_WAITING_FOR_PREAMBLE;
    ctx->rx.frame_offset = 0;
    ctx->rx.receive_buffer_offset = 0;

    ctx->rx.fifo_buffer = init->rx_buffer;
    ctx->rx.fifo_buffer_capacity = init->rx_buffer_capacity;
    ctx->rx.fifo_buffer_offset = 0;
    ctx->rx.fifo_buffer_length = 0;
    
    ctx->tx.state = UART_FRAMING_LAYER_CTX_TX_STATE_IDLE;
    ctx->tx.frame_offset = 0;
    ctx->tx.fifo_buffer = init->tx_buffer;
    ctx->tx.fifo_buffer_capacity = init->tx_buffer_capacity;
    ctx->tx.fifo_buffer_offset = 0;
    ctx->tx.fifo_buffer_length = 0;

    ctx->enable_RXNE_interrupt_callback(ctx->user_data);
    ctx->enable_TC_interrupt_callback(ctx->user_data);
    ctx->start_receive_callback(ctx->user_data);

    return UART_FRAMING_LAYER_OPERATION_STATUS_OK;
}

uart_framing_layer_operation_status_t uart_framing_layer_send_frame(uart_framing_layer_ctx_t* ctx, uint8_t* data, uint32_t length) {
    ctx->disable_TC_interrupt_callback(ctx->user_data);

    uart_framing_layer_operation_status_t return_status = UART_FRAMING_LAYER_OPERATION_STATUS_OK;

    // check if we have enough available space in the fifo buffer (sizeof(uart_framing_layer_tx_fifo_entry_header_t) bytes + length of data)
    if (ctx->tx.fifo_buffer_length + sizeof(uart_frame_header_t) + length > ctx->tx.fifo_buffer_capacity) {
        return_status = UART_FRAMING_LAYER_OPERATION_STATUS_OUT_OF_MEMORY;
        goto exit_failure;
    }

    if (length > UART_FRAME_MAX_DATA_LEN) {
        return_status = UART_FRAMING_LAYER_OPERATION_STATUS_INVALID_ARGUMENT;
        goto exit_failure;
    }

    // write the header
    uart_frame_header_t header = {
        .length = length,
        .checksum = calculate_checksum(data, length)
    };

    if (tx_fifo_write_bytes(ctx, (uint8_t*)&header, sizeof(header)) != UART_FRAMING_LAYER_OPERATION_STATUS_OK) {
        clear_tx_fifo(ctx); // the safe thing to do here is to clear the fifo buffer
        return_status = UART_FRAMING_LAYER_OPERATION_STATUS_ERROR;
        goto exit_failure;
    }

    // write the data to the fifo buffer
    if (tx_fifo_write_bytes(ctx, data, length) != UART_FRAMING_LAYER_OPERATION_STATUS_OK) {
        clear_tx_fifo(ctx); // the safe thing to do here is to clear the fifo buffer
        return_status = UART_FRAMING_LAYER_OPERATION_STATUS_ERROR;
        goto exit_failure;
    }

    send_check(ctx);

    ctx->enable_TC_interrupt_callback(ctx->user_data);

    return UART_FRAMING_LAYER_OPERATION_STATUS_OK;

exit_failure:
    ctx->enable_RXNE_interrupt_callback(ctx->user_data);
    ctx->enable_TC_interrupt_callback(ctx->user_data);

    return return_status;
}

// call this from the main execution context periodically. This function will call the frame_receive_callback at most once per call if there are frames in the receive fifo buffer
void uart_framing_layer_tick(uart_framing_layer_ctx_t* ctx) {
    bool has_frame = false;
    uart_frame_header_t header;


    ctx->disable_RXNE_interrupt_callback(ctx->user_data);
    ctx->disable_TC_interrupt_callback(ctx->user_data);

    send_check(ctx);

    // dequeue a frame from the receive fifo buffer
    if (ctx->rx.fifo_buffer_length > 0) {
        has_frame = true;
        rx_fifo_read_bytes(ctx, (uint8_t*)&header, sizeof(header));
        rx_fifo_read_bytes(ctx, ctx->rx.frame_callback_data_buffer, header.length);
    }

    ctx->enable_RXNE_interrupt_callback(ctx->user_data);
    ctx->enable_TC_interrupt_callback(ctx->user_data);
    ctx->start_receive_callback(ctx->user_data);

    // call the callback once we have reenabled the interrupts so that we do not miss any incoming data with a long-running callback
    if (has_frame) {
        ctx->frame_receive_callback(ctx->user_data, ctx->rx.frame_callback_data_buffer, header.length);
    }

}
