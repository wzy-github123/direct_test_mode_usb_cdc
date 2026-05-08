/**
 * Copyright (c) 2012 - 2021, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/**
  @defgroup dtm_standalone main.c
  @{
  @ingroup ble_sdk_app_dtm_serial
  @brief Stand-alone DTM application for UART interface.

 */

#include <stdint.h>
#include <stdbool.h>
#include "nrf.h"
#include "ble_dtm.h"
#include "boards.h"
#include "app_uart.h"
#include "app_error.h"
#include "app_usbd_core.h"
#include "app_usbd.h"
#include "app_usbd_string_desc.h"
#include "app_usbd_cdc_acm.h"
#include "app_usbd_serial_num.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "nrf_nvic.h"
#include "nrf52840.h"

#include "nrf_pwr_mgmt.h"

#include <stdarg.h>
#include <string.h>
#include <stdio.h>

#include "hardfault.h"


static void cdc_acm_user_ev_handler( app_usbd_class_inst_t const * p_inst,  app_usbd_cdc_acm_user_event_t event );

#define CDC_ACM_COMM_INTERFACE  0
#define CDC_ACM_COMM_EPIN       NRF_DRV_USBD_EPIN2

#define CDC_ACM_DATA_INTERFACE  1
#define CDC_ACM_DATA_EPIN       NRF_DRV_USBD_EPIN1
#define CDC_ACM_DATA_EPOUT      NRF_DRV_USBD_EPOUT1

/**
 * @brief CDC_ACM class instance
 * */
APP_USBD_CDC_ACM_GLOBAL_DEF(m_app_cdc_acm,
                            cdc_acm_user_ev_handler,
                            CDC_ACM_COMM_INTERFACE,
                            CDC_ACM_DATA_INTERFACE,
                            CDC_ACM_COMM_EPIN,
                            CDC_ACM_DATA_EPIN,
                            CDC_ACM_DATA_EPOUT,
                            APP_USBD_CDC_COMM_PROTOCOL_AT_V250
);

#define READ_SIZE 1
static char m_rx_buffer[READ_SIZE];

static bool m_usb_init = false;
static bool m_usb_connected = false;
uint8_t usb_cdc_data = 0;
static bool usb_rec_ok = false;
uint8_t dtm_data[2] = {0};
uint8_t dtm_data_index = 0;

/**
 * @brief User event handler @ref app_usbd_cdc_acm_user_ev_handler_t (headphones)
 * */
static void cdc_acm_user_ev_handler( app_usbd_class_inst_t const * p_inst, app_usbd_cdc_acm_user_event_t event )
{
    app_usbd_cdc_acm_t const * p_cdc_acm = app_usbd_cdc_acm_class_get( p_inst );

    switch( event )
    {
        case APP_USBD_CDC_ACM_USER_EVT_PORT_OPEN:
        {
            /*Setup first transfer*/
            ret_code_t ret = app_usbd_cdc_acm_read( &m_app_cdc_acm, m_rx_buffer, READ_SIZE );
            UNUSED_VARIABLE(ret);
            break;
        }

        case APP_USBD_CDC_ACM_USER_EVT_PORT_CLOSE:
            break;

        case APP_USBD_CDC_ACM_USER_EVT_TX_DONE:
            break;
        
        case APP_USBD_CDC_ACM_USER_EVT_RX_DONE:
        {
            ret_code_t ret;
            NRF_LOG_INFO( "Bytes waiting: %d", app_usbd_cdc_acm_bytes_stored( p_cdc_acm ));
            do
            {
                /*Get amount of data transfered*/
                size_t size = app_usbd_cdc_acm_rx_size( p_cdc_acm );
                NRF_LOG_INFO("RX: size: %lu char: %c", size, m_rx_buffer[0] );

                dtm_data[dtm_data_index] = m_rx_buffer[0];
                dtm_data_index++;
                if(dtm_data_index >=2)
                {
                    usb_rec_ok = true;   
                    dtm_data_index = 0;        
                }
                /* Fetch data until internal buffer is empty */
                ret = app_usbd_cdc_acm_read( &m_app_cdc_acm, m_rx_buffer, READ_SIZE );
            } while( ret == NRF_SUCCESS );
            
            break;
        }
        default:
            break;
    }
}

static void usbd_user_ev_handler( app_usbd_event_type_t event )
{
    switch( event )
    {
        case APP_USBD_EVT_DRV_SUSPEND:
            break;
        case APP_USBD_EVT_DRV_RESUME:
            break;
        case APP_USBD_EVT_STARTED:
            break;
        case APP_USBD_EVT_STOPPED:
            app_usbd_disable( );
            break;
        case APP_USBD_EVT_POWER_DETECTED:
            NRF_LOG_INFO( "USB power detected" );
            if( !nrf_drv_usbd_is_enabled( ))
            {
                app_usbd_enable( );
            }
            break;
        case APP_USBD_EVT_POWER_REMOVED:
            NRF_LOG_INFO( "USB power removed" );
            app_usbd_stop( );
            m_usb_connected = false;
            break;
        case APP_USBD_EVT_POWER_READY:
            NRF_LOG_INFO( "USB ready" );
            app_usbd_start( );
            m_usb_connected = true;
            break;
        default:
            break;
    }
}

void hal_usb_cdc_init( void )
{
    ret_code_t ret;
    static const app_usbd_config_t usbd_config = {
        .ev_state_proc = usbd_user_ev_handler
    };

    while( !nrf_drv_clock_lfclk_is_running( ))
    {
        /* Just waiting */
    }

    app_usbd_serial_num_generate( );

    ret = app_usbd_init( &usbd_config );
    APP_ERROR_CHECK( ret );

    app_usbd_class_inst_t const *class_cdc_acm = app_usbd_cdc_acm_class_inst_get( &m_app_cdc_acm );
    ret = app_usbd_class_append( class_cdc_acm );
    APP_ERROR_CHECK( ret );

    ret = app_usbd_power_events_enable( );
    APP_ERROR_CHECK( ret );

    m_usb_init = true;
}

void hal_usb_cdc_deinit( void )
{
    app_usbd_uninit( );
}

void hal_usb_cdc_write( uint8_t* buff, uint16_t len )
{
    if( m_usb_connected )
    {
        app_usbd_cdc_acm_write( &m_app_cdc_acm, buff, len );
    }
}

void hal_usb_cdc_read( uint8_t* buff, uint16_t len )
{
    if( m_usb_connected )
    {
        app_usbd_cdc_acm_read( &m_app_cdc_acm, buff, len );
    }
}

void hal_usb_cdc_event_queue_process( void )
{
    if( m_usb_init )
    {
        app_usbd_event_queue_process( );
    }
}

bool hal_usb_cdc_is_connected( void )
{
    return m_usb_connected;
}


#if defined(NRF21540_DRIVER_ENABLE) && (NRF21540_DRIVER_ENABLE == 1)
#include "nrf21540.h"
#endif

// @note: The BLE DTM 2-wire UART standard specifies 8 data bits, 1 stop bit, no flow control.
//        These parameters are not configurable in the BLE standard.

/**@details Maximum iterations needed in the main loop between stop bit 1st byte and start bit 2nd
 * byte. DTM standard allows 5000us delay between stop bit 1st byte and start bit 2nd byte.
 * As the time is only known when a byte is received, then the time between between stop bit 1st
 * byte and stop bit 2nd byte becomes:
 *      5000us + transmission time of 2nd byte.
 *
 * Byte transmission time is (Baud rate of 19200):
 *      10bits * 1/19200 = approx. 520 us/byte (8 data bits + start & stop bit).
 *
 * Loop time on polling UART register for received byte is defined in ble_dtm.c as:
 *   UART_POLL_CYCLE = 260 us
 *
 * The max time between two bytes thus becomes (loop time: 260us / iteration):
 *      (5000us + 520us) / 260us / iteration = 21.2 iterations.
 *
 * This is rounded down to 21.
 *
 * @note If UART bit rate is changed, this value should be recalculated as well.
 */
#define MAX_ITERATIONS_NEEDED_FOR_NEXT_BYTE ((5000 + 2 * UART_POLL_CYCLE) / UART_POLL_CYCLE)

#define MAX_TEST_DATA_BYTES     (15U)                /**< max number of test bytes to be used for tx and rx. */
#define UART_TX_BUF_SIZE 256                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE 256                         /**< UART RX buffer size. */

// Error handler for UART
void uart_error_handle(app_uart_evt_t * p_event)
{
    if (p_event->evt_type == APP_UART_COMMUNICATION_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_communication);
    }
    else if (p_event->evt_type == APP_UART_FIFO_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_code);
    }
}

/**@brief Function for UART initialization.
 */
static void uart_init(void)
{   
    uint32_t err_code;
    const app_uart_comm_params_t comm_params =
      {
          RX_PIN_NUMBER,
          TX_PIN_NUMBER,
          RTS_PIN_NUMBER,
          CTS_PIN_NUMBER,
          APP_UART_FLOW_CONTROL_DISABLED,
          false,
          DTM_BITRATE
      };

    APP_UART_FIFO_INIT(&comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_error_handle,
                       APP_IRQ_PRIORITY_LOWEST,
                       err_code);

    APP_ERROR_CHECK(err_code);
}


/**@brief Function for application main entry.
 *
 * @details This function serves as an adaptation layer between a 2-wire UART interface and the
 *          dtmlib. After initialization, DTM commands submitted through the UART are forwarded to
 *          dtmlib and events (i.e. results from the command) is reported back through the UART.
 */


int main(void)
{
    ret_code_t ret;
    uint32_t    current_time;
    uint32_t    dtm_error_code;
    uint32_t    msb_time          = 0;     // Time when MSB of the DTM command was read. Used to catch stray bytes from "misbehaving" testers.
    bool        is_msb_read       = false; // True when MSB of the DTM command has been read and the application is waiting for LSB.
    uint16_t    dtm_cmd_from_uart = 0;     // Packed command containing command_code:freqency:length:payload in 2:6:6:2 bits.
    uint8_t     rx_byte;                   // Last byte read from UART.
    dtm_event_t result;                    // Result of a DTM operation.
    uint8_t buffer[2] = {0};

    // nrf_gpio_cfg_output( 12 );
    // nrf_gpio_pin_write( 12, 1 );


    /* Initializing power and clock */
    ret = nrf_drv_clock_init();
    APP_ERROR_CHECK(ret);

    ret = nrf_drv_power_init(NULL);
    APP_ERROR_CHECK(ret);

    nrf_drv_clock_hfclk_request(NULL);
    nrf_drv_clock_lfclk_request(NULL);

    bsp_board_init(BSP_INIT_LEDS);
    
    // uart_init();

    hal_usb_cdc_init();

    dtm_error_code = dtm_init();

#if defined(NRF21540_DRIVER_ENABLE) && (NRF21540_DRIVER_ENABLE == 1)
    //Initialization of nRF21540 front-end Bluetooth® range extender chip. Do not use if your hardware doesn't support it.
    APP_ERROR_CHECK(nrf21540_init());
#endif

    if (dtm_error_code != DTM_SUCCESS)
    {
        // If DTM cannot be correctly initialized, then we just return.
        return -1;
    }

    for (;;)
    {
        hal_usb_cdc_event_queue_process( );

        // Will return every UART pool timeout,

        current_time = dtm_wait();     
        if(dtm_data_index == 1)
        {
            msb_time = current_time;
        
        }
        if (usb_rec_ok != true)
        {
            // Nothing read from the UART.
            continue;
        }

        usb_rec_ok = false;
        //if (!is_msb_read)
        //{
        //    // This is first byte of two-byte command.
        //    is_msb_read       = true;
        //    dtm_cmd_from_uart = ((dtm_cmd_t)usb_cdc_data) << 8;
        //    msb_time          = current_time;

        //    // Go back and wait for 2nd byte of command word.
        //    continue;
        //}

        //// This is the second byte read; combine it with the first and process command
        //if (current_time > (msb_time + MAX_ITERATIONS_NEEDED_FOR_NEXT_BYTE))
        //{
        //    // More than ~5mS after msb: Drop old byte, take the new byte as MSB.
        //    // The variable is_msb_read will remains true.
        //    // Go back and wait for 2nd byte of the command word.
        //    dtm_cmd_from_uart = ((dtm_cmd_t)usb_cdc_data) << 8;
        //    msb_time          = current_time;
        //    continue;
        //}

        //// 2-byte UART command received.
        //is_msb_read        = false;
        //dtm_cmd_from_uart |= (dtm_cmd_t)usb_cdc_data;
         


        dtm_cmd_from_uart = (((dtm_cmd_t)dtm_data[0]) <<8|(dtm_cmd_t)dtm_data[1]);
        if (dtm_cmd(dtm_cmd_from_uart) != DTM_SUCCESS)
        {
            // Extended error handling may be put here.
            // Default behavior is to return the event on the UART (see below);
            // the event report will reflect any lack of success.
        }

        // Retrieve result of the operation. This implementation will busy-loop
        // for the duration of the byte transmissions on the UART.
        if (dtm_event_get(&result))
        {
            // Report command status on the UART.
            // Transmit MSB of the result.
            buffer[0] = (result >> 8) & 0xFF;
            // Transmit LSB of the result.
            buffer[1] = result & 0xFF;
            hal_usb_cdc_write( buffer, 2 );

        }
    }
}

/// @}
