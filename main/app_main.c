/* SPI Slave example, sender (uses SPI master driver)

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>



#include "spi_messaging.h"
#include "spi_protocol.h"

#include "spi_callbacks.h"


//---------------------------------------------------------------------
// For other MCU implementations send and receive methods and assign them to the function pointer here. 
//---------------------------------------------------------------------
uint8_t (*send_spi_ptr)(SpiProtocolPacket* spiSendPacket) = &esp32_send_spi;
uint8_t (*recv_spi_ptr)(char* recvbuf) = &esp32_recv_spi;

uint8_t generic_send_spi(SpiProtocolPacket* spiSendPacket){
    return (*send_spi_ptr)(spiSendPacket); 
}

uint8_t generic_recv_spi(char* recvbuf){
    return (*recv_spi_ptr)(recvbuf);
}
//---------------------------------------------------------------------


void debugPrintPacket(char * packet){
    for (int i = 0; i < 256; i++)
    {
        printf("%02X", packet[i]);
    }
    printf("\n\n");
}

uint8_t spi_get_size(SpiGetSizeResp *response, char * stream_name, SpiProtocolInstance* spiProtoInstance, SpiProtocolPacket* spiSendPacket){
    uint8_t success = 0;
    printf("sending GET_SIZE cmd.\n");
    spi_generate_command(spiSendPacket, GET_SIZE, strlen(stream_name)+1, stream_name);
    generic_send_spi(spiSendPacket);

    printf("receive GET_SIZE response from remote device...\n");
    char recvbuf[BUFF_MAX_SIZE] = {0};
    uint8_t recv_success = generic_recv_spi(recvbuf);

    if(recv_success){
        if(recvbuf[0]==0xaa){
            SpiProtocolPacket* spiRecvPacket = spi_protocol_parse(spiProtoInstance, (uint8_t*)recvbuf, sizeof(recvbuf));
            spi_parse_get_size_resp(response, spiRecvPacket->data);
            success = 1;
            
        }else if(recvbuf[0] != 0x00){
            printf("*************************************** got a half/non aa packet ************************************************\n");
            success = 0;
        }
    } else {
        printf("*************************************** failed to recv packet ************************************************\n");
        success = 0;
    }

    return success;
}


uint8_t spi_get_message(SpiGetMessageResp *response, char * stream_name, uint32_t size, SpiProtocolInstance* spiProtoInstance, SpiProtocolPacket* spiSendPacket){
    uint8_t success = 0;
    printf("sending GET_MESSAGE cmd.\n");
    spi_generate_command(spiSendPacket, GET_MESSAGE, strlen(stream_name)+1, stream_name);
    generic_send_spi(spiSendPacket);

    uint32_t total_recv = 0;
    int debug_skip = 0;
    while(total_recv < size){
        if(debug_skip%20 == 0){
            printf("receive GET_MESSAGE response from remote device... %d/%d\n", total_recv, size);
        }
        debug_skip++;

        char recvbuf[BUFF_MAX_SIZE] = {0};
        uint8_t recv_success = generic_recv_spi(recvbuf);
        if(recv_success){
            if(recvbuf[0]==0xaa){
                SpiProtocolPacket* spiRecvPacket = spi_protocol_parse(spiProtoInstance, (uint8_t*)recvbuf, sizeof(recvbuf));
                uint32_t remaining_data = size-total_recv;
                if ( remaining_data < PAYLOAD_MAX_SIZE ){
                    memcpy(response->data+total_recv, spiRecvPacket->data, remaining_data);
                    total_recv += remaining_data;
                } else {
                    memcpy(response->data+total_recv, spiRecvPacket->data, PAYLOAD_MAX_SIZE);
                    total_recv += PAYLOAD_MAX_SIZE;
                }

            }else if(recvbuf[0] != 0x00){
                printf("*************************************** got a half/non aa packet ************************************************\n");
                break;
            }
        } else {
            printf("*************************************** failed to recv packet ************************************************\n");
            break;
        }
    }


/*
printf("asdfasdf %d %d\n", total_recv, size);
for (int i=0; i<total_recv; i++){
    if(i%80==0){
        printf("\n");
    }
    printf("%02x", response->data[i]);
}
printf("\n");
*/

    if(total_recv==size){
        spi_parse_get_message(response, response->data, size);
        success = 1;
    } else {
        success = 0;
    }

    return success;
}

uint8_t spi_pop_messages(SpiPopMessagesResp *response, char * stream_name, SpiProtocolInstance* spiProtoInstance, SpiProtocolPacket* spiSendPacket){
    uint8_t success = 0;

    printf("sending POP_MESSAGES cmd.\n");
    spi_generate_command(spiSendPacket, POP_MESSAGES, strlen(stream_name)+1, stream_name);
    generic_send_spi(spiSendPacket);

    printf("receive POP_MESSAGES response from remote device...\n");
    char recvbuf[BUFF_MAX_SIZE] = {0};
    uint8_t recv_success = generic_recv_spi(recvbuf);

    if(recv_success){
        if(recvbuf[0]==0xaa){
            SpiProtocolPacket* spiRecvPacket = spi_protocol_parse(spiProtoInstance, (uint8_t*)recvbuf, sizeof(recvbuf));
            spi_parse_pop_messages_resp(response, spiRecvPacket->data);
            success = 1;
            
        }else if(recvbuf[0] != 0x00){
            printf("*************************************** got a half/non aa packet ************************************************\n");
            success = 0;
        }
    } else {
        printf("*************************************** failed to recv packet ************************************************\n");
        success = 0;
    }


    return success;
}

uint8_t spi_get_streams(SpiGetStreamsResp *response, SpiProtocolInstance* spiProtoInstance, SpiProtocolPacket* spiSendPacket){
    uint8_t success = 0;
    printf("sending GET_STREAMS cmd.\n");
    spi_generate_command(spiSendPacket, GET_STREAMS, 1, "");
    generic_send_spi(spiSendPacket);

    printf("receive GET_STREAMS response from remote device...\n");
    char recvbuf[BUFF_MAX_SIZE] = {0};
    uint8_t recv_success = generic_recv_spi(recvbuf);

    if(recv_success){
        if(recvbuf[0]==0xaa){
            SpiProtocolPacket* spiRecvPacket = spi_protocol_parse(spiProtoInstance, (uint8_t*)recvbuf, sizeof(recvbuf));
            spi_parse_get_streams_resp(response, spiRecvPacket->data);
            success = 1;
            
        }else if(recvbuf[0] != 0x00){
            printf("*************************************** got a half/non aa packet ************************************************\n");
            success = 0;
        }
    } else {
        printf("*************************************** failed to recv packet ************************************************\n");
        success = 0;
    }

    return success;
}

//Main application
void app_main()
{
    uint8_t req_success = 0;

//--------------------------------------------------
// ------------------------------------------------
// testing spi_protocol.
// ------------------------------------------------

    SpiProtocolInstance* spiProtoInstance = malloc(sizeof(SpiProtocolInstance));
    SpiProtocolPacket* spiSendPacket = malloc(sizeof(SpiProtocolPacket));

//--------------------------------------------------

    // init spi for the esp32
    init_esp32_spi();

/*
    spi_device_handle_t handle;

    spi_transaction_t spi_trans;
    memset(&spi_trans, 0, sizeof(spi_trans));

    //Configuration for the SPI bus
    spi_bus_config_t buscfg={
        .mosi_io_num=GPIO_MOSI,
        .miso_io_num=GPIO_MISO,
        .sclk_io_num=GPIO_SCLK,
        .quadwp_io_num=-1,
        .quadhd_io_num=-1
    };

    //Configuration for the SPI device on the other side of the bus
    spi_device_interface_config_t devcfg={
        .command_bits=0,
        .address_bits=0,
        .dummy_bits=0,
        .clock_speed_hz=4000000, // TODO Tried 5 MHz and it became unstable?
        .duty_cycle_pos=128,        //50% duty cycle
        .mode=0,
        .spics_io_num=GPIO_CS,
        .cs_ena_posttrans=3,        //Keep the CS low 3 cycles after transaction, to stop slave from missing the last bit when CS has less propagation delay than CLK
        .queue_size=3
    };

    //GPIO config for the handshake line.
    gpio_config_t io_conf={
        .intr_type=GPIO_PIN_INTR_NEGEDGE,
        .mode=GPIO_MODE_INPUT,
        .pull_up_en=1,
        .pin_bit_mask=(1<<GPIO_HANDSHAKE)
    };

    //Create the semaphore.
    rdySem=xSemaphoreCreateBinary();

    //Set up handshake line interrupt.
    gpio_config(&io_conf);
    gpio_install_isr_service(0);
    gpio_set_intr_type(GPIO_HANDSHAKE, GPIO_PIN_INTR_NEGEDGE);
    gpio_isr_handler_add(GPIO_HANDSHAKE, gpio_handshake_isr_handler, NULL);

    //Initialize the SPI bus and add the device we want to send stuff to.
    ret=spi_bus_initialize(HSPI_HOST, &buscfg, 1);
    assert(ret==ESP_OK);
    ret=spi_bus_add_device(HSPI_HOST, &devcfg, &handle);
    assert(ret==ESP_OK);

    //Assume the slave is ready for the first transmission: if the slave started up before us, we will not detect 
    //positive edge on the handshake line.
    xSemaphoreGive(rdySem);
*/



    // init spi protocol
    spi_protocol_init(spiProtoInstance);

    // example of grabbing the available streams. they'll be the same as what's defined in the nodes.
    SpiGetStreamsResp p_get_streams_resp;
    req_success = spi_get_streams(&p_get_streams_resp, spiProtoInstance, spiSendPacket);
    printf("Available Streams: \n");
    for(int i=0; i<p_get_streams_resp.numStreams; i++){
        printf("%s\n", p_get_streams_resp.stream_names[i]);
    }

    while(1) {
//usleep(1000000);
        // do a get_size before trying to retreive message.
        SpiGetSizeResp p_get_size_resp;
        req_success = spi_get_size(&p_get_size_resp, "spimetaout", spiProtoInstance, spiSendPacket);
        printf("response: %d\n", p_get_size_resp.size);

        // get message (assuming we got size)
        if(req_success){
            SpiGetMessageResp p_get_message_resp;
            p_get_message_resp.data = malloc(p_get_size_resp.size);
            req_success = spi_get_message(&p_get_message_resp, "spimetaout", p_get_size_resp.size, spiProtoInstance, spiSendPacket);

            if(!p_get_message_resp.data){
                printf("total free %d\n", esp_get_free_heap_size());
                printf("heap_caps_get_largest_free_block: %d\n", heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
                printf("Failed to allocate memory for response of size %d.\n", p_get_size_resp.size);
                assert(0);
            }
            req_success = spi_get_message(&p_get_message_resp, "spimetaout", p_get_size_resp.size, spiProtoInstance, spiSendPacket);
            free(p_get_message_resp.data);

            if(req_success){
                SpiPopMessagesResp p_pop_messages_resp;
                req_success = spi_pop_messages(&p_pop_messages_resp, "spimetaout", spiProtoInstance, spiSendPacket);
            }
        }
    }


    //Never reached.
    deinit_esp32_spi();

}
