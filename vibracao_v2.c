// Programa para os Testes de Vibração do RGL
// Funções: Controle de pino de RST
//          Verificação periódica de saúde do RGL
//          Geração de Logs
//          


// Pino SODIMM-127 -> RST HackRF
// Pino SODIMM-129 -> PWR HackRF

// ##### Bibliotecas #####
#include <stdio.h>
#include <unistd.h>
#include <gpiod.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include "HEtechSerial.h"

// #####  HackRF  #####
// #include "hackrf.h"
// #include "hackrf_info.h"

// void hackrf_rst(void);

// #####  Main  #####
int main(){
    
    // hackrf_device* device;
    // read_partid_serialno_t read_partid_serialno;

    // ##### Time #####
    struct tm* ptr;
    time_t start, now, end;
    start = time(NULL);

    // ##### GPIO #####
    // Obs: 2 gpio's
    // SODIMM 127 ------> Reset
    // SODIMM 129 ------> Power UP
    int line_value = 0;
    int line;
	int ret;
	char chip[10];
	unsigned int offset;
    const char* gpiochip4 = "4";
    int SODIMM127 = 5;
    int SODIMM129 = 3;

    // ##### File #####
    FILE* fptr;
    fptr = fopen("Vibe_Test_Log.txt","w+");
    snprintf(chip, sizeof(chip), "gpiochip%s", gpiochip4);
    offset = SODIMM127;
    gpiod_ctxless_set_value(chip, offset, 0, false, "gpio-toggle", NULL, NULL); // Lower the hackRF RST_pin

    // ##### Time  #####
    now = time(NULL);
    ptr = gmtime(&now);

    // ##### HackRF_CHECK #####
    // hackrf_info board_info;
	// board_info.init = hackrf_init();
	// if (board_info.init != HACKRF_SUCCESS) {
	// 	fprintf(fptr, "hackrf_init() failed: %s (%d)\n", hackrf_error_name(board_info.init), board_info.init);
    //     hackrf_rst();
    // }

    // ##### UART #####
    char buff_tx[100];
    memset(&buff_tx,'\0', sizeof(buff_tx));
    char buff_aux[100];
    memset(&buff_aux,'\0',sizeof(buff_aux));
    int ID_Unit = 0xD4;
    int ID_msg = 0x4F;
    double lat = -23.17944, longit = -45.88694;
    int counter = 0;
    int bytes;
    // unsigned char crc = getCRC((unsigned char*)&buff_aux[0],100); char* ptr=strchr(buff_aux,'\0');
    // buff_aux[ptr-buff_aux+1]=crc;
    //    // char *command = "hackrf_info";
    //    // char *argument_list[] = {"", "", NULL};
    //    // int status_code;
    int temp = 1048575;
    while (1)
    {
        // ##### UART - Write #####
        // status_code = execvp(command, argument_list);
        // printf("tá vivo? %d",status_code);
        // sleep(1);
        // hackrf_board_partid_serialno_read(device, &read_partid_serialno);
        ptr = gmtime(&now);
        sprintf((char *)buff_aux,
                "%s|Counter:%d|%#x|%#x|Part_ID Number: 0x%08x 0x%08x|%d|%d\n",
                asctime(ptr),
                counter,
                ID_Unit,
                ID_msg,
                temp,temp,
                // read_partid_serialno.part_id[0],
                // read_partid_serialno.part_id[1],
                lat,
                longit);
        bytes = serial4send(&buff_aux[0]);
        if (bytes >= 0)
        {
            printf("Msg Enviada, %d Bytes\n\r", bytes);
        }
        else
        {
            printf("Erro ao Enviar a Msg\n\r");
        }
        fprintf(fptr, "%s | counter:%d | msg: %s\n\r", asctime(ptr), counter, buff_aux);
        counter++;
    }

    fclose(fptr);
    return 0;
}

// void hackrf_rst(void){
//     bool line_value = 0;
//     int line;
//     int ret;
//     char chip[10];
//     unsigned int offset;
//     const char *gpiochip4 = "4";
//     int SODIMM127 = 5;
//     int SODIMM129 = 3;
//     snprintf(chip, sizeof(chip), "gpiochip%s", gpiochip4);
//     offset = SODIMM127;
//     gpiod_ctxless_set_value(chip, offset, !line_value, false, "gpio-toggle", NULL, NULL); // Lower the hackRF RST_pin
//     usleep(100000);
//     gpiod_ctxless_set_value(chip, offset, line_value, false, "gpio-toggle", NULL, NULL);
// }
