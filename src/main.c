/*
*   CMAS AR
*   dado transmitido: até 7 caracteres
*/

#include <stdio.h>
#include <stddef.h>
#include <string.h>
#include <zephyr/toolchain.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/gpio.h>

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS 1000
#define STACKSIZE 1024
#define PRIORITY 0
#define limiteFIFO 1000

K_FIFO_DEFINE(fifo_in);
K_FIFO_DEFINE(fifo_out);

int elementosFIFO_IN = 0;
int elementosFIFO_OUT = 0;

struct pacote {
    uint8_t lixo;
    uint8_t syn;
    uint8_t stx;
    uint8_t id;
    uint8_t N;
    char dado[7];
    uint8_t etx;
};

char tecladoIn[7];
//---------------------------------------------------------------------------
extern void enviaPacote(struct k_timer *timer_id){
    //envio de todos os dados

    //lixo: 0110 0110
    //SYN: 0001 0110
    //STX: 0000 0010
    //ID: 11011 e N: definido pela entrada
    //DADO: inserido pelo usuário
    //ETX: 0000 0011

    static struct pacote segmento;
    segmento.lixo = 0b01100110;
    segmento.syn = 0b00010110;
    segmento.stx = 0b00000010;
    segmento.id = 0b11011000 & 0xF8; //5 bits iniciais

    printk("\nMensagem de ate 7 caracteres:\n");
    scanf("%7s", tecladoIn); //confere se string tem <= 7 caracteres

    segmento.N = strlen(tecladoIn) & 0x07; // 3 bits finais
    strcpy(segmento.dado, tecladoIn); //manda info para segmento.dado
    segmento.etx = 0b00000011;

    k_fifo_put(&entradaFIFO, &tx_data); //envia info p/fifo
    elementosFIFO_IN++;
}

K_TIMER_DEFINE(timerTX, pacote, NULL);
//---------------------------------------------------------------------------
void IN(void){
    /* teclado */
    k_timer_start(&timerTX, K_MSEC(10), K_MSEC(10));
}

void TX(void){
    /* transmissão de informação */
    //timer!!! 10ms
    //Ji: tirei o while
    if(elementosFIFO >= limiteFIFO){
        while(elementosFIFO > (limiteFIFO/2)){
            //FIFO cheia
            k_msleep(SLEEP_TIME_MS);
        }
    } else{
        strcpy(tx1_data.value, "oioi!"); //manda info para tx1_data
        k_fifo_put(&my_fifo, &tx1_data); //envia info p/fifo
        elementosFIFO++;
    }
    k_msleep(SLEEP_TIME_MS);

}

void RX(void){
    /* recepção de sinal */
    struct data_item_t  *rx1_data;

    while(1){
        rx1_data = k_fifo_get(&my_fifo, K_FOREVER);

        if(rx1_data == NULL){ //ou se elementosFIFO == 0;
            printk("\n Lista vazia.\n");
        } else{
            printf("\n RX1: %s \n", rx1_data->value);
            elementosFIFO--;
        }
        printf("\nItens na fila: %d\n", elementosFIFO);

        k_msleep(SLEEP_TIME_MS);
    }
}

void OUT(void){
    /* printa informação VÁLIDA recebida */
    struct data_item_t  *rx2_data;

    while(1){
        if(K_fifo_is_empty(saidaFIFO)){
            k_msleep(SLEEP_TIMEMS);
        } else{
            
        }
        rx2_data = k_fifo_get(&my_fifo, K_FOREVER);

        if(rx2_data == NULL){ //ou se elementosFIFO == 0;
            printk("\n Lista vazia.\n");
        } else{
            printf("\n RX2: %s \n", rx2_data->value);
            elementosFIFO--;
        }
        printf("\nItens na fila: %d\n", elementosFIFO);

        k_msleep(SLEEP_TIME_MS);
    }
}

/*define threads*/
K_THREAD_DEFINE(tx_id, STACKSIZE, TX, NULL, NULL, NULL, PRIORITY, 0, 0);
//transmissão 
K_THREAD_DEFINE(rx_id, STACKSIZE, RX, NULL, NULL, NULL, PRIORITY, 0, 0);
//recepção
K_THREAD_DEFINE(in_id, STACKSIZE, IN, NULL, NULL, NULL, PRIORITY, 0, 0);
//escrita -> entrada
K_THREAD_DEFINE(out_id, STACKSIZE, OUT, NULL, NULL, NULL, PRIORITY, 0, 0);
//leitura -> saída