/*
*   CMAS AR
*   dado transmitido: até 7 caracteres
*/

#include <stdio.h>
#include <stddef.h>
#include <zephyr/toolchain.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/gpio.h>

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS 1000
#define STACKSIZE 1024
#define PRIORITY 0
#define limiteFIFO 1000

K_FIFO_DEFINE(entradaFIFO);
K_FIFO_DEFINE(saidaFIFO);

int elementosFIFO_IN = 0;
int elementosFIFO_OUT = 0;

struct data_item_t {
    void *fifo_reserved;   /* 1st word reserved for use by FIFO */
    char value[250];
};

struct data_item_t tx_data;
string tecladoIn;

void IN(void){
    /* teclado */
    while(1){
        scanf("%s", &tecladoIn);
        //confere se string tem <= 7 caracteres
        if(telcadoIn)
        strcpy(tx_data.value, tecladoIn); //manda info para tx_data

        //config FIFO 


        k_fifo_put(&entradaFIFO, &tx_data); //envia info p/fifo
        elementosFIFO_IN++;
    }
}

void TX(void){
    /* transmissão de informação */
    while(1){
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