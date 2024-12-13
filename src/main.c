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

#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS 1000
#define STACKSIZE 1024
#define PRIORITY 0

//FIFO e UART -----------------------------------------------------------------
#define limiteFIFO 1000

K_FIFO_DEFINE(entradaFIFO);
K_FIFO_DEFINE(saidaFIFO);

int elementosFIFO_IN = 0;
int elementosFIFO_OUT = 0;

/* change this to any other UART peripheral if desired */
#define UART_DEVICE_NODE DT_CHOSEN(zephyr_shell_uart)

#define MSG_SIZE 32 //tem que

static const struct device *const uart_dev = DEVICE_DT_GET(UART_DEVICE_NODE);

/* receive buffer used in UART ISR callback */
static char rx_buf[MSG_SIZE];
static int rx_buf_pos;

//
const struct device *stx = DEVICE_DT_GET(DT_NODELABEL(gpiob));
//----------------------------------------------------------------------------
//tempo timer
int tempoT = 5000;

struct pacote {
    //até 12 bytes
    uint8_t lixo;
    uint8_t syn;
    uint8_t stx;
    uint8_t id;
    uint8_t N;
    char dado[7];
    uint8_t etx;
};

char tecladoIn[7];

int aux, tamanhoDado, i = 0, pegarFifo = 0;
struct data_item_t {
    void *fifo_reserved;   /* 1st word reserved for use by FIFO */
    char value[100];
};
//---------------------------------------------------------------------------
/*
 * Read characters from UART until line end is detected. Afterwards push the
 * data to the message queue.
 */
void serial_cb(const struct device *dev, void *user_data, struct k_timer *timer_id)
{
	uint8_t c;

	if (!uart_irq_update(uart_dev)) {
		return;
	}

	if (!uart_irq_rx_ready(uart_dev)) {
		return;
	}

    //envio de todos os dados

    //lixo: 0110 0110
    //SYN: 0001 0110
    //STX: 0000 0010
    //ID: 11011 e N: definido pela entrada
    //DADO: inserido pelo usuário
    //ETX: 0000 0011

    static struct pacote segmento;
    segmento.lixo = 0b01100110; //é o U
    segmento.syn = 0b00010110;
    segmento.stx = 0b00000010;
    segmento.id = 0b11011000 & 0xF8; //5 bits iniciais

    /* ECHO_BOT
    segmento.N = strlen(tecladoIn) & 0x07; // 3 bits finais
    strcpy(segmento.dado, tecladoIn); //manda info para segmento.dado

    tamanhoDado = strlen(DADO_DIGITADO);
    */
    segmento.etx = 0b00000011;

	//lê até a FIFO encher
	while (uart_fifo_read(uart_dev, &c, 1) == 1) {
		if ((c == '\n' || c == '\r') && rx_buf_pos > 0) {
			/* terminate string */
			rx_buf[rx_buf_pos] = '\0';

			/* if queue is full, message is silently dropped */
			k_fifo_put(&entradaFIFO, &rx_buf);

			/* reset the buffer (it was copied to the msgq) */
			rx_buf_pos = 0;
		} else if (rx_buf_pos < (sizeof(rx_buf) - 1)) {
			rx_buf[rx_buf_pos++] = c;
		}
		/* else: characters beyond buffer size are dropped */
	}
    
    //ERRADO AINDA
    k_fifo_put(&entradaFIFO, &segmento); //envia 1 pacote completo p/fifo
    elementosFIFO_IN++;
}

//---------------------------------------------------------------------------
/*extern void enviaPacote(){
    //envio de todos os dados

    //lixo: 0110 0110
    //SYN: 0001 0110
    //STX: 0000 0010
    //ID: 11011 e N: definido pela entrada
    //DADO: inserido pelo usuário
    //ETX: 0000 0011

    static struct pacote segmento;
    segmento.lixo = 0b01100110; //é o U
    segmento.syn = 0b00010110;
    segmento.stx = 0b00000010;
    segmento.id = 0b11011000 & 0xF8; //5 bits iniciais

    //ECHO_BOT
    printk("\nMensagem de ate 7 caracteres:\n");
    scanf("%7s", tecladoIn); //confere se string tem <= 7 caracteres

    segmento.N = strlen(tecladoIn) & 0x07; // 3 bits finais
    strcpy(segmento.dado, tecladoIn); //manda info para segmento.dado

    tamanhoDado = strlen(DADO_DIGITADO);
    
    segmento.etx = 0b00000011;

    k_fifo_put(&entradaFIFO, &segmento); //envia 1 pacote completo p/fifo
    elementosFIFO_IN++;
}
*/

K_TIMER_DEFINE(timerTX, TX, NULL);
//---------------------------------------------------------------------------
int main (void){
    gpio_pin_configure(stx, 0x3, GPIO_OUTPUT_ACTIVE); //0x3 é o pino

    char tx_buf[MSG_SIZE];

    //confere erro na UART
	if (!device_is_ready(uart_dev)) {
		printk("UART device not found!");
		return 0;
	}

	//configura a interrupção e callback para receber dados
	int ret = uart_irq_callback_user_data_set(uart_dev, serial_cb, NULL);

    //mais erros na UART
	if (ret < 0) {
		if (ret == -ENOTSUP) {
			printk("Interrupt-driven UART API support not enabled\n");
		} else if (ret == -ENOSYS) {
			printk("UART device does not support interrupt-driven API\n");
		} else {
			printk("Error setting UART callback: %d\n", ret);
		}
		return 0;
	}
	uart_irq_rx_enable(uart_dev);

	print_uart("Hola, o codigo esta comecando.\r\n");
	print_uart("Escreva algo de ate 7 caracteres:\r\n");

	//espera indefinidamente por input do usuário
	while (1) {
	
    }
    return 0;
}

void IN(void){
    /* teclado */
    k_timer_start(&timerTX, K_MSEC(tempoT), K_MSEC(tempoT));
    //Ji: pegar o que está na fifo e mandar para a TX
    // eu to perdido socorro 
}

void TX(void){
    // transmissão de informação 
    static struct pacote *transmitindo;

    if(pegarFifo <= ())
    transmitindo = k_fifo_get(&entradaFIFO, K_FOREVER);
    
    if(tamanhoDado != 0){
        if(pegarFifo <= (8*(5 + tamanhoDado))){
            pegarFifo++;
            transmitindo = transmitindo & 0b1;
            gpio_pin_set(stx, 0x3, aux); 
            transmitindo = transmitindo >> 1;
		    printk("\n RX1: %s \n", rx1_data->value);
        }
        for(int i = 0; i < (8*(5 + tamanhoDado)); i++){
            
	    }
	    printk("\nItens na fila: %d\n", elementosFIFO);
    }
}
/*
static struct pacote *transmitindo;
transmitindo = k_fifo_get(&entradaFIFO, K_FOREVER);

if(tamanhoDado != 0){
        for(int i = 0; i < (8*(5 + tamanhoDado)); i++){
            aux = transmitindo & 0b1;
            gpio_pin_set(stx, 0x3, aux); 
            transmitindo = transmitindo >> 1;
		    printk("\n RX1: %s \n", rx1_data->value);
	    }
	    printk("\nItens na fila: %d\n", elementosFIFO);
    }
*/
void RX(void){
    /* recepção de sinal */
    //se detectar o sync, pega o mutex
    //quando pegar o mutex o TX não envia nada

    //confere se é válida a recepção ou não
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