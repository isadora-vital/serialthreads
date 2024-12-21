#include <stdio.h>
#include <stddef.h>
#include <string.h>
#include <zephyr/toolchain.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>

//DEFINIÇÕES ----------------------------------------------------------------------
/* 1000 msec = 1 sec */
#define STACKSIZE 1024
#define PRIORITY 0

// change this to any other UART peripheral if desired 
#define UART_DEVICE_NODE DT_CHOSEN(zephyr_shell_uart)

#define MSG_SIZE 9

// queue to store up to 10 messages (aligned to 4-byte boundary) 
K_MSGQ_DEFINE(uart_msgq, MSG_SIZE, 100, 4);

static const struct device *const uart_dev = DEVICE_DT_GET(UART_DEVICE_NODE);

// receive buffer used in UART ISR callback
static char rx_buf[MSG_SIZE];
static int rx_buf_pos;

//acesso ao gpiob
const struct device *stx = DEVICE_DT_GET(DT_NODELABEL(gpiob));

int tempoT = 5000;
//MSGQ ----------------------------------------------------------------------
struct pacote {
    //até 12 bytes
    uint8_t lixo; //1 byte
    uint8_t syn; //1 byte
    uint8_t stx; //1 byte
    uint8_t idN; //1 byte
    char dado[8]; //até 7 bytes
    uint8_t etx; //1 byte
};

K_MSGQ_DEFINE(msgqIn, sizeof(struct pacote), 100, 4);

struct pacote segmento;

int elementosFifoIn = 0;

//DEFINIÇÕES ----------------------------------------------------------------------
/*
 * Read characters from UART until line end is detected. Afterwards push the
 * data to the message queue.
 */
void serial_cb(const struct device *dev, void *user_data)
{
	uint8_t c, id, N;
    char dadoTemp[8] = {0}; //7 caracteres + "/0"

	if (!uart_irq_update(uart_dev)) {
		return;
	}

	if (!uart_irq_rx_ready(uart_dev)) {
		return;
	}

	segmento.lixo = 0b01100110; //é o U
    segmento.syn = 0b00010110;
    segmento.stx = 0b00000010;
    id = 0b11011; //5 bits iniciais

	/* read until FIFO empty */
	while (uart_fifo_read(uart_dev, &c, 1) == 1) {
		if ((c == '\n' || c == '\r') && rx_buf_pos > 0) {
			/* terminate string */
			rx_buf[rx_buf_pos] = '\0';

            strncpy(dadoTemp, rx_buf, sizeof(dadoTemp) - 1);

			/* reset the buffer (it was copied to the msgq) */
			rx_buf_pos = 0;
		} else if (rx_buf_pos < (sizeof(rx_buf) - 1)) {
			rx_buf[rx_buf_pos++] = c;
		}
		/* else: characters beyond buffer size are dropped */
	}
	
    N = strlen(dadoTemp);

	//ajusta o dado de entrada
    segmento.idN = (id << 3) | (N & 0x07);
    strncpy(segmento.dado, dadoTemp, sizeof(segmento.dado) - 1);
    segmento.dado[sizeof(segmento.dado) - 1] = '\0'; //termina bonitinho o array
    segmento.etx = 0b00000011;

	k_msgq_put(&msgqIn, &segmento, K_NO_WAIT);
	/* if queue is full, message is silently dropped */
	k_msgq_put(&uart_msgq, &rx_buf, K_NO_WAIT);
	
    elementosFifoIn++;
}

/*
 * Print a null-terminated string character by character to the UART interface
 */

//PRECISAREMOS DISSO PARA PRINTAR DEPOIS
void print_uart(char *buf)
{
	int msg_len = strlen(buf);

	for (int i = 0; i < msg_len; i++) {
		uart_poll_out(uart_dev, buf[i]);
	}
}

int main(void){
	if (!device_is_ready(uart_dev)) {
		printk("UART device not found!");
		return 0;
	}

	/* configure interrupt and callback to receive data */
	int ret = uart_irq_callback_user_data_set(uart_dev, serial_cb, NULL);

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

	printf("Ola!\r\n");
	printf("Insira ate 7 caracteres e aperte enter:\r\n");

    gpio_pin_configure(stx, 0x3, GPIO_OUTPUT_ACTIVE); //0x3 é o pino
}

//TIMER -----------------------------------------------------------------------------------------
void IN(struct k_timer *timer_id){
	char tx_buf[MSG_SIZE];
	char tx_buf2[MSG_SIZE];
	printf("Elementos no msgq: %d\n", elementosFifoIn);

	/* indefinitely wait for input from the user */
	if (k_msgq_get(&uart_msgq, &tx_buf, K_FOREVER) == 0 && k_msgq_get(&msgqIn, &tx_buf2, K_FOREVER) == 0) {
		elementosFifoIn--;
		printf("Echo1: ");
		print_uart(tx_buf);
		print_uart("\r\n");

		printf("Echo2 (bits): ");
	    uint8_t *segmento_pont = (uint8_t *)&segmento; // ponteiro para os dados do struct

		// Tamanho total do struct em bytes
		size_t segmento_tam = sizeof(segmento);

		for (size_t byte = 0; byte < (segmento_tam); byte++) {
			uint8_t byteAtual = segmento_pont[byte];
			
			// Processa cada bit no byte (do MSB para o LSB)
			for (int bit = 7; bit >= 0; bit--) {
				int bitEnvio = (byteAtual >> bit) & 0x01; // Isola o bit atual
				printk("%d", bitEnvio);                      // Imprime o bit no console
        		gpio_pin_set(stx, 0x3, bitEnvio); //envia bit pelo GPIO
			}
			printk(" "); // Espaço entre bytes para melhor visualização
		}
		print_uart("\r\n");
		printf("Elementos no msgq: %d\n", elementosFifoIn);
	}
	return;
}

K_TIMER_DEFINE(timerTX, IN, NULL);

void TX(void){
    k_timer_start(&timerTX, K_MSEC(tempoT), K_MSEC(tempoT));
    //Ji: pegar o que está na fifo e mandar para a TX
    // eu to perdido socorro 
}

//THREADS ----------------------------------------------------------------------------------------
K_THREAD_DEFINE(tx_id, STACKSIZE, TX, NULL, NULL, NULL, PRIORITY, 0, 0);
//transmissão 
//K_THREAD_DEFINE(rx_id, STACKSIZE, RX, NULL, NULL, NULL, PRIORITY, 0, 0);
//recepção
