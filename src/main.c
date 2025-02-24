/*
*	SerialThreads 2.0
*/
#include <stdio.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS 1000
#define MY_STACK_SIZE 1024
#define MY_PRIORITY 0
#define BUF_TX_SIZE 1
#define BUF_RX_SIZE 4

//---------------------------------------------------------------------
int Ndados = 0, cont_byby1 = 0, cont_bibi1 = 0, deu_errado = 0;
uint8_t dado_tx = 0, dado_tx_p2 = 0;

int cont_bibi2 = 0, cont_byby2 = 0, cont_4bits = 0;
uint8_t dado_rx = 0, dado_rx_p2 = 0, byteN_rx = 0;

uint32_t dado32_rx, byte_rx = 0, recebido = 0;

int elementosFIFO = 0;

//DEFINIÇÃO UART-------------------------------------------------------
#define UART_DEVICE_NODE DT_CHOSEN(zephyr_shell_uart)

#define MSG_SIZE 8

K_MSGQ_DEFINE(uartIn_msgq, MSG_SIZE, 100, 1);

static const struct device *const uart_dev = DEVICE_DT_GET(UART_DEVICE_NODE);

// Recebe buffer usado na callback UART ISR  
static char uartIn_buf[MSG_SIZE];
static int uartIn_buf_pos = 0;
char txIn_buf[MSG_SIZE];

//DEFINIÇÃO MUTEX e CONDVAR---------------------------------------------
K_MUTEX_DEFINE(mutex_envio);
K_CONDVAR_DEFINE(condvar_envio);

K_MUTEX_DEFINE(mutex_esperaUart);
K_CONDVAR_DEFINE(condvar_esperaUart);

K_MUTEX_DEFINE(mutex_espera1);
K_CONDVAR_DEFINE(condvar_espera1);

K_MUTEX_DEFINE(mutex_recepcao);
K_CONDVAR_DEFINE(condvar_recepcao);

K_MUTEX_DEFINE(mutex_erroCSMA); // Precisa implementar

//GPIOs-----------------------------------------------------------------
 // Identificador do nó para o alias do "led1"
 #define LED0_NODE DT_ALIAS(led1)
 static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
 
 // Acesso ao gpiob
 const struct device *tx_gpio = DEVICE_DT_GET(DT_NODELABEL(gpiob));
 const struct device *rx_gpio = DEVICE_DT_GET(DT_NODELABEL(gpiob));

//DEFINIÇÃO BUFFERS-----------------------------------------------------
RING_BUF_DECLARE(bufTX, BUF_TX_SIZE); // 1 byte sentido GPIO
RING_BUF_DECLARE(bufRX, BUF_RX_SIZE); // 4 bytes sentido thread

//DEFINIÇÃO TIMERS------------------------------------------------------
void envio(struct k_timer *timerTX);
void recebimento(struct k_timer *timerRX);

K_TIMER_DEFINE(timerTX, envio, NULL);
K_TIMER_DEFINE(timerRX, recebimento, NULL);

//MSGQ------------------------------------------------------------------
struct pacote {
    //até 12 bytes
    uint8_t lixo; //1 byte - U
    uint8_t syn; //1 byte
    uint8_t stx; //1 byte
    uint8_t idN; //1 byte
    char dado[7]; //até 7 bytes, DEVE SER 7
    uint8_t etx; //1 byte
};

struct pacote segmento;
struct pacote pego;

int id = 0b11011; //5 bits iniciais

// Converte a struct para ponteiro de bytes
uint8_t *pego_pont = (uint8_t *)&pego;

// Definir msgq
K_MSGQ_DEFINE(empacotada, sizeof(struct pacote), 10, 1);
int i = 0, jaPegouDado = 0, tamPacote = sizeof(segmento);
char dadoIn[8]; // 7 caracteres + '\0'

//-----------------------------------------
void serial_cb(const struct device *dev, void *user_data)
{
	uint8_t contUart;

	if (!uart_irq_update(uart_dev)) {
		return;
	}

	if (!uart_irq_rx_ready(uart_dev)) {
		return;
	}

	// Lê até a msgq esvaziar 
	while (uart_fifo_read(uart_dev, &contUart, 1) == 1) {
        // Se inserir palavra com mais de 7 caracteres, corta a palavra
		if ((contUart == '\n' || contUart == '\r') && uartIn_buf_pos > 0) {
			uartIn_buf[uartIn_buf_pos] = '\0'; // Termina a string 

			// Se a fila estiver cheia, mensagem é inserida na msgq 
			k_msgq_put(&uartIn_msgq, &uartIn_buf, K_NO_WAIT); //NÃO ESTÀ SENDO FEITO
            elementosFIFO++;
            k_condvar_signal(&condvar_esperaUart);

			uartIn_buf_pos = 0; // Reinicia o buffer (foi copiado para a msgq) 
		} else if (uartIn_buf_pos < (sizeof(uartIn_buf) - 1)) {
			uartIn_buf[uartIn_buf_pos++] = contUart;
		}
		// Se não: caracteres além do tamanho do buffer são descartados
	}
}

void empacotadora(void *, void *, void *){
    while(1){
        k_mutex_lock(&mutex_esperaUart, K_FOREVER);

        k_condvar_wait(&condvar_esperaUart, &mutex_esperaUart, K_FOREVER);
        k_msgq_get(&uartIn_msgq, &dadoIn, K_NO_WAIT);
        
        memset(segmento.dado, 0, 7);  // Zera o buffer para evitar lixo de memória
        memcpy(segmento.dado, dadoIn, 7);  // Copia exatamente 7 bytes, sem o \0, para tx_data

        Ndados = strlen(dadoIn);
        //printk("\n %s \n", segmento.dado);
        segmento.idN = (id << 3) | (Ndados & 0x07);

        k_msgq_put(&empacotada, &segmento, K_NO_WAIT);
        dadoIn[0] = '\0'; // Ficar atenta
    }
} 

void bufInTX(void *, void *, void *){
    while(1){
        k_mutex_lock(&mutex_envio, K_FOREVER); 

		// cont_byby1 faz o controle dos bytes enviados
		// Pego é um pacote, 6-12 bytes

		if(cont_byby1 == 0 && deu_errado == 0){
			k_msgq_get(&empacotada, &pego, K_NO_WAIT);
			printk("Retirado MSGQ | ");
            printk("N dados: %d ", Ndados);
		}
         /*
        else if(k_mutex_lock(&mutex_erroCSMA, K_NO_WAIT)){
        // SE O MUTEX ESTIVER LIVRE(?) REINICIA 
            k_mutex_unlock(&mutex_erroCSMA);
            k_msgq_get(&empacotada, &pego, K_NO_WAIT);
			printk("Retirado MSGQ | ");
            printk("N dados: %d ", Ndados);
        }
        */

        dado_tx = pego_pont[cont_byby1]; // Pega determinado byte do dado
        printk("\nDado_tx: %d | ", dado_tx);

        uint8_t ret;

		// Só pode mandar quando "envio" sinalizar
		// "Buffer livre"
        ret = ring_buf_put(&bufTX, &dado_tx, BUF_TX_SIZE);
        if (ret != BUF_TX_SIZE) {
            //printk("Copia parcial de dados.\n");
            // Sem espaço suficiente, cópia parcial  
			if(cont_byby1 == 0){
				deu_errado = 1;
			}
        } else{
            //printk("Envio feito.\n");
			cont_byby1++;
			// Controla o envio do 1º byte
			deu_errado = 0;
        }

        k_mutex_lock(&mutex_espera1, K_FOREVER);
		if(cont_byby1 == (tamPacote)){
            k_condvar_wait(&condvar_espera1, &mutex_espera1, K_FOREVER);

            // Zera contadores e reestaura valores e infos para nulo 
			cont_byby1 = 0;
            Ndados = 0;
            memset(pego.dado, 0, 7);  // Esvazia dado ao finalizar envio
            pego.idN = (id << 3) | (Ndados & 0x07); 
			printk("\nContador reiniciado\n");
		}

        k_condvar_wait(&condvar_envio, &mutex_envio, K_FOREVER);
    }
 }

 void envio(struct k_timer *timerTX){
    if(k_mutex_lock(&mutex_envio, K_NO_WAIT)){ // Se pegou mutex: 
        // Altera o estado do led
        gpio_pin_toggle_dt(&led);

        // Solta mutex
        k_mutex_unlock(&mutex_envio);

        // Confere dado_tx_p2 
        if(jaPegouDado == 0){
            int ret = ring_buf_get(&bufTX, &dado_tx_p2, BUF_TX_SIZE);
            if (ret == BUF_TX_SIZE) {
                // printk("Dado recebido: 0x%X | ", dado_tx_p2); // Printa em hexadecimal
                // Envio do bit menos significativo
                int bit = (dado_tx_p2 >> cont_bibi1) & 0x1;
                printk("Info: %d\n", bit);
                gpio_pin_set(tx_gpio, 3, bit);
                cont_bibi1++;
                jaPegouDado = 1;
            } else {
                printk("Falha ao obter dados do buffer.\n");
                k_condvar_signal(&condvar_envio);
                return;
            }
        } else{
            // Envio do bit menos significativo
            int bit = (dado_tx_p2 >> cont_bibi1) & 0x1;
            printk("Info: %d\n", bit);
            gpio_pin_set(tx_gpio, 0x3, bit);
            if(cont_bibi1 < 7){
                cont_bibi1++;
            } else {
                k_condvar_signal(&condvar_espera1);
                cont_bibi1 = 0;
                jaPegouDado = 0;
                k_condvar_signal(&condvar_envio);
            }
        }
    } else{
        // Não pegou o mutex - printk("Nao pegou mutex\n");
        k_mutex_unlock(&mutex_envio);
        k_condvar_signal(&condvar_envio);
    }
 }

 void recebimento(struct k_timer *timerRX){
    // Recebe info da GPIO
    int bitRX = gpio_pin_get(rx_gpio, 0x2);
    //printk("Recebido: %d \n", bitRX);

    if(cont_bibi2 < 32){
        recebido = (recebido << 1) | bitRX;
        cont_bibi2++;
    } else{
        cont_bibi2 = 0;
        int retRX = ring_buf_put(&bufRX, (uint8_t *)&recebido, BUF_RX_SIZE);
        if (retRX == BUF_RX_SIZE) {
            //printk("Certo 22222\n");
        } else{
            //printk("Nao certo 2222\n");
        }
        k_condvar_signal(&condvar_recepcao);
    }
 }

 //RECEPCAO AAAAAAAAAAAAAAAAAAAAAAAAAAAAAa
 // Aqui são conferidos os dados se batem os do meio
 void bufInRX(void *, void *, void *){
    while(1){
        k_mutex_lock(&mutex_erroCSMA, K_FOREVER);
        k_mutex_lock(&mutex_recepcao, K_FOREVER);
        k_condvar_wait(&condvar_recepcao, &mutex_recepcao, K_FOREVER);

        // Só pode pegar novo dado quando terminar de conferir dado atual
        int retRX = ring_buf_get(&bufRX, (uint8_t *)&dado32_rx, BUF_RX_SIZE);
        if(retRX == BUF_RX_SIZE){
            //printk("\nCERTOOO\n");
        } else{
            //printk("\nnao certo :( :(\n");
        }
        //printk("AQUIII: %u\n", dado32_rx);

        // byteN_rx será enviado para uma msgq
        // Divide 32 bits em conjuntos de 4 = 8 conjuntos
        if(cont_4bits < 8){
            byte_rx = (dado32_rx >> (4 * cont_4bits)) & 0xF; // 0XF = 0b1111
            printk("\n byte_rx = %d\n", byte_rx);
            // As operações estão corretas! :)

            if(byte_rx == 9 || byte_rx == 8 || byte_rx == 1 || byte_rx == 0){
                // Dado do tipo _00_
                byteN_rx = (byteN_rx << 1) | 0;
                cont_4bits++;
            } else if(byte_rx == 15 || byte_rx == 14 || byte_rx == 7 || byte_rx == 6){
                // Dado do tipo _11_
                byteN_rx = (byteN_rx << 1) | 1;
                cont_4bits++;
            } else{
                // Dado inválido -> cancela o envio do pacote inteiro
                cont_4bits = 0;
                //k_mutex_unlock(&mutex_erroCSMA);
            }
        } else{
            cont_4bits = 0;
            //printk("\nFINAL: %d !!!\n", byteN_rx);
            // Insere dado na fifo
        }
    }
 }

//----------------------------------------------------------
 int main(void){
    segmento.lixo = 0b01100110; //U, em ASCII: f
    segmento.syn = 0b00010110; //22
    segmento.stx = 0b00000010; //2
    segmento.idN = (id << 3) | (Ndados & 0x07);
    segmento.etx = 0b00000011; //3
    k_msgq_put(&empacotada, &segmento, K_NO_WAIT);

	if (!device_is_ready(uart_dev)) {
		printk("UART device not found!");
		return 0;
	}
	
	// Configura interrupção e callback para receber dados 
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

    // Configuração do led
    if (!gpio_is_ready_dt(&led)) {
        return 0;
    }
    gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
   
    // Configuração TX e RX
    gpio_pin_configure(tx_gpio, 3, GPIO_OUTPUT_ACTIVE); // PTB3 = tx_gpio
    gpio_pin_configure(rx_gpio, 2, GPIO_INPUT); // PTB2 = rx_gpio
    
    // Inicia o timer se conseguir conexão com o GPIO
	k_timer_start(&timerTX, K_NSEC(10000000), K_NSEC(10000000));
	k_timer_start(&timerRX, K_NSEC(2500000), K_NSEC(2500000));

    //while(1){}
    return 0;
 }

// Def. e inicia threads
K_THREAD_DEFINE(encaminhaDados, MY_STACK_SIZE, bufInTX, NULL, NULL, NULL, MY_PRIORITY, 0, 0);
K_THREAD_DEFINE(empacota, MY_STACK_SIZE, empacotadora, NULL, NULL, NULL, MY_PRIORITY, 0, 0);
K_THREAD_DEFINE(integridadeDados, MY_STACK_SIZE, bufInRX, NULL, NULL, NULL, MY_PRIORITY, 0, 0);
