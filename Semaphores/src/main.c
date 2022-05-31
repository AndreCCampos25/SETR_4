//sem
#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>
#include <drivers/pwm.h>
#include <drivers/adc.h>
#include <sys/printk.h>
#include <sys/__assert.h>
#include <timing/timing.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
/*ADC includes*/
#include <hal/nrf_saadc.h>

#define GPIO0_NID DT_NODELABEL(gpio0) 
#define PWM0_NID DT_NODELABEL(pwm0) /**< Node label do PWM */
#define BOARDLED_PIN 0x0e /**< Endereço do led da placa a ser usado */

/*ADC definitions*/
#define ADC_NID DT_NODELABEL(adc) /**< Node label da ADC */
#define ADC_RESOLUTION 10 	/**< Resolução da ADC */
#define ADC_GAIN ADC_GAIN_1_4 /**< Ganho da ADC */
#define ADC_REFERENCE ADC_REF_VDD_1_4	/**< Tensão de referência da ADC */
#define ADC_ACQUISITION_TIME ADC_ACQ_TIME(ADC_ACQ_TIME_MICROSECONDS, 40) /**< Tempo de aquisição da ADC */
#define ADC_CHANNEL_ID 1  /**< ID do canal da ADC */

/* This is the actual nRF ANx input to use. Note that a channel can be assigned to any ANx. In fact a channel can */
/*    be assigned to two ANx, when differential reading is set (one ANx for the positive signal and the other one for the negative signal) */  
/* Note also that the configuration of differnt channels is completely independent (gain, resolution, ref voltage, ...) */
#define ADC_CHANNEL_INPUT NRF_SAADC_INPUT_AIN1 /**< Entrada da ADC a utilizar */

#define BUFFER_SIZE 1 /**< Tamanho do buffer de amostragem da ADC */
#define SIZE 10 /**< Tamanho do array que guarda as amostras da ADC */

/* Size of stack area used by each thread (can be thread specific, if necessary)*/
#define STACK_SIZE 1024 /**< Tamanho da stack usada por cada thread */

/* Thread scheduling priority */
#define thread_ADC_prio 1 	/**< Prioridade de escalonamento da thread que recebe as amostras da ADC */ 
#define thread_FILTRO_prio 1 	/**< Prioridade de escalonamento da thread que atua como filtro digital */
#define thread_PWM_prio 1	/**< Prioridade de escalonamento da thread que manipula o duty-cycle do PWM */

/* Therad periodicity (in ms)*/
#define thread_ADC_period 1000 /**< Período de amostragem da ADC em milisegundos */

/* ADC channel configuration */
static const struct adc_channel_cfg my_channel_cfg = {
	.gain = ADC_GAIN,
	.reference = ADC_REFERENCE,
	.acquisition_time = ADC_ACQUISITION_TIME,
	.channel_id = ADC_CHANNEL_ID,
	.input_positive = ADC_CHANNEL_INPUT
};

/* Global vars */
struct k_timer my_timer; 
const struct device *adc_dev = NULL; /**< Ponteiro para a estrutura do tipo "device" */
static uint16_t adc_sample_buffer[BUFFER_SIZE]; /**< Incialização do array que recebe os valores da ADC  */

static uint16_t val_1;	/**< Variável que recebe o valor vindo da ADC */  
static uint16_t media_final; /**< Variável que recebe a média depois de aplicado o filtro digital*/ 

/* Takes one sample */
 /** @brief Função que retorna amostras da ADC
 *
 * Esta função lê um sinal analógico e converte o em tensão.
 * 
 * @return Tensão em milivolts.
 */  

uint16_t adc_sample(void)
{
	int ret;
	const struct adc_sequence sequence = {
		.channels = BIT(ADC_CHANNEL_ID),
		.buffer = adc_sample_buffer,
		.buffer_size = sizeof(adc_sample_buffer),
		.resolution = ADC_RESOLUTION,
	};

	if (adc_dev == NULL) {
            printk("adc_sample(): error, must bind to adc first \n\r");
            return -1;
	}

	ret = adc_read(adc_dev, &sequence);
	if (ret) {
            printk("adc_read() failed with code %d\n", ret);
	}	

	return ret;
}

/* Create thread stack space */
K_THREAD_STACK_DEFINE(thread_ADC_stack, STACK_SIZE);	/**< Cria espaço na stack para a thread_ADC*/  	
K_THREAD_STACK_DEFINE(thread_FILTRO_stack, STACK_SIZE); /**< Cria espaço na stack para a thread_FILTRO*/
K_THREAD_STACK_DEFINE(thread_PWM_stack, STACK_SIZE);	/**< Cria espaço na stack para a thread_PWM*/
  
/* Create variables for thread data */
struct k_thread thread_ADC_data; 	/**< Declaração da variável de dados para a thread_ADC */
struct k_thread thread_FILTRO_data; /**< Declaração da variável de dados para a thread_FILTRO*/
struct k_thread thread_PWM_data; 	/**< Declaração da variável de dados para a thread_PWM */

/* Create task IDs */
k_tid_t thread_ADC_tid;	/**< Task ID da thread_ADC */
k_tid_t thread_FILTRO_tid;	/**< Task ID da thread_FILTRO */
k_tid_t thread_PWM_tid;	/**< Task ID da thread_PWM */

/* Semaphores for task synch */
struct k_sem sem_val_1;	/**< Declaração do semáforo referente à variável val_1 */
struct k_sem sem_media_final;  /**< Declaração do semáforo referente à variável media_final */

/* Thread code prototypes */
void thread_ADC_code(void *argA, void *argB, void *argC); 
void thread_FILTRO_code(void *argA, void *argB, void *argC);
void thread_PWM_code(void *argA, void *argB, void *argC);

/** @brief Função main
 *
 * Aqui, são inicializados os semáforos e as threads criados. 
 * 
 */
 
void main(void)
{
     /* Create and init semaphores */
    k_sem_init(&sem_val_1, 0, 1);
    k_sem_init(&sem_media_final, 0, 1);
    
    /* Create tasks */
    thread_ADC_tid = k_thread_create(&thread_ADC_data, thread_ADC_stack,
        K_THREAD_STACK_SIZEOF(thread_ADC_stack), thread_ADC_code,
        NULL, NULL, NULL, thread_ADC_prio, 0, K_NO_WAIT);

    thread_FILTRO_tid = k_thread_create(&thread_FILTRO_data, thread_FILTRO_stack,
        K_THREAD_STACK_SIZEOF(thread_FILTRO_stack), thread_FILTRO_code,
        NULL, NULL, NULL, thread_FILTRO_prio, 0, K_NO_WAIT);

    thread_PWM_tid = k_thread_create(&thread_PWM_data, thread_PWM_stack,
        K_THREAD_STACK_SIZEOF(thread_PWM_stack), thread_PWM_code,
        NULL, NULL, NULL, thread_PWM_prio, 0, K_NO_WAIT); 

    return;
}

/* Thread code implementation */
/** @brief Thread ADC
 *
 * Esta thread é periódica. Recebe os valores da ADC num\n
 * período de 1000 milisegundos (thread_ADC_period). 
 * 
 */
void thread_ADC_code(void *argA , void *argB, void *argC)
{
  /* Timing variables to control task periodicity */
    int64_t fin_time=0, release_time=0;

    int err=0;

    /* Welcome message */
    printk("\n\r Simple adc demo for  \n\r");
    printk(" Reads an analog input connected to AN%d and prints its raw and mV value \n\r", ADC_CHANNEL_ID);
    printk(" *** ASSURE THAT ANx IS BETWEEN [0...3V]\n\r");
         
    /* ADC setup: bind and initialize */
    adc_dev = device_get_binding(DT_LABEL(ADC_NID));
	if (!adc_dev) {
        printk("ADC device_get_binding() failed\n");
    } 
    err = adc_channel_setup(adc_dev, &my_channel_cfg);
    if (err) {
        printk("adc_channel_setup() failed with error code %d\n", err);
    }
    
    NRF_SAADC->TASKS_CALIBRATEOFFSET = 1;   

    /* Compute next release instant */
    release_time = k_uptime_get() + thread_ADC_period;

    while(1) 
    {
        err=adc_sample();
        val_1=(uint16_t)(1000*adc_sample_buffer[0]*((float)3/1023));
        
        if(err) 
        {
            printk("adc_sample() failed with error code %d\n\r",err);
        }
        else 
        {
            if(adc_sample_buffer[0] > 1023) 
            {
                printk("adc reading out of range\n\r");
            }
            else 
            {
                printk("adc reading: raw:%4u /  mV: %4u \n\r",adc_sample_buffer[0],(uint16_t)(1000*adc_sample_buffer[0]*((float)3/1023)));
            }
        }

        k_sem_give(&sem_val_1);

       
        /* Wait for next release instant */ 
        fin_time = k_uptime_get();
        if( fin_time < release_time) {
            k_msleep(release_time - fin_time);
            release_time += thread_ADC_period;

        }
    }
}

/* Thread code implementation */
/** @brief Thread FILTRO
 *
 * Esta thread é esporádica (só é posta em execução quando a respetiva \n
 * variável de controlo o permite). Aqui, é feita uma média das amostras\n
 * recebidas da ADC, sendo de seguida, retiradas aquelas que possuem um\n
 * desvio de 10% da media. Por fim é calculada uma média final, com as\n 
 * amostras que sobram.
 * 
 */
void thread_FILTRO_code(void *argA , void *argB, void *argC)
{
    int idx=0,media=0,desvio=0;
    int sum_final,sum,k;
    uint16_t array[SIZE]={0,0,0,0,0,0,0,0,0,0};
    uint16_t array2[SIZE]={0,0,0,0,0,0,0,0,0,0};
    
    while(1) {
        k_sem_take(&sem_val_1,  K_FOREVER);
       
        sum=0;
        array[idx]=val_1;
        idx++;
        idx%=SIZE;
        for(int i=0;i<SIZE;i++)
        {
          sum+=array[i];
        }
        media=sum/SIZE;
        desvio=media*0.1;
        
        k=0;

        for(int j=0;j<SIZE;j++)
        {
          if(array[j]>=(media-desvio) && array[j]<=(media+desvio))
          {
            array2[k]=array[j];
            k++;
          }
        }

        sum_final=0;

        for(int l=0;l<k;l++)
        {
          sum_final+=array2[l];
	}

        if(k==0)
        {
          k=1;
          media_final=sum_final/k;
        }
        else
        {
          media_final=sum_final/k;
        }

        k_sem_give(&sem_media_final);

  }
}

/* Thread code implementation */
/** @brief Thread PWM
 *
 * Esta thread é esporádica (só é posta em execução quando a respetiva \n
 * variável de controlo o permite). Aqui, é calculado o duty-cycle do PWM\n
 * através da média calculada. 
 * 
 */
void thread_PWM_code(void *argA , void *argB, void *argC)
{

    const struct device *pwm0_dev;          /* Pointer to PWM device structure */
    unsigned int pwmPeriod_us = 1000;       /* PWM priod in us */
    unsigned int val_duty=0;

    pwm0_dev = device_get_binding(DT_LABEL(PWM0_NID));
    if (pwm0_dev == NULL) {
	printk("Error: Failed to bind to PWM0\n r");
	return;
    }
    else  {
        printk("Bind to PWM0 successfull\n\r");            
    }

    while(1) 
    {

        k_sem_take(&sem_media_final, K_FOREVER);

        val_duty=(media_final*100)/3000;
        printk("PWM DC value set to %u %%\n\r",val_duty);
        printk("Media Final mV: %u \n\r",media_final);
        
        pwm_pin_set_usec(pwm0_dev, BOARDLED_PIN, pwmPeriod_us,val_duty, PWM_POLARITY_NORMAL);

    }
}
