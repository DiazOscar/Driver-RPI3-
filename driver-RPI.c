#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/miscdevice.h> 	// misc dev
#include <linux/fs.h>         	// file operations
#include <linux/uaccess.h>     	// copy to/from user space
#include <linux/wait.h>       	// waiting queue
#include <linux/sched.h>     	// TASK_INTERRUMPIBLE
#include <linux/kfifo.h>	// KFIFO
#include <linux/jiffies.h>	// JIFFIES
#include <linux/delay.h>      	// DELAY
#include <linux/interrupt.h> 	// TASKLETS
#include <linux/gpio.h>  	// GPIO
#include <linux/timer.h>	// TIMERS

#define DRIVER_AUTHOR "Oscar Diaz - COMP"
#define DRIVER_DESC   "Driver Berryclip DSO"

//GPIOS BERRYCLIP RPI

#define GPIO_BUTTON1 2
#define GPIO_BUTTON2 3

#define GPIO_SPEAKER 4

#define GPIO_GREEN1  27
#define GPIO_GREEN2  22
#define GPIO_YELLOW1 17
#define GPIO_YELLOW2 11
#define GPIO_RED1    10
#define GPIO_RED2    9

static int LED_GPIOS[]= {GPIO_GREEN1, GPIO_GREEN2, GPIO_YELLOW1, GPIO_YELLOW2, GPIO_RED1, GPIO_RED2} ;

static char *led_desc[]= {"GPIO_GREEN1","GPIO_GREEN2","GPIO_YELLOW1","GPIO_YELLOW2","GPIO_RED1","GPIO_RED2"} ;

//COLA DE ESPERA
DECLARE_WAIT_QUEUE_HEAD(espera);
// SEMAFORO ABIERTP PARA CONCURRENCIA
DEFINE_SEMAPHORE(semaforo); 
// CONDICION DE BLOQUEO
int bloqueo=1;
//COLA KFIFO PARA PULSACIONES
DEFINE_KFIFO(lfifo, unsigned char, 64);
//MANEJADORES DE TASKLET
static void tasklet_handler1(struct tasklet_struct *);
static void tasklet_handler2(struct tasklet_struct *);
//TASKLET PARA PLANIFICAR TAREAS DE LAS PULSACIONES
DECLARE_TASKLET(tasklet1 /*nombre*/, tasklet_handler1 /*funcion*/);
DECLARE_TASKLET(tasklet2 /*nombre*/, tasklet_handler2 /*funcion*/);
//MANEJADORES PARA TIMER
static void timer_handler1(struct timer_list *);
static void timer_handler2(struct timer_list *);
//TIMER PARA CONTROLAR INTERRUPCIONES Y EVITAR REBOTES
DEFINE_TIMER(mytimer1 /*nombre*/, timer_handler1 /*funcion*/);
DEFINE_TIMER(mytimer2 /*nombre*/, timer_handler2 /*funcion*/);

// TIEMPO DE ESPERA ENTRE PULSACION EN MS (250 DEBIDO A SER EL TIEMPO DE 
// ESTIMULO MAS RAPIDO HUMANO)
unsigned int tiempo = 250;
//DECLARACION DE TIEMPO AL CARGAR MODULO 
module_param(tiempo, int, S_IRUGO);

/****************************************************************************/
/* VARIABLE DE CONTROL IRQ                                             */
/****************************************************************************/
static unsigned int irq_BUTTON1    = 0;
static unsigned int irq_BUTTON2    = 0;

// text below will be seen in 'cat /proc/interrupt' command
#define GPIO_BUTTON1_DESC           "Boton 1"
#define GPIO_BUTTON2_DESC           "Boton 2"

// below is optional, used in more complex code, in our case, this could be
#define GPIO_BUTTON1_DEVICE_DESC    "BOTONERA BERRYCLIP RPI - BOTON 1"
#define GPIO_BUTTON2_DEVICE_DESC    "BOTONERA BERRYCLIP RPI - BOTON 2"

/****************************************************************************/
/* LEDs write/read using gpio kernel API                                    */
/****************************************************************************/

static void byte2leds(char ch)
{
    int i; 
    int val=(int)ch; 
    //VARIABLE PARA EL BIT 6
    char pos6 = ((val >> 6) & 1);
    //VARIABLE PARA EL BIT 7
    char pos7 = ((val >> 7) & 1);
    //VARIABLE PARA TENER LOS BITS 7 Y 6 PARA CONTROL DE OPERACION
    char control = (pos7 << 1) | pos6;
    //MUESTRO LA OPCION ELEGIDA
    printk( KERN_INFO "Control = %i\n", control);
	
    //COMPROBACIONES DE OPCION
    if(control == 0){ //MODO 00
	    for(i=0; i<6; i++) gpio_set_value(LED_GPIOS[i], (val >> i) & 1);
	    
    }else if(control == 1){ //MODO 01 COMPRUEBO SI ESTA A 1 Y ENCIENDO
	    for(i=0; i<6; i++){
		    if(((val>>i) & 1) == 1)
		     gpio_set_value(LED_GPIOS[i], (val >> i) & 1);
	    }
	    
    }else if(control == 2){ //MODO 10 COMPRUEBO SI ESTA A 1 Y APAGO
	    for(i=0; i<6; i++){
		    if(((val>>i) & 1) == 1)
		     gpio_set_value(LED_GPIOS[i], (val >> i) & 0);
	    }
	    
    }else{ //MODO 11
	    printk( KERN_INFO "Opcion no valida\n");
    }
}

static char leds2byte(void)
{
    unsigned int val;
    char ch;
    int i;
    ch=0;

    for(i=0; i<6; i++)
    {
	//OBTENGO VALOR DE GPIO
        val=gpio_get_value(LED_GPIOS[i]);
	//GUARDO BIT EN SU POSICION CORRECTA
        ch = ch | (val << i);
    }
    
    return ch;
}

/****************************************************************************/
/* LEDs device file operations                                              */
/****************************************************************************/

static ssize_t leds_write(struct file *file, const char __user *buf,
                          size_t count, loff_t *ppos)
{

    char ch;
    //COJO LOS VALORES PROPORCIONADOS POR EL USUARIO AL BUFFER
    if (copy_from_user( &ch, buf, 1 )) {
        return -EFAULT;
    }
    //MUESTRO EL VALOR EN ESPACIO KERNEL
    printk( KERN_INFO " (write) valor recibido: %d\n",(int)ch);
    //ESCRIBO EN SUS GPIO CORRESPONDIENTE
    byte2leds(ch);

    return 1;
}

static ssize_t leds_read(struct file *file, char __user *buf,
                         size_t count, loff_t *ppos)
{
    char ch;

    if(*ppos==0) *ppos+=1;
    else return 0;
    //LEO LOS GPIO DE LOS LEDS
    ch=leds2byte();
    //MUESTRO EL VALOR EN ESPACIO KERNEL
    printk( KERN_INFO " (read) valor entregado: %d\n",(int)ch);
    //COPIAMOS A LA SALIDA ESTANDAR DEL USUARIO
    if(copy_to_user(buf,&ch,1)) return -EFAULT;

    return 1;
}
// BLOQUE DE OPERACIONES PARA MODULO KERNEL
static const struct file_operations leds_fops = {
    .owner	= THIS_MODULE,
    .write	= leds_write,
    .read	= leds_read,
};

/****************************************************************************/
/* SPEAKER device file operations                                              */
/****************************************************************************/
static ssize_t speaker_write(struct file *file, const char __user *buf,
                          size_t count, loff_t *ppos)
{

    char ch;
    //RECOJO EL VALOR DEL BUFFER DE USUARIO
    if (copy_from_user( &ch, buf, 1 )) {
        return -EFAULT;
    }

    printk( KERN_INFO " (write) valor recibido: %d\n",(int)ch);
    //COMPRUEBO SI ES 0 PARA APAGAR
    if(ch == '0')
	gpio_set_value(GPIO_SPEAKER, 0);
    else // ENCENDER
	gpio_set_value(GPIO_SPEAKER, 1);

    return 1;
}
// BLOQUE DE OPERACIONES PARA MODULO KERNEL
static const struct file_operations speaker_fops = {
    .owner	= THIS_MODULE,
    .write	= speaker_write,
};

/****************************************************************************/

/* BUTTONS device file operations                                              */
/****************************************************************************/

static ssize_t button_read(struct file *file, char __user *buf,
                         size_t count, loff_t *ppos)
{

    unsigned int buffer;
    unsigned int len_kfifo;
    
    // BLOQUEO EL SEMAFORO  
    if (down_interruptible(&semaforo)) return -ERESTARTSYS;

    /* NO FUNCIONA CORRECTAMENTE
     * len_kfifo = kfifo_len(&lfifo);
     * if (len_kfifo == 0) bloqueo=1; 
     * 
     */
    
    while (bloqueo) // COMPROBAMOS SI NOS TENEMOS QUE BLOQUEAR
    { 
	    up(&semaforo); // LIBERO EL SEMAFORO
	    printk( KERN_INFO " (read) comienza bloqueo\n");
	    //HAGO WAIT Y REEVALUO LA CONDICION DE BLOQUEO
	    if (wait_event_interruptible(espera, !bloqueo))  return -ERESTARTSYS; 
	    //VUELVO A BLOQUEAR EL SEMAFORO
	    if (down_interruptible(&semaforo)) return -ERESTARTSYS;
    }
    
    printk( KERN_INFO " (read) fin bloqueo\n");
    //RECOJO EL TAMAÑO DE LA COLA
    len_kfifo = kfifo_len(&lfifo);
    //MODIFICO CONDICION DE BLOQUEO
    bloqueo=1; 
    //LIBERO EL SEMAFORO
    up(&semaforo);
	
    //COMPRUEBO SI LA CANTIDAD DE VALORES ES MEJOR QUE EL TAMAÑO DE LA COLA
    if(count<len_kfifo){
	len_kfifo = count;
    }
    
    //MUESTRO POR LA SALIDA ESTANDAR DEL USUARIO LOS DATOS DE LA COLA
    if(kfifo_to_user(&lfifo,buf,len_kfifo,&buffer))return -EFAULT; 
    
    //DEVUELVO LA CANTIDAD DE DATOS A MOSTRAR
    return len_kfifo;
}
// BLOQUE DE OPERACIONES PARA MODULO KERNEL
static const struct file_operations button_fops = {
    .owner	= THIS_MODULE,
    .read	= button_read,
};

/****************************************************************************/
/* ESTRUCTURA DE MISC PARA PROPORCIONAR NOMBRE, OPERACIONES Y MODOS AL DRIVER*/
static struct miscdevice leds_miscdev = {
    .minor	= MISC_DYNAMIC_MINOR,
    .name	= "leds",
    .fops	= &leds_fops,
    .mode       = S_IRUGO | S_IWUGO, 	//LECTURA ESCRITURA
};

static struct miscdevice speaker_miscdev = {
    .minor	= MISC_DYNAMIC_MINOR,
    .name	= "speaker",
    .fops	= &speaker_fops,
    .mode       = S_IWUGO, 		//ESCRITURA
};

static struct miscdevice button_miscdev = {
    .minor	= MISC_DYNAMIC_MINOR,
    .name	= "buttons",
    .fops	= &button_fops,
    .mode       = S_IRUGO,		//LECTURA
};

/****************************************************************************/
/* MANEJADOR DEL TIMER - CUANDO SE INVOCA ACTIVA LA IRQ                     */
/****************************************************************************/

static void timer_handler1(struct timer_list *ptr)
{
    enable_irq(irq_BUTTON1);
    
}

static void timer_handler2(struct timer_list *ptr)
{
    enable_irq(irq_BUTTON2);
    
}

/****************************************************************************/
/* MANEJADOR TASKLET                                       */
/****************************************************************************/

static void tasklet_handler1(struct tasklet_struct *ptr){
    down(&semaforo); // BLOQUEAMOS EL SEMAFORO
    bloqueo=0; // DESACTIVA LA CONDICION DE BLOQUEO
    kfifo_put(&lfifo,'1'); //ESCRIBE 1 EN LA COLA KFIFO
    up(&semaforo); // DESBLOQUEA EL SEMAFORO
    printk( KERN_INFO " (interrupcion) despierta procesos\n");
    wake_up(&espera); //DESPIERTA A LOS PROCESOS EN LA LISTA ESPERA
}

//MISMO COMPORTAMIENTO QUE TASKLET_HANDLER1 PERO CON EL 2º BOTON
static void tasklet_handler2(struct tasklet_struct *ptr){
    down(&semaforo); 
    bloqueo=0; 
    kfifo_put(&lfifo,'2');
    up(&semaforo);
    printk( KERN_INFO " (interrupcion) despierta procesos\n");
    wake_up(&espera); 
}
/****************************************************************************/
/* MANEJADOR DE IRQ - INTERRUPCIONES                                      */
/****************************************************************************/
static irqreturn_t r_irq_handler1(int irq, void *dev_id, struct pt_regs *regs) {
    //CALCULO EL TIEMPO DE ESPERA CON LOS JIFFIES
    unsigned long ticks_pulsacion = msecs_to_jiffies(tiempo);
    //DESACTIVO IRQ 
    disable_irq_nosync(irq_BUTTON1);
    //PLANIFICO EL TIMER PARA EL TIEMPO ACTUAL + LA ESPERA
    mod_timer(&mytimer1, jiffies + ticks_pulsacion);  
    //PLANIFICO LA TAREA DE LA PULSACION
    tasklet_schedule(&tasklet1);
 
    return IRQ_HANDLED;
}
//IGUAL QUE EN EL MANEJADOR ANTERIOR PERO PARA EL BOTON 2
static irqreturn_t r_irq_handler2(int irq, void *dev_id, struct pt_regs *regs) {

    unsigned long ticks_pulsacion = msecs_to_jiffies(tiempo);
    disable_irq_nosync(irq_BUTTON2);
    mod_timer(&mytimer2, jiffies + ticks_pulsacion);  
    tasklet_schedule(&tasklet2);
   
 
    return IRQ_HANDLED;
}

/*****************************************************************************/
/* This functions registers devices, requests GPIOs and configures interrupts */
/*****************************************************************************/

/*******************************
 *  REGISTRO DE LOS PERIFERICOS
 *******************************/

static int r_dev_config(void)
{
    int ret=0;
    ret = misc_register(&leds_miscdev); //LEDS
    if (ret < 0) {
        printk(KERN_ERR "misc_register failed\n");
    }else
	printk(KERN_NOTICE "misc_register OK... leds_miscdev.minor=%d\n", leds_miscdev.minor);
    
    ret = misc_register(&speaker_miscdev); //SPEAKER
    if (ret < 0) {
        printk(KERN_ERR "misc_register failed\n");
    }else
	printk(KERN_NOTICE "misc_register OK... speaker_miscdev.minor=%d\n", speaker_miscdev.minor);
    
    ret = misc_register(&button_miscdev); //BOTONES
    if (ret < 0) {
        printk(KERN_ERR "misc_register failed\n");
    }else
	printk(KERN_NOTICE "misc_register OK... button_miscdev.minor=%d\n", button_miscdev.minor);
	
    return ret;
}

/*******************************
 *  SOLICITAR E INICIALIZAR GPIOS
 *******************************/

static int r_GPIO_config(void)
{
    int i;
    int res=0;
    for(i=0; i<6; i++)
    {
	printk(KERN_NOTICE "CONFIG LED%i...\n", i);

        if ((res=gpio_request_one(LED_GPIOS[i], GPIOF_INIT_LOW, led_desc[i]))) //SOLICITUD DE CADA LED
        {
            printk(KERN_ERR "GPIO request faiure: led GPIO %d %s\n",LED_GPIOS[i], led_desc[i]);
            return res;
        }
        gpio_direction_output(LED_GPIOS[i],0); //CONFIGURADO COMO SALIDA Y APAGADO
	printk(KERN_NOTICE "CONFIG LED%i - DONE\n", i);
    }
    printk(KERN_NOTICE "CONFIG SPEAKER...\n");
    if ((res=gpio_request_one(GPIO_SPEAKER, GPIOF_INIT_LOW, "SPEAKER")))  //SOLICITUD DE SPEAKER
    {
	printk(KERN_ERR "GPIO request faiure: speaker GPIO %d %s\n",GPIO_SPEAKER, "SPEAKER");
	return res;
    }
    gpio_direction_output(GPIO_SPEAKER,0); //CONFIGURADO COMO SALIDA Y APAGADO
    printk(KERN_NOTICE "CONFIG SPEAKER - DONE\n");
    
    printk(KERN_NOTICE "CONFIG BUTTON1...\n");
    if ((res=gpio_request_one(GPIO_BUTTON1, GPIOF_INIT_LOW, GPIO_BUTTON1_DESC)))  //SOLICITUD DE BOTON1
    {
	printk(KERN_ERR "GPIO request faiure: BUTTON1 GPIO %d %s\n",GPIO_BUTTON1, "BUTTON1");
	return res;
    }
    gpio_direction_input(GPIO_BUTTON1); //CONFIGURADO COMO ENTRADA
    printk(KERN_NOTICE "CONFIG BUTTON1 - DONE\n");
    
    printk(KERN_NOTICE "CONFIG BUTTON2...\n");
    if ((res=gpio_request_one(GPIO_BUTTON2, GPIOF_INIT_LOW, GPIO_BUTTON2_DESC))) //SOLICITUD DE BOTON2
    {
	printk(KERN_ERR "GPIO request faiure: BUTTON2 GPIO %d %s\n",GPIO_BUTTON2, "BUTTON2");
	return res;
    }
    gpio_direction_input(GPIO_BUTTON2);//CONFIGURADO COMO ENTRADA
    printk(KERN_NOTICE "CONFIG BUTTON2 - DONE\n");
    
    return res;
}

/*******************************
 *  ESTABLECER INTERRUPCIONES PARA BOTONES
 *******************************/

static int r_int_config(void)
{
    int res=0;
    printk(KERN_NOTICE "CONFIG INTERRUPT BUTTON1...\n");
    if ( (irq_BUTTON1 = gpio_to_irq(GPIO_BUTTON1)) < 0 ) { //CONFIGURAR GPIO BOTON 1 COMO IRQ
        printk(KERN_ERR "GPIO to IRQ mapping faiure %s\n", GPIO_BUTTON1_DESC);
        return irq_BUTTON1;
    }

    printk(KERN_NOTICE "  Mapped int %d for button1 in gpio %d\n", irq_BUTTON1, GPIO_BUTTON1);

    // DECLARACION DE IRQ
    if ((res=request_irq(irq_BUTTON1,
                    (irq_handler_t ) r_irq_handler1,
                    IRQF_TRIGGER_FALLING,
                    GPIO_BUTTON1_DESC,
                    GPIO_BUTTON1_DEVICE_DESC))) {
        printk(KERN_ERR "Irq Request failure\n");
        return res;
    }

   printk(KERN_NOTICE "CONFIG INTERRUPT BUTTON2...\n");
    if ( (irq_BUTTON2 = gpio_to_irq(GPIO_BUTTON2)) < 0 ) { //CONFIGURAR GPIO BOTON 2 COMO IRQ
        printk(KERN_ERR "GPIO to IRQ mapping faiure %s\n", GPIO_BUTTON2_DESC);
        return irq_BUTTON2;
    }

    printk(KERN_NOTICE "  Mapped int %d for button2 in gpio %d\n", irq_BUTTON2, GPIO_BUTTON2);
    
    //DECLARACION DE IRQ
    if ((res=request_irq(irq_BUTTON2,
                    (irq_handler_t ) r_irq_handler2,
                    IRQF_TRIGGER_FALLING,
                    GPIO_BUTTON2_DESC,
                    GPIO_BUTTON2_DEVICE_DESC))) {
        printk(KERN_ERR "Irq Request failure\n");
        return res;
    }

    return res;

}

/****************************************************************************/
/* Module init / cleanup block.                                             */
/****************************************************************************/
//PARA LIMPIAR EL MODULO DEL SO 
static void r_cleanup(void) {
    int i;
    printk(KERN_NOTICE "%s module cleaning up...\n", KBUILD_MODNAME);
    
    for(i=0; i<6; i++)
        gpio_free(LED_GPIOS[i]); 	//LIBERO GPIO LED 
    gpio_free(GPIO_SPEAKER);		//LIBERO GPIO SPEAKER
    gpio_free(GPIO_BUTTON1);		//LIBERO GPIO BOTON1
    gpio_free(GPIO_BUTTON2);		//LIBERO GPIO BOTON2
    
    //DESCONFIGURO LOS DISPOSITIVOS DE CADA PERIFERICO
    if (leds_miscdev.this_device) misc_deregister(&leds_miscdev);
    if (speaker_miscdev.this_device) misc_deregister(&speaker_miscdev);
    if (button_miscdev.this_device) misc_deregister(&button_miscdev);
    
    //LIBERO LAS IRQ
    if(irq_BUTTON1) free_irq(irq_BUTTON1, GPIO_BUTTON1_DEVICE_DESC);
    if(irq_BUTTON2) free_irq(irq_BUTTON2, GPIO_BUTTON2_DEVICE_DESC);
    
    //ELIMINO LA COLA
    kfifo_free(&lfifo);
    
    //ACABO CON LAS TAREAS PENDIENTES
    tasklet_kill(&tasklet1);
    tasklet_kill(&tasklet2);
    //ELIMINO LOS TIMERS
    del_timer(&mytimer1);
    del_timer(&mytimer2);
    
    printk(KERN_NOTICE "Done. Bye from %s module\n", KBUILD_MODNAME);
    return;
}

static int r_init(void) {
	int res=0;
    printk(KERN_NOTICE "Hello, loading %s module!\n", KBUILD_MODNAME);
    printk(KERN_NOTICE "%s - devices config...\n", KBUILD_MODNAME);

    //CONFIGURO LOS DISPOSITIVOS COMO DRIVERS
    if((res = r_dev_config()))
    {
	    r_cleanup();
	    return res;
    }
    printk(KERN_NOTICE "%s - GPIO config...\n", KBUILD_MODNAME);
    
    //CONFIGURO LOS GPIO
    if((res = r_GPIO_config()))
    {
	    r_cleanup();
	    return res;
    }
    
    //CONFIGURO INTERRUPCIONES
    if((res = r_int_config()))
    {
	    r_cleanup();
	    return res;
    }

    return res;
}

module_init(r_init);
module_exit(r_cleanup);

/****************************************************************************/
/* Module licensing/description block.                                      */
/****************************************************************************/
MODULE_LICENSE("GPL");
MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
