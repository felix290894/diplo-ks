/*
 * Este programa configura el UART en bajo nivel accediendo directamente a sus 
 * registros. Para realizar esto, de debe hacer una traduccion de las direcciones fisicas
 * de los perifericos en el procesador a las direcciones virtuales que maneja el kernel.
 * Problemas encontrados:
 * 1. El UART del controlador PL011 esta usado por el módulo BLUETOOTH.
 * 2. La IRQ del Uart debe habilitarse en el archivo /boot/config.txt con uart_enable = 1.
 * 3. La IRQ tiene asignado el numero 59. Esto se observa en /proc/interrupts.
 * 4. Esta IRQ esta compartida con otros modulos, por lo que se debe usar la bandera IRQF_SHARED.
 * 5. Se debe usar un identificador de dispositivo unico para hacer referencia al manejador en request_irq
 * 	  Ese mismo manejador se debe usar con free_irq.
 * 6. La direccion base del UART para la Raspberry Pi 3 no se encuentra en la hoja de datos del BCM2835.
 * 7. Se tuvo que deducir esta direccion a partir del archivo /proc/iomem. La direccion base no se muestra
 *    de forma clara.
 * 8. Para acceder a las registros se debe solicitar su direccion virtual con ioremap, esto debido a la
 * 	  E/S mapeada que tiene el ARM. 
 * 9. Se debe configurar el reloj del UART en el archivo /boot/config.txt con core_freq = 250.
 * 10. Se debe deshabilitar la consola en el archivo /boot/cmdline.txt quitando "console=serial0,115200".
 * 11.Se debe deshabilitar el servicio: serial-getty@ttyS0.service 
 * 12.Para los registros que no se encuentran mapeados se deben mapear con request_mem_region y despues
 * 	  ya se puede solicitar su direccion virtual con ioremap
 * 13.Los bits TX y RX en el registro IER estan intercambiados con respecto al manual de BROADCOM
 */

#include <linux/module.h>
#include <linux/fs.h>
#include <asm/uaccess.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/interrupt.h>

#include <linux/kernel.h>
#include <asm/io.h>		//Funciones ioremap
#include <linux/ioport.h>	//Funciones request_mem_region
#include <linux/gpio.h>

#define IRQ_UART		59
// Direcciones fisicas de registros del UART
#define AUX_ENB			0x3F215004
#define AUX_MU_IO		0x3F215040
#define AUX_MU_IER		0x3F215044
#define AUX_MU_IIR		0x3F215048
#define AUX_MU_LCR		0x3F21504C
#define AUX_MU_LSR		0x3F215054
#define AUX_MU_CNTL		0x3F215060
#define AUX_MU_BAUD		0x3F215068

#define DRIVER_LIC    "GPL"
#define DRIVER_AUTHOR "Victor Hugo Garcia Ortega"
#define DRIVER_DESC   "Manejo de UART a bajo nivel"

MODULE_LICENSE		(DRIVER_LIC);
MODULE_AUTHOR		(DRIVER_AUTHOR);	
MODULE_DESCRIPTION	(DRIVER_DESC);	

union direccion_virtual {
    void *map;
    volatile unsigned int *addr;
};
static union direccion_virtual aux_enb;
static union direccion_virtual aux_mu_io;
static union direccion_virtual aux_mu_baud;
static union direccion_virtual aux_mu_cntl;
static union direccion_virtual aux_mu_ier;
static union direccion_virtual aux_mu_iir;

static unsigned char muestra;

DEFINE_SPINLOCK(pulso_lock);

void tasklet_muestras( unsigned long muestra )
{
	printk(KERN_INFO "muestra: %d \n", (int)*((unsigned char *)muestra));

	return;
}

DECLARE_TASKLET( my_tasklet, tasklet_muestras, (unsigned long)&muestra );

/*
 * ISR del UART.
 */
static irqreturn_t irq_handler(int irq, void *dev_id, struct pt_regs *regs)
{
	static unsigned long flags;

	spin_lock_irqsave(&pulso_lock, flags);
	muestra = *(aux_mu_io.addr);
	spin_unlock_irqrestore(&pulso_lock, flags);

	tasklet_schedule( &my_tasklet );

	return IRQ_HANDLED;
}

/*
 * init_uart - Esta funcion inicializa el miniUART, esto se hace configurando directamente los registros 
 * 
 */
static int init_uart( void )
{
	static unsigned long flags;
	/* 
	 * Registro AUXENB. Bits  2    1    0
     * DesHabilitamos uart.	SPI2 SPI1 UART
	 */
//	*(aux_enb.addr) = 0x00000000;

	/* 																 |    	  |  	   |
	 * Registro GPFS1. Bits 31 30 29 28 27 26 25 24 23 22 21 20 19 18|17 16 15|14 13 12|11 10 9 8 7 6 5 4 3 2 1 0
     * GPIOS que controla:											 |GPIO 15 |GPIO 14 |
	 * Funcion alterna 5											 |0  1  0 |0  1  0 |	
	 * Auxiliary I/O												 | RXD1	  |	TXD1   |
	 */
//	GPFSEL1 = GPFSEL1 & 0xFFFC0FFF;
//	GPFSEL1 = GPFSEL1 | (1 << 16) | (1 << 13);
	/* 																 |    	  |  	   |
	 * Registro GPPUD. Bits 31 30 29 28 27 26 25 24 23 22 21 20 19 18|17 16 15|14 13 12|11 10 9 8 7 6 5 4 3 2 1 0
     * Se deshabilitan PULL-UP, PULL-DOWN
	 */	
//	GPPUD = 0x00000000;
//	delay(150);
//	GPPUDCLK0 = GPPUDCLK0 | (1 << 14) | (1 << 15) ;
//	delay(150);
//	GPPUDCLK0 = 0x00000000;

	spin_lock_irqsave(&pulso_lock, flags);

	/* 
	 * Registro AUXENB. Bits  2    1    0
     * Habilitamos uart.	SPI2 SPI1 UART
	 */
	*(aux_enb.addr) = 0x00000001;
	/* 
	 * Registro IIR. 			 Bits 7 6 5 4 3    2      1    0
     * Se limpian y habilitan FIFOS.  1 1 0 0 0	TXFIFO RXFIFO 
	 */
	*(aux_mu_iir.addr) = 0x000000C6;
	/* 
	 *BAUD_RATE 	= SYSTEM_CLOCK / 8*(AUX_MU_BAUD + 1)
     *AUX_MU_BAUD 	= (250 000 000 / 8*9600) - 1 = 3254.2 ~ 3254
	 */
	*(aux_mu_baud.addr) = 3254;
	/* 
	 * Registro CNTL				Bits:	1    0	
	 * Habilitamos el modulo de recepcion. TXEN RXEN 
	 */
	*(aux_mu_cntl.addr) = 0x01;
	/* 
	 * Establecemos el manejador de interrupcion
	 */
    if( request_irq(IRQ_UART, (irq_handler_t)irq_handler, IRQF_SHARED, "IRQ_del_UART_Raspberry", (void *)(irq_handler)) ) 
    {
		printk(KERN_ERR "No se puede registrar la IRQ 59 del UART\n");
        return -EIO;
    }
	/* 
	 * Registro IER.							Bits:	   1    0	
	 * Habilitamos interrupcion de recepcion en el UART. TXIE RXIE  
	 */
	*(aux_mu_ier.addr) = 0x05;

	//Habilitamos interrupcion 59
//	*(ier2.addr) = *(ier2.addr) | 0x08000000;

	spin_unlock_irqrestore(&pulso_lock, flags);
	return 0;
}

static int aux_mu_request_virtual( void )
{
	if (!request_mem_region(AUX_ENB, 4, "AUX_ENB")) 
	{
		printk(KERN_ALERT "Error de asignacion de memoria\n");
		return -EBUSY;
	}
    aux_enb.map = ioremap(AUX_ENB, 4);
	if (!aux_enb.map) 
	{
		printk(KERN_ALERT "Error de asignacion de AUXENB\n");
		release_mem_region(AUX_ENB, 4);
		return -EBUSY;
	}
    aux_mu_baud.map 	= ioremap(AUX_MU_BAUD, 4);
    if (!(aux_mu_baud.addr))
	{
		printk(KERN_ALERT "Error de asignacion de AUX_MU_BAUD\n");
		return -EBUSY;
	}
    aux_mu_ier.map 	= ioremap(AUX_MU_IER, 4);
    if (!(aux_mu_ier.addr))
	{
		printk(KERN_ALERT "Error de asignacion de AUX_MU_IER\n");
		return -EBUSY;
	}
    aux_mu_iir.map 	= ioremap(AUX_MU_IIR, 4);
    if (!(aux_mu_iir.addr))
	{
		printk(KERN_ALERT "Error de asignacion de AUX_MU_IIR\n");
		return -EBUSY;
	}
    aux_mu_cntl.map 	= ioremap(AUX_MU_CNTL, 4);
    if (!(aux_mu_cntl.addr))
	{
		printk(KERN_ALERT "Error de asignacion de AUX_MU_CNTL\n");
		return -EBUSY;
	}
    aux_mu_io.map 	= ioremap(AUX_MU_IO, 4);
    if (!(aux_mu_io.addr))
	{
		printk(KERN_ALERT "Error de asignacion de AUX_MU_IO\n");
		return -EBUSY;
	}
	return 0;
}

static int __init pulso_init(void)
{
	printk(KERN_INFO "Inicializando Modulo\n");

    aux_mu_request_virtual();
    init_uart();

    printk(KERN_INFO "AUX ENB:     	%x\n", (unsigned int)*(aux_enb.addr));
	printk(KERN_INFO "AUX_MU_BAUD: 	%x\n", (unsigned int)*(aux_mu_baud.addr));
	printk(KERN_INFO "AUX_MU_CNTL: 	%x\n", (unsigned int)*(aux_mu_cntl.addr));
	printk(KERN_INFO "AUX_MU_IER: 	%x\n", (unsigned int)*(aux_mu_ier.addr));
	printk(KERN_INFO "AUX_MU_IIR: 	%x\n", (unsigned int)*(aux_mu_iir.addr));

    printk(KERN_INFO "Dato leido: %d\n", (unsigned int)*(aux_mu_io.addr));

    return 0;
}

static void __exit pulso_exit(void)
{
	printk(KERN_INFO "liberando direcciones virtuales\n");

	iounmap(aux_enb.map		);
    iounmap(aux_mu_io.map	);
    iounmap(aux_mu_baud.map	);
    iounmap(aux_mu_cntl.map	);
	iounmap(aux_mu_ier.map	);
	iounmap(aux_mu_iir.map	);

	release_mem_region(AUX_ENB, 4);

    printk(KERN_INFO "liberando tasklet\n");
	tasklet_kill( &my_tasklet );
    printk(KERN_INFO "liberando IRQ\n");
	free_irq(IRQ_UART, (void *)(irq_handler));
}

module_init(pulso_init);
module_exit(pulso_exit);
