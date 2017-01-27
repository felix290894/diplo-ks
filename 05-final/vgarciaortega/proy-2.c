/*
 * Este programa configura el UART en bajo nivel accediendo directamente a sus 
 * registros. Para realizar esto, de debe hacer una traduccion de las direcciones fisicas
 * de los perifericos en el procesador a las direcciones virtuales que maneja el kernel.
 * Se configura un device driver de caracter en /dev/RpiDDchar y se accede con un programa
 * en espacio de usuario.
 * Se usa el ejemplo de Derek Molloy http://www.derekmolloy.ie/.
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

#include <linux/init.h>         // Macros used to mark up functions e.g. __init __exit
#include <linux/module.h>       // Core header for loading LKMs into the kernel
#include <linux/device.h>       // Header to support the kernel Driver Model
#include <linux/kernel.h>       // Contains types, macros, functions for the kernel
#include <linux/fs.h>           // Header for the Linux file system support
#include <asm/uaccess.h>        // Required for the copy to user function
#include <linux/interrupt.h>	// Se requiere para el manejador de interrupcion	
#include <asm/io.h>				// Se requiere para funciones ioremap
#include <linux/sched.h>		// Se requiere para el tasklet

#define  DEVICE_NAME "rpiDDchar"    ///< The device will appear at /dev/ebbchar using this value
#define  CLASS_NAME  "rpi"        ///< The device class -- this is a character device driver

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

MODULE_LICENSE("GPL");            ///< The license type -- this affects available functionality
MODULE_AUTHOR("Victor Garcia");    ///< The author -- visible when you use modinfo
MODULE_DESCRIPTION("Un Device Driver de caracter para manejo de uart");  ///< The description -- see modinfo
MODULE_VERSION("0.1");            ///< A version number to inform users

static int    majorNumber;                  ///< Stores the device number -- determined automatically
static int    numberOpens = 0;              ///< Counts the number of times the device is opened
static struct class*  ebbcharClass  = NULL; ///< The device-driver class struct pointer
static struct device* ebbcharDevice = NULL; ///< The device-driver device struct pointer

union direccion_virtual 					//union que tiene la direccion virtual de los registros del uart
{
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

// Declaracion de las funciones para el driver de caracter
static int     dev_open		(struct inode *, struct file *);
static int     dev_release	(struct inode *, struct file *);
static ssize_t dev_read		(struct file  *, char *, size_t, loff_t *);
static ssize_t dev_write	(struct file  *, const char *, size_t, loff_t *);

// Declaracion de las funciones del driver propietario (nuestro!!! jejeje)
static int aux_mu_request_virtual( void );
static int init_uart( void );
void tasklet_muestras( unsigned long muestra );
static irqreturn_t irq_handler(int irq, void *dev_id, struct pt_regs *regs);
static void disable_uart( void );


DEFINE_SPINLOCK(pulso_lock);				//Spin lock para manejo de concurrencia en registros

/** @brief Devices are represented as file structure in the kernel. The file_operations structure from
 *  /linux/fs.h lists the callback functions that you wish to associated with your file operations
 *  using a C99 syntax structure. char devices usually implement open, read, write and release calls
 */
static struct file_operations fops =
{
   	.open 	= dev_open,		//Llamada cada vez que el dispositivo es abierto desde espacio de usuario
   	.read 	= dev_read,		//Llamada cuando los datos son enviados del dispositivo a espacio de usuario
   	.write 	= dev_write,	//Llamada cuando los datos son enviados del espacio de usuario al dispositivo
   	.release = dev_release,	//Llamada cuando el dispositivo es cerrado desde el espacio de usuario
};

/** @brief Funcion de inicializacion del LKM
 *  The static keyword restricts the visibility of the function to within this C file. The __init
 *  macro means that for a built-in driver (not a LKM) the function is only used at initialization
 *  time and that it can be discarded and its memory freed up after that point.
 *  @return returns 0 if successful
 */
static int __init ebbchar_init(void)
{
   	printk(KERN_INFO "RpiChar: Registrando el Device Driver de Caracter LKM\n");

   	// Cuando el numero mayor es cero, se asignara dinamicamente. El numero mayor es usado por el kernel 
   	// para identificar el device driver correcto cuando el dispositivo es accesado.
   	// El numero menor es dependiente del dispositivo y es manejado internamente por el driver, solo en 
   	// caso de que el driver maneje mas de un dispositivo.
   	// int register_chrdev(unsigned int major, const char *name, struct file_operations *fops);	
   	majorNumber = register_chrdev(0, DEVICE_NAME, &fops);
   	if (majorNumber<0)
	{
      	printk(KERN_ALERT "RpiDDChar fallo al registrar el numero mayor\n");
      	return majorNumber;
   	}
   	printk(KERN_INFO "RpiDDChar: Dispositivo registrado correctamente con numero mayor %d\n", majorNumber);

   	// Se Registra el "device class"
   	// class_create(owner, name)
   	ebbcharClass = class_create(THIS_MODULE, CLASS_NAME);
   	if (IS_ERR(ebbcharClass))	// Verifica si hay error y limpia el dispositivo de caracter si lo hay
	{                
      	unregister_chrdev(majorNumber, DEVICE_NAME);
      	printk(KERN_ALERT "Fallo al registrar el device class\n");
      	return PTR_ERR(ebbcharClass);          // Forma correcta de regresar un error en un apuntador
   	}
   	printk(KERN_INFO "RpiDDChar: device class registrado correctamente\n");

   	// Se registra el "device driver"
   	ebbcharDevice = device_create(ebbcharClass, NULL, MKDEV(majorNumber, 0), NULL, DEVICE_NAME);
   	if (IS_ERR(ebbcharDevice))	// Verifica si hay error y limpia el device class y el dispositivo de caracter
	{               
    	class_destroy(ebbcharClass);           
      	unregister_chrdev(majorNumber, DEVICE_NAME);
      	printk(KERN_ALERT "Falla al crear el dispositivo\n");
      	return PTR_ERR(ebbcharDevice);
   	}
   	printk(KERN_INFO "RpiDDChar: device class creado correctamente\n"); // Listo! el dispositivo fue inicializado

	aux_mu_request_virtual();

   	return 0;
}
/** @brief Tasklet ejecutada como "Deferrable function", en contexto de kernel
 *  Toma la muestra obtenida por la ISR y la imprime en syslog
 */
void tasklet_muestras( unsigned long muestra )
{
	printk(KERN_INFO "muestra: %d \n", (int)*((unsigned char *)muestra));

	return;
}
DECLARE_TASKLET( my_tasklet, tasklet_muestras, (unsigned long)&muestra ); //Declaracion de tasklet

/** @brief Funcion de limpieza del LKM 
 *  Similar to the initialization function, it is static. The __exit macro notifies that if this
 *  code is used for a built-in driver (not a LKM) that this function is not required.
 */
static void __exit ebbchar_exit(void)
{
	printk(KERN_INFO "Removiendo device driver\n");

   	device_destroy(ebbcharClass, MKDEV(majorNumber, 0));     // remove the device
   	class_unregister(ebbcharClass);                          // unregister the device class
   	class_destroy(ebbcharClass);                             // remove the device class
   	unregister_chrdev(majorNumber, DEVICE_NAME);             // unregister the major number

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
//	free_irq(IRQ_UART, (void *)(irq_handler));

   	printk(KERN_INFO "RpiDDChar: Saliendo del LKM!\n");
}

/** @brief The device open function that is called each time the device is opened
 *  This will only increment the numberOpens counter in this case.
 *  @param inodep A pointer to an inode object (defined in linux/fs.h)
 *  @param filep A pointer to a file object (defined in linux/fs.h)
 */
static int dev_open(struct inode *inodep, struct file *filep)
{
	init_uart();
    printk(KERN_INFO "Dato leido %d\n", (unsigned int)AUX_MU_IO);

	numberOpens++;
   	printk(KERN_INFO "RpiDDChar: El Dispositivo ha sido abierto %d veces\n", numberOpens);
   	return 0;
}

/** @brief This function is called whenever device is being read from user space i.e. data is
 *  being sent from the device to the user. In this case is uses the copy_to_user() function to
 *  send the buffer string to the user and captures any errors.
 *  @param filep A pointer to a file object (defined in linux/fs.h)
 *  @param buffer The pointer to the buffer to which this function writes the data
 *  @param len The length of the b
 *  @param offset The offset if required
 */
static ssize_t dev_read(struct file *filep, char *buffer, size_t len, loff_t *offset)
{
   	//put_user tiene la firma ( ); put_user ( var, * to );
   	put_user(muestra, buffer);
   	// copy_to_user has the format ( * to, *from, size) and returns 0 on success
   	//error_count = copy_to_user(buffer, message, size_of_message);
   	printk(KERN_INFO "RpiDDChar: Se mando la muestra %d \n", (int)muestra);

	return 0;
}

/** @brief This function is called whenever the device is being written to from user space i.e.
 *  data is sent to the device from the user. The data is copied to the message[] array in this
 *  LKM using the sprintf() function along with the length of the string.
 *  @param filep A pointer to a file object
 *  @param buffer The buffer to that contains the string to write to the device
 *  @param len The length of the array of data that is being passed in the const char buffer
 *  @param offset The offset if required
 */
static ssize_t dev_write(struct file *filep, const char *buffer, size_t len, loff_t *offset)
{
   	printk(KERN_INFO "RpiDDChar: Se recibieron %d caracteres desde el espacio de usuario\n", (int)len);
   	return len;
}

/** @brief The device release function that is called whenever the device is closed/released by
 *  the userspace program
 *  @param inodep A pointer to an inode object (defined in linux/fs.h)
 *  @param filep A pointer to a file object (defined in linux/fs.h)
 */
static int dev_release(struct inode *inodep, struct file *filep)
{
	disable_uart();
   	printk(KERN_INFO "RpiDDChar: Dispositivo cerrado correctamente...\n");
   	return 0;
}

/** @brief ISR del UART en la IRQ 59 Tasklet ejecutada como "Deferrable function", en contexto de kernel
 *  Toma la muestra obtenida por la ISR y la imprime en syslog
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


/** @brief Esta funcion mapea los registros al espacio de direcciones virtuales
 *  y obtiene dichas direcciones
 */
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
/** @brief Esta funcion deshabilita el miniUART, esto se hace configurando directamente los registros 
 * del controlador
 */
static void disable_uart( void )
{
	static unsigned long flags;
	spin_lock_irqsave(&pulso_lock, flags);

	/* 
	 * Registro IIR. 			 Bits 7 6 5 4 3    2      1    0
     * Se limpian y habilitan FIFOS.  1 1 0 0 0	TXFIFO RXFIFO 
	 */
	*(aux_mu_iir.addr) = 0x000000C6;
	/* 
	 * Registro CNTL				Bits:	1    0	
	 * Habilitamos el modulo de recepcion. TXEN RXEN 
	 */
	*(aux_mu_cntl.addr) = 0x00;
	/* 
	 * Establecemos el manejador de interrupcion
	 */
	free_irq(IRQ_UART, (void *)(irq_handler));
	/* 
	 * Registro IER.							Bits:	   1    0	
	 * Habilitamos interrupcion de recepcion en el UART. TXIE RXIE  
	 */
	*(aux_mu_ier.addr) = 0x00;

	/* 
	 * Registro AUXENB. Bits  2    1    0
     * Habilitamos uart.	SPI2 SPI1 UART
	 */
	*(aux_enb.addr) = 0x00000000;

	spin_unlock_irqrestore(&pulso_lock, flags);

}
/** @brief Esta funcion inicializa el miniUART, esto se hace configurando directamente los registros 
 * del controlador
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

/** @brief A module must use the module_init() module_exit() macros from linux/init.h, which
 *  identify the initialization function at insertion time and the cleanup function (as
 *  listed above)
 */
module_init(ebbchar_init);
module_exit(ebbchar_exit);
