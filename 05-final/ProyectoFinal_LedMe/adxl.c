#include <linux/kernel.h>         // Contiene kernel types, macros, funciones
#include <linux/module.h>         // Core header para cargar LKMs 
#include <linux/fs.h>
#include <asm/uaccess.h>       
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/types.h>
#include <linux/kdev_t.h>
#include <linux/gpio.h>         // Requerido para las funciones de GPIO
#include <linux/interrupt.h>         // Requerido para habilitar IRQ’s
#include <linux/workqueue.h>

#define DEVICE_NAME "ADXL345_PATHOS"
#define MAJOR_NO 115

//ADXL345.- Definición de los registros:

#define REG_DEVID 0x00
#define REG_DATA_FORMAT 0x31
#define REG_INT_ENABLE 0x2E
#define REG_POWER_CTL 0x2D
#define REG_INT_MAP 0x2F
#define REG_INT_SOURCE 0x30
#define REG_DATAX0 0x32
#define REG_DATAX1 0x33
#define REG_DATAY0 0x34
#define REG_DATAY1 0x35
#define REG_DATAZ0 0x36
#define REG_DATAZ1 0x37

#define INT_GPIO 17         // pin 11 (GPIO17)

/*Esta estructura de datos, denominada adxl_data, es necesaria
* para poder almacenar en variables de tipo unsigned de 8 bits l
* as lecturas de cada eje. Además, el dato para el número corre
* spondiente de interrupción.  */

static struct adxl_data_struct {
    u8 xdata;
    u8 ydata;
    u8 zdata;
    unsigned int irq;
} adxl_data;

/*Declaración de variables globales  */

static struct class *adxl_class;    
static dev_t dev_number;
struct class *cl;
static struct adxl_data data;
static unsigned int irqNumber;    //Variable para almacenar el número de interrupción

/*Apuntador al cliente de i2c, necesario para registrar mi
* driver en el i2c core. Su estructura contiene distintos pa
* rámetros para poder registrar el dispositivo  */

static struct i2c_client * my_client;    

/*Apuntador al adaptador de i2c, necesario para adquirir 
* el bus de i2c. En el caso de la raspberry pi 3, nos perm
* ite elegir entre los dos bus de i2c con los que cuenta la
* tarjeta  */

struct i2c_adapter * my_adap;    
static dev_t first;
static struct cdev c_dev;

/*Buffer para almacenar los datos de lectura*/
static u8 axis_data[3];

/*Handler de interrupción por GPIO*/
static void mykmod_work_handler(struct work_struct *w);

/*Se declara la cola de trabajo para leer los datos del acel
* erómetro. Incluye la primera mitad de operación de la int
* errupción.  */

static struct workqueue_struct *wq = 0;
static DECLARE_WORK(mykmod_work, mykmod_work_handler);

/*Función para leer los registros del ADXL345*/
static u8 adxl_read(struct i2c_client *client, u8 reg)
{
    int ret;

    ret = i2c_smbus_read_byte_data(client, reg);
    if (ret < 0)
        dev_err(&client->dev,
            "can not read register, returned %d\n", ret);

    return (u8)ret;
}

/*Función para escribir a los registros del ADXL345*/
static int adxl_write(struct i2c_client *client, u8 reg, u8 data)
{
    int ret;

    ret = i2c_smbus_write_byte_data(client, reg, data);
    if (ret < 0)
        dev_err(&client->dev,"can not write register, returned %d\n", ret);

    return ret;
}

/*La siguiente función se manda a llamar cuando una aplicación
* en el espacio del usuario quiere leer información. La informaci
* ón del arreglo axis_data del kernel al usuario con copy_to_user() 
* y regresa el número de bytes copiados  */

static ssize_t my_read(struct file *f, char __user *buf, size_t len, loff_t *off) {    

    printk(KERN_INFO "Driver read()\n");

    if (*off == 0)
    {
        if (copy_to_user(buf, &axis_data,3) != 0)
        {
            printk(KERN_INFO "Driver read: Inside if\n");
            return -EFAULT;
        }
        else
        {
            return 3;
        }
    }
    else
        return 0;
}


static int my_open(struct inode *i, struct file *f)
{
      printk(KERN_INFO "Driver: open()\n");
        return 0;
}
static int my_close(struct inode *i, struct file *f)
{
      printk(KERN_INFO "Driver: close()\n");
        return 0;
}

static ssize_t my_write(struct file *f, const char __user *buf,
           size_t len, loff_t *off)
{
      printk(KERN_INFO "Driver: write()\n");
        return len;
}

static struct file_operations my_dev_ops = {.read = my_read,
                        .owner = THIS_MODULE,
                        .open = my_open,
                        .release = my_close,
                        .write = my_write};

/*Subrutina de interrupción que es ejecutada en cuanto se detecta
* la disponibilidad de un nuevo set de datos. Implementa una cola 
* de trabajo para leer los datos del acelerómetro, realizando la lect
* ura directamente de smbus_read(). En consecuencia congela el 
* procesador permitiendo una rutina de interrupción, ISR, atómica. 
* NOTA: Se debe evitar colocar código que duerma la ejecución del
* programa dentro de la ISR.  */

static irq_handler_t adxl_irq_handler(unsigned int irq, void *dev_id, struct pt_regs *regs)
{
    static unsigned int intCount = 0;    //Este contador nos permite determinar el número de veces que se accede a la interrupción
    intCount++;
    printk(KERN_INFO "ADXL: interrupt_handler(),Count=%d\n",intCount);

    if (wq)
        queue_work(wq, &mykmod_work);

    return (irq_handler_t) IRQ_HANDLED;    //Se notifica que la interrupción ha concluido
}

/*
Subrutina de interrupción. Es ejecutada por el handler de interrupción y es la que realiza el trabajo fuerte de lectura de los valores del sensor. 
*/
static void mykmod_work_handler(struct work_struct *w)
{
    printk(KERN_INFO "ADXL: inside mykmod_work_handler\n");

    axis_data[0] = adxl_read(my_client, REG_DATAX1);
    axis_data[1] = adxl_read(my_client, REG_DATAY1);
    axis_data[2] = adxl_read(my_client, REG_DATAZ1);
}


/*The LKM initialization function
 * The static keyword restricts the visibility of the function to within
 * this C file. The __init macro means that for a built-in driver (not
 * a LKM) the function is only used at initialization time and that it
 * can be discarded and its memory freed up after that point.
 * @return returns 0 if successful  */

static int __init adxl_init(void)
{
    int ret_val,errno;
    int result = 0;
    u8 readvalue,temp;
        printk(KERN_INFO "Welcome to ADXL345 Accelerometer!\n");
       
        if (alloc_chrdev_region(&first, 0, 1, DEVICE_NAME) < 0) {
        printk(KERN_DEBUG "Can't register device\n");
        return -1;
        }
    if (IS_ERR(cl = class_create(THIS_MODULE, "chardrv"))){
            unregister_chrdev_region(first, 1);
            }

    if (IS_ERR(device_create(cl, NULL, first, NULL, "adxl"))) {
        class_destroy(cl);
        unregister_chrdev_region(first,1);
    }

    cdev_init(&c_dev,&my_dev_ops);
    if (cdev_add(&c_dev, first,1) < 0)
    {
    device_destroy(cl, first);
    class_destroy(cl);
    unregister_chrdev_region(first,1);
    return -1;
    }

    my_adap = i2c_get_adapter(1); // 1 means i2c-1 bus
    if (!(my_client = i2c_new_dummy(my_adap, 0x53))){
        printk(KERN_INFO "Couldn't acquire i2c slave");
        unregister_chrdev(MAJOR_NO, DEVICE_NAME);
        device_destroy(cl, first);
        class_destroy(cl);
        return -1;
    }
        
    readvalue = adxl_read(my_client,0x00); //Lee el registro 0x00 del dispositivo para poder obtener su ID y lo compara con el valor 0xE5 que viene especificado en la datasheet del sensor ADXL345
    if (readvalue == (0b11100101)) {
        printk("Accelerometer detected, value = %d\r\n",readvalue);
    }

    //Setting Data format to be left justified
    temp = adxl_read(my_client,REG_DATA_FORMAT);
    temp = temp | (1<<2);
    adxl_write(my_client, REG_DATA_FORMAT, temp);
    
    //Habilitar interrupción
    temp = adxl_read(my_client,REG_INT_ENABLE);
    temp = temp | (1<<7);
    adxl_write(my_client, REG_INT_ENABLE, temp);

    
    
    //Comienza la medición
    temp = adxl_read(my_client,REG_POWER_CTL);
    temp = temp | (1<<3);
    adxl_write(my_client, REG_POWER_CTL, temp);

//Si no existe un workqueue, crearlo.
    if (!wq)
        wq = create_singlethread_workqueue("mykmod");


    irqNumber = gpio_to_irq(INT_GPIO);  //Obtiene el valor de IRQ correspondiente al GPIO en donde se encuentra conectado el pin de interrupción del sensor

/*Configuración de los parámetros de la interrupción: flanco de subida, valor de IRQ. */
    result = request_irq(irqNumber,             // interrupt number requested
     (irq_handler_t) adxl_irq_handler,        // handler function
            IRQF_TRIGGER_RISING,        // on rising edge 
            "ADXL_INT",                // used in /proc/interrupts
            NULL);                // *dev_id for shared interrupt lines
    printk(KERN_INFO "ADXL: The interrupt request result is: %d\n", result);

    axis_data[0] = adxl_read(my_client, REG_DATAX1);
    axis_data[1] = adxl_read(my_client, REG_DATAY1);
    axis_data[2] = adxl_read(my_client, REG_DATAZ1);

    return 0;
}
 
/* The LKM cleanup function
 * Similar to the initialization function, it is static. The __exit
 * macro notifies that if this code is used for a built-in driver (not
 *  a LKM) that this function is not required.    */

//Fin: Devolver y eliminar todos los recursos ocupados 
static void __exit adxl_exit(void)
{                                
printk(KERN_INFO "ADXL: Removing module");
     
        free_irq(irqNumber,NULL);
        destroy_workqueue(wq);
        i2c_unregister_device(my_client);
    cdev_del(&c_dev);
device_destroy(cl, first);
        class_destroy(cl);
        unregister_chrdev_region(first, 1);
}


module_init(adxl_init);
module_exit(adxl_exit);

MODULE_DESCRIPTION("ADXL345 Driver");
MODULE_AUTHOR("Pathos");
MODULE_LICENSE("GPL");

