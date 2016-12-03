#include<linux/init.h>
#include<linux/module.h>
#include<linux/kernel.h>
#include<linux/kthread.h>
#include<linux/sched.h>
#include<linux/time.h>

#define DRIVER_LIC    "GPL"
#define DRIVER_AUTHOR "Victor Hugo Garcia Ortega"
#define DRIVER_DESC   "Un hilo de kernel simple con schedule_timeout"

MODULE_LICENSE		(DRIVER_LIC);
MODULE_AUTHOR		(DRIVER_AUTHOR);	
MODULE_DESCRIPTION	(DRIVER_DESC);	

struct task_struct *khilo;
/*
 * La funcion que es ejecutada por el hilo del kernel
 * Valor de retorno: 
 * 		Regresa id
 */

int hilo_kernel(void *data)
{
	int id = *(int *)data;

	printk(KERN_INFO "Hilo en ejecucion con id: %d...\n", id);
    while( !kthread_should_stop() )	//Termina el hilo cuando kthread_stop es llamada
	{
		set_current_state(TASK_INTERRUPTIBLE);
		schedule_timeout(5 * HZ);

		printk("Num de SEGUNDOS: %ld\n", jiffies/HZ );

    	//schedule();		//bloquea el hilo por un intervalo preestablecido de tiempo.
		//ssleep(2);	//bloquea el hilo por un numero de segundos = 2
    }
	return id;
}
/*
 * La funcion kthread_run crea y despierta a un hilo, lo pone en ejecucion.
 * Ejecuta una llamada a kthread_create() seguida de una llamada a wake_up_process()
 * Valor de retorno: 
 * 		Regresa el hilo creado ó
 * 		ERR_PTR(-ENOMEM ) 
 */

static int __init kernel_init(void)
{
	static int id_thread = 10;

	printk( KERN_INFO "\n--------------------------------------------\n" );
   	khilo = kthread_run( hilo_kernel, (void *)&id_thread, "kmi_hilo" );

	if( IS_ERR(khilo) ) 
	{
		printk("Error en la creación del hilo... \n");
		return PTR_ERR(khilo);
	}
   	printk( KERN_INFO "Hilo de Kernel creado: %s con PID %d \n", khilo->comm, khilo->pid );

   	return 0;
}
/*
 * La funcion kthread_stop realiza lo siguiente:
 * 1. Activa kthread->kthread_should_stop = true
 * 2. Despierta al hilo
 * 3. Espera a que el hilo termine
 * 4. Valor de retorno: 
 * 		Regresa el resultado de hilo_kernel ó
 * 		-EINTR si no se llama a la funcion wake_up_process()
 */
static void __exit kernel_exit(void)
{
	int ret_hilo_kernel = kthread_stop( khilo );

	if( ret_hilo_kernel == -EINTR )
		printk( KERN_INFO "Error en la terminación del hilo \n" );
	else
		printk( KERN_INFO "Hilo de Kernel con id: %d detenido\n", ret_hilo_kernel );		
}

module_init(kernel_init);
module_exit(kernel_exit);

