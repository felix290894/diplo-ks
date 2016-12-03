#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/timer.h>
#include <linux/sched.h>

#define DRIVER_LIC    "GPL"
#define DRIVER_AUTHOR "Victor Hugo Garcia Ortega"
#define DRIVER_DESC   "Ejemplo con schedule_timeout"

MODULE_LICENSE		(DRIVER_LIC);
MODULE_AUTHOR		(DRIVER_AUTHOR);	
MODULE_DESCRIPTION	(DRIVER_DESC);	

/*
 *	Esta función genera un retardo relativo de 5 segundos. Esta función es ejecutada
 *	cuando insertamos el módulo en el kernel mediante: sudo insmod timer-3.ko
*/

int kernel_init( void )
{
	printk("En espera por 5 segundos con schedule_timeout\n");

	printk("Num de SEGUNDOS antes del retardo: (%ld).\n", jiffies/HZ );

	set_current_state(TASK_INTERRUPTIBLE);
	// signed long schedule_timeout(signed long timeout);
	schedule_timeout(5 * HZ);

	printk("Num de SEGUNDOS después del retardo (%ld)\n", jiffies/HZ );

	return 0;
}
/*
 *	Esta función finaliza el timer. Esta función es ejecutada 
 *	cuando retiramos el módulo del kernel  mediante: sudo rmmod timer-3.ko.
*/
void kernel_exit( void )
{
	printk("Terminando el programa de schedule_timeout...\n");

	return;
}

module_init(kernel_init);
module_exit(kernel_exit);

