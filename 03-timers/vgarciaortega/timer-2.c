#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/timer.h>
#include <linux/sched.h>

#define DRIVER_LIC    "GPL"
#define DRIVER_AUTHOR "Victor Hugo Garcia Ortega"
#define DRIVER_DESC   "Ejemplo con time_before"

MODULE_LICENSE		(DRIVER_LIC);
MODULE_AUTHOR		(DRIVER_AUTHOR);	
MODULE_DESCRIPTION	(DRIVER_DESC);	

/*
 *	Esta función genera un retardo relativo de 5 segundos. Esta función es ejecutada
 *	cuando insertamos el módulo en el kernel mediante: sudo insmod timer-2.ko
*/

int kernel_init( void )
{
	unsigned long delay = jiffies + 5 * HZ;

	printk("En espera por 5 segundos con TIME_BEFORE\n");

	printk("Num de segundos antes del retardo: (%ld).\n", jiffies/HZ );

	// Se genera el retardo de 5 seg y se planifica
	while( time_before(jiffies, delay) )
		cond_resched();

	printk("Num de segundos después del retardo (%ld)\n", jiffies/HZ );

	return 0;
}
/*
 *	Esta función finaliza el timer. Esta función es ejecutada 
 *	cuando retiramos el módulo del kernel  mediante: sudo rmmod timer-2.ko.
*/
void kernel_exit( void )
{
	printk("Terminando el programa de bucle ocupado...\n");

	return;
}

module_init(kernel_init);
module_exit(kernel_exit);

