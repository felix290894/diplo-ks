#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/timer.h>

#define DRIVER_LIC    "GPL"
#define DRIVER_AUTHOR "Victor Hugo Garcia Ortega"
#define DRIVER_DESC   "Ejemplo de timer"

MODULE_LICENSE		(DRIVER_LIC);
MODULE_AUTHOR		(DRIVER_AUTHOR);	
MODULE_DESCRIPTION	(DRIVER_DESC);	

static struct timer_list my_timer;

void my_timer_callback( unsigned long data )
{
	printk("fUNCION TEMPORIZADA CON SETUP_TIMER Y MOD_TIMER, EJECUTADA, JIFFIES: (%ld).\n", jiffies );
}

/*
 *	Esta función inicializa el timer y lo programa. Esta función es ejecutada
 *	cuando insertamos el módulo en el kernel mediante: sudo insmod timer-1.ko
*/
int kernel_init( void )
{
	int ret;

	printk("Inicializando el timer\n");
	// my_timer.function, my_timer.data

	// setup_timer(timer, fn, data) 
	setup_timer( &my_timer, my_timer_callback, 0 );
	printk("Programando el timer para comenzar en 1 seg, jiffies: (%ld)\n", jiffies );

	// int mod_timer(struct timer_list *timer, unsigned long expires);
	// ret = mod_timer( &my_timer, jiffies + msecs_to_jiffies(1000) );
	ret = mod_timer( &my_timer, jiffies + HZ );	

	if( ret ) 
		printk("Error al programar el timer\n");

	return 0;
}
/*
 *	Esta función finaliza el timer. Esta función es ejecutada 
 *	cuando retiramos el módulo del kernel  mediante: sudo rmmod timer-1.ko.
*/
void kernel_exit( void )
{
	int ret;

	// int del_timer_sync(struct timer_list *timer);
	ret = del_timer_sync( &my_timer );
	if (ret) 
		printk("El timer esta en uso...\n");

	printk("Timer desinstalado...\n");
}

module_init(kernel_init);
module_exit(kernel_exit);

