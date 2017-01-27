#include<stdio.h>
#include<stdlib.h>
#include<errno.h>
#include<fcntl.h>
#include<string.h>
#include<unistd.h>

int main()
{
	int ret, fd;
	unsigned char muestra = 0;

   	printf("Comenzando el codigo de prueba...\n");
   	fd = open("/dev/rpiDDchar", O_RDWR);	// Abriendo el dispositivo para lectura y escritura
   	if( fd < 0 ) 
	{
    	perror("Falla al abrir el dispositivo...");
      	return errno;
   	}
   	printf("Dispositivo abierto con descriptor: %d \n", fd);
	sleep(1);
   	printf("Leyendo del dispositivo...\n");
   	ret = read(fd, &muestra, 1);        	// Leemos la muestra del LKM
   	if( ret < 0 )
	{
      perror("Falla al leer del dispositivo.");
      return errno;
   	}
   	printf("La muestra leida es: %d\n", (int)muestra);

   	printf("Termino el programa de prueba...\n");
   	
	close(fd);

   	return 0;
}
