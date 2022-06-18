#include <stdio.h>
#include <stdlib.h>

#define DEVICE "/dev/buttons"

int main(int argc, char **argv)
{
	int i;
	char c;
	FILE  *f;
	
	if (argc>1) fork();
	
	printf("Abriendo fichero %s por pid %d\n", DEVICE, getpid());
	
	while((f=fopen(DEVICE, "r"))==NULL)
	{
		printf("ERROR en pid %d\n", getpid());
		perror("No pudo abrir el fichero, reintento en 5 segundos");
	    sleep(5);
	}
		
	for(i=0; i<5; i++)
	{
		fscanf(f,"%c",&c);
		printf("Leida pulsación #%d = '%c' por pid %d\n", i, c ,getpid());
	}
	printf("Cerrando fichero por pid %d\n", getpid());
	fclose(f);
	exit(0);
}

