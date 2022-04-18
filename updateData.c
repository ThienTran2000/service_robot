#include <mysql.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <wiringPi.h>
#include <wiringSerial.h>
#include <stdint.h>


int main()
{
	int fd;
	unsigned char receive_data;
	printf("Raspberry's receiving: \n");
	wiringPiSetup();
	fd = serialOpen ("/dev/ttyAMA0", 115200);
	
	MYSQL *conn;
	MYSQL_RES *res;
	MYSQL_ROW row;

	char *server = "localhost";
	char *user = "admin";
	char *password = "123456"; 
	char *database = "service_robot";
	char sql[200];
	
	conn = mysql_init(NULL);
	mysql_real_connect(conn,server,user,password,database,0,NULL,0); 
	
	while(1)
	{
		if(serialDataAvail(fd)>0)
		{
			receive_data = serialGetchar(fd);
			printf("data = %d\n", receive_data);
			fflush(stdout);
			sprintf(sql,"update update_data set room_%d='%d'", receive_data/10, receive_data%10);
			mysql_query(conn,sql);	
		}
	}
	
	mysql_close(conn);
	return 0;
}
