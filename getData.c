#include <mysql.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <wiringPi.h>
#include <wiringSerial.h>
#include <stdint.h>

int currenttime()
{
	int result = 0;
	time_t t = time(NULL);
	struct tm tm = *localtime(&t);
	//printf("now: %d-%02d-%02d %02d:%02d:%02d\n", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
	result = tm.tm_hour*3600 + tm.tm_min*60 + tm.tm_sec;
	return result;
}

int getTime(char *s)
{
	int hour;
	int min;
	int sec;
	int result;
	hour = (s[11] - 48)*10 + (s[12] - 48);
	min = (s[14] - 48)*10 + (s[15] - 48);
	sec = (s[17] - 48)*10 + (s[18] - 48);
	result = hour*3600 + min*60 + sec;
	return result;
}

int main()
{
	MYSQL *conn;
	MYSQL_RES *res;
    MYSQL_ROW row;
	
	int fd;
	fd = serialOpen ("/dev/ttyAMA0", 115200);
	
	char *server = "localhost";
    char *user = "admin";
    char *password = "123456"; 
    char *database = "service_robot";
	
	char sql[200];
	
	int check_time;
	unsigned char *room_data[5];
	
	conn = mysql_init(NULL);
	mysql_real_connect(conn,server,user,password,database,0,NULL,0); 
	
	printf("Running ....\n");
	while(1)
	{
		sprintf(sql, "select * from set_data");
		mysql_query(conn,sql);
		res = mysql_store_result(conn); 	
		row = mysql_fetch_row(res);
		
		check_time = currenttime() - getTime(row[0]);
		room_data[0] = row[1];
		room_data[1] = row[2];
		room_data[2] = row[3];
		room_data[3] = row[4];
		room_data[4] = row[5];
		
		if(check_time > 0 && check_time < 10)
		{
			printf("Room 1 = %c\n", *room_data[0]);
			printf("Room 2 = %c\n", *room_data[1]);
			printf("Room 3 = %c\n", *room_data[2]);
			printf("Room 4 = %c\n", *room_data[3]);
			printf("Room 5 = %c\n", *room_data[4]);
			
			printf("Sending....\n");
			
			for(int i = 0; i < 5; i++)
			{
				serialPuts(fd, room_data[i]);
				serialFlush(fd);
			}
		}
		delay(9000);
	}
	serialClose(fd);
	mysql_close(conn);
}
