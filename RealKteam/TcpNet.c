/*
 * TcpNet.c
 *
 *  Created on: Nov 25, 2013
 *      Author: bluebird
 */
#include <stdio.h> //基本初始化文件
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/wait.h>
#include <arpa/inet.h>
#include <unistd.h>

#define MaxConnect 10

int Sockdf; //套接字描述符,用于监听
int client_fd;//套接字描述符,用于传输

//本机地址信息
struct sockaddr_in ThisComputer;
struct sockaddr_in OutSideComputer;

void ThisComputerInf(short int Port)
{
	ThisComputer.sin_family = AF_INET;
	ThisComputer.sin_port = htons(Port);
	ThisComputer.sin_addr.s_addr = INADDR_ANY;
	bzero(&(ThisComputer.sin_zero),8);
}

int SocketMaker(short int Port)
{
	if((Sockdf = socket(AF_INET, SOCK_STREAM, 0)) == -1)
	{
		printf("套接字创建没成功!\r\n");
		return -1;
	}
	else
	{
		//录入本机信息
		ThisComputerInf(Port);
		//捆绑
		if(bind(Sockdf,(struct sockaddr *)&ThisComputer, sizeof(struct sockaddr)) == -1)
		{
			printf("捆绑套接字没成功!\r\n");
			return -1;
		}
		else
		{
			//开始启动监听
			if(listen(Sockdf,MaxConnect) == -1)
			{
				printf("启动监听失败!\r\n");
				return -1;
			}
			else
			{
				return 1;
			}
		}
	}

}

int WaitForClient()
{
	//服务器处理主函数
	unsigned int sin_size = sizeof(struct sockaddr_in);
	//while(1)
	//{
		if((client_fd = accept(Sockdf, (struct sockaddr *)&OutSideComputer, &sin_size)) == -1)
		{
			printf("还在等。。。\r\n");
			//continue;
		}
		printf("收到来自 %s 的连结\r\n",inet_ntoa(OutSideComputer.sin_addr));
		return client_fd;
		//break;
	//}

}

void CloseConnection()
{
	close(client_fd);
	close(Sockdf);
}

void Printer1()
{
	printf("Net,Head File Test!\r\n");

}

int MassageSender(char *Message,int client_fd)
{
	int MessageLength = strlen(Message);
	int SendReturn;
	SendReturn = send(client_fd,Message,MessageLength,0);
	return SendReturn;
}


int MassageReceiver(char *Message,int client_fd, int length)
{
	int RecvReturn;
	RecvReturn = recv(client_fd, Message,length,0);
	return RecvReturn;
}



