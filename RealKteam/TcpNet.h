/*
 * TcpNet.h
 *
 *  Created on: Nov 25, 2013
 *      Author: bluebird
 */

#ifndef TCPNET_H_
#define TCPNET_H_

#endif /* TCPNET_H_ */


void Printer1();
void ThisComputerInf(short int Port);
int SocketMaker(short int Port);
int WaitForClient();
void CloseConnection();
int MassageSender(char *Message,int client_fd);
int MassageReceiver(char *Message,int client_fd, int length);
