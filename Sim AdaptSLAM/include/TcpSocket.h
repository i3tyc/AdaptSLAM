
//
// Created by:
//      - Ali J. Ben Ali (https://github.com/benaliny)
//      - Yash Narendra Saraf (https://github.com/yash21saraf)
//
//      December 2019
//
#ifndef TCPSOCKET_H
#define TCPSOCKET_H

#include <iostream>
#include <sys/types.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <string.h>
#include <string>
#include <chrono>
#include <thread>
#include <mutex>
#include <future>
#include <vector>
#include <fcntl.h>  // For nonblocking sockets

class TcpSocket{
public:
    TcpSocket();
    TcpSocket(std::string ip_address, int port_number); // Constructor for server setup
    TcpSocket(std::string ip_address, int port_number, std::string other_ip_address, int other_port_number); // Constructor for client setup
    void setupSocket(); // Setup and Bind Socket
    int waitForConnection(); // Server waitForConnection method uses listen method to listen for connections
    int waitForConnection(int timeOut); // Overloaded waitForConnection method with timeout
    void disconnectConnection(); // Disconnects socket uses close to remove socket
    void reconnect(); // Reconnect calls disconnect and set's up the socket again
    int sendMessage(std::string& message); // Used to send the message
    std::string recieveMessage(); // Used to recieve the message
    int sendConnectionRequest(); // sendConnectionRequest is used to connect to the server
    int sendConnectionRequest(int timeOut); // sendConnectionRequest is overloaded with timeout
    ~TcpSocket(); // Destructor shuts down connection and unbinds ports
    void setAlive(bool alive); // Set termination flag
    bool checkAlive(); // Check termination flag
    void setConnTimeOut(bool timeOut); // Set connection timeout flag
    bool checkConnTimeOut(); // Check connection timeout flag

private:
    int mySocket, otherSocket; // Socket descriptors
    int socketHandle; // SocketHandle is used to manage send and recieve address for send and recieve
    sockaddr_in myAddress, otherAddress; // Socket address objects created using ip and port number
    const char * ip_address;  // Self IP address
    const char * other_ip_address; // Other end IP address
    int port_number, other_port_number; // Self and other port number
    bool isClient; // Used to mark the particular object as client or server based on constructor used
    bool isAlive ; // Used during termination. isAlive is set to false by main thread when termination is required
    std::mutex isAliveMutex; // Used to lock the isAlive flag
    bool connTimeOut ; // Set to true when the connection timeout has been reached
    std::mutex connTimeOutMutex; // Used to lock the connTimeOut flag

};

#endif //TCPSOCKET_H
