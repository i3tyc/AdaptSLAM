//
// Created by:
//      - Ali J. Ben Ali (https://github.com/benaliny)
//      - Yash Narendra Saraf (https://github.com/yash21saraf)
//
//      December 2019
//
#include "TcpSocket.h"

// Empty constructor
TcpSocket::TcpSocket(){}

/**
 * Constructor - For Server Initialization
 *
 * Initializes Object Variables and calls setupSocket to bind to the address.
 *
 * @param ip_address String IP address of server.
 * @param port_number Server Port Number
 */
TcpSocket::TcpSocket(std::string ip_address, int port_number)
{
    this->ip_address = ip_address.c_str();
    this->port_number = port_number;
    this->isClient = false;
    this->setupSocket();
    std::cout<<"TcpSocket::TcpSocket: Call method waitForConnection() method to open socket for listening to connections.\n";
    setAlive(true);
    setConnTimeOut(false);
}

/**
 * Constructor - For Client Initialization
 *
 * Initializes Object Variables and calls setupSocket to bind to the address.
 *
 * @param ip_address String IP address of the mobile device.
 * @param port_number Mobile device Port Number
 * @param other_ip_address String IP address of the Server.
 * @param other_port_number Server device Port Number
 */
TcpSocket::TcpSocket(std::string ip_address, int port_number, std::string other_ip_address, int other_port_number)
{
    this->ip_address = ip_address.c_str();
    this->port_number = port_number;
    this->other_port_number = other_port_number;
    this->other_ip_address = other_ip_address.c_str();
    this->isClient = true;

    bzero((char*)&this->otherAddress, sizeof(this->otherAddress));
    this->otherAddress.sin_family       = AF_INET;
    this->otherAddress.sin_port         = htons(this->other_port_number);
    this->otherAddress.sin_addr.s_addr  = inet_addr(this->other_ip_address);

    this->setupSocket();
    std::cout<<"TcpSocket::TcpSocket: Call method sendConnectionRequest() method to connect to the Server Socket.\n";
    setAlive(true);
    setConnTimeOut(false);
}

/**
 * Setup Socket - Initializes and binds the port
 *
 * Uses the class variables to setup connection
 */
void TcpSocket::setupSocket()
{
    std::cout<<"TcpSocket::setupSocket: Setting up socket.\n";

    this->mySocket = socket(AF_INET, SOCK_STREAM, 0);
    if (this->mySocket == -1)
    {
        std::cout << "TcpSocket::setupSocket: Unable to create Socket.\n";
    }

    // Bind the ip address and port to a socket

    this->myAddress.sin_family = AF_INET;
    this->myAddress.sin_port = htons(this->port_number);
    inet_pton(AF_INET, this->ip_address, &this->myAddress.sin_addr);

    if (bind(this->mySocket, (sockaddr*)&this->myAddress, sizeof(this->myAddress)) == -1)
    {
        std::cout << "TcpSocket::setupSocket: Unable to bind socket to the Address.\n";
    }

    std::cout<<"TcpSocket::setupSocket: Socket Initialized succesfully.\n";
}

/**
 * waitForConnection - Starts listening on the port.
 * It uses a nonblocking socket to connect, and then switches the socket back to
 * blocking.
 *
 * @return returns 1 when connected else returns -1
 */
int TcpSocket::waitForConnection()
{
    std::cout<<"TcpSocket::waitForConnection: Listening for connections using a nonblocking socket.\n";

    // Set the Socket to Non-blocking mode
    int flags = fcntl(this->mySocket, F_GETFL);
    fcntl(this->mySocket, F_SETFL, flags | O_NONBLOCK);

    if(listen(this->mySocket, 1)== -1)
    {
        return -1;
    }
    socklen_t clientSize = sizeof(this->otherSocket);
    int count = 0;
    for(;;)
    {
        otherSocket = accept(this->mySocket, (sockaddr*)&this->otherSocket, &clientSize);
        if (otherSocket == -1)
        {
            if (errno == EWOULDBLOCK)
            {
                if(count % 15 == 0)
                    std::cout<<"TcpSocket::waitForConnection: Still waiting for connection.\n";
                count++;
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            }
            else
            {
                std::cout<<"TcpSocket::waitForConnection: Error when accepting connection.\n";
                return -1;
            }
        }
        else
        {
            std::cout<<"TcpSocket::waitForConnection: Connected successfully.\n" ;
            this->socketHandle = this->otherSocket;

            // Set the Socket to Blocking mode
            std::cout<<"TcpSocket::waitForConnection: Switch socket to blocking.\n";
            flags = fcntl(this->mySocket, F_GETFL);
            fcntl(this->mySocket, F_SETFL, flags & (~O_NONBLOCK));
            return 1;
        }
    }
}

/**
 * waitForConnection with timeout
 *
 * Overloaded the waitForConnection function which allows for timeOut Break.
 * It uses a nonblocking socket to connect, and then switches the socket back to
 * blocking.
 *
 * @return 1 if connected else return -1.
 */
int TcpSocket::waitForConnection(int timeOut)
{
    std::cout<<"TcpSocket::waitForConnection(TimeOut): Listening for connections using a nonblocking socket.\n";

    // Set the Socket to Non-blocking mode
    int flags = fcntl(this->mySocket, F_GETFL);
    fcntl(this->mySocket, F_SETFL, flags | O_NONBLOCK);

    if(listen(this->mySocket, 1)== -1)
    {
        return -1;
    }
    socklen_t clientSize = sizeof(this->otherSocket);
    int count = 0;
    for(int i=0; i<timeOut; i++)
    {
        otherSocket = accept(this->mySocket, (sockaddr*)&this->otherSocket, &clientSize);
        if (otherSocket == -1)
        {
            if (errno == EWOULDBLOCK)
            {
                if(count % 15 ==0)
                    std::cout<<"TcpSocket::waitForConnection(TimeOut): Still waiting for connection.\n";
                count++;
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            }
            else
            {
                std::cout<<"TcpSocket::waitForConnection(TimeOut): Error when accepting connection.\n";
                return -1;
            }
        }
        else
        {
            std::cout<<"TcpSocket::waitForConnection(TimeOut): Connected successfully.\n" ;
            this->socketHandle = this->otherSocket;

            // Set the Socket to Blocking mode
            std::cout<<"TcpSocket::waitForConnection(TimeOut): Switch socket to blocking.\n";
            flags = fcntl(this->mySocket, F_GETFL);
            fcntl(this->mySocket, F_SETFL, flags & (~O_NONBLOCK));
            return 1;
        }
    }

    // Set the Socket to Blocking mode
    std::cout<<"TcpSocket::waitForConnection(TimeOut): No connection available. Switch socket to blocking and return.\n";
    flags = fcntl(this->mySocket, F_GETFL);
    fcntl(this->mySocket, F_SETFL, flags & (~O_NONBLOCK));
    return -1;
}

/**
 * Disconnect connection
 *
 * Uses close function to close both sockets
 *
 */
void TcpSocket::disconnectConnection()
{
    std::cout<<"TcpSocket::disconnectConnection.\n";

    close(this->mySocket);
    std::cout<<"TcpSocket::disconnectConnection: Stopped listening for Connections\n";
    close(this->otherSocket);
    std::cout<<"TcpSocket::disconnectConnection: Disconnected the connected Socket\n";
}

/**
 * Reconnect - Calls disconnect and setupSocket
 *
 * uses ifClient boolean to also either listen or send the request.
 *
 */
void TcpSocket::reconnect()
{
    std::cout<<"TcpSocket::reconnect: Reconnecting. Please Wait.\n";

    this->disconnectConnection();
    this->setupSocket();
    if(this->isClient)
        this->sendConnectionRequest();
    else
        this->waitForConnection();
}

/**
 * sendMessage - Finds the size of message and loops over
 * it and sends the complete message
 *
 * Initializes Object Variables and calls setupSocket to bind to the address.
 *
 * @param message String message which needs to be sent
 * @return returns 1 if sent successfullt, else return 0
 */
int TcpSocket::sendMessage(std::string& message)
{
    std::cout<<"TcpSocket::sendMessage.\n";

    // Send message size
    unsigned int messageLen = message.size();
    std::cout<<"TcpSocket::sendMessage: Message length: "<<messageLen<<"\n";
    if(!(send(this->socketHandle, &messageLen, sizeof(unsigned int), MSG_NOSIGNAL) == sizeof(unsigned int)))
    {
        std::cout<<"TcpSocket::sendMessage: Incorrect send size. Reattempting to connect.\n";
        this->reconnect();
        return 0;
    }

    // Receive ack
    {
        std::cout<<"TcpSocket::sendMessage: Waiting for first acknowledgement.\n";
        char buf[128];
        memset(buf,0,128);
        if(recv(this->socketHandle, buf, 128, 0) != 2)
        {
            std::cout<<"TcpSocket::sendMessage: Incorrect acknowledgement received. Reattempting to reconnect.\n";
            this->reconnect();
            return 0;
        }
    }

    // Send message
    int offset = 0;
    unsigned int payloadSize = 1024;
    while(messageLen > payloadSize)
    {
        if(!(send(this->socketHandle, (&message[0]) + offset, payloadSize, MSG_NOSIGNAL) == payloadSize))
        {
            std::cout<<"TcpSocket::sendMessage: Incorrect send size. Reattempting to connect.\n";
            this->reconnect();
            return 0;
        }
        offset += payloadSize;
        messageLen -= payloadSize;
    }

    if(messageLen > 0)
    {
        if(!(send(this->socketHandle, (&message[0]) + offset, messageLen, MSG_NOSIGNAL) == messageLen))
        {
            std::cout<<"TcpSocket::sendMessage: Incorrect send size. Reattempting to connect.\n";
            this->reconnect();
            return 0;
        }
    }

    // Receive ack
    {
        std::cout<<"TcpSocket::sendMessage: Waiting for second acknowledgement.\n";
        char buf[128];
        memset(buf,0,128);
        if(recv(this->socketHandle, buf, 128, 0) != 2)
        {
            std::cout<<"TcpSocket::sendMessage: Incorrect acknowledgement received. Reattempting to reconnect.\n";
            this->reconnect();
            return 0;
        }
    }

    std::cout<<"TcpSocket::sendMessage: Data sent succesfully.\n";
    return 1;
}

/**
 * recieveMessage - Recieve is a blocking function.
 *
 * First recieves the size of message and acknowledges
 * the size and loops over and recieves the complete message
 *
 * @return returns string recieved
 */
std::string TcpSocket::recieveMessage()
{
    std::cout<<"TcpSocket::receiveMessage.\n";

    // Receive message size
    unsigned int sizeOfMessage;
    recv(this->socketHandle, &sizeOfMessage, sizeof(unsigned int), 0);
    std::cout<<"TcpSocket::recieveMessage: Expecting message of size: "<<sizeOfMessage<<"\n";

    // Send ack
    if(!(send(this->socketHandle, "OK", 2, MSG_NOSIGNAL) == 2))
    {
        std::cout<<"TcpSocket::recieveMessage: Incorrect send size. Reattempting to connect.\n";
        this->reconnect();
        return "";
    }

    // Receive message
    const unsigned int MAX_BUF_LENGTH = 1024;
    std::vector<char> buffer(MAX_BUF_LENGTH);
    std::string rcv;
    int bytesReceived = 0;
    do {
        bytesReceived = recv(this->socketHandle, &buffer[0], MAX_BUF_LENGTH, 0);

        // append string from buffer.
        if(bytesReceived < 1)
        {
            std::cout<<"TcpSocket::recieveMessage: Error, Reconnecting again.\n";
            this->reconnect();
            return "";
        }
        else
        {
            rcv.append(&buffer[0], (&buffer[0])+bytesReceived);
        }
        sizeOfMessage -= bytesReceived;
    } while(sizeOfMessage > 0);

    std::cout<<"TcpSocket::recieveMessage: Remaining size is "<<sizeOfMessage<<", Size of recieved message is "<<rcv.size()<<"\n";

    // Send ack
    if(!(send(this->socketHandle, "OK", 2, MSG_NOSIGNAL) == 2))
    {
        std::cout<<"TcpSocket::recieveMessage: Incorrect send size. Reattempting to connect.\n";
        this->reconnect();
        return "";
    }

    return rcv;
}

/**
 * sendConnectionRequest - Overloaded with timeout
 *
 * Used to send connection request to the server.
 * The message has been sent in a busy waiting loop with sleeps.
 * The timeout is one way to make sure the loop gets terminated.
 *
 * Another implementation is the isAlive boolean and it's complimentary mutex
 * which has been used. Refer sendConnectionRequest()
 *
 * @return 1 if connects successfully and -1 if terminated using isAlive flag.
 *
 */
int TcpSocket::sendConnectionRequest(int timeOut)
{
    std::cout<<"TcpSocket::sendConnectionRequest(TimeOut).\n";

    std::mutex m;
    std::condition_variable cv;
    int retValue = -1;

    setConnTimeOut(false);
    std::thread t([&cv, &retValue, this]()
                  {
                      retValue = TcpSocket::sendConnectionRequest();
                      cv.notify_one();
                  });

    t.detach();

    {
        std::unique_lock<std::mutex> l(m);
        if(cv.wait_for(l, std::chrono::milliseconds(timeOut)) == std::cv_status::timeout)
        {
            setConnTimeOut(true);
            cv.wait(l);
            std::cout<<"TcpSocket::sendConnectionRequest(TimeOut): Unable to connect.\n";
        }
    }

    return retValue;
}

/**
 * sendConnectionRequest
 *
 * Used to send connection request to the server.
 * The message has been sent in a busy waiting loop with sleeps.
 * The timeout is one way to make sure the loop gets terminated.
 *
 * Another implementation is the isAlive boolean and it's complimentary mutex
 * which has been used.
 *
 * @return 1 if connects successfully and -1 if terminated using isAlive flag.
 *
 */
int TcpSocket::sendConnectionRequest()
{
    std::cout<<"TcpSocket::sendConnectionRequest.\n";

    int connected = -1;
    int count = 0;
    while(connected != 0 )
    {
        if((!checkAlive()) || (checkConnTimeOut()))
            return -1;

        connected = connect(this->mySocket, (struct sockaddr*)&otherAddress, sizeof(this->otherAddress));
        if((count % 15 == 0) && (connected != 0))
            std::cout<<"TcpSocket::sendConnectionRequest: Still trying to connect.\n";
        count++;
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
    this->socketHandle = this->mySocket;
    std::cout<<"TcpSocket::sendConnectionRequest: Socket connected.\n";
    return 1;
}

/**
 * Set termination flag
 *
 * Uses isAliveMutex
 *
 */
void TcpSocket::setAlive(bool alive)
{
    std::cout<<"TcpSocket::setAlive: Set termination flag to " << alive << "\n";

    std::unique_lock<std::mutex> lock(isAliveMutex);
    isAlive = alive;
}

/**
 * Check termination flag
 *
 * Uses isAliveMutex
 *
 */
bool TcpSocket::checkAlive()
{
    //std::cout<<"TcpSocket::checkAlive: Check termination flag.\n";

    std::unique_lock<std::mutex> lock(isAliveMutex);
    return isAlive;
}

/**
 * Set connection timeout flag
 *
 * Uses connTimeOutMutex
 *
 */
void TcpSocket::setConnTimeOut(bool timeOut)
{
    std::cout<<"TcpSocket::setConnTimeOut: Set connection timeout flag to " << timeOut << "\n";

    std::unique_lock<std::mutex> lock(connTimeOutMutex);
    connTimeOut = timeOut;
}

/**
 * Check connection timeout flag
 *
 * Uses connTimeOutMutex
 *
 */
bool TcpSocket::checkConnTimeOut()
{
    //std::cout<<"TcpSocket::checkConnTimeOut: Check connection timeout flag.\n";

    std::unique_lock<std::mutex> lock(connTimeOutMutex);
    return connTimeOut;
}

/**
 * Destructor - shutDown sockets so that the busy waiting function gets \
 * terminated and we can then terminate the thread.
 *
 * Also sets the isAlive as false which helps terminate the sendConnectionRequest
 * busy waiting loop.
 *
 */
TcpSocket::~TcpSocket()
{
    std::cout<<"TcpSocket::~TcpSocket: Destructor.\n";

    shutdown(this->mySocket, 2);

    setAlive(false);
    setConnTimeOut(true);

    close(this->mySocket);
    std::cout<<"TcpSocket::~TcpSocket: Stopped listening for Connections\n";
    close(this->otherSocket);
    std::cout<<"TcpSocket::~TcpSocket: Disconnected the connected Socket\n";
}
