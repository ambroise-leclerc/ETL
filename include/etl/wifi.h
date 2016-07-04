#pragma once
#include <memory>

extern "C"
{
#include "mem.h"
#include "ets_sys.h"
#include "osapi.h"
#include "user_interface.h"
#include <espmissingincludes.h>
#include <ip_addr.h>
#include <espconn.h>
#include <espmissingincludes.h>
#include "os_type.h"
	void* __dso_handle;
  
}

#include <etl/architecture/uart_esp.h>



using uartWifi =   etl::Uart0<>;
void writeUartWifi(sint8 s)
{
 uartWifi::write((char)(s+48));
}

void writeUartWifi(const char * string)
{
    auto i = 0;
    while (string[i] != '\0')
    {
        uartWifi::write(string[i]);
        i++;
    }    
    uartWifi::write('\r');
    uartWifi::write('\n');
}

extern "C" void abort() {
    while (true)
        ; // enter an infinite loop and get reset by the WDT
}



namespace std
{
    void __throw_bad_function_call()
    {
        abort();
    }

    void __throw_length_error(char const*)
    {
        abort();
    }

    void __throw_bad_alloc()
    {
        abort();
    }

    void __throw_logic_error(const char* str)
    {
        abort();
    }

    void __throw_out_of_range(const char* str)
    {
        abort();
    }
}

void * operator new(size_t size) {
    writeUartWifi("inside new ope");
    os_delay_us(100);
    auto rsize = os_malloc(size);
    writeUartWifi("end new ope");
    return rsize;
}

void * operator new[](size_t size) {
    return os_malloc(size);
}

void operator delete(void * ptr, size_t) {
    os_free(ptr);
}

void operator delete[](void * ptr, size_t) {
    os_free(ptr);
} 

void operator delete(void * ptr) {
    os_free(ptr);
}

void operator delete[](void * ptr) {
    os_free(ptr);
} 

extern "C" void __cxa_pure_virtual(void) __attribute__((__noreturn__));
extern "C" void __cxa_deleted_virtual(void) __attribute__((__noreturn__));


void __cxa_pure_virtual(void) {
    abort();
}

void __cxa_deleted_virtual(void) {
    abort();
} 

void  dnsResolved(const char *name, ip_addr_t *ipaddr, void *arg);

class Client
{
public:
      
    Client(const char * hostP)
        : host(hostP) {}
    ~Client() {}
  
    void post(const char * nameP)
    {
        currentyType = POST;
        name = nameP;
        espconn_gethostbyname(&connection, host, &ip, dnsResolved);
    }
  
    enum REQUEST_TYPE
    {
        POST, 
        GET
    };
    
    const char * host;
    const char * name;
   
    REQUEST_TYPE currentyType;
    
    const char * path = "/update";
    char data[256];
    char buffer[2048];

    espconn connection;
    ip_addr_t  ip;
    esp_tcp  tcp;
};

Client* clientTab[10];

struct ClientManager
{
    auto static createClient(const char * hostP)
    {
        writeUartWifi("createClient by host");
        writeUartWifi(hostP);
        auto clientPtr = new Client(hostP);
        auto oldPtr = clientTab[clientIndex] ;
        if(oldPtr){
            delete(oldPtr);
        }
        clientTab[clientIndex] = clientPtr;
        if (clientIndex == 9)
        {
            clientIndex = 0;
        }
        else
        {
            clientIndex++;
        }
        return clientPtr;
    }
    
    
    auto static getClient(const char * hostP)
    {
        writeUartWifi("getClient by host");
        writeUartWifi(hostP);
        for (int i = 0; i < clientIndex; i++)
        {
            auto clientPtr = clientTab[i];
            if (strcmp(clientPtr->host, hostP) == 0 )
            {
                return clientPtr;
            }
            
        }
        return createClient(hostP);
    }
    
    
    static Client* getClient(espconn * connection)
    {
        writeUartWifi("getClient by ptr");
       
        for (int i = 0; i < clientIndex; i++)
        {
            auto clientPtr = clientTab[i];
            if (connection == &(clientPtr->connection))
            {
                writeUartWifi("find");
                return clientPtr;
            }
            
        }
        writeUartWifi("nullptr");
        return nullptr ;
    }
    
    static int clientIndex;
    
};


int ClientManager::clientIndex = 0;

    

void  connected(void *arg)
{
    writeUartWifi("tcp_connected");
    auto clientPtr = ClientManager::getClient((espconn *)arg);
    auto currentyType = clientPtr->currentyType;
    switch (currentyType)
    {
    case Client::POST :
    {
        const char * prenom = clientPtr->name;
        auto number = 10;
        os_sprintf(clientPtr->data, "api_key=1LBURH919EGVNR97&field1=%s&field2=%d", prenom, number);
        os_sprintf(clientPtr->buffer,
            "POST %s HTTP/1.1 \r\nHost: %s \r\nCache-Control: no-cache \r\nContent-Type: application/x-www-form-urlencoded \r\nContent-Length: %d \r\n\r\n%s",
            clientPtr->path,
            clientPtr->host,
            os_strlen(clientPtr->data),
            clientPtr->data);
    
        writeUartWifi("Sending: ");
        writeUartWifi(clientPtr->buffer);
        auto value = espconn_sent(&(clientPtr->connection), (uint8_t *)(clientPtr->buffer), os_strlen(clientPtr->buffer));
        writeUartWifi("value:");
        writeUartWifi(value);
        writeUartWifi("END Sending!!!");
    }
        break;
     case Client::GET :
           break;
    }
     
}

void  response(void *arg, char *pdata, unsigned short len)
{
    writeUartWifi("response:");
    writeUartWifi(pdata);
}

void  sentcb(void *arg)
{
    writeUartWifi("sentcb:");
}

void finishWrite(void *arg)
{
     writeUartWifi("finishWrite:");
}

void  disconnected(void *arg)
{
    writeUartWifi("tcp_disconnected");
}

void  dnsResolved(const char *name, ip_addr_t *ipaddr, void *arg)
{
    writeUartWifi("dns_done");
        
    if (ipaddr == NULL) 
    {
        writeUartWifi("DNS lookup failed\n");
        wifi_station_disconnect();
    }
    else  if (arg == NULL) 
    {
         writeUartWifi("arg null");
    }
    else
    {
        writeUartWifi("Connecting dnsResolved...");
        writeUartWifi(name);
        auto clientPtr = ClientManager::getClient(name);
        writeUartWifi("ClientPtr retrieved");
        writeUartWifi("ESPCONN_TCP");
        auto connection = &(clientPtr->connection);
        connection->type = ESPCONN_TCP;
 writeUartWifi("ESPCONN_NONE");
        connection->state = ESPCONN_NONE;
 writeUartWifi("&(clientPtr->tcp)");
        connection->proto.tcp = &(clientPtr->tcp);
writeUartWifi("espconn_port");
        connection->proto.tcp->local_port = espconn_port();
writeUartWifi("remote_port");
        connection->proto.tcp->remote_port = 80;
writeUartWifi("os_memcpy");
        os_memcpy(connection->proto.tcp->remote_ip, &ipaddr->addr, 4);
writeUartWifi("espconn_regist_connectcb");
        espconn_regist_connectcb(connection, connected);
writeUartWifi("espconn_regist_disconcb");
        espconn_regist_disconcb(connection, disconnected);
writeUartWifi("espconn_regist_recvcb");
        espconn_regist_recvcb(connection, response);
        espconn_regist_sentcb(connection, sentcb);
        espconn_regist_write_finish(connection,finishWrite);
writeUartWifi("espconn_connect");
        espconn_connect(connection);
     writeUartWifi("end dns_done");
    }
}

class Wifi
{
public:
    static void connect(const char * ssid, const char * password)
    {
        static struct station_config config;
        config.bssid_set = 0;
        os_memcpy(&config.ssid, ssid, 32);
        os_memcpy(&config.password, password, 64);
        wifi_station_set_config(&config);
    }
    
    static void connect(const char * ssid, const char * password, wifi_event_handler_cb_t handler)
    {
       for(int i = 0 ; i < 10 ; i++){
            clientTab[i] = nullptr;
        }
        wifi_set_opmode(0x1);
        connect(ssid, password);
        wifi_set_event_handler_cb(handler);
    }
    

};

