#pragma once
//#include <memory>

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



using uartWifi =   etl::Uart1<>;
void ICACHE_FLASH_ATTR writeUartWifi(sint8 s)
{
    uartWifi::write((char)(s+48));
    uartWifi::write('\r');
    uartWifi::write('\n');
}

void ICACHE_FLASH_ATTR writeUartWifi(const char * string)
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

extern "C" ICACHE_FLASH_ATTR void abort() {
    while (true)
        ; // enter an infinite loop and get reset by the WDT
}



namespace std
{
	void ICACHE_FLASH_ATTR __throw_bad_function_call()
    {
        abort();
    }

	void ICACHE_FLASH_ATTR  __throw_length_error(char const*)
    {
        abort();
    }

	void ICACHE_FLASH_ATTR __throw_bad_alloc()
    {
        abort();
    }

	void ICACHE_FLASH_ATTR __throw_logic_error(const char* str)
    {
        abort();
    }

	void ICACHE_FLASH_ATTR __throw_out_of_range(const char* str)
    {
        abort();
    }
}

void *  ICACHE_FLASH_ATTR operator new(size_t size) {
    auto rsize = os_malloc(size);
    return rsize;
}

void *  ICACHE_FLASH_ATTR operator new[](size_t size) {
    return os_malloc(size);
}

void ICACHE_FLASH_ATTR operator delete(void * ptr, size_t) {
    os_free(ptr);
}

void ICACHE_FLASH_ATTR operator delete[](void * ptr, size_t) {
    os_free(ptr);
} 

void ICACHE_FLASH_ATTR operator delete(void * ptr) {
    os_free(ptr);
}

void ICACHE_FLASH_ATTR operator delete[](void * ptr) {
    os_free(ptr);
} 

extern "C" void __cxa_pure_virtual(void) __attribute__((__noreturn__));
extern "C" void __cxa_deleted_virtual(void) __attribute__((__noreturn__));


void ICACHE_FLASH_ATTR __cxa_pure_virtual(void) {
    abort();
}

void ICACHE_FLASH_ATTR __cxa_deleted_virtual(void) {
    abort();
} 

void ICACHE_FLASH_ATTR dnsResolved(const char *name, ip_addr_t *ipaddr, void *arg);
void ICACHE_FLASH_ATTR response(void *arg, char *pdata, unsigned short len);

using responseReceived = void(*)(char * receivedData);


class QueryParam {
public:
    auto& setPath(const char* pathP) {
        path = pathP;
        return *this;
    }

    auto& setData(const char* dataP) {
        data = dataP;
        return *this;
    }

    auto& setContentType(const char* contentpeP) {
        contentype = contentpeP;
        return *this;
    }
    
    auto& setCallBack(responseReceived callBackP) {
        callback = callBackP;
        return *this;
    }
    
    void setPost()
    {
        currentyType = QueryParam::POST;
    }
    
    void setGet()
    {
        currentyType = QueryParam::GET;
    }


    enum REQUEST_TYPE
    {
        POST = 13, 
        GET = 14
    };
    
    REQUEST_TYPE currentyType;
    
    const char * path;
    const char * contentype;
    const char * data;
    responseReceived callback = nullptr;
    char buffer[2048];
};

class Client
{
public:
      
    Client(const char * hostP)
        : host(hostP) {}
    ~Client() {}
  
    void send()
    {
        run();
    }

	QueryParam& buildPost() {
    	queryParam.setPost();
    	return queryParam;
    }
   
    QueryParam& buildGet() {
        queryParam.setGet();
        return queryParam;
    }
  
    bool isBusy()
    {
        return busy;
    }
    
    const char * host;
   
    QueryParam queryParam;
    
    bool busy = false;
    
    espconn connection;
    ip_addr_t  ip;
    esp_tcp  tcp;
    
private:
    void run()
    {
        busy = true;
        auto value = espconn_gethostbyname(&connection, host, &ip, dnsResolved);
    }
};

Client* clientTab[10];

struct ClientManager
{
    auto static createClient(const char * hostP)
    {
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
        for (int i = 0; i < clientIndex; i++)
        {
            auto clientPtr = clientTab[i];
            if (connection == &(clientPtr->connection))
            {
                return clientPtr;
            }
            
        }
        return nullptr ;
    }
    
    static int clientIndex;
    
};

int ClientManager::clientIndex = 0;

void ICACHE_FLASH_ATTR connected(void *arg)
{
    writeUartWifi("tcp_connected");
    auto clientPtr = ClientManager::getClient((espconn *)arg);
    auto currentyType = clientPtr->queryParam.currentyType;
    switch (currentyType)
    {
    case QueryParam::POST :
    {
        writeUartWifi("POST");
        os_sprintf(clientPtr->queryParam.buffer,
            "POST %s HTTP/1.1 \r\nHost: %s \r\nCache-Control: no-cache \r\nContent-Type: %s \r\nContent-Length: %d \r\n\r\n%s",
            clientPtr->queryParam.path,
            clientPtr->host,
            clientPtr->queryParam.contentype,
            os_strlen(clientPtr->queryParam.data),
            clientPtr->queryParam.data);
    
        writeUartWifi("Sending: ");
        writeUartWifi(clientPtr->queryParam.buffer);
        auto value = espconn_sent(&(clientPtr->connection), (uint8_t *)(clientPtr->queryParam.buffer), os_strlen(clientPtr->queryParam.buffer));
        clientPtr->busy = false;
    }
        break;
    case QueryParam::GET :
           break;
    }
     
}

void ICACHE_FLASH_ATTR response(void *arg, char *pdata, unsigned short len)
{
    auto clientPtr = ClientManager::getClient((espconn *)arg);
   
    if (clientPtr->queryParam.callback)
    {
        clientPtr->queryParam.callback(pdata);
    }
}


void ICACHE_FLASH_ATTR disconnected(void *arg)
{
    writeUartWifi("tcp_disconnected");
}

void ICACHE_FLASH_ATTR dnsResolved(const char *name, ip_addr_t *ipaddr, void *arg)
{
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
        auto connection = &(clientPtr->connection);
        connection->type = ESPCONN_TCP;
        connection->state = ESPCONN_NONE;
        connection->proto.tcp = &(clientPtr->tcp);
        connection->proto.tcp->local_port = espconn_port();
        connection->proto.tcp->remote_port = 80;
        os_memcpy(connection->proto.tcp->remote_ip, &ipaddr->addr, 4);
        espconn_regist_connectcb(connection, connected);
        espconn_regist_disconcb(connection, disconnected);
        espconn_regist_recvcb(connection, response);
        espconn_connect(connection);
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

