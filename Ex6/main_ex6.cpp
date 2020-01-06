/* Active click feedback Example */
#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>
#include <nvs_flash.h>
#include <esp_system.h>
#include <esp_log.h>
#include <esp_task_wdt.h>
#include <esp_spi_flash.h>
#include <rom/uart.h>
#include <driver/uart.h>
#include <driver/adc.h>
#include <driver/mcpwm.h>
#include <math.h>

#include <string.h>
#include <time.h>
#include <sys/time.h>
#include <esp_wifi.h>
#include <esp_event_loop.h>
#include <lwip/err.h>
#include <lwip/sockets.h>
#include <lwip/sys.h>
#include <lwip/netdb.h>
#include <lwip/dns.h>

#include <cJSON.h>

#include <freertos/queue.h>

const char* TAG = "main";

//----------------------------------------------------------------

#define MAX_COUNT 1000
#define SEND_SAMPLE_INTERVAL 10
#define SEND_SIZE  (MAX_COUNT/SEND_SAMPLE_INTERVAL)

int ad_w=0;
int ad_last=0;
int ad_list[MAX_COUNT];
// time_t now_w=0;
int64_t now_w=0;
// time_t now_list[MAX_COUNT];
int64_t now_last=0;
double now_list[MAX_COUNT];
double freq_value=200.0;

int count = 0;

//----------------------------------------------------------------

QueueHandle_t  q=NULL;

typedef struct queue_data_st {
      int ad;
      double now;
} queue_data_t;

#define QUEUE_LENGTH 100

//----------------------------------------------------------------
/* The examples use simple WiFi configuration that you can set via
   'make menuconfig'.

   If you'd rather not, just change the below entries to strings with
   the config you want - ie #define EXAMPLE_WIFI_SSID "mywifissid"
*/
// #define EXAMPLE_WIFI_SSID CONFIG_WIFI_SSID
// #define EXAMPLE_WIFI_PASS CONFIG_WIFI_PASSWORD

/* FreeRTOS event group to signal when we are connected & ready to make a request */
static EventGroupHandle_t wifi_event_group;

/* The event group allows multiple bits for each event,
   but we only care about one event - are we connected
   to the AP with an IP? */
const int CONNECTED_BIT = BIT0;

// need to rewrite for WiFi AP
// #define WIFI_SSID "ICTEX51"
// #define WIFI_PASS "espwroom32"
#define WIFI_SSID "ICT-EX5"
#define WIFI_PASS "embedded"
// need to rewrite for your RPi
// RPi Flask
#define WEB_SERVER "10.0.0.1"
#define WEB_PORT "5000"
#define WEB_URL "http://10.0.0.1:5000/getadclist?"
// PC -flask
//#define WEB_SERVER "172.16.11.161"
//#define WEB_PORT "5000"
//#define WEB_URL "http://172.16.11.161:5000/getadclist?"

//----------------------------------------------------------------
// WiFi Initialize

static esp_err_t event_handler(void *ctx, system_event_t *event)
{
//    ESP_LOGI(TAG, "event_handler.");
    switch(event->event_id) {
    case SYSTEM_EVENT_STA_START:
//        ESP_LOGI(TAG, "event_handler. SYSTEM_EVENT_STA_START");
        esp_wifi_connect();
        break;
    case SYSTEM_EVENT_STA_GOT_IP:
//        ESP_LOGI(TAG, "event_handler. SYSTEM_EVENT_STA_GOT_IP");
        xEventGroupSetBits(wifi_event_group, CONNECTED_BIT);
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
        /* This is a workaround as ESP32 WiFi libs don't currently
           auto-reassociate. */
//        ESP_LOGI(TAG, "event_handler. SYSTEM_EVENT_STA_DISCONNECTED");
        esp_wifi_connect();
        xEventGroupClearBits(wifi_event_group, CONNECTED_BIT);
        break;
    default:
        break;
    }
    return ESP_OK;
}

static void initialise_wifi(void)
{
    wifi_event_group = xEventGroupCreate();

    tcpip_adapter_init();
    ESP_ERROR_CHECK( esp_event_loop_init(event_handler, NULL) );
#if 0
/* start static IP addr */
    tcpip_adapter_dhcpc_stop(TCPIP_ADAPTER_IF_STA);

    tcpip_adapter_ip_info_t ipInfo;
    IP4_ADDR(&ipInfo.ip, 172,16,11,110);
    IP4_ADDR(&ipInfo.gw, 172,16,11,251);
    IP4_ADDR(&ipInfo.netmask, 255,255,255,0);
    tcpip_adapter_set_ip_info(TCPIP_ADAPTER_IF_STA, &ipInfo);

    ip_addr_t dnsserver;
    IP_ADDR4( &dnsserver, 172,16,11,251);
    dns_setserver(0, &dnsserver);
/* end static IP addr */
#endif
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
/* ------------------------*/
//    ESP_LOGI(TAG, "Setting WiFi configuration SSID %s...", WIFI_SSID);
    wifi_config_t wifi_config;
    memset(&wifi_config, 0, sizeof(wifi_config));
    sprintf (reinterpret_cast<char*>(wifi_config.sta.ssid), WIFI_SSID );
    sprintf (reinterpret_cast<char*>(wifi_config.sta.password), WIFI_PASS);
//    wifi_config_t wifi_config = { };
//    wifi_config.sta.ssid=(char *) WIFI_SSID;
//    wifi_config.sta.password=(char *) WIFI_PASS;
/* ------------------------*/
//    wifi_config_t wifi_config = {
//        .sta = {
//            { .ssid = WIFI_SSID },
//            { .password = WIFI_PASS },
////            { .ssid = EXAMPLE_WIFI_SSID },
////            { .password = EXAMPLE_WIFI_PASS },
//        },
//    };
/* ------------------------*/
    ESP_LOGI(TAG, "Setting WiFi configuration SSID %s...", wifi_config.sta.ssid);
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK( esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
    ESP_ERROR_CHECK( esp_wifi_start() );
}

//----------------------------------------------------------------
// Data Send and receive

static void http_get_task(void *pvParameters)
{

//    const struct addrinfo hints = {
//        { .ai_family = AF_INET },
//        { .ai_socktype = SOCK_STREAM }
//    };
    struct addrinfo hints;
    memset(&hints, 0, sizeof(hints));
    hints.ai_family   = AF_INET;
    hints.ai_socktype = SOCK_STREAM;

    struct addrinfo *res;
    struct in_addr *addr;
    int s, r;
    char recv_buf[64];

    int    count_buf;
//    int    ad_buf_list[MAX_COUNT];
//    double now_buf_list[MAX_COUNT];
    int    ad_send_list[MAX_COUNT];
    double now_send_list[MAX_COUNT];
    int i, k;

    queue_data_t qd;

    while (1) {
        const char *REQUEST1 = "GET " WEB_URL;
        const char *REQUEST2 = " HTTP/1.0\r\n"
            "Host: " WEB_SERVER "\r\n"
            "User-Agent: esp-idf/1.0 esp32\r\n"
            "\r\n";
        char REQUEST[18000];/* 1000x8x2+alpha */

//        xQueueReceive(q,&pack_r,(TickType_t )(1000/portTICK_PERIOD_MS));
        ESP_LOGI(TAG, "restart http_get_task : ADC %d time %lld count %d", ad_w, now_w, count);

        /* Wait for the callback to set the CONNECTED_BIT in the
           event group.
        */
//        ESP_LOGI(TAG, "Start Connection to AP: ADC %d", ad);
        xEventGroupWaitBits(wifi_event_group, CONNECTED_BIT,
                            false, true, portMAX_DELAY);
        ESP_LOGI(TAG, "Connected to AP");

        int err = getaddrinfo(WEB_SERVER, WEB_PORT, &hints, &res);

        if(err != 0 || res == NULL) {
            ESP_LOGE(TAG, "DNS lookup failed err=%d res=%p", err, res);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            continue;
        }

        /* Code to print the resolved IP.

           Note: inet_ntoa is non-reentrant, look at ipaddr_ntoa_r for "real" code */
        addr = &((struct sockaddr_in *)res->ai_addr)->sin_addr;
        ESP_LOGI(TAG, "DNS lookup succeeded. IP=%s", inet_ntoa(*addr));

        s = socket(res->ai_family, res->ai_socktype, 0);
        if(s < 0) {
            ESP_LOGE(TAG, "... Failed to allocate socket.");
            freeaddrinfo(res);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            continue;
         }
        ESP_LOGI(TAG, "... allocated socket");

        if(connect(s, res->ai_addr, res->ai_addrlen) != 0) {
            ESP_LOGE(TAG, "... socket connect failed errno=%d", errno);
            close(s);
            freeaddrinfo(res);
            vTaskDelay(4000 / portTICK_PERIOD_MS);
            continue;
         }

        ESP_LOGI(TAG, "... connected");
        freeaddrinfo(res);

        /* ------------------------*/
//  get ad and now (time) list from global var.
//  pack them as cjson object and array
// sample the ad and now time

//        ESP_LOGI(TAG, "Count1 %d", count);
        count_buf = count;
//        for ( i=0; i<MAX_COUNT; i++) {
//            ad_buf_list[i]=ad_list[i];
//            now_buf_list[i]=now_list[i];
//        }
//        ESP_LOGI(TAG, "Count2 %d", count);
        for ( i=count_buf, k=0; i<MAX_COUNT; i+=SEND_SAMPLE_INTERVAL, k++) {
            ad_send_list[k]=ad_list[i];
            now_send_list[k]=now_list[i];
        }
        ESP_LOGI(TAG, "Count11 %d i %d k %d ", count_buf, i, k);
        for ( i=count_buf%SEND_SAMPLE_INTERVAL; i<count_buf; i+=SEND_SAMPLE_INTERVAL, k++) {
            ad_send_list[k]=ad_list[i];
            now_send_list[k]=now_list[i];
        }
        ESP_LOGI(TAG, "Count12 %d i %d k %d ", count_buf, i, k);
//        ESP_LOGI(TAG, "Count3 %d", count);

        cJSON *roots = NULL;
        char *obj, *op;
        roots = cJSON_CreateObject();
        cJSON_AddItemToObject(roots, "adc", cJSON_CreateIntArray(ad_send_list, SEND_SIZE));
        cJSON_AddItemToObject(roots, "time", cJSON_CreateDoubleArray(now_send_list, SEND_SIZE));
        cJSON_AddNumberToObject(roots, "count", count_buf);

        strcpy(REQUEST, REQUEST1);
        obj=cJSON_PrintUnformatted(roots);
//        for (op=obj; *op!='\0'; op++)
//             if (*op=='\n' || *op=='\r') *op=' ';
        sprintf(REQUEST + strlen(REQUEST), "JSON=%s", obj);
        cJSON_Delete(roots);
        strcat(REQUEST, REQUEST2);
//        ESP_LOGI(TAG, "SEND REQUEST %s", REQUEST);
        if (write(s, REQUEST, strlen(REQUEST)) < 0) {
            ESP_LOGE(TAG, "... socket send failed");
            close(s);
            vTaskDelay(4000 / portTICK_PERIOD_MS);
            continue;
        }
        ESP_LOGI(TAG, "... socket send success");
        /* ------------------------*/

        struct timeval receiving_timeout;
        receiving_timeout.tv_sec = 5;
        receiving_timeout.tv_usec = 0;
        if (setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, &receiving_timeout,
                sizeof(receiving_timeout)) < 0) {
            ESP_LOGE(TAG, "... failed to set socket receiving timeout");
            close(s);
            vTaskDelay(4000 / portTICK_PERIOD_MS);
            continue;
        }
        ESP_LOGI(TAG, "... set socket receiving timeout success");

        /* Read HTTP response */
        char  buf[1024];
        buf[0] = '\0';
        do {
            bzero(recv_buf, sizeof(recv_buf));
            r = read(s, recv_buf, sizeof(recv_buf)-1);
            for(int i = 0; i < r; i++) {
                putchar(recv_buf[i]);
            }
            strcat(buf, recv_buf);
        } while(r > 0);
        /* ------------------------*/
        // printf("Buffer %s\n", buf);

        cJSON *root = NULL;
        char  *bufptr;

        for (bufptr=buf; *bufptr != '\0'&& *bufptr != '{'; bufptr++) ;
        if (*bufptr == '{') {
            // printf("Buffer stripped %s\n", bufptr);
            root=cJSON_Parse(bufptr);
            freq_value=atof(cJSON_Print(cJSON_GetObjectItem(root, "freq")));
            printf("received freq_value = %lf", freq_value);
            if (freq_value<100) freq_value=100;
            else if (freq_value>500) freq_value=500;
            printf(" modified freq_value = %lf\n", freq_value);
        }
        /* ------------------------*/

        ESP_LOGI(TAG, "... done reading from socket. Last read return=%d errno=%d\r\n", r, errno);
        close(s);
        vTaskDelay(1000 / portTICK_PERIOD_MS); /* wait for 1000 ms */
    } /* end while(1) */
}

//----------------------------------------------------------------
// #define USE_TIMER   //  Whther use the timer or not. Without this definition, the function is called from a normal task.

#ifdef USE_TIMER
# define DT 0.0001  //  In the case of the timer, the minimum period is 50 micro second.
#else
# define DT (1.0/configTICK_RATE_HZ)
                    //  In the case of the task, the time period is the time slice of the OS specified in menuconfig,
                    //  which is set to 1 ms=1 kHz.
#endif


struct WaveParam{
    const double damp[3] = {-10, -20, -30};
    const int nDamp = sizeof(damp) / sizeof(damp[0]);
    const double freq[4] = {100, 200, 300, 500};
    const int nFreq = sizeof(freq)/sizeof(freq[0]);
    const double amplitude = 2;
} wave;    //
// int count = 0;
double time_c = -1;

void hapticFunc(void* arg){
    const char* TAG = "H_FUNC";
    static int i;               //  An integer to select waveform.
    static double omega = 0;    //  angular frequency
    static double B=0;          //  damping coefficient
    int ad=0;
    int64_t now;
//    time_t now;
    queue_data_t qd;

/* -------- */
    // int ad = adc1_get_raw(ADC1_CHANNEL_6);
    ad = adc1_get_raw(ADC1_CHANNEL_6);
    ad_list[count]=ad;
    now = esp_timer_get_time();
//    time(&now);
    now_list[count]=(double) ((int64_t) (now/1000));
    ad_w = ad;
    now_w = now;
//    ESP_LOGI(TAG, "call http_get_task : ADC %d time %lld count %d", ad, now, count);
//    http_data_send();
//    xTaskCreate(&http_get_task, "http_get_task", 4096, NULL, 5, NULL);

/* -------- */
    if (ad < 2100 && time_c > 0.3){
        time_c = -1;
        printf("\r\n");
    }
    if (ad > 2400 && time_c == -1){   //  When the button is pushed after finishing to output an wave.
//        ESP_LOGI(TAG, "call http_get_task : ADC %d time %lld count %d", ad, now, count);
//        ad_w = ad; now_w = now;
//        http_data_send(ad, now);

        //  set the time_c to 0 and update the waveform parameters.
        time_c = 0;
// Frequency
//        omega = wave.freq[i % wave.nFreq] * M_PI * 2;
        omega = freq_value * M_PI * 2;
        B = wave.damp[i/wave.nFreq];
        printf("Wave: %3.1fHz, A=%2.2f, B=%3.1f ", omega/(M_PI*2), wave.amplitude, B);
        i++;
        if (i >= wave.nFreq * wave.nDamp) i = 0;
    }
    //  Output the wave
    double pwm = 0;
    if (time_c >= 0){
        pwm = wave.amplitude * cos (omega * time_c) * exp(B*time_c);
        time_c += DT;
    }else{
        pwm = 0;
    }
    //  Rotating direction
    if (pwm > 0){
        gpio_set_level(GPIO_NUM_5, 0);
        gpio_set_level(GPIO_NUM_17, 1);
#       ifndef USE_TIMER
        if (time_c >= 0) printf("+");
#       endif
    }else{
        gpio_set_level(GPIO_NUM_5, 1);
        gpio_set_level(GPIO_NUM_17, 0);
        pwm = -pwm;
#       ifndef USE_TIMER
        if (time_c >= 0) printf("-");
#       endif
    }
    if (pwm > 1) pwm = 1;

    //  Set duty rate of pwm
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, pwm* 100);
    mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
    count ++;
//    if (count >= 1000 ){
    if (count >= 1000 ){
        ESP_LOGI(TAG, "count 1000 : ADC %d time %lld count %d", ad, now, count);
//        ad_w = ad; now_w = now;
//        http_data_send(ad, now);
////        xTaskCreate(&http_get_task, "http_get_task", 4096, NULL, 5, NULL);
        count = 0;
    }
}

#ifndef USE_TIMER
void hapticTask(void* arg){
    while(1){
        hapticFunc(arg);
        vTaskDelay(1);
    }
}
#endif

extern "C" void app_main()
//void app_main()
{
    q=xQueueCreate(QUEUE_LENGTH, sizeof(queue_data_t));

    //----------------------------------
    //Initialize NVS
    ESP_LOGI("main", "Initialize NVS");
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    //----------------------------------

    /* Print chip information */
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    printf("This is ESP32 chip with %d CPU cores, WiFi%s%s, ",
        chip_info.cores,
        (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
        (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");
    printf("silicon revision %d, ", chip_info.revision);
    printf("%dMB %s flash\n", spi_flash_get_chip_size() / (1024 * 1024),
            (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    //----------------------------------
    ESP_LOGI("main", "Initialize WiFi");
    initialise_wifi();
    //----------------------------------
    printf("!!! Active Haptic Feedback Start !!!\n");

    ESP_LOGI("main", "Initialize ADC");
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_11);
    ESP_LOGI("main", "Initialize PWM");
    //1. mcpwm gpio initialization
    const int GPIO_PWM0A_OUT = 16;
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, GPIO_PWM0A_OUT);
    //2. initial mcpwm configuration
    printf("Configuring Initial Parameters of mcpwm...\n");
    mcpwm_config_t pwm_config;
    pwm_config.frequency = 1000;    //frequency = 1000Hz,
    pwm_config.cmpr_a = 0;          //duty cycle of PWMxA = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);    //Configure PWM0A with above settings
    mcpwm_set_frequency(MCPWM_UNIT_0, MCPWM_TIMER_0, 20000);
    gpio_config_t conf;
    conf.pin_bit_mask = (1 << (17)) | (1 << (5));
    conf.mode = GPIO_MODE_OUTPUT;
    conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    conf.pull_up_en = GPIO_PULLUP_DISABLE;
    conf.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&conf);

#ifdef USE_TIMER
    esp_timer_init();
    esp_timer_create_args_t timerDesc={
        callback: hapticFunc,
        arg: NULL,
        dispatch_method: ESP_TIMER_TASK,
        name: "haptic"
    };
    esp_timer_handle_t timerHandle = NULL;
    esp_timer_create(&timerDesc, &timerHandle);
    esp_timer_start_periodic(timerHandle, (int)(1000*1000*DT));     // period in micro second (100uS=10kHz)
#else
    TaskHandle_t taskHandle = NULL;
    xTaskCreate(hapticTask, "Haptic", 1024 * 15, NULL, 6, &taskHandle);
#endif
    xTaskCreate(&http_get_task, "http_get_task", 1024 * 40, NULL, 3, NULL);


    uart_driver_install(UART_NUM_0, 1024, 1024, 10, NULL, 0);
    while(1){
        uint8_t ch;
        uart_read_bytes(UART_NUM_0, &ch, 1, portMAX_DELAY);
        printf("'%c' received.\r\n", ch);
        switch(ch){
            case 'a':
            //  do something
            break;
        }
    }
}
