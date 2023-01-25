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
#include <lwip/err.h>
#include <lwip/sockets.h>
#include <lwip/sys.h>
#include <lwip/netdb.h>
#include <lwip/dns.h>

#include <cJSON.h>

const char* TAG = "main";

//----------------------------------------------------------------

#define MAX_COUNT 1000

int ad_w=0;

int count = 0;

double freq_value=200.0;

//----------------------------------------------------------------
/* The examples use simple WiFi configuration that you can set via
   'make menuconfig'.

   If you'd rather not, just change the below entries to strings with
   the config you want - ie #define EXAMPLE_WIFI_SSID "mywifissid"
*/
// #define EXAMPLE_WIFI_SSID CONFIG_WIFI_SSID
// #define EXAMPLE_WIFI_PASS CONFIG_WIFI_PASSWORD

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WEP


/* The event group allows multiple bits for each event,
   but we only care about one event - are we connected
   to the AP with an IP? */
const int CONNECTED_BIT = BIT0;

#define WIFI_SSID "ICT_EX5_2G"
#define WIFI_PASS "embedded"
#define WEB_SERVER "10.0.1.6"                       //  <- Use your raspberry pi's ip address here.
#define WEB_URL "http://10.0.1.6/getadc?ADC="       //  <- Use your raspberry pi's ip address here.
#define WEB_PORT "50000"

static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        esp_wifi_connect();
        xEventGroupClearBits(s_wifi_event_group, CONNECTED_BIT);
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        xEventGroupSetBits(s_wifi_event_group, CONNECTED_BIT);
    }
}
static void initialise_wifi(void)
{
    s_wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
            /* Authmode threshold resets to WPA2 as default if password matches WPA2 standards (pasword len => 8).
             * If you want to connect the device to deprecated WEP/WPA networks, Please set the threshold value
             * to WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK and set the password with length and format matching to
         * WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK standards.
             */
            .threshold.authmode = ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD,
            .sae_pwe_h2e = WPA3_SAE_PWE_BOTH,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );
    ESP_LOGI(TAG, "wifi_init_sta finished.");
    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);
    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "connected to ap SSID:%s password:%s",
                 WIFI_SSID, WIFI_PASS);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
                 WIFI_SSID, WIFI_PASS);
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }
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
    char recv_buf[128];

    while (1) {
        const char *REQUEST1 = "GET " WEB_URL;
        const char *REQUEST2 = " HTTP/1.0\r\n"
            "Host: " WEB_SERVER "\r\n"
            "User-Agent: esp-idf/1.0 esp32\r\n"
            "\r\n";
        char REQUEST[2048];

        ESP_LOGI(TAG, "restart http_get_task : ADC %d count %d", ad_w, count);

        /* Wait for the callback to set the CONNECTED_BIT in the
           event group.
        */
//        ESP_LOGI(TAG, "Start Connection to AP: ADC %d", ad);
        xEventGroupWaitBits(s_wifi_event_group, CONNECTED_BIT,
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

//  get adc value from global var.

        strcpy(REQUEST, REQUEST1);
        sprintf(REQUEST + strlen(REQUEST), "ADC=%d", ad_w);
        strcat(REQUEST, REQUEST2);
        if (write(s, REQUEST, strlen(REQUEST)) < 0) {
            ESP_LOGE(TAG, "... socket send failed");
            close(s);
            vTaskDelay(4000 / portTICK_PERIOD_MS);
            continue;
        }
        ESP_LOGI(TAG, "... socket send success");

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
        //   printf("Buffer stripped %s\n", bufptr);
            root=cJSON_Parse(bufptr);
            if (root == NULL) {
               ESP_LOGE(TAG, "... received wrong json format");
               continue;
            }
            if (cJSON_GetObjectItem(root, "freq")==NULL) {
               ESP_LOGE(TAG, "... freq not found");
               continue;
            }
        //    printf("freq: %s\n", cJSON_Print(cJSON_GetObjectItem(root, "freq")));
            freq_value=atof(cJSON_Print(cJSON_GetObjectItem(root, "freq")));
            printf("received freq_value = %lf\n", freq_value);
        /* check the received freq_value */
            if (freq_value<100) freq_value=100;
            else if (freq_value>500) freq_value=500;
            printf(" modified freq_value = %lf\n", freq_value);
        }

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
    const double damp[3];
    const int nDamp;
    const double freq[4];
    const int nFreq;
    const double amplitude;
} wave = {{-10, -20, -30},sizeof(wave.damp) / sizeof(wave.damp[0]), {100, 200, 300, 500}, sizeof(wave.freq)/sizeof(wave.freq[0]), 2};


double time_c = -1;

void hapticFunc(void* arg){
    const char* TAG = "H_FUNC";
    static int i;               //  An integer to select waveform.
    static double omega = 0;    //  angular frequency
    static double B=0;          //  damping coefficient
    int ad=0;

/* -------- */
    // int ad = adc1_get_raw(ADC1_CHANNEL_6);
    ad = adc1_get_raw(ADC1_CHANNEL_6);
    ad_w = ad;

/* -------- */
    if (ad < 2100 && time_c > 0.3){
        time_c = -1;
        printf("\r\n");
    }
    if (ad > 2400 && time_c == -1){   //  When the button is pushed after finishing to output an wave.
        //  set the time_c to 0 and update the waveform parameters.
        time_c = 0;
// Frequency
        omega = freq_value * M_PI * 2;
        // omega = wave.freq[i % wave.nFreq] * M_PI * 2;
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
    if (count >= 1000 ){
        ESP_LOGI(TAG, "count 1000 : ADC %d count %d", ad, count);
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

// extern "C" void app_main()
void app_main()
{
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
    xTaskCreate(&http_get_task, "http_get_task", 1024 * 15, NULL, 3, NULL);


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