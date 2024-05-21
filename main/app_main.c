#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "esp_system.h"
#include "esp_partition.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "protocol_examples_common.h"

#include "esp_log.h"
#include "mqtt_client.h"
#include "esp_tls.h"
#include "esp_ota_ops.h"
#include <sys/param.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/xtensa_api.h" // Include for xTaskGetTickCount()

#define SAMPLE_FREQ_MAX 1000
#define FFT_SIZE 1024   
#define AGGREGATE_TIME_WINDOW 5
#define VOLTAGE 3.3 // Voltage supplied to the device
#define CURRENT 0.2 // Current drawn by the device


static const char *TAG = "MQTT_EVENT:";


#if CONFIG_BROKER_CERTIFICATE_OVERRIDDEN == 1
static const uint8_t mqtt_eclipseprojects_io_pem_start[]  = "-----BEGIN CERTIFICATE-----\n" CONFIG_BROKER_CERTIFICATE_OVERRIDE "\n-----END CERTIFICATE-----";
#else
extern const uint8_t mqtt_eclipseprojects_io_pem_start[]   asm("_binary_mqtt_eclipseprojects_io_pem_start");
#endif
extern const uint8_t mqtt_eclipseprojects_io_pem_end[]   asm("_binary_mqtt_eclipseprojects_io_pem_end");


// Function to generate signal
float generate_signal(float t) {
    return 2 * sin(2 * M_PI * 3 * t) + 4 * sin(2 * M_PI * 5 * t);
}

// Function to perform FFT
void fft(float x_real[], float x_imag[], int N) {
    int i, j, k, m, m2;
    int n = 0;
    float angle;
    float cos_angle, sin_angle;
    float temp_real, temp_imag;
    float scale = 1.0 / N; // Scale factor

    // Calculate log2(N) as an integer
    for (i = N; i > 1; i >>= 1) {
        n++;
    }

    // Apply Hamming window to the input signal
    for (i = 0; i < N; i++) {
        float window = 0.54 - 0.46 * cos(2 * M_PI * i / (N - 1));
        x_real[i] *= window;
        x_imag[i] *= window;
    }

    // Bit-reverse permutation
    j = 0;
    for (i = 1; i < N; i++) {
        k = N / 2;
        while (j >= k) {
            j -= k;
            k /= 2;
        }
        j += k;
        if (i < j) {
            // Swap real parts
            temp_real = x_real[i];
            x_real[i] = x_real[j];
            x_real[j] = temp_real;
            // Swap imaginary parts
            temp_imag = x_imag[i];
            x_imag[i] = x_imag[j];
            x_imag[j] = temp_imag;
        }
    }

    // Cooley-Tukey FFT algorithm
    for (i = 1; i <= n; i++) {
        m = 1 << i; 
        m2 = m / 2;
        angle = -2 * M_PI / m;
        for (j = 0; j < N; j += m) {
            for (k = 0; k < m2; k++) {
                cos_angle = cos(k * angle);
                sin_angle = sin(k * angle);
                temp_real = cos_angle * x_real[j + k + m2] - sin_angle * x_imag[j + k + m2];
                temp_imag = cos_angle * x_imag[j + k + m2] + sin_angle * x_real[j + k + m2];
                x_real[j + k + m2] = x_real[j + k] - temp_real;
                x_imag[j + k + m2] = x_imag[j + k] - temp_imag;
                x_real[j + k] += temp_real;
                x_imag[j + k] += temp_imag;
            }
        }
    }

    // Scale the output by 1/N as in the cooley-tukey algorithm the magnitude of the transformed values are increased by a factor of N.
    for (i = 0; i < N; i++) {
        x_real[i] *= scale;
        x_imag[i] *= scale;
    }
}

float find_max_frequency(float x_real[], float x_imag[], int size, float sample_freq) {
    float max_magnitude = 0;
    int max_index = 0;

    for (int i = 1; i < size / 2; i++) { // Only consider up to Nyquist frequency and ignore DC component
        float magnitude = sqrt(x_real[i] * x_real[i] + x_imag[i] * x_imag[i]);
        if (magnitude > max_magnitude) {
            max_magnitude = magnitude;
            max_index = i;
        }
    }
    printf("max magnitude: %f\n", max_magnitude);
    printf("max index: %d\n", max_index);
    printf("sample freq: %f\n", sample_freq);
    printf("size: %d\n", size);
    float max_frequency = (max_index * sample_freq) / size;
    return max_frequency;
}
float calculate_time_window_aggregate(float signal[], int size, float sample_freq, float time_window) {
    int window_size = sample_freq * time_window; // Calculate the number of samples in the time window
    printf("window size %d\n", window_size);
    float aggregate = 0;
    int actual_size = (window_size < size) ? window_size : size; // Use the smaller of window_size or size
    for (int i = 0; i < actual_size; i++) {
        aggregate += signal[i];
    }
    return aggregate / actual_size; // Return the mean of the window
}

__attribute__((aligned(16)))
float signal_input_real[FFT_SIZE];

__attribute__((aligned(16)))
float signal_input_imag[FFT_SIZE];

size_t calculate_data_volume(int sample_freq, int sample_size) {
    return sample_freq * sample_size * sizeof(float); // Number of samples per second * sample size * size of each sample (float)
}

float sampled_aggregate_value = 0;
float resampled_aggregate_value = 0;
// Declare global variable to store start time and latency.
static TickType_t start_time;
TickType_t latency;


// Function to calculate energy consumption
float calculate_energy_consumption(float power_consumption, float time_seconds) {
    return power_consumption * time_seconds;
}


void task_signal_processing(void *pvParameters) {
    
    // Capture start time for latency
    start_time = xTaskGetTickCount();

        //Capture start time for power consumption
        TickType_t start_time_oversampled = xTaskGetTickCount();


        float sample_freq = SAMPLE_FREQ_MAX;
        float t = 0;    
        // Collect samples over 1 second and remove DC component
        float mean = 0;
        for (int i = 0; i < FFT_SIZE; i++) {
            signal_input_real[i] = generate_signal(t);
            mean += signal_input_real[i];
            t += 1.0 / sample_freq;
        }
        mean /= FFT_SIZE;
        for (int i = 0; i < FFT_SIZE; i++) {
            signal_input_real[i] -= mean;
            signal_input_imag[i] = 0; // Imaginary part is zero for real signals
        }

        // Print the generated signal
        printf("Generated Signal:\n");
        for (int i = 0; i < FFT_SIZE; i++) {
            printf("%.4f ,", signal_input_real[i]);
        }
        printf("\n");

        // Perform FFT
        fft(signal_input_real, signal_input_imag, FFT_SIZE);

        // Print FFT results
        printf("FFT Output (Real part):\n");
        for (int i = 0; i < FFT_SIZE / 2; i++) {
            printf("%f, ", signal_input_real[i]);
            vTaskDelay(1 / portTICK_PERIOD_MS);
        }
        printf("\n");
        printf("FFT Output (Imaginary part):\n");
        for (int i = 0; i < FFT_SIZE / 2; i++) {
            printf("%f, ", signal_input_imag[i]);
            vTaskDelay(1 / portTICK_PERIOD_MS);
        }
        printf("\n");

        // Calculate data volume for original oversampled scenario
        size_t original_data_volume = calculate_data_volume(SAMPLE_FREQ_MAX, FFT_SIZE);

        // Calculate the maximum frequency of the generated signal
        float max_freq = find_max_frequency(signal_input_real, signal_input_imag, FFT_SIZE, sample_freq);
        printf("Maximum frequency: %f Hz\n", max_freq);

        // Calculate the aggregate over a window of the sampled signal
        sampled_aggregate_value = calculate_time_window_aggregate(signal_input_real, FFT_SIZE, sample_freq, AGGREGATE_TIME_WINDOW);
        printf("Aggregate over %d seconds: %f\n", AGGREGATE_TIME_WINDOW, sampled_aggregate_value);

        // End the time for power consumption oversampled.
        TickType_t end_time_oversampled = xTaskGetTickCount();
        float time_seconds = (end_time_oversampled - start_time_oversampled) * portTICK_PERIOD_MS / 1000.0;
        float oversampled_energy_consumption = calculate_energy_consumption(CURRENT * VOLTAGE, time_seconds);
        printf("Energy consumption oversampled: %f J\n", oversampled_energy_consumption);
        // Adjust the sampling frequency based on the detected maximum frequency
        if (max_freq > 0 && max_freq < SAMPLE_FREQ_MAX / 2) {
            sample_freq = 2 * max_freq;
            if (sample_freq > SAMPLE_FREQ_MAX) {
                sample_freq = SAMPLE_FREQ_MAX;
            }
            printf("Adjusting sampling frequency to %f Hz.\n", sample_freq);
        } else {
            sample_freq = SAMPLE_FREQ_MAX;
            printf("Sampling frequency is optimal at %f Hz.\n", sample_freq);
        }

        printf("Orignal Data Volume: %d\n", original_data_volume);
        
        // Resample the signal with the new frequency
        t = 0; // Reset time
        mean = 0;
        //Start the time for resampled power consumption
        TickType_t start_time_resampled = xTaskGetTickCount();
        for (int i = 0; i < FFT_SIZE; i++) {
            signal_input_real[i] = generate_signal(t);
            mean += signal_input_real[i];
            t += 1.0 / sample_freq;
            //vTaskDelay(1 / portTICK_PERIOD_MS); // Yield control to other tasks
        }
        mean /= FFT_SIZE;
        for (int i = 0; i < FFT_SIZE; i++) {
            signal_input_real[i] -= mean;
            signal_input_imag[i] = 0; // Imaginary part is zero for real signals
            //vTaskDelay(1 / portTICK_PERIOD_MS); // Yield control to other tasks
        }

        // Perform FFT on the resampled signal
        fft(signal_input_real, signal_input_imag, FFT_SIZE);

        // Print FFT results of resampled signal
        printf("Resampled FFT Output (Real part):\n");
        for (int i = 0; i < FFT_SIZE / 2; i++) {
            printf("%f, ", signal_input_real[i]);
            vTaskDelay(1 / portTICK_PERIOD_MS); // Yield control to other tasks
        }
        printf("\n");
        printf("Resampled FFT Output (Imaginary part):\n");
        for (int i = 0; i < FFT_SIZE / 2; i++) {
            printf("%f, ", signal_input_imag[i]);
            vTaskDelay(1 / portTICK_PERIOD_MS); // Yield control to other tasks
        }
        printf("\n");
        //End the time for Resampled power consumption.
        TickType_t end_time_resampled = xTaskGetTickCount();
        time_seconds = (end_time_resampled - start_time_resampled) * portTICK_PERIOD_MS / 1000.0;
        float adaptive_energy_consumption = calculate_energy_consumption(CURRENT * VOLTAGE, time_seconds);
        // Calculate data volume for new/adaptive sampling frequency scenario
        size_t new_data_volume = calculate_data_volume(sample_freq, FFT_SIZE); // Assuming sample_freq is updated based on the scenariow

        // Calculate the maximum frequency of the resampled signal
        max_freq = find_max_frequency(signal_input_real, signal_input_imag, FFT_SIZE, sample_freq);
        printf("Maximum frequency after resampling: %f Hz\n", max_freq);

        // Calculate the aggregate over a window of the resampled signal
        resampled_aggregate_value = calculate_time_window_aggregate(signal_input_real, FFT_SIZE, sample_freq, AGGREGATE_TIME_WINDOW);
        printf("Aggregate over %d seconds: %f\n", AGGREGATE_TIME_WINDOW, resampled_aggregate_value);
        
        printf("Energy consumption for resampled is: %f J\n", adaptive_energy_consumption);
        printf("Energy savings: %f J\n", oversampled_energy_consumption - adaptive_energy_consumption);
        
        // Compare the data volumes
        printf("New data Volume: %d\n", new_data_volume);
        if (new_data_volume < original_data_volume) {
            printf("New/adaptive sampling frequency reduces data volume transmitted over the network.\n");
        } else if (new_data_volume > original_data_volume) {
            printf("New/adaptive sampling frequency increases data volume transmitted over the network.\n");
        } else {
            printf("Data volume remains the same for both scenarios.\n");
    }   

        // Delay to control the sampling rate
        vTaskDelete(NULL); 
}

//
// Note: this function is for testing purposes only publishing part of the active partition
//       (to be checked against the original binary)
//
static void send_binary(esp_mqtt_client_handle_t client)
{
    esp_partition_mmap_handle_t out_handle;
    const void *binary_address;
    const esp_partition_t *partition = esp_ota_get_running_partition();
    esp_partition_mmap(partition, 0, partition->size, ESP_PARTITION_MMAP_DATA, &binary_address, &out_handle);
    // sending only the configured portion of the partition (if it's less than the partition size)
    int binary_size = MIN(CONFIG_BROKER_BIN_SIZE_TO_SEND, partition->size);
    int msg_id = esp_mqtt_client_publish(client, "/topic/binary", binary_address, binary_size, 0, 0);
    ESP_LOGI(TAG, "binary sent with msg_id=%d", msg_id);
}

/*
 * @brief Event handler registered to receive MQTT events
 *
 *  This function is called by the MQTT client event loop.
 *
 * @param handler_args user data registered to the event.
 * @param base Event base for the handler(always MQTT Base in this example).
 * @param event_id The id for the received event.
 * @param event_data The data for the event, esp_mqtt_event_handle_t.
 */
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%" PRIi32, base, event_id);
    esp_mqtt_event_handle_t event = event_data;
    esp_mqtt_client_handle_t client = event->client;
    int msg_id;
    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:

        // Convert aggregate values to strings
        char sampled_aggregate[20]; 
        char resampled_aggregate[20]; 
        snprintf(sampled_aggregate, sizeof(sampled_aggregate), "%.10f", sampled_aggregate_value);
        snprintf(resampled_aggregate, sizeof(resampled_aggregate), "%.10f", resampled_aggregate_value);

        // Subscribe to topics to receive data
        msg_id = esp_mqtt_client_subscribe(client, "/topic/sample_aggregate", 1);
        ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);
        msg_id = esp_mqtt_client_subscribe(client, "/topic/resample_aggregate", 1);
        ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);
        
        // Publish sampled aggregate value
        msg_id = esp_mqtt_client_publish(client, "/topic/sample_aggregate", sampled_aggregate, 0, 1, 0);
        ESP_LOGI(TAG, "Sent sampled aggregate value, msg_id=%d", msg_id);

        // Publish resampled aggregate value
        msg_id = esp_mqtt_client_publish(client, "/topic/resample_aggregate", resampled_aggregate, 0, 1, 0);
        ESP_LOGI(TAG, "Sent resampled aggregate value, msg_id=%d", msg_id);
        
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");

        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
        break;

    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
        msg_id = esp_mqtt_client_publish(client, "/topic/sampled_aggregate", sampled_aggregate, 0, 1, 0);
        msg_id = esp_mqtt_client_publish(client, "/topic/resampled_aggregate", resampled_aggregate, 0, 1, 0);
        ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
        break;
    case MQTT_EVENT_UNSUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_PUBLISHED:
        // Capture time when data is published
        TickType_t publish_time = xTaskGetTickCount();
        // Calculate latency
        latency = publish_time - start_time;
        ESP_LOGI(TAG, "End-to-end latency: %u ms", (unsigned int)(latency * portTICK_PERIOD_MS));
        ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
        printf("Latency of the system is %ld0\r\n", latency);
        break;
    case MQTT_EVENT_DATA:
        ESP_LOGI(TAG, "MQTT_EVENT_DATA");
        printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
        printf("DATA=%.*s\r\n", event->data_len, event->data);
        if (strncmp(event->data, "send binary please", event->data_len) == 0) {
            ESP_LOGI(TAG, "Sending the binary");
            send_binary(client);
        }
        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
        if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
            ESP_LOGI(TAG, "Last error code reported from esp-tls: 0x%x", event->error_handle->esp_tls_last_esp_err);
            ESP_LOGI(TAG, "Last tls stack error number: 0x%x", event->error_handle->esp_tls_stack_err);
            ESP_LOGI(TAG, "Last captured errno : %d (%s)",  event->error_handle->esp_transport_sock_errno,
                     strerror(event->error_handle->esp_transport_sock_errno));
        } else if (event->error_handle->error_type == MQTT_ERROR_TYPE_CONNECTION_REFUSED) {
            ESP_LOGI(TAG, "Connection refused error: 0x%x", event->error_handle->connect_return_code);
        } else {
            ESP_LOGW(TAG, "Unknown error type: 0x%x", event->error_handle->error_type);
        }
        break;
    default:
        ESP_LOGI(TAG, "Other event id:%d", event->event_id);
        break;
    }
}

static void mqtt_app_start(void)
{
    const esp_mqtt_client_config_t mqtt_cfg = {
        .broker = {
            .address.uri = CONFIG_BROKER_URI,
            .verification.certificate = (const char *)mqtt_eclipseprojects_io_pem_start
        },
    };

    ESP_LOGI(TAG, "[APP] Free memory: %" PRIu32 " bytes", esp_get_free_heap_size());
    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
    /* The last argument may be used to pass data to the event handler, in this example mqtt_event_handler */
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);
}

void app_main(void)
{   

    ESP_LOGI(TAG, "[APP] Startup..");
    ESP_LOGI(TAG, "[APP] Free memory: %" PRIu32 " bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "[APP] IDF version: %s", esp_get_idf_version());

    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set("esp-tls", ESP_LOG_VERBOSE);
    esp_log_level_set("mqtt_client", ESP_LOG_VERBOSE);
    esp_log_level_set("mqtt_example", ESP_LOG_VERBOSE);
    esp_log_level_set("transport_base", ESP_LOG_VERBOSE);
    esp_log_level_set("transport", ESP_LOG_VERBOSE);
    esp_log_level_set("outbox", ESP_LOG_VERBOSE);

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
     * Read "Establishing Wi-Fi or Ethernet Connection" section in
     * examples/protocols/README.md for more information about this function.
     */
    ESP_ERROR_CHECK(example_connect());
    
    // Create signal processing task
    xTaskCreate(task_signal_processing, "signal_processing_task", 4096, NULL, 5, NULL);

    mqtt_app_start();
}
