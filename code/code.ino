/**
 * Example for Getting Started with nRF24L01+ radios. 
 * Basic sample program for recieving packets via the NRF24L01+chip using the nRF24L01.h lib
 * Tested on the Arduino
 */
#include "nRF24L01.h"
#include "RF24.h"
#include "printf.h"
#include "dht.h"
#include <SoftwareSerial.h>
//#include <string.h>
//#include <stdlib.h>

dht DHT;

#define DATA_PERCISION 5     // (DTH11 Data) PD2
#define DHT11_PIN 2          // (DTH11 Data) PD2
#define DHT11_SAMPLE_SIZE 10 //serial id for the device
#define DHT11_MAX_ERRORS 10  //serial id for the device
#define RX 0                 // (PCINT16/RXD) PD0
#define TX 1                 // (PCINT17/TXD) PD1
#define SERIAL 0x01          //serial id for the device
#define CHANNEL 0x76         //radio channel

#define LOCK_SLEEP 5000
#define UPDATE_PERIOD 1800000

#define MIN_RETRY_TIME 15000
#define MAX_RETRY_TIME 300000

#define SUCCESS_PIN 4
#define FAILED_PIN 3

// Hardware configuration: Set up nRF24L01 radio on SPI bus plus pins 9 (CE) & 10 (CSN)
RF24 radio(9, 10);

SoftwareSerial Monitor(RX, TX);

// set 2 default pipes 0xe7e7e7e7e7 0xc2c2c2c2c2
uint64_t pipes[] = {0xe7e7e7e7e7LL, 0xc2c2c2c2c2LL};
unsigned char device_id = 3; //device id

unsigned short int id_length = 1;        //device id length (2 bytes)
unsigned short int device_type = 1;      //message type id length (2 bytes)
unsigned short int msg_type_length = 1;  //message type id length (2 bytes)
unsigned short int tail_length = 1;      //tail id length .. is this the last message (2 bytes)
unsigned short int msg_body_length = 20; //main message body length (2 bytes)

int packet_size()
{
    return id_length + device_type + msg_type_length + tail_length + msg_body_length;
}

void setup()
{
    //serial configuration for debugging AVR chips
    Serial.begin(9600);
    printf_begin();
    Monitor.begin(9600);
    pinMode(SUCCESS_PIN, OUTPUT);
    pinMode(FAILED_PIN, OUTPUT);

    pinMode(TX, OUTPUT);
    pinMode(RX, INPUT);
    digitalWrite(SUCCESS_PIN, HIGH);
    digitalWrite(FAILED_PIN, LOW);
    //Monitor.print("Hello,\t\n");
    // Setup and configure rf radio
    radio.begin();       // Start up the radio
    radio.setAutoAck(1); // Ensure autoACK is enabled
    radio.setPayloadSize(id_length + device_type + msg_type_length + tail_length + msg_body_length);
    radio.enableAckPayload();
    radio.enableDynamicPayloads();

    // the channel and PA level must match transmitter
    radio.setChannel(CHANNEL);
    radio.setPALevel(RF24_PA_LOW);
    radio.setDataRate(RF24_250KBPS);

    //set channels to recieve
    radio.openWritingPipe(pipes[1]);

    radio.printDetails();   // Dump the configuration of the rf unit for debugging
    radio.startListening(); // Start listening
}

int readTHData(double &humidity, double &temperature)
{
    printf("Getting temp data.. \n\r");
    char error = 0;
    double temperature_samples[DHT11_SAMPLE_SIZE] = {0.0};
    double humidity_samples[DHT11_SAMPLE_SIZE] = {0.0};
    for (int sample = 0; sample < DHT11_SAMPLE_SIZE; sample)
    {
        int chk = DHT.read11(DHT11_PIN);
        switch (chk)
        {
        case DHTLIB_OK:
            temperature_samples[sample] = DHT.temperature;
            humidity_samples[sample] = DHT.humidity;
            sample++;
            break;
        case DHTLIB_ERROR_CHECKSUM:
            Serial.print("DHT11: Checksum error,\t\n");
            error++;
            break;
        case DHTLIB_ERROR_TIMEOUT:
            Serial.print("DHT11: Time out error,\t\n");
            error++;
            break;
        default:
            Serial.print("DHT11: Unknown error,\t\n");
            error++;
            break;
        }
        if (error > DHT11_MAX_ERRORS)
            sample = DHT11_SAMPLE_SIZE;
        delay(200);
    }

    if (error < DHT11_MAX_ERRORS)
    {
        double max_temp = temperature_samples[0];
        double min_temp = temperature_samples[0];
        double max_humid = humidity_samples[0];
        double min_humid = humidity_samples[0];
        humidity = max_humid;
        temperature = max_temp;
        for (char i = 1; i < DHT11_SAMPLE_SIZE; i++)
        {
            min_temp = temperature_samples[i] < min_temp ? temperature_samples[i] : min_temp;
            max_temp = temperature_samples[i] > max_temp ? temperature_samples[i] : max_temp;
            min_humid = humidity_samples[i] < min_humid ? temperature_samples[i] : min_humid;
            max_humid = humidity_samples[i] > max_humid ? temperature_samples[i] : max_humid;
            humidity += humidity_samples[i];
            temperature += temperature_samples[i];
        }
        humidity = (humidity - (max_humid + min_humid)) / (DHT11_SAMPLE_SIZE - 2);
        temperature = (temperature - (max_temp + min_temp)) / (DHT11_SAMPLE_SIZE - 2);
    }
    else
    {
        humidity = -1.0;
        temperature = -1.0;
    }
    Serial.print(humidity, 1);
    Serial.print(",\t");
    Serial.println(temperature, 1);
    //sort arrays
}

void build_message(char *msg, unsigned char device, unsigned char device_type, unsigned char type, char *body, unsigned char tail)
{
    msg[0] = device;
    msg[1] = device_type;
    msg[2] = type;
    msg[3] = tail;
    for (char i = 0; i < msg_body_length; i++)
    {
        msg[i + 4] = body[i];
    }
}

void build_message_handshake(char *msg, unsigned char device, unsigned char device_type)
{
    clear_msg(msg, id_length + device_type + msg_type_length + tail_length + msg_body_length);
    msg[0] = device;
    msg[1] = device_type;
}

void clear_msg(char *msg, int length)
{
    memset(msg, 0, length);
}

void transmit(unsigned char type, char *data)
{
    //init packet
    char msg[id_length + device_type + msg_type_length + tail_length + msg_body_length] = {0};
    char channel_available = 0;
    char done = 0;

    if (type == 1)
        printf("Beggining communication to transmit temperature data.\n\r");
    else if (type == 2)
        printf("Beggining communication to transmit humidity data.\n\r");

    //build handshake packet
    build_message_handshake(msg, device_id, device_type);

    long error_delay = MIN_RETRY_TIME;

    while (done == 0 && error_delay < MAX_RETRY_TIME)
    {
        if (channel_available == 1)
        {
            build_message(msg, device_id, device_type, type, data, 1);
            if (!radio.write(&msg, sizeof(msg)))
            {
                printf("Send failed. Sleeping for %ld seconds and retrying.\n\r", error_delay / 1000);
                digitalWrite(SUCCESS_PIN, LOW);
                digitalWrite(FAILED_PIN, HIGH);
                delay(error_delay);
                error_delay = error_delay * 2;
            }
            else
            {
                error_delay = MIN_RETRY_TIME;
                char result[id_length + device_type + msg_type_length + tail_length + msg_body_length] = {0};
                radio.read(&result, radio.getDynamicPayloadSize());
                printf("Base waiting for device: %x\n", result[0]);
                if (result[0] == device_id)
                {
                    printf("Message processed by home base, releasing channel.\n\r");
                    digitalWrite(SUCCESS_PIN, HIGH);
                    digitalWrite(FAILED_PIN, LOW);
                    done = 1;
                }
                else
                {
                    printf("Home base is locked by another device, sleeping for %ld seconds and retrying channel lock.\n\r", LOCK_SLEEP / 1000);
                    digitalWrite(SUCCESS_PIN, LOW);
                    digitalWrite(FAILED_PIN, HIGH);
                    channel_available = 0;
                    build_message_handshake(msg, device_id, device_type);
                    delay(LOCK_SLEEP);
                }
            }
        }
        else
        {
            if (!radio.write(&msg, sizeof(msg)))
            {
                printf("Send failed. Sleeping for %ld seconds and retrying.\n\r", error_delay / 1000);
                digitalWrite(SUCCESS_PIN, LOW);
                digitalWrite(FAILED_PIN, HIGH);
                delay(error_delay);
                error_delay = error_delay * 2;
            }
            else
            {
                error_delay = MIN_RETRY_TIME;
                char result[id_length + device_type + msg_type_length + tail_length + msg_body_length] = {0};
                radio.read(&result, radio.getDynamicPayloadSize());
                printf("Base waiting for device: %x\n", result[0]);
                if (result[0] == 0)
                {
                    printf("Home base free and ready to recieve.\n\r");
                    digitalWrite(SUCCESS_PIN, HIGH);
                    digitalWrite(FAILED_PIN, LOW);
                    channel_available = 1;
                }
                else if (result[0] == device_id)
                {
                    printf("Something is out of sync, home base is already waiting for this device.\n\r");
                }
                else
                {
                    printf("Home base is locked, sleeping for 5 seconds and trying again.\n\r");
                    digitalWrite(SUCCESS_PIN, LOW);
                    digitalWrite(FAILED_PIN, HIGH);
                    delay(LOCK_SLEEP);
                }
            }
        }
    }
    if (error_delay > MAX_RETRY_TIME)
    {
        digitalWrite(SUCCESS_PIN, LOW);
        digitalWrite(FAILED_PIN, HIGH);
        printf("Maximum error delay met, will try again on next data point.\n\r");
    }
}

void sciToFloat(char *data, int len)
{
    int power = data[len + 5];
    power = power - 48;
    data[len + 2] = 0;
    data[len + 3] = 0;
    data[len + 4] = 0;
    data[len + 5] = 0;
    for (int i = 1; i <= power; i++)
    {
        char tmp = "";
        tmp = data[i];
        data[i] = data[i + 1];
        data[i + 1] = tmp;
    }
}

void loop(void)
{
    unsigned long got_time = micros();
    radio.stopListening(); // First, stop listening so we can talk.
    printf("Now sending.. \n\r");
    double h = 0.0;
    double t = 0.0;
    readTHData(h, t);

    unsigned char message_type = 1;
    char data[msg_body_length] = "";
    dtostre(h, data, DATA_PERCISION, 0);
    sciToFloat(data, DATA_PERCISION);
    transmit(2, data);
    dtostre(t, data, DATA_PERCISION, 0);
    sciToFloat(data, DATA_PERCISION);
    transmit(1, data);
    delay(UPDATE_PERIOD);
}
