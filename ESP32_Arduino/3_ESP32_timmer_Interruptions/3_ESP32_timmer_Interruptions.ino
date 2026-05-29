// ESP32 Arduino IDE
// Timer de 30 ms que conmuta un LED

const uint8_t LED_PIN = 2;     // LED integrado en modulos ESP32

hw_timer_t *timer = NULL;      // timer pointer declaration

volatile bool led_state = false; 


// ISR del timer
void IRAM_ATTR onTimer()
{
    led_state = !led_state;
    digitalWrite(LED_PIN, led_state);
}


void setup()
{
    pinMode(LED_PIN, OUTPUT);

    // Timer a 1 MHz
    // 1 tick = 1 microsegundo  timerBegin(frecuency) Frecuency of the timer clock, not of the timer interruption
    timer = timerBegin(1000000);

    // Attach timer struct with timmer interruption handle function
    timerAttachInterrupt(timer, &onTimer);

    //Configure alarm value and autoreload of the timer. Alarm is automatically enabled.
    // Alarm each 30000 us = 30 ms
    // autoreload = true
    // reload_count = 0 => infinito
    timerAlarm(timer, 30000, true, 0);
}


void loop()
{
}