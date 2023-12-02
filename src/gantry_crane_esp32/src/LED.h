class LED
{
private:
    int ledPin;
    bool isBlinking;
    unsigned long previousMillis;
    unsigned long blinkPeriod;
    bool periodChanged;
    unsigned long pulseDuration = 50;

public:
    // Constructor
    LED(int pin) : ledPin(pin), isBlinking(false), blinkPeriod(1000), previousMillis(0)
    {
    }

    void begin()
    {
        pinMode(ledPin, OUTPUT);
        blinkPeriod = 0;
    }

    // Method to turn the LED on
    void turnOn()
    {
        digitalWrite(ledPin, HIGH);
        isBlinking = false;
    }

    // Method to turn the LED off
    void turnOff()
    {
        digitalWrite(ledPin, LOW);
        isBlinking = false;
    }

    // Method to update the LED state (call in loop())
    void blink(unsigned long blinkPeriod)
    {
        if (this->blinkPeriod != blinkPeriod)
        {
            this->blinkPeriod = blinkPeriod;
            isBlinking = true;
        }
        if (isBlinking)
        {
            unsigned long currentMillis = millis();
            if (currentMillis - previousMillis >= this->blinkPeriod)
            {
                previousMillis = currentMillis;
                digitalWrite(ledPin, !digitalRead(ledPin)); // Toggle the LED state
            }
        }
    }

    void pulse(unsigned long pulsePeriod = 1000)
    {
        if (this->blinkPeriod != pulsePeriod)
        {
            this->blinkPeriod = pulsePeriod;
            isBlinking = true;
        }
        if (isBlinking)
        {
            unsigned long currentMillis = millis();
            if (currentMillis - previousMillis >= this->blinkPeriod)
            {
                previousMillis = currentMillis;
                digitalWrite(ledPin, HIGH); // Toggle the LED state
            }
            if (millis() - previousMillis >= pulseDuration)
            {
                digitalWrite(ledPin, LOW);
            }
        }
    }
};
