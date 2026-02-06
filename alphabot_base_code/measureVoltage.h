class measureVoltage {
public:
  // Initialize with pin and optional parameters
  void init(int pin, float reference_voltage = 5.0, float r1 = 47000.0, float r2 = 20000.0) {
    if (pin >= A0 && pin <= A5) {  // Basic pin validation for most Arduino boards
      ANALOG_IN_PIN = pin;
      referance_voltage_ = reference_voltage;
      R1 = r1;
      R2 = r2;
      SCALE = (R1 + R2) / R2; // = 67k / 20k = 3.35
      initialized = true;
    }
  }

  // Set battery type (or custom min/max)
  void setBatteryType(String type) {
    if(type == "12V_LEAD_ACID") {
      min_voltage = 11.3;
      max_voltage = 12.7;
    }
    else if(type == "LIPO_1S") {
      min_voltage = 3.0;
      max_voltage = 4.2;
    }
    // Add other battery types...
  }

  void setVoltageRange(float minV, float maxV) {
    min_voltage = minV;
    max_voltage = maxV;
  }

  // Get battery percentage (0-100)
  int get_battery_percent() {
    float voltage = get_battery_level();
    voltage = constrain(voltage, min_voltage, max_voltage);
    return mapFloat(voltage, min_voltage, max_voltage, 0, 100);
  }

  // Get the measured voltage
  float get_battery_level() {
    if (!initialized) return 0.0;  // Or some error value
    
    pin_reading = analogRead(ANALOG_IN_PIN);
    adc_voltage = (pin_reading * referance_voltage_) / 1024.0;
    
    return adc_voltage * SCALE;
  }

  // Optional: get the raw ADC reading
  int get_raw_reading() {
    return initialized ? analogRead(ANALOG_IN_PIN) : 0;
  }

private:
  int ANALOG_IN_PIN = A0;
  float R1 = 47000.0; // top resistor (ohms)
  float R2 = 20000.0; // bottom resistor (ohms)
  float SCALE = (R1 + R2) / R2; // = 67k / 20k = 3.35

  int pin_reading = 0;
  float adc_voltage = 0.0;
  float referance_voltage_ = 5.0;
  bool initialized = false;
  
  float min_voltage = 0.0;
  float max_voltage = 5.0; // Defaults for safety

  // Helper function for floating-point mapping
  float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  }

};