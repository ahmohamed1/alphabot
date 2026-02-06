

#if defined(BOARD_PICO)
// Pins for Raspberry Pi Pico
#define L298N_enB 13
#define L298N_in4 11
#define L298N_in3 12

#define L298N_enA 10
#define L298N_in2 9
#define L298N_in1 8
#define right_encoder_phaseA 7
#define right_encoder_phaseB 6
#define left_encoder_phaseA 5
#define left_encoder_phaseB 4

#elif defined(BOARD_NANO)
// Pins for Arduino Nano (modify as per actual Nano connections)


// PINS for new Motor controller
#define L298N_in2 6   // Dir Motor A
#define L298N_in1 9   // Dir Motor A

// RIGHT motor
#define L298N_in3 10  // Dir Motor B
#define L298N_in4 11  // Dir Motor B



// Wheel Encoders Connection PIN 
#define right_encoder_phaseA 2  // Interrupt 
#define right_encoder_phaseB 4  
#define left_encoder_phaseA 3   // Interrupt
#define left_encoder_phaseB 5
#define voltage_pin A0

#elif defined(BOARD_ESP32)
// Pins for Arduino Nano (modify as per actual Nano connections)
// left
#define L298N_enB 25  // PWM
#define L298N_in4 26  // Dir Motor B
#define L298N_in3 27 // Dir Motor B

//right
#define L298N_enA 12  // PWM
#define L298N_in2 12   // Dir Motor A
#define L298N_in1 13   // Dir Motor A

// Wheel Encoders Connection PIN 
#define right_encoder_phaseA 34  // Interrupt 
#define right_encoder_phaseB 35
#define left_encoder_phaseA 32   // Interrupt
#define left_encoder_phaseB 33
#define voltage_pin A0


#else
#error "Please define BOARD_PICO or BOARD_NANO"
#endif

#define LOOP_FREQ 10
#define TICK_PER_REVOLUT 990.0 //330.0