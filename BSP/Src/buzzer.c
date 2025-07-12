#include "buzzer.h"
#include "tim.h"
#include "cmsis_os.h"

// Private variables
static BuzzerState_t buzzer_state = BUZZER_OFF;
static uint8_t buzzer_volume = 1; // Default 50% volume
static uint32_t timer_clock_freq = 200000000; // 200MHz timer clock
static uint32_t prescaler = 24; // TIM12 prescaler + 1

// Private function prototypes
static void Buzzer_SetFrequency(uint32_t frequency);
static void Buzzer_SetDutyCycle(uint8_t duty_percent);

/**
 * @brief Initialize the buzzer
 */
void Buzzer_Init(void) {
    // Start PWM on TIM12 Channel 2
    HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);
    
    // Initially turn off the buzzer
    Buzzer_Stop();
    
    buzzer_state = BUZZER_OFF;
}

/**
 * @brief Play a tone with specified frequency and duration
 * @param frequency Frequency in Hz (20-20000 typical range)
 * @param duration_ms Duration in milliseconds
 */
void Buzzer_PlayTone(uint32_t frequency, uint32_t duration_ms) {
    if (frequency == 0) {
        Buzzer_Stop();
        return;
    }
    
    Buzzer_PlayToneAsync(frequency);
    
    if (duration_ms > 0) {
        osDelay(duration_ms);
        Buzzer_Stop();
    }
}

/**
 * @brief Play a tone asynchronously (non-blocking)
 * @param frequency Frequency in Hz
 */
void Buzzer_PlayToneAsync(uint32_t frequency) {
    if (frequency == 0) {
        Buzzer_Stop();
        return;
    }
    
    Buzzer_SetFrequency(frequency);
    Buzzer_SetDutyCycle(buzzer_volume);
    buzzer_state = BUZZER_ON;
}

/**
 * @brief Stop the buzzer
 */
void Buzzer_Stop(void) {
    // Set duty cycle to 0 to stop sound
    TIM12->CCR2 = 0;
    buzzer_state = BUZZER_OFF;
}

/**
 * @brief Set buzzer volume (duty cycle)
 * @param volume_percent Volume from 0-100%
 */
void Buzzer_SetVolume(uint8_t volume_percent) {
    if (volume_percent > 100) {
        volume_percent = 100;
    }
    
    buzzer_volume = volume_percent;
    
    // If buzzer is currently on, update the duty cycle
    if (buzzer_state == BUZZER_ON) {
        Buzzer_SetDutyCycle(buzzer_volume);
    }
}

/**
 * @brief Get current buzzer state
 * @return Current buzzer state (ON/OFF)
 */
BuzzerState_t Buzzer_GetState(void) {
    return buzzer_state;
}

/**
 * @brief Play a simple beep
 */
void Buzzer_Beep(void) {
    Buzzer_PlayTone(NOTE_BEEP, 100); // 100ms beep
}

/**
 * @brief Play an alert sound
 */
void Buzzer_Alert(void) {
    Buzzer_PlayTone(NOTE_ALERT, 200); // 200ms alert
}

/**
 * @brief Play a melody
 * @param frequencies Array of frequencies
 * @param durations Array of durations in ms
 * @param length Number of notes
 */
void Buzzer_PlayMelody(const uint32_t* frequencies, const uint32_t* durations, uint8_t length) {
    for (uint8_t i = 0; i < length; i++) {
        Buzzer_PlayTone(frequencies[i], durations[i]);
        osDelay(50); // Small gap between notes
    }
}

/**
 * @brief Set the PWM frequency for the buzzer
 * @param frequency Desired frequency in Hz
 */
static void Buzzer_SetFrequency(uint32_t frequency) {
    if (frequency == 0) return;
    
    // Calculate ARR value for the given frequency
    // ARR = (Timer Clock / Prescaler) / frequency - 1
    uint32_t arr = (timer_clock_freq / prescaler) / frequency - 1;
    
    // Ensure ARR is within valid range
    if (arr < 1) arr = 1;
    if (arr > 65535) arr = 65535; // 16-bit timer
    
    TIM12->ARR = arr;
}

/**
 * @brief Set the PWM duty cycle (volume)
 * @param duty_percent Duty cycle percentage (0-100)
 */
static void Buzzer_SetDutyCycle(uint8_t duty_percent) {
    if (duty_percent > 100) duty_percent = 100;
    
    uint32_t arr = TIM12->ARR;
    uint32_t ccr = (arr * duty_percent) / 100;
    
    TIM12->CCR2 = ccr;
}

/**
 * @brief Play startup melody for 2 seconds
 * This function initializes the buzzer and plays a startup melody
 */
void Buzzer_PlayStartupMelody(void) {
    // Initialize buzzer system
    Buzzer_Init();
    Buzzer_SetVolume(30); // Set moderate volume (30%)
    
    // Define startup melody - plays for approximately 2 seconds
    const uint32_t melody_frequencies[] = {NOTE_C4, NOTE_E4, NOTE_G4, NOTE_C5, NOTE_G4, NOTE_E4, NOTE_C4};
    const uint32_t melody_durations[] = {250, 250, 250, 300, 250, 250, 400};
    
    // Play melody
    Buzzer_PlayMelody(melody_frequencies, melody_durations, 7);
    osDelay(500); // Small additional delay for safety
}
