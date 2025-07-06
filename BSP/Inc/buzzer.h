#ifndef __BUZZER_H
#define __BUZZER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Define frequencies for musical notes (in Hz)
#define NOTE_C4     262     // Middle C
#define NOTE_D4     294
#define NOTE_E4     330
#define NOTE_F4     349
#define NOTE_G4     392
#define NOTE_A4     440
#define NOTE_B4     494
#define NOTE_C5     523

// Additional useful frequencies
#define NOTE_BEEP   262    // Generic beep tone
#define NOTE_ALERT  2000    // Alert tone
#define NOTE_LOW    500     // Low tone
#define NOTE_HIGH   3000    // High tone

// Buzzer states
typedef enum {
    BUZZER_OFF = 0,
    BUZZER_ON = 1
} BuzzerState_t;

// Function prototypes
void Buzzer_Init(void);
void Buzzer_PlayTone(uint32_t frequency, uint32_t duration_ms);
void Buzzer_PlayToneAsync(uint32_t frequency);
void Buzzer_Stop(void);
void Buzzer_SetVolume(uint8_t volume_percent);
BuzzerState_t Buzzer_GetState(void);

// Convenience functions
void Buzzer_Beep(void);
void Buzzer_Alert(void);
void Buzzer_PlayMelody(const uint32_t* frequencies, const uint32_t* durations, uint8_t length);
void Buzzer_PlayStartupMelody(void);

#ifdef __cplusplus
}
#endif

#endif /* __BUZZER_H */ 
