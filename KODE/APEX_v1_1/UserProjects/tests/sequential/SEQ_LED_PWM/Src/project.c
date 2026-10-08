#include "project.h"
#include "led.h"

#include "float3.h"
#include "waveform.h"


// led_rgb_t led0;
waveform_space_t waveform0;

float3_t waveform_test_fct(float t, const void *ctx) {
    float red = waveform_sine(t, &(waveform_sine_t){
        .start = 0.0f,
        .end = 0.33f
    });
    float green = waveform_sine(t, &(waveform_sine_t){
        .start = 0.33f,
        .end = 0.66f
    });
    float blue = waveform_sine(t, &(waveform_sine_t){
        .start = 0.66f,
        .end = 1.0f
    });
    return (float3_t){.x = red, .y = green, .z = blue};
}


void setup(void) {
    // Initialize LED driver

    LED_RGB_SetColor(&led0, (float3_t){.x = 0.0f, .y = 0.0f, .z = 0.0f}); // Set LED to red

    // Initialize waveform
    Waveform_Init_Space(&waveform0, waveform_test_fct, NULL, 2000, true); // 2 second period, repeating
}


void loop(void) {
    LED_RGB_SetColor(&led0, Waveform_Play_Space(&waveform0, HAL_GetTick()));
}