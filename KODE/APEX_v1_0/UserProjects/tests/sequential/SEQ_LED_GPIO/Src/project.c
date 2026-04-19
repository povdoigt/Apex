#include "project.h"


void setup(void) {

}


void loop(void) {
    HAL_GPIO_TogglePin(LED0R_GPIO_Port, LED0R_Pin);
    HAL_Delay(500);
    HAL_GPIO_TogglePin(LED0R_GPIO_Port, LED0R_Pin);
    HAL_GPIO_TogglePin(LED0G_GPIO_Port, LED0G_Pin);
    HAL_Delay(500);
    HAL_GPIO_TogglePin(LED0G_GPIO_Port, LED0G_Pin);
    HAL_GPIO_TogglePin(LED0B_GPIO_Port, LED0B_Pin);
    HAL_Delay(500);
    HAL_GPIO_TogglePin(LED0B_GPIO_Port, LED0B_Pin);
}