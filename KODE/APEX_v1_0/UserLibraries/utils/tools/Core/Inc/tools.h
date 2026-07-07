#ifndef TOOLS_H
#define TOOLS_H

#include "float3.h"
#include "stm32f4xx_hal.h"

#include <stdlib.h>
#include <string.h>
#include <math.h> 

typedef struct float_ts_t {
    uint32_t ts;
    float data;
} float_ts_t;

typedef struct float3_ts_t {
    uint32_t ts;
    float3_t data;
} float3_ts_t;


void float_format(char* buff, float num, int precision, int width);


// Renvoie le nombre de champ d'une ligne type csv.
int count_nbr_elems(char buffer[], char sep);

// Modifie un [buffer] caracterisant la ligne d'un fichier de csv afin de decouper les differentes chaines
// de caracteres. Remplie un tableau de pointeur pointant vers ces differentes chaines de caracteres.
void set_elems_from_csv(char **elems, char buffer[], char sep, int nbr_elems);

// Remplie un [buffer] caracterisant la ligne d'un fichier de csv a partir d'un tableau de
// pointeur pointant vers les differentes chaines de caracteres.
void set_line_to_csv(char **elems, char buffer[], char sep, int nbr_elems);

HAL_StatusTypeDef TIM_Delay_Micro(uint32_t delay);

// uint32_t GetMicros(void);


#endif // TOOLS_H