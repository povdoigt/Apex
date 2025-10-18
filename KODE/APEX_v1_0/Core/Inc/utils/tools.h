
#ifndef TOOLS_H
#define TOOLS_H

#include "stm32f4xx_hal.h"
#include "utils/scheduler.h"
#include "utils/gms.h"

#include <stdlib.h>
#include <string.h>
#include <math.h> 

// #define MAX_LENGTH_SPI_BUF 1024

typedef union FLOAT_U32_UNION {
    float    float_type;
    uint32_t uint32_type;
} FLOAT_U32_UNION;

typedef struct SPI_HandleTypeDef_flag {
    SPI_HandleTypeDef   *hspi;
    bool                 is_used;
} SPI_HandleTypeDef_flag;



void float_format(char* buff, float num, int precision, int width);


// Renvoie le nombre de champ d'une ligne type csv.
int count_nbr_elems(char buffer[], char sep);

// Modifie un [buffer] caracterisant la ligne d'un fichier de csv afin de decouper les differentes chaines
// de caracteres. Remplie un tableau de pointeur pointant vers ces differentes chaines de caracteres.
void set_elems_from_csv(char **elems, char buffer[], char sep, int nbr_elems);

// Remplie un [buffer] caracterisant la ligne d'un fichier de csv a partir d'un tableau de
// pointeur pointant vers les differentes chaines de caracteres.
void set_line_to_csv(char **elems, char buffer[], char sep, int nbr_elems);

// uint32_t GetMicros(void);

// #define ASYNC_Delay_NUMBER 10

// typedef struct ASYNC_Delay_CONTEXT {
//     uint32_t stop_time; // Temps d'arret de la tache
// } ASYNC_Delay_CONTEXT;

// extern TASK_POOL_CREATE(ASYNC_Delay);

// void ASYNC_Delay_ms_init(TASK *self, uint32_t delay);
// void ASYNC_Delay_us_init(TASK *self, uint32_t delay);
// TASK_RETURN ASYNC_Delay_ms(SCHEDULER *scheduler, TASK *self);
// TASK_RETURN ASYNC_Delay_us(SCHEDULER *scheduler, TASK *self);



#endif // TOOLS_H