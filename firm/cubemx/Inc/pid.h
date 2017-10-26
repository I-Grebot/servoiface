#ifndef _PID_H
#define _PID_H

/*
 * PID Library dependencies
 * ------------------------
 */

#include <stdint.h>
#include "main.h"
// Include the board-specific hardware configuration,
// such as clock frequencies definition, modules used etc.


/*
 * Structures definition
 * ---------------------
 */

typedef struct PID_struct_t{
    int32_t KP;     	// Proportional gain
    int32_t KI;       	// Integrative gain
    int32_t KD;       	// Derivative gain
    int32_t err;
    int32_t last_err;
    int32_t err_I;
    int32_t I_limit;
}PID_struct_t;

typedef struct PID_process_t{
    int32_t curr;
    int32_t last;
    int32_t ref;
    int32_t last_ref;
    // PID structure
    PID_struct_t *PID;
    int32_t speed_Limit;     	// Speed saturation, 0 => no limit
    int32_t acceleration_Limit; // Acceleration saturation, 0 => no limit
}PID_process_t;


/*
 * PID Functions Prototypes
 */

int32_t PID_Process(PID_struct_t *PID, int32_t error);
void PID_Process_Speed(PID_process_t *sPID, int32_t position);
void PID_Process_Position(PID_process_t *pPID, PID_process_t *sPID, int32_t position);
void PID_Process_Polar(PID_process_t *pDIST, PID_process_t *pROT, int32_t positionR, int32_t positionL);
void PID_Set_Coefficient(PID_struct_t *PID,int32_t KP,int32_t KI,int32_t KD,int32_t I_limit);
void PID_Reset(PID_process_t *xPID);
void PID_Set_limitation(PID_process_t *xPID,int32_t S_limit, int32_t A_limit);
PID_process_t* pid_init(void);
void PID_Set_Cur_Position(PID_process_t *pPID, int32_t position);
void PID_Set_Ref_Position(PID_process_t *pPID, int32_t position);
int32_t PID_Get_Cur_Position(PID_process_t *pPID);
void PID_Set_Ref_Speed(PID_process_t *sPID, int16_t speed);
int32_t PID_Get_Cur_Speed(PID_process_t *sPID);

#endif /* ! _PID_H */
