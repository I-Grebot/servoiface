/* -----------------------------------------------------------------------------
 * pid
 * I-Grebot Dc motor PID library
 * -----------------------------------------------------------------------------
 * File        : pid.c
 * Language    : C
 * Author      : Sebastien Brulais
 * Creation    : 2013-06-07
 * -----------------------------------------------------------------------------
 * Library usage
 *
 *  /!\ Keep in mind that PID gains are sample time dependent
 *
 *  /!\ Do not forget to set a heap size in your linker for mallocs
 *  /!\ At least 4096 bytes /!\
 *
 * -----------------------------------------------------------------------------
 * Version     Comment                                   Author       Date
 * 1.0         Initial release                           Seb B.      2013-06-07
 * 1.1	       Separation speed/position update		     Pierrick B. 2013-12-11
 * 1.2         Adding PID Process + Testing              Pierrick B. 2014-01-04
 * -----------------------------------------------------------------------------
 */

#include "pid.h"
#include <stdlib.h>


int32_t PID_Process(PID_struct_t *PID, int32_t error)
{
    int32_t command;
    int32_t err_D;

    PID->last_err = PID->err;
    PID->err = error;
    PID->err_I += error;
    err_D = PID->last_err - PID->err;

    if(PID->KI!=0)
    {
        if(PID->err_I > PID->I_limit)
        {
            PID->err_I = PID->I_limit;
        }else if(PID->err_I < -PID->I_limit)
        {
            PID->err_I = -PID->I_limit;
        }
    }
    /*Main PID computation*/
    command = PID->err*PID->KP/100 + PID->err_I*PID->KI/1000 - err_D*PID->KD/100;

    return command;
 }

void PID_Process_Speed(PID_process_t *sPID, int32_t position){
    int32_t command=0;

    sPID->curr = (position - sPID->last);

    // Compute Speed errors
    command = PID_Process(sPID->PID, sPID->ref - sPID->curr);
   
    // Speed saturation
    if(sPID->speed_Limit)
    {
	if (command > sPID->speed_Limit)
	{
            command = sPID->speed_Limit;
        }else if (command < - sPID->speed_Limit)
	{
            command = - sPID->speed_Limit;
	}
    }

    // Acceleration saturation
    if(sPID->acceleration_Limit)
    {
        if ( (command - sPID->last_ref) > sPID->acceleration_Limit)
	{
            command = sPID->last_ref + sPID->acceleration_Limit;
	}else if (  (sPID->last_ref - command )  >  sPID->acceleration_Limit )
	{
            command = sPID->last_ref - sPID->acceleration_Limit;
	}
    }

    // Send new motor reference
    SetMotorSpeed(command);
    sPID->last_ref = command;
    sPID->last = position;
}

void PID_Process_Position(PID_process_t *pPID, PID_process_t *sPID, int32_t position){
    
    int32_t ref_speed;

    pPID->curr = position;
	
    // Compute position errors
    ref_speed = PID_Process(pPID->PID, pPID->ref - pPID->curr);
 
    // Speed saturation
    if(pPID->speed_Limit)
    {
	if (ref_speed > pPID->speed_Limit)
	{
            ref_speed = pPID->speed_Limit;
        }else if (ref_speed < - pPID->speed_Limit)
	{
            ref_speed = - pPID->speed_Limit;
	}
    }
	
    // Acceleration saturation
    if(pPID->acceleration_Limit)
    {
        if ( (ref_speed - pPID->last_ref) > pPID->acceleration_Limit)
	{
            ref_speed = pPID->last_ref + pPID->acceleration_Limit;
	}else if (  (pPID->last_ref - ref_speed )  >  pPID->acceleration_Limit )
	{
            ref_speed = pPID->last_ref - pPID->acceleration_Limit;
	}
    }

    
    if (sPID != NULL)
    {
        sPID->ref=ref_speed;
    	PID_Process_Speed(sPID,position);
    }
    else
    {
        // Send new motor reference
    	SetMotorSpeed(ref_speed);
        pPID->last_ref = ref_speed;
    }
}

void PID_Set_Coefficient(PID_struct_t *PID,int32_t KP,int32_t KI,int32_t KD,int32_t I_limit){
    // Set coefficients
    PID->KP = KP;
    PID->KI = KI;
    PID->KD = KD;
    PID->I_limit = I_limit;
}

void PID_Reset(PID_process_t *xPID){
    xPID->PID->I_limit = 0;
    xPID->PID->err = 0;
    xPID->PID->err_I = 0;
    xPID->PID->last_err = 0;
    xPID->last = 0;
    xPID->last_ref = 0;

    // Reset PID gains
    PID_Set_Coefficient(xPID->PID,0,0,0,0);
	
    PID_Set_limitation(xPID,0,0);
}

void PID_Set_limitation(PID_process_t *xPID,int32_t S_limit, int32_t A_limit){
    // Set Saturations
    xPID->speed_Limit = S_limit;
    xPID->acceleration_Limit = A_limit;
}

PID_process_t* pid_init(void){
    PID_process_t *xPID;
    xPID =(PID_process_t *) malloc(sizeof(PID_process_t));
    xPID->PID=(PID_struct_t *) malloc(sizeof(PID_struct_t));
    PID_Reset(xPID);
    return xPID;
}

void PID_Set_Cur_Position(PID_process_t *pPID, int32_t position) {
    pPID->curr = position;
}

void PID_Set_Ref_Position(PID_process_t *pPID, int32_t position) {
    pPID->ref = position;
}

int32_t PID_Get_Cur_Position(PID_process_t *pPID){
    return pPID->curr;
}

void PID_Set_Ref_Speed(PID_process_t *sPID, int16_t speed) {
    sPID->ref = speed;
}

int32_t PID_Get_Cur_Speed(PID_process_t *sPID){
    return sPID->curr;
}
