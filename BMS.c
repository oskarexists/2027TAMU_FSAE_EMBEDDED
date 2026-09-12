#include "HAL.h"

//define constants for readability and scalability
#define MAX_VOLTAGE 4.2
#define MIN_VOLTAGE 2.5
#define MAX_TEMP 60
#define DELTA_MAX 0.2
#define I_MAX 200000 //milliamps

#define OVER_VOLT_FAULT_ID 0x0B0
#define UNDER_VOLT_FAULT_ID 0x0B1
#define OVER_TEMP_FAULT_ID 0x0B2
#define DELTA_FAULT_ID 0x0B3
#define OVER_CURRENT_FAULT_ID 0x0B4
#define CLEAR_FAULT_ID 0x1CF
#define ISENSE_ID 0x511
#define MAX_FAULT_AMOUNT 5 //for the 5 possible faults

static float VOLTS[N_CELLS];
static float TEMPS[N_CELLS];
static volatile uint32_t CURRENT = 0;

static bool MASTER_FAULTS[MAX_FAULT_AMOUNT]; 
static uint16_t CURR_ID;
static uint8_t DATA[CAN_LEN];
static uint8_t THEFAULT[CAN_LEN];

static volatile bool TO_CLEAR = false;


/*Custom CAN bus for sending faults:

MASTER_FAULTS will internally keep the status of latched faults.
Byte 0 will hold 1 if there's an over-voltage fault active, 0 otherwise
Byte 1 will hold 1 if there's an under-voltage fault active, 0 otherwise
2, an over-temperature
3, a delta fault
4, an over-current

INDIVIDUAL FAULT CAN IDS AND BYTE FORMAT (to be sent):
    0x0B0: over-voltage         | 0th byte is which cell faulted, [1st,4th] is recorded voltage in milli-volts (0.001V)
    0x0B1: under-voltage        | 0th byte is which cell faulted, [1st,4th] is recorded voltage in milli-volts (0.001V)
    0x0B2: over-temperature     | 0th byte is which cell faulted, [1st,4th] is recorded temperature in milli-degrees celsius (0.001C)
    0x0B3: delta fault          | 0th byte is low-volt cell, 1st is hi-volt cell
                                | [2nd, 4th] is recorded low-volt in milli-volts (0.001V)
                                | [5th, 7th] is recorded hi-volt in milli-volts (0.001V)
    0x0B4: over-current         | [2nd, 5th] is recorded current in milli-amps (0.001A)
    *Unsigned big-endian for all
*/


void throw_fault(uint16_t ID, const uint8_t FAULT[CAN_LEN]){
    HAL_SetSDC(false);
    HAL_SetLED(255, 0, 0);
    HAL_SendCanMsg(ID,FAULT);
}


void Init()
{
   // This function runs once on startup
   /*considerations:

   LED: Red = open circuit, Green = closed
   */
    HAL_SetLED(255,0,0);
}


void Iter()
{
   // This function runs periodically at ~20Hz

   /*
    ASSUMPTION: car only charges while off AND regen braking won't make net current negative
    If there IS a negative current, assume the car is charging, and don't call it a fault

    ASSUMPTION: should immediately throw a fault and open SDC when one is detected
    to save previous milliseconds, don't collect all data to throw all existing faults, just open SDC when one is detected

    ACKNOWLEDGEMENT: To store data in the fault CAN bus I am effectively casting to uint24s, which leaves potential for overflow since
    volts and temps are initially stored in floats (64 bits). Since casting is done after comparisons anyway, the fault will be thrown 
    regardless of overflow, and if your battery voltage or temperature is really like 2^64 ur cooked anyway. The same applies for any
    multiplications used to convert units into their respective milli-forms.
   */
  
    //1: check if faults are to be cleared
    if(TO_CLEAR){
        for(int i = 0; i<MAX_FAULT_AMOUNT; i++){
            MASTER_FAULTS[i] = 0;
        }
        HAL_SetSDC(true);
        HAL_SetLED(0,255,0);
        TO_CLEAR = false;
    }


    //2: check for any active faults
    else{
        for(int i = 0; i<MAX_FAULT_AMOUNT; i++){
            if(MASTER_FAULTS[i] == 1){
                return; //drop cycle if fault is latched
            }
        }
    }

    //3: Checks, start w/ current
    uint32_t CURR_SNAP = CURRENT; //locks current in place
    if(CURR_SNAP > I_MAX){
            MASTER_FAULTS[4] = 1;
            THEFAULT[2] = (uint8_t)(CURR_SNAP >> 24);
            THEFAULT[3] = (uint8_t)(CURR_SNAP >> 16);
            THEFAULT[4] = (uint8_t)(CURR_SNAP >> 8);
            THEFAULT[5] = (uint8_t)(CURR_SNAP);
            throw_fault(OVER_CURRENT_FAULT_ID, THEFAULT);
    }



    //voltage check
    HAL_ReadVoltages(VOLTS);
    uint8_t MAXMIN_POS[2]; //For delta faults, 0 holds min pos, 1 max pos
    float CURR_VOLT_MAX = 0;
    float CURR_VOLT_MIN = 200; //shouldn't ever be this high anyway
    for(int i = 0; i<N_CELLS; i++){
        float CURR_VOLT = VOLTS[i];
        if(CURR_VOLT > MAX_VOLTAGE){
            //throw over voltage
            MASTER_FAULTS[0] = 1;
            THEFAULT[0] = i;
            uint32_t CURR_VOLT_32 = (uint32_t)(CURR_VOLT * 1000); //bring millivolts to ones place
            THEFAULT[1] = (uint8_t)(CURR_VOLT_32 >> 24);
            THEFAULT[2] = (uint8_t)(CURR_VOLT_32 >> 16);
            THEFAULT[3] = (uint8_t)(CURR_VOLT_32 >> 8);
            THEFAULT[4] = (uint8_t)(CURR_VOLT_32);
            throw_fault(OVER_VOLT_FAULT_ID, THEFAULT);
            return;
        }
        if(CURR_VOLT < MIN_VOLTAGE){
            //throw under voltage
            MASTER_FAULTS[1] = 1;
            THEFAULT[0] = i;
            uint32_t CURR_VOLT_32 = (uint32_t)(CURR_VOLT * 1000); //bring millivolts to ones place
            THEFAULT[1] = (uint8_t)(CURR_VOLT_32 >> 24);
            THEFAULT[2] = (uint8_t)(CURR_VOLT_32 >> 16);
            THEFAULT[3] = (uint8_t)(CURR_VOLT_32 >> 8);
            THEFAULT[4] = (uint8_t)(CURR_VOLT_32);
            throw_fault(UNDER_VOLT_FAULT_ID, THEFAULT);
            return;
        }
        if(CURR_VOLT > CURR_VOLT_MAX){CURR_VOLT_MAX = CURR_VOLT; MAXMIN_POS[1] = i;}
        if(CURR_VOLT < CURR_VOLT_MIN){CURR_VOLT_MIN = CURR_VOLT; MAXMIN_POS[0] = i;}
    }
    if(CURR_VOLT_MAX - CURR_VOLT_MIN > DELTA_MAX){
            //throw delta exceeded
            MASTER_FAULTS[3] = 1;
            THEFAULT[0] = MAXMIN_POS[0];
            THEFAULT[1] = MAXMIN_POS[1];

            uint32_t VOLTMAX_32 = (uint32_t)(CURR_VOLT_MAX*1000);
            uint32_t VOLTMIN_32 = (uint32_t)(CURR_VOLT_MIN*1000);
            THEFAULT[2] = (uint8_t)(VOLTMIN_32>>16);
            THEFAULT[3] = (uint8_t)(VOLTMIN_32>>8);
            THEFAULT[4] = (uint8_t)(VOLTMIN_32);
            THEFAULT[5] = (uint8_t)(VOLTMAX_32>>16);
            THEFAULT[6] = (uint8_t)(VOLTMAX_32>>8);
            THEFAULT[7] = (uint8_t)(VOLTMAX_32);
            throw_fault(DELTA_FAULT_ID, THEFAULT);
            return;
        }

    //temp check
    HAL_ReadTemperatures(TEMPS);
    for(int i = 0; i<N_CELLS; i++){
        if(TEMPS[i] > MAX_TEMP){
            //throw over temp
            MASTER_FAULTS[2] = 1;
            THEFAULT[0] = i;
            
            uint32_t TEMP32 = (uint32_t)(TEMPS[i]*1000);
            THEFAULT[1] = (uint8_t)(TEMP32>>24);
            THEFAULT[2] = (uint8_t)(TEMP32>>16);
            THEFAULT[3] = (uint8_t)(TEMP32>>8);
            THEFAULT[4] = (uint8_t)(TEMP32);
            throw_fault(OVER_TEMP_FAULT_ID, THEFAULT);
            return;
        }
    }
    
}


void RxCan()
{
   // Called every time a CAN frame is received on the bus, using an interrupt.
   // Keep in mind, this can be called at any point in the execution of your program.
   // You may not use any HAL_* functions here except HAL_RecvCanMsg,
   // which is how you can pull the message from the bus.

    HAL_RecvCanMsg(&CURR_ID, DATA);
    if(CURR_ID == CLEAR_FAULT_ID){
        TO_CLEAR = true;
        return;
    }else if(CURR_ID == ISENSE_ID){
        if(!((DATA[2] >> 7) & 1)){ //check if MSB is 0, i.e. positive current
            CURRENT = ((uint32_t)DATA[2] << 24) |
                      ((uint32_t)DATA[3] << 16) |
                      ((uint32_t)DATA[4] << 8)  |
                      ((uint32_t)DATA[5]);
            
        }
    } 
}

