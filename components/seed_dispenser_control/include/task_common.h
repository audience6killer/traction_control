
#define SEED_DISPENSER_MCPWM_GROUP_ID   1

#define SEED_DISPENSER_CORE_ID          0
#define SEED_DISPENSER_STACK_SIZE       4096
#define SEED_DISPENSER_TASK_PRIORITY    3
#define SEED_DISPENSER_FREQ_HZ          10000

#define SEED_DISPENSER_ENCODER_A        10
#define SEED_DISPENSER_ENCODERA_B       10
#define SEED_DISPENSER_PWM_A            10
#define SEED_DISPENSER_PWM_B            11
#define SEED_DISPENSER_FREQ_HZ          10000
#define CUTTER_DISC_ENCODER_A           12
#define CUTTER_DISC_ENCODER_B           13
#define CUTTER_DISC_PWM_A               15
#define CUTTER_DISC_PWM_B               16
#define CUTTER_DISC_FREQ_HZ             10000

#define SEED_DISPENSER_KP               0.5
#define SEED_DISPENSER_KI               0.7
#define SEED_DISPENSER_KD               0.7

#define CUTTER_DISC_KP                  0.5
#define CUTTER_DISC_KI                  0.7
#define CUTTER_DISC_KD                  0.7

#define BDC_MCPWM_TIMER_RESOLUTION_HZ       10000000 // 10MHz, 1 tick = 0.1us
#define BDC_ENCODER_PCNT_HIGH_LIMIT         2000
#define BDC_ENCODER_PCNT_LOW_LIMIT          -2000
#define SEED_DISPENSER_PID_LOOP_PERIOD_MS   10


#define BDC_MCPWM_TIMER_RESOLUTION_HZ   10000000