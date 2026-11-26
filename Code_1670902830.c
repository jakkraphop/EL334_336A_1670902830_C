#include <MK64F12.h>
#include <stdint.h>

/* ============================================================================
 * 1. HARDWARE PIN DEFINITIONS & CONSTANTS
 * ============================================================================ */
#define BIT_MASK(pin)       (1UL << (pin))

/* Multi-Function Shield Pins (Arduino Header Mapping) */
#define PIN_7SEG_LATCH      23  /* PTB23 (D4)  - Shift Register Latch */
#define PIN_7SEG_CLK        3   /* PTC3  (D7)  - Shift Register Clock */
#define PIN_7SEG_DATA       12  /* PTC12 (D8)  - Shift Register Serial Data */
#define PIN_SHIELD_LED      0   /* PTD0  (D10) - Status LED D4 (Active-Low) */
#define PIN_BUZZER          1   /* PTA1  (D3)  - Passive Buzzer */

/* Buttons on Multi-Function Shield (Active-Low with Internal Pull-up) */
#define PIN_BTN_S1          3   /* PTB3  (A1)  - Start/Stop | Unit | Preset */
#define PIN_BTN_S2          10  /* PTB10 (A2)  - Reset | Start/Cancel Alarm */
#define PIN_BTN_S3          11  /* PTB11 (A3)  - Switch Mode */

/* Onboard RGB-LED Pins on FRDM-K64F (Active-Low) */
#define PIN_RGB_RED         22  /* PTB22 */
#define PIN_RGB_BLUE        21  /* PTB21 */
#define PIN_RGB_GREEN       26  /* PTE26 */

/* Timing & Debounce Constants (Based on 60 MHz Bus Clock) */
#define PIT0_LOAD_250US     14999UL     /* 0.25 ms -> 4 kHz toggle = 2 kHz tone */
#define PIT1_LOAD_1SEC      59999999UL  /* 1.00 second exact interval */
#define DEBOUNCE_TICKS      800UL       /* 800 * 0.25 ms = 200 ms debounce */
#define MAX_STOPWATCH_SEC   5999        /* 99 minutes 59 seconds */

/* Temperature Thresholds (Celsius) */
#define TEMP_COLD_LIMIT     29.0f
#define TEMP_WARM_LIMIT     37.0f

/* ============================================================================
 * 2. DATA TYPES & LOOKUP TABLES
 * ============================================================================ */
typedef enum {
    MODE_STOPWATCH   = 0,
    MODE_THERMOMETER = 1,
    MODE_ALARM_TIMER = 2,
    MODE_COUNT       = 3
} SystemMode_t;

typedef enum {
    UNIT_CELSIUS    = 0,
    UNIT_FAHRENHEIT = 1
} TempUnit_t;

typedef enum {
    DISP_TIME_MMSS = 0,     /* Format: MM.SS */
    DISP_TEMP_2DEC = 1,     /* Format: XX.XX (< 100.0) */
    DISP_TEMP_1DEC = 2      /* Format: XXX.X (>= 100.0) */
} DisplayFormat_t;

typedef struct {
    volatile SystemMode_t mode;
    volatile uint32_t     ticks_250us;

    /* Stopwatch State */
    volatile int          sw_running;
    volatile int          sw_seconds;

    /* Thermometer State */
    volatile float        temp_celsius;
    volatile TempUnit_t   temp_unit;

    /* Alarm Timer State */
    volatile int          alarm_running;
    volatile int          alarm_elapsed;
    volatile int          alarm_preset_idx;
    volatile int          alarm_triggered;
    volatile int          alarm_beep_on;
} SystemState_t;

static const int ALARM_PRESETS[] = {5, 10, 30, 60, 300, 600};
#define NUM_ALARM_PRESETS 6

/* 7-Segment Active-Low Encoding (0-9) and Digit Selectors (Left to Right) */
static const uint8_t SEG_FONT[10]  = {0xC0, 0xF9, 0xA4, 0xB0, 0x99, 0x92, 0x82, 0xF8, 0x80, 0x90};
static const uint8_t DIGIT_POS[4]  = {0xF1, 0xF2, 0xF4, 0xF8};

/* Global System State Instance */
static SystemState_t g_sys = {
    MODE_STOPWATCH, 0,
    0, 0,
    0.0f, UNIT_CELSIUS,
    0, 0, 0, 0, 0
};

/* ============================================================================
 * 3. HARDWARE ABSTRACTION LAYER (HAL) - LOW-LEVEL DRIVERS
 * ============================================================================ */
static void HAL_Buzzer_Isolate(void) {
    /* Set PTA1 to High-Z (Disabled) to prevent crosstalk noise in other modes */
    PORTA->PCR[PIN_BUZZER] = PORT_PCR_MUX(0);
    PTA->PDDR &= ~BIT_MASK(PIN_BUZZER);
}

static void HAL_Buzzer_Enable(void) {
    /* Connect PTA1 as GPIO Output for clean 2 kHz square wave */
    PORTA->PCR[PIN_BUZZER] = PORT_PCR_MUX(1);
    PTA->PDDR |= BIT_MASK(PIN_BUZZER);
}

static void HAL_Buzzer_Toggle(void) {
    PTA->PTOR = BIT_MASK(PIN_BUZZER);
}

static void HAL_ShieldLED_Set(int turn_on) {
    if (turn_on) PTD->PCOR = BIT_MASK(PIN_SHIELD_LED); /* Active-Low: 0 = ON */
    else         PTD->PSOR = BIT_MASK(PIN_SHIELD_LED); /* Active-Low: 1 = OFF */
}

static void HAL_ShieldLED_Toggle(void) {
    PTD->PTOR = BIT_MASK(PIN_SHIELD_LED);
}

static void HAL_RGB_Off(void) {
    PTB->PSOR = BIT_MASK(PIN_RGB_RED) | BIT_MASK(PIN_RGB_BLUE);
    PTE->PSOR = BIT_MASK(PIN_RGB_GREEN);
}

static void HAL_RGB_SetByTemp(float temp_c) {
    HAL_RGB_Off();
    if (temp_c < TEMP_COLD_LIMIT) {
        PTB->PCOR = BIT_MASK(PIN_RGB_BLUE);  /* Cold: Blue */
    } else if (temp_c <= TEMP_WARM_LIMIT) {
        PTE->PCOR = BIT_MASK(PIN_RGB_GREEN); /* Normal: Green */
    } else {
        PTB->PCOR = BIT_MASK(PIN_RGB_RED);   /* Hot: Red */
    }
}

static void HAL_ShiftRegister_Write(uint8_t byte_data) {
    int bit;
    for (bit = 7; bit >= 0; bit--) {
        if (byte_data & (1U << bit)) PTC->PSOR = BIT_MASK(PIN_7SEG_DATA);
        else                         PTC->PCOR = BIT_MASK(PIN_7SEG_DATA);

        PTC->PSOR = BIT_MASK(PIN_7SEG_CLK);
        PTC->PCOR = BIT_MASK(PIN_7SEG_CLK);
    }
}

static void HAL_Display_RefreshDigit(int value, DisplayFormat_t format) {
    static int current_digit = 0;
    int digits[4];
    uint8_t pattern;

    if (format == DISP_TIME_MMSS) {
        int mins = value / 60;
        int secs = value % 60;
        digits[0] = (mins / 10) % 10;
        digits[1] = mins % 10;
        digits[2] = (secs / 10) % 10;
        digits[3] = secs % 10;
    } else {
        digits[0] = (value / 1000) % 10;
        digits[1] = (value / 100) % 10;
        digits[2] = (value / 10) % 10;
        digits[3] = value % 10;
    }

    pattern = SEG_FONT[digits[current_digit]];

    /* Add Decimal Point (Bit 7 = 0 for Active-Low) */
    if ((format == DISP_TIME_MMSS || format == DISP_TEMP_2DEC) && current_digit == 1) {
        pattern &= ~0x80;
    } else if (format == DISP_TEMP_1DEC && current_digit == 2) {
        pattern &= ~0x80;
    }

    PTB->PCOR = BIT_MASK(PIN_7SEG_LATCH);
    HAL_ShiftRegister_Write(pattern);
    HAL_ShiftRegister_Write(DIGIT_POS[current_digit]);
    PTB->PSOR = BIT_MASK(PIN_7SEG_LATCH);

    current_digit = (current_digit + 1) % 4;
}

static float HAL_ADC_ReadTempCelsius(void) {
    uint32_t raw_adc;
    float voltage_mv;

    ADC0->SC1[0] = ADC_SC1_ADCH(26); /* Internal Temperature Sensor Channel */
    while ((ADC0->SC1[0] & ADC_SC1_COCO_MASK) == 0);

    raw_adc    = ADC0->R[0];
    voltage_mv = ((float)raw_adc * 3300.0f) / 4095.0f;
    return 25.0f - ((voltage_mv - 716.0f) / 1.628f);
}

/* ============================================================================
 * 4. SYSTEM INITIALIZATION
 * ============================================================================ */
static void System_Init_GPIO(void) {
    SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK | SIM_SCGC5_PORTB_MASK |
                  SIM_SCGC5_PORTC_MASK | SIM_SCGC5_PORTD_MASK | SIM_SCGC5_PORTE_MASK;

    /* Buttons S1, S2, S3: Falling-edge Interrupt + Internal Pull-up */
    PORTB->PCR[PIN_BTN_S1] = PORT_PCR_MUX(1) | PORT_PCR_IRQC(0x0A) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
    PORTB->PCR[PIN_BTN_S2] = PORT_PCR_MUX(1) | PORT_PCR_IRQC(0x0A) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
    PORTB->PCR[PIN_BTN_S3] = PORT_PCR_MUX(1) | PORT_PCR_IRQC(0x0A) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
    PTB->PDDR &= ~(BIT_MASK(PIN_BTN_S1) | BIT_MASK(PIN_BTN_S2) | BIT_MASK(PIN_BTN_S3));

    /* 7-Segment Shift Register Pins */
    PORTB->PCR[PIN_7SEG_LATCH] = PORT_PCR_MUX(1);
    PORTC->PCR[PIN_7SEG_CLK]   = PORT_PCR_MUX(1);
    PORTC->PCR[PIN_7SEG_DATA]  = PORT_PCR_MUX(1);
    PTB->PDDR |= BIT_MASK(PIN_7SEG_LATCH);
    PTC->PDDR |= BIT_MASK(PIN_7SEG_CLK) | BIT_MASK(PIN_7SEG_DATA);

    /* Shield Status LED D4 */
    PORTD->PCR[PIN_SHIELD_LED] = PORT_PCR_MUX(1);
    PTD->PDDR |= BIT_MASK(PIN_SHIELD_LED);
    HAL_ShieldLED_Set(0);

    /* Buzzer (Isolated initially) */
    HAL_Buzzer_Isolate();

    /* Onboard RGB-LED */
    PORTB->PCR[PIN_RGB_RED]   = PORT_PCR_MUX(1);
    PORTB->PCR[PIN_RGB_BLUE]  = PORT_PCR_MUX(1);
    PORTE->PCR[PIN_RGB_GREEN] = PORT_PCR_MUX(1);
    PTB->PDDR |= BIT_MASK(PIN_RGB_RED) | BIT_MASK(PIN_RGB_BLUE);
    PTE->PDDR |= BIT_MASK(PIN_RGB_GREEN);
    HAL_RGB_Off();

    NVIC_EnableIRQ(PORTB_IRQn);
}

static void System_Init_ADC(void) {
    SIM->SCGC6 |= SIM_SCGC6_ADC0_MASK;
    ADC0->CFG1 = ADC_CFG1_MODE(1) | ADC_CFG1_ADIV(3); /* 12-bit resolution */
}

static void System_Init_Timers(void) {
    SIM->SCGC6 |= SIM_SCGC6_PIT_MASK;
    PIT->MCR = 0x00;

    /* PIT0: 0.25 ms Fast Tick (Display Multiplexing & 2 kHz Buzzer) */
    PIT->CHANNEL[0].LDVAL = PIT0_LOAD_250US;
    PIT->CHANNEL[0].TCTRL = PIT_TCTRL_TIE_MASK | PIT_TCTRL_TEN_MASK;

    /* PIT1: 1.00 s Slow Tick (Timekeeping & Beep Cadence) */
    PIT->CHANNEL[1].LDVAL = PIT1_LOAD_1SEC;
    PIT->CHANNEL[1].TCTRL = PIT_TCTRL_TIE_MASK | PIT_TCTRL_TEN_MASK;

    NVIC_EnableIRQ(PIT0_IRQn);
    NVIC_EnableIRQ(PIT1_IRQn);
}

/* ============================================================================
 * 5. APPLICATION LOGIC HANDLERS (CLEAN FLOWCHART SUB-PROCESSES)
 * ============================================================================ */
static void App_ResetAlarm(void) {
    g_sys.alarm_running   = 0;
    g_sys.alarm_elapsed   = 0;
    g_sys.alarm_triggered = 0;
    g_sys.alarm_beep_on   = 0;
    HAL_Buzzer_Isolate();
    HAL_ShieldLED_Set(0);
}

static void App_OnButtonS1_Pressed(void) {
    switch (g_sys.mode) {
        case MODE_STOPWATCH:
            g_sys.sw_running = !g_sys.sw_running;
            break;

        case MODE_THERMOMETER:
            g_sys.temp_unit = (g_sys.temp_unit == UNIT_CELSIUS) ? UNIT_FAHRENHEIT : UNIT_CELSIUS;
            break;

        case MODE_ALARM_TIMER:
            if (!g_sys.alarm_running && !g_sys.alarm_triggered && g_sys.alarm_elapsed == 0) {
                g_sys.alarm_preset_idx = (g_sys.alarm_preset_idx + 1) % NUM_ALARM_PRESETS;
            }
            break;

        default:
            break;
    }
}

static void App_OnButtonS2_Pressed(void) {
    if (g_sys.mode == MODE_STOPWATCH) {
        g_sys.sw_running = 0;
        g_sys.sw_seconds = 0;
        HAL_ShieldLED_Set(0);
    } else if (g_sys.mode == MODE_ALARM_TIMER) {
        if (!g_sys.alarm_running && !g_sys.alarm_triggered && g_sys.alarm_elapsed == 0) {
            g_sys.alarm_running = 1;
        } else {
            App_ResetAlarm();
        }
    }
}

static void App_OnButtonS3_Pressed(void) {
    g_sys.mode = (SystemMode_t)((g_sys.mode + 1) % MODE_COUNT);
    App_ResetAlarm();

    if (g_sys.mode == MODE_THERMOMETER) {
        HAL_RGB_SetByTemp(g_sys.temp_celsius);
    } else {
        HAL_RGB_Off();
    }
}

static void App_UpdateDisplay(void) {
    if (g_sys.mode == MODE_STOPWATCH) {
        HAL_Display_RefreshDigit(g_sys.sw_seconds, DISP_TIME_MMSS);
    } 
    else if (g_sys.mode == MODE_THERMOMETER) {
        float disp_temp = (g_sys.temp_unit == UNIT_FAHRENHEIT)
                          ? ((g_sys.temp_celsius * 1.8f) + 32.0f)
                          : g_sys.temp_celsius;

        if (disp_temp < 100.0f) {
            HAL_Display_RefreshDigit((int)(disp_temp * 100.0f), DISP_TEMP_2DEC);
        } else {
            HAL_Display_RefreshDigit((int)(disp_temp * 10.0f), DISP_TEMP_1DEC);
        }
    } 
    else {
        int show_sec = (!g_sys.alarm_running && !g_sys.alarm_triggered && g_sys.alarm_elapsed == 0)
                       ? ALARM_PRESETS[g_sys.alarm_preset_idx]
                       : g_sys.alarm_elapsed;
        HAL_Display_RefreshDigit(show_sec, DISP_TIME_MMSS);
    }
}

static void App_OnOneSecondTick(void) {
    if (g_sys.mode == MODE_STOPWATCH && g_sys.sw_running) {
        g_sys.sw_seconds = (g_sys.sw_seconds + 1 > MAX_STOPWATCH_SEC) ? 0 : (g_sys.sw_seconds + 1);
        HAL_ShieldLED_Toggle();
    } 
    else if (g_sys.mode == MODE_ALARM_TIMER) {
        if (g_sys.alarm_triggered) {
            g_sys.alarm_beep_on = !g_sys.alarm_beep_on;
            if (g_sys.alarm_beep_on) HAL_Buzzer_Enable();
            else                     HAL_Buzzer_Isolate();
            HAL_ShieldLED_Toggle();
        } 
        else if (g_sys.alarm_running) {
            g_sys.alarm_elapsed++;
            if (g_sys.alarm_elapsed >= ALARM_PRESETS[g_sys.alarm_preset_idx]) {
                g_sys.alarm_running   = 0;
                g_sys.alarm_triggered = 1;
                g_sys.alarm_beep_on   = 1;
                HAL_Buzzer_Enable();
                HAL_ShieldLED_Set(1);
            } else {
                HAL_ShieldLED_Toggle();
            }
        }
    }
}

/* ============================================================================
 * 6. INTERRUPT SERVICE ROUTINES (ISRs)
 * ============================================================================ */
void PORTB_IRQHandler(void);
void PORTB_IRQHandler(void) {
    static uint32_t last_press_tick = 0;

    if ((g_sys.ticks_250us - last_press_tick) > DEBOUNCE_TICKS) {
        if      (PORTB->ISFR & BIT_MASK(PIN_BTN_S1)) App_OnButtonS1_Pressed();
        else if (PORTB->ISFR & BIT_MASK(PIN_BTN_S2)) App_OnButtonS2_Pressed();
        else if (PORTB->ISFR & BIT_MASK(PIN_BTN_S3)) App_OnButtonS3_Pressed();

        last_press_tick = g_sys.ticks_250us;
    }

    PORTB->ISFR = BIT_MASK(PIN_BTN_S1) | BIT_MASK(PIN_BTN_S2) | BIT_MASK(PIN_BTN_S3);
}

void PIT0_IRQHandler(void);
void PIT0_IRQHandler(void) {
    PIT->CHANNEL[0].TFLG = PIT_TFLG_TIF_MASK;
    g_sys.ticks_250us++;

    if (g_sys.mode == MODE_ALARM_TIMER && g_sys.alarm_triggered && g_sys.alarm_beep_on) {
        HAL_Buzzer_Toggle();
    }

    App_UpdateDisplay();
}

void PIT1_IRQHandler(void);
void PIT1_IRQHandler(void) {
    PIT->CHANNEL[1].TFLG = PIT_TFLG_TIF_MASK;
    App_OnOneSecondTick();
}

/* ============================================================================
 * 7. MAIN ENTRY POINT
 * ============================================================================ */
int main(void) {
    volatile int delay_cnt;

    System_Init_GPIO();
    System_Init_ADC();
    System_Init_Timers();

    g_sys.temp_celsius = HAL_ADC_ReadTempCelsius();

    while (1) {
        if (g_sys.mode == MODE_THERMOMETER) {
            g_sys.temp_celsius = HAL_ADC_ReadTempCelsius();
            HAL_RGB_SetByTemp(g_sys.temp_celsius);
            for (delay_cnt = 0; delay_cnt < 600000; delay_cnt++);
        }
    }
}
