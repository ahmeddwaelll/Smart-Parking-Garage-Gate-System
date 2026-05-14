#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"         
#include "tm4c123gh6pm.h"
#include <stdint.h>
#include <stdbool.h>
#include "basic_io.h"

/* RGB pin masks on Port F */
#define LED_RED      (1U << 1)
#define LED_BLUE     (1U << 2)
#define LED_GREEN    (1U << 3)
#define LED_MASK     (LED_RED | LED_BLUE | LED_GREEN)

/* Button pin masks */
#define BTN_PF4      (1U << 4)
#define BTN_PE0      (1U << 0)
#define BTN_PE1      (1U << 1)
#define BTN_PB0      (1U << 0)
#define BTN_PB1      (1U << 1)
#define BTN_PD0      (1U << 0)
#define BTN_PD1      (1U << 1)

/* Clock-gate masks for Ports B, D, E, F */
#define RCGCGPIO_B   (1U << 1)
#define RCGCGPIO_D   (1U << 3)
#define RCGCGPIO_E   (1U << 4)
#define RCGCGPIO_F   (1U << 5)
#define RCGCGPIO_ALL (RCGCGPIO_B | RCGCGPIO_D | RCGCGPIO_E | RCGCGPIO_F)

QueueHandle_t xButtonQueue;           // Queue for button events
QueueHandle_t xValidatedQueue;        // Queue for validated button presses
SemaphoreHandle_t xObstacleSemaphore; // For obstacle detection
SemaphoreHandle_t xOpenLimitSemaphore;
SemaphoreHandle_t xClosedLimitSemaphore;
SemaphoreHandle_t xGateMutex;         // Protect gate state


//Gate States
typedef enum {
    IDLE_OPEN,
    IDLE_CLOSED,
    OPENING,
    CLOSING,
    STOPPED_MIDWAY,
    REVERSING
} GateState_t;

GateState_t gateState = IDLE_CLOSED;

static void GateState_Set(GateState_t newState) {
    xSemaphoreTake(xGateMutex, portMAX_DELAY);
    gateState = newState;
    xSemaphoreGive(xGateMutex);
}

static GateState_t GateState_Get(void) {
    GateState_t state;
    xSemaphoreTake(xGateMutex, portMAX_DELAY);
    state = gateState;
    xSemaphoreGive(xGateMutex);
    return state;
}

static bool GateState_CompareAndSet(GateState_t expected, GateState_t newState) {
    bool success = false;
    xSemaphoreTake(xGateMutex, portMAX_DELAY);
    if (gateState == expected) {
        gateState = newState;
        success = true;
    }
    xSemaphoreGive(xGateMutex);
    return success;
}


//Events States
typedef enum {
    BTN_DRIVER_OPEN,
    BTN_DRIVER_CLOSE,
    BTN_SECURITY_OPEN,
    BTN_SECURITY_CLOSE,
    BTN_OPEN_LIMIT,
    BTN_CLOSED_LIMIT,
    BTN_OBSTACLE,
	  BTN_RELEASED
} ButtonEvent_t;

ButtonEvent_t ButtonEvent = BTN_CLOSED_LIMIT;

void LED_Set(uint8_t led, bool on) {
    if (on) GPIO_PORTF_DATA_R |= led;
    else GPIO_PORTF_DATA_R &= ~led;
}

void LED_AllOff(void) {
    GPIO_PORTF_DATA_R &= ~(LED_GREEN | LED_RED);
}

//To check if button still pressed after a time (Auto / Manual mode) and to check if button was pressed by wrong
bool IsButtonStillPressedByID(uint8_t u8Button) {
    switch(u8Button) {
        case BTN_DRIVER_OPEN:
            return (GPIO_PORTD_DATA_R & BTN_PD0) ? true : false;
            
        case BTN_DRIVER_CLOSE:
            return (GPIO_PORTD_DATA_R & BTN_PD1) ? true : false;
            
        case BTN_SECURITY_OPEN:
            return (GPIO_PORTB_DATA_R & BTN_PB0) ? true : false;
            
        case BTN_SECURITY_CLOSE:
            return (GPIO_PORTB_DATA_R & BTN_PB1) ? true : false;
            
        case BTN_OPEN_LIMIT:
            return (GPIO_PORTE_DATA_R & BTN_PE0) ? true : false;
            
        case BTN_CLOSED_LIMIT:
            return (GPIO_PORTE_DATA_R & BTN_PE1) ? true : false;
            
        case BTN_OBSTACLE:
            return ((GPIO_PORTF_DATA_R & BTN_PF4) == 0) ? true : false;
            
        default:
            return false;
    }
}

static void GPIO_Init(void)
{
    /* Enable clocks for ports B, D, E, F and wait until ready */
    SYSCTL_RCGCGPIO_R |= RCGCGPIO_ALL;
    while ((SYSCTL_PRGPIO_R & RCGCGPIO_ALL) != RCGCGPIO_ALL) { }

    /* ---------- Port F(obstacle): RGB outputs (PF1-3) and button PF4 ---------- */
    GPIO_PORTF_AMSEL_R &= ~(BTN_PF4 | LED_MASK);
    GPIO_PORTF_PCTL_R  &= ~0x000FFFF0U;   /* digital GPIO function for PF1..PF4 */
    GPIO_PORTF_AFSEL_R &= ~(BTN_PF4 | LED_MASK);

    GPIO_PORTF_DIR_R   |=  LED_MASK;      /* PF1-3 outputs */
    GPIO_PORTF_DIR_R   &= ~BTN_PF4;       /* PF4 input */

    GPIO_PORTF_PUR_R   |=  BTN_PF4;       /* enable pull-up on PF4 switch */
    GPIO_PORTF_DEN_R   |=  BTN_PF4 | LED_MASK;
    GPIO_PORTF_DATA_R  &= ~LED_MASK;      /* LEDs off */

    /* ---------- Port E(limit): PE0, PE1 buttons (pull-down, active-high) ---------- */
    GPIO_PORTE_AMSEL_R &= ~(BTN_PE0 | BTN_PE1);
    GPIO_PORTE_PCTL_R  &= ~0x000000FFU;
    GPIO_PORTE_AFSEL_R &= ~(BTN_PE0 | BTN_PE1);
    GPIO_PORTE_DIR_R   &= ~(BTN_PE0 | BTN_PE1);
    GPIO_PORTE_PDR_R   |=  (BTN_PE0 | BTN_PE1);
    GPIO_PORTE_DEN_R   |=  (BTN_PE0 | BTN_PE1);

    /* ---------- Port B(security): PB0, PB1 buttons (pull-down, active-high) ---------- */
    GPIO_PORTB_AMSEL_R &= ~(BTN_PB0 | BTN_PB1);
    GPIO_PORTB_PCTL_R  &= ~0x000000FFU;
    GPIO_PORTB_AFSEL_R &= ~(BTN_PB0 | BTN_PB1);
    GPIO_PORTB_DIR_R   &= ~(BTN_PB0 | BTN_PB1);
    GPIO_PORTB_PDR_R   |=  (BTN_PB0 | BTN_PB1);
    GPIO_PORTB_DEN_R   |=  (BTN_PB0 | BTN_PB1);

    /* ---------- Port D(driver): PD0, PD1 buttons (pull-down, active-high) ---------- */
    GPIO_PORTD_AMSEL_R &= ~(BTN_PD0 | BTN_PD1);
    GPIO_PORTD_PCTL_R  &= ~0x000000FFU;
    GPIO_PORTD_AFSEL_R &= ~(BTN_PD0 | BTN_PD1);
    GPIO_PORTD_DIR_R   &= ~(BTN_PD0 | BTN_PD1);
    GPIO_PORTD_PDR_R   |=  (BTN_PD0 | BTN_PD1);
    GPIO_PORTD_DEN_R   |=  (BTN_PD0 | BTN_PD1);
		
		
		//Port F interrupt (obstacle button)
		GPIO_PORTF_IS_R &= ~BTN_PF4;   // Edge-sensitive
    GPIO_PORTF_IBE_R &= ~BTN_PF4;  // Not both edges
    GPIO_PORTF_IEV_R &= ~BTN_PF4;  // Falling edge (button press)
    GPIO_PORTF_ICR_R = BTN_PF4;    // Clear any prior interrupts
    GPIO_PORTF_IM_R |= BTN_PF4;    // Enable interrupts 
    
    NVIC_EN0_R |= (1 << 30);
    NVIC_PRI7_R = (NVIC_PRI7_R & 0xFF00FFFF) | (6 << 21);
		
		//Port E interrupt (limit buttons)
		GPIO_PORTE_IS_R &= ~(BTN_PE0 | BTN_PE1);   // Edge-sensitive
    GPIO_PORTE_IBE_R &= ~(BTN_PE0 | BTN_PE1);  // Not both edges
    GPIO_PORTE_IEV_R |= (BTN_PE0 | BTN_PE1);  // Rising edge (button press)
    GPIO_PORTE_ICR_R = (BTN_PE0 | BTN_PE1);    // Clear any prior interrupts
    GPIO_PORTE_IM_R |= (BTN_PE0 | BTN_PE1);    // Enable interrupts for both switches
    
    NVIC_EN0_R |= (1 << 4);
    NVIC_PRI1_R = (NVIC_PRI1_R & 0xFFFFFF1FU) | (7U << 5);
		
		
		//Port B interrupt (security buttons)
		GPIO_PORTB_IS_R &= ~(BTN_PB0 | BTN_PB1);   // Edge-sensitive
    GPIO_PORTB_IBE_R &= ~(BTN_PB0 | BTN_PB1);  // Not both edges
    GPIO_PORTB_IEV_R |= (BTN_PB0 | BTN_PB1);  // Rising edge (button press)
    GPIO_PORTB_ICR_R = (BTN_PB0 | BTN_PB1);    // Clear any prior interrupts
    GPIO_PORTB_IM_R |= (BTN_PB0 | BTN_PB1);    // Enable interrupts for both switches
    
    NVIC_EN0_R |= (1 << 1);
    NVIC_PRI0_R = (NVIC_PRI0_R & 0xFFFF1FFF) | (6 << 13);
		
		//Port D interrupt (driver buttons)
		GPIO_PORTD_IS_R &= ~(BTN_PD0 | BTN_PD1);   // Edge-sensitive
    GPIO_PORTD_IBE_R &= ~(BTN_PD0 | BTN_PD1);  // Not both edges
    GPIO_PORTD_IEV_R |= (BTN_PD0 | BTN_PD1);  // Rising edge (button press)
    GPIO_PORTD_ICR_R = (BTN_PD0 | BTN_PD1);    // Clear any prior interrupts
    GPIO_PORTD_IM_R |= (BTN_PD0 | BTN_PD1);    // Enable interrupts for both switches
    
    NVIC_EN0_R |= (1 << 3);
    NVIC_PRI0_R = (NVIC_PRI0_R & 0x1FFFFFFF) | (7U << 29);
}

//Obstacle Handler
void GPIOF_Handler(void) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    
    if (GPIO_PORTF_RIS_R & BTN_PF4) {
        GPIO_PORTF_ICR_R = BTN_PF4;
        xSemaphoreGiveFromISR(xObstacleSemaphore, &xHigherPriorityTaskWoken);
    }
    
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

//Limit Handler
void GPIOE_Handler(void) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    
    if (GPIO_PORTE_RIS_R & BTN_PE0) {
        GPIO_PORTE_ICR_R = BTN_PE0; 
        xSemaphoreGiveFromISR(xOpenLimitSemaphore, &xHigherPriorityTaskWoken);
    }
    
    if (GPIO_PORTE_RIS_R & BTN_PE1) {
        GPIO_PORTE_ICR_R = BTN_PE1; 
        xSemaphoreGiveFromISR(xClosedLimitSemaphore, &xHigherPriorityTaskWoken);
    }
    
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

//security Handler
void GPIOB_Handler(void) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    ButtonEvent_t eEvent;
    
    if (GPIO_PORTB_RIS_R & BTN_PB0) {
        GPIO_PORTB_ICR_R = BTN_PB0;
        eEvent = BTN_SECURITY_OPEN;
        xQueueSendFromISR(xButtonQueue, &eEvent, &xHigherPriorityTaskWoken);
    }
    
    if (GPIO_PORTB_RIS_R & BTN_PB1) {
        GPIO_PORTB_ICR_R = BTN_PB1;
        eEvent = BTN_SECURITY_CLOSE;
        xQueueSendFromISR(xButtonQueue, &eEvent, &xHigherPriorityTaskWoken);
    }
    
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}


//Driver Handler
void GPIOD_Handler(void) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    ButtonEvent_t eEvent;
    
    if (GPIO_PORTD_RIS_R & BTN_PD0) {
        GPIO_PORTD_ICR_R = BTN_PD0;
        eEvent = BTN_DRIVER_OPEN;
        xQueueSendFromISR(xButtonQueue, &eEvent, &xHigherPriorityTaskWoken);
    }
    
    if (GPIO_PORTD_RIS_R & BTN_PD1) {
        GPIO_PORTD_ICR_R = BTN_PD1;
        eEvent = BTN_DRIVER_CLOSE;
        xQueueSendFromISR(xButtonQueue, &eEvent, &xHigherPriorityTaskWoken);
    }
    
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

//Obstacle Task
void vSafetyTask(void *pvParameters) {
    
    while(1) {
        if (xSemaphoreTake(xObstacleSemaphore, portMAX_DELAY) == pdTRUE) {
					if (GateState_CompareAndSet(CLOSING, REVERSING)) {
                vPrintString("OBSTACLE DETECTED! Gate stopped.\n");
                vTaskDelay(pdMS_TO_TICKS(500));
						    GateState_CompareAndSet(REVERSING, STOPPED_MIDWAY);
            }
        }
    }
}

void vInputTask(void *pvParameters) {
    ButtonEvent_t eEvent;
    TickType_t xPressTime;
    const TickType_t HOLD_THRESHOLD = pdMS_TO_TICKS(300);
    
    while(1) {
        // Wait for button event from ISR
        xQueueReceive(xButtonQueue, &eEvent, portMAX_DELAY);
        
        // Record press time
        xPressTime = xTaskGetTickCount();
        
        // Debounce
        vTaskDelay(pdMS_TO_TICKS(20));
        
        // Verify still pressed using YOUR existing function
        // Verify still pressed using YOUR existing function
        if (IsButtonStillPressedByID(eEvent)) {

            // --- CONFLICT CHECK ---
            if ((IsButtonStillPressedByID(BTN_DRIVER_OPEN) && IsButtonStillPressedByID(BTN_DRIVER_CLOSE)) ||
                (IsButtonStillPressedByID(BTN_SECURITY_OPEN) && IsButtonStillPressedByID(BTN_SECURITY_CLOSE))) {
                
                // Force a safe stop
                ButtonEvent_t releaseEvent = BTN_RELEASED;
                xQueueSend(xValidatedQueue, &releaseEvent, 0);
                continue; // Skip the rest, jump back to reading the queue
            }

            // --- SECURITY PRIORITY ---
            if ((eEvent == BTN_DRIVER_OPEN || eEvent == BTN_DRIVER_CLOSE) &&
                (IsButtonStillPressedByID(BTN_SECURITY_OPEN) || IsButtonStillPressedByID(BTN_SECURITY_CLOSE))) {
                
                continue; // Ignore the driver's command completely
            }

            GateState_t currentState = GateState_Get();

            // Forward press event to Gate Task
            xQueueSend(xValidatedQueue, &eEvent, 0);

            // Wait for release
            while (IsButtonStillPressedByID(eEvent)) {
                
                // --- ESCAPE HATCH FOR SIMULTANEOUS PRESSES ---
                // Break if Security overrides Driver mid-press
                if ((eEvent == BTN_DRIVER_OPEN || eEvent == BTN_DRIVER_CLOSE) &&
                    (IsButtonStillPressedByID(BTN_SECURITY_OPEN) || IsButtonStillPressedByID(BTN_SECURITY_CLOSE))) {
                    break;
                }
                
                // Break if user mashes the opposite button mid-press
                if (eEvent == BTN_DRIVER_OPEN && IsButtonStillPressedByID(BTN_DRIVER_CLOSE)) break;
                if (eEvent == BTN_DRIVER_CLOSE && IsButtonStillPressedByID(BTN_DRIVER_OPEN)) break;
                if (eEvent == BTN_SECURITY_OPEN && IsButtonStillPressedByID(BTN_SECURITY_CLOSE)) break;
                if (eEvent == BTN_SECURITY_CLOSE && IsButtonStillPressedByID(BTN_SECURITY_OPEN)) break;

                vTaskDelay(pdMS_TO_TICKS(10));
            }

            // Calculate hold duration
            if ((xTaskGetTickCount() - xPressTime) >= HOLD_THRESHOLD) {
                // Manual mode: send RELEASED event to stop the gate
                ButtonEvent_t release = BTN_RELEASED;
                xQueueSend(xValidatedQueue, &release, 0);
            } 
        }
    }
}

void vGateControlTask(void *pvParameters) {
    ButtonEvent_t eReceivedEvent;
    GateState_t eCurrentState;

    while(1) {
        // Step 1: Check Limits First
        if (xSemaphoreTake(xOpenLimitSemaphore, 0) == pdTRUE) {
            eCurrentState = GateState_Get();
            
            if (eCurrentState == OPENING) {
                GateState_Set(IDLE_OPEN);
                vPrintString("Open Limit Hit -> IDLE_OPEN\n");
            } 
        }

        if (xSemaphoreTake(xClosedLimitSemaphore, 0) == pdTRUE) {
            eCurrentState = GateState_Get();
            
            if (eCurrentState == CLOSING) {
                GateState_Set(IDLE_CLOSED);
                vPrintString("Closed Limit Hit -> IDLE_CLOSED\n");
            }
        }

        // Step 2: Check for User Commands
        if (xQueueReceive(xValidatedQueue, &eReceivedEvent, pdMS_TO_TICKS(20)) == pdTRUE) {
            eCurrentState = GateState_Get();

            switch(eCurrentState) {
                case IDLE_CLOSED:
                    if (eReceivedEvent == BTN_DRIVER_OPEN || eReceivedEvent == BTN_SECURITY_OPEN) {
                        GateState_Set(OPENING);
                    }
                    break;

                case IDLE_OPEN:
                    if (eReceivedEvent == BTN_DRIVER_CLOSE || eReceivedEvent == BTN_SECURITY_CLOSE) {
                        GateState_Set(CLOSING);
                    }
                    break;

                case OPENING:
                    if (eReceivedEvent == BTN_RELEASED || 
                        eReceivedEvent == BTN_DRIVER_CLOSE || 
                        eReceivedEvent == BTN_SECURITY_CLOSE) {
                        GateState_Set(STOPPED_MIDWAY);
                    }
                    break;

                case CLOSING:
                    if (eReceivedEvent == BTN_RELEASED || 
                        eReceivedEvent == BTN_DRIVER_OPEN || 
                        eReceivedEvent == BTN_SECURITY_OPEN) {
                        GateState_Set(STOPPED_MIDWAY);
                    }
                    break;

                case STOPPED_MIDWAY:
                    if (eReceivedEvent == BTN_DRIVER_OPEN || eReceivedEvent == BTN_SECURITY_OPEN) {
                        GateState_Set(OPENING);
                    } 
                    else if (eReceivedEvent == BTN_DRIVER_CLOSE || eReceivedEvent == BTN_SECURITY_CLOSE) {
                        GateState_Set(CLOSING);
                    }
                    break;

                case REVERSING:
                    // Do nothing! 
                    break;
            }
        }
    }
}

void vLedControlTask(void *pvParameters) {
    GateState_t eCurrentState;
    
    while(1) {
        xSemaphoreTake(xGateMutex, portMAX_DELAY);
        eCurrentState = gateState;
        xSemaphoreGive(xGateMutex);
        
        if (eCurrentState == OPENING || eCurrentState == REVERSING) {
            LED_AllOff();
            LED_Set(LED_GREEN, true);
        }
        else if (eCurrentState == CLOSING) {
            LED_AllOff();
            LED_Set(LED_RED, true);
        }
        else {
            LED_AllOff();
        }
        
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

int main(void) {
    GPIO_Init();
    
    xButtonQueue = xQueueCreate(10, sizeof(ButtonEvent_t));
		xValidatedQueue = xQueueCreate(10, sizeof(ButtonEvent_t));
    xObstacleSemaphore = xSemaphoreCreateBinary();
	  xOpenLimitSemaphore = xSemaphoreCreateBinary();
    xClosedLimitSemaphore = xSemaphoreCreateBinary();
    xGateMutex = xSemaphoreCreateMutex();
    
    gateState = IDLE_CLOSED;
    
    xTaskCreate(vSafetyTask, "Safety", 256, NULL, 4, NULL);
    xTaskCreate(vInputTask, "Input", 256, NULL, 3, NULL);
    xTaskCreate(vGateControlTask, "Gate Control", 512, NULL, 2, NULL);
    xTaskCreate(vLedControlTask, "LED", 128, NULL, 2, NULL);
    
    vTaskStartScheduler();
    
    while(1);
}
