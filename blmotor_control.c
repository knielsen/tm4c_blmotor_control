/*
  Controlling brushless DC motor with my custom Tiva TM4C PCB with ST L6234.
*/

#include <inttypes.h>
#include <math.h>

#include "inc/hw_gpio.h"
#include "inc/hw_memmap.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "inc/hw_timer.h"
#include "inc/hw_nvic.h"
#include "driverlib/gpio.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/timer.h"
#include "driverlib/interrupt.h"


/*
  Pinout:

    IN1   PG0   (timer t4ccp0)
    IN2   PG1   (timer t4ccp1)
    IN3   PG2   (timer t5ccp0)
    EN1   PG3
    EN2   PG4
    EN3   PG5
*/

/* To change this, must fix clock setup in the code. */
#define MCU_HZ 80000000


#define PWM_FREQ 50000
#define PWM_PERIOD (MCU_HZ/PWM_FREQ)
/* L6234 adds 300 ns of deadtime. */
#define DEADTIME (MCU_HZ/1000*300/1000000)

static const float F_PI = 3.141592654f;

static void motor_update(void);


static void
serial_output_hexdig(uint32_t dig)
{
  ROM_UARTCharPut(UART0_BASE, (dig >= 10 ? 'A' - 10 + dig : '0' + dig));
}


 __attribute__ ((unused))
static void
serial_output_hexbyte(uint8_t byte)
{
  serial_output_hexdig(byte >> 4);
  serial_output_hexdig(byte & 0xf);
}


static void
serial_output_str(const char *str)
{
  char c;

  while ((c = *str++))
    ROM_UARTCharPut(UART0_BASE, c);
}


 __attribute__ ((unused))
static void
println_uint32(uint32_t val)
{
  char buf[13];
  char *p = buf;
  uint32_t l, d;

  l = 1000000000UL;
  while (l > val && l > 1)
    l /= 10;

  do
  {
    d = val / l;
    *p++ = '0' + d;
    val -= d*l;
    l /= 10;
  } while (l > 0);

  *p++ = '\r';
  *p++ = '\n';
  *p = '\0';
  serial_output_str(buf);
}


static void
float_to_str(char *buf, float f, uint32_t dig_before, uint32_t dig_after)
{
  float a;
  uint32_t d;
  uint8_t leading_zero;

  if (f == 0.0f)
  {
    buf[0] = '0';
    buf[1] = '\0';
    return;
  }
  if (f < 0)
  {
    *buf++ = '-';
    f = -f;
  }
  a =  powf(10.0f, (float)dig_before);
  if (f >= a)
  {
    buf[0] = '#';
    buf[1] = '\0';
    return;
  }
  leading_zero = 1;
  while (dig_before)
  {
    a /= 10.0f;
    d = (uint32_t)(f / a);
    if (leading_zero && d == 0 && a >= 10.0f)
      *buf++ = ' ';
    else
    {
      leading_zero = 0;
      *buf++ = '0' + d;
      f -= d*a;
    }
    --dig_before;
  }
  if (!dig_after)
  {
    *buf++ = '\0';
    return;
  }
  *buf++ = '.';
  do
  {
    f *= 10.0f;
    d = (uint32_t)f;
    *buf++ = '0' + d;
    f -= (float)d;
    --dig_after;
  } while (dig_after);
  *buf++ = '\0';
}


 __attribute__ ((unused))
static void
println_float(float f, uint32_t dig_before, uint32_t dig_after)
{
  char buf[21];
  char *p = buf;

  float_to_str(p, f, dig_before, dig_after);
  while (*p)
    ++p;
  *p++ = '\r';
  *p++ = '\n';
  *p = '\0';
  serial_output_str(buf);
}


static void
config_led(void)
{
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
  ROM_GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_7);
}


__attribute__ ((unused))
static void
led_on(void)
{
  ROM_GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, GPIO_PIN_7);
}


__attribute__ ((unused))
static void
led_off(void)
{
  ROM_GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, 0);
}


static volatile uint32_t pwm1_match_value;
static volatile uint32_t pwm2_match_value;
static volatile uint32_t pwm3_match_value;


static void
setup_timer_pwm(void)
{
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER4);
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER5);
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG);
  ROM_GPIOPinConfigure(GPIO_PG0_T4CCP0);
  ROM_GPIOPinConfigure(GPIO_PG1_T4CCP1);
  ROM_GPIOPinConfigure(GPIO_PG2_T5CCP0);
  ROM_GPIOPinTypeTimer(GPIO_PORTG_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2);
  //ROM_GPIOPadConfigSet(GPIO_PORTG_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2,
  //                     GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_STD);
  ROM_GPIOPinTypeGPIOOutput(GPIO_PORTG_BASE, GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5);
  /* Set EN pins low, for off. */
  ROM_GPIOPinWrite(GPIO_PORTG_BASE, GPIO_PIN_3, 0);
  ROM_GPIOPinWrite(GPIO_PORTG_BASE, GPIO_PIN_4, 0);
  ROM_GPIOPinWrite(GPIO_PORTG_BASE, GPIO_PIN_5, 0);

  ROM_TimerConfigure(TIMER4_BASE,
                     TIMER_CFG_SPLIT_PAIR|TIMER_CFG_A_PWM|TIMER_CFG_B_PWM);
  ROM_TimerConfigure(TIMER5_BASE,
                     TIMER_CFG_SPLIT_PAIR|TIMER_CFG_A_PWM|TIMER_CFG_B_ONE_SHOT);

  ROM_TimerLoadSet(TIMER4_BASE, TIMER_BOTH, PWM_PERIOD);
  ROM_TimerLoadSet(TIMER5_BASE, TIMER_A, PWM_PERIOD);
  pwm1_match_value = pwm2_match_value = pwm3_match_value = PWM_PERIOD-1;
  ROM_TimerMatchSet(TIMER4_BASE, TIMER_A, pwm1_match_value);
  ROM_TimerMatchSet(TIMER4_BASE, TIMER_B, pwm2_match_value);
  ROM_TimerMatchSet(TIMER5_BASE, TIMER_A, pwm3_match_value);
  /*
    Set the MRSU bit in config register, so that we can change the PWM duty
    cycle on-the-fly, and the new value will take effect at the start of the
    next period.
  */
  HWREG(TIMER4_BASE + TIMER_O_TAMR) |= TIMER_TAMR_TAMRSU;
  HWREG(TIMER4_BASE + TIMER_O_TBMR) |= TIMER_TBMR_TBMRSU;
  HWREG(TIMER5_BASE + TIMER_O_TAMR) |= TIMER_TAMR_TAMRSU;

  ROM_IntMasterEnable();
  ROM_TimerControlEvent(TIMER4_BASE, TIMER_BOTH, TIMER_EVENT_POS_EDGE);
  ROM_TimerControlEvent(TIMER5_BASE, TIMER_A, TIMER_EVENT_POS_EDGE);
  ROM_TimerIntEnable(TIMER4_BASE, TIMER_CAPA_EVENT|TIMER_CAPB_EVENT);
  ROM_TimerIntEnable(TIMER5_BASE, TIMER_CAPA_EVENT);
  ROM_IntEnable(INT_TIMER4A);
  ROM_IntEnable(INT_TIMER4B);
  ROM_IntEnable(INT_TIMER5A);

  ROM_TimerEnable(TIMER4_BASE, TIMER_BOTH);
  ROM_TimerEnable(TIMER5_BASE, TIMER_A);

  /*
    Synchronise the timers.

    We can not use wait-for-trigger, as there is an errata GPTM#04 that
    wait-for-trigger is not available for PWM mode.

    So we need to use the SYNC register.
    There is also an errata for SYNC:

      "GPTM#01 GPTMSYNC Bits Require Manual Clearing"

    Since the sync register for all timers is in timer 0, that timer must be
    enabled.
  */
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
  HWREG(TIMER0_BASE+TIMER_O_SYNC) |=
    (uint32_t)(TIMER_4A_SYNC|TIMER_4B_SYNC|TIMER_5A_SYNC);
  HWREG(TIMER0_BASE+TIMER_O_SYNC) &=
    ~(uint32_t)(TIMER_4A_SYNC|TIMER_4B_SYNC|TIMER_5A_SYNC);
}


static void
l6234_enable(void)
{
  ROM_GPIOPinWrite(GPIO_PORTG_BASE, GPIO_PIN_3, GPIO_PIN_3);
  ROM_GPIOPinWrite(GPIO_PORTG_BASE, GPIO_PIN_4, GPIO_PIN_4);
  ROM_GPIOPinWrite(GPIO_PORTG_BASE, GPIO_PIN_5, GPIO_PIN_5);
}


static void
l6234_disable(void)
{
  ROM_GPIOPinWrite(GPIO_PORTG_BASE, GPIO_PIN_3, 0);
  ROM_GPIOPinWrite(GPIO_PORTG_BASE, GPIO_PIN_4, 0);
  ROM_GPIOPinWrite(GPIO_PORTG_BASE, GPIO_PIN_5, 0);
}


static const float damper = 0.3f;

static void
set_pwm(float duty1, float duty2, float duty3)
{
  /*
    The L6234 inserts 300 ns of deadtime to protect against both MOSFETs being
    open at the same time. Adjust the PWM match value accordingly, so that we
    get correct ratios between duty cycles also for small duty cycle values
    (which may be needed to limit the current used in beefy motors).
  */
  pwm1_match_value = (PWM_PERIOD-DEADTIME) -
    (uint32_t)(duty1 * (damper*(float)(PWM_PERIOD-2*DEADTIME)));
  pwm2_match_value = (PWM_PERIOD-DEADTIME) -
    (uint32_t)(duty2 * (damper*(float)(PWM_PERIOD-2*DEADTIME)));
  pwm3_match_value = (PWM_PERIOD-DEADTIME) -
    (uint32_t)(duty3 * (damper*(float)(PWM_PERIOD-2*DEADTIME)));
}


static void
set_motor(float angle)
{
  float s1, s2, s3;
  s1 = sinf(angle);
  s2 = sinf(angle + (2.0f*F_PI/3.0f));
  s3 = sinf(angle + (2.0f*F_PI*2.0f/3.0f));
  set_pwm(0.5f + 0.5f*s1, 0.5f + 0.5f*s2, 0.5f + 0.5f*s3);
}


void
IntHandlerTimer4A(void)
{
  /* Clear the interrupt. */
  HWREG(TIMER4_BASE + TIMER_O_ICR) = TIMER_CAPA_EVENT;
  /* Set the duty cycle for the following PWM period. */
  HWREG(TIMER4_BASE + TIMER_O_TAMATCHR) = pwm1_match_value;
}


void
IntHandlerTimer4B(void)
{
  /* Clear the interrupt. */
  HWREG(TIMER4_BASE + TIMER_O_ICR) = TIMER_CAPB_EVENT;
  /* Set the duty cycle for the following PWM period. */
  HWREG(TIMER4_BASE + TIMER_O_TBMATCHR) = pwm2_match_value;
}


void
IntHandlerTimer5A(void)
{
  /* Clear the interrupt. */
  HWREG(TIMER5_BASE + TIMER_O_ICR) = TIMER_CAPA_EVENT;
  /* Set the duty cycle for the following PWM period. */
  HWREG(TIMER5_BASE + TIMER_O_TAMATCHR) = pwm3_match_value;

  motor_update();
}


static volatile uint32_t motor_max = 0;
static uint32_t motor_idx = 0;

static void
motor_update(void)
{
  uint32_t l_idx, l_max;
  float angle;

  l_max = motor_max;
  if (l_max == 0)
    return;
  l_idx = motor_idx;
  ++l_idx;
  if (l_idx >= motor_max)
  {
    l_idx = 0;
  }
  motor_idx = l_idx;

  angle = (float)l_idx/(float)l_max*(2.0f*F_PI);
  set_motor(angle);
}


int main()
{
  uint32_t counter;
  uint32_t led_state;

  /* Use the full 80MHz system clock. */
  ROM_SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL |
                     SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
  ROM_FPULazyStackingEnable();

  /* Configure serial. */
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
  ROM_GPIOPinConfigure(GPIO_PA0_U0RX);
  ROM_GPIOPinConfigure(GPIO_PA1_U0TX);
  ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
  ROM_UARTConfigSetExpClk(UART0_BASE, (ROM_SysCtlClockGet()), 500000,
                      (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                       UART_CONFIG_PAR_NONE));

  config_led();

  /* Set interrupts to use no sub-priorities. */
  ROM_IntPriorityGroupingSet(7);
  /* Setup the priorities of the interrupts we use. */
  ROM_IntPrioritySet(INT_TIMER4A, 0 << 5);
  ROM_IntPrioritySet(INT_TIMER4B, 0 << 5);
  ROM_IntPrioritySet(INT_TIMER5A, 0 << 5);

  setup_timer_pwm();
  l6234_disable();

  serial_output_str("Motor controller init done\r\n");

  counter = 0;
  led_state = 0;

  motor_max = 50000/(7*1);
  l6234_enable();

  for (;;)
  {
    ROM_SysCtlDelay(MCU_HZ/3/1000);
    if (++counter >= 1000)
    {
      counter = 0;

      /* Show the value of the pwm match registers... */
      serial_output_str("ph1: ");
      println_uint32(HWREG(TIMER4_BASE + TIMER_O_TAMATCHR));
      serial_output_str("  ph2: ");
      println_uint32(HWREG(TIMER4_BASE + TIMER_O_TBMATCHR));
      serial_output_str("  ph3: ");
      println_uint32(HWREG(TIMER5_BASE + TIMER_O_TAMATCHR));

      if (led_state)
      {
        led_off();
        led_state = 0;
      }
      else
      {
        led_on();
        led_state = 1;
      }
    }
  }
}
