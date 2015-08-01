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
#include "inc/hw_i2c.h"
#include "driverlib/gpio.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/timer.h"
#include "driverlib/interrupt.h"
#include "driverlib/i2c.h"

#include "mpu6050.h"


/*
  Pinout:

    IN1   PG0   (timer t4ccp0)
    IN2   PG1   (timer t4ccp1)
    IN3   PG2   (timer t5ccp0)
    EN1   PG3
    EN2   PG4
    EN3   PG5


  MPU6050:
    SCL   PD0  (POV3 SCLK på pcb_pov)
    SDA   PD1  (POV3 MODE på pcb_pov)
    AD0   GND
*/

/* To change this, must fix clock setup in the code. */
#define MCU_HZ 80000000


#define PWM_FREQ 50000
#define PWM_PERIOD (MCU_HZ/PWM_FREQ)
/* L6234 adds 300 ns of deadtime. */
#define DEADTIME (MCU_HZ/1000*300/1000000)


/* MPU6050 stuff */
#define MPU6050_I2C_ADDR 0x68
#define SAMPLE_RATE 1000


/* Define this to get raw accelerometer data out. */
// ToDo: Enable/disable this with a key instead?
//#define RAW_DATA 1
//#define MPU_MEASURE 1


static const float F_PI = 3.141592654f;

static void motor_update(void);


static void
serial_output_hexdig(uint32_t dig)
{
#ifndef RAW_DATA
  ROM_UARTCharPut(UART0_BASE, (dig >= 10 ? 'A' - 10 + dig : '0' + dig));
#endif
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
#ifndef RAW_DATA
  char c;

  while ((c = *str++))
    ROM_UARTCharPut(UART0_BASE, c);
#endif
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


static float damper = 0.3f;

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


/*
  The current electrical angle.
  One turn corresponds to a value of 2**32.
*/
static volatile uint32_t cur_angle = 0;
/* The current speed, added to cur_angle every 1/PWM_FREQ. */
static volatile uint32_t angle_inc = 0;
/* Counter, counting at PWM_FREQ. */
static volatile uint32_t cur_pwm_tick = 0;
/* Counter, increments by one every electric turn (360 degree). */
static volatile uint32_t cur_elec_turns = 0;
/*
  Speed control.

  A value of SPEED_STABLE means the speed is currently not changing.

  A value of SPEED_CHANGING means the interrupt routine is in the process
  of speeding the motor gently up or down.

  Setting this to SPEED_CHANGE causes the interrupt routine to start
  changing the speed to change_speed_start, over an interval of
  speed_change_duration ticks (at PWM_FREQ Hz). The speed is the value
  of angle_inc.
*/
enum { SPEED_STABLE, SPEED_CHANGE, SPEED_CHANGING };
static volatile uint8_t speed_change_status = SPEED_STABLE;
static volatile uint32_t speed_change_to = 0;
static volatile uint32_t speed_change_duration = 0;
/* Internal state for speed change. */
static uint32_t change_speed_from = 0;
static int32_t change_speed_delta = 0;
static uint32_t change_speed_start = 0;
static uint32_t change_speed_interval = 0;

static void
motor_update(void)
{
  uint32_t l_angle, l_inc, tmp;
  uint32_t l_tick;
  uint8_t l_status;
  float f_angle;

  /* Set the PWM for the current electrical angle. */
  l_angle = cur_angle;
  f_angle = (float)l_angle * (2.0f*F_PI/4294967296.0f);
  set_motor(f_angle);

  /* Increment the electrical angle at the current speed. */
  l_inc = angle_inc;
  tmp = l_angle + angle_inc;
  if (tmp < l_angle)
    ++cur_elec_turns;
  l_angle = tmp;
  cur_angle = l_angle;

  l_tick = cur_pwm_tick;
  l_status = speed_change_status;
  if (l_status == SPEED_CHANGING)
  {
    /* Do the next step of speed change. */
    uint32_t l_start = change_speed_start;
    uint32_t l_interval = change_speed_interval;
    uint32_t l_from = change_speed_from;
    int64_t delta = change_speed_delta;
    uint32_t sofar;

    sofar = l_tick - l_start;
    delta = (delta * sofar + l_interval/2) / l_interval;
    l_inc = (int64_t)l_from + delta;
    angle_inc = l_inc;
    /* Check if speed change is complete. */
    if (sofar >= l_interval)
      speed_change_status = l_status = SPEED_STABLE;
  }
  else if (l_status == SPEED_CHANGE)
  {
    /* Setup state to start changing speed as requested. */
    change_speed_from = l_inc;
    change_speed_delta = (int32_t)(speed_change_to - l_inc);
    change_speed_start = l_tick;
    change_speed_interval = speed_change_duration;
    speed_change_status = l_status = SPEED_CHANGING;
  }

  cur_pwm_tick = l_tick + 1;
}


static void
setup_systick(void)
{
  ROM_SysTickPeriodSet(0xffffff+1);
  /* Force reload. */
  HWREG(NVIC_ST_CURRENT) = 0;
  ROM_SysTickEnable();
}


static const uint32_t time_period= 0x1000000;
static inline uint32_t
get_time(void)
{
  return HWREG(NVIC_ST_CURRENT);
}


static void
write_mpu6050_reg(uint8_t reg, uint8_t val)
{
  unsigned long err;

  ROM_I2CMasterSlaveAddrSet(I2C3_MASTER_BASE, MPU6050_I2C_ADDR, 0);
  ROM_I2CMasterDataPut(I2C3_MASTER_BASE, reg);
  ROM_I2CMasterControl(I2C3_MASTER_BASE, I2C_MASTER_CMD_BURST_SEND_START);
  /*
    ToDo: Need some kind of timeout here. Otherwise it seems we get
    stuck forever if eg. there is no MPU6050 or the communication fails
    somehow.
  */
  while (ROM_I2CMasterBusy(I2C3_MASTER_BASE))
    ;
  err = ROM_I2CMasterErr(I2C3_MASTER_BASE);
  if (err)
  {
    if (!(err & I2C_MCS_ARBLST))
      ROM_I2CMasterControl(I2C3_MASTER_BASE, I2C_MASTER_CMD_BURST_SEND_STOP);
    serial_output_str("Got error from I2C on first byte: ");
    println_uint32((uint32_t)err);
    return;
  }
  ROM_I2CMasterDataPut(I2C3_MASTER_BASE, val);
  ROM_I2CMasterControl(I2C3_MASTER_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
  while (ROM_I2CMasterBusy(I2C3_MASTER_BASE))
    ;
  err = ROM_I2CMasterErr(I2C3_MASTER_BASE);
  if (err)
  {
    serial_output_str("Got error from I2C on last byte: ");
    println_uint32((uint32_t)err);
  }
}


static void
read_mpu6050_reg_multi(uint8_t reg, uint8_t *buf, uint32_t len)
{
  unsigned long err;

  if (len == 0)
    return;
  ROM_I2CMasterSlaveAddrSet(I2C3_MASTER_BASE, MPU6050_I2C_ADDR, 0);
  ROM_I2CMasterDataPut(I2C3_MASTER_BASE, reg);
  ROM_I2CMasterControl(I2C3_MASTER_BASE, I2C_MASTER_CMD_BURST_SEND_START);
  while (ROM_I2CMasterBusy(I2C3_MASTER_BASE))
    ;
  err = ROM_I2CMasterErr(I2C3_MASTER_BASE);
  if (err)
  {
    if (!(err & I2C_MCS_ARBLST))
      ROM_I2CMasterControl(I2C3_MASTER_BASE, I2C_MASTER_CMD_BURST_SEND_STOP);
    serial_output_str("Receive: Got error from I2C on reg: ");
    println_uint32((uint32_t)err);
    return;
  }

  ROM_I2CMasterSlaveAddrSet(I2C3_MASTER_BASE, MPU6050_I2C_ADDR, 1);
  ROM_I2CMasterControl(I2C3_MASTER_BASE,
                       (len == 1 ?
                        I2C_MASTER_CMD_SINGLE_RECEIVE :
                        I2C_MASTER_CMD_BURST_RECEIVE_START));
  for (;;)
  {
    while (ROM_I2CMasterBusy(I2C3_MASTER_BASE))
      ;
    err = ROM_I2CMasterErr(I2C3_MASTER_BASE);
    if (err)
    {
      if (!(err & I2C_MCS_ARBLST))
        ROM_I2CMasterControl(I2C3_MASTER_BASE, I2C_MASTER_CMD_BURST_RECEIVE_ERROR_STOP);
      serial_output_str("Receive: Got error from I2C: ");
      println_uint32((uint32_t)err);
      return;
    }
    *buf++ = ROM_I2CMasterDataGet(I2C3_MASTER_BASE);
    --len;
    if (len == 0)
      break;
    ROM_I2CMasterControl(I2C3_MASTER_BASE,
                         (len == 1 ? I2C_MASTER_CMD_BURST_RECEIVE_FINISH :
                                     I2C_MASTER_CMD_BURST_RECEIVE_CONT));
  }
}


static uint8_t
read_mpu6050_reg(uint8_t reg)
{
  uint8_t val = 0;

  read_mpu6050_reg_multi(reg, &val, 1);
  return val;
}


static void
config_i2c_mpu6050(void)
{
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C3);
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
  {
    //ROM_GPIOPinTypeI2CSCL(GPIO_PORTD_BASE, GPIO_PIN_0);
    ROM_GPIODirModeSet(GPIO_PORTD_BASE, GPIO_PIN_0, GPIO_DIR_MODE_HW);
    ROM_GPIOPadConfigSet(GPIO_PORTD_BASE, GPIO_PIN_0,
                         GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD);
  }
  ROM_GPIOPinTypeI2C(GPIO_PORTD_BASE, GPIO_PIN_1);
  ROM_GPIOPinConfigure(GPIO_PD0_I2C3SCL);
  ROM_GPIOPinConfigure(GPIO_PD1_I2C3SDA);
  ROM_I2CMasterInitExpClk(I2C3_MASTER_BASE, MCU_HZ, 1);
  ROM_I2CMasterSlaveAddrSet(I2C3_MASTER_BASE, MPU6050_I2C_ADDR, 0);
}


static inline float
sensor_value(uint8_t *p, float max)
{
  int16_t iv;
  float v;

  iv = (int16_t)(((uint16_t)p[0] << 8) | (uint16_t)p[1]);
  v = (float)iv * (max / 32768.0f);
  return v;
}


 __attribute__ ((unused))
static void
config_mpu6050(void)
{
  uint8_t buf[14];
  uint32_t i;

  config_i2c_mpu6050();

  for (;;)
  {
    uint8_t res;

    /*
      First take it out of sleep mode (writes seem to not stick until we take it
      out of sleep mode). Then issue a reset to get a well-defined starting state
      (and go out of sleep mode again).
    */
    write_mpu6050_reg(MPU6050_REG_PWR_MGMT_1, 0x02);
    ROM_SysCtlDelay(150000);
    res = read_mpu6050_reg(MPU6050_REG_PWR_MGMT_1);
    serial_output_str("Read PWR_MGMT=");
    println_uint32(res);
    if (res != 0x02)
      continue;
    write_mpu6050_reg(MPU6050_REG_PWR_MGMT_1, 0x82);
    ROM_SysCtlDelay(150000);
    res = read_mpu6050_reg(MPU6050_REG_PWR_MGMT_1);
    serial_output_str("Read2 PWR_MGMT=");
    println_uint32(res);
    if (res != 0x40)
      continue;
    write_mpu6050_reg(MPU6050_REG_PWR_MGMT_1, 0x02);
    ROM_SysCtlDelay(150000);
    res = read_mpu6050_reg(MPU6050_REG_PWR_MGMT_1);
    serial_output_str("Read3 PWR_MGMT=");
    println_uint32(res);
    if (res != 0x02)
      continue;

    /* Disable digital low-pass filter (DLPF) */
    write_mpu6050_reg(MPU6050_REG_CONFIG, 0);
    /* 1000 Hz sample rate. */
    write_mpu6050_reg(MPU6050_REG_SMPRT_DIV, 7);
    /*
      Resolution and range. 3 is lowest resolution, +-2000 degrees / second
      and +-16g.
    */
    write_mpu6050_reg(MPU6050_REG_GYRO_CONFIG, 2 << 3);
    write_mpu6050_reg(MPU6050_REG_ACCEL_CONFIG, 0 << 3);
    /* Disable the Fifo (write 0xf8 to enable temp+gyros_accel). */
    write_mpu6050_reg(MPU6050_REG_FIFO_EN, 0x00);
    /*
      Interrupt. Active high, push-pull, hold until cleared, cleared only on
      read of status.
    */
    write_mpu6050_reg(MPU6050_REG_INT_PIN_CFG, 0x20);
    /* Enable FIFO overflow and data ready interrupts. */
    write_mpu6050_reg(MPU6050_REG_INT_ENABLE, 0x11);
    /* Disable the FIFO and external I2C master mode. */
    write_mpu6050_reg(MPU6050_REG_USER_CTRL, 0x00);

    break;
  }

  serial_output_str("Read ACCEL_CONFIG=");
  serial_output_hexbyte(read_mpu6050_reg(MPU6050_REG_ACCEL_CONFIG));
  serial_output_str("\n");

  read_mpu6050_reg_multi(MPU6050_REG_ACCEL_XOUT_H, buf, 14);
  serial_output_str("Reading:");
  for (i = 0; i < 14; ++i)
  {
    serial_output_str(" ");
    serial_output_hexbyte(buf[i]);
  }
  serial_output_str("\r\n");
  serial_output_str("ACC_X:\t");
  println_float(sensor_value(&buf[0], 2.0f), 2, 3);
  serial_output_str("ACC_Y:\t");
  println_float(sensor_value(&buf[2], 2.0f), 2, 3);
  serial_output_str("ACC_Z:\t");
  println_float(sensor_value(&buf[4], 2.0f), 2, 3);
  serial_output_str("TEMP:\t");
  println_float(sensor_value(&buf[6], 32768.0f)/340.0f + 36.53f, 3, 2);
  serial_output_str("GYRO_X:\t");
  println_float(sensor_value(&buf[8], 1000.0f), 4, 1);
  serial_output_str("GYRO_Y:\t");
  println_float(sensor_value(&buf[10], 1000.0f), 4, 1);
  serial_output_str("GYRO_Z:\t");
  println_float(sensor_value(&buf[12], 1000.0f), 4, 1);
}


 __attribute__ ((unused))
static void
mpu6050_read_accel(uint8_t buf[6])
{
  read_mpu6050_reg_multi(MPU6050_REG_ACCEL_XOUT_H, buf, 6);
}


int main()
{
  uint32_t led_state;
  uint32_t time_wraps, last_time;
  float rampup_seconds, electric_rps;
  float next_checkpoint_seconds;
#ifdef MPU_MEASURE
  float next_mpu_seconds, end_mpu_seconds;
  uint32_t stop_mpu_angle = 0, stop_mpu_eturns = 0, last_mpu_delta = 0;
  uint32_t prev_angle = 0;
#endif
  uint32_t hit_target = 0;
  long user_input;

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

#ifdef MPU_MEASURE
  config_mpu6050();
#endif

  setup_systick();
  last_time = get_time();
  time_wraps = 0;

  serial_output_str("Motor controller init done\r\n");

  led_state = 0;

  /*
    Start slowly, then ramp up speed.
    That's a primitive way to start up without actually measuring the
    position of the motor phases with feedback or hall sensors.
  */
  rampup_seconds = 8.0f;
  electric_rps = 4.0f*25.0f;
  damper = 0.5f;

  speed_change_to = electric_rps*((float)0x100000000/(float)PWM_FREQ);
  speed_change_duration = (uint32_t)(rampup_seconds * PWM_FREQ);
  speed_change_status = SPEED_CHANGE;
  l6234_enable();

  next_checkpoint_seconds = 0.500f;
#ifdef MPU_MEASURE
  next_mpu_seconds = 0.001f;
  end_mpu_seconds = 0.000f;
#endif
  for (;;)
  {
    float seconds;
    uint32_t cur_time;

    cur_time = get_time();
    if (cur_time > last_time)
      ++time_wraps;
    seconds = (float)time_wraps*((float)time_period/(float)MCU_HZ) +
      (float)((time_period-1) - cur_time)*(1.0f/(float)MCU_HZ);
    last_time = cur_time;

    if (seconds >= next_checkpoint_seconds)
    {
      next_checkpoint_seconds += 0.500f;
      serial_output_str("eRPS:\t");
      println_float((float)angle_inc*((float)PWM_FREQ/(float)0x100000000), 3, 3);
      serial_output_str("  ainc:\t");
      println_uint32(angle_inc);
      serial_output_str("  damp:\t");
      println_float(damper, 1, 4);
#ifdef MPU_MEASURE
      serial_output_str("  ang:\t");
      println_uint32(cur_angle);
      serial_output_str("  turns:\t");
      println_uint32(cur_elec_turns);
      serial_output_str("  hit:\t");
      println_uint32(hit_target);
#endif

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

    if (speed_change_status == SPEED_STABLE && !hit_target)
    {
      hit_target = 1;
#ifdef MPU_MEASURE
      next_mpu_seconds = seconds + 4.000f;
#endif
    }

    user_input = ROM_UARTCharGetNonBlocking(UART0_BASE);
    if (user_input > 0)
    {
      char ch = (char)user_input;

      switch(ch)
      {
      case 'w':
        if (angle_inc >= 100000)
          angle_inc -= 100000;
        break;
      case 'q':
        if (angle_inc < 1000000000)
          angle_inc += 100000;
        break;
      case 'a':
        if (angle_inc >= 10000)
          angle_inc -= 10000;
        break;
      case 's':
        if (angle_inc < 1000000000)
          angle_inc += 10000;
        break;
      case 'z':
        if (angle_inc >= 1000)
          angle_inc -= 1000;
        break;
      case 'x':
        if (angle_inc < 1000000000)
          angle_inc += 1000;
        break;
      case 'o':
        if (damper >= 0.05f)
          damper -= 0.05f;
        break;
      case 'p':
        if (damper <= 1.0f - 0.05f)
          damper += 0.05f;
        break;
      case 'l':
        if (damper >= 0.01f)
          damper -= 0.01f;
        break;
      case ';':
        if (damper <= 1.0f - 0.01f)
          damper += 0.01f;
        break;
      case '.':
        if (damper >= 0.001f)
          damper -= 0.001f;
        break;
      case '/':
        if (damper <= 1.0f - 0.001f)
          damper += 0.001f;
        break;
#ifndef RAW_DATA
      default:
        ROM_UARTCharPut(UART0_BASE, '[');
        ROM_UARTCharPut(UART0_BASE, ch);
        ROM_UARTCharPut(UART0_BASE, ']');
#endif
      }
    }

#ifdef MPU_MEASURE
    /*
      When the motor has been spinning at full speed for a bit, start measuring.
      We wait until the motor reaches position 0 before starting to measure.
      And after measuring we stop the motor again at position 0; this provides
      a stable reference position for interpreting the measurements.

      ToDo: This will get rounding errors, could be done better ...
    */
    if (hit_target == 1 && seconds >= next_mpu_seconds &&
        (cur_elec_turns % 4) == 0 && cur_angle < prev_angle)
    {
      /*
        After measurements, we want to stop at the zero point, which is the
        point here, where we start measurements.

        We will first slow down to 0.5 RPS, wait until we get to the point
        stop_mpu_angle, then stop completely within 0.1 seconds. In 0.1
        seconds at 4*0.5 electrical RPS, we should move around
        0.5 * (4*0.5) * 0.1 electrical turns, or 1/10.
      */
      uint32_t diff = (uint64_t)0x100000000/10;
      end_mpu_seconds = seconds + 1.000f;
      stop_mpu_angle = cur_angle;
      if (stop_mpu_angle > diff)
        stop_mpu_eturns = cur_elec_turns;
      else
        stop_mpu_eturns = cur_elec_turns - 1;
      stop_mpu_angle -= diff;
      hit_target = 2;
    }
    if (hit_target == 2)
    {
      if (seconds >= next_mpu_seconds && seconds < end_mpu_seconds)
      {
        /* Measure vibrations for a few seconds. */
        uint8_t buf[6];
        int i;
        next_mpu_seconds += 0.001001f;
        mpu6050_read_accel(buf);
#ifdef RAW_DATA
        for (i = 0; i < 6; ++i)
          ROM_UARTCharPut(UART0_BASE, buf[i]);
#endif
      }
      else if (seconds >= end_mpu_seconds && speed_change_status == SPEED_STABLE)
      {
        /* Slow down. */
        speed_change_to = 4.0f*0.5f*((float)0x100000000/(float)PWM_FREQ);
        speed_change_duration = (uint32_t)(10.0f*PWM_FREQ);
        speed_change_status = SPEED_CHANGE;
        hit_target = 3;
      }
    }
    else if (hit_target == 3 && speed_change_status == SPEED_STABLE &&
             ((cur_elec_turns - stop_mpu_eturns) % 4) == 0)
    {
      /* Continue at slow speed until just before the zero point. */
      last_mpu_delta = cur_angle - stop_mpu_angle;
      hit_target = 4;
    }
    else if (hit_target == 4)
    {
      /* Now stop completely at the zero point. */
      uint32_t delta = cur_angle - stop_mpu_angle;
      if (((cur_elec_turns - stop_mpu_eturns) % 4) == 0 &&
          delta < last_mpu_delta)
      {
        speed_change_to = 0;
        speed_change_duration = (uint32_t)(0.1f*PWM_FREQ);
        speed_change_status = SPEED_CHANGE;
        hit_target = 5;
      }
      else
        last_mpu_delta = delta;
    }
    else if (hit_target == 5)
    {
      if (speed_change_status == SPEED_STABLE)
        damper = 0.05f;
    }
    prev_angle= cur_angle;
#endif
  }
}
