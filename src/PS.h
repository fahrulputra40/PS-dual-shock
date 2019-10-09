#ifndef _PS_H_
#define _PS_H_
#include "Arduino.h"

#define CTRL_CLK        4
#define CTRL_BYTE_DELAY 3

//#define PS_COM_DEBUG

#define NOP __asm__ __volatile__ ("nop\n\t")


//These are our button constants
#define PSB_SELECT      0x0001
#define PSB_L3          0x0002
#define PSB_R3          0x0004
#define PSB_START       0x0008
#define PSB_PAD_UP      0x0010
#define PSB_PAD_RIGHT   0x0020
#define PSB_PAD_DOWN    0x0040
#define PSB_PAD_LEFT    0x0080
#define PSB_L2          0x0100
#define PSB_R2          0x0200
#define PSB_L1          0x0400
#define PSB_R1          0x0800
#define PSB_GREEN       0x1000
#define PSB_RED         0x2000
#define PSB_BLUE        0x4000
#define PSB_PINK        0x8000
#define PSB_TRIANGLE    0x1000
#define PSB_CIRCLE      0x2000
#define PSB_CROSS       0x4000
#define PSB_SQUARE      0x8000

//These are stick values
#define PSS_RX 5
#define PSS_RY 6
#define PSS_LX 7
#define PSS_LY 8


#define SET(x,y) (x|=(1<<y))          //set bit
#define CLR(x,y) (x&=(~(1<<y)))       //reset bit
#define CHK(x,y) (x & (1<<y))         //get bit
#define TOG(x,y) (x^=(1<<y))

class PS
{
  public:
    PS();
    bool Button(uint16_t);
    unsigned int ButtonDataByte();
    boolean  read_gamepad();
    bool config_gamepad(uint8_t, uint8_t, uint8_t, uint8_t);


    uint8_t analogRX(void);
    uint8_t analogRY(void);
    uint8_t analogLX(void);
    uint8_t analogLY(void);

    bool pressCross(void);// Press cross
    bool pressCircle(void);// Circle press
    bool pressSquare(void);//  Square press
    bool pressTriangle(void);//triagle press

    bool pressR1(void);
    bool pressR2(void);
    bool pressL1(void);
    bool pressL2(void);

    bool pressLeft(void);
    bool pressRight(void);
    bool pressUp(void);
    bool pressDown(void);

    bool pressStart(void);
    bool pressSelect(void);


  private:

    void reconfig_gamepad();

    unsigned char PS2data[21];
    unsigned int buttons;

    inline void CLK_SET(void);
    inline void CLK_CLR(void);
    inline void CMD_SET(void);
    inline void CMD_CLR(void);
    inline void ATT_SET(void);
    inline void ATT_CLR(void);
    inline bool DAT_CHK(void);

    unsigned char _gamepad_shiftinout (char);
    void sendCommandString(byte*, byte);
    unsigned int last_buttons;

#ifdef __AVR__
    uint8_t maskToBitNum(uint8_t);
    uint8_t _clk_mask;
    volatile uint8_t *_clk_oreg;
    uint8_t _cmd_mask;
    volatile uint8_t *_cmd_oreg;
    uint8_t _att_mask;
    volatile uint8_t *_att_oreg;
    uint8_t _dat_mask;
    volatile uint8_t *_dat_ireg;
#else

    uint8_t clk;
    uint8_t cmd;
    uint8_t att;
    uint8_t dat;

#endif



    unsigned long last_read = 0;
    byte read_delay;

};


#endif

