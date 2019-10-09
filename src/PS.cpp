#include "PS.h"

void delayForMicros(unsigned long delay_) {
  delay_ = delay_ < 0 ? 0 : delay_;
  unsigned long delayMicros = micros();
  while (micros() - delayMicros < delay_) {

  }
}

void delayForMillis(unsigned long delay_) {
  delay_ = delay_ < 0 ? 0 : delay_;
  unsigned long delayMillis = millis();
  while (millis() - delayMillis < delay_) {

  }
}

static byte enter_config[] = {0x01, 0x43, 0x00, 0x01, 0x00};
static byte set_mode[] = {0x01, 0x44, 0x00, 0x01, 0x03, 0x00, 0x00, 0x00, 0x00};
static byte exit_config[] = {0x01, 0x43, 0x00, 0x00, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A};

PS::PS() {

}

#ifdef __AVR__
inline void  PS::CLK_SET(void) {        //set high pin CLK
  register uint8_t old_sreg = SREG;
  cli();
  *_clk_oreg |= _clk_mask;
  SREG = old_sreg;
}

inline void  PS::CLK_CLR(void) {        //reset pin CLK
  register uint8_t old_sreg = SREG;
  cli();
  *_clk_oreg &= ~_clk_mask;
  SREG = old_sreg;
}

inline void  PS::CMD_SET(void) {        //set pin CMD
  register uint8_t old_sreg = SREG;
  cli();
  *_cmd_oreg |= _cmd_mask; // SET(*_cmd_oreg,_cmd_mask);
  SREG = old_sreg;
}

inline void  PS::CMD_CLR(void) {        //set pin CMD
  register uint8_t old_sreg = SREG;
  cli();
  *_cmd_oreg &= ~_cmd_mask; // SET(*_cmd_oreg,_cmd_mask);
  SREG = old_sreg;
}

inline void  PS::ATT_SET(void) {        //set pin enable
  register uint8_t old_sreg = SREG;
  cli();
  *_att_oreg |= _att_mask ;
  SREG = old_sreg;
}

inline void PS::ATT_CLR(void) {         //reset pin enable
  register uint8_t old_sreg = SREG;
  cli();
  *_att_oreg &= ~_att_mask;
  SREG = old_sreg;
}

inline bool PS::DAT_CHK(void) {       //get data (bit)
  return (*_dat_ireg & _dat_mask) ? true : false;
}


#else

inline void  PS::CLK_SET(void) {        //set high pin CLK
  digitalWrite(this->clk, HIGH);
}

inline void  PS::CLK_CLR(void) {        //reset pin CLK
  digitalWrite(this->clk, LOW);
}

inline void  PS::CMD_SET(void) {        //set pin CMD
  digitalWrite(this->cmd, HIGH);
}

inline void  PS::CMD_CLR(void) {        //set pin CMD
  digitalWrite(this->cmd, low);
}

inline void  PS::ATT_SET(void) {        //set pin enable
  digitalWrite(this->att, HIGH);
}

inline void PS::ATT_CLR(void) {         //reset pin enable
  digitalWrite(this->att, LOW);
}

inline bool PS::DAT_CHK(void) {       //get data (bit)
  return (digitalRead(dat) == HIGH) ? true : false;
}

#endif

unsigned char PS::_gamepad_shiftinout (char byte) {
  unsigned char tmp = 0;
  for (unsigned char i = 0; i < 8; i++) {
    if (CHK(byte, i)) CMD_SET();
    else CMD_CLR();

    CLK_CLR();
    delayForMicros(CTRL_CLK);

    //if(DAT_CHK()) SET(tmp,i);
    if (DAT_CHK()) bitSet(tmp, i);

    CLK_SET();
#if CTRL_CLK_HIGH
    delayForMicros(CTRL_CLK_HIGH);
#endif
  } // end for

  CMD_SET();
  delayForMicros(CTRL_BYTE_DELAY);
  return tmp;
}

boolean PS::read_gamepad() {
  NOP;
  NOP;

  double temp = millis() - last_read;

  if (temp > 1500) //waited to long
    reconfig_gamepad();

  if (temp < read_delay) //waited too short
    delayForMillis(read_delay - temp);

  char dword[9] = {0x01, 0x42, 0, 0, 0, 0, 0, 0, 0};
  byte dword2[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

  for (byte RetryCnt = 0; RetryCnt < 5; RetryCnt++) {
    CMD_SET();
    CLK_SET();
    ATT_CLR(); // low enable joystick

    NOP;
    NOP;

    for (int i = 0; i < 9; i++) {
      PS2data[i] = _gamepad_shiftinout(dword[i]);
    }

    if (PS2data[1] == 0x79) { //if controller is in full data return mode, get the rest of data
      for (int i = 0; i < 12; i++) {
        PS2data[i + 9] = _gamepad_shiftinout(dword2[i]);
      }
    }

    ATT_SET(); // HI disable joystick
    if ((PS2data[1] & 0xf0) == 0x70)
      break;

    reconfig_gamepad(); // try to get back into Analog mode.
    delayForMillis(read_delay);
  }

  if ((PS2data[1] & 0xf0) != 0x70) {
    if (read_delay < 10)
      read_delay++;   // see if this helps out...
  }

  last_buttons = buttons; //store the previous buttons states

  buttons = *(uint16_t* )(PS2data + 3);

  last_read = millis();
  NOP;
  NOP;

  if ((PS2data[1] & 0xf0) == 0x70)return true;
  else return false;
}

bool PS::config_gamepad(uint8_t clk, uint8_t cmd, uint8_t att, uint8_t dat) {

#ifdef __AVR__
  _clk_mask = digitalPinToBitMask(clk);
  _clk_oreg = portOutputRegister(digitalPinToPort(clk));
  _cmd_mask = digitalPinToBitMask(cmd);
  _cmd_oreg = portOutputRegister(digitalPinToPort(cmd));
  _att_mask = digitalPinToBitMask(att);
  _att_oreg = portOutputRegister(digitalPinToPort(att));
  _dat_mask = digitalPinToBitMask(dat);
  _dat_ireg = portInputRegister(digitalPinToPort(dat));
#endif

  pinMode(clk, OUTPUT); //configure ports
  pinMode(att, OUTPUT);
  pinMode(cmd, OUTPUT);
  pinMode(dat, INPUT);

#if defined(__AVR__)
  digitalWrite(dat, HIGH); //enable pull-up
#endif

  CMD_SET(); ;
  CLK_SET();

  //new error checking. First, read gamepad a few times to see if it's talking

  unsigned long Millis = millis();

  do {
    read_gamepad();
  }
  while (millis() - Millis < 500 && (PS2data[1] != 0x41 && PS2data[1] != 0x73 && PS2data[1] != 0x79));

  NOP;
  NOP;

  if (PS2data[1] != 0x41 && PS2data[1] != 0x73 && PS2data[1] != 0x79) {
    Serial.println("Controller mode not matched or no controller found");
    Serial.print("Expected 0x41, 0x73 or 0x79, but got ");
    Serial.println(PS2data[1], HEX);
    return false;
  }
  return true; //no error if here
}

void PS::sendCommandString(byte string[], byte len) {
  NOP;
  NOP;
  ATT_CLR(); // low enable joystick
  for (int y = 0; y < len; y++)
    _gamepad_shiftinout(string[y]);
  ATT_SET(); //high disable joystick
  delayForMillis(read_delay);                  //wait a few
  NOP;
  NOP;
}

void PS::reconfig_gamepad() {
  NOP;
  NOP;
  sendCommandString(enter_config, sizeof(enter_config));
  sendCommandString(set_mode, sizeof(set_mode));
  sendCommandString(exit_config, sizeof(exit_config));
  NOP;
  NOP;
}

unsigned int PS::ButtonDataByte() {
  return (~buttons);
}

uint8_t PS::analogRX(void) {
  if ((PS2data[1] & 0x70) != 0x70)return 127;
  return PS2data[5];
}
uint8_t PS::analogRY(void) {
  if ((PS2data[1] & 0x70) != 0x70)return 127;
  return PS2data[6];
}

uint8_t PS::analogLX(void) {
  if ((PS2data[1] & 0x70) != 0x70)return 127;
  return PS2data[7];
}
uint8_t PS::analogLY(void) {
  if ((PS2data[1] & 0x70) != 0x70)return 127;
  return PS2data[8];
}

bool PS::pressCross(void) {
  if ((ButtonDataByte() & PSB_CROSS) > 0 )return true;
  else return false;
}

bool PS::pressCircle(void) {
  if ((ButtonDataByte() & PSB_CIRCLE) > 0 )return true;
  else return false;
}

bool PS::pressTriangle(void) {
  if ((ButtonDataByte() & PSB_TRIANGLE) > 0 )return true;
  else return false;
}

bool PS::pressSquare(void) {
  if ((ButtonDataByte() & PSB_SQUARE) > 0 )return true;
  else return false;
}

bool PS::pressR1(void) {
  if ((ButtonDataByte() & PSB_R1) > 0 )return true;
  else return false;
}

bool PS::pressR2(void) {
  if ((ButtonDataByte() & PSB_R2) > 0 )return true;
  else return false;
}

bool PS::pressL1(void) {
  if ((ButtonDataByte() & PSB_L1) > 0 )return true;
  else return false;
}

bool PS::pressL2(void) {
  if ((ButtonDataByte() & PSB_L2) > 0)return true;
  else return false;
}


bool PS::pressLeft(void) {
  if ((ButtonDataByte() & PSB_PAD_LEFT) > 0 )return true;
  else return false;
}

bool PS::pressRight(void) {
  if ((ButtonDataByte() & PSB_PAD_RIGHT) > 0 )return true;
  else return false;
}

bool PS::pressUp(void) {
  if ((ButtonDataByte() & PSB_PAD_UP) > 0 )return true;
  else return false;
}

bool PS::pressDown(void) {
  if ((ButtonDataByte() & PSB_PAD_DOWN) > 0 )return true;
  else return false;
}

bool PS::pressStart(void) {
  if ((ButtonDataByte() & PSB_START) > 0 )return true;
  else return false;
}

bool PS::pressSelect(void) {
  if ((ButtonDataByte() & PSB_START) > 0 )return true;
  else return false;
}
