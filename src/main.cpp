#include <Arduino.h>

#include <SPI.h>

#define LOW_SPEED 2000000 // 在CLKPLL完成前需要使用低速SPI，只有在加电启动和睡眠时需要，CLKPLL锁相完成后需要马上切换为高速状态，否则会不正常
#define HIGH_SPEED 20000000

SPIClass *vspi = NULL;

void dw1000isr();

// 定义DW1000控制IO
const uint8_t PIN_RST = 4;  // 复位
const uint8_t PIN_IRQ = 13; // 中断

// 定义SPI接口
const uint8_t PIN_SS = SS; // SPI片选

boolean flag = false;

#define BIT_0 (1 << 0)

void setBit(byte data[], uint16_t dataLen, uint16_t bitIdx, boolean isSet)
{
  uint16_t index;
  uint8_t shift;

  // 计算字节位置
  index = bitIdx / 8;
  if (index >= dataLen)
  {
    return; // 字节索引需要在data内，超出是错误
  }

  byte *targetByte = &data[index];

  shift = bitIdx % 8; // 求模，表示在targetByte中的位置
  if (isSet)
  {
    bitSet(*targetByte, shift);
  }
  else
  {
    bitClear(*targetByte, shift);
  }
}

boolean getBit(byte data[], uint16_t len, uint16_t bitIdx)
{
  uint16_t idx;
  uint8_t shift;

  idx = bitIdx / 8;
  if (idx >= len)
  {
    return false;
  }

  byte targetByte = data[idx];
  shift = bitIdx % 8;

  return bitRead(targetByte, shift);
}

void readBytesFromRegister(byte cmd, uint16_t offset, byte data[], uint16_t data_size)
{
  byte header[3];
  uint8_t headerLen = 1;

  if (offset == 0xFF)
  {
    header[0] = 0x00 | cmd;
  }
  else
  {
    header[0] = 0x40 | cmd;
    if (offset < 128)
    {
      header[1] = (byte)offset;
      ++headerLen;
    }
    else
    {
      header[1] = 0x80 | (byte)offset;
      header[2] = (byte)(offset >> 7);
      headerLen += 2;
    }
  }

  digitalWrite(SS, LOW);
  delayMicroseconds(5);

  for (auto i = 0; i < headerLen; ++i)
  {
    vspi->transfer(header[i]);
  }

  for (auto i = 0; i < data_size; ++i)
  {
    data[i] = vspi->transfer(0x00);
  }

  delayMicroseconds(5);

  digitalWrite(SS, HIGH);
}

void writeBytesToRegister(byte cmd, uint16_t offset, byte data[], uint16_t data_size)
{
  byte header[3];
  uint8_t headerLen = 1;
  if (offset == 0xFF)
  {
    header[0] = 0x80 | cmd;
  }
  else
  {
    header[0] = 0xC0 | cmd;
    if (offset < 128)
    {
      header[1] = (byte)offset;
      ++headerLen;
    }
    else
    {
      header[1] = 0x80 | (byte)offset;
      header[2] = (byte)(offset >> 7);
      headerLen += 2;
    }
  }

  // 写入寄存器
  digitalWrite(SS, LOW);

  delayMicroseconds(5);
  for (auto i = 0; i < headerLen; ++i)
  {
    vspi->transfer(header[i]);
  }
  for (auto i = 0; i < data_size; ++i)
  {
    vspi->transfer(data[i]);
  }
  delayMicroseconds(5);

  digitalWrite(SS, HIGH);
}

void getDeviceIdentifier(char msgBuffer[])
{
  byte data[4];

  readBytesFromRegister(0x00, 0xFF, data, 4);

  sprintf(msgBuffer, "Device ID: %02X -model: %d, version: %d, revision: %d",
          (uint16_t)((data[3] << 8) | data[2]), data[1], (data[0] >> 4) & 0x0F, data[0] & 0x0F);
}

void getExtendUniqueId(char msgBuffer[])
{
  byte data[8];
  readBytesFromRegister(0x01, 0xFF, data, 8);
  sprintf(msgBuffer, "Unique ID: %X:%X:%X:%X:%X:%X:%X:%X",
          data[7], data[6], data[5], data[4], data[3], data[2], data[1], data[0]);
}

void getPanIdentifierAndShortAddress(char msgBuffer[])
{
  byte data[4];
  readBytesFromRegister(0x03, 0xFF, data, 4);
  sprintf(msgBuffer, "Network ID & Device Address: PAN ID: %02X -Short Addr: %02X",
          (uint16_t)((data[3] << 8) | data[2]), (uint16_t)((data[1] << 8) | data[0]));
}

// 读取寄存器0x1F——Channel Controll和0x08——Transmit Frame Control
void getDeviceMode(char msgBuffer[])
{
  uint16_t drIdx;
  uint16_t dataRate;
  uint8_t prf;   // Pluse Repetition Frequency
  uint16_t plen; // preamble length
  uint8_t pcode; // Preamble Code
  uint8_t ch;    // channel

  byte chan_ctrl[4]; // 0x1F，通道控制
  byte tx_fctrl[5];  // 0x08，发送振控制

  readBytesFromRegister(0x1F, 0XFF, chan_ctrl, 4);
  readBytesFromRegister(0x08, 0xFF, tx_fctrl, 5);

  // 计算波特率
  drIdx = (uint16_t)(tx_fctrl[1] >> 5 & 0x03);
  switch (drIdx)
  {
  case 0x00:
    dataRate = 110;
    break;
  case 0x01:
    dataRate = 850;
    break;
  case 0x02:
    dataRate = 6800;
    break;
    ;
  default:
    return;
  }

  // 计算PRF（Pluse Repetition Frequency）脉冲重复频率，只能有16MHz和64MHz
  // PRF是Preamble和SFD的频率，在UWB的传输帧中有两种调制方式，Preamble + SFD使用PPM调制，
  // 在发送完SFD后（同时激发计数器保存时间戳，这个点命名为RMARK）转换为BPSK调制发送PHR和Data
  // 由于BPSK无法从中间解码，并且UWB功率谱密度在白噪音以下，所以CCA只能对Preamble检测，当
  // 使用BPSK调制后，UWB芯片无法对空中信号进行解码，在这个阶段也就无法实现CCA
  prf = (uint8_t)(chan_ctrl[2] >> 2 & 0x03);
  if (prf == 0x01)
  {
    prf = 16;
  }
  else if (prf == 0x02)
  {
    prf = 64;
  }
  else
  {
    return;
  }

  // 计算Preamble长度，Preamble使用四位表示，分别是：两位TXPSR(Bit19, Bit18)，两位PE（Bit21，Bit20）
  plen = (uint16_t)((tx_fctrl[2] >> 2) & 0x0F);
  Serial.println(tx_fctrl[2] >> 2 & 0x0F);

  switch (plen)
  {
  case 0x01:
    plen = 64;
    break;
  case 0x05:
    plen = 128;
    break;
  case 0x09:
    plen = 256;
    break;
  case 0x0D:
    plen = 512;
    break;
  case 0x02:
    plen = 1024;
    break;
  case 0x06:
    plen = 1536;
    break;
  case 0x0A:
    plen = 2048;
    break;
  case 0x03:
    plen = 4096;
    break;
  default:
    break;
  }

  // 计算Channel
  ch = (uint8_t)(chan_ctrl[0] & 0x0F);

  // 前导码
  pcode = (uint8_t)(chan_ctrl[3] >> 3 & 0x1F);

  sprintf(msgBuffer, "Data rate: %u kb/s, PRF: %u MHz, Preamble Length: %u Symbols, Channel: #%u, Preamble Code: #%u",
          dataRate, prf, plen, ch, pcode);
}

void setPanIdentifierAndShortAddress(uint16_t panId, uint16_t shortAddress)
{
  byte data[4]; // PAN ID and short Address
  // 寄存器0x03，数据长度4
  data[3] = (byte)((panId >> 8) & 0xFF);
  data[2] = (byte)(panId & 0xFF);
  data[1] = (byte)((shortAddress >> 8) & 0xFF);
  data[0] = (byte)(shortAddress & 0x0F);

  writeBytesToRegister(0x03, 0xFF, data, 4);
}

void startTransmit(byte data[])
{
  byte sysctrl[4]; // 写入寄存器0x0D——SYS_CTRL的控制字，
  memset(sysctrl, 0, sizeof(sysctrl));

  setBit(sysctrl, 4, 0, true); // bit0：SFCST位，抑制自动FCS发送，该位与TXSTRT（Transmit Start）位共同决定是否自动计算
                               // FCS（Frame-Check-Sequence）字节
  setBit(sysctrl, 4, 1, true); // bit1：TXSTART_BIT，该位控制DW1000开始发送，当DW1000在Idle模式，该位使DW1000立即发送

  writeBytesToRegister(0x0D, 0xFF, sysctrl, 4);
}

void setInterruptPolarity(boolean val)
{
  byte syscfg[4];
  memset(syscfg, 0, sizeof(syscfg));

  setBit(syscfg, 4, 9, val); // 寄存器0x04：System Configuration
                             // bit9：HIRQ_POL，默认置位，控制DW1000中断管脚极性，1：高电平；0:低电平
  writeBytesToRegister(0x04, 0xFF, syscfg, 4);
}

void enableInterrupt()
{
  // 寄存器0x0E——SYS_MASK：System Event Mask Register
  byte eventmask[4];             // 4字节
  setBit(eventmask, 4, 7, true); // MTXFRS：bit7，发送帧发送事件
  writeBytesToRegister(0x0E, 0xFF, eventmask, 4);
}

EventGroupHandle_t eventGroup;

void setup()
{
  Serial.begin(9600);
  const char *pmsg = "DW1000 setup...";

  Serial.println(pmsg);

  eventGroup = xEventGroupCreate();
  if (NULL == eventGroup)
  {
    Serial.println("xEventGroupCreate has failed");
  }

  pinMode(PIN_RST, OUTPUT);
  digitalWrite(PIN_RST, LOW);
  delayMicroseconds(10);
  pinMode(PIN_RST, INPUT);   // Why ?  wrong typing? 

  pinMode(PIN_IRQ, INPUT);

  attachInterrupt(PIN_IRQ, dw1000isr, RISING);

  // 判断SPI使用的GPIO
  Serial.print("MOSI: ");
  Serial.print(MOSI);
  Serial.print("MISO: ");
  Serial.print(MISO);
  Serial.print("SCK: ");
  Serial.print(SCK);
  Serial.print("SS: ");
  Serial.print(SS);

  vspi = new SPIClass(VSPI);

  vspi->begin();
  vspi->begin(SCK, MISO, MOSI, SS);
  pinMode(SS, OUTPUT);

  // vspi->setFrequency(LOW_SPEED);
  vspi->setBitOrder(MSBFIRST);
  vspi->setDataMode(SPI_MODE0);
  vspi->setFrequency(HIGH_SPEED); // 必须使用20MHz速率，否则会出现错误读取，例如：使用2M时会把Dev ID错误读取为BC95

  // 写入PAN ID：0A，Short ID：05
  setPanIdentifierAndShortAddress(0x0A, 0x05);

  char buf[128];

  // 读取寄存器0x00——Device Identifier
  memset(buf, 0, sizeof(buf));
  getDeviceIdentifier(buf);
  Serial.println(buf);

  // 读取寄存器0x01——Extended Unique Identifier
  memset(buf, 0, sizeof(buf));
  getExtendUniqueId(buf);
  Serial.println(buf);

  // 读取寄存器0x03——PAN identifier and Short Address
  memset(buf, 0, sizeof(buf));
  getPanIdentifierAndShortAddress(buf);
  Serial.println(buf);

  // 读取寄存器0x1F——Channel Controll和0x08——Transmit Frame Control
  memset(buf, 0, sizeof(buf));
  getDeviceMode(buf);
  Serial.println(buf);

  // 设置中断极性
  setInterruptPolarity(true); // 设置中断管脚高电平有效，需要与attachInterrupt配置一致
  // 使能发送完成中断
  enableInterrupt();
}

uint8_t i = 0;

void loop()
{
  char buf[128];
  byte sysStatus[4];
  boolean transmitSent = false;

  // 发送数据
  const char *msg = "This is from DW1000";
  byte data[strlen(msg) + 2];
  memset(data, 0, sizeof(data));
  memcpy(data, msg, strlen(msg));
  data[strlen(msg) + 1] = i;

  startTransmit(data);

  EventBits_t ledBits = xEventGroupWaitBits(eventGroup, BIT_0, pdTRUE, pdFALSE, 5000);
  if (ledBits && BIT_0)
  {
    Serial.println("get dw1000 irq");
    // 读取状态寄存器数据，并且把发送中断置位
    readBytesFromRegister(0x0F, 0xFF, sysStatus, 4);
    // memset(buf, 0, sizeof(buf));
    // sprintf(buf, "sysStatus: %X, %X, %X, %X", sysStatus[3], sysStatus[2], sysStatus[1], sysStatus[0]);
    // Serial.println(buf);

    // 判断是否是发送完成中断
    transmitSent = (sysStatus, 4, 7); // Bit7：TXFRS_BIT，发送完成标识位
    if (transmitSent)
    {
      setBit(sysStatus, 4, 3, true);
      setBit(sysStatus, 4, 4, true);
      setBit(sysStatus, 4, 5, true);
      setBit(sysStatus, 4, 6, true);
      setBit(sysStatus, 4, 7, true);

      writeBytesToRegister(0x0F, 0xFF, sysStatus, 4);
    }
  }

  ++i;
  Serial.println(i);
  delay(5000);
}

void dw1000isr() // PIN_IRQ=2，使用GPIO作为中断输入
{
  // 使用freeRTOS的事件通知外部
  BaseType_t xHigherPriorityTaskWoken, xResult;
  xHigherPriorityTaskWoken = pdFALSE;
  xResult = xEventGroupSetBitsFromISR(eventGroup, BIT_0, &xHigherPriorityTaskWoken);

  if (xResult != pdFAIL)
  {
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }
  flag = !flag;
}
