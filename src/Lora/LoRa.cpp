#include "devconfig.h"
#include "LoRa.h"
#include <RadioLib.h>
#include "radio.h"
#include "driver/board.h"

#if defined ( WIFI_LoRa_32_V3 )
//#include "sx126x\sx1262.h"
#endif

#if defined ( WIFI_LoRa_32_V2 )
#include "sx1276.h"
#endif

#if defined ( WIFI_LoRa_32_V3 )
SX1262 radio = new Module(SS,DIO0,RST_LoRa,BUSY_LoRa);
#else //defined ( WIFI_LoRa_32_V2 )
SX1276 radio = new Module(SS, DIO0, RST_LoRa, DIO1);
#endif
volatile bool rxFlag = false;
char buf [10];
char Readback[50];
bool newvalue=0;
int packetSize = 0;
char frame[50];

#define timetosleep 1000
#define timetowake 20000
static TimerEvent_t sleep1;
static TimerEvent_t wakeup1;

LoRaClass::LoRaClass() :
  _spiSettings(8E6, MSBFIRST, SPI_MODE0),
  _ss(LORA_DEFAULT_SS_PIN), _reset(LORA_DEFAULT_RESET_PIN), _dio0(LORA_DEFAULT_DIO0_PIN),
  _frequency(0),
  _packetIndex(0),
  _implicitHeaderMode(0),
  _onReceive(NULL)
{
  // overide Stream timeout value
  setTimeout(0);
}

// flag to indicate that a packet was sent or received
volatile bool operationDone = false;

void setFlag(void) {
  // we sent or received  packet, set the flag
  operationDone = true;
}


// Can't do Serial or display things here, takes too much time for the interrupt
void rx() {
  rxFlag = true;
}


void LoRaClass::SetTimer(uint32_t timervalue)
{
  Serial.printf("Timer OnSleep %d ms\r\n",timetowake);

  //timetosleep ms later wake up;
  TimerSetValue( &wakeup1, timervalue );
  TimerStart( &wakeup1 );
}


int LoRaClass::begin(long frequency,bool PABOOST)
{
  #if defined( WIFI_LoRa_32_V3 ) 
      SPI.begin(SCK,MISO,MOSI,SS);
      /*int16_t begin(float freq = 434.0, float bw = 125.0, uint8_t sf = 9, uint8_t cr = 7, 
                      uint8_t syncWord = RADIOLIB_SX126X_SYNC_WORD_PRIVATE, int8_t power = 10, 
                      uint16_t preambleLength = 8, float tcxoVoltage = 1.6, bool useRegulatorLDO = false);
        */ 
      int state = radio.begin(LORA_FREQUENCY_V3,LORA_BW_V3,LORA_SF,LORA_CR,RADIOLIB_SX126X_SYNC_WORD_PRIVATE, 
                                LORA_POWER, 8, 1.6, false);

      if (state == RADIOLIB_ERR_NONE)
        Serial.println("Success!!!!");
      else {
        Serial.print("failed, code =");
        Serial.println(state);
        while(true);
      } 

      radio.setFrequency(frequency);
      //radio.setDataRate(LORA_DATARATE);
      radio.setBandwidth(LORA_BW_V3);
      radio.setSpreadingFactor(LORA_SF);
      radio.setOutputPower(TRANSMIT_POWER);

      //teste rff
      // Set the callback function for received packets
      radio.setDio1Action(rx);
      radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_INF);      
      //end teste rff


  #else //( WIFI_LoRa_32_V2 ) 
  int state = radio.begin();

  // set the function that will be called
  // when new packet is received
  radio.setDio0Action(setFlag, RISING);

#if ROUTER
    // send the first packet on this node
    //sprintf(frame,"%s%d",frame1,count1);
    //count1++;
    //transmissionState = radio.startTransmit(frame);
    //transmitFlag = true;
#else
    // start listening for LoRa packets on this node
    state = radio.startReceive();
    if (state == RADIOLIB_ERR_NONE) {
      Serial.println(F("success!"));
    } else {
      Serial.print(F("failed, code "));
      Serial.println(state);
      while (true) { delay(10); }
    }
#endif

  #endif

  return 1;
}

void LoRaClass::end()
{
  // put in sleep mode
  sleep();
  // stop SPI
  SPI.end();
}

int LoRaClass::beginPacket(int implicitHeader)
{
  // put in standby mode
  idle();
  if (implicitHeader) {
    implicitHeaderMode();
  } else {
    explicitHeaderMode();
  }
  // reset FIFO address and paload length
  writeRegister(REG_FIFO_ADDR_PTR, 0);
  writeRegister(REG_PAYLOAD_LENGTH, 0);
  return 1;
}

int LoRaClass::endPacket(bool async)
{
  // put in TX mode
  writeRegister(REG_OPMODE, MODE_LONG_RANGE_MODE | MODE_TX);
  if (async) {
    // grace time is required for the radio
    delayMicroseconds(150);
  } else {
    // wait for TX done
    //V3 -  
#if defined( WIFI_LoRa_32_V3 )    
    while ((readRegister(REG_IRQ_FLAGS) & RADIOLIB_SX126X_IRQ_TX_DONE) == 0) {
      log_i("write3");
      yield();
    }
    // clear IRQ's
    log_i("write4");
    writeRegister(REG_IRQ_FLAGS, RADIOLIB_SX126X_IRQ_TX_DONE);
#else
    while ((readRegister(REG_IRQ_FLAGS) & IRQ_TX_DONE_MASK) == 0) {
      yield();
    }
    // clear IRQ's
    writeRegister(REG_IRQ_FLAGS, IRQ_TX_DONE_MASK);
#endif

  }

  return 1;
}

int LoRaClass::parsePacket(int size)
{
  int packetLength = 0;
  int irqFlags = readRegister(REG_IRQ_FLAGS);

  if (size > 0) {
    implicitHeaderMode();
    writeRegister(REG_PAYLOAD_LENGTH, size & 0xff);
  } else {
    explicitHeaderMode();
  }

  // clear IRQ's
  writeRegister(REG_IRQ_FLAGS, irqFlags);
//RADIOLIB_SX126X_IRQ_RX_DONE
#if defined(WIFI_LoRa_32_V3)
  if ((irqFlags & RADIOLIB_SX126X_IRQ_RX_DONE) && (irqFlags & RADIOLIB_SX126X_IRQ_CRC_ERR) == 0) {

    // received a packet
    _packetIndex = 0;
    // read packet length
    if (_implicitHeaderMode) {
      packetLength = readRegister(REG_PAYLOAD_LENGTH);
    } else {
      packetLength = readRegister(REG_RX_NB_BYTES);
    }
    // set FIFO address to current RX address
    writeRegister(REG_FIFO_ADDR_PTR, readRegister(REG_FIFO_RX_CURRENT_ADDR));
    // put in standby mode
    idle();
  }
  else if (readRegister(REG_OPMODE) != (MODE_LONG_RANGE_MODE | MODE_RX_SINGLE)) {
    // not currently in RX mode
    // reset FIFO address
    writeRegister(REG_FIFO_ADDR_PTR, 0);
    // put in single RX mode
    writeRegister(REG_OPMODE, MODE_LONG_RANGE_MODE | MODE_RX_SINGLE);
  }
#else
  if ((irqFlags & IRQ_RX_DONE_MASK) && (irqFlags & IRQ_PAYLOAD_CRC_ERROR_MASK) == 0) {

    // received a packet
    _packetIndex = 0;
    // read packet length
    if (_implicitHeaderMode) {
      packetLength = readRegister(REG_PAYLOAD_LENGTH);
    } else {
      packetLength = readRegister(REG_RX_NB_BYTES);
    }
    // set FIFO address to current RX address
    writeRegister(REG_FIFO_ADDR_PTR, readRegister(REG_FIFO_RX_CURRENT_ADDR));
    // put in standby mode
    idle();
  }
  else if (readRegister(REG_OPMODE) != (MODE_LONG_RANGE_MODE | MODE_RX_SINGLE)) {
    // not currently in RX mode
    // reset FIFO address
    writeRegister(REG_FIFO_ADDR_PTR, 0);
    // put in single RX mode
    writeRegister(REG_OPMODE, MODE_LONG_RANGE_MODE | MODE_RX_SINGLE);
  }

#endif

  return packetLength;
}

int LoRaClass::packetRssi()
{
	int8_t snr=0;
    int8_t SnrValue = readRegister( 0x19 );
    int16_t rssi = readRegister(REG_PKT_RSSI_VALUE);

	if( SnrValue & 0x80 ) // The SNR sign bit is 1
	{
		// Invert and divide by 4
		snr = ( ( ~SnrValue + 1 ) & 0xFF ) >> 2;
		snr = -snr;
	}
	else
	{
		// Divide by 4
		snr = ( SnrValue & 0xFF ) >> 2;
	}
    if(snr<0)
    {
    	rssi = rssi - (_frequency < 525E6 ? 164 : 157) + ( rssi >> 4 ) + snr;
    }
    else
    {
    	rssi = rssi - (_frequency < 525E6 ? 164 : 157) + ( rssi >> 4 );
    }

  return ( rssi );
}

float LoRaClass::packetSnr()
{
  return (((int8_t)readRegister(REG_PKT_SNR_VALUE) +2) >> 2);
}

size_t LoRaClass::write(uint8_t byte)
{
  return write(&byte, sizeof(byte));
}

size_t LoRaClass::write(const uint8_t *buffer, size_t size)
{
  int currentLength = readRegister(REG_PAYLOAD_LENGTH);
  // check size
  if ((currentLength + size) > MAX_PKT_LENGTH) {
    size = MAX_PKT_LENGTH - currentLength;
  }
  // write data
  for (size_t i = 0; i < size; i++) {
    writeRegister(REG_FIFO, buffer[i]);
  }
  // update length
  writeRegister(REG_PAYLOAD_LENGTH, currentLength + size);
  return size;
}

int LoRaClass::available()
{
  return (readRegister(REG_RX_NB_BYTES) - _packetIndex);
}

int LoRaClass::read()
{
  if (!available()) {
  	return -1; 
	}
  _packetIndex++;
  return readRegister(REG_FIFO);
}

int LoRaClass::peek()
{
  if (!available()) {
  	return -1; 
	}
  // store current FIFO address
  int currentAddress = readRegister(REG_FIFO_ADDR_PTR);
  // read
  uint8_t b = readRegister(REG_FIFO);
  // restore FIFO address
  writeRegister(REG_FIFO_ADDR_PTR, currentAddress);
  return b;
}

void LoRaClass::flush()
{
}

void LoRaClass::onReceive(void(*callback)(int))
{
  _onReceive = callback;

  if (callback) {
    writeRegister(REG_DIOMAPPING1, 0x00);
    attachInterrupt(digitalPinToInterrupt(_dio0), LoRaClass::onDio0Rise, RISING);
//    attachInterrupt(digitalPinToInterrupt(_dio0), LoRaClass::onDio0Rise, RISING);
  } else {
    detachInterrupt(digitalPinToInterrupt(_dio0));
  }
}

void LoRaClass::receive(int size)
{
  if (size > 0) {
    implicitHeaderMode();
    writeRegister(REG_PAYLOAD_LENGTH, size & 0xff);
  } else {
    explicitHeaderMode();
  }

  writeRegister(REG_OPMODE, MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS);
}

void LoRaClass::idle()
{
  writeRegister(REG_OPMODE, MODE_LONG_RANGE_MODE | MODE_STDBY);
}

void LoRaClass::sleep()
{
  writeRegister(REG_OPMODE, MODE_LONG_RANGE_MODE | MODE_SLEEP);
}

void LoRaClass::setTxPower(int8_t power, int8_t outputPin)
{
	  uint8_t paConfig = 0;
	  uint8_t paDac = 0;

	  paConfig = readRegister( REG_PACONFIG );
	  paDac = readRegister( REG_PADAC );

	  paConfig = ( paConfig & RF_PACONFIG_PASELECT_MASK ) | outputPin;
	  paConfig = ( paConfig & RF_PACONFIG_MAX_POWER_MASK ) | 0x70;

	  if( ( paConfig & RF_PACONFIG_PASELECT_PABOOST ) == RF_PACONFIG_PASELECT_PABOOST )
	  {
	    if( power > 17 )
	    {
	      paDac = ( paDac & RF_PADAC_20DBM_MASK ) | RF_PADAC_20DBM_ON;
	    }
	    else
	    {
	      paDac = ( paDac & RF_PADAC_20DBM_MASK ) | RF_PADAC_20DBM_OFF;
	    }
	    if( ( paDac & RF_PADAC_20DBM_ON ) == RF_PADAC_20DBM_ON )
	    {
	      if( power < 5 )
	      {
	        power = 5;
	      }
	      if( power > 20 )
	      {
	        power = 20;
	      }
	      paConfig = ( paConfig & RF_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( power - 5 ) & 0x0F );
	    }
	    else
	    {
	      if( power < 2 )
	      {
	        power = 2;
	      }
	      if( power > 17 )
	      {
	        power = 17;
	      }
	      paConfig = ( paConfig & RF_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( power - 2 ) & 0x0F );
	    }
	  }
	  else
	  {
	    if( power < -1 )
	    {
	      power = -1;
	    }
	    if( power > 14 )
	    {
	      power = 14;
	    }

	    paConfig = ( paConfig & RF_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( power + 1 ) & 0x0F );
	  }
	  writeRegister( REG_PACONFIG, paConfig );
	  writeRegister( REG_PADAC, paDac );
}

void LoRaClass::setTxPowerMax(int level)
{
	if (level < 5)		{
		level = 5;
	}
	else if(level > 20)	{
		level = 20;
	}
	writeRegister(REG_OCP,0x3f);
	writeRegister(REG_PADAC,0x87);//Open PA_BOOST
	writeRegister(REG_PACONFIG, RF_PACONFIG_PASELECT_PABOOST | (level - 5));
}

void LoRaClass::setFrequency(long frequency)
{
  _frequency = frequency;

  uint64_t frf = ((uint64_t)frequency << 19) / 32000000;
  writeRegister(REG_FRFMSB, (uint8_t)(frf >> 16));
  writeRegister(REG_FRFMID, (uint8_t)(frf >> 8));
  writeRegister(REG_FRFLSB, (uint8_t)(frf >> 0));
}

void LoRaClass::setSpreadingFactor(int sf)
{
  if (sf < 6) {
  	sf = 6; 
	}
  else if (sf > 12) {
  	sf = 12; 
  	}
  if (sf == 6) {
    writeRegister(REG_DETECTION_OPTIMIZE, 0xc5);
    writeRegister(REG_DETECTION_THRESHOLD, 0x0c);
  } else {
    writeRegister(REG_DETECTION_OPTIMIZE, 0xc3);
    writeRegister(REG_DETECTION_THRESHOLD, 0x0a);
  }

  writeRegister(REG_MODEM_CONFIG_2, (readRegister(REG_MODEM_CONFIG_2) & 0x0f) | ((sf << 4) & 0xf0));

}

void LoRaClass::setSignalBandwidth(long sbw)
{
  int bw;

  if (sbw <= 7.8E3) { bw = 0; }
  else if (sbw <= 10.4E3) { bw = 1; }
  else if (sbw <= 15.6E3) { bw = 2; }
  else if (sbw <= 20.8E3) { bw = 3; }
  else if (sbw <= 31.25E3) { bw = 4; }
  else if (sbw <= 41.7E3) { bw = 5; }
  else if (sbw <= 62.5E3) { bw = 6; }
  else if (sbw <= 125E3) { bw = 7; }
  else if (sbw <= 250E3) { bw = 8; }
  else /*if (sbw <= 250E3)*/ { bw = 9; }

  writeRegister(REG_MODEM_CONFIG_1,(readRegister(REG_MODEM_CONFIG_1) & 0x0f) | (bw << 4));
}

void LoRaClass::setCodingRate4(int denominator)
{
  if (denominator < 5) {
    denominator = 5;
  } else if (denominator > 8) {
    denominator = 8;
  }
  int cr = denominator - 4;

  writeRegister(REG_MODEM_CONFIG_1, (readRegister(REG_MODEM_CONFIG_1) & 0xf1) | (cr << 1));
}

void LoRaClass::setPreambleLength(long length)
{
  writeRegister(REG_PREAMBLE_MSB, (uint8_t)(length >> 8));
  writeRegister(REG_PREAMBLE_LSB, (uint8_t)(length >> 0));
}

void LoRaClass::setSyncWord(int sw)
{
  writeRegister(REG_SYNC_WORD, sw);
}

void LoRaClass::enableCrc()
{
  writeRegister(REG_MODEM_CONFIG_2, readRegister(REG_MODEM_CONFIG_2) | 0x04);
}

void LoRaClass::disableCrc()
{
  writeRegister(REG_MODEM_CONFIG_2, readRegister(REG_MODEM_CONFIG_2) & 0xfb);
}

void LoRaClass::enableTxInvertIQ()
{
  writeRegister( REG_LR_INVERTIQ, ( ( readRegister( REG_LR_INVERTIQ ) & RFLR_INVERTIQ_TX_MASK & RFLR_INVERTIQ_RX_MASK ) | RFLR_INVERTIQ_RX_OFF | RFLR_INVERTIQ_TX_ON ) );
  writeRegister( REG_LR_INVERTIQ2, RFLR_INVERTIQ2_ON );
}

void LoRaClass::enableRxInvertIQ()
{
  writeRegister( REG_LR_INVERTIQ, ( ( readRegister( REG_LR_INVERTIQ ) & RFLR_INVERTIQ_TX_MASK & RFLR_INVERTIQ_RX_MASK ) | RFLR_INVERTIQ_RX_ON | RFLR_INVERTIQ_TX_OFF ) );
  writeRegister( REG_LR_INVERTIQ2, RFLR_INVERTIQ2_ON );
}

void LoRaClass::disableInvertIQ()
{
  writeRegister( REG_LR_INVERTIQ, ( ( readRegister( REG_LR_INVERTIQ ) & RFLR_INVERTIQ_TX_MASK & RFLR_INVERTIQ_RX_MASK ) | RFLR_INVERTIQ_RX_OFF | RFLR_INVERTIQ_TX_OFF ) );
  writeRegister( REG_LR_INVERTIQ2, RFLR_INVERTIQ2_OFF );
}

void LoRaClass::enableInvertIQ()
{
  writeRegister( REG_LR_INVERTIQ, ( ( readRegister( REG_LR_INVERTIQ ) & RFLR_INVERTIQ_TX_MASK & RFLR_INVERTIQ_RX_MASK ) | RFLR_INVERTIQ_RX_ON | RFLR_INVERTIQ_TX_ON ) );
  writeRegister( REG_LR_INVERTIQ2, RFLR_INVERTIQ2_ON );
}

byte LoRaClass::random()
{
  return readRegister(REG_RSSI_WIDEBAND);
}

void LoRaClass::setPins(int ss, int reset, int dio0)
{
  _ss = ss;
  _reset = reset;
  _dio0 = dio0;
}

void LoRaClass::setSPIFrequency(uint32_t frequency)
{
  _spiSettings = SPISettings(frequency, MSBFIRST, SPI_MODE0);
}

void LoRaClass::dumpRegisters(Stream& out)
{
  for (int i = 0; i < 128; i++) {
    out.print("0x");
    out.print(i, HEX);
    out.print(": 0x");
    out.println(readRegister(i), HEX);
  }
}

void LoRaClass::explicitHeaderMode()
{
  _implicitHeaderMode = 0;
  writeRegister(REG_MODEM_CONFIG_1, readRegister(REG_MODEM_CONFIG_1) & 0xfe);
}

void LoRaClass::implicitHeaderMode()
{
  _implicitHeaderMode = 1;
  writeRegister(REG_MODEM_CONFIG_1, readRegister(REG_MODEM_CONFIG_1) | 0x01);
}

void LoRaClass::handleDio0Rise()
{
  int irqFlags = readRegister(REG_IRQ_FLAGS);
  // clear IRQ's
  writeRegister(REG_IRQ_FLAGS, irqFlags);
#if defined(WIFI_LoRa_32_V3) 
  if ((irqFlags & RADIOLIB_SX126X_IRQ_CRC_ERR) == 0) {
#else
  if ((irqFlags & IRQ_PAYLOAD_CRC_ERROR_MASK) == 0) {
#endif    
    // received a packet
    _packetIndex = 0;
    // read packet length
    int packetLength = _implicitHeaderMode ? readRegister(REG_PAYLOAD_LENGTH) : readRegister(REG_RX_NB_BYTES);
    // set FIFO address to current RX address
    writeRegister(REG_FIFO_ADDR_PTR, readRegister(REG_FIFO_RX_CURRENT_ADDR));
    if (_onReceive) { _onReceive(packetLength); }
    // reset FIFO address
    writeRegister(REG_FIFO_ADDR_PTR, 0);
  }
}

uint8_t LoRaClass::readRegister(uint8_t address)
{
  return singleTransfer(address & 0x7f, 0x00);
}

void LoRaClass::writeRegister(uint8_t address, uint8_t value)
{
  singleTransfer(address | 0x80, value);
}

uint8_t LoRaClass::singleTransfer(uint8_t address, uint8_t value)
{
  uint8_t response;
  digitalWrite(_ss, LOW);
  SPI.beginTransaction(_spiSettings);
  SPI.transfer(address);
  response = SPI.transfer(value);
  SPI.endTransaction();
  digitalWrite(_ss, HIGH);
  return response;
}

void LoRaClass::onDio0Rise()
{
  LoRa.handleDio0Rise();
}

LoRaClass LoRa;

#if 0
uint8_t calc_crc (String data,size_t len,char *pframe) {
   uint8_t i;
   char buf[30];

   for (i=0;i<len;i++){
     buf[0] = (char) data.c_str();
   }
   log_i("len=%d %2x %2x %2x %2x",len, buf[0],buf[1],buf[2],buf[3]);
   return len;
}
#endif

void LoRaClass::SendFrame(String data,size_t len)
{
  String var1;
  uint8_t len1=0;
  char buf[10];

#if  defined ( WIFI_LoRa_32_V3 )
  //int16_t SX126x::transmit(uint8_t* data, size_t len, uint8_t addr) {
  int16_t ret = radio.transmit(data);
  Serial.print("transmited ");
  Serial.print(var1);
  Serial.print(" ret=");
  Serial.println(ret);

#else  //WIFI_LoRa_32_V2

  // send packet
  //LoRa.beginPacket();
  /*
  * LoRa.setTxPower(txPower,RFOUT_pin);
  * txPower -- 0 ~ 20
  * RFOUT_pin could be RF_PACONFIG_PASELECT_PABOOST or RF_PACONFIG_PASELECT_RFO
  *   - RF_PACONFIG_PASELECT_PABOOST -- LoRa single output via PABOOST, maximum output 20dBm
  *   - RF_PACONFIG_PASELECT_RFO     -- LoRa single output via RFO_HF / RFO_LF, maximum output 14dBm
  */
  //LoRa.setTxPower(14,RF_PACONFIG_PASELECT_PABOOST);
  //LoRa.print(data);
  //LoRa.endPacket();

  // send another one
  //disableCrc();
  //enableCrc();
  //len1 = calc_crc(data,len,frame);
  //transmissionState = radio.startTransmit(data);

  // the previous operation was transmission, listen for response
  // print the result
 // if (transmissionState == RADIOLIB_ERR_NONE) {
 //   log_i("transmission finished!");
 // } else {
 //   log_e("failed, code=%d ",transmissionState);
 // }


#endif

}
uint32_t getRssi(void) {
  uint32_t retRssi=0;

 #if  defined ( WIFI_LoRa_32_V3 ) 
     retRssi = (uint32_t) radio.getRSSI(1);
 #else
     retRssi = (uint32_t) LoRa.packetRssi();
 #endif
 
 return retRssi;
}

uint8_t LoRaClass::ReceiveFrame(char *pframe) {

 #if  defined ( WIFI_LoRa_32_V3 ) 
  String str;

  //radio.clearDio1Action();
  packetSize = 0;

  if (rxFlag) {
    rxFlag = false;
    int state = radio.receive(str);

    if (state == RADIOLIB_ERR_NONE) {
      // Packet received successfully
      packetSize = str.length();
      strcpy(pframe,str.c_str());

      log_i("Received packet [%d] rssi=%d",packetSize,getRssi());
    }
    //else {
      // Some other error occurred (excluding timeout and CRC mismatch)
    //  log_e("Failed to receive packet!!!! code=%d",state);
    //}

    //radio.setDio1Action(rx);
    radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_INF);    
  }


#else

#if 0
  packetSize = LoRa.parsePacket();
   Serial.print("Rx Packetsize=");
    Serial.println(packetSize);
  if (packetSize) {
    // read packet
    while (LoRa.available()) {
	   sprintf(Readback+strlen(Readback),"%c",(char)LoRa.read());
    }
    memcpy(pframe,Readback,sizeof(packetSize));
   
#if DISPLAY_ENABLE 
    //Heltec.DisplayShow(packetSize,Readback);
    //DisplayShow1(packetSize);
 
#endif	
    memset(Readback,0,50);
    
    // print RSSI of packet
    // Serial.print(" with RSSI ");
    // Serial.println(LoRa.packetRssi());
  }
#else
  String str;

  //disableCrc();
  enableCrc();
  int state = radio.readData(str);
  if (state == RADIOLIB_ERR_NONE) {
    // packet was successfully received
    log_i("Received packet len=%d",str.length());
    Serial.println(str);
  }
#endif
#endif


  return packetSize;
}