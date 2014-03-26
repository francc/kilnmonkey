#define	FAULT_OPEN	10000

class	MAX31855
{
  public:
    MAX31855(uint8_t SO, uint8_t CS, uint8_t SCK);
    double	readThermocouple();
    double	readJunction();

  private:
    uint8_t so;
    uint8_t cs;
    uint8_t sck;		
    uint32_t readData();
};


MAX31855::MAX31855(uint8_t SO, uint8_t CS, uint8_t SCK)
{
  so = SO;
  cs = CS;
  sck = SCK;
	
  pinMode(so, INPUT);
  pinMode(cs, OUTPUT);
  pinMode(sck, OUTPUT);

  digitalWrite(cs, HIGH);
  digitalWrite(sck, LOW);
}

double	MAX31855::readThermocouple()
{
  uint32_t data = readData();
	
  if ((data & 0x00010000) && (data & 0x00000007) == 0x01)
    return FAULT_OPEN;
  else
    return (data >> 18) * 0.25;
}

double	MAX31855::readJunction()
{
  return ((readData() >> 4) & 0x00000FFF)* 0.0625;
}

uint32_t MAX31855::readData()
{
  uint8_t bits;
  uint32_t data = 0;
  
  digitalWrite(cs, LOW);	
  for(bits = 32; bits > 0; bits--) {
    digitalWrite(sck, HIGH);
    if(digitalRead(so))
      data |= ((uint32_t)1 << (bits - 1));
    digitalWrite(sck, LOW);
  }
  digitalWrite(cs, HIGH);
  return data;
}

