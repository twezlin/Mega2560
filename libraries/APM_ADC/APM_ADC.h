#ifndef APM_ADC_h
#define APM_ADC_h


#define SPI_PRESCALER 256
#define SPI_CLOCK_RATE_2_COUNTER_START_VALUE(f) (256-F_CPU/SPI_PRESCALER/f)

#define bit_set(p,m) ((p) |= (1<<m)) 
#define bit_clear(p,m) ((p) &= ~(1<<m))

#define ADC_CHIP_SELECT 33     //PC4

class APM_ADC_Class
{
  private:
  public:
	APM_ADC_Class();  // Constructor
	void Init();
	int Ch(unsigned char ch_num);
};

extern APM_ADC_Class APM_ADC;

#endif