/**************************************************************************/
/*! 
    @file     si5351.cpp
    @author   Przemek Sadowski SQ9NJE
	@license  BSD (see license.txt)
	
	@section  HISTORY

    v1.0  - First release
*/
/**************************************************************************/

#include "si5351.h"


/**************************************************************************/
/*! 
    @brief  Performs basic initialisation of SI5351
*/
/**************************************************************************/
void si5351::begin()
{
	write_register(0x09, 0xFF);		// disable OEB pin
	write_register(0x0F, 0x00);		// XTAL as input for both PLLs
	write_register(0xB7, 0xC0);		// 10pF	XTAL load capacitance
	write_register(0x03, 0xFF);		// disable all outputs
}

/**************************************************************************/
/*! 
    @brief  Writes a value to a single register

    @param[in]	addr
    			Register address. Some constants defined in si5351.h
    @param[in]	value
    			Value to write.
*/
/**************************************************************************/
void si5351::write_register(uint8_t addr, uint8_t value)
{
	Wire.beginTransmission(_addr);
	Wire.write(addr);
	Wire.write(value);
	Wire.endTransmission();
}

/**************************************************************************/
/*! 
    @brief  Writes a continuous block of registers

    @param[in]	addr
    			Address of the first register to write.
    @param[in]	value
    			Pointer to the data block to transfer.
    @param[in]	len
    			Number of registers to write
*/
/**************************************************************************/
void si5351::write_block(uint8_t addr, uint8_t* value, uint8_t len)
{
	uint8_t i;
	Wire.beginTransmission(_addr);
	Wire.write(addr);
	for(i=0;i<len;i++)
		Wire.write(value[i]);
	Wire.endTransmission();
}

/**************************************************************************/
/*! 
    @brief  Reads a single register

    @param[in]	addr
    			Register address. Some constants defined in si5351.h
	@return		Register value.
*/
/**************************************************************************/
uint8_t si5351::read_register(uint8_t addr)
{
	Wire.beginTransmission(_addr);
	Wire.write(addr);
	Wire.requestFrom(_addr, (uint8_t)1);

	return Wire.read();
}

/**************************************************************************/
/*! 
    @brief  Sets the x-tal oscillator frequency

    @param[in]	xtal
    			X-tal frequency in Hz
*/
/**************************************************************************/
void si5351::set_xtal(uint32_t xtal)
{
	_xtal = xtal;
}

/**************************************************************************/
/*! 
    @brief  Sets the x-tal oscillator frequency

    @param[in]	xtal
    			X-tal frequency in Hz
    @param[in]	ppm
    			Calibration factor in ppm
*/
/**************************************************************************/
void si5351::set_xtal(uint32_t xtal, int ppm)
{
	_xtal = xtal;
	_xtal += xtal/1000000 * ppm;
}

/**************************************************************************/
/*! 
    @brief  Sets the PLL multiplication factor. Only integer mode supported

    @param[in]	pll
    			Which PLL to configure. Accepts PLL_A or PLL_B
    @param[in]	multiplier
    			Multiplication factor. No range checking for now
*/
/**************************************************************************/
void si5351::pll_integer_config(uint8_t pll, uint8_t multiplier)
{
	uint8_t config[] = {0x00, 0x01, 0x00, 0x0D, 0x00, 0x00, 0x00, 0x00};
	uint8_t t, r;
	uint32_t p1;

	p1 = 128*multiplier - 512;
	config[2] = (p1 & 0x00030000) >> 16;
	config[3] = (p1 & 0x0000FF00) >> 8;
	config[4] = (p1 & 0x000000FF);

	if(pll == PLL_A)
	{
		_mult_a = multiplier;
		r = 0x16;
	}
	else {
		_mult_b = multiplier;
		r = 0x17;
	}
	t = read_register(r);
	write_register(r, t | (1 << FBx_INT));	// integer mode for PLL A

	write_block(pll, config, 8);				// write MultiSynth config
	write_register(0xB1, 0xA0);				// reset both PLLs
}

/**************************************************************************/
/*! 
    @brief  Sets the PLL multiplication factor.

    @param[in]	pll
    			Which PLL to configure. Accepts PLL_A or PLL_B
    @param[in]	multiplier
    			Multiplication factor. No range checking for now
    @param[in]  num
			Numerator of fractional part
    @param[in]  denom
			Denominator of fractional part
*/
/**************************************************************************/
 /* Feedback Multisynth Divider Equation
   *
   * where: a = mult, b = num and c = denom
   *
   * P1 register is an 18-bit value using following formula:
   *
   * 	P1[17:0] = 128 * mult + floor(128*(num/denom)) - 512
   *
   * P2 register is a 20-bit value using the following formula:
   *
   * 	P2[19:0] = 128 * num - denom * floor(128*(num/denom))
   *
   * P3 register is a 20-bit value using the following formula:
   *
   * 	P3[19:0] = denom
   */
/**************************************************************************/

void si5351::pll_config(uint8_t pll, uint8_t mult, 
			uint32_t num, uint32_t denom)
{
	uint8_t t, reg;

	_frac_pll = (num > 0);

	/* Set bits */
	if(pll == PLL_A)
	{
		_mult_a = mult;
		reg = 0x16;
	}
	else {
		_mult_b = mult;
		reg = 0x17;
	}

	t = read_register(reg);
	if (_frac_pll)
	{
	  write_register(reg, t & ~(1 << FBx_INT));	// fractional mode for PLL X
	}
	else
	{
	  write_register(reg, t | (1 << FBx_INT));	// integer mode for PLL X
	}

	set_config(pll, mult, num, denom);
	write_register(0xB1, 0xA0);			// reset both PLLs
}

/**************************************************************************/
/*! 
    @brief  Configures an output channel

    @param[in]	clk
    			Number of the configured channel. Accepts CLK0, CLK1 or CLK2
    @param[in]	config
    			Configuration byte. Some constants defined in si5351.h. 
    			Refer to AN619 for description of registers 16 - 23.
*/
/**************************************************************************/
void si5351::clk_config(uint8_t clk, uint8_t config)
{
	if(config & (1<<MSx_SRC))
		_clk_src = _clk_src & ~(1<<clk) | (1<<clk);

	write_register(CLK0_CFG + clk, config);
}

/**************************************************************************/
/*! 
    @brief  Disable output

    @param[in]	clk
    			Number of the configured channel. Accepts CLK0, CLK1 or CLK2
*/
/**************************************************************************/
void si5351::clk_disable(uint8_t clk)
{
	write_register(CLK_EN, read_register(CLK_EN) | (1 << clk));
}

/**************************************************************************/
/*! 
    @brief  Enable output

    @param[in]	clk
    			Number of the configured channel. Accepts CLK0, CLK1 or CLK2
*/
/**************************************************************************/
void si5351::clk_enable(uint8_t clk)
{
	write_register(CLK_EN, read_register(CLK_EN) & ~(1 << clk));
}

/**************************************************************************/
/*! 
    @brief  Set phase offset

    @param[in]	clk
    			Number of the configured channel. Accepts CLK0, CLK1 or CLK2
    @param[in]	phase
    			Phase offset. Refer to AN619 for description 
    			of registers 165 - 170.
*/
/**************************************************************************/
void si5351::set_phase(uint8_t clk, uint8_t phase)
{
	write_register(CLK0_PHOFF + clk, phase & 0x7F);
}

/**************************************************************************/
/*! 
    @brief  Set output frequency. Automaticaly selects integer/fractional
    		mode as appropriate.

    @param[in]	clk
    			Number of the configured channel. Accepts CLK0, CLK1 or CLK2
    @param[in]	freq
    			Output frequency in Hz.
*/
/**************************************************************************/
void si5351::set_frequency(uint8_t clk, uint32_t freq)
{
	//float divider;
	uint8_t config[8];

	if (freq > FREQ_DIV4){
	  set_frequency_FIXDIV(clk, freq, 4);	//freq > 150MHz, set div=4, adjust PLL
	  return;
	}
	
	if (freq > FREQ_DIV6){
	  set_frequency_FIXDIV(clk, freq, 6);	////freq > 112.5MHz, set div=6, adjust PLL
	  return;
	}

	if (_frac_pll){
	  pll_config(PLL_A, PLL_MULT, 0, 1);	//<150 Mhz, PLL int config
	}

	if(_clk_src & (1<<clk))
		//divider = (_xtal * _mult_b) / (float)freq;
		calc_divider(_mult_b, _xtal, freq);
	else
		//divider = (_xtal * _mult_a) / (float)freq;
		calc_divider(_mult_a, _xtal, freq);

	if (_b == 0)
		write_register(MS_0 + clk, read_register(MS_0 + clk) | (1<<MSx_INT));
	else
		write_register(MS_0 + clk, read_register(MS_0 + clk) & ~(1<<MSx_INT));

	set_config(MS_0+clk*8, _a, _b, _c);
}

/**************************************************************************/
/*! 
    @brief  Set Synth Frequency ( DIV by 8, 6 or 4)

    @param[in]	clk
    			Number of the configured channel. Accepts CLK0, CLK1 or CLK2
    @param[in]	freq
    			Frequency to set to
    @param[in] DIV
			Divider
*/
/**************************************************************************/
void si5351::set_frequency_FIXDIV(uint8_t clk, uint32_t freq, uint8_t DIV)
{
	uint8_t config[8];

	calc_divider(DIV, freq, _xtal);	//d * xtal = DIV*freq

	pll_config(PLL_A, _a, _b, _c);		//Set PLL to DIV*freq

	write_register(MS_0 + clk, read_register(MS_0 + clk) | (1<<MSx_INT));

	set_config(MS_0+clk*8, DIV, 0, 1);
}

/**************************************************************************/
/*! 
    @brief  Calculate Divider

    @param[in]	mul
    			Multiplier
    @param[in]	tgt
    			Target frequency
    @param[in]  ref
			Reference freq  divider * ref = mul*tgt
*/
/**************************************************************************/
void si5351::calc_divider(uint8_t mul, uint32_t tgt, uint32_t ref){
	float divider;
	divider = mul * ((float)tgt/(float)ref);
	_a = (uint32_t)divider;
	divider -= _a;
	if(divider == 0) {
		_c = 1;
		_b = 0;
	}
	else {
		farey(divider, _b, _c);
	}
	return;
}

/**************************************************************************/
/*! 
    @brief  Set config registers

    @param[in]	reg
    			Register offset
    @param[in]	mul
    			Multiplier
    @param[in]  num
			Numerator
    @param[in]  denom
			Denominator
*/			
/**************************************************************************/
void si5351::set_config(uint8_t reg, uint32_t mul, uint32_t num, uint32_t denom){
	uint8_t config[] = {0x00, 0x01, 0x00, 0x0D, 0x00, 0x00, 0x00, 0x00};
	uint32_t p1, p2, p3;
	
	p1 = (uint32_t)(128 * mul + (uint32_t)(128 * ((float)num/(float)denom)) - 512);
	p2 = (uint32_t)(128 * num - denom * (uint32_t)(128 * ((float)num/(float)denom)));
	p3 = denom;

  	config[0] = (p3 & 0x0000FF00) >> 8;
  	config[1] = (p3 & 0x000000FF);
	config[2] = (p1 & 0x00030000) >> 16;
	config[3] = (p1 & 0x0000FF00) >> 8;
	config[4] = (p1 & 0x000000FF);
  	config[5] = ((p3 & 0x000F0000) >> 12) | ((p2 & 0x000F0000) >> 16);
  	config[6] = (p2 & 0x0000FF00) >> 8;
  	config[7] = (p2 & 0x000000FF);
	write_block(reg, config, 8);			// write MultiSynth config

}

/**************************************************************************/
/*! 
    @brief  Find best rational approximation using Farey series. Refer to
    http://www.johndcook.com/blog/2010/10/20/best-rational-approximation 
    for algorithm description.

    @param[in]	alpha
    			Real number that is to be approximated. Must be < 1.
    @param[out]	x
    			Numerator
    @param[out]	y
    			Denominator
*/
/**************************************************************************/
void si5351::farey(float alpha, uint32_t &x, uint32_t &y)
{
	uint32_t p, q, r, s;
	
	p = 0; q = 1;
	r = 1; s = 1;

	while(q <= FAREY_N && s <= FAREY_N)
	{
		if(alpha*(q+s) == (p+r))
			if( (q+s) <= FAREY_N) {
				x = p + r;
				y = q + s;
				return;
			}
			else if(s > q) {
				x = r;
				y = s;
				return;
			}
			else {
				x = p;
				y = q;
				return;
			}
		else if(alpha*(q+s) > (p+r)) {
			p += r;
			q += s;
		}
		else {
			r += p;
			s += q;
		}
	}

	if(q > FAREY_N) {
		x = r;
		y = s;
		return;
	}
	else {
		x = p;
		y = q;
		return;
	}
}

/**************************************************************************/
/*! 
    @brief  Set output frequency. Automaticaly selects integer/fractional
    		mode as appropriate. This function uses a very simplistic 
    		calculation algorithm which sacrifices accuracy of the output
    		frequency for speed of execution. 

    @param[in]	clk
    			Number of the configured channel. Accepts CLK0, CLK1 or CLK2
    @param[in]	freq
    			Output frequency in Hz.
*/
/**************************************************************************/
void si5351::simple_set_frequency(uint8_t clk, uint32_t freq)
{
	uint64_t vco;
	uint8_t config[8];

	if(_clk_src & (1<<clk))
		vco = _xtal*_mult_b;
	else
		vco = _xtal*_mult_a;

	vco = vco * 0x00100000;

	_c = 0x000FFFFE;

	_b = vco / freq;

	_a = (_b - (_b & 0x000FFFFF)) >> 20;
	_b = _b & 0x000FFFFF;

	if(_b == 0) {
		write_register(MS_0 + clk, read_register(MS_0 + clk) | (1<<MSx_INT));
		_c = 1;
		_p1 = 128*_a - 512;
		_p2 = 0;
	}
	else {
		write_register(MS_0 + clk, read_register(MS_0 + clk) & ~(1<<MSx_INT));
		_p1 = 128*_a + (128 * _b)/_c - 512;
		_p2 = 128*_b - _c*((128 * _b)/_c);
	}
	
	config[0] = (_c & 0x0000FF00) >> 8;
	config[1] = (_c & 0x000000FF);
	config[2] = (_p1 & 0x00030000) >> 16;
	config[3] = (_p1 & 0x0000FF00) >> 8;
	config[4] = (_p1 & 0x000000FF);
	config[5] = ((_p2 & 0x000F0000) >> 16) | (_c & 0x000F0000) >> 12;
	config[6] = (_p2 & 0x0000FF00) >> 8;
	config[7] = (_p2 & 0x000000FF);

	write_block(MS_0 + clk*8, config, 8);
}

/**************************************************************************/
/*! 
    @brief  For debug get the divider parameters.
    
    @param[in]	a, b, p1, p2, p3
*/
/**************************************************************************/
void si5351::get_params(uint32_t &a, uint32_t &b, uint32_t &c, uint8_t &frac)
{
	a = _a;
	b = _b;
	c = _c;
	frac = _frac_pll;
}
