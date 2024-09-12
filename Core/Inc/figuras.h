#ifndef FIGURAS_H_
#define FIGURAS_H_

//504 bytes por figura - 48x84 pixels

static const unsigned char tela_inicial [] = {
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0xfe, 0x02, 0xfa, 0x8a, 0x8a, 0x8a, 0x8a, 0x8a, 0x4a, 0x7a, 0x86, 0xfc,
	0x00, 0x00, 0x00, 0xfe, 0x02, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfe, 0x02, 0xfe, 0x00,
	0x00, 0x00, 0xfe, 0x02, 0xee, 0xd8, 0xb0, 0x60, 0xc0, 0x80, 0x00, 0xfe, 0x02, 0xfe, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0x80, 0xfe, 0x02, 0x02, 0x06, 0x1c, 0x30,
	0x67, 0xcd, 0x99, 0xf0, 0x00, 0x00, 0x00, 0x3f, 0x60, 0xdf, 0xb0, 0xa0, 0xa0, 0xa0, 0xa0, 0xb0,
	0xdf, 0x60, 0x3f, 0x00, 0x00, 0x00, 0xff, 0x80, 0xff, 0x00, 0x01, 0x03, 0x06, 0x0d, 0x1b, 0xf7,
	0x80, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x80, 0x80, 0x00, 0x00,
	0x80, 0x40, 0xa0, 0x50, 0x28, 0xa8, 0x54, 0x54, 0x54, 0x5c, 0x5c, 0x9c, 0x1c, 0x14, 0x14, 0x14,
	0x14, 0x14, 0x14, 0x14, 0x14, 0x1c, 0x1c, 0x1c, 0x1c, 0x14, 0x14, 0x14, 0x28, 0x28, 0x50, 0xa0,
	0x40, 0x80, 0x00, 0x00, 0x80, 0x80, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc3, 0x22,
	0xf2, 0x4b, 0xc6, 0x4d, 0x8a, 0x89, 0x08, 0x0e, 0x09, 0x0f, 0x09, 0x0b, 0x09, 0xfb, 0xf9, 0xff,
	0xf9, 0x0e, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x0e, 0xf9, 0xf9, 0xf9, 0xf9, 0x09, 0x09, 0x09,
	0x09, 0x09, 0x0e, 0x08, 0x89, 0x8a, 0x4d, 0xc6, 0x4b, 0xd2, 0x22, 0xc3, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0xf0, 0xff, 0x80, 0x07, 0x88, 0xcb, 0xd6, 0xd4, 0xd4, 0xd5, 0x96, 0x0c, 0x00, 0x18, 0x3c,
	0x74, 0xef, 0xd7, 0xaf, 0xd7, 0xac, 0xd4, 0xac, 0xd5, 0xad, 0xd4, 0xac, 0xd4, 0xaf, 0xd7, 0xaf,
	0xd7, 0x6c, 0x3c, 0x18, 0x00, 0x0c, 0x96, 0xd5, 0xd4, 0xd4, 0xd6, 0xcb, 0x88, 0x07, 0x80, 0xff,
	0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x7f, 0x7f, 0x7f, 0x70, 0x73, 0x77, 0x77, 0x77, 0x13, 0x13, 0x11,
	0x18, 0x1c, 0x1c, 0x1c, 0x1c, 0x1f, 0x1f, 0x17, 0x1b, 0x16, 0x1a, 0x16, 0x1a, 0x16, 0x1a, 0x16,
	0x1a, 0x17, 0x1b, 0x17, 0x1f, 0x1c, 0x1c, 0x1c, 0x1c, 0x18, 0x11, 0x13, 0x13, 0x77, 0x77, 0x77,
	0x73, 0x70, 0x7f, 0x7f, 0x7f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

static const unsigned char tela_final [] = {
	0x00, 0xf8, 0xf8, 0x06, 0x06, 0x66, 0x66, 0xe0, 0xe0, 0x00, 0x00, 0xf8, 0xf8, 0x66, 0x66, 0x66,
	0x66, 0xf8, 0xf8, 0x00, 0x00, 0xfe, 0xfe, 0x18, 0x18, 0x60, 0x60, 0x18, 0x18, 0xfe, 0xfe, 0x00,
	0x00, 0xfe, 0xfe, 0x66, 0x66, 0x66, 0x66, 0x06, 0x06, 0x00, 0x00, 0xf8, 0xf8, 0x06, 0x06, 0x06,
	0x06, 0xf8, 0xf8, 0x00, 0x00, 0x1e, 0x1e, 0xe0, 0xe0, 0x00, 0x00, 0xe0, 0xe0, 0x1e, 0x1e, 0x00,
	0x00, 0xfe, 0xfe, 0x66, 0x66, 0x66, 0x66, 0x06, 0x06, 0x00, 0x00, 0xfe, 0xfe, 0x66, 0x66, 0xe6,
	0xe6, 0x18, 0x18, 0x00, 0x00, 0x01, 0x01, 0x86, 0x46, 0x46, 0xc6, 0x81, 0x81, 0x00, 0x00, 0x07,
	0x07, 0x00, 0x00, 0x00, 0x00, 0x07, 0x07, 0x00, 0x00, 0x07, 0x07, 0xc0, 0x70, 0x50, 0x70, 0xb8,
	0x18, 0xd7, 0x87, 0x70, 0x70, 0x07, 0x07, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x00, 0x00, 0x01,
	0x01, 0x06, 0x06, 0x06, 0x06, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x06, 0x06, 0x01,
	0x01, 0x00, 0x00, 0x00, 0x00, 0x07, 0x07, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x00, 0x00, 0x07,
	0x07, 0x00, 0x00, 0x01, 0x01, 0x06, 0x06, 0x00, 0x00, 0x00, 0xe0, 0x7f, 0xa8, 0x13, 0xe9, 0x34,
	0x11, 0x0f, 0x00, 0x00, 0x80, 0xc0, 0xe0, 0xe0, 0xf0, 0x70, 0x70, 0xe0, 0xe0, 0xc0, 0x80, 0x53,
	0x5c, 0x5b, 0x5c, 0x4d, 0x26, 0x23, 0x20, 0x20, 0x10, 0x10, 0x28, 0xcc, 0x3e, 0x7f, 0x7f, 0xe7,
	0xe7, 0xe7, 0xff, 0x7e, 0x3c, 0x24, 0x28, 0x30, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0xde,
	0x23, 0x11, 0x09, 0x34, 0xb4, 0xd2, 0xe2, 0xd2, 0x93, 0x93, 0x97, 0x0f, 0x1f, 0x1e, 0x3c, 0x3c,
	0x3f, 0x9f, 0x8f, 0x86, 0x8c, 0x10, 0x60, 0xe0, 0xe0, 0xe0, 0xf0, 0xf0, 0xf0, 0xf0, 0xf0, 0x09,
	0x8e, 0x88, 0x70, 0x70, 0xa0, 0xe0, 0xe0, 0xe0, 0x78, 0x38, 0x18, 0x04, 0x03, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x3c, 0x5b, 0x9c, 0xf0, 0x0c, 0x0e, 0x0d, 0x1c, 0x18, 0x39, 0x31, 0x31, 0x73, 0xe3, 0xf3,
	0xfb, 0xfe, 0xfe, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x7f, 0x7f, 0x5e, 0xdd, 0xdb, 0xd3, 0x73,
	0x53, 0x2d, 0xa2, 0x17, 0x23, 0x19, 0x9c, 0xce, 0xc7, 0x03, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x02, 0x02,
	0x02, 0x02, 0x02, 0x03, 0x03, 0x33, 0x3b, 0x38, 0x19, 0x01, 0x03, 0x03, 0x02, 0x1a, 0x1d, 0x3d,
	0x3d, 0x3c, 0x7c, 0x78, 0x78, 0x3b, 0x13, 0x07, 0x06, 0x00, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};


// ---------------------------------------------------------------------------------------
// Os pixeis n�o empregados na definic�o dos bytes DEVEM ser ZERO, ou seja, os pixeis que n�o devem ser impressos
// A figura � desenhada conforme trabalho do LCD, do LSB to MSB (bytes alinhados verticalmente, ver manual do LCD5110)
//-----------------------------------------------------------------------------------------------------------------

const struct figura_t carro =
{
		19,
		11,
		{0xfe, 0x52, 0xfe, 0x8f, 0x07, 0xff, 0x56, 0x56, 0x56, 0x56, 0xfe, 0x02, 0x8e, 0x73, 0x53, 0x53,
		0x56, 0x56, 0xfc, 0x03, 0x02, 0x03, 0x07, 0x07, 0x07, 0x03, 0x03, 0x03, 0x03, 0x03, 0x02, 0x03,
		0x06, 0x06, 0x06, 0x03, 0x03, 0x01}
};

const struct figura_t apaga_carro =
{
		19,
		11,
		{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00}
};

const struct figura_t pista =
{
		94,
		2,
		{0x00, 0x00, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x00, 0x00, 0x00, 0x00, 0x03, 0x03, 0x03,
				0x03, 0x03, 0x03, 0x03, 0x00, 0x00, 0x00, 0x00, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x00,
				0x00, 0x00, 0x00, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x00, 0x00, 0x00, 0x00, 0x03,
				0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x00, 0x00, 0x00, 0x00, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03,
				0x03, 0x00, 0x00, 0x00, 0x00, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x00, 0x00, 0x00, 0x00,
				0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x00, 0x00, 0x03, 0x03, 0x03, 0x03, 0x03}
};

const struct figura_t coracao =
{
		5,
		4,
		{0x02, 0x07, 0x0e, 0x07, 0x02}
};

const struct figura_t apaga_coracao =
{
		5,
		4,
		{0x00, 0x00, 0x00, 0x00, 0x00}
};

const struct figura_t pedra =
{
		5,
		5,
		{0x0e, 0x1f, 0x1f, 0x1f, 0x0e}
};

const struct figura_t apaga_pedra =
{
		5,
		5,
		{0x00, 0x00, 0x00, 0x00, 0x00}
};


#endif /* FIGURAS_H_ */
