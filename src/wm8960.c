
#include "wm8960.h"
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/i2s.h>
#include <zephyr/kernel.h>

static const uint8_t wm8960_init_seq1[][2] = {
	WM8960_PAIR(WM8960_RESET, 0),      /**< Reset to defaults */
	WM8960_PAIR(WM8960_PWR1, 0x00FE),  /**< Everything on (vmid, vref, ain,
					        adc, micb, digenb) */
	WM8960_PAIR(WM8960_PWR2, 0x01FF),  /**< Everything on (dac, spk, out3,
					       pll) */
	WM8960_PAIR(WM8960_PWR3, 0x001F),  /**< Everything on (mixers/mics) */
	WM8960_PAIR(WM8960_IFACE2,0x0040), /**< Slave mode, 16 bits,
					        format I2S */
	WM8960_PAIR(WM8960_LOUT1, 0x016f), /**< 0dB */
	WM8960_PAIR(WM8960_ROUT1, 0x016f), /**< 0dB */
	WM8960_PAIR(WM8960_LMIX, 0x0100),  /**< DAC routing */
	WM8960_PAIR(WM8960_RMIX, 0x0100),  /**< DAC routing */
	//WM8960_PAIR(WM8960_CTL2, 0x0004),  /**< Unmute ramp volume */
	WM8960_PAIR(WM8960_CTL1, 0x0000),  /**< Unmute */
	WM8960_PAIR(WM8960_CLASSD1, 0x00F7), /**< enable class d  */
	WM8960_PAIR(WM8960_IFACE1,0x0002),
	WM8960_PAIR(WM8960_ADCL, 0x01B8),
	WM8960_PAIR(WM8960_LIN, 0x0117),
	WM8960_PAIR(WM8960_ADCR, 0x0178),
	WM8960_PAIR(WM8960_RINL, 0x0117),
	WM8960_PAIR(WM8960_ACTL1, 0x00C0),
	WM8960_PAIR(WM8960_ACTL4, 0x0040),
	WM8960_PAIR(WM8960_BYPASS1, 0x0000),
	WM8960_PAIR(WM8960_BYPASS2, 0x0000),
	WM8960_PAIR(WM8960_LADC, 0x01ff),
	WM8960_PAIR(WM8960_RADC, 0x01ff),
	//WM8960_PAIR(WM8960_LOUT2, 0x01FF),
	//WM8960_PAIR(WM8960_ROUT2, 0x01FF),
	//WM8960_PAIR(WM8960_CLOCK1, 0x01B0)
	WM8960_PAIR(WM8960_CLOCK1, 0x00D8),
	WM8960_PAIR(WM8960_LOUT2, 0x0100),
	WM8960_PAIR(WM8960_ROUT2, 0x0100)
};
K_MEM_SLAB_DEFINE(tx_mem_slab, 128, 4, 32);

/**
 * Sets up the WM8960 as a RTIO IODEV
 *
 * @param wm8960 RTIO WM8960 object
 * @param i2c    I2C device
 * @param i2s    I2S device
 * @param srate  Sampling rate in [Hz]
 * @param ch     Number of channels
 * @param ptime  Wanted packet-time in [ms]
 * @param aufmt  Sample format (enum aufmt)
 */
void wm8960_configure(struct rtio_wm8960 *wm8960,
		      const struct device *i2c,
		      const struct device *i2s,
		      uint32_t   srate,
		      uint8_t    ch,
		      uint32_t   ptime,
		      int        fmt)
{
	int ret;
	const struct i2s_config* i2s_cfg1;
	struct i2s_config i2s_cfg;

	wm8960->i2c = i2c;
	wm8960->i2s = i2s;

	printk("info name: %s\n", i2c->name);

	for(int i = 0; i < ARRAY_SIZE(wm8960_init_seq1); i++) {
		ret = i2c_reg_write_byte(wm8960->i2c, WM8960_ADDR,
					 wm8960_init_seq1[i][0],
					 wm8960_init_seq1[i][1]);
		printk("seq: %hhx, %hhx\n", wm8960_init_seq1[i][0],
		       wm8960_init_seq1[i][1]);
		if (ret < 0) {
			printk("Initialization step %d failed with error %d\n",
			       i, ret);
		}
		__ASSERT(ret == 0, "Failed to write out wm8960 init sequence");
	}

	i2s_cfg.word_size = 16U;
	i2s_cfg.channels = ch;
	i2s_cfg.format = I2S_FMT_DATA_FORMAT_I2S;
	i2s_cfg.frame_clk_freq = srate;
	i2s_cfg.block_size = srate * ch * ptime / 1000 * 2;
	i2s_cfg.timeout = 1000;
	i2s_cfg.options = I2S_OPT_FRAME_CLK_MASTER | I2S_OPT_BIT_CLK_MASTER;
	i2s_cfg.mem_slab = &tx_mem_slab;

	ret = i2s_configure(wm8960->i2s, I2S_DIR_TX, &i2s_cfg);

	i2s_cfg1 = i2s_config_get(wm8960->i2s, I2S_DIR_TX);
	printk("word_size %d\n", i2s_cfg1->word_size);
	printk("channels %d\n", i2s_cfg1->channels);
	printk("frame_clk_freq %d\n", i2s_cfg1->frame_clk_freq);
	printk("block_size %d\n", i2s_cfg1->block_size);
	printk("timeout %d\n", i2s_cfg1->timeout);
}
