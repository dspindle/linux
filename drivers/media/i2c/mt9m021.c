/*	--*- c -*--
 * Copyright (C) 2015 Enrico Scholz <enrico.scholz@sigma-chemnitz.de>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; version 2 and/or (at your option) version 3 of the
 * License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#define DEBUG	1
#define xDEBUG_FLASH	1
//??PATCHED dspindle@leuze.de 2015-08-24
#define CONFIG_VIDEO_ADV_DEBUG 1

#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/clk.h>
#include <linux/regmap.h>
#include <linux/videodev2.h>
#include <linux/thermal.h>
#include <linux/interrupt.h>

#include <media/v4l2-subdev.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>

#include "aptina-pll.h"

#define MT9M021_REG_CHIP_VERSION_REG				0x3000
#define MT9M021_REG_Y_ADDR_START(_ctx)				((_ctx) ? 0x308c : 0x3002)
#define MT9M021_REG_X_ADDR_START(_ctx)				((_ctx) ? 0x308a : 0x3004)
#define MT9M021_REG_Y_ADDR_END(_ctx)				((_ctx) ? 0x3090 : 0x3006)
#define MT9M021_REG_X_ADDR_END(_ctx)				((_ctx) ? 0x308e : 0x3008)
#define MT9M021_REG_FRAME_LENGTH_LINES(_ctx)			((_ctx) ? 0x30aa : 0x300A)
#define MT9M021_REG_LINE_LENGTH_PCK				0x300C
#  define MT9M021_VAL_LINE_LENGTH_PCK_min			(0x672)
#define MT9M021_REG_REVISION_NUMBER				0x300E

#define MT9M021_REG_COARSE_INTEGRATION_TIME(_ctx)		((_ctx) ? 0x3016 : 0x3012)
#define MT9M021_REG_FINE_INTEGRATION_TIME(_ctx)			((_ctx) ? 0x3018 : 0x3014)

#define MT9M021_REG_RESET_REGISTER				0x301A
#  define MT9M021_FLD_RESET_REGISTER_RESET			BIT(0)
#  define MT9M021_FLD_RESET_REGISTER_STREAM			BIT(2)
#  define MT9M021_FLD_RESET_REGISTER_DRIVE_PINS			BIT(6)
#  define MT9M021_FLD_RESET_REGISTER_PARALLEL_EN		BIT(7)
#  define MT9M021_FLD_RESET_REGISTER_GPI_EN			BIT(8)
#  define MT9M021_FLD_RESET_REGISTER_FORCED_PLL_ON		BIT(11)
#  define MT9M021_FLD_RESET_REGISTER_SMIA_SERIALISER_DIS	BIT(12)

#define MT9M021_REG_PRE_PLL_CLK_DIV				0x302E
#define MT9M021_REG_PLL_MULTIPLIER				0x3030
#define MT9M021_REG_VT_SYS_CLK_DIV				0x302C
#define MT9M021_REG_VT_PIX_CLK_DIV				0x302A

#define MT9M021_REG_FLASH					0x3046
#  define MT9M021_FLD_FLASH_INVERT_FLASH			BIT(7)
#  define MT9M021_FLD_FLASH_EN_FLASH				BIT(8)

#define MT9M021_REG_GLOBAL_GAIN					0x305e

#define MT9M021_REG_EMBEDED_DATA_CTRL				0x3064
#  define MT9M021_FLD_EMBEDED_DATA_CTRL_EMBEDDED_STATS_EN	BIT(7)
#  define MT9M021_FLD_EMBEDED_DATA_CTRL_EMBEDDED_DATA		BIT(8)

#define MT9M021_REG_DIGITAL_TEST_PATTERN_MODE			0x3070
#define MT9M021_REG_DIGITAL_TEST_DATA_RED			0x3072
#define MT9M021_REG_DIGITAL_TEST_DATA_GREENR			0x3074
#define MT9M021_REG_DIGITAL_TEST_DATA_BLUE			0x3076
#define MT9M021_REG_DIGITAL_TEST_DATA_GREENB			0x3078

#define MT9M021_REG_DIGITAL_TEST				0x30B0
#  define MT9M021_FLD_DIGITAL_TEST_PLL_COMPLETE_BYPASS		BIT(14)

#define MT9M021_REG_TEMPSENS_DATA				0x30B2
#define MT9M021_REG_TEMPSENS_CTRL				0x30B4
#  define MT9M021_FLD_TEMPSENS_POWER_ON				BIT(0)
#  define MT9M021_FLD_TEMPSENS_TEST_CTRL(_x)			((_x_) << 1)
#  define MT9M021_FLD_TEMPSENS_START_CONVERSION			BIT(4)
#  define MT9M021_FLD_TEMPSENS_CLEAR_VALUE			BIT(5)

#define MT9M021_REG_TEMPSENS_CALIB1				0x30C6
#define MT9M021_REG_TEMPSENS_CALIB2				0x30C8
#define MT9M021_REG_TEMPSENS_CALIB3				0x30Ca
#define MT9M021_REG_TEMPSENS_CALIB4				0x30Cc

#define MT9M021_REG_AE_CTRL					0x3100
#  define MT9M021_FLD_AE_CTRL_AE_ENABLE				BIT(0)
#  define MT9M021_FLD_AE_CTRL_AUTO_AG_ENABLE			BIT(1)
#  define MT9M021_FLD_AE_CTRL_AUTO_DG_ENABLE			BIT(4)
#  define MT9M021_FLD_AE_CTRL_MIN_ANA_GAIN(_g)			((_g) << 5)
#  define MT9M021_FLD_AE_CTRL_MIN_ANA_GAIN_msk			(3 << 5)

/* from MT9M021/MT9M031: Developer Guide, "Pixel Data Format" */
#define MT9M021_MIN_X	0		/* 2 ? */
#define MT9M021_MAX_X	1296		/* 1412-x ? */
#define MT9M021_MIN_Y	0		/* 2 ? */
#define MT9M021_MAX_Y	976		/* 1028-y ? */

enum {
	FLAG_EXTCLK_ENABLED,
	FLAG_EXTCLK_PENDING,
	FLAG_STREAMING,
	FLAG_IRQ_ENABLED,
};

enum mt9m021_mode {
	MT9M021_MODE_MASTER,
	MT9M021_MODE_TRIGGER_PULSED,
	MT9M021_MODE_TRIGGER_AUTO,
};

struct mt9m021_sensor {
	struct v4l2_subdev		subdev;
	struct media_pad		pad;
	struct i2c_client		*i2c;
	struct clk			*extclk;
	unsigned int			extclk_rate_hz;
	unsigned int			pixclk;
	unsigned long			flags;
	struct regmap			*regmap;

	struct v4l2_rect		crop;
	struct v4l2_ctrl_handler	ctrls;
	bool				is_color;
	unsigned int			ctx;
	unsigned int			color_code;

	uint16_t			test_data[4];
	unsigned int			exposure_us;
	enum mt9m021_mode		op_mode;

	struct gpio_desc		*gpio_nreset;
	struct gpio_desc		*gpio_standby;
	struct gpio_desc		*gpio_flash;
	struct gpio_desc		*gpio_trigger;
	struct gpio_desc		*gpio_oebar;

	struct thermal_zone_device	*tz;

	int				irq_flash;
	ktime_t				tm_flash;
};

#define sd_to_sensor(_sd) \
	container_of((_sd), struct mt9m021_sensor, subdev)

struct regval {
	uint16_t		val;
	uint16_t		msk;
	uint16_t		reg;
};

#define REG_UPDATE(_r, _v, _m) \
	((struct regval){ .val = (_v), .msk = (_m), .reg = (_r) })
#define REG_SET(_r, _v)		REG_UPDATE(_r, _v, _v)
#define REG_CLR(_r, _v)		REG_UPDATE(_r,  0, _v)
#define REG_WRITE(_r, _v)	REG_UPDATE(_r, _v, 0xffff)

static bool mt9m021_debug_flash(struct mt9m021_sensor const *sensor)
{
	return IS_ENABLED(DEBUG_FLASH) && sensor->irq_flash >= 0;
}

static irqreturn_t mt9m021_flash_handler(int num, void *sensor_)
{
	struct mt9m021_sensor	*sensor = sensor_;
	ktime_t			ts;
	struct timeval		delta;

	if (!IS_ENABLED(DEBUG_FLASH))
		return IRQ_NONE;

	ts = ktime_get();
	delta = ktime_to_timeval(ktime_sub(ts, sensor->tm_flash));
	sensor->tm_flash = ts;

	trace_printk("flash delta: %ld.%06ld\n", delta.tv_sec, delta.tv_usec);
	return IRQ_HANDLED;
}

static int mt9m021_enable_extclk(struct mt9m021_sensor *sensor, bool ena)
{
	int		rc;

	/* we should be called only locked so this should not be needed */
	if (WARN_ON(test_and_set_bit(FLAG_EXTCLK_PENDING, &sensor->flags)))
		return -EBUSY;

	/* skip operation when EXTCLK has been enabled already or when EXTCLK
	 * is not used */
	if (!sensor->extclk ||
	    !!test_bit(FLAG_EXTCLK_ENABLED, &sensor->flags) == ena) {
		rc = 1;
	} else if (ena) {
		rc = clk_enable(sensor->extclk);
		WARN_ON(rc > 0);
	} else {
		clk_disable(sensor->extclk);
		rc = 0;
	}

	if (rc < 0)
		dev_err(&sensor->i2c->dev, "failed to %s EXTCLK: %d\n",
			ena ? "enable" : "disable", rc);
	else if (rc > 0)
		rc = 0;
	else if (ena)
		set_bit(FLAG_EXTCLK_ENABLED, &sensor->flags);
	else
		clear_bit(FLAG_EXTCLK_ENABLED, &sensor->flags);

	clear_bit(FLAG_EXTCLK_PENDING, &sensor->flags);

	return rc;
}

static int _mt9m021_apply(struct mt9m021_sensor *sensor,
			  struct regval const regs[], size_t cnt)
{
	size_t		i;
	int		rc = 0;

	for (i = 0; i < cnt; ++i) {
		struct regval const	*r = &regs[i];

		if (r->msk == (uint16_t)~0)
			rc = regmap_write(sensor->regmap, r->reg, r->val);
		else
			rc = regmap_update_bits(sensor->regmap,
						r->reg, r->msk, r->val);

		if (rc < 0) {
			dev_warn(&sensor->i2c->dev,
				 "failed to set reg %02x to %02x&%02x: %d\n",
				 r->reg, r->val, r->msk, rc);
			break;
		}
	}

	return rc;
}

static __always_inline int mt9m021_apply_check(struct mt9m021_sensor *sensor,
					      struct regval regs[],
					      size_t max_cnt,
					      size_t cnt)
{
	if (0) /* causes false positives; disable it for now */
		BUILD_BUG_ON(max_cnt < cnt);

	BUG_ON(max_cnt < cnt);
	return _mt9m021_apply(sensor, regs, cnt);
}

#define mt9m021_apply(_ap, _regs, _cnt) \
	mt9m021_apply_check((_ap), (_regs), ARRAY_SIZE(_regs), (_cnt))

static int mt9m021_read_seq(struct mt9m021_sensor *sensor,
			    uint16_t const regs[], size_t cnt,
			    uint16_t res[])
{
	size_t		i;
	int		rc = 0;

	for (i = 0; i < cnt; ++i) {
		unsigned int	v;

		rc = regmap_read(sensor->regmap, regs[i], &v);
		if (rc < 0) {
			dev_warn(&sensor->i2c->dev,
				 "failed to read reg %02x: %d\n", regs[i], rc);
			break;
		}

		res[i] = v;
	}

	return rc;
}

static int mt9m021_soft_reset(struct mt9m021_sensor *sensor)
{
	int		rc;

	/* section "Soft Reset" in "MT9M021 Developer Guide" */
	rc = regmap_update_bits(sensor->regmap,
				MT9M021_REG_RESET_REGISTER,
				MT9M021_FLD_RESET_REGISTER_RESET,
				MT9M021_FLD_RESET_REGISTER_RESET);

	if (rc < 0) {
		dev_err(&sensor->i2c->dev,
			"failed to issue soft reset: %d\n", rc);
		return rc;
	}

	/* wait some cycles (t5=150000 required by datasheet) before sending
	 * the first I2C command */
	/* TODO: does t5 apply to soft reset case too? */
	mdelay(20);

	regcache_mark_dirty(sensor->regmap);

	/* configure sensor to drive pins and to be in standby mode */
	rc = regmap_update_bits(sensor->regmap,
				MT9M021_REG_RESET_REGISTER,
				MT9M021_FLD_RESET_REGISTER_SMIA_SERIALISER_DIS |
				MT9M021_FLD_RESET_REGISTER_DRIVE_PINS |
				MT9M021_FLD_RESET_REGISTER_PARALLEL_EN |
				MT9M021_FLD_RESET_REGISTER_GPI_EN |
				MT9M021_FLD_RESET_REGISTER_STREAM |
				MT9M021_FLD_RESET_REGISTER_RESET,

				/* when we can not control OE_BAR, drive
				 * parallel interface always */
				(sensor->gpio_oebar
				 ? MT9M021_FLD_RESET_REGISTER_DRIVE_PINS
				 : 0) |
				MT9M021_FLD_RESET_REGISTER_PARALLEL_EN |

				/* TODO: make it conditional to enable input
				 * drivers? */
				MT9M021_FLD_RESET_REGISTER_GPI_EN |

				/* TODO: allow to configure HiSPi mode */
				MT9M021_FLD_RESET_REGISTER_SMIA_SERIALISER_DIS);

	if (rc < 0) {
		dev_err(&sensor->i2c->dev,
			"failed to configure reset: %d\n", rc);
		return rc;
	}

	return 0;
}

static int mt9m021_set_pixclk(struct mt9m021_sensor *sensor,
			      unsigned long pixclk)
{
	/* TODO: aptina-pll driver does not support P2; assume static value
	 * for now although this removes some choices for pixclk */
	enum { P2 = 5 };
	static struct aptina_pll_limits const	pll_limits = {
		.ext_clock_min	=   6000000,
		.ext_clock_max	=  50000000,
		.int_clock_min	=   6000000 / 63,
		.int_clock_max	=  50000000 / 1,
		.out_clock_min	= 384000000,
		.out_clock_max	= 768000000,
		.pix_clock_max	=  74250000 * P2,

		.n_min		=   1,
		.n_max		=  63,

		.m_min		=  32,
		.m_max		= 255,

		.p1_min		=   1,
		.p1_max		=  16
	};

	int			rc;
	struct regval		seq[10];
	struct regval		*s = seq;

	if (sensor->extclk_rate_hz == pixclk) {
		*s++  = REG_SET(MT9M021_REG_DIGITAL_TEST,
				MT9M021_FLD_DIGITAL_TEST_PLL_COMPLETE_BYPASS);
	} else {
		struct aptina_pll			pll = {
			.ext_clock	= sensor->extclk_rate_hz,
			.pix_clock	= pixclk * P2,
		};


		rc = aptina_pll_calculate(&sensor->i2c->dev, &pll_limits, &pll);
		if (rc < 0)
			goto out;

		*s++ = REG_CLR(MT9M021_REG_DIGITAL_TEST,
			       MT9M021_FLD_DIGITAL_TEST_PLL_COMPLETE_BYPASS);
		*s++ = REG_WRITE(MT9M021_REG_PRE_PLL_CLK_DIV, pll.n);
		*s++ = REG_WRITE(MT9M021_REG_PLL_MULTIPLIER,  pll.m);
		*s++ = REG_WRITE(MT9M021_REG_VT_SYS_CLK_DIV,  pll.p1);
		*s++ = REG_WRITE(MT9M021_REG_VT_PIX_CLK_DIV,  P2);

		/* TODO: really correct? */
		pixclk = sensor->extclk_rate_hz / pll.n * pll.m / pll.p1 / P2;
	}

	rc = mt9m021_apply(sensor, seq, s - seq);
	if (rc < 0)
		goto out;

	dev_dbg(&sensor->i2c->dev, "pixclk set to %lu\n", pixclk);

	sensor->pixclk = pixclk;
	rc = 0;

out:
	if (rc < 0)
		dev_warn(&sensor->i2c->dev, "failed to set pixclk %lu: %d\n",
			 pixclk, rc);

	return rc;
}

static int mt9m021_core_reset(struct v4l2_subdev *sd, u32 val)
{
	struct mt9m021_sensor	*sensor = sd_to_sensor(sd);
	int			rc;

	rc = mt9m021_soft_reset(sensor);
	if (rc < 0)
		goto out;

	rc = mt9m021_set_pixclk(sensor, sensor->pixclk);
	if (rc < 0)
		goto out;

	rc = 0;

out:
	return rc;
}

static int mt9m021_core_registered_async(struct v4l2_subdev *sd)
{
/*
#ifdef CONFIG_MEDIA_CONTROLLER

	int rc = 0;
	rc = media_device_register_entity(sd->v4l2_dev->mdev, &sd->entity);
	if (rc < 0) {
		dev_err(sd->dev, "media_device_register_entity() failed: %d\n", rc);
		return rc;
	}

#endif
*/
	return 0;
}

static int mt9m021_set_exposure(struct mt9m021_sensor *sensor,
				unsigned int e_us)
{
	unsigned int		ctx = sensor->ctx;
	uint16_t const		regs[] = {
		[0] = MT9M021_REG_LINE_LENGTH_PCK,
		[1] = MT9M021_REG_FRAME_LENGTH_LINES(ctx),
		[2] = MT9M021_REG_Y_ADDR_START(ctx),
		[3] = MT9M021_REG_Y_ADDR_END(ctx),
		[4] = MT9M021_REG_FRAME_LENGTH_LINES(ctx),
		[5] = MT9M021_REG_EMBEDED_DATA_CTRL,
	};
	uint16_t		v[ARRAY_SIZE(regs)];

	struct regval		seq[2];
	struct regval		*s = seq;

	int			rc;
	uint64_t		tmp;
	unsigned int		coarse;
	unsigned int		fine;
	unsigned int		bad_coarse;

	rc =  mt9m021_read_seq(sensor, regs, ARRAY_SIZE(regs), v);
	if (rc < 0)
		goto out;

	if (v[0] < MT9M021_VAL_LINE_LENGTH_PCK_min) {
		dev_warn(&sensor->i2c->dev, "invalid line length %u\n", v[0]);
		rc = -EIO;
		goto out;
	}

	/* MT9M021 Developer Guide says on pg. 21 that some coarse values
	 * should be avoided .. (it seems to double the flash duration
	 * time) */
	bad_coarse = v[4] - (v[3] - v[2] + 1 + 8 + 6 + 7);
	if ((v[5] & MT9M021_FLD_EMBEDED_DATA_CTRL_EMBEDDED_DATA))
		bad_coarse -= 2;

	/* t_INT = t_COARSE + t_FINE */

	/* e_us is integration time in micro seconds; calculate number of
	 * pixels the exposure is active */
	tmp  = e_us;
	tmp *= sensor->pixclk;
	tmp  = div64_ul(tmp, 1000000u);

	/* there are 6+6 blank lines */
	if (tmp > (v[1] + 12) * v[0])
		dev_warn(&sensor->i2c->dev,
			 "exposure time exceeds frame time\n");

	coarse = ((unsigned int)(tmp)) / v[0];
	fine   = ((unsigned int)(tmp)) % v[0];

	if (coarse == bad_coarse) {
		dev_warn(&sensor->i2c->dev,
			 "bad coarse value; increasing it by one\n");
		++coarse;
		fine = 0;
	}

	/* 'fine' must not exceed 'line_length_pck - 750' */
	BUILD_BUG_ON(MT9M021_VAL_LINE_LENGTH_PCK_min < 750);
	if (fine + 750 > v[0]) {
		dev_dbg(&sensor->i2c->dev,
			"%s: fine value %u exceeds %u-750; adjusting\n",
			__func__, fine, v[0]);
		fine = v[0] - 750;
	}

	*s++ = REG_WRITE(MT9M021_REG_COARSE_INTEGRATION_TIME(sensor->ctx),
			 coarse);
	*s++ = REG_WRITE(MT9M021_REG_FINE_INTEGRATION_TIME(sensor->ctx),
			 fine);

	rc = mt9m021_apply(sensor, seq, s - seq);
	if (rc < 0)
		goto out;

	dev_dbg(&sensor->i2c->dev,
		"%s: e_us=%u -> coarse=%u, fine=%u (width %x, freq %u), bad=%u\n",
		__func__, e_us, coarse, fine, v[0], sensor->pixclk, bad_coarse);

	sensor->exposure_us = e_us;
	rc = 0;

out:
	return rc;
}

static int mt9m021_s_streamon(struct mt9m021_sensor *sensor)
{
	int		rc;
	struct regval	seq[3];
	struct regval	*s = seq;

	/* TODO: lock this! */
	set_bit(FLAG_STREAMING, &sensor->flags);

	/* set exposure time to honor changed geometry */
	rc = mt9m021_set_exposure(sensor, sensor->exposure_us);
	if (rc < 0)
		goto out;

	if (sensor->gpio_oebar)
		gpiod_set_value(sensor->gpio_oebar, 0);

	switch (sensor->op_mode) {
	case MT9M021_MODE_MASTER:
		*s++ = REG_SET(MT9M021_REG_RESET_REGISTER,
			       MT9M021_FLD_RESET_REGISTER_STREAM);
		break;

	case MT9M021_MODE_TRIGGER_PULSED:
	case MT9M021_MODE_TRIGGER_AUTO:
		*s++ = REG_UPDATE(MT9M021_REG_RESET_REGISTER,
				  MT9M021_FLD_RESET_REGISTER_FORCED_PLL_ON,
				  MT9M021_FLD_RESET_REGISTER_STREAM |
				  MT9M021_FLD_RESET_REGISTER_FORCED_PLL_ON);
		break;
	}

	if (mt9m021_debug_flash(sensor)) {
		*s++ = REG_UPDATE(MT9M021_REG_FLASH,
				  MT9M021_FLD_FLASH_EN_FLASH,
				  MT9M021_FLD_FLASH_EN_FLASH);
	}


	rc = mt9m021_apply(sensor, seq, s - seq);
	if (rc < 0)
		goto out;

	switch (sensor->op_mode) {
	case MT9M021_MODE_MASTER:
		if (sensor->gpio_trigger)
			gpiod_set_value(sensor->gpio_trigger, true);
		break;

	case MT9M021_MODE_TRIGGER_PULSED:
	case MT9M021_MODE_TRIGGER_AUTO:
		/* deassert trigger; application has to assert it */
		if (sensor->gpio_trigger)
			gpiod_set_value(sensor->gpio_trigger, false);
		break;
	}

	if (mt9m021_debug_flash(sensor) &&
	    !test_and_set_bit(FLAG_IRQ_ENABLED, &sensor->flags))
		enable_irq(sensor->irq_flash);

	rc = 0;

out:
	if (rc)
		clear_bit(FLAG_STREAMING, &sensor->flags);

	return rc;
}

static int mt9m021_s_streamoff(struct mt9m021_sensor *sensor)
{
	int		rc;
	struct regval	seq[3];
	struct regval	*s = seq;

	if (test_and_clear_bit(FLAG_IRQ_ENABLED, &sensor->flags))
		disable_irq(sensor->irq_flash);

	if (sensor->gpio_trigger)
		gpiod_set_value(sensor->gpio_trigger, false);


	*s++ = REG_CLR(MT9M021_REG_RESET_REGISTER,
		       MT9M021_FLD_RESET_REGISTER_STREAM |
		       MT9M021_FLD_RESET_REGISTER_FORCED_PLL_ON);

	if (mt9m021_debug_flash(sensor)) {
		*s++ = REG_UPDATE(MT9M021_REG_FLASH,
				  MT9M021_FLD_FLASH_EN_FLASH, 0);
	}

	rc = mt9m021_apply(sensor, seq, s - seq);
	if (rc < 0)
		goto out;

	if (sensor->gpio_oebar)
		gpiod_set_value(sensor->gpio_oebar, 1);

	rc = 0;

out:
	/* TODO: move it up and clear it only in success case? */
	clear_bit(FLAG_STREAMING, &sensor->flags);

	return rc;
}

static int mt9m021_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct mt9m021_sensor	*sensor = sd_to_sensor(sd);
	int			rc;

	if (enable)
		rc = mt9m021_s_streamon(sensor);
	else
		rc = mt9m021_s_streamoff(sensor);

	return rc;
}

static int mt9m021_g_mbus_config(struct v4l2_subdev *sd,
				 struct v4l2_mbus_config *cfg)
{
	/* TODO: allow to invert signals? */
	cfg->type = V4L2_MBUS_PARALLEL;
	cfg->flags = V4L2_MBUS_MASTER;

	return 0;
}

static struct v4l2_subdev_video_ops const	mt9m021_video_ops = {
	.g_mbus_config	= mt9m021_g_mbus_config,
	.s_stream	= mt9m021_s_stream,
};

static int mt9m021_enum_mbus_code(struct v4l2_subdev *subdev,
				  struct v4l2_subdev_pad_config *cfg,
				  struct v4l2_subdev_mbus_code_enum *code)
{
	struct mt9m021_sensor	*sensor = sd_to_sensor(subdev);

	if (code->index > 0)
		return -EINVAL;

	if (sensor->is_color)
		code->code = MEDIA_BUS_FMT_SGRBG12_1X12;
	else
		code->code = MEDIA_BUS_FMT_Y12_1X12;

	return 0;
}

static int mt9m021_enum_frame_size(struct v4l2_subdev *subdev,
				   struct v4l2_subdev_pad_config *cfg,
				   struct v4l2_subdev_frame_size_enum *fse)
{
	struct mt9m021_sensor	*sensor = sd_to_sensor(subdev);

	if ((sensor->is_color  && fse->code != MEDIA_BUS_FMT_SGRBG12_1X12) ||
	    (!sensor->is_color && fse->code != MEDIA_BUS_FMT_Y12_1X12)) {
		dev_dbg(&sensor->i2c->dev, "invalid format code %x\n",
			fse->code);
		return -EINVAL;
	}

	if (fse->index > 0)
		return -EINVAL;

	/* TODO: allow variable sizes */
	fse->min_width = 1280;
	fse->max_width = 1280;
	fse->min_height = 960;
	fse->max_height = 960;

	return 0;
}

static struct v4l2_rect *
__mt9m021_get_pad_crop(struct mt9m021_sensor *sensor,
		       struct v4l2_subdev_pad_config *cfg,
		       unsigned int pad,
		       enum v4l2_subdev_format_whence which)
{
	switch (which) {
	case V4L2_SUBDEV_FORMAT_TRY:
		return v4l2_subdev_get_try_crop(&sensor->subdev, cfg, pad);
	case V4L2_SUBDEV_FORMAT_ACTIVE:
		return &sensor->crop;
	default:
		return NULL;
	}
}

static int mt9m021_read_format(struct mt9m021_sensor *sensor,
			       struct v4l2_mbus_framefmt *fmt)
{
	fmt->code         = sensor->color_code;
	fmt->field        = V4L2_FIELD_NONE;
	fmt->colorspace   = V4L2_COLORSPACE_SRGB;
	fmt->quantization = V4L2_QUANTIZATION_DEFAULT;
	fmt->ycbcr_enc    = V4L2_YCBCR_ENC_DEFAULT;

	/* todo: honor binning/skipping */

	fmt->width  = sensor->crop.width;
	fmt->height = sensor->crop.height;

	return 0;
}

static int mt9m021_get_format(struct v4l2_subdev *subdev,
			      struct v4l2_subdev_pad_config *cfg,
			      struct v4l2_subdev_format *format)
{
	struct mt9m021_sensor	*sensor = sd_to_sensor(subdev);
	int			rc;

	if (format->pad != 0)
		return -EINVAL;

	switch (format->which) {
	case V4L2_SUBDEV_FORMAT_TRY:
		format->format = *v4l2_subdev_get_try_format(&sensor->subdev,
							     cfg, format->pad);
		rc = 0;
		break;

	case V4L2_SUBDEV_FORMAT_ACTIVE:
		rc = mt9m021_read_format(sensor, &format->format);
		break;

	default:
		rc = -EINVAL;
		break;
	}

	return rc;
}

static int mt9m021_write_format(struct mt9m021_sensor *sensor,
				struct v4l2_mbus_framefmt *format)
{
	if (format->width != sensor->crop.width ||
	    format->height != sensor->crop.height) {
		dev_warn(&sensor->i2c->dev,
			 "dimension mismatch %ux%u != %ux%u\n",
			 format->width, format->height,
			 sensor->crop.width, sensor->crop.height);

		/* TODO: reject this later (link validate? s_stream()?) */
	}

	if (sensor->is_color) {
		switch (format->code) {
		case MEDIA_BUS_FMT_SGRBG12_1X12:
		case MEDIA_BUS_FMT_SGRBG10_1X10:
		case MEDIA_BUS_FMT_SGRBG8_1X8:
			break;

		default:
			dev_warn(&sensor->i2c->dev,
				 "unsupported color format %08x\n",
				 format->code);
			return -EINVAL;
		}
	} else {
		switch (format->code) {
		case MEDIA_BUS_FMT_Y12_1X12:
		case MEDIA_BUS_FMT_Y10_1X10:
		case MEDIA_BUS_FMT_Y8_1X8:
			break;

		default:
			dev_warn(&sensor->i2c->dev,
				 "unsupported mono format %08x\n",
				 format->code);
			return -EINVAL;
		}
	}

	sensor->color_code = format->code;

	return 0;
}

static int mt9m021_set_format(struct v4l2_subdev *subdev,
			      struct v4l2_subdev_pad_config *cfg,
			      struct v4l2_subdev_format *format)
{
	struct mt9m021_sensor		*sensor = sd_to_sensor(subdev);
	int				rc;
	struct v4l2_mbus_framefmt	*s_fmt;

	switch (format->which) {
	case V4L2_SUBDEV_FORMAT_TRY:
		/* TODO: validate input */
		s_fmt = v4l2_subdev_get_try_format(&sensor->subdev, cfg,
						   format->pad);
		if (!s_fmt) {
			rc = -EINVAL;
			goto out;
		}

		*s_fmt = format->format;
		rc = 0;
		break;

	case V4L2_SUBDEV_FORMAT_ACTIVE:
		if (test_bit(FLAG_STREAMING, &sensor->flags)) {
			rc = -EBUSY;
			goto out;
		}

		rc = mt9m021_write_format(sensor, &format->format);
		break;

	default:
		rc = -EINVAL;
		break;
	}

out:
	return 0;
}

static int mt9m021_get_selection_default(struct mt9m021_sensor *sensor,
					 struct v4l2_rect *rect)
{
	uint16_t const		regs[] = {
		[0] = MT9M021_REG_Y_ADDR_START(sensor->ctx),
		[1] = MT9M021_REG_X_ADDR_START(sensor->ctx),
		[2] = MT9M021_REG_Y_ADDR_END(sensor->ctx),
		[3] = MT9M021_REG_X_ADDR_END(sensor->ctx),
		[4] = MT9M021_REG_FRAME_LENGTH_LINES(sensor->ctx),
		[5] = MT9M021_REG_LINE_LENGTH_PCK
	};
	uint16_t		v[ARRAY_SIZE(regs)];
	int			rc;

	rc =  mt9m021_read_seq(sensor, regs, ARRAY_SIZE(regs), v);
	if (rc < 0) {
		dev_warn(&sensor->i2c->dev,
			 "failed to read crop registers: %d\n", rc);
		goto out;
	}

	rect->top  = v[0];
	rect->left = v[1];
	rect->width = v[5];
	rect->height = v[4];

	/* TODO: subtract/add dark rows/cols */

	rc = 0;

out:
	return rc;
}

static int mt9m021_get_selection(struct v4l2_subdev *subdev,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_selection *sel)
{
	struct mt9m021_sensor	*sensor = sd_to_sensor(subdev);
	int			rc;

	if (sel->pad != 0)
		return -EINVAL;

	switch (sel->target) {
	case V4L2_SEL_TGT_CROP_DEFAULT:
		if (sel->which != V4L2_SUBDEV_FORMAT_ACTIVE)
			/* TODO: accept TRY? */
			return -EINVAL;

		rc = mt9m021_get_selection_default(sensor, &sel->r);
		break;

	case V4L2_SEL_TGT_CROP_BOUNDS:
		sel->r = (struct v4l2_rect){
			.left	= MT9M021_MIN_X,
			.width	= MT9M021_MAX_X,
			.top	= MT9M021_MIN_Y,
			.height = MT9M021_MAX_Y,
		};
		rc = 0;
		break;

	case V4L2_SEL_TGT_CROP:
		sel->r = *__mt9m021_get_pad_crop(sensor, cfg, sel->pad,
						 sel->which);

		rc = 0;
		break;

	default:
		rc = -EINVAL;
		break;
	}

	return rc;
}

static int mt9m021_read_geometry(struct mt9m021_sensor *sensor)
{
	static uint16_t const		REGS_B[] = {
		/* NOTE: keep then in order of context A */
		MT9M021_REG_Y_ADDR_START(1),
		MT9M021_REG_X_ADDR_START(1),
		MT9M021_REG_Y_ADDR_END(1),
		MT9M021_REG_X_ADDR_END(1),
		MT9M021_REG_FRAME_LENGTH_LINES(1),
		MT9M021_REG_LINE_LENGTH_PCK
	};

	int			rc;
	uint16_t		v[6];

	BUILD_BUG_ON((MT9M021_REG_LINE_LENGTH_PCK - MT9M021_REG_Y_ADDR_START(0))
		     / sizeof(v[0]) + 1 != ARRAY_SIZE(v));
	BUILD_BUG_ON(ARRAY_SIZE(REGS_B) != ARRAY_SIZE(v));

	if (sensor->ctx == 0)
		rc = regmap_bulk_read(sensor->regmap,
				      MT9M021_REG_Y_ADDR_START(0),
				      v, ARRAY_SIZE(v));
	else
		rc = mt9m021_read_seq(sensor, REGS_B, ARRAY_SIZE(REGS_B), v);

	if (rc < 0) {
		dev_err(&sensor->i2c->dev, "%s: failed to read geometry: %d\n",
			__func__, rc);
		return rc;
	}

#define I(_reg)	(((_reg) - MT9M021_REG_Y_ADDR_START(0)) / sizeof v[0])

	sensor->crop.left     = v[I(MT9M021_REG_X_ADDR_START(0))];
	sensor->crop.top      = v[I(MT9M021_REG_Y_ADDR_START(0))];
	sensor->crop.width    = (v[I(MT9M021_REG_X_ADDR_END(0))] + 1 -
				 v[I(MT9M021_REG_X_ADDR_START(0))]);
	sensor->crop.height   = (v[I(MT9M021_REG_Y_ADDR_END(0))] + 1 -
				 v[I(MT9M021_REG_Y_ADDR_START(0))]);

#undef I

	return 0;
}

static int mt9m021_write_crop(struct mt9m021_sensor *sensor,
			      struct v4l2_rect *rect)
{
	struct regval		seq[5];
	struct regval		*s = seq;
	int			rc;
	unsigned int		ctx = sensor->ctx;

	if (rect->left < MT9M021_MIN_X ||
	    rect->top < MT9M021_MIN_Y ||
	    rect->left + rect->width >= MT9M021_MAX_X ||
	    rect->top + rect->height >= MT9M021_MAX_Y)
		return -EINVAL;

	*s++ = REG_WRITE(MT9M021_REG_X_ADDR_START(ctx), rect->left);
	*s++ = REG_WRITE(MT9M021_REG_Y_ADDR_START(ctx), rect->top);
	*s++ = REG_WRITE(MT9M021_REG_X_ADDR_END(ctx),
			 rect->left + rect->width - 1);
	*s++ = REG_WRITE(MT9M021_REG_Y_ADDR_END(ctx),
			 rect->top + rect->height - 1);

	rc = mt9m021_apply(sensor, seq, s - seq);
	if (rc < 0)
		goto out;

	rc = mt9m021_read_geometry(sensor);

out:
	return rc;

}

static int mt9m021_set_selection_default(struct mt9m021_sensor *sensor,
					 struct v4l2_rect const *rect)
{
	struct regval		seq[2];
	struct regval		*s = seq;
	int			rc;

	*s++ = REG_WRITE(MT9M021_REG_LINE_LENGTH_PCK, rect->width);
	*s++ = REG_WRITE(MT9M021_REG_FRAME_LENGTH_LINES(sensor->ctx),
			 rect->height);

	rc = mt9m021_apply(sensor, seq, s - seq);
	if (rc < 0)
		goto out;

	rc = 0;

out:
	return rc;
}

static int mt9m021_set_selection(struct v4l2_subdev *subdev,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_selection *sel)
{
	struct mt9m021_sensor		*sensor = sd_to_sensor(subdev);
	struct v4l2_rect		*s_rect;
	int				rc;

	if (sel->pad != 0)
		return -EINVAL;

	switch (sel->target) {
	case V4L2_SEL_TGT_CROP_DEFAULT:
		if (sel->which != V4L2_SUBDEV_FORMAT_ACTIVE)
			/* TODO: accept TRY? */
			return -EINVAL;
		rc = mt9m021_set_selection_default(sensor, &sel->r);
		break;

	case V4L2_SEL_TGT_CROP:
		switch (sel->which) {
		case V4L2_SUBDEV_FORMAT_TRY:
			s_rect = v4l2_subdev_get_try_crop(&sensor->subdev, cfg,
							  sel->pad);
			if (!s_rect) {
				rc = -EINVAL;
				goto out;
			}

			*s_rect = sel->r;
			rc = 0;
			break;

		case V4L2_SUBDEV_FORMAT_ACTIVE:
			if (test_bit(FLAG_STREAMING, &sensor->flags) &&
			    ((sel->r.width != sensor->crop.width) ||
			     (sel->r.height != sensor->crop.height))) {
				/* prevent ROI changes which alter the
				 * dimension but allow ROI to be moved */
				rc = -EBUSY;
				goto out;
			}

			rc = mt9m021_write_crop(sensor, &sel->r);
			break;

		default:
			rc = -EINVAL;
			break;
		}
		break;

	case V4L2_SEL_TGT_CROP_BOUNDS:
	default:
		rc = -EINVAL;
		break;

	}

out:
	return rc;
}

static int mt9m021_querycap(struct mt9m021_sensor *sensor,
			    struct v4l2_capability *cap)
{
	strcpy(cap->driver, "mt9m021");

	return 0;
}

static long mt9m021_core_ioctl(struct v4l2_subdev *sd,
			       unsigned int cmd, void *arg)
{
	struct mt9m021_sensor	*sensor = sd_to_sensor(sd);

	switch (cmd) {
	case VIDIOC_QUERYCAP:
		return mt9m021_querycap(sensor, arg);

	default:
		return -ENOTTY;
	}
}

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int mt9m021_g_register(struct v4l2_subdev *sd,
			       struct v4l2_dbg_register *reg)
{
	struct mt9m021_sensor	*sensor = sd_to_sensor(sd);
	int			rc;
	unsigned int		val;

	rc = regmap_read(sensor->regmap, reg->reg, &val);
	if (rc < 0)
		return rc;

	reg->size = 2;
	reg->val  = val;

	return 0;
}

static int mt9m021_s_register(struct v4l2_subdev *sd,
			      struct v4l2_dbg_register const *reg)
{
	struct mt9m021_sensor	*sensor = sd_to_sensor(sd);
	int			rc;

	rc = regmap_write(sensor->regmap, reg->reg, reg->val);

	return rc;
}
#endif

static struct v4l2_subdev_pad_ops		mt9m021_pad_ops = {
	.enum_mbus_code		= mt9m021_enum_mbus_code,
	.enum_frame_size	= mt9m021_enum_frame_size,
	.get_fmt		= mt9m021_get_format,
	.set_fmt		= mt9m021_set_format,
	.get_selection		= mt9m021_get_selection,
	.set_selection		= mt9m021_set_selection,
};

static struct v4l2_subdev_core_ops const	mt9m021_core_ops = {
	.reset			= mt9m021_core_reset,
	.ioctl			= mt9m021_core_ioctl,

#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register		= mt9m021_g_register,
	.s_register		= mt9m021_s_register,
#endif
	.registered_async = mt9m021_core_registered_async,
};

static struct v4l2_subdev_ops const		mt9m021_subdev_ops = {
	.core		= &mt9m021_core_ops,
	.video		= &mt9m021_video_ops,
	.pad		= &mt9m021_pad_ops,
};

static int mt9m021_get_gpiod(struct mt9m021_sensor *sensor,
			     struct gpio_desc **gpio,
			     char const *name,
			     unsigned long flags)
{
	struct device		*dev = &sensor->i2c->dev;
	int			rc;
	struct gpio_desc	*desc;

	desc = devm_gpiod_get(dev, name, flags);
	rc = PTR_ERR_OR_ZERO(desc);

	switch (rc) {
	case 0:
		*gpio = desc;
		/* TODO: disable in final release */
		gpiod_export(desc, false);
		break;

	case -EPROBE_DEFER:
		/* noop */
		break;

	case -ENOENT:
		dev_dbg(dev, "skipping setup of %s gpio\n", name);
		*gpio = NULL;
		rc    = 0;
		break;

	default:
		dev_err(dev, "failed to get %s gpio: %d\n", name, rc);
		break;
	}

	return rc;
}

static int mt9m021_setup(struct mt9m021_sensor *sensor)
{
	struct device		*dev = &sensor->i2c->dev;
	int			rc;
	struct clk		*clk;

	clk = devm_clk_get(dev, "extclk");

	rc = PTR_ERR_OR_ZERO(clk);
	switch (rc) {
	case 0:
	case -EPROBE_DEFER:
		/* noop */
		break;

	case -ENOENT:
		dev_dbg(dev, "skipping setup of EXTCLK\n");
		clk = NULL;
		rc  = 0;
		break;

	default:
		dev_err(dev, "failed to get EXTCLK: %d\n", rc);
		break;
	}

	if (rc < 0)
		goto out;

	sensor->extclk = clk;
	sensor->extclk_rate_hz = 27000000;	/* TODO: take from dtree */

	rc = mt9m021_get_gpiod(sensor, &sensor->gpio_nreset, "nreset",
			       GPIOD_OUT_HIGH);
	if (rc)
		goto out;

	rc = mt9m021_get_gpiod(sensor, &sensor->gpio_standby, "standby",
			       GPIOD_OUT_LOW);
	if (rc)
		goto out;

	rc = mt9m021_get_gpiod(sensor, &sensor->gpio_flash, "flash",
			       GPIOD_IN);
	if (rc)
		goto out;

	rc = mt9m021_get_gpiod(sensor, &sensor->gpio_trigger, "trigger",
			       GPIOD_OUT_LOW);
	if (rc)
		goto out;

	rc = mt9m021_get_gpiod(sensor, &sensor->gpio_oebar, "oebar",
			       GPIOD_OUT_HIGH);
	if (rc)
		goto out;

	if (!sensor->gpio_flash || !IS_ENABLED(DEBUG_FLASH)) {
		sensor->irq_flash = -1;
	} else {
		sensor->irq_flash = gpiod_to_irq(sensor->gpio_flash);
		if (sensor->irq_flash < 0) {
			dev_warn(dev, "failed to map FLASH gpio to irq: %d\n",
				 sensor->irq_flash);
			sensor->irq_flash = -1;
			/* TODO: ignore error for now */
		}
	}

	if (sensor->irq_flash >= 0) {
		unsigned long flags = (IRQF_TRIGGER_RISING |
				       IRQF_TRIGGER_FALLING);

		rc = devm_request_irq(dev, sensor->irq_flash,
				      mt9m021_flash_handler, flags,
				      "MT9M021 flash", sensor);
		if (rc < 0) {
			dev_err(dev, "failed to request FLASH irq: %d\n", rc);
			goto out;
		}

		disable_irq(sensor->irq_flash);
	}

	if (1) {
		/* HACK: enable output unconditionally */
		gpiod_set_value(sensor->gpio_oebar, 0);
		sensor->gpio_oebar = NULL;
	}

	/* TODO: set sensor->is_color */
	sensor->is_color = false;

	rc = 0;

out:
	return rc;
}

static int mt9m021_init(struct mt9m021_sensor *sensor)
{
	struct device		*dev = &sensor->i2c->dev;
	int			rc;
	bool			clk_prepared = false;
	unsigned int		version;
	unsigned int		revision;

	if (sensor->extclk) {
		unsigned int	rate;

		rc = clk_prepare(sensor->extclk);
		if (rc < 0) {
			dev_err(dev, "failed to get prepare EXTCLK: %d\n", rc);
			goto out;
		}

		clk_prepared = true;

		rc = mt9m021_enable_extclk(sensor, true);
		if (rc < 0)
			goto out;

		rc = clk_set_rate(sensor->extclk, sensor->extclk_rate_hz);
		if (rc < 0) {
			dev_err(dev, "failed to set EXTCLK to %u Hz: %d\n",
				sensor->extclk_rate_hz, rc);
			goto out;
		}

		rate = clk_get_rate(sensor->extclk);
		if (rate < 6000000 || rate > 50000000) {
			dev_err(dev, "invalid extclk rate %u\n", rate);
			goto out;
		}

		sensor->extclk_rate_hz = rate;
	}

	if (!sensor->gpio_nreset) {
		/* wait some cycles (t5=150000 required by datasheet) before
		 * sending the first I2C command */
		mdelay(20);

		rc = mt9m021_soft_reset(sensor);
	} else {
		rc = gpiod_direction_output(sensor->gpio_nreset, 0);
		if (rc < 0) {
			dev_err(dev,
				"failed to configure nRESET as output: %d\n",
				rc);
			goto out;
		}

		/* t4 > 1ms */
		mdelay(1);

		gpiod_set_value(sensor->gpio_nreset, 1);

		/* wait some cycles (t5=150000 required by datasheet) before
		 * sending the first I2C command */
		mdelay(20);

		regcache_mark_dirty(sensor->regmap);
	}

	rc = regmap_read(sensor->regmap, MT9M021_REG_CHIP_VERSION_REG,
			 &version);
	if (rc < 0) {
		dev_err(dev, "failed to read device id: %d\n", rc);
		goto out;
	}

	if (version != 0x2401) {
		dev_err(dev, "invalid version id %02x\n", version);
		rc = -ENOENT;
		goto out;
	}

	rc = mt9m021_soft_reset(sensor);
	if (rc < 0)
		goto out;

	rc = regmap_read(sensor->regmap, MT9M021_REG_CHIP_VERSION_REG,
			 &version);
	if (rc < 0)
		goto out;

	rc = regmap_read(sensor->regmap, MT9M021_REG_REVISION_NUMBER,
			 &revision);
	if (rc < 0)
		goto out;

	rc = mt9m021_set_pixclk(sensor, sensor->extclk_rate_hz);
	if (rc < 0)
		goto out;

	rc = mt9m021_read_geometry(sensor);
	if (rc < 0)
		goto out;

	dev_info(dev, "chip %04x.%02x detected\n", version, revision);

	rc = 0;

out:
	if (rc < 0) {
		if (clk_prepared) {
			mt9m021_enable_extclk(sensor, false);
			clk_unprepare(sensor->extclk);
		}
	}

	return rc;
}


static char const * const	mt9m021_menu_test_pattern[] = {
	"Disabled",
	"Solid Color",
	"100% color bar",
	"Fade to Gray",
	"Walking 1s pattern",
};

static char const * const	mt9m021_menu_embedded_info[] = {
	"disabled",
	"statistics",
	"data",
	"both"
};

static char const * const	mt9m021_menu_operating_mode[] = {
	"master (video)",
	"pulsed trigger",
	"automatic trigger",
};

#define V4L2_CID_EMBEDDED_INFO		(V4L2_CID_USER_BASE | 0x1000)
#define V4L2_CID_OPERATING_MODE		(V4L2_CID_USER_BASE | 0x1001)
#define V4L2_CID_X_PIXEL_RATE		(V4L2_CID_USER_BASE | 0x1002)

static int mt9m021_set_test_pattern(struct mt9m021_sensor *sensor,
				    unsigned long idx)
{
	struct regval		seq[10];
	struct regval		*s = seq;

	*s++ = REG_WRITE(MT9M021_REG_DIGITAL_TEST_PATTERN_MODE,
			 idx < 4 ? idx : 256);
	*s++ = REG_WRITE(MT9M021_REG_DIGITAL_TEST_DATA_RED,
			 sensor->test_data[0]);
	*s++ = REG_WRITE(MT9M021_REG_DIGITAL_TEST_DATA_GREENR,
			 sensor->test_data[1]);
	*s++ = REG_WRITE(MT9M021_REG_DIGITAL_TEST_DATA_BLUE,
			 sensor->test_data[2]);
	*s++ = REG_WRITE(MT9M021_REG_DIGITAL_TEST_DATA_GREENB,
			 sensor->test_data[3]);

	return mt9m021_apply(sensor, seq, s - seq);
}

static int mt9m021_set_test_color(struct mt9m021_sensor *sensor,
				  unsigned int id, unsigned long val)
{
	unsigned int		reg;

	switch (id) {
	case V4L2_CID_TEST_PATTERN_RED:
		reg = MT9M021_REG_DIGITAL_TEST_DATA_RED;
		break;
	case V4L2_CID_TEST_PATTERN_GREENR:
		reg = MT9M021_REG_DIGITAL_TEST_DATA_GREENR;
		break;
	case V4L2_CID_TEST_PATTERN_BLUE:
		reg = MT9M021_REG_DIGITAL_TEST_DATA_BLUE;
		break;
	case V4L2_CID_TEST_PATTERN_GREENB:
		reg = MT9M021_REG_DIGITAL_TEST_DATA_GREENB;
		break;
	default:
		WARN_ON(1);
		return -EINVAL;
	}

	return regmap_write(sensor->regmap, reg, val);
}

static int mt9m021_set_gain(struct mt9m021_sensor *sensor, unsigned long val)
{
	/* todo: use a better unit/scale?  for now, we write fixed poimt
	 * xxx.yyyyy value (0-8) */
	return regmap_write(sensor->regmap, MT9M021_REG_GLOBAL_GAIN, val);
}

static int mt9m021_set_opmode(struct mt9m021_sensor *sensor,
			      enum mt9m021_mode op_mode)
{
	int		rc;

	if (test_bit(FLAG_STREAMING, &sensor->flags))
		return -EBUSY;

	if (op_mode > MT9M021_MODE_TRIGGER_AUTO)
		return -EINVAL;

	rc = regmap_update_bits(sensor->regmap,
				MT9M021_REG_RESET_REGISTER,
				MT9M021_FLD_RESET_REGISTER_STREAM,
				(op_mode == MT9M021_MODE_MASTER ?
				 MT9M021_FLD_RESET_REGISTER_STREAM : 0));
	if (rc < 0)
		return rc;

	sensor->op_mode = op_mode;

	return 0;
}

static int mt9m021_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct mt9m021_sensor	*sensor =
		container_of(ctrl->handler, struct mt9m021_sensor, ctrls);

	switch (ctrl->id) {
	case V4L2_CID_PIXEL_RATE:
	case V4L2_CID_X_PIXEL_RATE:
		return mt9m021_set_pixclk(sensor, ctrl->val);
	case V4L2_CID_TEST_PATTERN:
		return mt9m021_set_test_pattern(sensor, ctrl->val);

	case V4L2_CID_TEST_PATTERN_RED:
	case V4L2_CID_TEST_PATTERN_GREENR:
	case V4L2_CID_TEST_PATTERN_BLUE:
	case V4L2_CID_TEST_PATTERN_GREENB:
		return mt9m021_set_test_color(sensor, ctrl->id, ctrl->val);

	case V4L2_CID_EXPOSURE_AUTO:
		return regmap_update_bits(sensor->regmap,
					  MT9M021_REG_AE_CTRL,
					  MT9M021_FLD_AE_CTRL_AE_ENABLE,
					  ctrl->val == V4L2_EXPOSURE_AUTO
					  ? MT9M021_FLD_AE_CTRL_AE_ENABLE
					  : 0);

	case V4L2_CID_AUTOGAIN:
		return regmap_update_bits(sensor->regmap,
					  MT9M021_REG_AE_CTRL,
					  MT9M021_FLD_AE_CTRL_AUTO_AG_ENABLE |
					  MT9M021_FLD_AE_CTRL_AUTO_DG_ENABLE,
					  ctrl->val
					  ? (MT9M021_FLD_AE_CTRL_AUTO_AG_ENABLE |
					     MT9M021_FLD_AE_CTRL_AUTO_DG_ENABLE)
					  : 0);

	case V4L2_CID_EMBEDDED_INFO:
		return regmap_update_bits(sensor->regmap,
					  MT9M021_REG_EMBEDED_DATA_CTRL,
					  MT9M021_FLD_EMBEDED_DATA_CTRL_EMBEDDED_STATS_EN |
					  MT9M021_FLD_EMBEDED_DATA_CTRL_EMBEDDED_DATA,
					  ((ctrl->val & 1)
					   ? MT9M021_FLD_EMBEDED_DATA_CTRL_EMBEDDED_STATS_EN
					   : 0) |
					  ((ctrl->val & 2)
					   ? MT9M021_FLD_EMBEDED_DATA_CTRL_EMBEDDED_DATA
					   : 0));

	case V4L2_CID_OPERATING_MODE:
		return mt9m021_set_opmode(sensor, ctrl->val);

	case V4L2_CID_EXPOSURE:
		return mt9m021_set_exposure(sensor, ctrl->val);

	case V4L2_CID_GAIN:
		return mt9m021_set_gain(sensor, ctrl->val);

	default:
		WARN_ON(1);
		return -EINVAL;
	}
}

static int mt9m021_g_volatile_ctrl(struct v4l2_ctrl *ctrl)
{
	struct mt9m021_sensor	*sensor =
		container_of(ctrl->handler, struct mt9m021_sensor, ctrls);

	switch (ctrl->id) {
	case V4L2_CID_X_PIXEL_RATE:
		ctrl->val = sensor->pixclk;
		break;

	default:
		WARN_ON(1);
		return -EINVAL;
	}

	return 0;
}

struct v4l2_ctrl_ops const	mt9m021_ctrl_ops = {
	.s_ctrl			= mt9m021_s_ctrl,
	.g_volatile_ctrl	= mt9m021_g_volatile_ctrl,
};

static struct v4l2_ctrl_config const	mt9m021_ctrls[] = {
	{
		.ops		= &mt9m021_ctrl_ops,
		.id		= V4L2_CID_EMBEDDED_INFO,
		.type		= V4L2_CTRL_TYPE_MENU,
		.name		= "Embedded Information",
		.min		= 0,
		.max		= ARRAY_SIZE(mt9m021_menu_embedded_info) - 1,
		.qmenu		= mt9m021_menu_embedded_info,
	}, {
		.ops		= &mt9m021_ctrl_ops,
		.id		= V4L2_CID_OPERATING_MODE,
		.type		= V4L2_CTRL_TYPE_MENU,
		.name		= "Operating mode",
		.min		= 0,
		.max		= ARRAY_SIZE(mt9m021_menu_operating_mode) - 1,
		.qmenu		= mt9m021_menu_operating_mode,
	}, {
		.ops		= &mt9m021_ctrl_ops,
		.id		= V4L2_CID_X_PIXEL_RATE,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.name		= "X Pixel Rate",
		.min		= 6000000,
		.max		= 74250000,
		.step		= 1,
	}
};

static int mt9m021_get_temp(void *sensor_, int *res)
{
	enum {
		CALIB_T0	= 55000,
		CALIB_T1	= 70000,
		SENS_VAL_MAX	= 1023
	};
	struct mt9m021_sensor	*sensor = sensor_;
	unsigned int		v;
	int			rc;
	static int		calib70 = 0; // calibration value at 70 degrees celsius
	static int		calib55 = 0; // calibration value at 55 degrees celsius
	static long		slope = 0; // slope of the linear conversion function
	static long		intercept = 0; // y-intercept value of the lin conv func

	// if this is the first invocation read calibration values from sensor,
	// compute the conversion parameters and do the initial conversion
	if ( (calib70 == 0) || (calib55 == 0) ) {
		rc = regmap_read(sensor->regmap, MT9M021_REG_TEMPSENS_CALIB1, &calib70);
		if (rc < 0)
			goto out;

		rc = regmap_read(sensor->regmap, MT9M021_REG_TEMPSENS_CALIB2, &calib55);
		if (rc < 0)
			goto out;

		if (calib70 <= calib55) {
			dev_warn(&sensor->i2c->dev,
				"invalid calibration data [%d,%d]\n", calib70, calib55);
			rc = -EIO;
			goto out;
		}

		slope = ((CALIB_T1 - CALIB_T0) * 100) / (calib70 - calib55);
		intercept = (CALIB_T0 * 100) - (slope * calib55);

		dev_dbg(&sensor->i2c->dev, "calibration initialized... c70=%i, c55=%i, slope=%li, intercept=%li\n",
			calib70, calib55, slope, intercept);

		rc = regmap_update_bits(sensor->regmap,
			MT9M021_REG_TEMPSENS_CTRL,
			MT9M021_FLD_TEMPSENS_POWER_ON |
			MT9M021_FLD_TEMPSENS_START_CONVERSION |
			MT9M021_FLD_TEMPSENS_CLEAR_VALUE,
			MT9M021_FLD_TEMPSENS_POWER_ON |
			MT9M021_FLD_TEMPSENS_CLEAR_VALUE);
		if (rc < 0)
			goto out;

		rc = regmap_update_bits(sensor->regmap,
			MT9M021_REG_TEMPSENS_CTRL,
			MT9M021_FLD_TEMPSENS_POWER_ON |
			MT9M021_FLD_TEMPSENS_START_CONVERSION |
			MT9M021_FLD_TEMPSENS_CLEAR_VALUE,
			MT9M021_FLD_TEMPSENS_POWER_ON |
			MT9M021_FLD_TEMPSENS_START_CONVERSION);
		if (rc < 0)
			goto out;

		msleep(10);
	}

	// read temp sens data
	rc = regmap_read(sensor->regmap, MT9M021_REG_TEMPSENS_DATA, &v);
	if (rc < 0)
		goto out;

	// clear value
	rc = regmap_update_bits(sensor->regmap,
		MT9M021_REG_TEMPSENS_CTRL,
		MT9M021_FLD_TEMPSENS_POWER_ON |
		MT9M021_FLD_TEMPSENS_START_CONVERSION |
		MT9M021_FLD_TEMPSENS_CLEAR_VALUE,
		MT9M021_FLD_TEMPSENS_POWER_ON |
		MT9M021_FLD_TEMPSENS_CLEAR_VALUE);
	if (rc < 0)
		goto out;

	// start next conversion
	rc = regmap_update_bits(sensor->regmap,
		MT9M021_REG_TEMPSENS_CTRL,
		MT9M021_FLD_TEMPSENS_POWER_ON |
		MT9M021_FLD_TEMPSENS_START_CONVERSION |
		MT9M021_FLD_TEMPSENS_CLEAR_VALUE,
		MT9M021_FLD_TEMPSENS_POWER_ON |
		MT9M021_FLD_TEMPSENS_START_CONVERSION);
	if (rc < 0)
		goto out;

	// check sensor value plausibility
	if( (v == 0) || (v > SENS_VAL_MAX) ) {
		rc = -EIO;
		goto out;
	}

	// compute actual temperature value
	res[0] = ((slope * (long)v) + intercept) / 100;

	// we're done
	rc = 0;

out:
	return rc;
}

static struct thermal_zone_of_device_ops const	mt9m021_thermal_ops = {
	.get_temp	= mt9m021_get_temp,
};

static int mt9m021_init_controls(struct mt9m021_sensor *sensor)
{
	struct device		*dev = &sensor->i2c->dev;
	int			rc;
	size_t			i;

	rc = v4l2_ctrl_handler_init(&sensor->ctrls,
				    ARRAY_SIZE(mt9m021_ctrls) + 10);
	if (rc < 0) {
		dev_err(dev, "v4l2_ctrl_handler_init: %d\n", rc);
		return rc;
	}

	v4l2_ctrl_new_std_menu_items(&sensor->ctrls, &mt9m021_ctrl_ops,
				     V4L2_CID_TEST_PATTERN,
				     ARRAY_SIZE(mt9m021_menu_test_pattern) - 1,
				     0, 0, mt9m021_menu_test_pattern);

	v4l2_ctrl_new_std(&sensor->ctrls, &mt9m021_ctrl_ops,
			  V4L2_CID_PIXEL_RATE, 6000000, 74250000, 1,
			  sensor->pixclk);

	v4l2_ctrl_new_std(&sensor->ctrls, &mt9m021_ctrl_ops,
			  V4L2_CID_TEST_PATTERN_RED, 0, 0xffff, 1,
			  sensor->test_data[0]);

	v4l2_ctrl_new_std(&sensor->ctrls, &mt9m021_ctrl_ops,
			  V4L2_CID_TEST_PATTERN_GREENR, 0, 0xffff, 1,
			  sensor->test_data[1]);

	v4l2_ctrl_new_std(&sensor->ctrls, &mt9m021_ctrl_ops,
			  V4L2_CID_TEST_PATTERN_BLUE, 0, 0xffff, 1,
			  sensor->test_data[2]);

	v4l2_ctrl_new_std(&sensor->ctrls, &mt9m021_ctrl_ops,
			  V4L2_CID_TEST_PATTERN_GREENB, 0, 0xffff, 1,
			  sensor->test_data[3]);

	v4l2_ctrl_new_std(&sensor->ctrls, &mt9m021_ctrl_ops,
			  V4L2_CID_EXPOSURE, 0, 0x7fffffff, 1,
			  10000);	/* 10ms exposure time */

	v4l2_ctrl_new_std(&sensor->ctrls, &mt9m021_ctrl_ops,
			  V4L2_CID_GAIN, 0, 0xff, 1, 0x0020);

	v4l2_ctrl_new_std_menu(&sensor->ctrls, &mt9m021_ctrl_ops,
			       V4L2_CID_EXPOSURE_AUTO, 1, 0,
			       V4L2_EXPOSURE_MANUAL);

	v4l2_ctrl_new_std(&sensor->ctrls, &mt9m021_ctrl_ops,
			  V4L2_CID_AUTOGAIN, 0, 1, 1, 0);

        for (i = 0; i < ARRAY_SIZE(mt9m021_ctrls); ++i) {
		struct v4l2_ctrl_config		cfg = mt9m021_ctrls[i];

		if (cfg.id == V4L2_CID_X_PIXEL_RATE)
			cfg.def = sensor->pixclk;

                v4l2_ctrl_new_custom(&sensor->ctrls, &cfg, NULL);
	}

	rc = sensor->ctrls.error;
	if (rc) {
		dev_err(dev, "failed to create controls: %d\n", rc);
		goto out;
	}

	rc = v4l2_ctrl_handler_setup(&sensor->ctrls);
	if (rc < 0)
		goto out;

out:
	if (rc < 0)
		v4l2_ctrl_handler_free(&sensor->ctrls);

	return rc;
}

static bool mt9m021_regmap_volatile_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case MT9M021_REG_TEMPSENS_DATA:
		return true;

	default:
		return false;
	}
}

static struct regmap_config const	mt9m021_regmap = {
	.reg_bits		= 16,
	.reg_stride		=  2,
	.val_bits		= 16,
	.reg_format_endian	= REGMAP_ENDIAN_BIG,
	.val_format_endian	= REGMAP_ENDIAN_BIG,

	.volatile_reg		= mt9m021_regmap_volatile_reg,
};

static int mt9m021_probe(struct i2c_client *client,
			  const struct i2c_device_id *did)
{
	struct device		*dev = &client->dev;
	struct mt9m021_sensor	*sensor;
	int			rc;
	struct v4l2_subdev	*sd;

	sensor = devm_kzalloc(dev, sizeof *sensor, GFP_KERNEL);
	if (!sensor)
		return -ENOMEM;

	sensor->regmap = devm_regmap_init_i2c(client, &mt9m021_regmap);
	if (IS_ERR(sensor->regmap))
		return PTR_ERR(sensor->regmap);

	sensor->i2c = client;

	rc = mt9m021_setup(sensor);
	if (rc < 0)
		goto err;

	rc = mt9m021_init(sensor);
	if (rc < 0)
		goto err_init;

	rc = mt9m021_init_controls(sensor);
	if (rc < 0)
		goto err_ctrl;

	sensor->tz = thermal_zone_of_sensor_register(dev, 0, sensor,
						     &mt9m021_thermal_ops);

	rc = PTR_ERR_OR_ZERO(sensor->tz);
	if (rc < 0) {
		dev_warn(dev, "failed to register thermal zone device: %d\n",
			 rc);
		sensor->tz = NULL;
		/* ignore this error for now */
	}

	v4l2_i2c_subdev_init(&sensor->subdev, client, &mt9m021_subdev_ops);
	sensor->pad.flags |= MEDIA_PAD_FL_SOURCE;

	sd = &sensor->subdev;
	sd->owner  = THIS_MODULE;
	sd->dev    = dev;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	sd->ctrl_handler = &sensor->ctrls;
	snprintf(sd->name, sizeof sd->name, "mt9m021@%02x", client->addr);

	rc = media_entity_pads_init(&sd->entity, 1, &sensor->pad);
	if (rc < 0) {
		dev_err(dev, "media_entity_init() failed: %d\n", rc);
		goto err_media_entity_init;
	}

	rc = v4l2_async_register_subdev(sd);
	if (rc < 0) {
		dev_err(dev, "v4l2_async_register_subdev() failed: %d\n", rc);
		goto err_v4l2_async_register_subdev;
	}

	dev_info(dev, "MT9M021 sensor probed\n");

	return 0;

	v4l2_async_unregister_subdev(&sensor->subdev);
err_v4l2_async_register_subdev:

	media_entity_cleanup(&sensor->subdev.entity);
err_media_entity_init:

	if (sensor->tz)
		thermal_zone_of_sensor_unregister(dev, sensor->tz);

	v4l2_ctrl_handler_free(&sensor->ctrls);
err_ctrl:

	mt9m021_enable_extclk(sensor, false);
	if (sensor->extclk)
		clk_unprepare(sensor->extclk);
err_init:
err:

	return rc;
}

static int mt9m021_remove(struct i2c_client *client)
{
	struct v4l2_subdev	*subdev = i2c_get_clientdata(client);
	struct mt9m021_sensor	*sensor	= sd_to_sensor(subdev);

	if (test_and_clear_bit(FLAG_IRQ_ENABLED, &sensor->flags))
		disable_irq(sensor->irq_flash);

	v4l2_async_unregister_subdev(&sensor->subdev);
	media_entity_cleanup(&sensor->subdev.entity);
	v4l2_ctrl_handler_free(&sensor->ctrls);

	if (sensor->tz)
		thermal_zone_of_sensor_unregister(&client->dev, sensor->tz);

	mt9m021_enable_extclk(sensor, false);
	if (sensor->extclk)
		clk_unprepare(sensor->extclk);

	return 0;
}


static const struct i2c_device_id mt9m021_id[] = {
	{ "mt9m021", 0 },
	{ "mt9m031", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, mt9m021_id);

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id mt9m021_of_match[] = {
	{ .compatible = "aptina,mt9m021", },
	{ .compatible = "aptina,mt9m031", },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, mt9m021_of_match);
#endif

static struct i2c_driver mt9m021_i2c_driver = {
	.driver = {
		.of_match_table = of_match_ptr(mt9m021_of_match),
		.name = "mt9m021",
	},
	.probe          = mt9m021_probe,
	.remove         = mt9m021_remove,
	.id_table       = mt9m021_id,
};

module_i2c_driver(mt9m021_i2c_driver);

MODULE_DESCRIPTION("Aptina MT9M021 Camera driver");
MODULE_AUTHOR("Enrico Scholz <enrico.scholz@sigma-chemnitz.de>");
MODULE_LICENSE("GPL");
