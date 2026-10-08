// SPDX-License-Identifier: GPL-2.0-only
/*
 * Novatek NT35597 1440x2560 DSC command mode panel of the Essential PH-1.
 * Init sequence and timings from the downstream
 * qcom,mdss_dsi_nt35597_dsc_wqxga_cmd_mata panel node.
 *
 * The DDIC drives 1440 columns, but the glass shows only columns 64..1375
 * (1312 pixels; downstream hid the edges in userspace). The layout parameter
 * selects how that is presented (boot time, for testing):
 *
 *  full:   1440x2560 mode, DSC with 2 x 720 px slices (the downstream setup).
 *  center: <width>x2560 mode. The link still carries 1440 px lines (2 x 720
 *          px slices, same stream as "full"); the DSI host widens the mode to
 *          the DSC slice grid and the DPU centers the planes in it.
 *  window: <width>x2560 mode. The DDIC's column window (set_column_address)
 *          is set to the middle <width> columns and DSC carries <slices>
 *          slices of <width>/<slices> px. Works only if the DDIC accepts a
 *          DSC picture narrower than 1440 written into a column window.
 *
 * Example: panel_essential_nt35597.layout=window on the kernel command line.
 */

#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/regulator/consumer.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/string.h>

#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>
#include <drm/drm_probe_helper.h>
#include <drm/display/drm_dsc.h>
#include <drm/display/drm_dsc_helper.h>

#include <video/mipi_display.h>

#define NT35597_WIDTH		1440	/* DDIC columns */
#define NT35597_HEIGHT		2560
#define NT35597_WIDTH_MM	66	/* downstream, for 1440 columns */
#define NT35597_HEIGHT_MM	129

enum essential_nt35597_layout {
	LAYOUT_FULL,
	LAYOUT_CENTER,
	LAYOUT_WINDOW,
};

static const char * const essential_nt35597_layout_names[] = {
	[LAYOUT_FULL] = "full",
	[LAYOUT_CENTER] = "center",
	[LAYOUT_WINDOW] = "window",
};

static char *param_layout = "center";
module_param_named(layout, param_layout, charp, 0444);
MODULE_PARM_DESC(layout, "full, center (default) or window; see the driver source");

static int param_width = 1312;
module_param_named(width, param_width, int, 0444);
MODULE_PARM_DESC(width, "Visible width for center/window, even (default 1312)");

static int param_slices = 2;
module_param_named(slices, param_slices, int, 0444);
MODULE_PARM_DESC(slices, "DSC slices per line for window, even (default 2)");

static int param_slice_height = 32;
module_param_named(slice_height, param_slice_height, int, 0444);
MODULE_PARM_DESC(slice_height, "DSC slice height, divides 2560 (default 32)");

static const char * const essential_nt35597_supply_names[] = {
	"vddio",	/* pm8998 l14 */
	"avdd",		/* pmi8998 LAB */
	"avee",		/* pmi8998 IBB */
};

struct essential_nt35597 {
	struct drm_panel panel;
	struct mipi_dsi_device *dsi;
	struct drm_dsc_config dsc;
	struct gpio_desc *reset_gpio;
	struct regulator_bulk_data supplies[ARRAY_SIZE(essential_nt35597_supply_names)];

	enum essential_nt35597_layout layout;
	struct drm_display_mode mode;
	u16 col_start, col_end;		/* DDIC column window (layout window) */

	/* debugfs: last DCS read and whether debug transfers use LP mode */
	struct mutex dbg_lock;
	bool dbg_lpm;
	u8 dbg_cmd;
	ssize_t dbg_ret;
	u8 dbg_buf[16];
};

static inline struct essential_nt35597 *to_essential_nt35597(struct drm_panel *panel)
{
	return container_of(panel, struct essential_nt35597, panel);
}

/*
 * Raw DCS read that does not touch dsi->mode_flags, so it can run from debugfs
 * while a commit is in progress. A reply needs a bus turnaround, so a successful
 * read proves the panel is receiving commands (CMD_DMA_DONE alone does not).
 */
static ssize_t essential_nt35597_dcs_read_raw(struct essential_nt35597 *ctx, u8 cmd,
					      u8 *buf, size_t len, bool lpm)
{
	struct mipi_dsi_device *dsi = ctx->dsi;
	const struct mipi_dsi_host_ops *ops = dsi->host->ops;
	u8 size[2] = { len & 0xff, len >> 8 };
	struct mipi_dsi_msg msg = {
		.channel = dsi->channel,
		.type = MIPI_DSI_SET_MAXIMUM_RETURN_PACKET_SIZE,
		.tx_buf = size,
		.tx_len = sizeof(size),
		.flags = lpm ? MIPI_DSI_MSG_USE_LPM : 0,
	};
	ssize_t ret;

	if (!ops || !ops->transfer)
		return -ENOSYS;

	ret = ops->transfer(dsi->host, &msg);
	if (ret < 0)
		return ret;

	msg = (struct mipi_dsi_msg) {
		.channel = dsi->channel,
		.type = MIPI_DSI_DCS_READ,
		.tx_buf = &cmd,
		.tx_len = 1,
		.rx_buf = buf,
		.rx_len = len,
		.flags = lpm ? MIPI_DSI_MSG_USE_LPM : 0,
	};

	return ops->transfer(dsi->host, &msg);
}

static void essential_nt35597_reset(struct essential_nt35597 *ctx)
{
	/* qcom,mdss-dsi-reset-sequence = <1 1>, <0 1>, <1 20> */
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	usleep_range(1000, 2000);
	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	usleep_range(1000, 2000);
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	msleep(20);
}

static int essential_nt35597_on(struct essential_nt35597 *ctx)
{
	struct mipi_dsi_device *dsi = ctx->dsi;
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = dsi };
	struct drm_dsc_picture_parameter_set pps;

	dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xff, 0x10);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xfb, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xe5, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xba, 0x03);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x35, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xbb, 0x10);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb0, 0x03);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc0, 0x03);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xff, 0xe0);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xfb, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6b, 0x3d);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6c, 0x3d);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6d, 0x3d);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6e, 0x3d);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x3d);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x35, 0x02);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x36, 0x72);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x37, 0x10);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x08, 0xc0);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xff, 0x10);
	mipi_dsi_dcs_exit_sleep_mode_multi(&dsi_ctx);
	mipi_dsi_msleep(&dsi_ctx, 112);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xff, 0x24);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xfb, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc4, 0x20);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc5, 0x36);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc6, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc7, 0x02);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc9, 0xff);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xca, 0xa9);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xff, 0x10);
	mipi_dsi_dcs_set_display_on_multi(&dsi_ctx);
	mipi_dsi_msleep(&dsi_ctx, 48);

	/* Downstream enables compression and sends the PPS after the on sequence */
	mipi_dsi_compression_mode_multi(&dsi_ctx, true);
	drm_dsc_pps_payload_pack(&pps, &ctx->dsc);
	mipi_dsi_picture_parameter_set_multi(&dsi_ctx, &pps);

	/* Frames (narrower than the DDIC) go to the visible columns only */
	if (ctx->layout == LAYOUT_WINDOW) {
		mipi_dsi_dcs_set_column_address_multi(&dsi_ctx, ctx->col_start,
						      ctx->col_end);
		mipi_dsi_dcs_set_page_address_multi(&dsi_ctx, 0,
						    NT35597_HEIGHT - 1);
	}

	if (!dsi_ctx.accum_err) {
		u8 mode = 0;
		ssize_t ret;

		/* Expect 0x9c: booster on, sleep out, normal mode, display on */
		ret = essential_nt35597_dcs_read_raw(ctx, MIPI_DCS_GET_POWER_MODE,
						     &mode, 1, true);
		dev_info(&dsi->dev, "power mode after init: 0x%02x (read returned %zd)\n",
			 mode, ret);
	}

	dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;

	return dsi_ctx.accum_err;
}

static int essential_nt35597_off(struct essential_nt35597 *ctx)
{
	struct mipi_dsi_device *dsi = ctx->dsi;
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = dsi };

	dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;

	mipi_dsi_dcs_set_display_off_multi(&dsi_ctx);
	mipi_dsi_msleep(&dsi_ctx, 17);
	mipi_dsi_dcs_enter_sleep_mode_multi(&dsi_ctx);
	mipi_dsi_msleep(&dsi_ctx, 67);

	return dsi_ctx.accum_err;
}

static int essential_nt35597_prepare(struct drm_panel *panel)
{
	struct essential_nt35597 *ctx = to_essential_nt35597(panel);
	int ret, i;

	/* Downstream enables vddio, LAB, IBB in order, then waits 10 ms */
	for (i = 0; i < ARRAY_SIZE(ctx->supplies); i++) {
		ret = regulator_enable(ctx->supplies[i].consumer);
		if (ret) {
			while (--i >= 0)
				regulator_disable(ctx->supplies[i].consumer);
			return ret;
		}
	}
	msleep(10);

	/* The DSI host has computed the DSC parameters for the link mode */
	if (ctx->dsc.pic_width != ctx->dsc.slice_width * ctx->dsc.slice_count)
		dev_warn(panel->dev, "DSC picture width %u, expected %u\n",
			 ctx->dsc.pic_width,
			 ctx->dsc.slice_width * ctx->dsc.slice_count);

	essential_nt35597_reset(ctx);

	ret = essential_nt35597_on(ctx);
	if (ret < 0) {
		gpiod_set_value_cansleep(ctx->reset_gpio, 1);
		regulator_bulk_disable(ARRAY_SIZE(ctx->supplies), ctx->supplies);
		return ret;
	}

	return 0;
}

static int essential_nt35597_unprepare(struct drm_panel *panel)
{
	struct essential_nt35597 *ctx = to_essential_nt35597(panel);
	int ret;

	ret = essential_nt35597_off(ctx);
	if (ret < 0)
		dev_err(panel->dev, "Failed to un-initialize panel: %d\n", ret);

	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	regulator_bulk_disable(ARRAY_SIZE(ctx->supplies), ctx->supplies);

	return 0;
}

static int essential_nt35597_get_modes(struct drm_panel *panel,
				       struct drm_connector *connector)
{
	struct essential_nt35597 *ctx = to_essential_nt35597(panel);

	return drm_connector_helper_get_modes_fixed(connector, &ctx->mode);
}

/* Parse up to @max whitespace-separated hex bytes */
static int essential_nt35597_parse_hex(const char __user *ubuf, size_t count,
				       u8 *out, int max)
{
	char *kbuf, *p, *tok;
	int n = 0, ret = 0;

	kbuf = memdup_user_nul(ubuf, count);
	if (IS_ERR(kbuf))
		return PTR_ERR(kbuf);

	p = kbuf;
	while ((tok = strsep(&p, " \t\n")) != NULL) {
		if (!*tok)
			continue;
		if (n == max) {
			ret = -E2BIG;
			break;
		}
		ret = kstrtou8(tok, 16, &out[n]);
		if (ret)
			break;
		n++;
	}

	kfree(kbuf);

	return ret ? ret : n;
}

static int essential_nt35597_dcs_read_show(struct seq_file *m, void *data)
{
	struct essential_nt35597 *ctx = m->private;
	ssize_t i;

	mutex_lock(&ctx->dbg_lock);
	seq_printf(m, "cmd 0x%02x ret %zd:", ctx->dbg_cmd, ctx->dbg_ret);
	for (i = 0; i < ctx->dbg_ret && i < ARRAY_SIZE(ctx->dbg_buf); i++)
		seq_printf(m, " %02x", ctx->dbg_buf[i]);
	seq_putc(m, '\n');
	mutex_unlock(&ctx->dbg_lock);

	return 0;
}

static int essential_nt35597_dcs_read_open(struct inode *inode, struct file *file)
{
	return single_open(file, essential_nt35597_dcs_read_show, inode->i_private);
}

/* echo "<cmd> [len]" (hex) > dcs_read; cat dcs_read */
static ssize_t essential_nt35597_dcs_read_write(struct file *file,
						const char __user *ubuf,
						size_t count, loff_t *ppos)
{
	struct seq_file *m = file->private_data;
	struct essential_nt35597 *ctx = m->private;
	u8 args[2] = { 0, 1 };
	ssize_t ret;
	int n;

	n = essential_nt35597_parse_hex(ubuf, count, args, ARRAY_SIZE(args));
	if (n < 0)
		return n;
	if (n == 0 || args[1] == 0 || args[1] > ARRAY_SIZE(ctx->dbg_buf))
		return -EINVAL;

	mutex_lock(&ctx->dbg_lock);
	memset(ctx->dbg_buf, 0, sizeof(ctx->dbg_buf));
	ret = essential_nt35597_dcs_read_raw(ctx, args[0], ctx->dbg_buf, args[1],
					     ctx->dbg_lpm);
	ctx->dbg_cmd = args[0];
	ctx->dbg_ret = ret;
	mutex_unlock(&ctx->dbg_lock);

	return ret < 0 ? ret : count;
}

static const struct file_operations essential_nt35597_dcs_read_fops = {
	.owner = THIS_MODULE,
	.open = essential_nt35597_dcs_read_open,
	.read = seq_read,
	.write = essential_nt35597_dcs_read_write,
	.llseek = seq_lseek,
	.release = single_release,
};

/* echo "<cmd> [params...]" (hex) > dcs_write sends one DCS packet */
static ssize_t essential_nt35597_dcs_write_write(struct file *file,
						 const char __user *ubuf,
						 size_t count, loff_t *ppos)
{
	struct essential_nt35597 *ctx = file->private_data;
	struct mipi_dsi_device *dsi = ctx->dsi;
	const struct mipi_dsi_host_ops *ops = dsi->host->ops;
	struct mipi_dsi_msg msg = { .channel = dsi->channel };
	u8 buf[16];
	ssize_t ret;
	int n;

	n = essential_nt35597_parse_hex(ubuf, count, buf, ARRAY_SIZE(buf));
	if (n < 0)
		return n;
	if (n == 0)
		return -EINVAL;
	if (!ops || !ops->transfer)
		return -ENOSYS;

	if (n == 1)
		msg.type = MIPI_DSI_DCS_SHORT_WRITE;
	else if (n == 2)
		msg.type = MIPI_DSI_DCS_SHORT_WRITE_PARAM;
	else
		msg.type = MIPI_DSI_DCS_LONG_WRITE;
	msg.tx_buf = buf;
	msg.tx_len = n;

	mutex_lock(&ctx->dbg_lock);
	if (ctx->dbg_lpm)
		msg.flags |= MIPI_DSI_MSG_USE_LPM;
	ret = ops->transfer(dsi->host, &msg);
	mutex_unlock(&ctx->dbg_lock);

	return ret < 0 ? ret : count;
}

static const struct file_operations essential_nt35597_dcs_write_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.write = essential_nt35597_dcs_write_write,
	.llseek = noop_llseek,
};

static void essential_nt35597_debugfs_init(struct drm_panel *panel, struct dentry *root)
{
	struct essential_nt35597 *ctx = to_essential_nt35597(panel);

	debugfs_create_file("dcs_read", 0600, root, ctx, &essential_nt35597_dcs_read_fops);
	debugfs_create_file("dcs_write", 0200, root, ctx, &essential_nt35597_dcs_write_fops);
	debugfs_create_bool("lpm", 0600, root, &ctx->dbg_lpm);
}

static const struct drm_panel_funcs essential_nt35597_panel_funcs = {
	.prepare = essential_nt35597_prepare,
	.unprepare = essential_nt35597_unprepare,
	.get_modes = essential_nt35597_get_modes,
	.debugfs_init = essential_nt35597_debugfs_init,
};

/* Pick the layout from the module parameters; fall back to full if invalid */
static void essential_nt35597_setup_layout(struct essential_nt35597 *ctx)
{
	struct device *dev = &ctx->dsi->dev;
	struct drm_display_mode *mode = &ctx->mode;
	int layout, width = NT35597_WIDTH, slices = 2;
	int slice_height = param_slice_height;
	bool valid = true;

	layout = sysfs_match_string(essential_nt35597_layout_names, param_layout);
	if (layout < 0) {
		valid = false;
	} else if (layout != LAYOUT_FULL) {
		width = param_width;
		if (width < 64 || width > NT35597_WIDTH || width % 2)
			valid = false;
	}

	if (valid && layout == LAYOUT_WINDOW) {
		/* each of the two DSC encoders takes half of the slices */
		slices = param_slices;
		if (slices < 2 || slices % 2 || width % slices)
			valid = false;
	}

	if (slice_height < 1 || NT35597_HEIGHT % slice_height)
		valid = false;

	if (!valid) {
		dev_warn(dev, "invalid layout=%s width=%d slices=%d slice_height=%d, using full\n",
			 param_layout, param_width, param_slices, param_slice_height);
		layout = LAYOUT_FULL;
		width = NT35597_WIDTH;
		slices = 2;
		slice_height = 32;
	}

	ctx->layout = layout;
	ctx->col_start = (NT35597_WIDTH - width) / 2;
	ctx->col_end = ctx->col_start + width - 1;

	/* downstream config2: two DSC encoders merged onto one DSI link */
	ctx->dsc.dsc_version_major = 1;
	ctx->dsc.dsc_version_minor = 1;
	ctx->dsc.slice_height = slice_height;
	if (layout == LAYOUT_WINDOW) {
		ctx->dsc.slice_count = slices;
		ctx->dsc.slice_width = width / slices;
	} else {
		/* for center wider than the mode: the DSI host centers the mode */
		ctx->dsc.slice_count = 2;
		ctx->dsc.slice_width = NT35597_WIDTH / 2;
	}
	ctx->dsc.bits_per_component = 8;
	ctx->dsc.bits_per_pixel = 8 << 4; /* 4 fractional bits */
	ctx->dsc.block_pred_enable = true;

	/* Downstream porches; command mode, so they only set the clock */
	mode->hdisplay = width;
	mode->hsync_start = mode->hdisplay + 104;
	mode->hsync_end = mode->hsync_start + 12;
	mode->htotal = mode->hsync_end + 36;
	mode->vdisplay = NT35597_HEIGHT;
	mode->vsync_start = mode->vdisplay + 156;
	mode->vsync_end = mode->vsync_start + 1;
	mode->vtotal = mode->vsync_end + 155;
	mode->clock = mode->htotal * mode->vtotal * 60 / 1000;
	mode->width_mm = DIV_ROUND_CLOSEST(NT35597_WIDTH_MM * width, NT35597_WIDTH);
	mode->height_mm = NT35597_HEIGHT_MM;
	mode->type = DRM_MODE_TYPE_DRIVER;

	dev_info(dev, "layout %s: %dx%d mode, DSC %u x %ux%u slices, DDIC columns %u..%u\n",
		 essential_nt35597_layout_names[layout], width, NT35597_HEIGHT,
		 ctx->dsc.slice_count, ctx->dsc.slice_width, ctx->dsc.slice_height,
		 layout == LAYOUT_WINDOW ? ctx->col_start : 0,
		 layout == LAYOUT_WINDOW ? ctx->col_end : NT35597_WIDTH - 1);
}

static int essential_nt35597_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct essential_nt35597 *ctx;
	int ret, i;

	ctx = devm_drm_panel_alloc(dev, struct essential_nt35597, panel,
				   &essential_nt35597_panel_funcs,
				   DRM_MODE_CONNECTOR_DSI);
	if (IS_ERR(ctx))
		return PTR_ERR(ctx);

	ret = devm_mutex_init(dev, &ctx->dbg_lock);
	if (ret)
		return ret;
	ctx->dbg_lpm = true;

	for (i = 0; i < ARRAY_SIZE(ctx->supplies); i++)
		ctx->supplies[i].supply = essential_nt35597_supply_names[i];

	ret = devm_regulator_bulk_get(dev, ARRAY_SIZE(ctx->supplies),
				      ctx->supplies);
	if (ret < 0)
		return dev_err_probe(dev, ret, "Failed to get regulators\n");

	/* The bootloader leaves the panel on (continuous splash) */
	ctx->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(ctx->reset_gpio),
				     "Failed to get reset-gpios\n");

	ctx->dsi = dsi;
	mipi_dsi_set_drvdata(dsi, ctx);

	essential_nt35597_setup_layout(ctx);

	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_LPM | MIPI_DSI_CLOCK_NON_CONTINUOUS |
			  MIPI_DSI_MODE_DSC_ALL_SLICES_IN_PKT;

	ctx->panel.prepare_prev_first = true;

	ret = drm_panel_of_backlight(&ctx->panel);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to get backlight\n");

	drm_panel_add(&ctx->panel);

	dsi->dsc = &ctx->dsc;

	ret = mipi_dsi_attach(dsi);
	if (ret < 0) {
		dev_err_probe(dev, ret, "Failed to attach to DSI host\n");
		drm_panel_remove(&ctx->panel);
		return ret;
	}

	return 0;
}

static void essential_nt35597_remove(struct mipi_dsi_device *dsi)
{
	struct essential_nt35597 *ctx = mipi_dsi_get_drvdata(dsi);
	int ret;

	ret = mipi_dsi_detach(dsi);
	if (ret < 0)
		dev_err(&dsi->dev, "Failed to detach from DSI host: %d\n", ret);

	drm_panel_remove(&ctx->panel);
}

static const struct of_device_id essential_nt35597_of_match[] = {
	{ .compatible = "essential,mata-nt35597" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, essential_nt35597_of_match);

static struct mipi_dsi_driver essential_nt35597_driver = {
	.probe = essential_nt35597_probe,
	.remove = essential_nt35597_remove,
	.driver = {
		.name = "panel-essential-nt35597",
		.of_match_table = essential_nt35597_of_match,
	},
};
module_mipi_dsi_driver(essential_nt35597_driver);

MODULE_DESCRIPTION("DRM driver for the NT35597 DSC cmd mode panel of the Essential PH-1");
MODULE_LICENSE("GPL");
