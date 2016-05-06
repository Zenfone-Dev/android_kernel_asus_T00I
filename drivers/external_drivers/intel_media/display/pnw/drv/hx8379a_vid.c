/*
 * Copyright (c)  2012 Intel Corporation
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicensen
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice (including the next
 * paragraph) shall be included in all copies or substantial portions of the
 * Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 *
 */

#include "displays/hx8379a_vid.h"
#include "mdfld_dsi_dpi.h"
#include "mdfld_dsi_pkg_sender.h"
#include <linux/gpio.h>
#include "psb_drv.h"
#include <linux/lnw_gpio.h>
#include <asm/intel_scu_pmic.h>
#include <linux/HWVersion.h>
#include "asus_panel_id.h"

#define HX8379A_DEBUG 1

extern int Read_HW_ID(void);
static int board_hw_id=0;

#define HX8379A_PANEL_NAME	"HX8379A"
//#define HX8379A_PANEL_NAME	"OTM8018B"

static bool splendid_init = false;

/*
 * GPIO pin definition
 */
#define LCD_BL_EN 0x7E           //PMIC:BACKLIGHT_EN
#define LCD_LOGIC_PWR_EN 0x7F    //PMIC:PANEL_EN
#define LCD_ANALONG_PWR_EN 108   //CPU:GP_CORE_012

#define LCD_ANALONG_PWR_EN_ER 0xda    //PMIC:VEMMC2CNT for ER sku
#define LCD_ANALONG_PWR_EN_ER_OFF 0x4
#define LCD_ANALONG_PWR_EN_ER_AUTO 0x6

//change PANEL_ID , RESET_INNO for SR
#define LCM_ID_ER 54             //LCM_ID : GP_SPI_3_CLK = GP_AON_054 = 54
#define RESET_INNO_SR 116        //GP_CORE_020 = 20 + 96 =116

//MIPI Control
//#define MIPI_SW_SEL 159 //GP_CORE_063 = 63 + 96 = 159
//#define MIPI_PWR_EN 40 // GP_XDP_BLK_DP = gp_aon_040 = 40

#define PWMCTRL_REG 0xffae9000
#define PWMCTRL_SIZE 0x80
static void __iomem *pwmctrl_mmio;
#define PWM_ENABLE_GPIO 49
#define PWM_BASE_UNIT 0x444 //5,000Hz

union sst_pwmctrl_reg {
	struct {
		u32 pwmtd:8;
		u32 pwmbu:22;
		u32 pwmswupdate:1;
		u32 pwmenable:1;
	} part;
	u32 full;
};

static struct mdfld_dsi_config *hx8379a_dsi_config;

#define CMD_SIZE(x) (sizeof((x)) / sizeof((x)[0]))

//#if 0
static u8 hx8379_cm1[] = {0xB9,0xFF,0x83,0x79};
static u8 hx8379_cm2[] = {0xBA,0x71,0x93,0x0,0x16,0xac,0x0,0x18,0xff,0x0f,0x29,0x3,0x21,0x23,0x25,0x20,0x6};
static u8 hx8379_cm3[] = {0xB1,0x00,0x50,0x44,0xEA,0xd9,0x08,0x11,0x11,0x11,
 0x27,0x2F,0x9A,0x1A,0x42,0x0B,0x6E,0xF1,0x00,0xE6};
static u8 hx8379_cm4[] = {0xB2,0x00,0x00,0x3C,0x08,0x04,0x19,0x22,0x00,0xFF,
           0x08,0x04,0x19,0x20};
static u8 hx8379_cm5[] = {0xB4,0x82,0x08,0x00,0x32,0x10,0x03,0x32,0x13,0x70,
           0x32,0x10,0x08,0x37,0x01,0x28,0x07,0x37,0x08,0x3B,
           0x20,0x3F,0x3F,0x08,0x00,0x40,0x08,0x28,0x08,0x30,
           0x30,0x04};
static u8 hx8379_cm6[] = {0xD5,0x00,0x00,0x0A,0x00,0x01,0x05,0x00,0x03,0x00,
           0x88,0x88,0x88,0x88,0x23,0x01,0x67,0x45,0x02,0x13,
           0x88,0x88,0x88,0x88,0x88,0x88,0x88,0x88,0x88,0x88,
           0x54,0x76,0x10,0x32,0x31,0x20,0x88,0x88,0x88,0x88,
           0x88,0x88,0x00,0x00,0x00,0x00,0x00,0x00};
static u8 hx8379_cm7[] = {0xE0,0x79,0x00,0x00,0x02,0x2C,0x2F,0x3F,0x29,0x49,
           0x07,0x0E,0x0F,0x14,0x16,0x14,0x16,0x13,0x18,0x00,
           0x00,0x02,0x2C,0x2F,0x3F,0x27,0x49,0x07,0x0E,0x0F,
           0x14,0x16,0x14,0x16,0x13,0x18};
static u8 hx8379_cm8[] = {0xCC,0x02};
static u8 hx8379_cm9[] = {0xB6,0x00,0xa0,0x00,0xa0};
static u8 hx8379_cm_off[] = {0xB9,0xFF,0x83,0x00};
static u8 hx8379_cm10[] = {0x11};
static u8 hx8379_cm11[] = {0x29};

static u8 hx8479a_0xa[] = {0xa,0x1c};
static u8 hx8479a_0x9[] = {0x9,0x80,0x73,0x04,0x00};

//#endif

#if 0
static u8 hx8379_cm1[] = {0xB9,0xFF,0x83,0x79};
static u8 hx8379_cm2[] = {0xBA,0x51};
static u8 hx8379_cm3[] = {0xB1,0x00 ,0x50 ,0x24 ,0xEA ,0x90 ,0x08 ,0x11 ,0x11 ,0x71 ,0x2F ,0x37 ,0x9A ,0x1A ,0x42 ,0x0B ,0x7A ,0xF1 ,0x00 ,0xE6};
static u8 hx8379_cm4[] = {0xB2,0x00 ,0x00 ,0x3C ,0x08 ,0x0C ,0x19 ,0x52 ,0x00 ,0xFF ,0x08 ,0x0C ,0x19 ,0x20};
static u8 hx8379_cm5[] = {0xB4 ,0x82 ,0x08 ,0x00 ,0x32 ,0x10 ,0x05 ,0x32 ,0x13 ,0x31 ,0x32 ,0x10 ,0x08 ,0x37 ,0x01 ,0x20 ,0x07 ,0x37 ,0x08 ,0x40 ,0x18 ,0x48 ,0x08 ,0x08 ,0x00 ,0x40 ,0x08 ,0x28 ,0x08 ,0x30 ,0x30 ,0x04};
static u8 hx8379_cm6[] = {0xD5 ,0x00 ,0x00 ,0x0A ,0x00 ,0x01 ,0x05 ,0x00 ,0x0A ,0x00 ,0x88 ,0x88 ,0x89 ,0x88 ,0x23 ,0x01 ,0x67 ,0x45 ,0x88 ,0x01 ,0x88 ,0x45 ,0x88 ,0x88 ,0x88 ,0x88 ,0x88 ,0x88 ,0x88
,0x88 ,0x54 ,0x76 ,0x10 ,0x32 ,0x88 ,0x54 ,0x88 ,0x10 ,0x88 ,0x88 ,0x88 ,0x88 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00};
static u8 hx8379_cm7[] = {0xE0 ,0x79 ,0x07 ,0x12 ,0x14 ,0x3F ,0x3F ,0x3F ,0x25 ,0x4F ,0x06 ,0x0C ,0x0E ,0x12 ,0x17 ,0x12 ,0x13 ,0x14 ,0x1F ,0x07 ,0x12 ,0x14 ,0x3F ,0x3F ,0x3F ,0x25 ,0x4F ,0x06
,0x0C ,0x0E ,0x12 ,0x17 ,0x12 ,0x13 ,0x14 ,0x1F};
static u8 hx8379_cm8[] = {0xCC,0x02};
static u8 hx8379_cm9[] = {0xB6,0x00,0xa3,0x00,0xa3};
static u8 hx8379_cm10[] = {0x11};
static u8 hx8379_cm11[] = {0x29};
#endif

static int pwm_configure(int duty)
{
	union sst_pwmctrl_reg pwmctrl;

	/*Read the PWM register to make sure there is no pending
	*update.
	*/
	pwmctrl.full = readl(pwmctrl_mmio);

	/*check pwnswupdate bit */
	if (pwmctrl.part.pwmswupdate)
		return -EBUSY;
	pwmctrl.part.pwmswupdate = 0x1;
	pwmctrl.part.pwmbu = PWM_BASE_UNIT;
	pwmctrl.part.pwmtd = duty;
	writel(pwmctrl.full,  pwmctrl_mmio);

	return 0;
}


static void pwm_enable(){
	union sst_pwmctrl_reg pwmctrl;

	lnw_gpio_set_alt(PWM_ENABLE_GPIO, LNW_ALT_2);

	/*Enable the PWM by setting PWM enable bit to 1 */
	pwmctrl.full = readl(pwmctrl_mmio);
	pwmctrl.part.pwmenable = 1;
	writel(pwmctrl.full, pwmctrl_mmio);
}

static void pwm_disable(){
	union sst_pwmctrl_reg pwmctrl;
	/*setting PWM enable bit to 0 */
	pwmctrl.full = readl(pwmctrl_mmio);
	pwmctrl.part.pwmenable = 0;
	writel(pwmctrl.full,  pwmctrl_mmio);

	gpio_set_value(PWM_ENABLE_GPIO, 0);
	lnw_gpio_set_alt(PWM_ENABLE_GPIO, 0);
}

//normal
static int send_mipi_cmd2(
				struct mdfld_dsi_pkg_sender * sender,
				u8 * data,
				u32 len){
	int r=0,i;
	sender->status = MDFLD_DSI_PKG_SENDER_FREE;

#if 0
	printk("%s : %d len = %d,0x%02x",__func__,r,len,data[0]);
	for(i=1;i<len;i++){
		printk(",0x%02x", data[i]);
	}
	printk("\n");
#endif

	switch(len){
		case 1:
			mdfld_dsi_send_mcs_short_lp(sender, data[0],0, 0, 0);
			break;
		case 2:
			mdfld_dsi_send_mcs_short_lp(sender, data[0], data[1], 1, 0);;
			break;
		default:

			r = mdfld_dsi_send_mcs_long_lp(sender, data, len, 0);

			break;
	}
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL){
		printk("%s : MDFLD_DSI_CONTROL_ABNORMAL\n",__func__);
		return -EIO;
	}
	else{
		return 0;
	}

}
#define LP_RETRY 1

#if 0
static int read_mipi_reg(
				struct mdfld_dsi_pkg_sender * sender,
				u8 reg,
				u32 len){
	int ret =0;
	int retry_times=LP_RETRY;
	u8 data2[60]={0};
	int i,r=0;

	sender->status = MDFLD_DSI_PKG_SENDER_FREE;

	r = mdfld_dsi_read_mcs_lp(sender,reg,data2, len);

	printk("%s : len=%d, r=%d,0x%02x,",__func__,len,r,reg);
	for(i=0;i<len;i++){
		printk(",0x%02x", data2[i]);
	}
	printk("\n");
	return 0;

}
#endif

//compare
static int compare_mipi_reg(
				struct mdfld_dsi_pkg_sender * sender,
				u8 * data,
				u32 len,
				u8 * compare_data){
	int ret =0;
	int retry_times=LP_RETRY;
	u8 data2[60]={0};
	int i,r=0;

	//temp for read fail
	//return 0;

	sender->status = MDFLD_DSI_PKG_SENDER_FREE;

	r = mdfld_dsi_read_mcs_lp(sender,data[0],data2, len);

	ret = memcmp(data2, compare_data+1, len);
	if (!ret){
		return 0;
	}else{
		printk("%s : len=%d, r=%d,0x%02x,",__func__,len,r,data[0]);
		for(i=0;i<len;i++){
			printk(",0x%02x", data2[i]);
		}
		printk(" (compare fail, retry again)\n");
		return -EIO;
	}
}




static int hx8379a_mipi_cmd_setting(struct mdfld_dsi_config *dsi_config){

	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);
    int x0=0;

	int r = 0;
	u8 data[10]={0};
	u8 rdata;
	int retry_times;
	int reset_inno;
	static u8 read_data = 0;
	int read_ret = 0;
	read_data = 0x0;

	reset_inno = RESET_INNO_SR;


	retry_times=LP_RETRY;

hx8379a_retry:

	retry_times--;

    mdfld_dsi_read_gen_lp(sender,0xa,0x0,1,&read_data,1);

	//RESX : L > H > L > H

	if (gpio_direction_output(reset_inno, 0))
		gpio_set_value_cansleep(reset_inno, 0);

	usleep_range(1000, 1000);

	if (gpio_direction_output(reset_inno, 1))
		gpio_set_value_cansleep(reset_inno, 1);

	usleep_range(1000, 1000);

	if (gpio_direction_output(reset_inno, 0))
		gpio_set_value_cansleep(reset_inno, 0);

	usleep_range(5000, 5000);

	if (gpio_direction_output(reset_inno, 1))
		gpio_set_value_cansleep(reset_inno, 1);

	usleep_range(10000, 10000);


	send_mipi_cmd2(sender,hx8379_cm1,CMD_SIZE(hx8379_cm1));
	r = compare_mipi_reg(sender,hx8379_cm1,CMD_SIZE(hx8379_cm1)-1,hx8379_cm1);
	if(r) return -EIO;

	send_mipi_cmd2(sender,hx8379_cm2,CMD_SIZE(hx8379_cm2));
	r = compare_mipi_reg(sender,hx8379_cm2,CMD_SIZE(hx8379_cm2)-1,hx8379_cm2);
	if(r) return -EIO;

	send_mipi_cmd2(sender,hx8379_cm3,CMD_SIZE(hx8379_cm3));
	r = compare_mipi_reg(sender,hx8379_cm3,CMD_SIZE(hx8379_cm3)-1,hx8379_cm3);
	if(r) return -EIO;

	send_mipi_cmd2(sender,hx8379_cm4,CMD_SIZE(hx8379_cm4));
	r = compare_mipi_reg(sender,hx8379_cm4,CMD_SIZE(hx8379_cm4)-1,hx8379_cm4);
	if(r) return -EIO;

	send_mipi_cmd2(sender,hx8379_cm5,CMD_SIZE(hx8379_cm5));
	r = compare_mipi_reg(sender,hx8379_cm5,CMD_SIZE(hx8379_cm5)-1,hx8379_cm5);
	if(r) return -EIO;

	send_mipi_cmd2(sender,hx8379_cm6,CMD_SIZE(hx8379_cm6));
	r = compare_mipi_reg(sender,hx8379_cm6,CMD_SIZE(hx8379_cm6)-1,hx8379_cm6);
	if(r) return -EIO;

	send_mipi_cmd2(sender,hx8379_cm7,CMD_SIZE(hx8379_cm7));
	r = compare_mipi_reg(sender,hx8379_cm7,CMD_SIZE(hx8379_cm7)-1,hx8379_cm7);
	if(r) return -EIO;

	send_mipi_cmd2(sender,hx8379_cm8,CMD_SIZE(hx8379_cm8));
	r = compare_mipi_reg(sender,hx8379_cm8,CMD_SIZE(hx8379_cm8)-1,hx8379_cm8);
	if(r) return -EIO;

	if((board_hw_id == HW_ID_SR1)||(board_hw_id == HW_ID_SR2)){
		printk("[DISPLAY] hx8379a_mipi_cmd_setting: set VCOM default value.\n");
		send_mipi_cmd2(sender,hx8379_cm9,CMD_SIZE(hx8379_cm9));
		r = compare_mipi_reg(sender,hx8379_cm9,CMD_SIZE(hx8379_cm9)-1,hx8379_cm9);
		if(r) return -EIO;
	}

	//eng mode off
	send_mipi_cmd2(sender,hx8379_cm_off,CMD_SIZE(hx8379_cm_off));

	//check eng mode off
	read_data = 0xff;
	mdfld_dsi_read_gen_lp(sender,0xba,0x0,1,&read_data,1);
	if(read_data!=0x00) return -EIO;



	return 0;

}

static void
hx8379a_vid_dsi_controller_init(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_hw_context *hw_ctx = &dsi_config->dsi_hw_context;

	struct drm_device *dev = dsi_config->dev;
	struct csc_setting csc = {	.pipe = 0,
								.type = CSC_REG_SETTING,
								.enable_state = true,
								.data_len = CSC_REG_COUNT,
								.data.csc_reg_data = {
									0x400, 0x0, 0x4000000, 0x0, 0x0, 0x400}
							 };
	struct gamma_setting gamma = {	.pipe = 0,
									.type = GAMMA_REG_SETTING,
									.enable_state = true,
									.data_len = GAMMA_10_BIT_TABLE_COUNT,
									.gamma_tableX100 = {
										0x000000, 0x020202, 0x040404, 0x060606,
										0x080808, 0x0A0A0A, 0x0C0C0C, 0x0E0E0E,
										0x101010, 0x121212, 0x141414, 0x161616,
										0x181818, 0x1A1A1A, 0x1C1C1C, 0x1E1E1E,
										0x202020, 0x222222, 0x242424, 0x262626,
										0x282828, 0x2A2A2A, 0x2C2C2C, 0x2E2E2E,
										0x303030, 0x323232, 0x343434, 0x363636,
										0x383838, 0x3A3A3A, 0x3C3C3C, 0x3E3E3E,
										0x404040, 0x424242, 0x444444, 0x464646,
										0x484848, 0x4A4A4A, 0x4C4C4C, 0x4E4E4E,
										0x505050, 0x525252, 0x545454, 0x565656,
										0x585858, 0x5A5A5A, 0x5C5C5C, 0x5E5E5E,
										0x606060, 0x626262, 0x646464, 0x666666,
										0x686868, 0x6A6A6A, 0x6C6C6C, 0x6E6E6E,
										0x707070, 0x727272, 0x747474, 0x767676,
										0x787878, 0x7A7A7A, 0x7C7C7C, 0x7E7E7E,
										0x808080, 0x828282, 0x848484, 0x868686,
										0x888888, 0x8A8A8A, 0x8C8C8C, 0x8E8E8E,
										0x909090, 0x929292, 0x949494, 0x969696,
										0x989898, 0x9A9A9A, 0x9C9C9C, 0x9E9E9E,
										0xA0A0A0, 0xA2A2A2, 0xA4A4A4, 0xA6A6A6,
										0xA8A8A8, 0xAAAAAA, 0xACACAC, 0xAEAEAE,
										0xB0B0B0, 0xB2B2B2, 0xB4B4B4, 0xB6B6B6,
										0xB8B8B8, 0xBABABA, 0xBCBCBC, 0xBEBEBE,
										0xC0C0C0, 0xC2C2C2, 0xC4C4C4, 0xC6C6C6,
										0xC8C8C8, 0xCACACA, 0xCCCCCC, 0xCECECE,
										0xD0D0D0, 0xD2D2D2, 0xD4D4D4, 0xD6D6D6,
										0xD8D8D8, 0xDADADA, 0xDCDCDC, 0xDEDEDE,
										0xE0E0E0, 0xE2E2E2, 0xE4E4E4, 0xE6E6E6,
										0xE8E8E8, 0xEAEAEA, 0xECECEC, 0xEEEEEE,
										0xF0F0F0, 0xF2F2F2, 0xF4F4F4, 0xF6F6F6,
										0xF8F8F8, 0xFAFAFA, 0xFCFCFC, 0xFEFEFE,
										0x010000, 0x010000, 0x010000}
								 };

	dsi_config->lane_count = 2;
	dsi_config->lane_config = MDFLD_DSI_DATA_LANE_2_2;
	dsi_config->enable_gamma_csc = ENABLE_GAMMA | ENABLE_CSC;

	hw_ctx->cck_div = 1;
	hw_ctx->pll_bypass_mode = 0;
	hw_ctx->mipi_control = 0x18;
	hw_ctx->intr_en = 0xffffffff;
	hw_ctx->hs_tx_timeout = 0xffffff;
	hw_ctx->lp_rx_timeout = 0xffffff;
	hw_ctx->turn_around_timeout = 0x3f;
	hw_ctx->device_reset_timer = 0xffff;
	hw_ctx->high_low_switch_count = 0x00000013;
	hw_ctx->init_count = 0x7d0;
	hw_ctx->eot_disable = 0x3;
	hw_ctx->lp_byteclk = 0x3;
	hw_ctx->clk_lane_switch_time_cnt = 0x00130009;
	hw_ctx->dphy_param = 0x1f0c3409;

	/* Setup video mode format */
	hw_ctx->video_mode_format = 0xe;

	/* Set up func_prg, RGB888(0x200) */
	hw_ctx->dsi_func_prg = (0x200 | dsi_config->lane_count);

	/* Setup mipi port configuration */
	hw_ctx->mipi = PASS_FROM_SPHY_TO_AFE | dsi_config->lane_config;


	if(!splendid_init){
		if (dsi_config->enable_gamma_csc & ENABLE_CSC) {
			/* setting the tuned csc setting */
			drm_psb_enable_color_conversion = 1;
			mdfld_intel_crtc_set_color_conversion(dev, &csc);
		}

		if (dsi_config->enable_gamma_csc & ENABLE_GAMMA) {
			/* setting the tuned gamma setting */
			drm_psb_enable_gamma = 1;
			mdfld_intel_crtc_set_gamma(dev, &gamma);
		}
	}
	splendid_init = true;

	pr_debug("[DISPLAY] %s: Enter\n", __func__);
}

static int hx8379a_vid_detect(struct mdfld_dsi_config *dsi_config)
{
	int status;
	struct drm_device *dev = dsi_config->dev;
	struct mdfld_dsi_hw_registers *regs = &dsi_config->regs;
	u32 dpll_val, device_ready_val, dphy_val;
	int pipe = dsi_config->pipe;

	if (pipe == 0) {
		/*
		 * FIXME: WA to detect the panel connection status, and need to
		 * implement detection feature with get_power_mode DSI command.
		 */
		if (!ospm_power_using_hw_begin(OSPM_DISPLAY_ISLAND,
			OSPM_UHB_FORCE_POWER_ON)) {
			DRM_ERROR("hw begin failed\n");
			return -EAGAIN;
		}

		dpll_val = REG_READ(regs->dpll_reg);
		device_ready_val = REG_READ(regs->device_ready_reg);
		if ((device_ready_val & DSI_DEVICE_READY) &&
		    (dpll_val & DPLL_VCO_ENABLE)) {
			dsi_config->dsi_hw_context.panel_on = true;
			psb_enable_vblank(dev, pipe);
		} else {
			dsi_config->dsi_hw_context.panel_on = false;
			DRM_INFO("%s: panel is not detected!\n", __func__);
		}

		status = MDFLD_DSI_PANEL_CONNECTED;
		ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);
	} else {
		DRM_INFO("%s: do NOT support dual panel\n", __func__);
		status = MDFLD_DSI_PANEL_DISCONNECTED;
	}

	pr_debug("[DISPLAY] %s: Enter\n", __func__);
	return status;
}

static int hx8379a_vid_gpio_control(int on){

	if(on){

		usleep_range(1000, 2000);

		if((board_hw_id == HW_ID_SR1)||(board_hw_id == HW_ID_SR2)){
			intel_scu_ipc_iowrite8(LCD_LOGIC_PWR_EN,0x1);

			usleep_range(10000, 10000);

			if (gpio_direction_output(LCD_ANALONG_PWR_EN, 1))
				gpio_set_value_cansleep(LCD_ANALONG_PWR_EN, 1);
		}else{
			intel_scu_ipc_iowrite8(LCD_ANALONG_PWR_EN_ER,LCD_ANALONG_PWR_EN_ER_AUTO);
		}
	}else{

		if (gpio_direction_output(RESET_INNO_SR, 0))
			gpio_set_value_cansleep(RESET_INNO_SR, 0);

		usleep_range(1000, 1000);

		if((board_hw_id == HW_ID_SR1)||(board_hw_id == HW_ID_SR2)){

			if (gpio_direction_output(LCD_ANALONG_PWR_EN, 0))
				gpio_set_value_cansleep(LCD_ANALONG_PWR_EN, 0);

			usleep_range(1000, 1000);
			intel_scu_ipc_iowrite8(LCD_LOGIC_PWR_EN,0x0);
		}else{
			intel_scu_ipc_iowrite8(LCD_ANALONG_PWR_EN_ER,LCD_ANALONG_PWR_EN_ER_OFF);
		}

	}

	DRM_INFO("[DISPLAY] %s: Enter\n", __func__);

	return 0;

}

static int hx8379a_vid_power_on(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);
	int err;

	if (!sender) {
		DRM_ERROR("Failed to get DSI packet sender\n");
		return -EINVAL;
	}

	usleep_range(1000, 1200);

	/* Send TURN_ON packet */
	err = mdfld_dsi_send_dpi_spk_pkg_lp(sender,
				    MDFLD_DSI_DPI_SPK_TURN_ON);
	if (err) {
		DRM_ERROR("Failed to send turn on packet\n");
		return err;
	}

	send_mipi_cmd2(sender,hx8379_cm10,CMD_SIZE(hx8379_cm10));
	usleep_range(120000, 120000);
	send_mipi_cmd2(sender,hx8379_cm11,CMD_SIZE(hx8379_cm11));
	usleep_range(5000, 5000);

#if 0
	//check 0x9, 0xa
	r = compare_mipi_reg(sender,hx8479a_0x9,CMD_SIZE(hx8479a_0x9)-1,hx8479a_0x9);
	if(r) return -EIO;

	r = compare_mipi_reg(sender,hx8479a_0xa,CMD_SIZE(hx8479a_0xa)-1,hx8479a_0xa);
	if(r) return -EIO;
#endif

	pwm_enable();

	/* LCD_BL_EN*/
	intel_scu_ipc_iowrite8(LCD_BL_EN,0x1);

	DRM_INFO("[DISPLAY] %s: Enter\n", __func__);
	return 0;
}

static int hx8379a_vid_power_off(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);
	int err;
	int reset_inno;

	if (!sender) {
		DRM_ERROR("Failed to get DSI packet sender\n");
		return -EINVAL;
	}

	pwm_disable();
	/* LCD_BL_EN*/
	intel_scu_ipc_iowrite8(LCD_BL_EN,0x0);

	mdfld_dsi_send_mcs_short_lp(sender,0x10,0x00,0,0);

	usleep_range(40000, 40000);

	/* Send SHUT_DOWN packet */
	err = mdfld_dsi_send_dpi_spk_pkg_lp(sender,
				    MDFLD_DSI_DPI_SPK_SHUT_DOWN);
	if (err) {
		DRM_ERROR("Failed to send turn off packet\n");
		return err;
	}

	hx8379a_vid_gpio_control(0);

	DRM_INFO("[DISPLAY] %s: Enter\n", __func__);

	return 0;
}

static int hx8379a_vid_reset(struct mdfld_dsi_config *dsi_config)
{

	hx8379a_vid_gpio_control(1);

	pr_debug("[DISPLAY] %s: Enter\n", __func__);

	return 0;
}

#define PWM0DUTYCYCLE 0x67
#define DUTY_VALUE_MAX	0x63

static int bl_prev_level = 0;
static int hx8379a_vid_set_brightness(struct mdfld_dsi_config *dsi_config,
					 int level)
{
	int duty_val = 0;
	int ret = 0;
	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);

	if (!sender) {
		DRM_ERROR("Failed to get DSI packet sender\n");
		return -EINVAL;
	}

	duty_val = level==1?0:level;

	pwm_configure(duty_val);

	//add for debug ++
	if(!!bl_prev_level^!!duty_val)
		DRM_INFO("[DISPLAY] brightness level : %d > %d\n", bl_prev_level, duty_val);

	bl_prev_level = duty_val;
	//add for debug --

	PSB_DEBUG_ENTRY("level = %d , duty_val = %d\n", level, duty_val);

	return 0;
}

struct drm_display_mode *hx8379a_vid_get_config_mode(void)
{
	struct drm_display_mode *mode;

	mode = kzalloc(sizeof(*mode), GFP_KERNEL);
	if (!mode)
		return NULL;

	mode->hdisplay = 480;
	mode->vdisplay = 800;

	mode->hsync_start = mode->hdisplay + 30;
	mode->hsync_end = mode->hsync_start + 18;
	mode->htotal = mode->hsync_end + 18;
	mode->vsync_start = mode->vdisplay + 14;
	mode->vsync_end = mode->vsync_start + 3;
	mode->vtotal = mode->vsync_end + 4;

	mode->vrefresh = 60;
	mode->clock = mode->vrefresh * mode->vtotal * mode->htotal / 1000;
	mode->type |= DRM_MODE_TYPE_PREFERRED;

	drm_mode_set_name(mode);
	drm_mode_set_crtcinfo(mode, 0);

	pr_debug("[DISPLAY] %s: Enter\n", __func__);
	return mode;
}

static void hx8379a_vid_get_panel_info(int pipe, struct panel_info *pi)
{
	pi->width_mm = 52;
	pi->height_mm = 86;
}

static int hx8379a_vid_gpio_init(void)
{


    gpio_request(LCM_ID_ER,"LCM_ID_ER");

	gpio_request(RESET_INNO_SR,"RESET_INNO_SR");

	if((board_hw_id == HW_ID_SR1)||(board_hw_id == HW_ID_SR2)){
		gpio_request(LCD_ANALONG_PWR_EN,"LCD_ANALONG_PWR_EN");
	}

#if 0
	gpio_request(MIPI_SW_SEL,"MIPI_SW_SEL");
	gpio_request(MIPI_PWR_EN,"MIPI_PWR_EN");
#endif

	pr_debug("[DISPLAY] %s: Enter\n", __func__);
	return 0;
}

#define PWM0CLKDIV1 0x61
#define PWM0CLKDIV0 0x62

static int hx8379a_vid_brightness_init(void)
{
	int ret = 0;

	pwmctrl_mmio = ioremap_nocache(PWMCTRL_REG,PWMCTRL_SIZE);
	lnw_gpio_set_alt(PWM_ENABLE_GPIO, LNW_ALT_2);

	pr_debug("[DISPLAY] %s: Enter\n", __func__);
	return ret;
}

#ifdef HX8379A_DEBUG

static int send_mipi_ret = -1;
static int read_mipi_ret = -1;
static u8 read_mipi_data = 0;

static ssize_t send_mipi_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
    int x0=0, x1=0;
    struct mdfld_dsi_pkg_sender *sender
			= mdfld_dsi_get_pkg_sender(hx8379a_dsi_config);

    sscanf(buf, "%x,%x", &x0, &x1);

    send_mipi_ret = mdfld_dsi_send_mcs_short_lp(sender,x0,x1,1,0);

	DRM_INFO("[DISPLAY] send %x,%x : ret = %d\n",x0,x1,send_mipi_ret);

    return count;
}

static ssize_t send_mipi_show(struct device *dev,
	struct device_attribute *attr, const char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n",send_mipi_ret);
}


static ssize_t read_mipi_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
    int x0=0;
    struct mdfld_dsi_pkg_sender *sender
			= mdfld_dsi_get_pkg_sender(hx8379a_dsi_config);

    sscanf(buf, "%x", &x0);

    read_mipi_ret = mdfld_dsi_read_mcs_lp(sender,x0,&read_mipi_data,1);
    if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
        read_mipi_ret = -EIO;

	DRM_INFO("[DISPLAY] read 0x%x :ret=%d data=0x%x\n", x0, read_mipi_ret, read_mipi_data);

    return count;
}

static ssize_t read_mipi_show(struct device *dev,
	struct device_attribute *attr, const char *buf)
{
	return snprintf(buf, PAGE_SIZE, "ret=%d data=0x%x\n",read_mipi_ret,read_mipi_data);
}

DEVICE_ATTR(send_mipi_hx8379a,S_IRUGO | S_IWUSR, send_mipi_show,send_mipi_store);
DEVICE_ATTR(read_mipi_hx8379a,S_IRUGO | S_IWUSR, read_mipi_show,read_mipi_store);


static struct attribute *hx8379a_attrs[] = {
        &dev_attr_send_mipi_hx8379a.attr,
        &dev_attr_read_mipi_hx8379a.attr,
        NULL
};

static struct attribute_group hx8379a_attr_group = {
        .attrs = hx8379a_attrs,
        .name = "hx8379a",
};

#endif

static int init_asus_panel_id(){
	int panel_id_value=0;

	panel_id_value = gpio_get_value(LCM_ID_ER)?0x1:0x0;

	asus_panel_id = ME372CG_PANEL | panel_id_value;

	DRM_INFO("[DISPLAY] %s: asus_panel_id = 0x%x\n", __func__, asus_panel_id);

	return 0;
}

void hx8379a_vid_init(struct drm_device *dev, struct panel_funcs *p_funcs)
{
	struct drm_psb_private *dev_priv =
		(struct drm_psb_private *) dev->dev_private;
	struct mdfld_dsi_config *dsi_config = dev_priv->dsi_configs[0];

	int ret = 0;

	p_funcs->get_config_mode = hx8379a_vid_get_config_mode;
	p_funcs->get_panel_info = hx8379a_vid_get_panel_info;
	p_funcs->dsi_controller_init = hx8379a_vid_dsi_controller_init;
	p_funcs->detect = hx8379a_vid_detect;
	p_funcs->power_on = hx8379a_vid_power_on;
	p_funcs->drv_ic_init = hx8379a_mipi_cmd_setting;
	p_funcs->power_off = hx8379a_vid_power_off;
	p_funcs->reset = hx8379a_vid_reset;
	p_funcs->set_brightness = hx8379a_vid_set_brightness;

	board_hw_id = Read_HW_ID();

	ret = hx8379a_vid_gpio_init();
	if (ret)
		DRM_ERROR("Faild to request GPIO for B101UAN01.7 panel\n");

	ret = hx8379a_vid_brightness_init();
	if (ret)
		DRM_ERROR("Faild to initilize PWM of MSCI\n");

	init_asus_panel_id();

#ifdef HX8379A_DEBUG

    sysfs_create_group(&dev->dev->kobj, &hx8379a_attr_group);

#endif

	hx8379a_dsi_config = dsi_config;


	DRM_INFO("[DISPLAY] %s: board_hw_id = %d\n", __func__, board_hw_id);

	DRM_INFO("[DISPLAY] %s: Enter\n", __func__);

}

static int hx8379a_vid_lcd_probe(struct platform_device *pdev)
{
	int ret = 0;

	DRM_INFO("%s: hx8379a panel detected\n", __func__);
	intel_mid_panel_register(hx8379a_vid_init);


	return 0;
}

static int hx8379a_vid_lcd_shutdown(struct platform_device *pdev)
{
    struct mdfld_dsi_pkg_sender *sender
			= mdfld_dsi_get_pkg_sender(hx8379a_dsi_config);

	if(hx8379a_dsi_config->dsi_hw_context.panel_on==0){
		DRM_INFO("[DISPLAY] %s: panel_on=%d.\n",
			__func__, hx8379a_dsi_config->dsi_hw_context.panel_on);
		return 0;
	}

	esd_thread_enable = false;
	msleep(10);

	pwm_configure(0);
	intel_scu_ipc_iowrite8(LCD_BL_EN,0x0);

	msleep(300);

	mdfld_dsi_send_mcs_short_lp(sender,0x10,0x00,0,0);

	usleep_range(40000, 40000);

	hx8379a_vid_gpio_control(0);

	DRM_INFO("%s: Enter\n", __func__);

	return 0;
}


struct platform_driver hx8379a_vid_lcd_driver = {
	.probe	= hx8379a_vid_lcd_probe,
	.driver	= {
		.name	= HX8379A_PANEL_NAME,
		.owner	= THIS_MODULE,
	},
	.shutdown = hx8379a_vid_lcd_shutdown,
};
