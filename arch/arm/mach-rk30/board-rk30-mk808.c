/* based on arch/arm/mach-rk30/board-rk30-sdk.c
 *
 * Copyright (C) 2012 ROCKCHIP, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/skbuff.h>
#include <linux/spi/spi.h>
#include <linux/mmc/host.h>
#include <linux/ion.h>
#include <linux/cpufreq.h>
#include <linux/clk.h>
#include <mach/dvfs.h>

#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/flash.h>
#include <asm/hardware/gic.h>

#include <mach/board.h>
#include <mach/hardware.h>
#include <mach/io.h>
#include <mach/gpio.h>
#include <mach/iomux.h>
#include <linux/rk_fb.h>
#include <linux/regulator/machine.h>
#include <linux/rfkill-rk.h>
#include <linux/sensor-dev.h>
#include <linux/regulator/rk29-pwm-regulator.h>


#ifdef CONFIG_IAM_CHANGES
//    #define MMEDIA_BUF_SIZE	5*FB_MAXPGSIZE + SZ_1M
    #define MMEDIA_BUF_SIZE	64 * SZ_1M
    #define FB0_SIZE	FB_MAXPGSIZE
#else
    #define FB0_SIZE	get_fb_size()
#endif

#ifdef CONFIG_ION_ROCKCHIP_SIZE
    #define ION_RESERVE_SIZE	(CONFIG_ION_ROCKCHIP_SIZE * SZ_1M)
#else
    #define ION_RESERVE_SIZE	(120 * SZ_1M)
#endif

#define BLUE_LED_PIN_PWM 1
#define BLUE_LED_PIN_POWER_MNG 1

#ifdef BLUE_LED_PIN_POWER_MNG
#undef BLUE_LED_PIN_PWM
#endif

#include <plat/efuse.h>

#if defined(CONFIG_MFD_RK610)
    #include <linux/mfd/rk610_core.h>
#endif

#if defined(CONFIG_RK_HDMI)
    #include "../../../drivers/video/rockchip/hdmi/rk_hdmi.h"
#endif

#if defined(CONFIG_SPIM_RK29)
    #include "../../../drivers/spi/rk29_spim.h"
#endif

#if defined(CONFIG_MT6229)
    #include <linux/mt6229.h>
#endif

#if defined(CONFIG_GPS_RK)
    #include "../../../drivers/misc/gps/rk_gps/rk_gps.h"
#endif

#ifdef CONFIG_RK_REMOTECTL
    #include <mach/remotectl.h>
#endif
    #if defined(CONFIG_MT6620)
    #include <linux/gps.h>
#endif

#include <plat/key.h>

static struct rk29_keys_button key_button[] = {
	{
		.desc	= "vol-",
		.code	= KEY_VOLUMEDOWN,
		.adc_value	= 150,
		.gpio = INVALID_GPIO,
		.active_low = PRESS_LEV_LOW,
	},
	{
		.desc	= "play",
		.code	= KEY_POWER,
		.adc_value	= 1,
		.gpio = INVALID_GPIO,
		.active_low = PRESS_LEV_LOW,
		//.code_long_press = EV_ENCALL,
		.wakeup	= 1,
	},
	{
		.desc	= "vol+",
		.code	= KEY_VOLUMEUP,
		.adc_value	= 300,
		.gpio = INVALID_GPIO,
		.active_low = PRESS_LEV_LOW,
	},
	{
		.desc	= "menu",
		.code	= EV_MENU,
		.adc_value	= 450,
		.gpio = INVALID_GPIO,
		.active_low = PRESS_LEV_LOW,
	},
	{
		.desc	= "home",
		.code	= KEY_HOME,
		.adc_value	= 600,
		.gpio = INVALID_GPIO,
		.active_low = PRESS_LEV_LOW,
	},
	{
		.desc	= "esc",
		.code	= KEY_BACK,
		.adc_value	= 750,
		.gpio = INVALID_GPIO,
		.active_low = PRESS_LEV_LOW,
	},
	{
		.desc	= "camera",
		.code	= KEY_CAMERA,
		.adc_value	= 900,
		.gpio = INVALID_GPIO,
		.active_low = PRESS_LEV_LOW,
	},
};

struct rk29_keys_platform_data rk29_keys_pdata = {
	.buttons	= key_button,
	.nbuttons	= ARRAY_SIZE(key_button),
	.chn	= 1,  //chn: 0-7, if do not use ADC,set 'chn' -1
};


static struct spi_board_info board_spi_devices[] = {
};


#ifdef CONFIG_IAM_CHANGES

#define PWM_MUX_NAME      GPIO0A3_PWM0_NAME
#define PWM_MUX_MODE_GPIO GPIO0A_GPIO0A3
#define PWM_GPIO         RK30_PIN0_PA3
#define PWM_MUX_MODE      GPIO0A_PWM0
#include <linux/leds_pwm.h>

#ifdef BLUE_LED_PIN_POWER_MNG

static DEFINE_MUTEX(power_key_mutex);

static int power_led_state;

static void _led_blue_set(int value)
{
    if (value)
		gpio_direction_output(PWM_GPIO, GPIO_HIGH);
    else
		gpio_direction_input(PWM_GPIO);
    power_led_state = value;
}

static void led_blue_set(struct led_classdev *led_cdev,
	           enum led_brightness value)
{
    mutex_lock(&power_key_mutex);
    _led_blue_set(value);
    mutex_unlock(&power_key_mutex);
}

static struct led_classdev led_power_blue = {
    .name			= "mk808:blue:power",
    .default_trigger	= "heartbeat",
    .brightness_set		= led_blue_set,
    .flags			= LED_CORE_SUSPENDRESUME,
};
//**********************
void my_power_off(void)
{
    led_classdev_unregister(&led_power_blue);
	gpio_free(PWM_GPIO);
	rk30_mux_api_set(PWM_MUX_NAME, PWM_MUX_MODE_GPIO);	
	gpio_request(PWM_GPIO, NULL);
	gpio_direction_output(PWM_GPIO, GPIO_LOW);
//	gpio_direction_input(PWM_GPIO);
}

void my_power_on(void)
{
	gpio_free(PWM_GPIO);
	rk30_mux_api_set(PWM_MUX_NAME, PWM_MUX_MODE_GPIO);
	gpio_request(PWM_GPIO, NULL);
	gpio_direction_output(PWM_GPIO, GPIO_HIGH);
};
//**************************
#define POWER_KEY_SCAN_TIME_MS	100
static struct input_dev *power_keydev;
static struct timer_list powerkey_timer;
static int power_key_state;

static void powerkey_btimer_cb(unsigned long data)
{
    int res;

    mutex_lock(&power_key_mutex);
    gpio_direction_input(PWM_GPIO);
    udelay(100);
    res = !gpio_get_value(PWM_GPIO);
    _led_blue_set(power_led_state);
    mutex_unlock(&power_key_mutex);

    if(res){//pressed
		if(!power_key_state){//and was not pressed
			input_report_key(power_keydev, KEY_POWER, 1);
			input_sync(power_keydev);
		}
    }else{//not pressed
		if(power_key_state){//and was pressed
			input_report_key(power_keydev, KEY_POWER, 0);
			input_sync(power_keydev);
		}
    }

    power_key_state = res;
    mod_timer( &powerkey_timer, jiffies + msecs_to_jiffies(POWER_KEY_SCAN_TIME_MS));
}

static int __init power_led_init(void)
{
    gpio_free(PWM_GPIO);
    gpio_request(PWM_GPIO, NULL);
//        gpio_set_value(PWM_GPIO, GPIO_LOW);
//    rk30_mux_api_set(PWM_MUX_NAME, PWM_MUX_MODE_GPIO);
    led_classdev_register(NULL, &led_power_blue);

    power_key_state = 0;
    power_keydev = input_allocate_device();

    if(!power_keydev){
		printk(KERN_ERR "Could not allocate PowerLed input device\n");
		goto err0;
    }else{
		power_keydev->name = "mk808LedPinInput";
		power_keydev->phys = "mk808/input1";
		power_keydev->id.bustype = BUS_HOST;
//	power_keydev->dev.parent = dev;
		set_bit(EV_KEY, power_keydev->evbit);
		set_bit(EV_REL, power_keydev->evbit);
		set_bit(KEY_POWER, power_keydev->keybit);

		if(input_register_device(power_keydev) < 0){
			printk(KERN_ERR "Could not register PowerLed input device\n");
			goto err1;
		}else{
			setup_timer( &powerkey_timer, powerkey_btimer_cb, 0 );
			if(mod_timer( &powerkey_timer, jiffies + msecs_to_jiffies(POWER_KEY_SCAN_TIME_MS))){
				printk(KERN_ERR "Could not setup timer for PowerLed\n");
				goto err2;
			}
		}
		mutex_init(&power_key_mutex);
		printk("PowerLed input initialized\n");
		return 0;
    }
err2:
    del_timer(&powerkey_timer);
    input_unregister_device(power_keydev);
err1:
    input_free_device(power_keydev);
err0:
	return 0;
}

fs_initcall(power_led_init);

#else
#ifdef BLUE_LED_PIN_PWM
static struct led_pwm power_led_pwm_leds[] = {
        {
			.name           = "mk808:blue:power",
			.default_trigger= "heartbeat",
			.max_brightness = 255,
			.pwm_id		= 0,
			.pwm_period_ns  = 1000000,
        },
};

static struct led_pwm_platform_data power_led_pwm_data = {
        .num_leds       = ARRAY_SIZE(power_led_pwm_leds),
        .leds           = power_led_pwm_leds,
};

static struct platform_device power_led_pwm = {
        .name   = "leds_pwm",
        .id     = -1,
        .dev    = {
                .platform_data = &power_led_pwm_data,
        },
};


static struct resource rk30_resource_pwm0[] = {
        [0] = {
                .start  = RK30_PWM01_PHYS,// From plat-rk/pwm.c
                .end    = RK30_PWM01_PHYS + 16 - 1,// 4*4 bytes - plat-rk/include/plat/pwm.h
                .flags  = IORESOURCE_MEM,
        },
};

struct platform_device rk30_device_pwm0 = {
        .name           = "rk30-pwm",
        .id             = 0,
        .resource       = rk30_resource_pwm0,
        .num_resources  = ARRAY_SIZE(rk30_resource_pwm0),
};
#endif
#endif
static struct resource rk30_resource_pwm1[] = {
        [0] = {
                .start  = RK30_PWM01_PHYS + 0x10,// From plat-rk/pwm.c
                .end    = RK30_PWM01_PHYS + 0x10 + 16 - 1,// 4*4 bytes - plat-rk/include/plat/pwm.h
                .flags  = IORESOURCE_MEM,
        },
};

static struct resource rk30_resource_pwm2[] = {
        [0] = {
                .start  = RK30_PWM23_PHYS + 2 * 0x10,//?? maybe 0 From plat-rk/pwm.c
                .end    = RK30_PWM23_PHYS + 2 * 0x10 + 16 - 1,// 4*4 bytes - plat-rk/include/plat/pwm.h
                .flags  = IORESOURCE_MEM,
        },
};

static struct resource rk30_resource_pwm3[] = {
        [0] = {
                .start  = RK30_PWM23_PHYS + 3 * 0x10,//?? maybe 0x10 From plat-rk/pwm.c
                .end    = RK30_PWM23_PHYS + 3 * 0x10 + 16 - 1,// 4*4 bytes - plat-rk/include/plat/pwm.h
                .flags  = IORESOURCE_MEM,
        },
};

struct platform_device rk30_device_pwm1 = {
        .name           = "rk30-pwm",
        .id             = 1,
        .resource       = rk30_resource_pwm1,
        .num_resources  = ARRAY_SIZE(rk30_resource_pwm1),
};

struct platform_device rk30_device_pwm2 = {
        .name           = "rk30-pwm",
        .id             = 2,
        .resource       = rk30_resource_pwm2,
        .num_resources  = ARRAY_SIZE(rk30_resource_pwm2),
};

struct platform_device rk30_device_pwm3 = {
        .name           = "rk30-pwm",
        .id             = 3,
        .resource       = rk30_resource_pwm3,
        .num_resources  = ARRAY_SIZE(rk30_resource_pwm3),
};
//***********************Memory heap for video and mali acl************************
 static struct resource rk_resource_mmedia[] = {
         [0] = {
                 .start  = 0,
                 .end    = 0,
                 .flags  = IORESOURCE_MEM,
         },
 };
 
 struct platform_device rk_device_mmedia = {
         .name           = "rk-ump",
         .id             = -1,
         .resource       = rk_resource_mmedia,
         .num_resources  = ARRAY_SIZE(rk_resource_mmedia),
 };
//**********************************************************************************
void rk29_backlight_set(bool on){
}
EXPORT_SYMBOL(rk29_backlight_set);
#endif

#ifdef CONFIG_FB_ROCKCHIP

#define LCD_CS_PIN         RK30_PIN4_PC7
#define LCD_CS_VALUE       GPIO_HIGH

#define LCD_EN_PIN         RK30_PIN6_PB4
#define LCD_EN_VALUE       GPIO_LOW

static int rk_fb_io_init(struct rk29_fb_setting_info *fb_setting)
{
	int ret = 0;

	if(LCD_CS_PIN != INVALID_GPIO)
	{
		ret = gpio_request(LCD_CS_PIN, NULL);
		if (ret != 0)
		{
			gpio_free(LCD_CS_PIN);
			printk(KERN_ERR "request lcd cs pin fail!\n");
			return -1;
		}
		else
		{
			gpio_direction_output(LCD_CS_PIN, LCD_CS_VALUE);
		}
	}
/*	else
	{
	    gpio_direction_output(LCD_CS_PIN, LCD_CS_VALUE);
	}*/

	if(LCD_EN_PIN != INVALID_GPIO)
	{
		ret = gpio_request(LCD_EN_PIN, NULL);
		if (ret != 0)
		{
			gpio_free(LCD_EN_PIN);
			printk(KERN_ERR "request lcd en pin fail!\n");
			return -1;
		}
		else
		{
			gpio_direction_output(LCD_EN_PIN, LCD_EN_VALUE);
		}
	}
/*	else
	{
	    gpio_direction_output(LCD_EN_PIN, LCD_EN_VALUE);
	}*/
    return 0;
}

static int rk_fb_io_disable(void)
{
	if(LCD_CS_PIN != INVALID_GPIO)
	{
		gpio_set_value(LCD_CS_PIN, !LCD_CS_VALUE);
	}

	if(LCD_EN_PIN != INVALID_GPIO)
	{
		gpio_set_value(LCD_EN_PIN, !LCD_EN_VALUE);
	}
	return 0;
}
static int rk_fb_io_enable(void)
{
	if(LCD_CS_PIN != INVALID_GPIO)
	{
		gpio_set_value(LCD_CS_PIN, LCD_CS_VALUE);
	}
	if(LCD_EN_PIN != INVALID_GPIO)
	{
		gpio_set_value(LCD_EN_PIN, LCD_EN_VALUE);
	}
	return 0;
}

#if defined(CONFIG_LCDC0_RK30)
struct rk29fb_info lcdc0_screen_info = {
#if defined(CONFIG_RK_HDMI) && defined(CONFIG_HDMI_SOURCE_LCDC0) && defined(CONFIG_DUAL_LCDC_DUAL_DISP_IN_KERNEL)
		.prop	   = EXTEND,	//extend display device
		.io_init    = NULL,
		.io_disable = NULL,
		.io_enable = NULL,
		.set_screen_info = hdmi_init_lcdc,
#else
		.prop	   = PRMRY,		//primary display device
		.io_init   = rk_fb_io_init,
		.io_disable = rk_fb_io_disable,
		.io_enable = rk_fb_io_enable,
		.set_screen_info = set_lcd_info,
#endif

};
#endif

#if defined(CONFIG_LCDC1_RK30)
struct rk29fb_info lcdc1_screen_info = {
#if defined(CONFIG_RK_HDMI) && defined(CONFIG_HDMI_SOURCE_LCDC1) && defined(CONFIG_DUAL_LCDC_DUAL_DISP_IN_KERNEL)
		.prop	   = EXTEND,	//extend display device
		.io_init    = NULL,
		.io_disable = NULL,
		.io_enable = NULL,
		.set_screen_info = hdmi_init_lcdc,
#else
		.prop	   = PRMRY,		//primary display device
		.io_init   = rk_fb_io_init,
		.io_disable = rk_fb_io_disable,
		.io_enable = rk_fb_io_enable,
		.set_screen_info = set_lcd_info,
#endif

};
#endif

static struct resource resource_fb[] = {
	[0] = {
		.name  = "fb0 buf",
		.start = 0,
		.end   = 0,//RK30_FB0_MEM_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.name  = "ipp buf",  //for rotate
		.start = 0,
		.end   = 0,//RK30_FB0_MEM_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[2] = {
		.name  = "fb2 buf",
		.start = 0,
		.end   = 0,//RK30_FB0_MEM_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
};

static struct platform_device device_fb = {
	.name		= "rk-fb",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(resource_fb),
	.resource	= resource_fb,
};

#if defined(CONFIG_LCDC0_RK30)
static struct resource resource_lcdc0[] = {
	[0] = {
		.name  = "lcdc0 reg",
		.start = RK30_LCDC0_PHYS,
		.end   = RK30_LCDC0_PHYS + RK30_LCDC0_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	
	[1] = {
		.name  = "lcdc0 irq",
		.start = IRQ_LCDC0,
		.end   = IRQ_LCDC0,
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device device_lcdc0 = {
	.name		  = "rk30-lcdc",
	.id		  = 0,
	.num_resources	  = ARRAY_SIZE(resource_lcdc0),
	.resource	  = resource_lcdc0,
	.dev 		= {
		.platform_data = &lcdc0_screen_info,
	},
};
#endif
#if defined(CONFIG_LCDC1_RK30) 
extern struct rk29fb_info lcdc1_screen_info;
static struct resource resource_lcdc1[] = {
	[0] = {
		.name  = "lcdc1 reg",
		.start = RK30_LCDC1_PHYS,
		.end   = RK30_LCDC1_PHYS + RK30_LCDC1_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.name  = "lcdc1 irq",
		.start = IRQ_LCDC1,
		.end   = IRQ_LCDC1,
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device device_lcdc1 = {
	.name		  = "rk30-lcdc",
	.id		  = 1,
	.num_resources	  = ARRAY_SIZE(resource_lcdc1),
	.resource	  = resource_lcdc1,
	.dev 		= {
		.platform_data = &lcdc1_screen_info,
	},
};
#endif
#endif

#ifdef CONFIG_ION

static struct ion_platform_data rk30_ion_pdata = {
	.nr = 1,
	.heaps = {
		{
			.type = ION_HEAP_TYPE_CARVEOUT,
			.id = ION_NOR_HEAP_ID,
			.name = "norheap",
			.size = ION_RESERVE_SIZE,
		}
	},
};

static struct platform_device device_ion = {
	.name = "ion-rockchip",
	.id = 0,
	.dev = {
		.platform_data = &rk30_ion_pdata,
	},
};
#endif

/**************************************************************************************************
 * SDMMC devices,  include the module of SD,MMC,and SDIO.noted by xbw at 2012-03-05
**************************************************************************************************/
#ifdef CONFIG_SDMMC_RK29
#include "board-rk30-sdk-sdmmc.c"
#endif

#ifdef CONFIG_SDMMC0_RK29
static int rk29_sdmmc0_cfg_gpio(void)
{
#ifdef CONFIG_SDMMC_RK29_OLD
	rk30_mux_api_set(GPIO3B1_SDMMC0CMD_NAME, GPIO3B_SDMMC0_CMD);
	rk30_mux_api_set(GPIO3B0_SDMMC0CLKOUT_NAME, GPIO3B_SDMMC0_CLKOUT);
	rk30_mux_api_set(GPIO3B2_SDMMC0DATA0_NAME, GPIO3B_SDMMC0_DATA0);
	rk30_mux_api_set(GPIO3B3_SDMMC0DATA1_NAME, GPIO3B_SDMMC0_DATA1);
	rk30_mux_api_set(GPIO3B4_SDMMC0DATA2_NAME, GPIO3B_SDMMC0_DATA2);
	rk30_mux_api_set(GPIO3B5_SDMMC0DATA3_NAME, GPIO3B_SDMMC0_DATA3);

	rk30_mux_api_set(GPIO3B6_SDMMC0DETECTN_NAME, GPIO3B_GPIO3B6);

	rk30_mux_api_set(GPIO3A7_SDMMC0PWREN_NAME, GPIO3A_GPIO3A7);
	gpio_request(RK30_PIN3_PA7, "sdmmc-power");
	gpio_direction_output(RK30_PIN3_PA7, GPIO_LOW);

#else
	    rk29_sdmmc_set_iomux(0, 0xFFFF);

    #if defined(CONFIG_SDMMC0_RK29_SDCARD_DET_FROM_GPIO)
        rk30_mux_api_set(RK29SDK_SD_CARD_DETECT_PIN_NAME, RK29SDK_SD_CARD_DETECT_IOMUX_FGPIO);
    #else
	    rk30_mux_api_set(RK29SDK_SD_CARD_DETECT_PIN_NAME, RK29SDK_SD_CARD_DETECT_IOMUX_FMUX);
    #endif	

    #if defined(CONFIG_SDMMC0_RK29_WRITE_PROTECT)
	    gpio_request(SDMMC0_WRITE_PROTECT_PIN, "sdmmc-wp");
	    gpio_direction_input(SDMMC0_WRITE_PROTECT_PIN);
    #endif

#endif

	return 0;
}

#define CONFIG_SDMMC0_USE_DMA
struct rk29_sdmmc_platform_data default_sdmmc0_data = {
	.host_ocr_avail =
	    (MMC_VDD_25_26 | MMC_VDD_26_27 | MMC_VDD_27_28 | MMC_VDD_28_29 |
	     MMC_VDD_29_30 | MMC_VDD_30_31 | MMC_VDD_31_32 | MMC_VDD_32_33 |
	     MMC_VDD_33_34 | MMC_VDD_34_35 | MMC_VDD_35_36),
	.host_caps =
	    (MMC_CAP_4_BIT_DATA | MMC_CAP_MMC_HIGHSPEED | MMC_CAP_SD_HIGHSPEED),
	.io_init = rk29_sdmmc0_cfg_gpio,

#if !defined(CONFIG_SDMMC_RK29_OLD)
	.set_iomux = rk29_sdmmc_set_iomux,
#endif

	.dma_name = "sd_mmc",
#ifdef CONFIG_SDMMC0_USE_DMA
	.use_dma = 1,
#else
	.use_dma = 0,
#endif

#if defined(CONFIG_WIFI_COMBO_MODULE_CONTROL_FUNC) && defined(CONFIG_USE_SDMMC0_FOR_WIFI_DEVELOP_BOARD)
    .status = rk29sdk_wifi_mmc0_status,
    .register_status_notify = rk29sdk_wifi_mmc0_status_register,
#endif

#if defined(RK29SDK_SD_CARD_PWR_EN) || (INVALID_GPIO != RK29SDK_SD_CARD_PWR_EN)
    .power_en = RK29SDK_SD_CARD_PWR_EN,
    .power_en_level = RK29SDK_SD_CARD_PWR_EN_LEVEL,
#else
    .power_en = INVALID_GPIO,
    .power_en_level = GPIO_LOW,
#endif    
	.enable_sd_wakeup = 0,

#if defined(CONFIG_SDMMC0_RK29_WRITE_PROTECT)
	.write_prt = SDMMC0_WRITE_PROTECT_PIN,
	.write_prt_enalbe_level = SDMMC0_WRITE_PROTECT_ENABLE_VALUE;
#else
	.write_prt = INVALID_GPIO,
#endif

    .det_pin_info = {    
    #if defined(RK29SDK_SD_CARD_DETECT_N) || (INVALID_GPIO != RK29SDK_SD_CARD_DETECT_N)  
        .io             = RK29SDK_SD_CARD_DETECT_N, //INVALID_GPIO,
        .enable         = RK29SDK_SD_CARD_INSERT_LEVEL,
        #ifdef RK29SDK_SD_CARD_DETECT_PIN_NAME
        .iomux          = {
            .name       = RK29SDK_SD_CARD_DETECT_PIN_NAME,
            #ifdef RK29SDK_SD_CARD_DETECT_IOMUX_FGPIO
            .fgpio      = RK29SDK_SD_CARD_DETECT_IOMUX_FGPIO,
            #endif
            #ifdef RK29SDK_SD_CARD_DETECT_IOMUX_FMUX
            .fmux       = RK29SDK_SD_CARD_DETECT_IOMUX_FMUX,
            #endif
        },
        #endif
    #else
        .io             = INVALID_GPIO,
        .enable         = GPIO_LOW,
    #endif    
    }, 

};
#endif // CONFIG_SDMMC0_RK29

#ifdef CONFIG_SDMMC1_RK29
#define CONFIG_SDMMC1_USE_DMA
static int rk29_sdmmc1_cfg_gpio(void)
{
#if defined(CONFIG_SDMMC_RK29_OLD)
	rk30_mux_api_set(GPIO3C0_SMMC1CMD_NAME, GPIO3C_SMMC1_CMD);
	rk30_mux_api_set(GPIO3C5_SDMMC1CLKOUT_NAME, GPIO3C_SDMMC1_CLKOUT);
	rk30_mux_api_set(GPIO3C1_SDMMC1DATA0_NAME, GPIO3C_SDMMC1_DATA0);
	rk30_mux_api_set(GPIO3C2_SDMMC1DATA1_NAME, GPIO3C_SDMMC1_DATA1);
	rk30_mux_api_set(GPIO3C3_SDMMC1DATA2_NAME, GPIO3C_SDMMC1_DATA2);
	rk30_mux_api_set(GPIO3C4_SDMMC1DATA3_NAME, GPIO3C_SDMMC1_DATA3);
#else

#if defined(CONFIG_SDMMC1_RK29_WRITE_PROTECT)
	gpio_request(SDMMC1_WRITE_PROTECT_PIN, "sdio-wp");
	gpio_direction_input(SDMMC1_WRITE_PROTECT_PIN);
#endif

#endif

	return 0;
}

struct rk29_sdmmc_platform_data default_sdmmc1_data = {
	.host_ocr_avail =
	    (MMC_VDD_25_26 | MMC_VDD_26_27 | MMC_VDD_27_28 | MMC_VDD_28_29 |
	     MMC_VDD_29_30 | MMC_VDD_30_31 | MMC_VDD_31_32 | MMC_VDD_32_33 |
	     MMC_VDD_33_34),

#if !defined(CONFIG_USE_SDMMC1_FOR_WIFI_DEVELOP_BOARD)
	.host_caps = (MMC_CAP_4_BIT_DATA | MMC_CAP_SDIO_IRQ |
		      MMC_CAP_MMC_HIGHSPEED | MMC_CAP_SD_HIGHSPEED),
#else
	.host_caps =
	    (MMC_CAP_4_BIT_DATA | MMC_CAP_MMC_HIGHSPEED | MMC_CAP_SD_HIGHSPEED),
#endif

	.io_init = rk29_sdmmc1_cfg_gpio,

#if !defined(CONFIG_SDMMC_RK29_OLD)
	.set_iomux = rk29_sdmmc_set_iomux,
#endif

	.dma_name = "sdio",
#ifdef CONFIG_SDMMC1_USE_DMA
	.use_dma = 1,
#else
	.use_dma = 0,
#endif

#if defined(CONFIG_WIFI_CONTROL_FUNC) || defined(CONFIG_WIFI_COMBO_MODULE_CONTROL_FUNC)
    .status = rk29sdk_wifi_status,
    .register_status_notify = rk29sdk_wifi_status_register,
#endif

    #if defined(CONFIG_SDMMC1_RK29_WRITE_PROTECT)
    	.write_prt = SDMMC1_WRITE_PROTECT_PIN,    	
	    .write_prt_enalbe_level = SDMMC1_WRITE_PROTECT_ENABLE_VALUE;
    #else
    	.write_prt = INVALID_GPIO,
    #endif

    #if defined(CONFIG_RK29_SDIO_IRQ_FROM_GPIO)
        .sdio_INT_gpio = RK29SDK_WIFI_SDIO_CARD_INT,
    #endif

    .det_pin_info = {    
#if defined(CONFIG_USE_SDMMC1_FOR_WIFI_DEVELOP_BOARD)
     #if defined(RK29SDK_SD_CARD_DETECT_N) || (INVALID_GPIO != RK29SDK_SD_CARD_DETECT_N)  
        .io             = RK29SDK_SD_CARD_DETECT_N,
     #else
         .io             = INVALID_GPIO,
     #endif   

        .enable         = RK29SDK_SD_CARD_INSERT_LEVEL,
        #ifdef RK29SDK_SD_CARD_DETECT_PIN_NAME
        .iomux          = {
            .name       = RK29SDK_SD_CARD_DETECT_PIN_NAME,
            #ifdef RK29SDK_SD_CARD_DETECT_IOMUX_FGPIO
            .fgpio      = RK29SDK_SD_CARD_DETECT_IOMUX_FGPIO,
            #endif
            #ifdef RK29SDK_SD_CARD_DETECT_IOMUX_FMUX
            .fmux       = RK29SDK_SD_CARD_DETECT_IOMUX_FMUX,
            #endif
        },
        #endif
 #else
        .io             = INVALID_GPIO,
        .enable         = GPIO_LOW,
#endif
    },
   
	.enable_sd_wakeup = 0,
};
#endif //endif--#ifdef CONFIG_SDMMC1_RK29

/**************************************************************************************************
 * the end of setting for SDMMC devices
**************************************************************************************************/

#if defined(CONFIG_BATTERY_RK30_ADC)||defined(CONFIG_BATTERY_RK30_ADC_FAC)
static struct rk30_adc_battery_platform_data rk30_adc_battery_platdata = {
        .dc_det_pin      = RK30_PIN6_PA5,
        .batt_low_pin    = RK30_PIN6_PA0,
        .charge_set_pin  = INVALID_GPIO,
        .charge_ok_pin   = RK30_PIN6_PA6,
        .dc_det_level    = GPIO_LOW,
        .charge_ok_level = GPIO_HIGH,
};

static struct platform_device rk30_device_adc_battery = {
        .name   = "rk30-battery",
        .id     = -1,
        .dev = {
                .platform_data = &rk30_adc_battery_platdata,
        },
};
#endif

#ifdef CONFIG_RK_REMOTECTL

void rk30_remotectl_iomux(void)
{
	;
}

struct RKxx_remotectl_platform_data rk30_remotectl_pdata = {
    .gpio	=   RK30_PIN6_PA1, 
    .wakeup	= 1,
    .rep    = 0,
    .set_iomux = rk30_remotectl_iomux,    
};

static struct platform_device rk30_device_remotectl = {
	.name		= "rkxx-remotectl",
	.id		= -1,
	.dev		= {
		.platform_data	= &rk30_remotectl_pdata,
	},
};
#endif

#ifdef CONFIG_RK29_VMAC
#define PHY_PWR_EN_GPIO	RK30_PIN1_PD6
#include "board-rk30-sdk-vmac.c"
#endif

#ifdef CONFIG_RFKILL_RK
// bluetooth rfkill device, its driver in net/rfkill/rfkill-rk.c
static struct rfkill_rk_platform_data rfkill_rk_platdata = {
    .type               = RFKILL_TYPE_BLUETOOTH,

    .poweron_gpio       = { // BT_REG_ON
        .io             = RK30_PIN3_PC7,
        .enable         = GPIO_HIGH,
        .iomux          = {
            .name       = GPIO3C7_SDMMC1WRITEPRT_NAME,
            .fgpio      = GPIO3C_GPIO3C7,
        },
    },

    .reset_gpio         = { // BT_RST
        .io             = RK30_PIN3_PD1, // set io to INVALID_GPIO for disable it
        .enable         = GPIO_LOW,
        .iomux          = {
            .name       = GPIO3D1_SDMMC1BACKENDPWR_NAME,
            .fgpio      = GPIO3D_GPIO3D1,
        },
    },

    .wake_gpio          = { // BT_WAKE, use to control bt's sleep and wakeup
        .io             = RK30_PIN3_PC6, // set io to INVALID_GPIO for disable it
        .enable         = GPIO_HIGH,
        .iomux          = {
            .name       = GPIO3C6_SDMMC1DETECTN_NAME,
            .fgpio      = GPIO3C_GPIO3C6,
        },
    },

    .wake_host_irq      = { // BT_HOST_WAKE, for bt wakeup host when it is in deep sleep
        .gpio           = {
            .io         = RK30_PIN6_PA7, // set io to INVALID_GPIO for disable it
            .enable     = GPIO_LOW,      // set GPIO_LOW for falling, set 0 for rising
            .iomux      = {
                .name   = NULL,
            },
        },
    },

    .rts_gpio           = { // UART_RTS, enable or disable BT's data coming
        .io             = RK30_PIN1_PA3, // set io to INVALID_GPIO for disable it
        .enable         = GPIO_LOW,
        .iomux          = {
            .name       = GPIO1A3_UART0RTSN_NAME,
            .fgpio      = GPIO1A_GPIO1A3,
            .fmux       = GPIO1A_UART0_RTS_N,
        },
    },
};

static struct platform_device device_rfkill_rk = {
    .name   = "rfkill_rk",
    .id     = -1,
    .dev    = {
        .platform_data = &rfkill_rk_platdata,
    },
};
#endif


static struct platform_device *devices[] __initdata = {
#ifdef CONFIG_ION
	&device_ion,
#endif
#if defined(CONFIG_WIFI_CONTROL_FUNC)||defined(CONFIG_WIFI_COMBO_MODULE_CONTROL_FUNC)
	&rk29sdk_wifi_device,
#endif

#ifdef CONFIG_RFKILL_RK
	&device_rfkill_rk,
#endif

#ifdef CONFIG_RK_REMOTECTL	
	&rk30_device_remotectl,
#endif

#ifdef CONFIG_IAM_CHANGES
#ifdef BLUE_LED_PIN_PWM
	&rk30_device_pwm0,
	&power_led_pwm,
#endif
	&rk_device_mmedia,
#endif
};


static int rk_platform_add_display_devices(void)
{
	struct platform_device *fb = NULL;  //fb
	struct platform_device *lcdc0 = NULL; //lcdc0
	struct platform_device *lcdc1 = NULL; //lcdc1
	struct platform_device *bl = NULL; //backlight
#ifdef CONFIG_FB_ROCKCHIP
	fb = &device_fb;
#endif

#if defined(CONFIG_LCDC0_RK30)
	lcdc0 = &device_lcdc0,
#endif

#if defined(CONFIG_LCDC1_RK30)
	lcdc1 = &device_lcdc1,
#endif

	__rk_platform_add_display_devices(fb,lcdc0,lcdc1,bl);

	return 0;
	
};

// i2c
#ifdef CONFIG_I2C0_RK30
static struct i2c_board_info __initdata i2c0_info[] = {

};
#endif

#ifdef CONFIG_I2C1_RK30
static struct i2c_board_info __initdata i2c1_info[] = {

};
#endif

#ifdef CONFIG_I2C2_RK30
static struct i2c_board_info __initdata i2c2_info[] = {

};
#endif

#ifdef CONFIG_I2C3_RK30
static struct i2c_board_info __initdata i2c3_info[] = {
};
#endif

#ifdef CONFIG_I2C4_RK30
static struct i2c_board_info __initdata i2c4_info[] = {
};
#endif

#ifdef CONFIG_I2C_GPIO_RK30
#define I2C_SDA_PIN     INVALID_GPIO// RK30_PIN2_PD6   //set sda_pin here
#define I2C_SCL_PIN     INVALID_GPIO//RK30_PIN2_PD7   //set scl_pin here

static int rk30_i2c_io_init(void)
{
        //set iomux (gpio) here
        //rk30_mux_api_set(GPIO2D7_I2C1SCL_NAME, GPIO2D_GPIO2D7);
        //rk30_mux_api_set(GPIO2D6_I2C1SDA_NAME, GPIO2D_GPIO2D6);

        return 0;
}
struct i2c_gpio_platform_data default_i2c_gpio_data = {
       .sda_pin = I2C_SDA_PIN,
       .scl_pin = I2C_SCL_PIN,
       .udelay = 5, // clk = 500/udelay = 100Khz
       .timeout = 100,//msecs_to_jiffies(100),
       .bus_num    = 5,
       .io_init = rk30_i2c_io_init,
};

static struct i2c_board_info __initdata i2c_gpio_info[] = {
};
#endif

static void __init rk30_i2c_register_board_info(void)
{
#ifdef CONFIG_I2C0_RK30
	i2c_register_board_info(0, i2c0_info, ARRAY_SIZE(i2c0_info));
#endif
#ifdef CONFIG_I2C1_RK30
	i2c_register_board_info(1, i2c1_info, ARRAY_SIZE(i2c1_info));
#endif
#ifdef CONFIG_I2C2_RK30
	i2c_register_board_info(2, i2c2_info, ARRAY_SIZE(i2c2_info));
#endif
#ifdef CONFIG_I2C3_RK30
	i2c_register_board_info(3, i2c3_info, ARRAY_SIZE(i2c3_info));
#endif
#ifdef CONFIG_I2C4_RK30
	i2c_register_board_info(4, i2c4_info, ARRAY_SIZE(i2c4_info));
#endif
#ifdef CONFIG_I2C_GPIO_RK30
	i2c_register_board_info(5, i2c_gpio_info, ARRAY_SIZE(i2c_gpio_info));
#endif
}
//end of i2c

#define POWER_ON_PIN RK30_PIN6_PB0   //power_hold
static void rk30_pm_power_off(void)
{
	printk(KERN_ERR "rk30_pm_power_off start...\n");
	gpio_direction_output(POWER_ON_PIN, GPIO_LOW);
	#if defined(CONFIG_MFD_WM831X)	
	if(pmic_is_wm8326())
	{
		wm831x_set_bits(Wm831x,WM831X_GPIO_LEVEL,0x0001,0x0000);  //set sys_pwr 0
		wm831x_device_shutdown(Wm831x);//wm8326 shutdown
	}
	#endif
	#if defined(CONFIG_MFD_TPS65910)
	if(pmic_is_tps65910())
	{
		tps65910_device_shutdown();//tps65910 shutdown
	}
	#endif
#if defined(CONFIG_IAM_CHANGES) && defined(BLUE_LED_PIN_POWER_MNG)
	my_power_off();
#endif
	while (1);
}

static void __init machine_rk30_board_init(void)
{
	avs_init();
	gpio_request(POWER_ON_PIN, "poweronpin");
	gpio_direction_output(POWER_ON_PIN, GPIO_HIGH);
	
	pm_power_off = rk30_pm_power_off;

#if defined(CONFIG_IAM_CHANGES) && defined(BLUE_LED_PIN_POWER_MNG)
	my_power_on();	
#endif
	rk30_i2c_register_board_info();
	spi_register_board_info(board_spi_devices, ARRAY_SIZE(board_spi_devices));
	platform_add_devices(devices, ARRAY_SIZE(devices));
	rk_platform_add_display_devices();
	board_usb_detect_init(RK30_PIN6_PA3);

#if defined(CONFIG_WIFI_CONTROL_FUNC)
	rk29sdk_wifi_bt_gpio_control_init();
#elif defined(CONFIG_WIFI_COMBO_MODULE_CONTROL_FUNC)
    rk29sdk_wifi_combo_module_gpio_init();
#endif

#if defined(CONFIG_MT6620)
    clk_set_rate(clk_get_sys("rk_serial.0", "uart"), 48*1000000);
#endif
}

static void __init rk30_reserve(void)
{
#if CONFIG_IAM_CHANGES
    rk_resource_mmedia[0].start = board_mem_reserve_add("mmedia_buf", MMEDIA_BUF_SIZE);
    rk_resource_mmedia[0].end = rk_resource_mmedia[0].start + MMEDIA_BUF_SIZE- 1;
#endif
#ifdef CONFIG_ION
	rk30_ion_pdata.heaps[0].base = board_mem_reserve_add("ion", ION_RESERVE_SIZE);
#endif
#ifdef CONFIG_FB_ROCKCHIP
	resource_fb[0].start = board_mem_reserve_add("fb0 buf", FB0_SIZE);
	resource_fb[0].end = resource_fb[0].start + FB0_SIZE- 1;

#if defined(CONFIG_FB_ROTATE) && !defined(CONFIG_THREE_FB_BUFFER)
	resource_fb[2].start = board_mem_reserve_add("fb2 buf",get_fb_size());
	resource_fb[2].end = resource_fb[2].start + get_fb_size() - 1;
#endif
#endif
#ifdef CONFIG_VIDEO_RK29
	rk30_camera_request_reserve_mem();
#endif
	board_mem_reserved();
}

/**
 * dvfs_cpu_logic_table: table for arm and logic dvfs 
 * @frequency	: arm frequency
 * @cpu_volt	: arm voltage depend on frequency
 * @logic_volt	: logic voltage arm requests depend on frequency
 * comments	: min arm/logic voltage
 */
static struct dvfs_arm_table dvfs_cpu_logic_table[] = {
	{.frequency = 252 * 1000,	.cpu_volt = 1075 * 1000,	.logic_volt = 1125 * 1000},//0.975V/1.000V
	{.frequency = 504 * 1000,	.cpu_volt = 1100 * 1000,	.logic_volt = 1125 * 1000},//0.975V/1.000V
	{.frequency = 816 * 1000,	.cpu_volt = 1125 * 1000,	.logic_volt = 1150 * 1000},//1.000V/1.025V
	{.frequency = 1008 * 1000,	.cpu_volt = 1125 * 1000,	.logic_volt = 1150 * 1000},//1.025V/1.050V
	{.frequency = 1200 * 1000,	.cpu_volt = 1175 * 1000,	.logic_volt = 1200 * 1000},//1.100V/1.050V
	{.frequency = 1272 * 1000,	.cpu_volt = 1225 * 1000,	.logic_volt = 1200 * 1000},//1.150V/1.100V
	{.frequency = 1416 * 1000,	.cpu_volt = 1300 * 1000,	.logic_volt = 1200 * 1000},//1.225V/1.100V
	{.frequency = 1512 * 1000,	.cpu_volt = 1350 * 1000,	.logic_volt = 1250 * 1000},//1.300V/1.150V
	{.frequency = 1608 * 1000,	.cpu_volt = 1425 * 1000,	.logic_volt = 1300 * 1000},//1.325V/1.175V
	{.frequency = CPUFREQ_TABLE_END},
};

static struct cpufreq_frequency_table dvfs_gpu_table[] = {
	{.frequency = 266 * 1000,	.index = 1050 * 1000},
	{.frequency = 400 * 1000,	.index = 1275 * 1000},
	{.frequency = CPUFREQ_TABLE_END},
};

static struct cpufreq_frequency_table dvfs_ddr_table[] = {
    {.frequency = 200 * 1000 + DDR_FREQ_SUSPEND, .index = 1050 * 1000},
	{.frequency = 300 * 1000 + DDR_FREQ_VIDEO, .index = 1050 * 1000},
	{.frequency = 400 * 1000 + DDR_FREQ_NORMAL, .index = 1125 * 1000},
	{.frequency = CPUFREQ_TABLE_END},
};

#define DVFS_CPU_TABLE_SIZE	(ARRAY_SIZE(dvfs_cpu_logic_table))
static struct cpufreq_frequency_table cpu_dvfs_table[DVFS_CPU_TABLE_SIZE];
static struct cpufreq_frequency_table dep_cpu2core_table[DVFS_CPU_TABLE_SIZE];

void __init board_clock_init(void)
{
	rk30_clock_data_init(periph_pll_default, codec_pll_default, RK30_CLOCKS_DEFAULT_FLAGS);
	dvfs_set_arm_logic_volt(dvfs_cpu_logic_table, cpu_dvfs_table, dep_cpu2core_table);
	dvfs_set_freq_volt_table(clk_get(NULL, "gpu"), dvfs_gpu_table);
	dvfs_set_freq_volt_table(clk_get(NULL, "ddr"), dvfs_ddr_table);

}

MACHINE_START(RK30, "RK30board")
	.boot_params	= PLAT_PHYS_OFFSET + 0x800,
	.fixup		= rk30_fixup,
	.reserve	= &rk30_reserve,
	.map_io		= rk30_map_io,
	.init_irq	= rk30_init_irq,
	.timer		= &rk30_timer,
	.init_machine	= machine_rk30_board_init,
MACHINE_END
