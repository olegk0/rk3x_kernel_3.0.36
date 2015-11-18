/*
 * pwm driver for rk3066  olegvedi@gmail.com 2015
 * TODO need check time params
 *
 * based on 
 *
 * drivers/video/backlight/rk29_backlight.c
 * Copyright (C) 2009-2011 Rockchip Corporation.
 *
 * and
 *
 * linux/arch/arm/mach-pxa/pwm.c
 * simple driver for PWM (Pulse Width Modulator) controller
 * 2008-02-13	initial version eric miao <eric.miao@marvell.com>
 *
 * and
 *
 * rockchip pwm.h pwm.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/pwm.h>
#include <linux/gpio.h>
#include <mach/board.h>
#include <mach/iomux.h>
#include <plat/pwm.h>
#include <linux/spinlock.h>

#include <asm/div64.h>


#define PWM0_MUX_MODE_GPIO	GPIO0A_GPIO0A3
#define PWM0_GPIO		RK30_PIN0_PA3
#define PWM0_MUX_MODE		GPIO0A_PWM0

#define PWM1_MUX_MODE_GPIO	GPIO0A_GPIO0A4
#define PWM1_GPIO		RK30_PIN0_PA4
#define PWM1_MUX_MODE		GPIO0A_PWM1

#define PWM2_MUX_MODE_GPIO	GPIO0D_GPIO0D6
#define PWM2_GPIO		RK30_PIN0_PD6
#define PWM2_MUX_MODE		GPIO0D_PWM2

#define PWM3_MUX_MODE_GPIO	GPIO0D_GPIO0D7
#define PWM3_GPIO		RK30_PIN0_PD7
#define PWM3_MUX_MODE		GPIO0D_PWM3

#define PWM_APB_PRE_DIV		1000

#define PWM_DIV			PWM_DIV2

char *pwm0_mux_name=GPIO0A3_PWM0_NAME;
char *pwm1_mux_name=GPIO0A4_PWM1_NAME;
char *pwm2_mux_name=GPIO0D6_PWM2_NAME;
char *pwm3_mux_name=GPIO0D7_PWM3_NAME;

struct pwm_device {
	struct list_head	node;
	struct platform_device	*pdev;

	const char	*label;
	struct clk	*clk;
//	int		clk_enabled;
	void __iomem	*mmio_base;

	unsigned int	use_count;
	unsigned int	pwm_id;
	unsigned int	related_pwm;
};

static DEFINE_MUTEX(pwm_lock);
static LIST_HEAD(pwm_list);


static struct pwm_device *pwm_find(int pwm_id)
{
    struct pwm_device *pwm;
    int found = 0;

    list_for_each_entry(pwm, &pwm_list, node) {
	if (pwm->pwm_id == pwm_id) {
	    found = 1;
	    break;
	}
    }
    if(found)
	return pwm;
    else
	return NULL;
}

static int pwm_status_by_id(int pwm_id)
{
    struct pwm_device *pwm;

    pwm = pwm_find(pwm_id);
    if(pwm)
	return pwm->use_count;
    else
	return -ENODEV;
}

static spinlock_t pwm_hwlock[4] = {
    __SPIN_LOCK_UNLOCKED(pwm_lock0),
    __SPIN_LOCK_UNLOCKED(pwm_lock1),
    __SPIN_LOCK_UNLOCKED(pwm_lock2),
    __SPIN_LOCK_UNLOCKED(pwm_lock3),
};

static void pwm_setup(struct pwm_device *pwm, enum pwm_div div, u32 hrc, u32 lrc, int start)
{
    u32 off = div | PWM_RESET;
    u32 on = div | PWM_ENABLE | PWM_TIMER_EN;
    spinlock_t *lock;
    unsigned long flags;

    if (hrc > lrc) {
	pr_err("invalid hrc %d lrc %d\n", hrc, lrc);
	return;
    }

    if(lrc == 0){// just stop/start oops
	if(start){
	    lock = &pwm_hwlock[pwm->pwm_id];
	    spin_lock_irqsave(lock, flags);
	    barrier();
	    writel_relaxed(on, pwm->mmio_base + PWM_REG_CTRL);
	    dsb();
	    spin_unlock_irqrestore(lock, flags);
	}else{
	    lock = &pwm_hwlock[pwm->pwm_id];
	    spin_lock_irqsave(lock, flags);
	    barrier();
	    writel_relaxed(off, pwm->mmio_base + PWM_REG_CTRL);
	    dsb();
	    spin_unlock_irqrestore(lock, flags);
	}
    }else{
	lock = &pwm_hwlock[pwm->pwm_id];
	spin_lock_irqsave(lock, flags);
	barrier();
	writel_relaxed(off, pwm->mmio_base + PWM_REG_CTRL);
	dsb();
	writel_relaxed(hrc, pwm->mmio_base + PWM_REG_HRC);
	writel_relaxed(lrc, pwm->mmio_base + PWM_REG_LRC);
	writel_relaxed(0, pwm->mmio_base + PWM_REG_CNTR);
	dsb();
	writel_relaxed(on, pwm->mmio_base + PWM_REG_CTRL);
	dsb();
	spin_unlock_irqrestore(lock, flags);
    }
}

#define PWM_START(pwm) pwm_setup(pwm, PWM_DIV, 0, 0, 1);
#define PWM_STOP(pwm) pwm_setup(pwm, PWM_DIV, 0, 0, 0);

int pwm_config(struct pwm_device *pwm, int duty_ns, int period_ns)
{
	unsigned long long c;
	unsigned long period_cycles, pv, dc;

	if (pwm == NULL || period_ns == 0 || duty_ns > period_ns)
		return -EINVAL;

	c = clk_get_rate(pwm->clk) / PWM_APB_PRE_DIV;//TODO check PWM_APB_PRE_DIV
	c = c * period_ns;
	do_div(c, 1000000000);
	period_cycles = c;

	if (period_cycles < 1)
		period_cycles = 1;
//	prescale = (period_cycles - 1) / 0xFFFFFFFF;//32bit pwm counter?
//	prescale = period_cycles / 0xFFFFFFFF;//32bit pwm counter?
	pv = period_cycles >> (1 + (PWM_DIV >> 9));

//	if (prescale > 63)
//		return -EINVAL;

	if (duty_ns == period_ns)
		dc = pv;
	else
		dc = (pv + 1) * duty_ns / period_ns;

	pwm_setup(pwm, PWM_DIV, dc, pv, 1);

	return 0;
}
EXPORT_SYMBOL(pwm_config);

int pwm_enable(struct pwm_device *pwm)
{
	int rc = 0;

	PWM_START(pwm);

/*	if (!pwm->clk_enabled) {
		rc = clk_enable(pwm->clk);
		if (!rc)
			pwm->clk_enabled = 1;
	}*/
	return rc;
}
EXPORT_SYMBOL(pwm_enable);

void pwm_disable(struct pwm_device *pwm)
{
	PWM_STOP(pwm);
/*	if (pwm->clk_enabled) {
		clk_disable(pwm->clk);
		pwm->clk_enabled = 0;
	}*/
}
EXPORT_SYMBOL(pwm_disable);

static int pwm_gpio_setup(int pwm_id, int ToPWM)
{
    unsigned gpio_num=0;
    unsigned gpio_mode;
    char *mux_name="";
    char sbuf[10];
    int ret;

    gpio_mode = !!ToPWM;
    switch(pwm_id){
    case 0:
        gpio_num = PWM0_GPIO;
        mux_name = pwm0_mux_name;
        break;
    case 1:
        gpio_num = PWM1_GPIO;
        mux_name = pwm1_mux_name;
        break;
    case 2:
        gpio_num = PWM2_GPIO;
        mux_name = pwm2_mux_name;
        break;
    case 3:
        gpio_num = PWM3_GPIO;
        mux_name = pwm3_mux_name;
        break;
    }

    if(ToPWM){
	sprintf(sbuf,"PMW%d",pwm_id);
	ret = gpio_request(gpio_num, sbuf);
	if(!ret)
	    return ret;
	rk30_mux_api_set(mux_name, gpio_mode);
    }else{
	rk30_mux_api_set(mux_name, gpio_mode);
	gpio_direction_input(gpio_num);
	gpio_free(gpio_num);
    }

    return 0;
}

struct pwm_device *pwm_request(int pwm_id, const char *label)
{
    struct pwm_device *pwm;

    mutex_lock(&pwm_lock);

    pwm =  pwm_find(pwm_id);

    if(pwm) {
	if(pwm->use_count == 0 && pwm_gpio_setup(pwm_id, 1) == 0) {
	    if(pwm_status_by_id(pwm->related_pwm) < 1){//related pwm not activated
		if(clk_enable(pwm->clk) < 0)
		    return ERR_PTR(-EBUSY);
//		pwm->clk_enabled = 1;
	    }
	    pwm->use_count++;
	    pwm->label = label;
	}else
	    pwm = ERR_PTR(-EBUSY);
    }else
	pwm = ERR_PTR(-ENOENT);

    mutex_unlock(&pwm_lock);
    return pwm;
}
EXPORT_SYMBOL(pwm_request);

void pwm_free(struct pwm_device *pwm)
{
    mutex_lock(&pwm_lock);

    if(pwm->use_count){
	pwm->use_count--;
	pwm->label = NULL;
	pwm_gpio_setup(pwm->pwm_id, 0);

	if(pwm_status_by_id(pwm->related_pwm) < 1){//related pwm not activated
	    clk_disable(pwm->clk);
//	    pwm->clk_enabled = 0;
	}
    }else
	pr_warning("PWM device already freed\n");

    mutex_unlock(&pwm_lock);
}
EXPORT_SYMBOL(pwm_free);

static inline void __add_pwm(struct pwm_device *pwm)
{
	mutex_lock(&pwm_lock);
	list_add_tail(&pwm->node, &pwm_list);
	mutex_unlock(&pwm_lock);
}

static int __devinit pwm_probe(struct platform_device *pdev)
{
	struct pwm_device *pwm;
	struct resource *r;
	int ret = 0, pwm_id = pdev->id;
	
	if(pwm_id < 0 || pwm_id > 3){
		dev_err(&pdev->dev, "only four pwms support\n");
		return -ENODEV;
	}

	pwm = kzalloc(sizeof(struct pwm_device), GFP_KERNEL);
	if (pwm == NULL) {
		dev_err(&pdev->dev, "failed to allocate memory\n");
		return -ENOMEM;
	}

	if (pwm_id == 0 || pwm_id == 1)
	    pwm->clk = clk_get(NULL, "pwm01");
	else if (pwm_id == 2 || pwm_id == 3)
	    pwm->clk = clk_get(NULL, "pwm23");

	if (IS_ERR(pwm->clk)) {
		ret = PTR_ERR(pwm->clk);
		goto err_free;
	}
//	pwm->clk_enabled = 0;

	pwm->use_count = 0;
	pwm->pwm_id = pwm_id;
	pwm->pdev = pdev;

	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (r == NULL) {
		dev_err(&pdev->dev, "no memory resource defined\n");
		ret = -ENODEV;
		goto err_free_clk;
	}

	r = request_mem_region(r->start, resource_size(r), pdev->name);
	if (r == NULL) {
		dev_err(&pdev->dev, "failed to request memory resource\n");
		ret = -EBUSY;
		goto err_free_clk;
	}

	pwm->mmio_base =ioremap(r->start, resource_size(r));
	if (pwm->mmio_base == NULL) {
		dev_err(&pdev->dev, "failed to ioremap() registers\n");
		ret = -ENODEV;
		goto err_free_mem;
	}

	switch(pwm_id){
	case 0:
		pwm->related_pwm = 1;
		break;
	case 1:
		pwm->related_pwm = 0;
		break;
	case 2:
		pwm->related_pwm = 3;
		break;
	case 3:
		pwm->related_pwm = 2;
		break;

	}

	__add_pwm(pwm);

	platform_set_drvdata(pdev, pwm);
	return 0;

err_free_mem:
	release_mem_region(r->start, resource_size(r));
err_free_clk:
	clk_put(pwm->clk);
err_free:
	kfree(pwm);
	return ret;
}

static int __devexit pwm_remove(struct platform_device *pdev)
{
	struct pwm_device *pwm;
	struct resource *r;

	pwm = platform_get_drvdata(pdev);
	if (pwm == NULL)
		return -ENODEV;

	pwm_free(pwm);//paranoid

	mutex_lock(&pwm_lock);

	list_del(&pwm->node);
	mutex_unlock(&pwm_lock);

	iounmap(pwm->mmio_base);

	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	release_mem_region(r->start, resource_size(r));

	if(pwm_status_by_id(pwm->related_pwm) < 0)//related not exist
	    clk_put(pwm->clk);

	kfree(pwm);
	return 0;
}

static struct platform_driver pwm_driver = {
	.driver		= {
		.name	= "rk30-pwm",
		.owner	= THIS_MODULE,
	},
	.probe		= pwm_probe,
	.remove		= __devexit_p(pwm_remove),
};

static int __init pwm_init(void)
{
	return platform_driver_register(&pwm_driver);
}
arch_initcall(pwm_init);

static void __exit pwm_exit(void)
{
	platform_driver_unregister(&pwm_driver);
}
module_exit(pwm_exit);

MODULE_LICENSE("GPL v2");
