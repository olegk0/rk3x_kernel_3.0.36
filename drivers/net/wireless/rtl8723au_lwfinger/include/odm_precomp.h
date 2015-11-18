/******************************************************************************
 *
 * Copyright(c) 2007 - 2011 Realtek Corporation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110, USA
 *
 *
 ******************************************************************************/

#ifndef	__ODM_PRECOMP_H__
#define __ODM_PRECOMP_H__

#include "odm_types.h"

#define		TEST_FALG___		1

//2 Config Flags and Structs - defined by each ODM Type

#include <drv_conf.h>
#include <osdep_service.h>
#include <drv_types.h>
#include <hal_intf.h>


//2 Hardware Parameter Files
#include "Hal8723UHWImg_CE.h"


//2 OutSrc Header Files

#include "odm.h"
#include "odm_HWConfig.h"
#include "odm_debug.h"
#include "odm_RegDefine11AC.h"
#include "odm_RegDefine11N.h"

#if (RTL8192C_SUPPORT==1)
#include  "HalDMOutSrc8192C_CE.h"
#include <rtl8192c_hal.h>
#elif (RTL8192D_SUPPORT==1)
#include  "HalDMOutSrc8192D_CE.h"
#include  "rtl8192d_hal.h"
#elif (RTL8723A_SUPPORT==1)
#include "HalDMOutSrc8192C_CE.h" //for IQK,LCK,Power-tracking
#include "rtl8723a_hal.h"
#elif (RTL8188E_SUPPORT==1)
#include "HalPhyRf_8188e.h"//for IQK,LCK,Power-tracking
#include "Hal8188ERateAdaptive.h"//for  RA,Power training
#include "rtl8188e_hal.h"
#endif

#include "odm_interface.h"
#include "odm_reg.h"

#if (RTL8192C_SUPPORT==1)
#include "odm_RTL8192C.h"
#endif
#if (RTL8192D_SUPPORT==1)
#include "odm_RTL8192D.h"
#endif

#if (RTL8723A_SUPPORT==1)
#include "HalHWImg8723A_MAC.h"
#include "HalHWImg8723A_RF.h"
#include "HalHWImg8723A_BB.h"
#include "HalHWImg8723A_FW.h"
#include "odm_RegConfig8723A.h"
#endif

#if (RTL8188E_SUPPORT==1)
#include "HalHWImg8188E_MAC.h"
#include "HalHWImg8188E_RF.h"
#include "HalHWImg8188E_BB.h"
#include "Hal8188EReg.h"

#if (RTL8188E_FOR_TEST_CHIP >= 1)
#include "HalHWImg8188E_TestChip_MAC.h"
#include "HalHWImg8188E_TestChip_RF.h"
#include "HalHWImg8188E_TestChip_BB.h"
#endif

#ifdef CONFIG_WOWLAN
#if (RTL8188E_SUPPORT==1)
#include "HalHWImg8188E_FW.h"
#endif
#endif //CONFIG_WOWLAN

#include "odm_RegConfig8188E.h"
#include "odm_RTL8188E.h"
#endif

#endif	// __ODM_PRECOMP_H__
