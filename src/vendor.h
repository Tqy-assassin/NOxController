/*
 * vendor.h
 *
 *  Created on: 2020��12��16��
 *      Author: tianqingyuan
 */

#ifndef VENDOR_H_
#define VENDOR_H_
#include "config.h"
//=========================================================
#define PDU_P_MASK		0x1C000000
#define PDU_P_SHIFT		26
#define PDU_P(x)		(((uint32_t)(((uint32_t)(x))<<PDU_P_SHIFT))&PDU_P_MASK)
#define PDU_R_MASK		0x02000000
#define PDU_R_SHIFT		25
#define PDU_R(x)		(((uint32_t)(((uint32_t)(x))<<PDU_R_SHIFT))&PDU_R_MASK)
#define PDU_DP_MASK		0x01000000
#define PDU_DP_SHIFT	24
#define PDU_DP(x)		(((uint32_t)(((uint32_t)(x))<<PDU_DP_SHIFT))&PDU_DP_MASK)
#define PDU_PGN_MASK	0x00FFFF00
#define PDU_PGN_SHIFT	8
#define PDU_PGN(x)		(((uint32_t)(((uint32_t)(x))<<PDU_PGN_SHIFT))&PDU_PGN_MASK)
#define PDU_SA_MASK		0x000000FF
#define PDU_SA_SHIFT	0
#define PDU_SA(x)		(((uint32_t)(x))&PDU_SA_MASK)

#if VENDOR_ID == GWM_KIND
#include "gwm_vendor.h"
#elif VENDOR_ID == YUC_Y24
#include "YUC_Y24.h"
#elif VENDOR_ID == NTK_KIND
#include "vendor_NTK.h"
#elif VENDOR_ID == BENZ_KIND
#include "vendor_BENZ.h"
#else
#error "Device types do not matched"
#endif

#endif /* VENDOR_H_ */
