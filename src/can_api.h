/*
 * can_api.h
 *
 *  Created on: 2020年3月25日
 *      Author: tianqingyuan
 */

#ifndef _MSCAN_API_H__
#define _MSCAN_API_H__
#ifdef __cplusplus
extern "C" {
#endif
/******************************************************************************
* Global variables
******************************************************************************/
#include "CAN.h"
/******************************************************************************
* Constants and macros
******************************************************************************/
/******************************************************************************
* define MSCAN Frame Data Length Definition
*
*//*! @addtogroup frame_data_length_list
* @{
*******************************************************************************/
#define MAX_DATA_LENGTH				64
#define MAX_SINGLE_FRAME_LENGTH		8

/*! @} End of frame_data_length_list											*/

/******************************************************************************
* define MSCAN Frame Buffer Length
*
*//*! @addtogroup frame_buffer_length_list
* @{
*******************************************************************************/
#define CAN_BUFFER_LENGTH		32	/*!< MSCAN Tx or Rx buffer max length */

/*! @} End of frame_buffer_length_list											*/


/******************************************************************************
* Local types
******************************************************************************/
/******************************************************************************
*
*//*! @addtogroup item_info_List
* @{
*******************************************************************************/
/*!
 * @brief Tx or RX data information by CAN
 *
 */
typedef struct
{
	union{
		struct{
			uint32_t EID6_0 	:7;		/*!< ID[0:6] */
			uint32_t EID14_7	:8;		/*!< ID[14:7] */
			uint32_t EID17_15   :3;		/*!< ID[17:15] */
			uint32_t EID20_18  	:3;		/*!< ID[20:18] */
			uint32_t EID28_21   :8;  	/*!< ID[28:21] */
			uint32_t Reverse    :3;
		}ExtendIDType;//扩展帧29位
		struct{
			uint32_t SID2_0 	:3;		/*!< ID[0:2] */
			uint32_t SID10_3	:8;		/*!< ID[10:3] */
			uint32_t Reverse    :21;
		}StandardIDType;//标准帧11位
		uint32_t ID;
	}ID_Type;
	uint32_t u32DataLength;//数据长度
	uint8_t u8DataBuff[MAX_DATA_LENGTH];	/*!>data buffer 数据段*/
	uint8_t u8BPR;							/*!>frame priority 优先级*/
	uint8_t bIsRemoteFrame;//远程传输请求位，数据帧的RTR位为显性电平0，远程帧为隐性电平1（接收节点向具有相同ID的发送节点请求数据的帧），
	uint8_t bIsExtOrStand;//标识符扩展位，1为扩展帧，0为标准帧
	uint8_t u8TSRH;
	uint8_t u8TSRL;
}ItemInfoType,*ItemInfoPtr;
/*! @} End of item_info_List									*/

/******************************************************************************
*
*//*! @addtogroup frame_buffer_info_List
* @{
*******************************************************************************/
/*!
 * @brief save tx data buffer information for transmitting or save the receiving data buffer
 *
 */
typedef struct
{
	MSCAN_RegisterFrameType	sFrameRegisterBuffer[CAN_BUFFER_LENGTH];
	uint8_t u8Head;
	uint8_t u8Index;
	uint8_t u8FreeLength;//空闲空间
}FrameBufferInfoType,*FrameBufferInfoPtr;
/*! @} End of frame_buffer_info_List									*/


/******************************************************************************
* Local function prototypes
******************************************************************************/

/******************************************************************************
* Local variables
******************************************************************************/

extern FrameBufferInfoType	sCAN_TxBuff;
//static FrameBufferInfoType	sCAN_RxBuff;
extern MSCAN_FrameType sRxFrame[CAN_BUFFER_LENGTH];
extern uint8_t u8RxFrameBufferIndex;
extern uint8_t u8RxFrameHeader;
extern uint8_t u8RxFrameBufferFreeLength;
extern uint32_t u32RxInterruptCounter;
/******************************************************************************
* inline functions
******************************************************************************/


/******************************************************************************
* Global functions
******************************************************************************/
uint8_t CAN_SendItemToBuffer(MSCAN_Type *pCANx,ItemInfoPtr pTxItemInfo,FrameBufferInfoPtr pTxBuffInfo);
uint8_t CAN_CheckSendBufferFrame(MSCAN_Type *pCANx,FrameBufferInfoPtr pTxBuffInfo);
uint8_t CAN_TransmitItemByInt(MSCAN_Type *pCANx,ItemInfoPtr pTxItemInfo,FrameBufferInfoPtr pTxBuffInfo);
uint8_t CAN_ReadOneFramefromBufferQueue(MSCAN_FrameType* pRxFrameInfo);
void MSCAN_GlobeVaribleInit( MSCAN_Type *pCANx );
void MSCAN_TxProcessing( void );
void MSCAN_RxProcessing( void );

#ifdef __cplusplus
}
#endif
#endif //


