/*
 * can_api.h
 *
 *  Created on: 2020��3��25��
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
		}ExtendIDType;//��չ֡29λ
		struct{
			uint32_t SID2_0 	:3;		/*!< ID[0:2] */
			uint32_t SID10_3	:8;		/*!< ID[10:3] */
			uint32_t Reverse    :21;
		}StandardIDType;//��׼֡11λ
		uint32_t ID;
	}ID_Type;
	uint32_t u32DataLength;//���ݳ���
	uint8_t u8DataBuff[MAX_DATA_LENGTH];	/*!>data buffer ���ݶ�*/
	uint8_t u8BPR;							/*!>frame priority ���ȼ�*/
	uint8_t bIsRemoteFrame;//Զ�̴�������λ������֡��RTRλΪ���Ե�ƽ0��Զ��֡Ϊ���Ե�ƽ1�����սڵ��������ͬID�ķ��ͽڵ��������ݵ�֡����
	uint8_t bIsExtOrStand;//��ʶ����չλ��1Ϊ��չ֡��0Ϊ��׼֡
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
	uint8_t u8FreeLength;//���пռ�
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


