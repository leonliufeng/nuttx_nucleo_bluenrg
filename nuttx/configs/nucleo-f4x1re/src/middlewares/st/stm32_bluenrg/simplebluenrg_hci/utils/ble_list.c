/******************** (C) COPYRIGHT 2012 STMicroelectronics ********************
* File Name          : ble_list.c
* Author             : AMS - HEA&RF BU
* Version            : V1.0.0
* Date               : 19-July-2012
* Description        : Circular Linked List Implementation.
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/******************************************************************************
 * Include Files
******************************************************************************/
#include <arch/board/middlewares/st/stm32_bluenrg/simplebluenrg_hci/hal_types.h>
#include <arch/board/middlewares/st/stm32_bluenrg/simplebluenrg_hci/ble_list.h>

#ifdef USE_STM32F4XX_NUCLEO
  #include <arch/board/drivers/stm32f4xx_hal_driver/inc/stm32f4xx_hal.h>
#endif /* USE_STM32F4XX_NUCLEO */

#ifdef USE_STM32L0XX_NUCLEO
#include "stm32l0xx_hal.h"
#endif /* USE_STM32L0XX_NUCLEO */

#ifdef USE_STM32L4XX_NUCLEO
#include "stm32l4xx_hal.h"
#endif /* USE_STM32L4XX_NUCLEO */

/******************************************************************************
 * Function Definitions 
******************************************************************************/
void list_init_head (tListNode * listHead)
{
  listHead->next = listHead;
  listHead->prev = listHead;	
}

uint8_t list_is_empty (tListNode * listHead)
{
  uint32_t uwPRIMASK_Bit;
  uint8_t return_value;

  if(listHead->next == listHead)
  {
    return_value = TRUE;
  }
  else
  {
    return_value = FALSE;
  }

  return return_value;
}

void list_insert_head (tListNode * listHead, tListNode * node)
{
  uint32_t uwPRIMASK_Bit;

  
  node->next = listHead->next;
  node->prev = listHead;
  listHead->next = node;
  (node->next)->prev = node;
  
}

void list_insert_tail (tListNode * listHead, tListNode * node)
{
  uint32_t uwPRIMASK_Bit;

  
  node->next = listHead;
  node->prev = listHead->prev;
  listHead->prev = node;
  (node->prev)->next = node;
  
}

void list_remove_node (tListNode * node)
{
  uint32_t uwPRIMASK_Bit;

  
  (node->prev)->next = node->next;
  (node->next)->prev = node->prev;
  
}

void list_remove_head (tListNode * listHead, tListNode ** node )
{
  uint32_t uwPRIMASK_Bit;

  
  *node = listHead->next;
  list_remove_node (listHead->next);
  (*node)->next = NULL;
  (*node)->prev = NULL;
  
}

void list_remove_tail (tListNode * listHead, tListNode ** node )
{
  uint32_t uwPRIMASK_Bit;

  
  *node = listHead->prev;
  list_remove_node (listHead->prev);
  (*node)->next = NULL;
  (*node)->prev = NULL;
  
}

void list_insert_node_after (tListNode * node, tListNode * ref_node)
{
  uint32_t uwPRIMASK_Bit;

  
  node->next = ref_node->next;
  node->prev = ref_node;
  ref_node->next = node;
  (node->next)->prev = node;
  
}

void list_insert_node_before (tListNode * node, tListNode * ref_node)
{
  uint32_t uwPRIMASK_Bit;

  
  node->next = ref_node;
  node->prev = ref_node->prev;
  ref_node->prev = node;
  (node->prev)->next = node;
  
}

int list_get_size (tListNode * listHead)
{
  int size = 0;
  tListNode * temp;
  uint32_t uwPRIMASK_Bit;


  temp = listHead->next;
  while (temp != listHead)
  {
    size++;
    temp = temp->next;		
  }
  
  return (size);
}

void list_get_next_node (tListNode * ref_node, tListNode ** node)
{
  uint32_t uwPRIMASK_Bit;

  
  *node = ref_node->next;
    
}

void list_get_prev_node (tListNode * ref_node, tListNode ** node)
{
  uint32_t uwPRIMASK_Bit;

  
  *node = ref_node->prev;
  
}

