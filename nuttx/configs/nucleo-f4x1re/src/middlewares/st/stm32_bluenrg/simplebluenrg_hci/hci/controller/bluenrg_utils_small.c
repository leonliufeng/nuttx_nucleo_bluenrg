
#include <arch/board/middlewares/st/stm32_bluenrg/simplebluenrg_hci/hal.h>
#include <arch/board/middlewares/st/stm32_bluenrg/simplebluenrg_hci/hal_types.h>
#include <arch/board/middlewares/st/stm32_bluenrg/simplebluenrg_hci/ble_status.h>
#include <arch/board/middlewares/st/stm32_bluenrg/simplebluenrg_hci/bluenrg_aci.h>
#include <arch/board/middlewares/st/stm32_bluenrg/simplebluenrg_hci/bluenrg_utils.h>
#include <arch/board/middlewares/st/stm32_bluenrg/simplebluenrg_hci/hci.h>
#include <arch/board/middlewares/st/stm32_bluenrg/simplebluenrg_hci/osal.h>
#include "string.h"
#include <arch/board/drivers/bsp/x-nucleo-idb04a1/stm32_bluenrg_ble.h>


uint8_t getBlueNRGVersion(uint8_t *hwVersion, uint16_t *fwVersion)
{
  uint8_t status;
  uint8_t hci_version, lmp_pal_version;
  uint16_t hci_revision, manufacturer_name, lmp_pal_subversion;

  status = hci_le_read_local_version(&hci_version, &hci_revision, &lmp_pal_version, 
				     &manufacturer_name, &lmp_pal_subversion);

  if (status == BLE_STATUS_SUCCESS) {
    *hwVersion = hci_revision >> 8;
    *fwVersion = (hci_revision & 0xFF) << 8;              // Major Version Number
    *fwVersion |= ((lmp_pal_subversion >> 4) & 0xF) << 4; // Minor Version Number
    *fwVersion |= lmp_pal_subversion & 0xF;               // Patch Version Number
  }

  HCI_Process(); // To receive the BlueNRG EVT

  return status;
}
