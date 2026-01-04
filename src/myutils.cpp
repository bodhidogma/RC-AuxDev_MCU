
// #include "main.h"
// #include "usart.h"
// #include "usbd_cdc_if.h"

// #include <stdio.h>

#include <sys/unistd.h>  // STDOUT_FILENO, STDERR_FILENO

#include "mymain.h"
#include "stm_console.hpp"

#if 0
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}
#endif

#if 1
extern "C" int _write(int file, unsigned char *ptr, int len) {
  if ((file != STDOUT_FILENO) && (file != STDERR_FILENO)) {
    return -1;
  }
#if 0
    static uint8_t rc = USBD_OK;
    //do {
        rc = CDC_Transmit_FS(ptr, len);
    //} while (USBD_BUSY == rc);

    if (USBD_FAIL == rc) {
        /// NOTE: Should never reach here.
        /// TODO: Handle this error.
        return 0;
    }
    return len;
#else
  // arbitrary timeout 1000
  HAL_StatusTypeDef status =
# if 0
      HAL_UART_Transmit(&huart1, (uint8_t *)ptr, len, HAL_MAX_DELAY);
      //HAL_UART_Transmit_IT(&huart1, (uint8_t *)ptr, len); // issues
      //HAL_UART_Transmit_DMA(&huart1, (uint8_t *)ptr, len); // issues
# else
    (HAL_StatusTypeDef)console.Send((const char*)ptr, len);
# endif
  // return # of bytes written - as best we can tell
  return (status == HAL_OK ? len : 0);
#endif
}
#endif
