#include "csv_logger.h"
#include "fatfs.h"
#include <string.h>

static FIL g_csv_file;
static uint8_t g_csv_open = 0U;

uint8_t CsvLogger_Open(const char *file_name)
{
  FRESULT fr;
  const char *path = (file_name != NULL) ? file_name : "0:/gesture.csv";

  fr = f_mount(&SDFatFS, SDPath, 1U);
  if (fr != FR_OK)
  {
    return 1U;
  }

  fr = f_open(&g_csv_file, path, FA_OPEN_ALWAYS | FA_WRITE);
  if (fr != FR_OK)
  {
    return 2U;
  }

  fr = f_lseek(&g_csv_file, f_size(&g_csv_file));
  if (fr != FR_OK)
  {
    (void)f_close(&g_csv_file);
    return 3U;
  }

  g_csv_open = 1U;
  return 0U;
}

uint8_t CsvLogger_WriteLine(const char *line)
{
  UINT bw = 0U;
  FRESULT fr;

  if ((g_csv_open == 0U) || (line == NULL))
  {
    return 1U;
  }

  fr = f_write(&g_csv_file, line, (UINT)strlen(line), &bw);
  if ((fr != FR_OK) || (bw != strlen(line)))
  {
    return 2U;
  }

  fr = f_sync(&g_csv_file);
  if (fr != FR_OK)
  {
    return 3U;
  }

  return 0U;
}

void CsvLogger_Close(void)
{
  if (g_csv_open != 0U)
  {
    (void)f_close(&g_csv_file);
    g_csv_open = 0U;
  }
}

uint8_t CsvLogger_AppendLine(const char *file_name, const char *header, const char *line)
{
  FRESULT fr;
  UINT bw = 0U;
  const char *path = (file_name != NULL) ? file_name : "0:/gesture.csv";

  if (line == NULL)
  {
    return 1U;
  }

  if (g_csv_open == 0U)
  {
    fr = f_mount(&SDFatFS, SDPath, 1U);
    if (fr != FR_OK)
    {
      return 2U;
    }

    fr = f_open(&g_csv_file, path, FA_OPEN_ALWAYS | FA_WRITE);
    if (fr != FR_OK)
    {
      return 3U;
    }

    if ((f_size(&g_csv_file) == 0U) && (header != NULL))
    {
      fr = f_write(&g_csv_file, header, (UINT)strlen(header), &bw);
      if ((fr != FR_OK) || (bw != strlen(header)))
      {
        (void)f_close(&g_csv_file);
        return 4U;
      }
    }

    fr = f_lseek(&g_csv_file, f_size(&g_csv_file));
    if (fr != FR_OK)
    {
      (void)f_close(&g_csv_file);
      return 5U;
    }

    g_csv_open = 1U;
  }

  bw = 0U;
  fr = f_write(&g_csv_file, line, (UINT)strlen(line), &bw);
  if ((fr != FR_OK) || (bw != strlen(line)))
  {
    CsvLogger_Close();
    return 6U;
  }

  fr = f_sync(&g_csv_file);
  if (fr != FR_OK)
  {
    CsvLogger_Close();
    return 7U;
  }

  return 0U;
}
