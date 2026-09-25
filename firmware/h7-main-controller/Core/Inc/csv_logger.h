#ifndef CSV_LOGGER_H
#define CSV_LOGGER_H

#include <stdint.h>

/* 打开或创建 CSV 文件；存在则追加写入。 */
uint8_t CsvLogger_Open(const char *file_name);
/* 追加写入一整行 CSV 文本。 */
uint8_t CsvLogger_WriteLine(const char *line);
/* 关闭 CSV 文件。 */
void CsvLogger_Close(void);
/* 追加一行 CSV；文件为空时先写 header；函数内部会打开、同步并关闭文件。 */
uint8_t CsvLogger_AppendLine(const char *file_name, const char *header, const char *line);

#endif
