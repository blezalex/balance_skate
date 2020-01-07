
#include "settings.h"
#include "../proto/protocol.pb.h"
#include <pb_encode.h>
#include <pb_decode.h>


#define CONFIG_SIZE_MAX 1024
#define CONFIG_FLASH_PAGE_ADDR (FLASH_BASE + 256*1024 - CONFIG_SIZE_MAX)

bool saveToBufferFn(pb_ostream_t *stream, const uint8_t *buf, size_t count) {
	uint8_t** out = (uint8_t**)stream->state;

	for (size_t i = 0; i < count; i++) {
		*((*out)++) = *(buf++);
	}
	return true;
}

int32_t saveProtoToBuffer(uint8_t* buffer, int16_t max_size, const pb_field_t fields[], const void *src_struct) {
	pb_ostream_t sizestream = { 0 };
	pb_encode(&sizestream, fields, src_struct);

	if (sizestream.bytes_written > max_size) {
		return -sizestream.bytes_written;
	}

	pb_ostream_t save_stream = { &saveToBufferFn, &buffer, max_size, 0 };
	if (!pb_encode(&save_stream, fields, src_struct)) {
		return -1;
	}

	return sizestream.bytes_written;
}

bool saveSettingsToFlash(const Config &config) {
  uint8_t buffer[255];
  int32_t size = saveProtoToBuffer(buffer, sizeof(buffer), Config_fields,
                                   &config);
  if (size < 0)
    return false;

  if (HAL_FLASH_Unlock() != HAL_OK) {
    return false;
  }

  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGSERR );
  FLASH_Erase_Sector(5, VOLTAGE_RANGE_3);

  if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
                        (uint32_t) CONFIG_FLASH_PAGE_ADDR, size)) {
    return false;
  }
  int size_round_up = (size + 3) / 4;
  for (int i = 0; i < size_round_up; i++) {
    // +4 for size block
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
                          (uint32_t) CONFIG_FLASH_PAGE_ADDR + i * 4 + 4,
                          ((uint32_t*) buffer)[i]) != HAL_OK) {
      return false;
    }
  }

  HAL_FLASH_Lock();
  return true;
}


bool read_fn(pb_istream_t *stream, uint8_t *buf, size_t count) {
	uint32_t* read_offset = (uint32_t*)stream->state;
//	if (*read_offset + count > (CONFIG_FLASH_PAGE_ADDR + CONFIG_SIZE_MAX))
//		return false;

	memcpy(buf, (uint8_t*)*read_offset, count);
	*read_offset += count;
	return true;
}

bool readSettingsFromBuffer(Config* config, const uint8_t* data, uint32_t data_len) {
	uint32_t read_offset = (uint32_t)data;
	pb_istream_t stdinstream = { &read_fn, &read_offset, data_len };

	return pb_decode(&stdinstream, Config_fields, config);
}

bool readSettingsFromFlash(Config* config) {
	uint32_t size = *(uint32_t*)CONFIG_FLASH_PAGE_ADDR;
	return readSettingsFromBuffer(config, (const uint8_t*)(CONFIG_FLASH_PAGE_ADDR + 4), size);
}


