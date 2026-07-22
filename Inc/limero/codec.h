#ifndef _CODEC_H_
#define _CODEC_H_
#include <log.h>
#include <errno.h>
#include <stddef.h>
#include <stdint.h>
#include <cstring>
#include <functional>
#include <vector>
#include "result.h"
#include "option.h"
#include <assert.h>
#include <msg.h>

#define RET_ERR(x)    \
    if ((x).is_err()) \
    {                 \
        return x;     \
    }
#define RET_ER(x)     \
    if ((x).is_err()) \
    {                 \
        return;       \
    }

class FrameEncoder
{
private:
    uint8_t* _buffer;
    uint32_t _capacity;
    uint32_t _index;

public:
    FrameEncoder(uint8_t buffer[], uint32_t capacity, uint32_t size = 0);
    Result<Void> add_byte(uint8_t byte);
    Result<Void> add_crc();
    Result<Void> add_cobs();
    Result<Void> rewind();
    Result<std::string> to_string();
    uint8_t* data() { return _buffer; }
    uint32_t size() { return _index; }
    uint32_t capacity() { return _capacity; }
};



class FrameDecoder
{
private:
    Buffer _buffer;

public:
    FrameDecoder(uint32_t max);
    Result<uint8_t> read_next();
    Result<bool> check_crc();
    Result<Void> decode_cobs();
    Result<bool> add_byte(uint8_t byte);
    Result<bool> fill_buffer(std::vector<uint8_t> buffer);
    Result<bool> fill_buffer(uint8_t* buffer, uint32_t size);
    Result<Void> read_buffer(uint8_t* buffer, size_t len);
    Result<Void> read_buffer(std::vector<unsigned char>& buffer);
    Result<Void> clear();
    void rewind();
    uint8_t* data() { return _buffer.data(); }
    uint32_t size() { return _buffer.size(); }
    uint32_t capacity() { return _buffer.capacity(); }
};



#endif