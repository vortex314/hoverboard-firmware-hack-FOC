#include <limero/codec.h>

// CRC-16 function (CRC-CCITT)
uint16_t crc16(const uint8_t *data, size_t length)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < length; i++)
    {
        crc ^= data[i] << 8;
        for (uint8_t j = 0; j < 8; j++)
        {
            if (crc & 0x8000)
            {
                crc = (crc << 1) ^ 0x1021;
            }
            else
            {
                crc <<= 1;
            }
        }
    }
    return crc;
}

// COBS encoding function
std::vector<uint8_t> cobs_encode(const std::vector<uint8_t> &input)
{
    std::vector<uint8_t> output(input.size() + 2);
    size_t read_index = 0, write_index = 1, code_index = 0;
    uint8_t code = 1;

    while (read_index < input.size())
    {
        if (input[read_index] == 0)
        {
            output[code_index] = code;
            code_index = write_index++;
            code = 1;
        }
        else
        {
            output[write_index++] = input[read_index];
            code++;
            if (code == 0xFF)
            {
                output[code_index] = code;
                code_index = write_index++;
                code = 1;
            }
        }
        read_index++;
    }
    output[code_index] = code;
    output[write_index++] = 0; // COBS terminator

    output.resize(write_index);
    return output;
}

// COBS decoding function
std::vector<uint8_t> cobs_decode(const std::vector<uint8_t> &input)
{
    std::vector<uint8_t> output(input.size());
    size_t read_index = 0, write_index = 0;
    uint8_t code = 0, i = 0;

    while (read_index < input.size())
    {
        code = input[read_index];
        if (read_index + code > input.size() && code != 1)
        {
            output.clear();
            return output;
            // throw std::runtime_error("COBS decode error");
        }
        read_index++;
        for (i = 1; i < code; i++)
        {
            output[write_index++] = input[read_index++];
        }
        if (code != 0xFF && read_index < input.size())
        {
            output[write_index++] = 0;
        }
    }

    output.resize(write_index);
    return output;
}

// FrameEncoder Class

FrameEncoder::FrameEncoder(uint8_t buffer[], uint32_t capacity, uint32_t size) : _buffer(buffer), _capacity(capacity), _index(size)
{
    assert(_buffer != nullptr);
    assert(_capacity > 0);
    assert(_index <= _capacity);
}

Result<Void> FrameEncoder::add_byte(uint8_t byte)
{
    if (_index + 1 > _capacity)
    {
        return Result<Void>::Err(ENOSPC, "Buffer overflow");
    }
    _buffer[_index++] = byte;
    return Result<Void>::Ok(Void());
}

Result<Void> FrameEncoder::add_crc()
{
    uint16_t crc = crc16(_buffer, _index);
    RET_ERR(add_byte(crc >> 8));
    RET_ERR(add_byte(crc & 0xFF));
    return Result<Void>::Ok(Void());
}

Result<Void> FrameEncoder::add_cobs()
{
    std::vector<uint8_t> encoded = cobs_encode(std::vector<uint8_t>(_buffer, _buffer + _index));
    if (encoded.size() > _capacity)
    {
        return Result<Void>::Err(ENOSPC, "COBS encoded data exceeds buffer capacity");
    }
    std::memcpy(_buffer, encoded.data(), encoded.size());
    _index = encoded.size();
    return Result<Void>::Ok(Void());
}


Result<Void> FrameEncoder::rewind()
{
    _index = 0;
    return Result<Void>::Ok(Void());
}

//================================================================

FrameDecoder::FrameDecoder(uint32_t capacity) : _buffer(capacity)
{
}


Result<Void> FrameDecoder::decode_cobs()
{
    _buffer = cobs_decode(_buffer.to_vector());
    if (_buffer.size() == 0)
    {
        return Result<Void>::Err(EINVAL, "COBS decode error");
    }
    return Result<Void>::Ok(Void());
}

Result<bool> FrameDecoder::add_byte(uint8_t byte)
{
   return  _buffer.push_back(byte) ? Result<bool>::Err(ENOSPC, "Buffer overflow") : Result<bool>::Ok(false);
}

Result<bool> FrameDecoder::check_crc()
{
    if (_buffer.size() < 2)
    {
        return Result<bool>::Err(ENOSPC, "Buffer too small for CRC");
    }
    uint16_t crc_received = (_buffer[_buffer.size() - 2] << 8) | _buffer[_buffer.size() - 1];
    uint16_t crc_calculated = crc16(_buffer.data(), _buffer.size() - 2);
    if (crc_received != crc_calculated)
    {
        return Result<bool>::Err(EFAULT, "CRC check failed");
    }
    return Result<bool>::Ok(true);
}

Result<Void> FrameDecoder::clear()
{
    _buffer.clear();
    return Result<Void>::Ok(Void());
}

void FrameDecoder::rewind()
{
    _buffer.clear();
}
