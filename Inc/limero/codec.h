#ifndef _CODEC_H_
#define _CODEC_H_
#include <limero/Log.h>
#include <errno.h>
#include <stddef.h>
#include <stdint.h>
#include <cstring>
#include <functional>
#include <vector>

typedef bool Void;

template <typename T>
class Option
{
private:
    bool _has_value = true;
    T _value;

    Option(bool has_value, T value) : _has_value(has_value), _value(value) {}


public:
    Option() : _has_value(false) {};
    static Option None()
    {
        return Option{};
    }
    void none() { _has_value = false; }
    void some(T value) { _has_value = true; _value = value; }

    static Option Some(T value)
    {
        return Option{ true, value };
    }

    ~Option()
    {

    }
    bool is_some() { return _has_value; }
    bool is_none() { return _has_value; }
    T unwrap() { return _value; }
    void inspect(std::function<void(T)> ff) const
    {
        if (_has_value)
        {
            ff(_value);
        }
    }
    template <typename U>
    Option<U> map(std::function<Option<U>(T)> ff)
    {
        if (_has_value)
        {
            return ff(_value);
        }
        return Option<U>::None();
    }
    void operator = (const T& other)
    {
        _value = other;
        _has_value = true;
    }
};

template <typename T>
class Result
{
    T _value;
    int _err;
    const char* _msg;

public:
    Result() : _value(), _err(0) {}
    bool is_ok() { return _err == 0; }
    bool is_err() { return _err != 0; }
    const char* get_err_msg() { return (_err == 0) ? "No error" : _msg; }
    Result& on_error(std::function<void(const char*)> ff)
    {
        if (_err != 0)
        {
            ff(_msg);
        }
        return *this;
    }
    Result& on_ok(std::function<void(T)> ff)
    {
        if (_err == 0)
        {
            ff(_value);
        }
        else
        {
            ERROR("Error: %d\n", _err);
        }
        return *this;
    }
    T unwrap()
    {
        if (_err != 0)
        {
            ERROR("Error: %d\n", _err);
        }
        return _value;
    }
    static Result<T> Ok(T value)
    {
        Result<T> result;
        result._value = value;
        result._err = 0;
        return result;
    }
    void ok(T value) {
        _value = value;
        _err = 0;
        _msg = "No error";
    }
    static Result<T> Err(int err)
    {
        Result<T> result;
        result._err = err;
        result._msg = strerror(err);
        return result;
    }
    void err(int err, const char* msg = "unknown failure") {
        _err = err;
        _msg = msg;
    }
    static Result<T> Err(int err, const char* msg)
    {
        Result<T> result;
        result._err = err;
        result._msg = msg;
        return result;
    }
    void inspect(std::function<void(T)> ff)
    {
        if (_err == 0)
        {
            ff(_value);
        }
    }
    template <typename U>
    Result<U> map(std::function<Result<U>(T)> ff)
    {
        if (_err == 0)
        {
            return ff(_value);
        }
        return Result<U>::Err(this->_err, this->_msg);
    }
};

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
    FrameEncoder(uint8_t buffer[], uint32_t capacity);
    Result<Void> write_byte(uint8_t byte);

    Result<Void> add_crc();
    Result<Void> add_cobs();
    Result<Void> read_buffer(uint8_t* buffer, size_t len);
    Result<Void> read_buffer(std::vector<unsigned char>& buffer);
    Result<Void> clear();
    Result<Void> rewind();
    Result<std::string> to_string();
    uint8_t* data() { return _buffer; }
    uint32_t size() { return _index; }
    uint32_t capacity() { return _capacity; }
};



class FrameDecoder
{
private:
    std::vector<uint8_t> _buffer;
    uint32_t _index;
    uint32_t _max;

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
    Result<Void> rewind();
    Result<std::string> to_string();
};

// FNV-1a hash function for 32-bit hash value
constexpr uint32_t fnv1a_32_1(const char* str, uint32_t hash = 2166136261U)
{
    return *str == '\0' ? hash : fnv1a_32_1(str + 1, (hash ^ static_cast<uint32_t>(*str)) * 16777619U);
}

// Helper to compute the hash at compile time for a string literal
template <std::size_t N>
constexpr uint32_t FNV(const char(&str)[N])
{
    return fnv1a_32_1(str);
}

typedef std::vector<uint8_t> Bytes;

#include <cbor.h>

class Msg {
    public:
    static uint32_t msg_id() { return FNV("Msg"); };
    static const char* msg_name() { return "Msg"; };
    virtual Result<Void> encode(CborEncoder& encoder) = 0;
    virtual Result<Void> decode(CborValue& cbor_value) = 0;
};


#endif