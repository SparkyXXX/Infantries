/*
 * @Project: Infantry Code
 *
 * @Author: GDDG08
 * @Date: 2021-12-31 17:37:14
 * @LastEditors: Hatrix
 * @LastEditTime: 2024-07-17 19:21:06
 */
#pragma once

#include "cmsis_compiler.h"
#include <algorithm>
#include <cstddef>

namespace sheriff
{
    constexpr uint16_t RoundUpToPowerOfTwo(uint16_t x)
    {
        if ((x & (x - 1)) == 0)
            return x;
        x--;
        x |= x >> 1;
        x |= x >> 2;
        x |= x >> 4;
        x |= x >> 8;
        x++;
        return x;
    }

    template <typename T, uint16_t Len_T>
    class RingBuffer
    {
        static_assert(Len_T >= 2, "Length must be greater than or equal to 2");
        static_assert(Len_T <= 2048, "Length must be less than or equal to 2048");

    public:
        static constexpr uint16_t Capacity = RoundUpToPowerOfTwo(Len_T);
        static_assert(Capacity * sizeof(T) <= 2048, "Size must be less than or equal to 2048");

    private:
        uint16_t _in{0};
        uint16_t _out{0};
        T _buffer[Capacity];

    public:
        RingBuffer() = default;

        uint16_t Write(const T *buffer, uint16_t len)
        {
            uint16_t l;
            len = std::min(len, uint16_t(Capacity - _in + _out));

            __DMB();

            l = std::min(len, uint16_t(Capacity - (_in & (Capacity - 1))));
            memcpy(_buffer + (_in & (Capacity - 1)), buffer, l * sizeof(T));

            memcpy(_buffer, buffer + l, (len - l) * sizeof(T));

            __DMB();

            _in += len;

            return len;
        }

        uint16_t Write(const T &in)
        {
            return Write(&in, 1);
        }

        uint16_t Read(T *buffer, uint16_t len)
        {
            uint16_t l;

            len = std::min(len, uint16_t(_in - _out));

            __DMB();

            l = std::min(len, uint16_t(Capacity - (_out & (Capacity - 1))));
            memcpy(buffer, _buffer + (_out & (Capacity - 1)), l * sizeof(T));

            memcpy(buffer + l, _buffer, (len - l) * sizeof(T));

            __DMB();

            _out += len;

            return len;
        }

        T Read()
        {
            T res;
            Read(&res, 1);
            return std::move(res);
        }

        uint16_t Read(T &out)
        {
            return Read(&out, 1);
        }

        T operator[](uint16_t i)
        {
            return _buffer[(_in + i) & (Capacity - 1)];
        }

        RingBuffer &operator<<(const T &in)
        {
            Write(in);
            return *this;
        }

        RingBuffer &operator>>(T &out)
        {
            Read(out);
            return *this;
        }
    };

    template <typename T, size_t Len_T>
    class SlidingWindow
    {
    private:
        RingBuffer<T, Len_T> _ring_buffer;

    public:
        explicit SlidingWindow(const T &init_value)
        {
            for (size_t i = 0; i < Len_T; i++)
            {
                _ring_buffer.Write(init_value);
            }
        }

        explicit SlidingWindow()
        {
            T t{};
            for (size_t i = 0; i < Len_T; i++)
            {
                _ring_buffer.Write(t);
            }
        }

        SlidingWindow &operator<<(const T &in)
        {
            _ring_buffer.Write(in);
            _ring_buffer.Read();
            return *this;
        }

        T operator[](size_t i)
        {
            return _ring_buffer[i];
        }
    };
} // namespace sheriff
