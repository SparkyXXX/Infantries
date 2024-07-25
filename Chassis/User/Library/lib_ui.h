/*
 * @Project: Infantry Code
 *
 * @Author: GDDG08
 * @Date: 2021-12-31 17:37:14
 * @LastEditors: Hatrix
 * @LastEditTime: 2024-07-25 10:53:54
 */
#pragma once

#include "lib_crc.h"
#include "list"

namespace UI
{
    constexpr uint8_t SOF = 0xA5;

    enum class Color : uint8_t
    {
        Self = 0,
        Yellow = 1,
        Green = 2,
        Orange = 3,
        Purple = 4,
        Pink = 5,
        Cyan = 6,
        Black = 7,
        White = 8,
        Transparent = 9
    };

    using UiStruct = struct __attribute__((packed))
    {
        uint8_t figure_name[3];
        uint32_t operate_tpye : 3;
        uint32_t figure_tpye : 3;
        uint32_t layer : 4;
        uint32_t color : 4;
        uint32_t details_a : 9;
        uint32_t details_b : 9;
        uint32_t width : 10;
        uint32_t start_x : 11;
        uint32_t start_y : 11;
        uint32_t details_c : 10;
        uint32_t details_d : 11;
        uint32_t details_e : 11;
    };

    using UiHeader = struct __attribute__((packed))
    {
        uint16_t data_cmd_id;
        uint16_t sender_ID;
        uint16_t receiver_ID;
    };

    class Shape
    {
    public:
        using XY = struct
        {
            uint16_t x;
            uint16_t y;
        };

    protected:
        uint8_t _color{0};
        uint8_t _width{1};
        XY _first_dot{0, 0};

    public:
        Shape(Color color, uint8_t width, XY first_point)
            : _color{static_cast<uint8_t>(color)}, _width{width}, _first_dot{first_point} {};

        void update(Color color, uint8_t width, XY first_point)
        {
            _color = static_cast<uint8_t>(color);
            _width = width;
            _first_dot = first_point;
        }

        virtual void drawTo(UiStruct *data)
        {
            data->color = _color;
            data->width = _width;
            data->start_x = _first_dot.x;
            data->start_y = _first_dot.y;
        };

        void changeFirstPoint(XY first_point)
        {
            _first_dot = first_point;
        }

        void changeColor(Color color)
        {
            _color = static_cast<uint8_t>(color);
        }

        void changeWidth(uint8_t width)
        {
            _width = width;
        }
    };

    class Line : public Shape
    {
    private:
        XY _second_point;

    public:
        Line(Color color, uint8_t width, XY first_point, XY second_point)
            : Shape(color, width, first_point), _second_point{second_point} {}

        void update(Color color, uint8_t width, XY first_point, XY second_point)
        {
            Shape::update(color, width, first_point);

            _second_point = second_point;
        };

        void drawTo(UiStruct *data) override
        {
            Shape::drawTo(data);

            data->details_d = _second_point.x;
            data->details_e = _second_point.y;

            data->figure_tpye = 0;
        }

        void changeSecondPoint(XY point)
        {
            _second_point = point;
        }
    };

    class Rectangle : public Shape
    {
    private:
        XY _second_point;

    public:
        Rectangle(Color color, uint8_t width, XY first_point, XY second_point)
            : Shape(color, width, first_point), _second_point{second_point} {}

        void update(Color color, uint8_t width, XY first_point, XY second_point)
        {
            Shape::update(color, width, first_point);

            _second_point = second_point;
        };

        void drawTo(UiStruct *data) override
        {
            Shape::drawTo(data);

            data->details_d = _second_point.x;
            data->details_e = _second_point.y;

            data->figure_tpye = 1;
        }

        void changeSecondPoint(XY point)
        {
            _second_point = point;
        }
    };

    class Circle : public Shape
    {
    private:
        uint16_t _radius{0};

    public:
        Circle(Color color, uint8_t width, XY first_point, uint16_t radius)
            : Shape(color, width, first_point), _radius{radius} {}

        void update(Color color, uint8_t width, XY first_point, uint16_t radius)
        {
            Shape::update(color, width, first_point);

            _radius = radius;
        }

        void drawTo(UiStruct *data) override
        {
            Shape::drawTo(data);

            data->details_c = _radius;

            data->figure_tpye = 2;
        }

        void changeRadius(uint16_t radius)
        {
            _radius = radius;
        }
    };

    class Ellipse : public Shape
    {
    private:
        XY _length{0, 0};

    public:
        Ellipse(Color color, uint8_t width, XY first_point, XY length)
            : Shape(color, width, first_point), _length{length} {}

        void update(Color color, uint8_t width, XY first_point, XY length)
        {
            Shape::update(color, width, first_point);

            _length = length;
        }

        void drawTo(UiStruct *data) override
        {
            Shape::drawTo(data);

            data->details_d = _length.x;
            data->details_e = _length.y;

            data->figure_tpye = 3;
        }

        void changeLength(XY length)
        {
            _length = length;
        }
    };

    class Arc : public Shape
    {
    private:
        uint16_t _from{0};
        uint16_t _to{0};
        XY _length{0};

    public:
        Arc(Color color, uint8_t width, XY first_point, uint16_t from, uint16_t to, XY length)
            : Shape(color, width, first_point), _length{length}, _from{from}, _to{to} {}

        void update(Color color, uint8_t width, XY first_point, uint16_t from, uint16_t to, XY length)
        {
            Shape::update(color, width, first_point);

            _from = from;
            _to = to;
            _length = length;
        }

        void drawTo(UiStruct *data) override
        {
            Shape::drawTo(data);

            data->details_a = _from;
            data->details_b = _to;

            data->details_d = _length.x;
            data->details_e = _length.y;

            data->figure_tpye = 4;
        }

        void changeFrom(uint16_t from)
        {
            _from = from;
        }

        void changeTo(uint16_t to)
        {
            _to = to;
        }

        void changeLength(XY length)
        {
            _length = length;
        }
    };

    class Float : public Shape
    {
    private:
        uint16_t _size;
        union
        {
            struct
            {
                uint32_t c : 10;
                uint32_t d : 11;
                uint32_t e : 11;
            } detail;
            int32_t value;
        } _transfer{};

    public:
        Float(Color color, uint8_t width, XY first_point, uint16_t size, float value)
            : Shape(color, width, first_point), _size{size}
        {
            _transfer.value = static_cast<int32_t>(value * 1000);
        }

        void update(Color color, uint8_t width, XY first_point, uint16_t size, float value)
        {
            Shape::update(color, width, first_point);

            _size = size;
            _transfer.value = static_cast<int32_t>(value * 1000);
        }

        void drawTo(UiStruct *data) override
        {
            Shape::drawTo(data);

            data->figure_tpye = 5;
            data->details_a = _size;
            data->details_c = _transfer.detail.c;
            data->details_d = _transfer.detail.d;
            data->details_e = _transfer.detail.e;
        }

        void changeSize(uint16_t size)
        {
            _size = size;
        }

        void changeValue(float value)
        {
            _transfer.value = static_cast<int32_t>(value * 1000);
        }
    };

    class Int : public Shape
    {
    private:
        uint16_t _size;
        union
        {
            struct
            {
                uint32_t c : 10;
                uint32_t d : 11;
                uint32_t e : 11;
            } detail;
            int32_t value;
        } _transfer{};

    public:
        Int(Color color, uint8_t width, XY first_point, uint16_t size, int32_t value)
            : Shape(color, width, first_point), _size{size}
        {
            _transfer.value = value;
        }

        void update(Color color, uint8_t width, XY first_point, uint16_t size, int32_t value)
        {
            Shape::update(color, width, first_point);

            _size = size;
            _transfer.value = value;
        }

        void drawTo(UiStruct *data) override
        {
            Shape::drawTo(data);

            data->figure_tpye = 6;
            data->details_a = _size;
            data->details_c = _transfer.detail.c;
            data->details_d = _transfer.detail.d;
            data->details_e = _transfer.detail.e;
        }

        void changeSize(uint16_t size)
        {
            _size = size;
        }

        void changeValue(int32_t value)
        {
            _transfer.value = value;
        }
    };

    class Layer
    {
    private:
        Shape *_shapeArray[7]{nullptr};
        uint8_t _shapeArrayTop{0};
        uint8_t _name[2];
        uint8_t _layer;

    public:
        Layer(const char *name, uint8_t layer) : _name{name[0], name[1]}, _layer{layer} {};

        Layer(const char *name, uint8_t layer, std::initializer_list<Shape *> shape_list)
            : _name{name[0], name[1]}, _layer{layer}
        {
            for (auto *x : shape_list)
            {
                if (_shapeArrayTop > 6)
                {
                    break;
                }
                _shapeArray[_shapeArrayTop++] = x;
            }
        };

        bool add(Shape *ptr)
        {
            if (_shapeArrayTop > 6)
            {
                return false;
            }
            _shapeArray[_shapeArrayTop++] = ptr;
            return true;
        }

        void getInitBuffer(uint8_t *buffer, uint16_t id, uint16_t client_id)
        {
            memset(buffer, 0, 7 + 15 * 7 + 8);
            buffer[0] = SOF;

            auto *data_length_ptr = (uint16_t *)(buffer + 1);
            *data_length_ptr = 15 * 7 + 8;

            buffer[3] = 1;
            buffer[4] = CRC_GetCRC8CheckSum(buffer, 4, CRC8_INIT);

            buffer[5] = 0x01;
            buffer[6] = 0x03;

            auto *header = (UiHeader *)(buffer + 7);
            header->data_cmd_id = 0x0104;
            header->receiver_ID = client_id;
            header->sender_ID = id;

            UiStruct *data;
            for (int i = 0; i < 7; i++)
            {
                data = (UiStruct *)(buffer + 13 + 15 * i);

                data->figure_name[0] = _name[0];
                data->figure_name[1] = _name[1];
                data->figure_name[2] = i + 1;
                data->operate_tpye = 1;
                data->figure_tpye = 0;
                data->layer = _layer;
                data->color = 3;
                data->width = 1;
                data->start_x = 0;
                data->start_y = 0;
                data->details_d = 0;
                data->details_e = 0;
            }

            auto *crc16_ptr = (uint16_t *)(buffer + 7 + *data_length_ptr);
            *crc16_ptr = CRC_GetCRC16CheckSum(buffer, 7 + *data_length_ptr, CRC16_INIT);
        }

        void getUpdateBuffer(uint8_t *buffer, uint16_t id, uint16_t client_id)
        {
            buffer[0] = SOF;

            auto *data_length_ptr = (uint16_t *)(buffer + 1);
            *data_length_ptr = 15 * 7 + 8;

            buffer[3] = 1;
            buffer[4] = CRC_GetCRC8CheckSum(buffer, 4, CRC8_INIT);

            buffer[5] = 0x01;
            buffer[6] = 0x03;

            auto *header = (UiHeader *)(buffer + 7);
            header->data_cmd_id = 0x0104;
            header->receiver_ID = client_id;
            header->sender_ID = id;

            UiStruct *data;
            for (int i = 0; i < 7; i++)
            {
                if (_shapeArray[i] == nullptr)
                {
                    break;
                }

                data = (UiStruct *)(buffer + 13 + 15 * i);

                data->figure_name[0] = _name[0];
                data->figure_name[1] = _name[1];
                data->figure_name[2] = i + 1;
                data->operate_tpye = 2;
                data->layer = _layer;

                _shapeArray[i]->drawTo(data);
            }

            auto *crc16_ptr = (uint16_t *)(buffer + 7 + *data_length_ptr);
            *crc16_ptr = CRC_GetCRC16CheckSum(buffer, 7 + *data_length_ptr, CRC16_INIT);
        };

        static void getDelBuffer(uint8_t *buffer, uint16_t id, uint16_t client_id)
        {
            buffer[0] = SOF;

            auto *data_length_ptr = (uint16_t *)(buffer + 1);
            *data_length_ptr = 2 + 8;

            buffer[3] = 1;
            buffer[4] = CRC_GetCRC8CheckSum(buffer, 4, CRC8_INIT);

            buffer[5] = 0x01;
            buffer[6] = 0x03;

            auto *header = (UiHeader *)(buffer + 7);
            header->data_cmd_id = 0x0100;
            header->receiver_ID = client_id;
            header->sender_ID = id;

            buffer[13] = 2;
            buffer[14] = 0;

            auto *crc16_ptr = (uint16_t *)(buffer + 7 + *data_length_ptr);
            *crc16_ptr = CRC_GetCRC16CheckSum(buffer, 7 + *data_length_ptr, CRC16_INIT);
        }

        ~Layer() = default;
    };

    class String : public Shape
    {
    private:
        uint8_t _name[3];
        uint8_t _layer;
        uint16_t _font_size;

    public:
        String(const char *name, uint8_t layer, Color color, uint16_t font_size, uint8_t width, XY first_point)
            : _name{name[0], name[1], name[2]}, _layer{layer}, _font_size{font_size}, Shape(color, width, first_point) {};

        void getInitBuffer(uint8_t *buffer, uint16_t id, uint16_t client_id, const char *str, uint16_t len)
        {
            memset(buffer, 0, 7 + 15 * 7 + 8);
            buffer[0] = SOF;

            auto *data_length_ptr = (uint16_t *)(buffer + 1);
            *data_length_ptr = 45 + 8;

            buffer[3] = 0;
            buffer[4] = CRC_GetCRC8CheckSum(buffer, 4, CRC8_INIT);

            buffer[5] = 0x01;
            buffer[6] = 0x03;

            auto *header = (UiHeader *)(buffer + 7);
            header->data_cmd_id = 0x0110;
            header->receiver_ID = client_id;
            header->sender_ID = id;

            UiStruct *data = (UiStruct *)(buffer + 13);
            data->figure_name[0] = _name[0];
            data->figure_name[1] = _name[1];
            data->figure_name[2] = _name[2];
            data->operate_tpye = 1;
            data->figure_tpye = 7;
            data->layer = _layer;
            data->color = 3;
            data->width = _width;
            data->start_x = 0;
            data->start_y = 0;
            data->details_a = _font_size;
            data->details_b = len;
            data->details_c = 0;
            data->details_d = 0;
            data->details_e = 0;

            auto *str_ptr = (uint8_t *)(buffer + 28);
            memcpy(str_ptr, str, len);

            auto *crc16_ptr = (uint16_t *)(buffer + 7 + *data_length_ptr);
            *crc16_ptr = CRC_GetCRC16CheckSum(buffer, 7 + *data_length_ptr, CRC16_INIT);
        }

        void getUpdateBuffer(uint8_t *buffer, uint16_t id, uint16_t client_id, const char *str, uint16_t len)
        {
            memset(buffer, 0, 7 + 15 * 7 + 8);
            buffer[0] = SOF;

            auto *data_length_ptr = (uint16_t *)(buffer + 1);
            *data_length_ptr = 45 + 8;

            buffer[3] = 0;
            buffer[4] = CRC_GetCRC8CheckSum(buffer, 4, CRC8_INIT);

            buffer[5] = 0x01;
            buffer[6] = 0x03;

            auto *header = (UiHeader *)(buffer + 7);
            header->data_cmd_id = 0x0110;
            header->receiver_ID = client_id;
            header->sender_ID = id;

            UiStruct *data = (UiStruct *)(buffer + 13);
            data->figure_name[0] = _name[0];
            data->figure_name[1] = _name[1];
            data->figure_name[2] = _name[2];
            data->operate_tpye = 2;
            data->figure_tpye = 7;
            data->layer = _layer;
            data->color = _color;
            data->width = _width;
            data->start_x = _first_dot.x;
            data->start_y = _first_dot.y;
            data->details_a = _font_size;
            data->details_b = len;
            data->details_c = 0;
            data->details_d = 0;
            data->details_e = 0;

            auto *str_ptr = (uint8_t *)(buffer + 28);
            memcpy(str_ptr, str, len);

            auto *crc16_ptr = (uint16_t *)(buffer + 7 + *data_length_ptr);
            *crc16_ptr = CRC_GetCRC16CheckSum(buffer, 7 + *data_length_ptr, CRC16_INIT);
        };

        ~String() = default;
    };
} // namespace UI

#define GET_STR(str) str, (sizeof(str) < 30 ? sizeof(str) : 30)
