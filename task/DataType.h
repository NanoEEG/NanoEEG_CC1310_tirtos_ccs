#ifndef DATATYPE_H_
#define DATATYPE_H_


/*******************************************************************
 * TYPEDEFS
 */

/*!
    \brief  I2C传输数据结构
 */
#pragma pack(push)
#pragma pack(1)

typedef struct
{
    uint8_t     Index;          //!< 序号
    uint32_t    Ttor;           //!< 触发时间戳
    uint32_t    Tror;           //!< 接收时间戳
    uint32_t    Tsor;           //!< 同步时间戳
    uint8_t     Type;           //!< 标签类型
}I2CBuff_t;


#endif /* DATATYPE_H_ */
