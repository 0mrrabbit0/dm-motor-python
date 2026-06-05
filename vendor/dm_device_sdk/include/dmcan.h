#ifndef DM_DEVICE_PUB_USER_H
#define DM_DEVICE_PUB_USER_H

#define DEVICE_EXPORTS

#ifdef _WIN32
    #ifdef DEVICE_EXPORTS
        #define DEVICE_API  __declspec(dllexport)
    #else
        #define DEVICE_API   __declspec(dllimport)
    #endif
#else
    #define DEVICE_API  __attribute__((visibility("default")))
#endif


#ifdef __cplusplus

#include <stdint.h>
#include <stdbool.h>
#include <cstddef>


extern "C"
{

struct dmcan_context;
struct dmcan_device_handle;

#pragma pack(push,1)

    typedef struct
    {
        uint32_t  can_id:29;         //can id
        uint32_t   esi:1;            //错误状态指示 一般不用
        uint32_t   ext:1;            //类型：标准/拓展
        uint32_t   rtr:1;            //类型：数据/远程
        uint64_t  time_stamp;       //时间戳
        uint8_t   channel;          //发送通道
        uint8_t   canfd:1;          //类型：2.0/fd
        uint8_t   dir:1;            //方向：rx/tx
        uint8_t   brs:1;            //BRS
        uint8_t   ack:1;            //应答标志
        uint8_t   dlc:4;            //长度
        uint16_t  reserved;         //预留字节
    }usb_rx_frame_head_t;

    typedef struct
    {
        usb_rx_frame_head_t head;
        uint8_t payload[64];

    }usb_rx_frame_t ;

    typedef struct
    {
        uint8_t channel;
        bool canfd;
        uint32_t can_baudrate;
        uint32_t canfd_baudrate;
        float can_sp;
        float canfd_sp;
    }dmcan_channel_can_info_t;

    typedef enum
    {
        USB2CANFD=0,
        USB2CANFD_DUAL,
        LINKX4C
    }dmcan_device_type_t;

    typedef struct
    {
        uint8_t channel;
        uint8_t can_fd;
        uint8_t can_seg1;
        uint8_t can_seg2;
        uint8_t can_sjw;
        uint8_t can_prescaler;
        uint8_t canfd_seg1;
        uint8_t canfd_seg2;
        uint8_t canfd_sjw;
        uint8_t canfd_prescaler;
    }dmcan_ch_can_config_t;


#pragma pack(pop)


    typedef void (*dev_recv_callback)(dmcan_device_handle* handle,usb_rx_frame_t* rec_frame);
    typedef void (*dev_sent_callback)(dmcan_device_handle* handle,usb_rx_frame_t* sent_frame);
    typedef void (*dev_err_callback)(dmcan_device_handle* handle,usb_rx_frame_t* err_frame);


    DEVICE_API void dmcan_context_create(struct dmcan_context** ctx);
    DEVICE_API void dmcan_context_destroy(struct dmcan_context* ctx);
    DEVICE_API void dmcan_print_version(struct dmcan_context* ctx);
    DEVICE_API void dmcan_get_sdk_version(struct dmcan_context* ctx, uint32_t* version);
    DEVICE_API int dmcan_find_devices(struct dmcan_context* ctx);
    DEVICE_API int dmcan_find_devices_with_type(struct dmcan_context* ctx, dmcan_device_type_t type);
    DEVICE_API void dmcan_show_all_devices(struct dmcan_context* ctx);


    DEVICE_API bool dmcan_device_get(struct dmcan_context* ctx, struct dmcan_device_handle** dev_handle,int index);
    DEVICE_API bool dmcan_device_open(struct dmcan_device_handle* dev_handle);
    DEVICE_API void dmcan_device_close(struct dmcan_device_handle* dev_handle);
    DEVICE_API void dmcan_device_get_version(struct dmcan_device_handle* dev_handle, char* version_buf, size_t buf_size);
    DEVICE_API void dmcan_device_print_version(struct dmcan_device_handle* dev_handle);
    DEVICE_API bool dmcan_device_enable_channel(struct dmcan_device_handle* dev_handle,uint8_t channel);
    DEVICE_API bool dmcan_device_disable_channel(struct dmcan_device_handle* dev_handle,uint8_t channel);

    DEVICE_API bool dmcan_device_get_channel_baudrate(struct dmcan_device_handle* dev_handle, uint8_t channel, dmcan_channel_can_info_t* baud_info);
    DEVICE_API bool dmcan_device_get_channel_baudrate_details(struct dmcan_device_handle* dev_handle, uint8_t channel, dmcan_ch_can_config_t* config);
    DEVICE_API bool dmcan_device_set_channel_baudrate(struct dmcan_device_handle* dev_handle, uint8_t channel, dmcan_channel_can_info_t baud_info);
    DEVICE_API bool dmcan_device_set_channel_baudrate_details(struct dmcan_device_handle* dev_handle, uint8_t channel, dmcan_ch_can_config_t config);

    DEVICE_API void dmcan_device_hook_recv_callback(struct dmcan_device_handle* dev_handle, dev_recv_callback callback);
    DEVICE_API void dmcan_device_hook_sent_callback(struct dmcan_device_handle* dev_handle, dev_sent_callback callback);
    DEVICE_API void dmcan_device_hook_err_callback(struct dmcan_device_handle* dev_handle, dev_err_callback callback);


    DEVICE_API bool dmcan_device_send_can(struct dmcan_device_handle* dev_handle,uint8_t ch,uint32_t can_id,bool canfd,bool ext,bool rtr,bool brs,uint8_t dlen,uint8_t* payload);
    DEVICE_API bool dmcan_device_send_can_details(struct dmcan_device_handle* dev_handle,uint8_t ch,uint32_t interval_ms,uint16_t step_id,uint32_t stop_id,int send_times,uint32_t can_id,bool canfd,bool ext,bool rtr,bool brs,bool id_inc,bool data_inc,uint8_t dlen,uint8_t* payload);

    DEVICE_API bool dmcan_device_fill_can_queue(struct dmcan_device_handle* dev_handle,uint8_t ch,uint32_t can_id,bool canfd,bool ext,bool rtr,bool brs,uint8_t dlen,uint8_t* payload);
    DEVICE_API bool dmcan_device_can_queue_send(struct dmcan_device_handle* dev_handle);


    DEVICE_API int dmcan_utils_get_dlc_from_len(int dlen);
    DEVICE_API int dmcan_utils_get_len_from_dlc(int dlc);
}


#endif

#endif //DM_DEVICE_PUB_USER_H