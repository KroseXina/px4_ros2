#pragma once

#include <termios.h>
#include <poll.h>
#include <fcntl.h>
#include <thread>
#include <queue>
#include <mutex>

class XTserial {
public:
    /* 与lora模块的通信协议定义(uint8_t)：0xAA 0x55 length payload CRC_L CRC_H*/
	/* lora传输的payload结构体,长度为1+4+4=9字节，整帧长度为14字节 */
	struct lora_struct
	{
		uint8_t node_id;
	   	int32_t lat; //[degE7] Latitude
	   	int32_t lon; //[degE7] Longitude
	};


    bool start(const std::string &serial, int buadrate);
    void stop();

    bool get_data(lora_struct& lora_s);

private:

    int _fd{-1};

    std::thread _thread;
    bool _running{false};
    std::mutex _mutex;
    std::queue<lora_struct> _queue;

	/* 定义状态机进行拼包 */
	enum parse_state
	{
		WAIT_HEAD1,
		WAIT_HEAD2,
		WAIT_LENGTH,
		WAIT_PAYLOAD,
		WAIT_CRC1,
		WAIT_CRC2
	};

	/* 用以接收串口数据 */
	parse_state _parse_state {WAIT_HEAD1};
	uint8_t _length{0};
	uint8_t _payload[64];
	uint8_t _index{0};
	uint16_t _recv_crc;

    lora_struct _rec_struct;
    bool _rec_full{false};

    uint16_t crc_ccitt(const uint8_t *data, uint8_t len);
    bool open_uart(const std::string &serial, int buadrate);
    void read_data(uint8_t *data, int len);
    void serial_loop();
};