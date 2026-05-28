#include <xtoffboard.hpp>
#include <xtserial.hpp>

bool XTserial::start(const std::string &serial, int buadrate)
{
    if(!open_uart(serial,buadrate))
        return false;
    _running = true;
    _thread = std::thread(&XTserial::serial_loop,this);
    return true;
}

void XTserial::stop()
{
    _running = false;
    if (_thread.joinable()) {
        _thread.join();
    }
}

bool XTserial::get_data(lora_struct& lora_s)
{
    std::lock_guard<std::mutex> lock(_mutex);
    if(_queue.empty())
        return false;

    // 返回队列中存储的最旧值并删除
    lora_s = _queue.front();
    _queue.pop();
    return true;
}

void XTserial::serial_loop()
{
    uint8_t buffer[128];

    while(_running)
    {
        struct pollfd fds{};
        fds.fd = _fd;
        fds.events = POLLIN;
        
        int ret = poll(&fds, 1, 100);
        if (ret > 0 && (fds.revents & POLLIN))
        {
            int n = read(_fd,buffer,sizeof(buffer));
            if(n > 0)
                read_data(buffer,n);
            if(_rec_full)
            {
                // 将读取的数据放入队列中
                _rec_full = false;
                std::lock_guard<std::mutex> lock(_mutex);
                if (_queue.size() > 100) 
                    _queue.pop();  // 防止堆积太多
                _queue.push(_rec_struct);
            }
        }
    }
}

bool XTserial::open_uart(const std::string &serial, int buadrate)
{
    _fd = open(serial.c_str(),O_RDWR | O_NOCTTY | O_NONBLOCK);
    if(_fd < 0)
    {
        return false;
    }

    struct termios tty;

    tcgetattr(_fd, &tty);
    cfmakeraw(&tty);

    // 设置波特率
    switch (buadrate)
    {
    case 9600:
        cfsetispeed(&tty, B9600);
        cfsetospeed(&tty, B9600);
        break;
    case 57600:
        cfsetispeed(&tty, B57600);
        cfsetospeed(&tty, B57600);
        break;
    case 115200:
        cfsetispeed(&tty, B115200);
        cfsetospeed(&tty, B115200);
        break;
    case 921600:
        cfsetispeed(&tty, B921600);
        cfsetospeed(&tty, B921600);
        break;
    default:
        break;
    }

    //8N1
	tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
   	tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_oflag &= ~OPOST;

    tcsetattr(_fd, TCSANOW, &tty);

    return true;
}

void XTserial::read_data(uint8_t *data, int len)
{
    if(len !=14 )
        return;
    
    uint16_t cal_crc;
    for(int i=0;i<len;i++)
	{
		uint8_t byte = data[i];
		switch (_parse_state)
		{
		case WAIT_HEAD1:
			if(byte == 0xAA)
				_parse_state = WAIT_HEAD2;
			break;
		case WAIT_HEAD2:
			if(byte == 0x55) 
				_parse_state = WAIT_LENGTH;
			else
				_parse_state = WAIT_HEAD1;
			break;
        	case WAIT_LENGTH:
            		_length = byte;
            		_index = 0;

            		if (_length == 0 || _length > sizeof(_payload))
                		_parse_state = WAIT_HEAD1;
            		else
                		_parse_state = WAIT_PAYLOAD;
            		break;
		case WAIT_PAYLOAD:
			_payload[_index++] = byte;
			if(_index >= _length)
				_parse_state = WAIT_CRC1;
			break;
		case WAIT_CRC1:
			_recv_crc = byte;
			_parse_state = WAIT_CRC2;
			break;
		case WAIT_CRC2:
			_recv_crc |= ((uint16_t)byte << 8);
			//收到完整一帧，进行校验
			cal_crc = crc_ccitt(_payload,_length);
			if(cal_crc == _recv_crc)
			{
				//通过校验
				_rec_struct.node_id = _payload[0];
				_rec_struct.lat = (int32_t)_payload[1]
						|((int32_t)_payload[2] << 8)
						|((int32_t)_payload[3] << 16)
						|((int32_t)_payload[4] << 24);
				_rec_struct.lon = (int32_t)_payload[5]
						|((int32_t)_payload[6] << 8)
						|((int32_t)_payload[7] << 16)
						|((int32_t)_payload[8] << 24);
                _rec_full = true;
			}
			_parse_state = WAIT_HEAD1;
			break;
		}
	}
}

uint16_t XTserial::crc_ccitt(const uint8_t *data, uint8_t len)
{
	uint16_t crc = 0xFFFF;
	for(int i=0;i<len;i++)
	{
		crc ^= (uint16_t)data[i] << 8;
		for(int j=0;j<8;j++)
		{
			if(crc & 0x8000)
				crc = (crc << 1) ^ 0x1021;
			else
				crc <<= 1;
		}
	}
	return crc;
}
