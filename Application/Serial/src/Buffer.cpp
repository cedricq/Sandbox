#include "Buffer.hpp"
#include "BufferC.hpp"

static Buffer BufferSerialInstance;


void serial_add_char(unsigned char c)
{
	BufferSerialInstance.AddChar(c);
}


Buffer::Buffer()
{}

void Buffer::AddChar(unsigned char c)
{
	buf[index_in] = c;
	index_in++;
	index_in = index_in % BUFF_SIZE;
}


void Buffer::AddString(const char * str, unsigned int size)
{
	for (unsigned int i = 0; i < size; i++)
	{
		AddChar(str[i]);
	}
}


unsigned char Buffer::ReadChar()
{
	char c = '\0';
	if (index_in != index_out)
	{
		c = buf[index_out];
		index_out++;
		index_out = index_out % BUFF_SIZE;
	}
	return c;
}


unsigned int Buffer::RemainingBytes()
{
	return (BUFF_SIZE + index_in - index_out) % BUFF_SIZE;
}


void Buffer::ReadBuffer(char * str, unsigned int size_max)
{
	char c = 0;
	unsigned int i = 0;
	while (c != '\0' && c != '\r' && c != '\n' && RemainingBytes() != 0 && i < size_max)
	{
		c = ReadChar();
		str[i] = c;
		i++;
	}
}

