#ifndef BUFFER_HPP_
#define BUFFER_HPP_

#define BUFF_SIZE 255


class Buffer
{
public:
	Buffer();

	void AddChar(unsigned char);
	void AddString(const char * str, unsigned int size);
	void ReadBuffer(char * str, unsigned int size_max);


private:

	unsigned char buf[BUFF_SIZE] {0};
	unsigned int index_in {0};
	unsigned int index_out {0};

	unsigned char ReadChar();
	unsigned int RemainingBytes();
};

#endif /* BUFFER_HPP_ */
