#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <errno.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/un.h>
#include <sys/time.h>
#include <sys/ioctl.h>
#include <pthread.h>
#include <termios.h>

pthread_t m_thread;//ͨѶ�̱߳�ʶ��ID
int m_ExitThreadFlag;//�˳����ݽ����̱߳�־
int m_fd;
int m_DatLen;//�������ݽ��ճ���
char DatBuf[500];//�������ݽ���Buffer

typedef struct port_info {
	int baud_rate;
	int port_fd;
	char parity;
	char stop_bit;
	char flow_ctrl;
	char data_bits;
}*pport_info;

int get_baud_rate(unsigned long int baud_rate);
int set_port(pport_info p_info);


int OpenPort(int PortNo, int baudrate, char databits, char stopbits, char parity, int rt);
int ClosePort();//�رմ��ڲ��ͷ������Դ
int WritePort(const char* Buf, int len);//�򴮿�д����
void Flush();
int ReceiveThreadFunc(void* lparam);// �������ݽ����߳�

int main(int argc, char *argv[])
{
	int  	ret;
	int     portno,	baudRate;
	char    Buf[10];
	int i1;
	
	
	// ���������в��������ں�   ������
	if( argc > 1 )
	{
		portno = atoi( argv[1] );
	}
	else
	{
		portno = 2;
	}
	if( argc > 2 )
	{
		baudRate = atoi( argv[2]);
	}
	else
	{
		baudRate = 9600;
	}

	printf( "port:%d baudrate:%d\n", portno, baudRate);
	
	
	ret = OpenPort(portno, baudRate, '8', '1', 'N',0);
	if(ret<0 )
	{
		printf( "serial open fail\n");
		return -1;
	}

	Buf[0] = 0x31;
	Buf[1] = 0x32;
	Buf[2] = 0x33;
	Buf[3] = 0x34;
	Buf[4] = 0x35;
	Buf[5] = 0x36;
	Buf[6] = 0x37;
	Buf[7] = 0x38;
	Buf[8] = 0x39;


	//������ѭ��������ÿ��1s���һ����ʾ��Ϣ
	for( i1=0; i1<10000;i1++)
	{
		sleep(1);
		printf( "%d \n", i1+1);
		WritePort( Buf, 9 );
	}
	//ClosePort( );
	return 0;
}


int OpenPort(int PortNo, int baudrate, char databits, char stopbits, char parity, int rt)
{
	char				portname[20];
	struct port_info 	info;
	pthread_attr_t 		attr;
	int					res;

	sprintf( portname, "/dev/ttyS%d", PortNo );
	if((m_fd = open(portname,O_RDWR | O_NOCTTY | O_NONBLOCK))==-1)			//O_RDWR | O_NOCTTY | O_NDELAY |
	{
		perror("Cannot open the desired port");
		return -1;
	}

    //
    // Fill in the device control block.
    //
 	// �޸���ز���
	info.baud_rate = baudrate;
	info.parity = parity;
	info.data_bits = databits;
 	info.flow_ctrl = '0';
	info.port_fd = m_fd;
	info.stop_bit = stopbits;

	if(set_port(&info)==-1 )
	{
		printf( "set port fail\n");
		return -1;
	}


	// �������ڵĽ����߳�
	res = pthread_attr_init(&attr);
	if( res!=0 )
	{
		printf("Create attribute failed\n" );
	}
	// �����̰߳�����
	res = pthread_attr_setscope( &attr, PTHREAD_SCOPE_SYSTEM );
	// �����̷߳�������
	res += pthread_attr_setdetachstate( &attr, PTHREAD_CREATE_DETACHED );
	if( res!=0 )
	{
		printf( "Setting attribute failed\n" );
	}
	res = pthread_create( &m_thread, &attr, (void *(*) (void *))&ReceiveThreadFunc, NULL );
	if( res!=0 )
	{
		close( m_fd );
		return -1;
	}

	pthread_attr_destroy( &attr );
	return 0;
}

int ReceiveThreadFunc(void* lparam)
{
	//������¼�����
	fd_set fdRead;
	int ret;
	struct timeval	aTime;

	while( 1 )
	{
        //�յ��˳��¼��������߳�
		if(m_ExitThreadFlag )
		{
			break;
		}

		FD_ZERO(&fdRead);
		FD_SET(m_fd,&fdRead);

		aTime.tv_sec = 0;
		aTime.tv_usec = 300000;

		ret = select( m_fd+1,&fdRead,NULL,NULL,&aTime );
		printf( "select ret = %d\n", ret);

		if (ret < 0 )
		{
			//�رմ���
			ClosePort( );
			break;
		}

		if (ret > 0)
		{
			//�ж��Ƿ���¼�
			if (FD_ISSET(m_fd,&fdRead))
			{
				//data available, so get it!
				m_DatLen = read( m_fd, DatBuf, 100 );
				// �Խ��յ����ݽ��д���
				if( m_DatLen > 0 )
				{
					printf("%s", DatBuf);
					
				}
				// �������
			}
		}
	}

	printf( "ReceiveThreadFunc finished\n");
	pthread_exit( NULL );
	return 0;
}


int get_baud_rate(unsigned long int baud_rate)
{
	switch (baud_rate) {
	case 0:
		return B0;
	case 50:
		return B50;
	case 75:
		return B75;
	case 110:
		return B110;
	case 134:
		return B134;
	case 150:
		return B150;
	case 200:
		return B200;
	case 300:
		return B300;
	case 600:
		return B600;
	case 1200:
		return B1200;
	case 1800:
		return B1800;
	case 2400:
		return B2400;
	case 4800:
		return B4800;
	case 9600:
		return B9600;
	case 19200:
		return B19200;
	case 38400:
		return B38400;
	case 57600:
		return B57600;
	case 115200:
		return B115200;
	case 230400:
		return B230400;
	default:
		return -1;
	}
}

int set_port(pport_info p_info)
{
	struct termios new_opt;
	int baud_rate;
	int status;

	//get the current config -> new_opt
	tcgetattr(p_info->port_fd, &new_opt);
	bzero(&new_opt, sizeof(new_opt));

	//convert baud rate -> baud_flag
	baud_rate = get_baud_rate(p_info->baud_rate);

	tcflush(p_info->port_fd, TCIOFLUSH);
	//setup input/output baudrate
	cfsetispeed(&new_opt, baud_rate);
	cfsetospeed(&new_opt, baud_rate);
	status = tcsetattr(p_info->port_fd, TCSANOW, &new_opt);
	if (status != 0)
	{
		perror("tcsetattr::set baud rate failed\n");
		return -1;
	}

	//�޸Ŀ���ģʽ����֤���򲻻�ռ�ô���
	new_opt.c_cflag |= CLOCAL;

	//�޸Ŀ���ģʽ��ʹ���ܹ��Ӵ��ڶ�ȡ��������
	new_opt.c_cflag |= CREAD;

	new_opt.c_cflag |= HUPCL;
	//setup control flow
	switch (p_info->flow_ctrl)
	{
	case '0':
		//no control-flow
		new_opt.c_cflag &= ~CRTSCTS;
		break;
	case '1':
		//hardware control-flow
		new_opt.c_cflag |= CRTSCTS;
		break;
	case '2':
		new_opt.c_iflag |= IXON | IXOFF | IXANY;
		break;
	}
	//printf("c_cflag(no ctl-flow) = %x\r\n", new_opt.c_cflag);

	//setup bit size
	new_opt.c_cflag &= ~CSIZE;
	switch (p_info->data_bits)
	{
	case '5':
		new_opt.c_cflag |= CS5;
		break;
	case '6':
		new_opt.c_cflag |= CS6;
		break;
	case '7':
		new_opt.c_cflag |= CS7;
		break;
	case '8':
		new_opt.c_cflag |= CS8;
		break;
	default:
		new_opt.c_cflag |= CS8;
	}
	//printf("c_cflag |= CS8 => %x\r\n", new_opt.c_cflag);

	//setup parity
	switch (p_info->parity)
	{
	case 'n':
	case 'N':
		new_opt.c_cflag &= ~PARENB; /* Clear parity enable */
		new_opt.c_iflag &= ~INPCK; /* Enable parity checking */
		break;
	case 'o':
	case 'O':
		new_opt.c_cflag |= (PARODD | PARENB); /* ����Ϊ��Ч��*/
		new_opt.c_iflag |= INPCK; /* Disable parity checking */
		break;
	case 'e':
	case 'E':
		new_opt.c_cflag |= PARENB; /* Enable parity */
		new_opt.c_cflag &= ~PARODD; /* ת��ΪżЧ��*/
		new_opt.c_iflag |= INPCK; /* Disable parity checking */
		break;
	case 'S':
	case 's': /*as no parity*/
		new_opt.c_cflag &= ~PARENB;
		new_opt.c_cflag &= ~CSTOPB;
		break;
	default:
		fprintf(stderr, "Unsupported parity\n");
		return -1;
	}
	//printf("c_cflag &=~PARENB => %x\r\n", new_opt.c_cflag);

	//setup stop-bit
	if (p_info->stop_bit == '2') {
		new_opt.c_cflag |= CSTOPB;
	} else {
		new_opt.c_cflag &= ~CSTOPB;
	}
	//printf("c_cflag &=~CSTOPB => %x\r\n", new_opt.c_cflag);

	/* Set input parity option */
	if ((p_info->parity != 'n') || (p_info->parity != 'N'))
	{
		new_opt.c_iflag |= INPCK;
	}

	//�޸����ģʽ��ԭʼ�������(raw ģʽ)
	new_opt.c_lflag &= ~(ICANON | ECHO | ISIG); /*Input*/
	new_opt.c_oflag &= ~OPOST; /*Output*/

	//�޸Ŀ����ַ�����ȡ�ַ������ٸ���Ϊ1
	new_opt.c_cc[VMIN] = 1;

	//�޸Ŀ����ַ�����ȡ��һ���ַ��ĳ�ʱʱ��Ϊ1��100ms
	new_opt.c_cc[VTIME] = 1;

	//��ͼȥ���ڽ���ʱ�����յ�'\n'�ŷ��ص�����
	//��������Ļس�
	//new_opt.c_iflag |= IGNCR;
	//new_opt.c_iflag &= ~(IXON|IXOFF|IXANY);

	//�����������������������ݣ����ǲ��ٶ�ȡ
	tcflush(p_info->port_fd, TCIFLUSH);

	status = tcsetattr(p_info->port_fd, TCSANOW, &new_opt);
	if (status != 0)
	{
		perror("Cannot set the serial port parameters");
		return -1;
	}

	return status;
}

int WritePort(const char *buf, int len)
{
	if (m_fd == 0) return -1;
	int sendlen = 0;
	sendlen = write(m_fd, buf, len);

	if (sendlen == len)
	{
		return sendlen;
	}
	else
	{
		//�������������
		tcflush(m_fd, TCOFLUSH);
		return -1;
	}
}

void Flush()
{
	tcflush(m_fd, TCIFLUSH);
}

int ClosePort()
{
	m_ExitThreadFlag = 1;
	close(m_fd);
	printf("close serial port\n");
	return 0;
}