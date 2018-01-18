#include <stdio.h>
#include <sys/termios.h>
#include <fcntl.h> // O_RDWR | O_NOCTTY | O_NDELAY
#include <unistd.h>
#include <string.h>
#include <stdint.h>

#define SCAN_BUF_SIZE (size_t) 41

static int setInterfaceAttributes (int fd, int speed, int parity)
{
    struct termios tty;
    memset (&tty, 0, sizeof tty);
    if (tcgetattr (fd, &tty) != 0)
    {
        //perror("tcgetattr");
        perror("tggetattr");
        return 0;
    }
    
    cfsetospeed (&tty, speed);
    cfsetispeed (&tty, speed);
    
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
    // disable IGNBRK for mismatched speed tests; otherwise receive break
    // as \000 chars
    tty.c_iflag &= ~IGNBRK;         // disable break processing
    tty.c_lflag = 0;                // no signaling chars, no echo,
    // no canonical processing
    tty.c_oflag = 0;                // no remapping, no delays
    tty.c_cc[VMIN]  = 0;            // read doesn't block
    tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout
    
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl
    
    tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
    // enable reading
    tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
    tty.c_cflag |= parity;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;
    
    if (tcsetattr (fd, TCSANOW, &tty) != 0)
    {
        perror("tcsetattr");
        
        return 0;
    }
    return 1;
}

int openSerialPort( const char* portName)
{
    if( portName ==  NULL)
        return -1;
    
    int fd =  open(portName, O_RDWR | O_NOCTTY | O_NDELAY);
    
    if (fd <=0 )
    {
        return -1;
    }

    if( setInterfaceAttributes (fd, B230400, 0) == 0)
    {
        printf("Error set_interface_attribs\n");
    }
    
    return fd;   
}



int main(int argc, const char * argv[])
{
    const char* portName = argc > 1? argv[1] : "/dev/ttyUSB0";
    
    printf("try open '%s' \n" ,portName);


    int fd = openSerialPort(portName);
    
    if( fd < 0 )
    {
        printf("Error while opening port '%s'\n" , portName);
        return 1;
    }

    printf("FD num is %i\n", fd);

    const size_t bufferScanSize = SCAN_BUF_SIZE;

    const char wA = 'b';
    write(fd , &wA , 1);

    char bufferScan[ SCAN_BUF_SIZE ] = {0};
    while(1)
    {
	uint8_t startByte = 0;
        ssize_t ret = read(fd, &startByte, 1);
        
        if( ret == 0)
        {
            continue;
        }

	if( startByte == 0xFA)
        {
		
		ret = read(fd , bufferScan , bufferScanSize);
		if( ret == bufferScanSize)
		{

			if( bufferScan[0] >= 0xA0  && bufferScan[0] <= 0xDB) // TODO: checksum
      			{
				int degree_count_num = 0;
			        int index = (bufferScan[0] - 0xA0) * 6;

				if( index == 0)
				{
//					printf("Got Scan index %i\n" , index);

					for(uint16_t j = 4; j < 40; j = j + 6)
        				{
          					uint8_t byte0 = bufferScan[j-1];
          					uint8_t byte1 = bufferScan[j+1-1];
          					uint8_t byte2 = bufferScan[j+2-1];
          					uint8_t byte3 = bufferScan[j+3-1];

					        uint16_t intensity = (byte1 << 8) + byte0;
				 	        uint16_t range     = (byte3 << 8) + byte2;

						float fRange = range / 1000.f;
						float fIntensity = intensity;
						
						if( degree_count_num == 0)
						printf("Got Scan index %i deg %i range %f | %f\n" , index , degree_count_num , fRange , fIntensity);

						degree_count_num++;
					}
				}
			}
		}
		else 
		{
			printf("Error getting scan %zi \n",ret);
		}
        }
    }




    close(fd);
    return 0;
}
