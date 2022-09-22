
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>

#define ADXL34X_IOC_MAGIC    'a'
#define ADXL34X_IOCINIT      _IOWR(ADXL34X_IOC_MAGIC, 0, void *)
#define ADXL34X_IOCEXIT      _IOWR(ADXL34X_IOC_MAGIC, 1, void *)


struct adxl34x_info_msg {
    int max_speed_hz;
    unsigned short mode;
    unsigned short chip_select;
    int irq_pin;
};


int main(int argc, char **argv)
{
	int fd;
	int ret;
	unsigned int status;
	
	struct adxl34x_info_msg ad_info_msg = {
		.max_speed_hz = 5000000,
		.mode         = 3,
		.chip_select  = 0,
		.irq_pin	  = 116,
	};
	
	if(argc != 3)
	{
		printf("Usage:");
		printf("%s /dev/adxl345 0/1\n",argv[0]);
	}

	fd = open(argv[1],O_RDWR);
	if(fd == -1)
    {
		printf("can not open file %s\n", argv[1]);
		return -1;
    }

	if(argv[2][0] == '1')
	{
		ret = ioctl(fd, ADXL34X_IOCINIT, &ad_info_msg);
		
	}
	if(argv[2][0] == '0')
	{
		ret = ioctl(fd, ADXL34X_IOCEXIT, &ad_info_msg);
	}

	close(fd);
	return 0;
}	





