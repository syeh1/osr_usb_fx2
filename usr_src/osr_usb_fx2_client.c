/******************************************************************************
 * Copyright @ 2015 Sinclair Yeh
 *
 * This program is created to test the bulk read/write function of the driver
 *****************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>


#define OSR_USB_FX2_DEV "/dev/osr_usb0"
#define MAX_PACKET_SIZE 64

/* OSR FX2 device is double buffered on the read and write, so there
 * are a total of 4 buffers.  Each write fills up one buffer and each
 * read drains one buffer.  If more than 4 are written, then the writes
 * will start failing.
 *
 * Change this value to > 4 and watch it fail
 */
#define TOTAL_PACKETS  4


static void print_usage(char *name)
{
	fprintf(stderr, "usage: %s <up to 64 bytes of characters\n",
		name);
	exit(1);
}



int main(int argc, char *argv[])
{
	int fd, i;
	int ret;
	char packet[MAX_PACKET_SIZE], tmp[MAX_PACKET_SIZE];
	char reply[MAX_PACKET_SIZE];

	if (argc < 2)
		print_usage(argv[0]);

	fd = open(OSR_USB_FX2_DEV, O_RDWR);

	if (-1 == fd)
	{
		fprintf(stderr, "Failed to open %s\n", OSR_USB_FX2_DEV);
		return 1;
	}


	/* Save two characters for packet number and one for NULL */
	memset(tmp, 0, MAX_PACKET_SIZE);
	strncpy(tmp, argv[1], MAX_PACKET_SIZE - 3);

	for (i = 0; i < TOTAL_PACKETS; i++)
	{
		sprintf(packet, "%s %i", tmp, i);

		ret = write(fd, packet, MAX_PACKET_SIZE);
		if (ret < 0)
		{
			fprintf(stderr,
				"Failed to write to device after %d packets\n",
				i);
			close(fd);
			return 1;
		}

		printf("Send: %s\n", packet);

		sleep(1);
	}

	for (i = 0; i < TOTAL_PACKETS; i++)
	{
		memset(reply, 0, MAX_PACKET_SIZE);
		ret = read(fd, reply, MAX_PACKET_SIZE);
		if (ret < 0)
		{
			fprintf(stderr, "Failed to read from device\n");
			close(fd);
			return 1;
		}

		printf("Reply: %s\n", reply);
		sleep(1);
	}

	/* Clean up */
	close(fd);

	return 0;
}

