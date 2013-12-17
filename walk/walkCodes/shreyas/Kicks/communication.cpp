#include "communication.h"
#include <stdio.h>

const char Communication::ftdiID[] = "AD025KMG";//"AD025JOE";//"A800d2dg";//"AD025JOH";"A900fDp;//;//"A900fDpz";

int Communication::checksum(byte packet[])
{
	int i = 0;
	int chksum = 0xff;
	for(i=2;i<packet[3]+3;i++)
	chksum -= packet[i];
	return chksum;
}

int Communication::motorSend(int instruction, int dataLength, byte data[], int id)
{
	byte packet[dataLength+6];
	int i = 0;
	packet[0] = 0xff;
	packet[1] = 0xff;
	packet[2] = id;
	packet[3] = dataLength+2;
	packet[4] = instruction;
	
	for(i=0;i<dataLength;i++)
	packet[5+i]=data[i];
	
	packet[5+dataLength] = checksum(packet);
	
	if(ftdi_write_data(&bodyFTDI, packet, packet[3]+4)>0)
		return EXIT_SUCCESS;
	else 
		return EXIT_FAILURE;
}


int Communication::addSyncWrite(int location, int dataLength, byte data[], int id)
{
	if(syncPacketSize == 0)
	{
		//making (almost) complete packet except checksum
		syncPacket[0] = 0xff;
		syncPacket[1] = 0xff;
		syncPacket[2] = 0xfe;
		syncPacket[3] = 0;	//length not filled in yet, will be calculated when sending
		syncPacket[4] = 0x83;
		syncPacket[5] = location;
		syncPacket[6] = dataLength;
		syncPacket[7] = id;
		for (int i = 0; i < dataLength; ++i)
		{
			syncPacket[8 + i] = data[i];
		}
		syncPacketSize = 8 + dataLength;
	}
	else
	{
		if(syncPacket[5] != location)
		{
			syncPacketSize = 0;
			printf("SyncWrite conflict! Different location passed while stack not empty.\n");
			printf("Emptying stack.\n");
			return addSyncWrite(location, dataLength, data, id);
		}
		else
		{
			if(syncPacketSize > SYNCPACKET_MAXSIZE - (dataLength + 2))
			{
				printf("SyncWrite packet size is too large! Erasing previous stack.\n");
				syncPacketSize = 0;
				return addSyncWrite(location, dataLength, data, id);
			}
			else
			{
				//need to check here if same motor has been given some other packet,
				//if so, overwrite the previous position
				bool overwriteFlag = false;
				for (int i = 7; i < syncPacketSize; i = i + dataLength + 1)
				{
					if(syncPacket[i]==id)
					{
						overwriteFlag = true;
						for (int j = 0; j < dataLength; ++j)
						{
							syncPacket[i+j+1] = data[j];
						}
						break;
					}	
				}
				if(!overwriteFlag)
				{
					syncPacket[syncPacketSize] = id;
					for (int i = 0; i < dataLength; ++i)
					{
						syncPacket[syncPacketSize + 1 + i] = data[i];
					}
					syncPacketSize = syncPacketSize + dataLength + 1;
				}
			}
		}
	}
	return EXIT_SUCCESS;
}


int Communication::syncFlush()
{
	if(syncPacketSize == 0)
	{
		printf("Nothing to flush!\n");
		return EXIT_FAILURE;
	}
	else
	{
		#ifdef DEBUG
		for (int i = 0; i < syncPacketSize+1; ++i)
		{
		 	printf("%x\t", syncPacket[i]);
		}
		printf("\n\n\n\n");
		#endif
		
		syncPacket[3] = syncPacketSize - 3;
		syncPacket[syncPacketSize] = checksum(syncPacket);
		syncPacketSize = 0;	//Resetting size to 0 regardless of whether flush was successful or not
		if(ftdi_write_data(&bodyFTDI, syncPacket, syncPacket[3]+4)>0)
			return EXIT_SUCCESS;
		else 
			return EXIT_FAILURE;
	}
}


int Communication::motorReceive(byte data[] ,int tries)
{
	byte dataRec;
	int dataCount = 0;
	int length = 0;
	int packetLength = 0;
	int motorID = -1;
	int errorNo = 0;
	byte chksum = 0xff;
	bool pingFlag = false;
	while(tries)
	{
		// printf("val = %d\n", dataRec);
		if(ftdi_read_data(&bodyFTDI,&dataRec,1) > 0)
		{
			if(dataRec==0xff && dataCount<2)
			{
				dataCount++;
				//printf("dataRec==0xff && dataCount<2\n");
				continue;
			}
			else if (dataCount<2)
			{
				dataCount=0;
				//printf("dataCount<2\n");
				continue;
			}
			
			if(dataCount ==2)
			{
				dataCount++;
				motorID = dataRec;
				chksum -=dataRec;
				//printf("dataCount ==2\n");
				continue;
			}
			else if(dataCount ==4)
			{
				dataCount++;
				errorNo = dataRec;
				chksum-=dataRec;
				
				//printf("dataCount ==4\n");
				continue;	
			}
			else if(dataCount == 3)
			{
				dataCount++;
				length = dataRec;
				chksum -= dataRec;
				//printf("dataCount == 3\n");
				continue;
			}
			else if(dataCount>4 &&  packetLength<length-2)
			{
				dataCount++;
				chksum -= dataRec;
				data[packetLength] = dataRec;
				packetLength++;
				//printf("dataCount>4 &&  packetLength<length-2\n");
				continue;
			}
			else if(dataCount>4 && packetLength == (length-2) &&  dataRec== chksum)
				return EXIT_SUCCESS;
		}
		tries--;	
	}
	return EXIT_FAILURE;

}

int Communication::getBaudrate()
{
	return BAUD;
}

Communication::Communication()
{
	syncPacketSize = 0;
	if(ftdi_init(&bodyFTDI)<0)
	{
		fprintf(stderr, "ftdi_init failed.\n");
		return;
	}	
	if(ftdi_usb_open_desc(&bodyFTDI,0x403,0x6001,NULL, ftdiID)<0)
	{
		fprintf(stderr, "Unable to open ftdi device: (%s).\n", ftdi_get_error_string(&bodyFTDI));
      	printf("Unable to open ftdi.\n");
		return;	
	}
	else
	{
		if(ftdi_set_baudrate(&bodyFTDI, BAUD)<0)
		{
			printf("Unable to set baudrate.\n");
			return;
		}	
		printf("%s ftdi successfully opened.\n", ftdiID);
	}	
}

Communication::~Communication()
{
	printf("Communication through ftdi %s closing.\n", ftdiID);
}
