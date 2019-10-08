#define _GNU_SOURCE
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>

#define PORT     55560
#define MAXLINE  2000

#define DATE_LENGTH 60
#define FASTPACKET_SIZE (1)
#define FASTPACKET_BUCKET_0_SIZE (6)
#define FASTPACKET_BUCKET_N_SIZE (7)
#define FASTPACKET_MAX_INDEX (0x1f)
#define FASTPACKET_MAX_SIZE (FASTPACKET_BUCKET_0_SIZE + FASTPACKET_BUCKET_N_SIZE * (FASTPACKET_MAX_INDEX - 1))

#define SRC      4

#define GLOBALS

typedef struct
{
  char     timestamp[DATE_LENGTH];
  uint8_t  prio;
  uint32_t pgn;
  uint8_t  dst;
  uint8_t  src;
  uint8_t  len;
  uint8_t  data[FASTPACKET_MAX_SIZE];
} RawMessage;

static int openPortUDP(int *sockfd, int port);
static int  openCanDevice(char *device, int *socketc);
static void writeRawPGNToCanSocket(RawMessage *msg, int socketc);
static void sendCanFrame(struct can_frame *frame, int socketc);
static void sendN2kFastPacket(RawMessage *msg, struct can_frame *frame, int socketc);
unsigned int getCanIdFromISO11783Bits(unsigned int prio, unsigned int pgn, unsigned int src, unsigned int dst);

int main(int argc, char **argv)
{
  char           msg[2000];
  int            socketc, sockfd;


  if (argc != 2)
  {
    puts("Usage: socketcan-writer <can-device>");
    exit(1);
  }

  if (openCanDevice(argv[1], &socketc))
  {
    exit(1);
  }


  if(openPortUDP(&sockfd,PORT))
  {
    exit(1);
  }

  struct sockaddr_in cliaddr;
   memset(&cliaddr, 0, sizeof(cliaddr));
  int len, n;

  while (1) {
        n = recvfrom(sockfd, (char *)msg, MAXLINE,
                MSG_WAITALL, ( struct sockaddr *) &cliaddr,&len);
        msg[n] = '\0';

   RawMessage m;
    m.prio=msg[2];
    m.pgn=(msg[5]<<16)+(msg[4]<<8)+(msg[3]);
    m.src=SRC;
    m.dst=msg[6];
    m.len=msg[7];
    memcpy(m.data, msg+8, m.len);
  writeRawPGNToCanSocket(&m, socketc);
  }
  close(sockfd);
  close(socketc);
  exit(0);
}

static int openPortUDP(int *sockfd, int port)
{
  struct sockaddr_in servaddr;

          // Creating socket file descriptor
                if ( (*sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0 ) {
                        perror("socket creation failed");
                        //exit(EXIT_FAILURE);
			return 1;
                }

                memset(&servaddr, 0, sizeof(servaddr));

        // Filling server information
                servaddr.sin_family = AF_INET; // IPv4
                servaddr.sin_addr.s_addr = INADDR_ANY;
                servaddr.sin_port = htons(port);
        // Bind the socket with the server address
                if ( bind(*sockfd, (const struct sockaddr *)&servaddr,
                 sizeof(servaddr)) < 0 )
                {
                        perror("bind failed");
                        //exit(EXIT_FAILURE);
			return 1;
                }

 return 0;
}

/*
  Opens SocketCAN socket to given device, see: https://www.kernel.org/doc/Documentation/networking/can.txt
*/
static int openCanDevice(char *device, int *canSocket)
{
  struct sockaddr_can addr;
  struct ifreq        ifr;

  if ((*canSocket = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0)
  {
    perror("socket");
    return 1;
  }

  strncpy(ifr.ifr_name, device, IFNAMSIZ - 1);
  ifr.ifr_name[IFNAMSIZ - 1] = '\0';
  ifr.ifr_ifindex            = if_nametoindex(ifr.ifr_name);
  if (!ifr.ifr_ifindex)
  {
    perror("if_nametoindex");
    return 1;
  }

  addr.can_family  = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  if (bind(*canSocket, (struct sockaddr *) &addr, sizeof(addr)) < 0)
  {
    perror("bind");
    return 1;
  }

  return 0;
}

static void writeRawPGNToCanSocket(RawMessage *msg, int socket)
{
  struct can_frame frame;
  memset(&frame, 0, sizeof(frame));

  if (msg->pgn >= (1 << 18)) // PGNs can't have more than 18 bits, otherwise it overwrites priority bits
  {
    //logError("Invalid PGN, too big (0x%x). Skipping.\n", msg->pgn);
    return;
  }

  frame.can_id = getCanIdFromISO11783Bits(msg->prio, msg->pgn, msg->src, msg->dst);

  if (msg->len <= 8) // 8 or less bytes of data -> PGN fits into a single CAN frame
  {
    frame.can_dlc = msg->len;
    memcpy(frame.data, msg->data, msg->len);
    sendCanFrame(&frame, socket);
  }
  else
  { // Send PGN as n2k fast packet (spans multiple CAN frames, but CAN ID is still same for each frame)
    sendN2kFastPacket(msg, &frame, socket);
  }
}

static void sendCanFrame(struct can_frame *frame, int socket)
{
  if (write(socket, frame, sizeof(struct can_frame)) != sizeof(struct can_frame))
  {
    perror("write");
  }
}

/*
  See pgn.h for n2k fast packet data format
*/
static void sendN2kFastPacket(RawMessage *msg, struct can_frame *frame, int socket)
{
  int index              = 0;
  int remainingDataBytes = msg->len;
  while (remainingDataBytes > 0)
  {
    frame->data[0]
        = index; // fast packet index (increases by 1 for every CAN frame), 'order' (the 3 uppermost bits) is left as 0 for now

    if (index == 0) // 1st frame
    {
      frame->data[1] = msg->len;             // fast packet payload size
      memcpy(frame->data + 2, msg->data, 6); // 6 first data bytes
      frame->can_dlc = 8;
      remainingDataBytes -= 6;
    }
    else // further frames
    {
      if (remainingDataBytes > 7)
      {
        memcpy(frame->data + 1, msg->data + 6 + (index - 1) * 7, 7); // 7 next data bytes
        frame->can_dlc = 8;
        remainingDataBytes -= 7;
      }
      else
      {
        memcpy(frame->data + 1, msg->data + 6 + (index - 1) * 7, remainingDataBytes); // 7 next data bytes
        frame->can_dlc     = 1 + remainingDataBytes;
        remainingDataBytes = 0;
      }
    }
    sendCanFrame(frame, socket);
    index++;
  }
}

/*
  This does the opposite from getISO11783BitsFromCanId: given n2k fields produces the extended frame CAN id
*/
unsigned int getCanIdFromISO11783Bits(unsigned int prio, unsigned int pgn, unsigned int src, unsigned int dst)
{
  unsigned int canId = src | 0x80000000U; // src bits are the lowest ones of the CAN ID. Also set the highest bit to 1 as n2k uses
                                          // only extended frames (EFF bit).

  if ((unsigned char) pgn == 0)
  { // PDU 1 (assumed if 8 lowest bits of the PGN are 0)
    canId += dst << 8;
    canId += pgn << 8;
    canId += prio << 26;
  }
  else
  { // PDU 2
    canId += pgn << 8;
    canId += prio << 26;
  }

  return canId;
}
