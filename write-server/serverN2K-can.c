/*

Reads raw n2k ASCII data from stdin and writes it to a Linux SocketCAN device (e.g. can0).

(C) 2009-2015, Kees Verruijt, Harlingen, The Netherlands.

This file is part of CANboat.

CANboat is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

CANboat is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with CANboat.  If not, see <http://www.gnu.org/licenses/>.

*/
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
#define SRC      4

#define GLOBALS
#include "common.h"
#include "pgn.h"

static int  openCanDevice(char *device, int *socketc);
static void writeRawPGNToCanSocket(RawMessage *msg, int socketc);
static void sendCanFrame(struct can_frame *frame, int socketc);
static void sendN2kFastPacket(RawMessage *msg, struct can_frame *frame, int socketc);
unsigned long time_diff(struct timeval x , struct timeval y, char *timestamp);

int main(int argc, char **argv)
{
  FILE *file = stdin;
  char           msg[2000];
  char *         milliSecond;
  int            socketc, sockfd;
  struct timeval frameTime;
  struct timeval prevFrameTime;
  struct tm      ctime;
  unsigned long  usWait;

  struct sockaddr_in servaddr , cliaddr;

  setProgName(argv[0]);
  if (argc != 2)
  {
    puts("Usage: socketcan-writer <can-device>");
    exit(1);
  }

  if (openCanDevice(argv[1], &socketc))
  {
    exit(1);
  }

          // Creating socket file descriptor
                if ( (sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0 ) {
                        perror("socket creation failed");
                        exit(EXIT_FAILURE);
                }

                memset(&servaddr, 0, sizeof(servaddr));
                memset(&cliaddr, 0, sizeof(cliaddr));

        // Filling server information
                servaddr.sin_family = AF_INET; // IPv4
                servaddr.sin_addr.s_addr = INADDR_ANY;
                servaddr.sin_port = htons(PORT);
        // Bind the socket with the server address
                if ( bind(sockfd, (const struct sockaddr *)&servaddr,
                 sizeof(servaddr)) < 0 )
                {
                        perror("bind failed");
                        exit(EXIT_FAILURE);
                }


  prevFrameTime.tv_sec = 0;
  prevFrameTime.tv_usec = 0;

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
/*
  while (fgets(msg, sizeof(msg) - 1, file))
  {

    RawMessage m;
    if (parseRawFormatFast(msg, &m, false))
    {
      continue; // Parsing failed -> skip the line
    }
    if (strlen(m.timestamp) >= 19)
    {
      m.timestamp[10] = 'T'; // to support 'T', '-' and ' ' separators
      memset(&ctime, 0, sizeof(struct tm));
      milliSecond = strptime(m.timestamp, "%Y-%m-%dT%H:%M:%S", &ctime);
      if ((milliSecond != NULL) && (milliSecond - m.timestamp >= 19)) // convert in tm struct => OK
      {
        frameTime.tv_sec = mktime(&ctime);
        frameTime.tv_usec = (sscanf(milliSecond, ".%3ld", &frameTime.tv_usec) == 1) ? frameTime.tv_usec * 1000 : 0;
        usWait = ((prevFrameTime.tv_sec == 0) && (prevFrameTime.tv_usec == 0)) ? 0 : time_diff(prevFrameTime, frameTime, m.timestamp);
        prevFrameTime = frameTime;
      }
      else // convert in tm struct failed
      {
        usWait = 0;
      }
    }
    else // bad timestamp format YYYY-mm-dd[T|-| ]HH:MM:SS[.xxx] & min length 19 chrs
    {
      usWait = 0;
    }
    if (usWait > 0)
    {
        usleep(usWait);
    }
    writeRawPGNToCanSocket(&m, socketc);
  }
*/
  close(sockfd);
  close(socketc);
  exit(0);
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
    logError("Invalid PGN, too big (0x%x). Skipping.\n", msg->pgn);
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

unsigned long time_diff(struct timeval x , struct timeval y, char *timestamp)
{
  double x_ms , y_ms , diff;

  x_ms = (double)x.tv_sec*1000000 + (double)x.tv_usec;
  y_ms = (double)y.tv_sec*1000000 + (double)y.tv_usec;

  diff = (double)y_ms - (double)x_ms;
  if (diff < 0.0)
  {
    logError("Timestamp back in time at %s\n", timestamp);
    return 0;
  }

  return (unsigned long)diff;
}
