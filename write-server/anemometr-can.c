/*

Reads raw n2k ASCII data from stdin and writes it to a Linux SocketCAN device (e.g. can0).

(C) 2019, Piotr Nowakowski

This file is soft for anemometr

anemometr is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

anemometr is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with CANboat.  If not, see <http://www.gnu.org/licenses/>.

*/
#define _GNU_SOURCE

#include <stdint.h>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <math.h>

#define GLOBALS
//#include "common.h"
//#include "pgn.h"

#define SRC 5
#define PRIO 2
#define PGN1 130306
#define DST 255
#define LEN 8
#define TIMEOUTMS 100
#define TIMETOSENDPGN 0.50              //in seconds

#define DATE_LENGTH 60
//#define FASTPACKET_SIZE (1)
#define FASTPACKET_BUCKET_0_SIZE (6)
#define FASTPACKET_BUCKET_N_SIZE (7)
//#define FASTPACKET_BUCKET_0_OFFSET (2)
//#define FASTPACKET_BUCKET_N_OFFSET (1)
#define FASTPACKET_MAX_INDEX (0x1f)
#define FASTPACKET_MAX_SIZE (FASTPACKET_BUCKET_0_SIZE + FASTPACKET_BUCKET_N_SIZE * (FASTPACKET_MAX_INDEX - 1))

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

unsigned int getCanIdFromISO11783Bits(unsigned int prio, unsigned int pgn, unsigned int src, unsigned int dst);
static int  openCanDevice(char *device, int *socket);
static void writeRawPGNToCanSocket(RawMessage *msg, int socket);
static void sendCanFrame(struct can_frame *frame, int socket);
static void sendN2kFastPacket(RawMessage *msg, struct can_frame *frame, int socket);
static int  readCanFrame(struct can_frame *frame, int socket);
static int  CheckReadCanFrame(struct can_frame *frame);
static void ReadAnemometer(int sog, RawMessage *msg);

int main(int argc, char **argv)
{
  //FILE *file = stdin;
  char          msg[2000];
  int           socket;


  struct can_frame frame;
  memset(&frame, 0, sizeof(frame));

//  setProgName(argv[0]);
  if (argc != 2)
  {
    puts("Usage: socketcan-writer <can-device>");
    exit(1);
  }

  if (openCanDevice(argv[1], &socket))
  {
    exit(1);
  }

  setbuf(stdout, NULL); //disable buffor for print

  RawMessage m;

  int sog;
  struct timespec tm;
  double tscur, tsold;
  while (1)
  {

        if(readCanFrame(&frame, socket))
         if((sog=CheckReadCanFrame(&frame))){
           printf ("SOG:%d\n",sog); //sog potrzebny do obliczenia predkosci pozornej wiatru
         };

        clock_gettime(CLOCK_REALTIME,&tm);              //read curent time
        tscur=tm.tv_nsec/1000000;                       //precision i ms
        tscur/=1000;                                    //as .ms
        tscur+=tm.tv_sec;                               //sec.ms

        if((tscur-tsold)>TIMETOSENDPGN) //delay is > TIMETOSENDPGN - time send new wind parameter
        {
         tsold=tscur;           //copy CurTime to OldTime
         printf(".");

         sog=100;       //na chwile


         ReadAnemometer(sog, &m);       //read data from anemometr
         writeRawPGNToCanSocket(&m, socket);    //send wind to can
        };


}

  close(socket);
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

//Read frame CAN from sockets
static int  readCanFrame(struct can_frame *frame, int socket)
{
 fd_set set;
 struct timeval timeout;
 int rv;

 FD_ZERO(&set); /* clear the set */
 FD_SET(socket, &set); /* add our file descriptor to the set */

 timeout.tv_sec = 0;
 timeout.tv_usec = TIMEOUTMS*1000;
 rv = select(socket + 1, &set, NULL, NULL, &timeout);
 if(rv == -1)
   perror("select"); /* an error accured */
 else if(rv == 0)
   return 0;
  else
   if(read( socket, frame, sizeof(struct can_frame) ) != sizeof(struct can_frame))
             {
                perror("Read");
                return 0;
             }
 return 1;
}


static void sendCanFrame(struct can_frame *frame, int socket)
{
  if (write(socket, frame, sizeof(struct can_frame)) != sizeof(struct can_frame))
  {
    perror("write");
  }
}


static int CheckReadCanFrame(struct can_frame *frame)
{
 unsigned long pgn;

 pgn= (frame->can_id >>8);      //decode pgn
 pgn&=0x01FFFF;                 //number

 if(pgn==129026)                //GPS SOG needed to calculate referece speed wind
  return (frame->data[5] <<8)+frame->data[4];           //speed over groud
 else
  return 0;                     //not sog
}

static void ReadAnemometer(int sog, RawMessage *msg)   //if sog>100 (1m/s) send wind reference on boat else send wind true
{
   int wind_speed, wind_dir;


   msg->prio=PRIO;
   msg->pgn=PGN1;
   msg->src=SRC;
   msg->dst=DST;
   msg->len=LEN;
   msg->data[0]=0;
//   msg->data[1]=0;            //speed L in  cm/s
//   msg->data[2]=0;            //speed H
//   msg->data[3]=0;            //direct L in 0,1 rad *10000
//   msg->data[4]=0;            //direct H
   msg->data[5]=4;              //4 - reference dir wind, 3 - true wind
   msg->data[6]=0;
   msg->data[7]=0;

   //read wind and direction from rs (device)
    wind_speed=rand() % 6000;
    wind_dir=rand();

   if(sog>=100) {                  //calculate wind speed and direct. true when sog > 1m/s
    double wind_dir_rad=wind_dir;
    double wind_dir_true;
    int wind_speed_true;
    wind_dir_rad/=10000;

    wind_dir_true=atan(sin(wind_dir_rad)/((wind_speed/sog)-cos(wind_dir_rad)));
    wind_speed_true=(sin(wind_dir_rad)/sin(wind_dir_true))*100;
    wind_dir_true+=wind_dir_rad;

     msg->data[5]=3;            //true wind
     wind_speed=wind_speed_true; //safe to oryginal var
     wind_dir=wind_dir_true*10000;
    }

 msg->data[1]=(wind_speed & 0x00FF);    //convert wind speed int to 2xchar cm/s
 msg->data[2]=(wind_speed >>8);

 msg->data[3]=(wind_dir &0x00ff);       //convert wind dir int to 2xchar
 msg->data[4]=(wind_dir >>8);
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
