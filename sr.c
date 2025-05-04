#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include "emulator.h"
#include "sr.h"

/* ******************************************************************
   Go Back N protocol.  Adapted from J.F.Kurose
   ALTERNATING BIT AND GO-BACK-N NETWORK EMULATOR: VERSION 1.2

   Network properties:
   - one way network delay averages five time units (longer if there
   are other messages in the channel for GBN), but can be larger
   - packets can be corrupted (either the header or the data portion)
   or lost, according to user-defined probabilities
   - packets will be delivered in the order in which they were sent
   (although some can be lost).

   Modifications:
   - removed bidirectional GBN code and other code not used by prac.
   - fixed C style to adhere to current programming style
   - added GBN implementation
**********************************************************************/

#define RTT  16.0       /* round trip time.  MUST BE SET TO 16.0 when submitting assignment */
#define WINDOWSIZE 6    /* the maximum number of buffered unacked packet
                          MUST BE SET TO 6 when submitting assignment */
#define SEQSPACE 12      /* the min sequence space for SR must be at least windowsize*2 */
#define NOTINUSE (-1)   /* used to fill header fields that are not being used */

/* generic procedure to compute the checksum of a packet.  Used by both sender and receiver
   the simulator will overwrite part of your packet with 'z's.  It will not overwrite your
   original checksum.  This procedure must generate a different checksum to the original if
   the packet is corrupted.
*/
int ComputeChecksum(struct pkt packet)
{
  int checksum = 0;
  int i;

  checksum = packet.seqnum;
  checksum += packet.acknum;
  for ( i=0; i<20; i++ )
    checksum += (int)(packet.payload[i]);

  return checksum;
}

bool IsCorrupted(struct pkt packet)
{
  if (packet.checksum == ComputeChecksum(packet))
    return (false);
  else
    return (true);
}


/********* Sender (A) variables and functions ************/

static struct pkt buffer[WINDOWSIZE];  /* array for storing packets waiting for ACK */
static int windowfirst, windowlast;    /* array indexes of the first/last packet awaiting ACK */
static int windowcount;                /* the number of packets currently awaiting an ACK */
static int A_nextseqnum;               /* the next sequence number to be used by the sender */
static int acked[SEQSPACE];            /* array for storing packets ACKed status (0 unacked /1 acked) */
static int timer_running;              /* Added variable to track the status of timer */

/* called from layer 5 (application layer), passed the message to be sent to other side */
void A_output(struct msg message)
{
  struct pkt sendpkt;
  int i;

  /* if not blocked waiting on ACK */
  if ( windowcount < WINDOWSIZE) {
    if (TRACE > 1)
      printf("----A: New message arrives, send window is not full, send new messge to layer3!\n");

    /* create packet */
    sendpkt.seqnum = A_nextseqnum;
    sendpkt.acknum = NOTINUSE;
    for ( i=0; i<20 ; i++ )
      sendpkt.payload[i] = message.data[i];
    sendpkt.checksum = ComputeChecksum(sendpkt);

    /* put packet in window buffer */
    /* windowlast will always be 0 for alternating bit; but not for GoBackN */
    windowlast = (windowlast + 1) % WINDOWSIZE;
    buffer[windowlast] = sendpkt;
    windowcount++;
    /* set current pkt sent as unacked*/
    acked[A_nextseqnum] = 0;

    /* send out packet */
    if (TRACE > 0)
      printf("Sending packet %d to layer 3\n", sendpkt.seqnum);
    tolayer3 (A, sendpkt);

    /* start timer if first packet in window */
    if (windowcount == 1) {
      starttimer(A,RTT);
      timer_running = 1;
    }

    /* get next sequence number, wrap back to 0 */
    A_nextseqnum = (A_nextseqnum + 1) % SEQSPACE;
  }
  /* if blocked,  window is full */
  else {
    if (TRACE > 0)
      printf("----A: New message arrives, send window is full\n");
    window_full++;
  }
}


/* called from layer 3, when a packet arrives for layer 4
   In this practical this will always be an ACK as B never sends data.
*/
void A_input(struct pkt packet)
{
  /* remember current base so we can detect later if window moved */
  int old_windowfirst = windowfirst;

  /* if received ACK is not corrupted */
  if (!IsCorrupted(packet)) {
    if (TRACE > 0)
      printf("----A: uncorrupted ACK %d is received\n",packet.acknum);
    total_ACKs_received++;

    /* Only first arrival of this ACK matters; duplicate ACK ignored */
    if(!acked[packet.acknum]) {

      /* Set pkt acked status to acked (1) */
      acked[packet.acknum] = 1;   /* mark this seqnum as ACKed */
      
      if (TRACE > 0)
      printf("----A: ACK %d is not a duplicate\n",packet.acknum);

      new_ACKs++;

    } else {
      if (TRACE > 0)
         printf ("----A: duplicate ACK received, do nothing!\n");
    }

    /* slide window once the oldest packet ACKed */
    while (windowcount > 0 && acked[buffer[windowfirst].seqnum]) {
      /* Reset ACKed status to 0 */
      acked[buffer[windowfirst].seqnum] = 0;
      /* Slide window to right by 1 */
      windowfirst = (windowfirst + 1) % WINDOWSIZE;
      /* Total unacked pkt count reduce by 1 */
      windowcount --;
    }

    /* ----- start timer again if there are still more unacked packets in window ----- */
    /* if there's no pkt waiting to be acked AND timer is running. Stop the timer. */
    if (windowcount == 0 && timer_running) {
      stoptimer(A); /* Only stop timer if it's already running */
      timer_running = 0;
      if (TRACE > 0)
        printf("----A: No packets to track, timer stopped.\n");
    }
    /* Start a new timer only if:  
      (a) there are still unACKed packets, AND  
      (b) base actually moved (i.e., we now time a new oldest unacked pkt) */
    else if (windowcount > 0 && windowfirst != old_windowfirst) {
      if (timer_running) stoptimer(A);  /*avoid stacking timers*/ 
      starttimer(A, RTT);
      timer_running = 1;}

      /* Track which packet timer's tracking*/
      if (TRACE > 0)
        printf("----A: Timer restarted for packet %d\n", buffer[windowfirst].seqnum);
  } else { 
    /* Else if ACK is corrupted, ignore it and do ntohing to the winow or timer */
      if (TRACE > 0)
        printf ("----A: corrupted ACK is received, do nothing!\n");
    }
}

/* called when A's timer goes off */
void A_timerinterrupt(void)
{

  if (TRACE > 0)
    printf("----A: time out,resend packets!\n");
  
  /* Retransmit the packet at the head of the send window */
  /* buffer[windowfirst] is the oldest unacknowledged frame */
  if (TRACE > 0)
    printf ("---A: resending packet %d\n", (buffer[windowfirst]).seqnum);
  tolayer3(A, buffer[windowfirst]);
  packets_resent++;

  /* RTT is the fixed timeout value */
  starttimer(A,RTT);
  timer_running = 1; /* record that the timer is active */
  
}


/* the following routine will be called once (only) before any other */
/* entity A routines are called. You can use it to do any initialization */
void A_init(void)
{
  /* initialise A's window, buffer and sequence number */
  A_nextseqnum = 0;  /* A starts with seq num 0, do not change this */
  windowfirst = 0;
  windowlast = -1;   /* windowlast is where the last packet sent is stored.
		     new packets are placed in winlast + 1
		     so initially this is set to -1
		   */
  windowcount = 0;
  timer_running = 0; /* timer_running is whether the timer is on/off, 0 means off, 1 means on */
  
}



/********* Receiver (B)  variables and procedures ************/

static int expectedseqnum; /* the sequence number expected next by the receiver */
static int B_nextseqnum;   /* the sequence number for the next packets sent by B */
static struct pkt recv_buffer[SEQSPACE]; /* Buffer received packets */
static int received[SEQSPACE]; /* To track whether each packet received by B */

/* called from layer 3, when a packet arrives for layer 4 at B*/
void B_input(struct pkt packet)
{
  struct pkt sendpkt;
  int i;
  int seq = packet.seqnum;

  /* if not corrupted and received packet is in order */
  if  (!IsCorrupted(packet))  {
    if (TRACE > 0)
      printf("----B: packet %d is correctly received, send ACK!\n",packet.seqnum);
    packets_received++;

    /* if packet is within the receiver window and never received, buffer it */
    if (((seq - expectedseqnum + SEQSPACE) % SEQSPACE) < WINDOWSIZE && !received[seq]) {
      recv_buffer[seq] = packet;
      received[seq] = 1;
      if (TRACE > 0)
        printf("----B: packet %d buffered\n", seq);
    }
    
    /* --------------- Send an ACK for the received packet --------------- */
    /* create packet */
    sendpkt.acknum = seq;
    sendpkt.seqnum = B_nextseqnum;
    B_nextseqnum = (B_nextseqnum + 1) % 2;
    for (i=0; i<20; i++)
      sendpkt.payload[i] = '0';
    /* computer checksum */
    sendpkt.checksum = ComputeChecksum(sendpkt);
    
    /* send out packet */
    tolayer3 (B, sendpkt);
    if (TRACE > 0)
      printf("----B: ACK %d sent\n", seq);

    /* -------------------------------------------------------------------- */
    /* Deliver buffered packet to Application Layer,
       starting from expectedseqnum */
    while (received[expectedseqnum]) {
      
      /* deliver to receiving application */
      tolayer5(B, recv_buffer[expectedseqnum].payload);
      received[expectedseqnum] = 0; /* Reset corresponding position to unreceived */
      
      /* Test TRACE */
      if (TRACE > 0)
        printf("----B: packet %d delivered to layer5\n", expectedseqnum);
      
      /* update state variables */
      expectedseqnum = (expectedseqnum + 1) % SEQSPACE;
    }
      
  }
  else {
    /* packet is corrupted or out of order resend last ACK */
    if (TRACE > 0)
      printf("----B: packet corrupted or not expected sequence number, resend ACK!\n");
    sendpkt.acknum = (expectedseqnum == 0) ? (SEQSPACE - 1) : (expectedseqnum - 1);
    sendpkt.seqnum = B_nextseqnum;
    B_nextseqnum = (B_nextseqnum + 1) % 2;
    for (i = 0; i < 20; i++) sendpkt.payload[i] = '0';
    sendpkt.checksum = ComputeChecksum(sendpkt);
    tolayer3(B, sendpkt);
  }
}

/* the following routine will be called once (only) before any other */
/* entity B routines are called. You can use it to do any initialization */
void B_init(void)
{
  expectedseqnum = 0;
  B_nextseqnum = 1;
}

/******************************************************************************
 * The following functions need be completed only for bi-directional messages *
 *****************************************************************************/

/* Note that with simplex transfer from a-to-B, there is no B_output() */
void B_output(struct msg message)
{
}

/* called when B's timer goes off */
void B_timerinterrupt(void)
{
}
