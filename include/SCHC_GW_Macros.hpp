#ifndef SCHC_Macros_hpp
#define SCHC_Macros_hpp

#define _SESSION_POOL_SIZE 10   // Sessions numbers

/* Fragmentation traffic direction */
#define SCHC_FRAG_UP 1
#define SCHC_FRAG_DOWN 2

/* Fragmentation Rule ID*/
#define SCHC_FRAG_UPDIR_RULE_ID 20
#define SCHC_FRAG_DOWNDIR_RULE_ID 21   

/* L2 Protocols */
#define SCHC_FRAG_LORAWAN 1
#define SCHC_FRAG_NBIOT 2
#define SCHC_FRAG_SIGFOX 3

/* Machine States */
#define STATE_RX_INIT 0
#define STATE_RX_RCV_WINDOW 1
#define STATE_RX_WAIT_x_MISSING_FRAGS 2
#define STATE_RX_WAIT_END 3
#define STATE_RX_END 4
#define STATE_RX_TERMINATE_ALL 5

/* SCHC Messages */
#define SCHC_REGULAR_FRAGMENT_MSG 0
#define SCHC_ALL1_FRAGMENT_MSG 1
#define SCHC_ACK_MSG 2
#define SCHC_ACK_REQ_MSG 3
#define SCHC_SENDER_ABORT_MSG 4
#define SCHC_RECEIVER_ABORT_MSG 5 

/* ACK Modes */
#define ACK_MODE_ACK_END_WIN 1
#define ACK_MODE_ACK_END_SES 2
#define ACK_MODE_COMPOUND_ACK 3

#endif