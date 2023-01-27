#define CANMODE      STD          // 9 identifier bits, EXT == 29

#define CANBAUD     500000
// Using their standard numbers here, these are variable
#define NUM_TX_MAILBOXES  2
#define NUM_RX_MAILBOXES  16

int canSetup(){
  Can0.begin();
  Can0.setBaudRate(CANBAUD);

  // Setup Max Mailbox Count
  Can0.setMaxMB(NUM_TX_MAILBOXES + NUM_RX_MAILBOXES);

  // Setup RX mailbox address space
  for(int n = 0; n < NUM_RX_MAILBOXES; n++){
    Can0.setMB((FLEXCAN_MAILBOX)n, RX, CANMODE);
  }

  // Setup TX mailbox address space
  for(int n = NUM_RX_MAILBOXES; n < (NUM_RX_MAILBOXES + NUM_TX_MAILBOXES); n++){
    Can0.setMB((FLEXCAN_MAILBOX)n, TX, CANMODE);
  }

  Can0.setMBFilter(REJECT_ALL);   // Reject all messages not specified for MBs
  Can0.enableMBInterrupts();


  // Filter [low ID, high ID]
  Can0.onReceive(MB0, emergency_CFGS_handler);       Can0.setMBFilter(MB0, 0x000, 0x00F);  // Emergency messaging and configuration/control  // Highest Priority
  Can0.onReceive(MB1, motor_impulse_handler);        Can0.setMBFilter(MB1, 0x010, 0x01F);  // Motor, Arm, and Torpedo Controls
  Can0.onRecieve(MB2, sensor_interaction_handler);   Can0.setMBFilter(MB2, 0x020, 0x02F);  // Handle Sensor DREQ and DRES
  

  Can0.onReceive(MB15, aux_function_handler);       Can0.setMBFilter(MB15, 0x100, 0x1FF);  // Data streams, readback, etc
  
  


  // Specific value to Mailbox, only use if a range isn't desired
  //Can0.enhanceFilter(MB0);
  //Can0.enhanceFilter(MB1);
  //Can0.enhanceFilter(MB2);
  return 0;
}


void canDecode(const CAN_message_t &msg){
  Serial.printf("Mailbox: %i Triggered!\n", msg.mb);
  
  // Set to 1 if CAN data is recieved but buffers are full
  // This should always show 0 if things aren't broken
  Serial.printf("Overrun Status:\t%i\n", msg.flags.overrun);

  Serial.printf("Message Length:\t%i\n",msg.len);
  Serial.printf("Extended Msg??\t%i\n", msg.flags.extended);
  Serial.printf("Time Stamp Msg:\t%u\n", msg.timestamp);
  Serial.printf("Message ID:\t%.3X\n", msg.id);
  Serial.printf("Message Contents:\n\t");
  
  for(uint8_t n = 0; n < msg.len; n++){
    Serial.printf("%.2X ", msg.buf[n]);
  }
  Serial.println();
}
