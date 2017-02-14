#
#  ======== readme.txt ========
#

BigdataIPC example


Program Logic:
Initially, the host sends first message with shared memory init information
 followed by  2 more dummy messages to slave core ( all three messages sent in
 sequence without waiting for reply).
Independantly, the slave processor receives messages and  sends back reply
 back for each of the messages to the host.
Then the host receives one message from the slave and sends a message with
 Big data buffer allocated from the Big data heap and filled with an
 incrementing pattern. (This process is repeated with 10 Big data Buffer
 messages). Each of these Messages are received by slave and the values
 in the buffers are updated with a modified incrementing pattern
 and sent back to the host.

Note the Slave and Host processors checks the expected incrementing
 pattern for errors.

At this point only 7 Big data buffer messages would have been received.
Then the host sends two dummy messages plus one  shutdown message to the slave
 core when receiving the remaining three Big data buffer messages.
The slave core on receiving the shutdown message, shuts itself down and
 reinitializes itself for future runs.

Then the host receives back the remaining returned messages before shutting
 down.

Build_procedure
---------------
Build is structured to be initiated from the Processor SDK top level makefile.

If you want to override the components or do a local build, then the product.mak
need to be modified to supply the path names of the different components.
