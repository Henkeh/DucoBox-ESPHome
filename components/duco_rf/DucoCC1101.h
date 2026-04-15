/*
 * Author: Arne Mauer
 * Parts of this code is based on the itho-library made by 'supersjimmie', 'Thinkpad', 'Klusjesman' and 'jodur'.
 */

#ifndef __DUCOCC1101_H__
#define __DUCOCC1101_H__

#include <stdio.h>
#include <stdint.h>
#include <functional>
#include <string>
#include <vector>

#define CCPACKET_BUFFER_LEN                      35
#define CCPACKET_DATA_LEN                        (CCPACKET_BUFFER_LEN - 3)
#define CC1101_DATA_LEN                          61

struct CC1101Packet {
	uint8_t data[CCPACKET_BUFFER_LEN];
	uint8_t length;
	uint8_t rssi;
	uint8_t lqi;
	bool crc_ok;
};

struct DucoPacket {
	uint8_t messageType;
	uint8_t networkId[4];
	uint8_t sourceAddress;
	uint8_t destinationAddress;
	uint8_t originalSourceAddress;
	uint8_t originalDestinationAddress;
	uint8_t counter;
	uint8_t rssi;
	uint8_t rssiSender;
	uint8_t lqi;
	bool    crc_ok;
	uint8_t length;
	uint8_t data[CC1101_DATA_LEN];
	uint8_t dataLength;
};

struct InboxQMessage {
	DucoPacket    packet;
	bool          messageProcessed    = true;
	unsigned long timeReceivedMessage = 0;
};

struct OutboxQMessage {
	DucoPacket    packet;
	bool          hasSent      = true;   // true = slot is free
	bool          waitForAck   = false;
	bool          ackReceived  = false;
	unsigned long ackTimer     = 0;
	uint8_t       sendRetries  = 0;
};

// this temporarly networkId is the hardware device ID of the node
const uint8_t joinCO2NetworkId[4] = {0x00,0x00,0x7C,0x3E};

// 0x00 = not initialised
// 0x01 = join -> step 1, join1 message sent, waiting for join2 message
// 0x02 = join -> step 2, join2 message received
// 0x03 = join -> step 3, join3 message sent, waiting for join4 message
// 0x04 = join -> step 4, join4 message received,
// 0x05 = join succesfull, new networkid and deviceid

// 0x0A = disjoin command sent
// 0x0B = waiting for ack
// 0x0C = ducobox disjoin confirmation
// 0x0D = device disjointed

// 0x14 = initialised with address/networkid from config
// 0x15 = initialisation failed stage 1 -> calibration failed or no response from cc1101
// 0x16 = initialisation failed stage 2 -> set RX mode failed or no response from cc1101

const uint8_t ducoDeviceState_notInitialised = 0x00;
const uint8_t ducoDeviceState_join1 = 0x01;
const uint8_t ducoDeviceState_join2 = 0x02;
const uint8_t ducoDeviceState_join3 = 0x03;
const uint8_t ducoDeviceState_join4 = 0x04;
const uint8_t ducoDeviceState_joinSuccessful = 0x05;

const uint8_t ducoDeviceState_disjoinRequest = 0x0A;
const uint8_t ducoDeviceState_disjoinWaitingForAck = 0x0B;
const uint8_t ducoDeviceState_disjoinWaitingForConfirmation = 0x0C;
const uint8_t ducoDeviceState_disjointed = 0x0D;

const uint8_t ducoDeviceState_initialised = 0x14;
const uint8_t ducoDeviceState_initialisationCalibrationFailed = 0x15;
const uint8_t ducoDeviceState_initialisationRxmodeFailed = 0x16;

const uint8_t ducomsg_network 	= 0x00;
const uint8_t ducomsg_join1		= 0x01;
const uint8_t ducomsg_join2		= 0x02;
const uint8_t ducomsg_join3 	= 0x03;
const uint8_t ducomsg_join4 	= 0x04;
const uint8_t ducomsg_ack 		= 0x06;
const uint8_t ducomsg_message 	= 0x07;

class DucoCC1101 {
	private:
		//receive
		CC1101Packet inMessage;					//temp storage message
		CC1101Packet outMessage;

		//settings
		uint8_t sendTries;						//number of times a command is send at one button press

		uint8_t deviceAddress;
		uint8_t radioPower;
		uint8_t networkId[4];

		//uint8_t pingCounter;
		uint8_t messageCounter;			// counter for messages the gateway sends (1-15 dec)

		// keeps the current ventilationmode values
		uint8_t currentVentilationMode;
		bool permanentVentilationMode;
		uint8_t overrulePercentage;
		uint8_t temperature;
		uint8_t ducoDeviceState;

		bool installerModeActive;
		bool logRFMessages = false;
		std::function<bool(const std::vector<uint8_t> &)> tx_callback_;

		uint8_t numberOfLogmessages = 0;

	//functions
	public:
		DucoCC1101(uint8_t counter = 0, uint8_t sendTries = 3);		//set initial counter value
		~DucoCC1101();

		#define OUTBOXQ_MESSAGES       3		// duco uses a buffer of 8 messages
		#define INBOXQ_MESSAGES        3 		// duco uses a buffer of 8 messages

		OutboxQMessage outboxQ[OUTBOXQ_MESSAGES];
		InboxQMessage inboxQ[INBOXQ_MESSAGES];

		uint8_t getInboxQFreeSpot();
		uint8_t getOutboxQFreeSpot();

		void arrayToString(uint8_t array[], unsigned int len, char buffer[]);

		//init
		void init() {}											// retained for API compatibility
		void setSendTries(uint8_t sendTries) { this->sendTries = sendTries; }
		void setTemperature(int temperature); // set temperature in degrees celsius x 10. 21.5 C = int (215)
		void set_tx_callback(std::function<bool(const std::vector<uint8_t> &)> callback);
		bool ingest_packet(const std::vector<uint8_t> &payload, int rssi_dbm, uint8_t lqi);
		uint8_t getCurrentVentilationMode() { return this->currentVentilationMode; }	//retrieve last received/parsed command from remote
		bool getCurrentPermanentMode() { return this->permanentVentilationMode; }		//retrieve last received/parsed command from remote
		uint8_t getDucoDeviceState() { return this->ducoDeviceState; } // retrieve radio/duco state
		void setLogRFMessages(bool logRFMessages);
		bool getLogRFMessages() { return logRFMessages; }

		//deviceid
		void setGatewayAddress(uint8_t deviceAddress); // if this function name is "setDeviceAddress" the esp8266 crashes on sendData
		uint8_t getDeviceAddress() { return this->deviceAddress; }

		//get/set radio power
		void setRadioPower(uint8_t radioPower);
		uint8_t getRadioPower() { return radioPower; }

		//networkid
		void setNetworkId(uint8_t newNetworkId[4]);
		uint8_t* getnetworkID() { return networkId; }

		// logging
		#define NUMBER_OF_LOG_STRING 32
		#define MAX_LOG_STRING_SIZE 128
		uint8_t getNumberOfLogMessages();
		char logMessages[NUMBER_OF_LOG_STRING][MAX_LOG_STRING_SIZE];

		#define ULINT_MAX 4294967295
		unsigned long int messageReceivedCounter; 	// 0 to 4,294,967,295
		unsigned long int messageSentCounter;		// 0 to 4,294,967,295

		unsigned long int getMessageReceivedCounter() { return this->messageReceivedCounter; }
		unsigned long int getMessageSentCounter() { return this->messageSentCounter; }

		void checkForAck();
		void processReceivedAck(uint8_t messageCounter);

		void sendJoinPacket();
		void sendDisjoinPacket();

		bool matchingNetworkId(uint8_t id[]);

		//receive
		void processNewMessages(); // process messages in InboxQ
		void processMessage(uint8_t messageQNumber); // process message

		void requestVentilationMode(uint8_t ventilationMode, bool setPermanentVentilationMode, uint8_t percentage, uint8_t buttonPresses);
		void sendVentilationModeMessage(bool setPermanent, bool setVentilationMode, uint8_t ventilationMode, uint8_t percentage, uint8_t temp, bool subscribe, bool activateInstallerMode, uint8_t buttonPresses);
		void prepareVentilationModeMessage(uint8_t outboxQMessageNumber, uint8_t commandNumber, bool setPermanent, bool setVentilationMode, uint8_t ventilationMode, uint8_t percentage, uint8_t temp, bool subscribe, bool activateInstallerMode, uint8_t buttonPresses);
		void sendSubscribeMessage();

		void enableInstallerMode();
		void disableInstallerMode();

		uint8_t getRssi(uint8_t rssi);

		bool getInstallerModeActive(){ return installerModeActive; }
		bool pollNewDeviceAddress();
		int convertRssiHexToDBm(uint8_t rssi);


	protected:
	private:
		DucoCC1101( const DucoCC1101 &c);
		DucoCC1101& operator=( const DucoCC1101 &c);

		// logging
		void setLogMessage(const char *newLogMessage);

		void sendDataToDuco(CC1101Packet *packet, uint8_t outboxMessageQNumber);

		void resetOutDucoPacket(uint8_t outboxQMessageNumber);

  		char valToHex(uint8_t val);

 		std::string byteToHexString(uint8_t b);

		void processJoin2Packet(uint8_t inboxQMessageNumber);
		void sendJoin3Packet(uint8_t inboxQMessageNumber);
		bool joinPacketValidNetworkId(uint8_t inboxQMessageNumber);

		void processJoin4Packet(uint8_t inboxQMessageNumber);
		void sendJoin4FinishPacket(uint8_t inboxQMessageNumber);

		void finishDisjoin(uint8_t inboxQMessageNumber);

		void processNetworkPacket(uint8_t messageQNumber);

		uint8_t updateMessageCounter();

		//parse received message
		void parseMessageCommand(uint8_t inboxQMessageNumber);
		void repeatMessage(uint8_t inboxQMessageNumber);
		bool processNewVentilationMode(uint8_t inboxMessageQNumber, uint8_t commandNumber, uint8_t startByteCommand);
		void sendNodeParameterValue(uint8_t outboxQMessageNumber,  uint8_t inboxQMessageNumber,uint8_t commandNumber, uint8_t startByteCommand);
		bool matchingDeviceAddress(uint8_t compDeviceAddress);

		//send
		void ducoToCC1101Packet(DucoPacket *duco, CC1101Packet *packet);
		void prefillDucoPacket(DucoPacket *ducoOutPacket, uint8_t receiverAddress);
		void prefillDucoPacket(DucoPacket *ducoOutPacket, uint8_t receiverAddress, uint8_t originalSourceAddress, uint8_t originalDestinationAddress );

		void setCommandLength(DucoPacket *ducoOutPacket, uint8_t commandNumber, uint8_t commandLength);
		uint8_t getCommandLength(uint8_t commandLengthByte1, uint8_t commandLengthByte2, uint8_t commandNumber);
		void sendAck(uint8_t inboxQMessageNumber);
		void waitForAck(uint8_t outboxQMessageNumber);

}; //DucoCC1101

#endif //__DUCOCC1101_H__
