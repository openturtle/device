/*
 * device.cpp
 *
 * Created: 6/28/2017 7:52:00 AM
 * Author : @ibrohimislam
 */ 

#include <avr/io.h>
#include "lib/packet.h"
#include "lib/ldpc.h"
#include "lib/ST7540.h"
#include "lib/state_machine.h"

extern "C" {
	#include "pca.h"
	#include "bus.h"
	#include "serial.h"
	#include "spi_common.h"
	#include "spi_hw_poll.h"
}

uint8_t ST7540_REG[3] = {0x05, 0x24, 0x27};

uint16_t myAddress = 0x7702;
PacketInTransit packetToSend = packetInitializer();

typedef struct {
	uint16_t src;
	uint16_t dst;
	uint16_t tos;
	uint8_t payload[14];
} AppPacket;

typedef union {
	uint8_t byte[20];
	AppPacket data;
} AppPacketInTransit;

AppPacketInTransit appPacketInTransit;

void handlePacket(uint8_t* packet, struct bus_t &spi_bus);
bool isPacketForMe(uint8_t* packet);

int main(void)
{
	struct bus_t spi_bus = initialize();
	
	serial_init(E_BAUD_9600);
	serial_install_interrupts(E_FLAGS_SERIAL_RX_INTERRUPT);
	serial_flush();

	serial_install_stdio();
	
	//PacketPointer packet;
	//setPacketPointer(packet, packetToSend);
	
	StateMachine stateMachine;
	
    while (1) {
		
		if (PIN_CD_PD & (1<<CD_PD)) {
			PORT_LED = PORT_LED & ~(1<<LED1);
			} else {
			PORT_LED = PORT_LED | (1<<LED1);
		}
		
		while (spi_bus.f_avail(spi_bus.priv)){
			
			uint8_t c;
			spi_bus.f_getc(spi_bus.priv, &c);
			
			stateMachine.handleInput(c);
			
			if (stateMachine.getEvent() == EVENT_RECEIVE_PACKET) {
						
				uint8_t* receivedPacket = stateMachine.output;
				
				if (isPacketForMe(receivedPacket)) {
					handlePacket(receivedPacket, spi_bus);
				}
				
			}
			
			PORT_LED = PORT_LED ^ (1<<LED2);
				
		}
		
    }
}

bool isPacketForMe(uint8_t* packet) {
	uint16_t address = (packet[3] << 8) | packet[2];
	return address == myAddress;
}

void handlePacket(uint8_t* packet, struct bus_t &spi_bus) {
	
	for (int i=0; i<20; i++) {
		appPacketInTransit.byte[i] = packet[i];
	}
	
	if (appPacketInTransit.data.tos == 0x0001) {
		
		appPacketInTransit.data.tos = 0x0002;
		appPacketInTransit.data.dst = appPacketInTransit.data.src;
		appPacketInTransit.data.src = myAddress;
		
		for (int i=0; i<20; i++) {
			packetToSend.data.body[i] = appPacketInTransit.byte[i];
		}
		
		tx_mode(); _delay_ms(10);
		spi_bus.f_send(spi_bus.priv, packetToSend.byte, 32, 1);
		rx_mode(); _delay_ms(10);
	}
	
}

