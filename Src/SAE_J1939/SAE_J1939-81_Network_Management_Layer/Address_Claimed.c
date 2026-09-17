/*
 * Address_Claimed.c
 *
 *  Created on: 14 juli 2021
 *      Author: Daniel Mårtensson
 */

#include "Network_Management_Layer.h"

/* Layers */
#include "../SAE_J1939-21_Transport_Layer/Transport_Layer.h"
#include "../../Hardware/Hardware.h"

/*
 * Send request address claimed to other ECU. Every time we asking addresses from other ECU, then we clear our storage of other ECU
 * PGN: 0x00EE00 (60928)
 */
ENUM_J1939_STATUS_CODES SAE_J1939_Send_Request_Address_Claimed(J1939 *j1939, uint8_t DA) {
	/* Delete all addresses by setting them to broadcast address and set the counters to 0 */
	memset(j1939->other_ECU_address, 0xFF, 0xFF);
	j1939->number_of_cannot_claim_address = 0;
	j1939->number_of_other_ECU = 0;
	return SAE_J1939_Send_Request(j1939, DA, PGN_ADDRESS_CLAIMED);
}

/* Pack struct Name into standard 8-byte J1939 NAME format */
static void Pack_NAME(const struct Name *name, uint8_t data[8]) {
	data[0] = (uint8_t)name->identity_number;
	data[1] = (uint8_t)(name->identity_number >> 8);
	data[2] = (uint8_t)((name->identity_number >> 16) | (name->manufacturer_code << 5));
	data[3] = (uint8_t)(name->manufacturer_code >> 3);
	data[4] = (uint8_t)((name->function_instance << 3) | name->ECU_instance);
	data[5] = (uint8_t)name->function;
	data[6] = (uint8_t)(name->vehicle_system << 1);
	data[7] = (uint8_t)((name->arbitrary_address_capable << 7) | (name->industry_group << 4) | name->vehicle_system_instance);
}

/*
 * Compare two 64-bit NAMEs according to SAE J1939-81.
 * Byte 7 is most significant, byte 0 is least significant.
 * Lower numerical value has higher priority.
 * Returns:
 *   -1 if name1 < name2 (name1 has HIGHER priority)
 *    1 if name1 > name2 (name1 has LOWER priority)
 *    0 if identical
 */
static int Compare_NAMEs(const uint8_t name1[8], const uint8_t name2[8]) {
	for (int8_t i = 7; i >= 0; i--) {
		if (name1[i] < name2[i]) {
			return -1;
		} else if (name1[i] > name2[i]) {
			return 1;
		}
	}
	return 0;
}

/* Check if address is in use by another ECU in the network */
static bool Is_Address_Used(const J1939 *j1939, uint8_t address) {
	if (address == j1939->information_this_ECU.this_ECU_address) {
		return true;
	}
	for (uint8_t i = 0; i < j1939->number_of_other_ECU; i++) {
		if (j1939->other_ECU_address[i] == address) {
			return true;
		}
	}
	return false;
}

/*
 * Find next available address in arbitrary address range (128..247 / 0x80..0xF7).
 * Starts search from (current_address + 1) and wraps around the range.
 * Returns 0xFE if no address is available.
 */
uint8_t SAE_J1939_Find_Available_Arbitrary_Address(const J1939 *j1939) {
	uint8_t current = j1939->information_this_ECU.this_ECU_address;
	if (current < 128 || current > 247) {
		current = 128;
	}
	for (uint16_t step = 1; step <= (247 - 128 + 1); step++) {
		uint8_t candidate = 128 + (uint8_t)((current - 128 + step) % (247 - 128 + 1));
		if (!Is_Address_Used(j1939, candidate)) {
			return candidate;
		}
	}
	return 0xFE;
}

/*
 * Relinquish current address when collision is detected with another device,
 * and attempt to claim a new available address if Arbitrary Address Capable.
 */
void SAE_J1939_Relinquish_Address(J1939 *j1939, uint8_t taken_address) {
	/* 1. Record taken_address in other_ECU_address list */
	bool already_stored = false;
	for (uint8_t i = 0; i < j1939->number_of_other_ECU; i++) {
		if (j1939->other_ECU_address[i] == taken_address) {
			already_stored = true;
			break;
		}
	}
	if (!already_stored && j1939->number_of_other_ECU < 255) {
		j1939->other_ECU_address[j1939->number_of_other_ECU++] = taken_address;
	}

	/* 2. If Arbitrary Address Capable, find new address and claim it */
	if (j1939->information_this_ECU.this_name.arbitrary_address_capable) {
		uint8_t next_addr = SAE_J1939_Find_Available_Arbitrary_Address(j1939);
		if (next_addr != 0xFE) {
			j1939->information_this_ECU.this_ECU_address = next_addr;
			SAE_J1939_Response_Request_Address_Claimed(j1939);
			return;
		}
	}

	/* Cannot claim address */
	SAE_J1939_Send_Address_Not_Claimed(j1939);
}

/*
 * Response the request address claimed about this ECU to all ECU - Broadcast. This function must be called at the ECU start up according to J1939 standard
 * PGN: 0x00EE00 (60928)
 */
ENUM_J1939_STATUS_CODES SAE_J1939_Response_Request_Address_Claimed(J1939 *j1939) {
	uint32_t ID = (0x18EEFF << 8) | j1939->information_this_ECU.this_ECU_address;
	uint8_t data[8];
	Pack_NAME(&j1939->information_this_ECU.this_name, data);
	return CAN_Send_Message(ID, data);
}

/*
 * Store the address claimed information about other ECU and perform address collision arbitration
 * PGN: 0x00EE00 (60928)
 */
void SAE_J1939_Read_Response_Request_Address_Claimed(J1939 *j1939, uint8_t SA, uint8_t data[]) {
	/* Check for address collision with our ECU */
	if (j1939->information_this_ECU.this_ECU_address < 0xFE && j1939->information_this_ECU.this_ECU_address == SA) {
		uint8_t our_name[8];
		Pack_NAME(&j1939->information_this_ECU.this_name, our_name);
		int cmp = Compare_NAMEs(our_name, data);

		if (cmp < 0) {
			/* Our ECU has lower numerical NAME -> HIGHER priority -> We WIN.
			 * Broadcast Address Claimed to reassert claim on our address. */
			SAE_J1939_Response_Request_Address_Claimed(j1939);
			return;
		} else {
			/* Other ECU has higher priority (or identical NAME) -> We LOSE.
			 * Record that other ECU took this address. */
			bool already_stored = false;
			for (uint8_t i = 0; i < j1939->number_of_other_ECU; i++) {
				if (j1939->other_ECU_address[i] == SA) {
					already_stored = true;
					break;
				}
			}
			if (!already_stored && j1939->number_of_other_ECU < 255) {
				j1939->other_ECU_address[j1939->number_of_other_ECU++] = SA;
			}

			/* If Arbitrary Address Capable, attempt to claim a new address */
			if (j1939->information_this_ECU.this_name.arbitrary_address_capable) {
				uint8_t next_addr = SAE_J1939_Find_Available_Arbitrary_Address(j1939);
				if (next_addr != 0xFE) {
					j1939->information_this_ECU.this_ECU_address = next_addr;
					SAE_J1939_Response_Request_Address_Claimed(j1939);
					return;
				}
			}

			/* Cannot claim address: send Address Not Claimed (SA = 0xFE) */
			SAE_J1939_Send_Address_Not_Claimed(j1939);
			return;
		}
	}

	/* No collision: store other ECU name and address */
	j1939->from_other_ecu_name.identity_number = ((data[2] & 0b00011111) << 16) | (data[1] << 8) | data[0];
	j1939->from_other_ecu_name.manufacturer_code = (data[3] << 3) | (data[2] >> 5);
	j1939->from_other_ecu_name.function_instance = data[4] >> 3;
	j1939->from_other_ecu_name.ECU_instance = data[4] & 0b00000111;
	j1939->from_other_ecu_name.function = data[5];
	j1939->from_other_ecu_name.vehicle_system = data[6] >> 1;
	j1939->from_other_ecu_name.arbitrary_address_capable = data[7] >> 7;
	j1939->from_other_ecu_name.industry_group = (data[7] >> 4) & 0b0111;
	j1939->from_other_ecu_name.vehicle_system_instance = data[7] & 0b00001111;
	j1939->from_other_ecu_name.from_ecu_address = SA;

	/* Remember the source address of the ECU */
	bool exist = false;
	uint8_t i;
	for (i = 0; i < j1939->number_of_other_ECU; i++){
		if (j1939->other_ECU_address[i] == SA){
			exist = true;
			break;
		}
	}
	if (!exist && j1939->number_of_other_ECU < 255){
		j1939->other_ECU_address[j1939->number_of_other_ECU++] = SA;	/* For every new ECU address, count how many ECU */
	}
}
