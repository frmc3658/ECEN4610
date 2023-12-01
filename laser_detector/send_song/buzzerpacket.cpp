#include "buzzerpacket.h"


BuzzerPacket::BuzzerPacket(BuzzerNote noteToSend, int* durationOfNote)
{
    // Set GBLT header
    header = GBLT_HEADER;

    // Extract the 16-bit note
    note = (int)noteToSend & 0xFFFF;

    // Extract the 4-bit duration
    duration = *durationOfNote & 0xF;

    calculateChecksum();
}


BuzzerPacket::~BuzzerPacket()
{

}


void BuzzerPacket::calculateChecksum(void)
{
    // Simple polynomial addition checksum
    checksum = (header + note + duration) % (1 << 12);
}


bool BuzzerPacket::validateChecksum(void)
{
    // Calculate the checksum based on received data
    uint64_t calculatedChecksum = (header + note + duration) % (1 << 12);

    // Compare the calculated checksum with the stored checksum
    return checksum == (calculatedChecksum & 0xFFF);
}