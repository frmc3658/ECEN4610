#include "songpacket.h"

SongPacket::SongPacket(int* noteToSend, int* durationOfNote)
{
    // Set GBLT header
    header = GBLT_HEADER;

    // Extract the 16-bit note
    note = *noteToSend & 0xFFFF;

    // Extract the 4-bit duration
    duration = *durationOfNote & 0xF;

    calculateChecksum();
}

SongPacket::~SongPacket()
{

}

void SongPacket::calculateChecksum(void)
{
    checksum = (header + note + duration) % (1 << 12);
}

bool SongPacket::validateChecksum(void)
{
    // Calculate the checksum based on received data
    uint64_t calculatedChecksum = (header + note + duration) % (1 << 12);

    // Compare the calculated checksum with the stored checksum
    return checksum == calculatedChecksum;
}