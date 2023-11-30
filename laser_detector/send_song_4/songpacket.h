#ifndef SONGPACKET_H
#define SONGPACKET_H

#include <stdint.h>

#define GBLT_HEADER  0x47424C54  // "GBLT" in ASCII

#pragma pack(push, 1)

class SongPacket
{
    private:
        uint64_t header : 32;
        uint64_t note : 16;
        uint64_t duration : 4;
        uint64_t checksum : 12;

        void calculateChecksum(void);

    public:
        SongPacket(int* noteToSend, int* durationOfNote);
        ~SongPacket();

        bool validateChecksum(void);
};

#pragma pack(pop)

#endif /* SONGPACKET */