#ifndef BUZZERPACKET_H
#define BUZZERPACKET_H

#include <stdint.h>
#include "buzzernote.h"

#define GBLT_HEADER  0x47424C54  // "GBLT" in ASCII

#pragma pack(push, 1)
class BuzzerPacket
{
    private:
        uint64_t header : 32;
        uint64_t note : 16;
        uint64_t duration : 4;
        uint64_t checksum : 12;

        void calculateChecksum(void);

    public:
        BuzzerPacket(BuzzerNote noteToSend, int* durationOfNote);
        ~BuzzerPacket();

        bool validateChecksum(void);
};
#pragma pack(pop)

#endif /* BUZZERPACKET */