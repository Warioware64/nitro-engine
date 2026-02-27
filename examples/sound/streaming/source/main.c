// SPDX-License-Identifier: CC0-1.0
//
// SPDX-FileContributor: Warioware64, 2026
//
// This file is part of Nitro Engine Advanced
//
// WAV streaming example: demonstrates NEA_StreamOpen() and NEA_StreamClose()
// by streaming a WAV file from NitroFS through a circular buffer. Based on
// the BlocksDS maxmod streaming example.

#include <stdio.h>
#include <string.h>

#include <filesystem.h>
#include <maxmod9.h>
#include <nds.h>

#include <NEAMain.h>

#define DATA_ID 0x61746164
#define FMT_ID  0x20746d66
#define RIFF_ID 0x46464952
#define WAVE_ID 0x45564157

typedef struct WAVHeader {
    uint32_t chunkID;
    uint32_t chunkSize;
    uint32_t format;
    uint32_t subchunk1ID;
    uint32_t subchunk1Size;
    uint16_t audioFormat;
    uint16_t numChannels;
    uint32_t sampleRate;
    uint32_t byteRate;
    uint16_t blockAlign;
    uint16_t bitsPerSample;
    uint32_t subchunk2ID;
    uint32_t subchunk2Size;
} WAVHeader_t;

#define BUFFER_LENGTH 16384

static FILE *wavFile = NULL;
static char stream_buffer[BUFFER_LENGTH];
static int stream_buffer_in;
static int stream_buffer_out;

static mm_word streamingCallback(mm_word length,
                                 mm_addr dest,
                                 mm_stream_formats format)
{
    size_t multiplier = 0;

    if (format == MM_STREAM_8BIT_MONO)
        multiplier = 1;
    else if (format == MM_STREAM_8BIT_STEREO)
        multiplier = 2;
    else if (format == MM_STREAM_16BIT_MONO)
        multiplier = 2;
    else if (format == MM_STREAM_16BIT_STEREO)
        multiplier = 4;

    size_t size = length * multiplier;
    size_t bytes_until_end = BUFFER_LENGTH - stream_buffer_out;

    if (bytes_until_end > size)
    {
        memcpy(dest, &stream_buffer[stream_buffer_out], size);
        stream_buffer_out += size;
    }
    else
    {
        char *dst = dest;
        memcpy(dst, &stream_buffer[stream_buffer_out], bytes_until_end);
        dst += bytes_until_end;
        size -= bytes_until_end;
        memcpy(dst, &stream_buffer[0], size);
        stream_buffer_out = size;
    }

    return length;
}

static void readFile(char *buffer, size_t size)
{
    while (size > 0)
    {
        int res = fread(buffer, 1, size, wavFile);
        size -= res;
        buffer += res;

        if (feof(wavFile))
        {
            fseek(wavFile, sizeof(WAVHeader_t), SEEK_SET);
            res = fread(buffer, 1, size, wavFile);
            size -= res;
            buffer += res;
            printf("Restarting...\n");
        }
    }
}

static void streamingFillBuffer(bool force_fill)
{
    if (!force_fill)
    {
        if (stream_buffer_in == stream_buffer_out)
            return;
    }

    if (stream_buffer_in < stream_buffer_out)
    {
        size_t size = stream_buffer_out - stream_buffer_in;
        readFile(&stream_buffer[stream_buffer_in], size);
        stream_buffer_in += size;
    }
    else
    {
        size_t size = BUFFER_LENGTH - stream_buffer_in;
        readFile(&stream_buffer[stream_buffer_in], size);
        stream_buffer_in = 0;

        size = stream_buffer_out - stream_buffer_in;
        readFile(&stream_buffer[stream_buffer_in], size);
        stream_buffer_in += size;
    }

    if (stream_buffer_in >= BUFFER_LENGTH)
        stream_buffer_in -= BUFFER_LENGTH;
}

static int checkWAVHeader(const WAVHeader_t header)
{
    if (header.chunkID != RIFF_ID)
    {
        printf("Wrong RIFF_ID %lx\n", header.chunkID);
        return 1;
    }
    if (header.format != WAVE_ID)
    {
        printf("Wrong WAVE_ID %lx\n", header.format);
        return 1;
    }
    if (header.subchunk1ID != FMT_ID)
    {
        printf("Wrong FMT_ID %lx\n", header.subchunk1ID);
        return 1;
    }
    if (header.subchunk2ID != DATA_ID)
    {
        printf("Wrong Subchunk2ID %lx\n", header.subchunk2ID);
        return 1;
    }
    return 0;
}

static mm_stream_formats getMMStreamType(uint16_t numChannels,
                                         uint16_t bitsPerSample)
{
    if (numChannels == 1)
    {
        if (bitsPerSample == 8)
            return MM_STREAM_8BIT_MONO;
        else
            return MM_STREAM_16BIT_MONO;
    }
    else if (numChannels == 2)
    {
        if (bitsPerSample == 8)
            return MM_STREAM_8BIT_STEREO;
        else
            return MM_STREAM_16BIT_STEREO;
    }
    return MM_STREAM_8BIT_MONO;
}

int main(int argc, char *argv[])
{
    irqEnable(IRQ_HBLANK);
    irqSet(IRQ_VBLANK, NEA_VBLFunc);
    irqSet(IRQ_HBLANK, NEA_HBLFunc);

    NEA_Init3D();

    NEA_TextureSystemReset(0, 0, NEA_VRAM_AB);
    consoleDemoInit();

    printf("WAV Streaming Example\n");
    printf("=====================\n\n");

    bool initOK = nitroFSInit(NULL);
    if (!initOK)
    {
        printf("nitroFSInit failed!\n");
        while (1)
            swiWaitForVBlank();
    }

    wavFile = fopen("nitro:/AuldLangSyne.wav", "rb");
    if (wavFile == NULL)
    {
        printf("Could not open WAV file!\n");
        while (1)
            swiWaitForVBlank();
    }

    WAVHeader_t wavHeader = { 0 };
    if (fread(&wavHeader, 1, sizeof(WAVHeader_t), wavFile) != sizeof(WAVHeader_t))
    {
        printf("Failed to read WAV header!\n");
        while (1)
            swiWaitForVBlank();
    }
    if (checkWAVHeader(wavHeader) != 0)
    {
        printf("WAV header is corrupt!\n");
        while (1)
            swiWaitForVBlank();
    }

    printf("Format: %d ch, %ld Hz, %d bit\n",
           wavHeader.numChannels,
           wavHeader.sampleRate,
           wavHeader.bitsPerSample);
    printf("\n");

    // Fill the circular buffer before starting
    streamingFillBuffer(true);

    // Initialize maxmod manually (no soundbank)
    mm_ds_system mmSys = {
        .mod_count    = 0,
        .samp_count   = 0,
        .mem_bank     = 0,
        .fifo_channel = FIFO_MAXMOD
    };
    mmInit(&mmSys);

    // Initialize NEA sound system pool only (maxmod already inited above)
    NEA_SoundSystemResetPool(1);

    // Open the stream using NEA wrapper
    mm_stream_formats fmt = getMMStreamType(wavHeader.numChannels,
                                            wavHeader.bitsPerSample);
    NEA_StreamOpen(wavHeader.sampleRate, 2048, streamingCallback,
                   fmt, MM_TIMER0);

    printf("Streaming WAV...\n\n");
    printf("START: Return to loader\n");

    while (1)
    {
        NEA_WaitForVBL(0);

        // Keep the circular buffer filled
        streamingFillBuffer(false);

        printf("\x1b[12;0HPosition: %u samples\n",
               NEA_StreamGetPosition());

        scanKeys();
        uint16_t keys_down = keysDown();

        if (keys_down & KEY_START)
            break;
    }

    NEA_StreamClose();
    NEA_SoundSystemEnd();

    if (fclose(wavFile) != 0)
        printf("fclose failed\n");

    return 0;
}
