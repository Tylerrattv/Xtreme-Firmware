#include <furi.h>
#include <toolbox/protocols/protocol.h>
#include <lfrfid/tools/fsk_demod.h>
#include <lfrfid/tools/fsk_osc.h>
#include "lfrfid_protocols.h"
#include <lfrfid/tools/bit_lib.h>

#define JITTER_TIME (20)
#define MIN_TIME (64 - JITTER_TIME)
#define MAX_TIME (80 + JITTER_TIME)

// Constants for Pyramid Format
#define PYRAMID_DATA_SIZE 13
#define PYRAMID_PREAMBLE_SIZE 3
#define PYRAMID_ENCODED_DATA_SIZE (2 * PYRAMID_PREAMBLE_SIZE + PYRAMID_DATA_SIZE)
#define PYRAMID_ENCODED_BIT_SIZE (8 * PYRAMID_ENCODED_DATA_SIZE)
#define PYRAMID_DECODED_DATA_SIZE 4
#define PYRAMID_DECODED_BIT_SIZE (2 * (PYRAMID_ENCODED_BIT_SIZE - 8 * PYRAMID_PREAMBLE_SIZE))

// Constants for Pyramid Wiegand Format
#define PYRAMID_WIEGAND_ENCODED_DATA_SIZE 39
#define PYRAMID_WIEGAND_PREAMBLE_SIZE 3
#define PYRAMID_WIEGAND_ENCODED_BIT_SIZE (8 * (PYRAMID_WIEGAND_PREAMBLE_SIZE + PYRAMID_WIEGAND_ENCODED_DATA_SIZE))
#define PYRAMID_WIEGAND_DECODED_DATA_SIZE 2
#define PYRAMID_WIEGAND_DECODED_BIT_SIZE (8 * (PYRAMID_WIEGAND_ENCODED_DATA_SIZE - PYRAMID_WIEGAND_PREAMBLE_SIZE))

// Define named constants
enum PyramidFormat {
    PYRAMID_FORMAT_26 = 26,
    PYRAMID_FORMAT_39 = 39,
};

typedef struct {
    FSKOsc* fsk_osc;
    uint8_t encoded_index;
    uint32_t pulse;
} ProtocolPyramidEncoder;

typedef struct {
    FSKDemod* fsk_demod;
    uint8_t encoded_index;
    uint32_t pulse;
} ProtocolPyramidDecoder;

typedef struct {
    ProtocolPyramidDecoder decoder;
    ProtocolPyramidEncoder encoder;
    uint8_t encoded_data[PYRAMID_ENCODED_DATA_SIZE];
    uint8_t data[PYRAMID_DECODED_DATA_SIZE];
    bool is_39_bit_format;
} ProtocolPyramid;

// Function prototypes
static void protocol_pyramid_wiegand_encode(ProtocolPyramid* protocol);
static void set_data_size(ProtocolBase* protocol, uint8_t format);

// Allocate memory for ProtocolPyramid
ProtocolPyramid* protocol_pyramid_alloc(void) {
    ProtocolPyramid* protocol = malloc(sizeof(ProtocolPyramid));
    if (protocol) {
        protocol->decoder.fsk_demod = fsk_demod_alloc(MIN_TIME, 6, MAX_TIME, 5);
        protocol->encoder.fsk_osc = fsk_osc_alloc(8, 10, 50);
        set_data_size((ProtocolBase*)&protocol_pyramid, PYRAMID_FORMAT_26);
    }
    return protocol;
}

// Free memory allocated for ProtocolPyramid
void protocol_pyramid_free(ProtocolPyramid* protocol) {
    if (protocol) {
        fsk_demod_free(protocol->decoder.fsk_demod);
        fsk_osc_free(protocol->encoder.fsk_osc);
        free(protocol);
    }
}

// Get data from ProtocolPyramid
uint8_t* protocol_pyramid_get_data(ProtocolPyramid* protocol) {
    return protocol->data;
}

// Initialize the decoder
void protocol_pyramid_decoder_start(ProtocolPyramid* protocol) {
    memset(protocol->encoded_data, 0, PYRAMID_ENCODED_DATA_SIZE);
    protocol->is_39_bit_format = false; // Initialize the format flag
}

// Check if the data can be decoded in the Pyramid format
static bool protocol_pyramid_can_be_decoded(uint8_t* data, ProtocolPyramid* protocol) {
    // Check preamble as you currently do
    if (bit_lib_get_bits_16(data, 0, 16) != 0b0000000000000001 ||
        bit_lib_get_bits(data, 16, 8) != 0b00000001) {
        return false;
    }

    if (bit_lib_get_bits_16(data, 128, 16) != 0b0000000000000001 ||
        bit_lib_get_bits(data, 136, 8) != 0b00000001) {
        return false;
    }

    uint8_t checksum = bit_lib_get_bits(data, 120, 8);
    uint8_t checksum_data[13] = {0x00};
    for (uint8_t i = 0; i < 13; i++) {
        checksum_data[i] = bit_lib_get_bits(data, 16 + (i * 8), 8);
    }

    uint8_t calc_checksum = bit_lib_crc8(checksum_data, 13, 0x31, 0x00, true, true, 0x00);
    if (checksum != calc_checksum) return false;

    // Remove parity
    bit_lib_remove_bit_every_nth(data, 8, 15 * 8, 8);

    // Determine Startbit and format
    int j;
    for (j = 0; j < 105; ++j) {
        if (bit_lib_get_bit(data, j)) break;
    }
    uint8_t fmt_len = 105 - j;

    protocol->is_39_bit_format = (fmt_len == PYRAMID_FORMAT_39); // Set the format flag

    return true;
}

// Decode the data in the Pyramid format
static void protocol_pyramid_decode(ProtocolPyramid* protocol) {
    // Check if it's a 39-bit format
    if (protocol->is_39_bit_format) {
        // Handle decoding for 39-bit format
        uint32_t facility = bit_lib_get_bits_32(protocol->encoded_data, 8, 17);
        uint32_t card_id = bit_lib_get_bits_32(protocol->encoded_data, 26, 20);

        // Set the format in the decoded data
        bit_lib_set_bits(protocol->data, 0, PYRAMID_FORMAT_39, 8);
        bit_lib_set_bits_32(protocol->data, 8, facility, 17);
        bit_lib_set_bits_32(protocol->data, 26, card_id, 20);
    } else {
        // Handle decoding for 26-bit format
        // Format
        bit_lib_set_bits(protocol->data, 0, PYRAMID_FORMAT_26, 8);

        // Facility Code
        bit_lib_copy_bits(protocol->data, 8, 8, protocol->encoded_data, 73 + 8);

        // Card Number
        bit_lib_copy_bits(protocol->data, 16, 16, protocol->encoded_data, 81 + 8);
    }
}

// Feed data to the decoder
bool protocol_pyramid_decoder_feed(ProtocolPyramid* protocol, bool level, uint32_t duration) {
    bool value;
    uint32_t count;
    bool result = false;

    fsk_demod_feed(protocol->decoder.fsk_demod, level, duration, &value, &count);
    if (count > 0) {
        for (size_t i = 0; i < count; i++) {
            bit_lib_push_bit(protocol->encoded_data, PYRAMID_ENCODED_DATA_SIZE, value);
            if (protocol_pyramid_can_be_decoded(protocol->encoded_data, protocol)) {
                protocol_pyramid_decode(protocol);
                result = true;
            }
        }
    }

    return result;
}

// Calculate parity for a given set of bits
bool protocol_pyramid_get_parity(const uint8_t* bits, uint8_t type, int length) {
    int x = 0;
    for (int i = 0; i < length; i++) {
        x += bit_lib_get_bit(bits, i);
    }
    x %= 2;
    return x ^ type;
}

// Add parity to the target bits
void protocol_pyramid_add_wiegand_parity(
    uint8_t* target,
    uint8_t target_position,
    uint8_t* source,
    uint8_t length) {
    bit_lib_set_bit(
        target, target_position, protocol_pyramid_get_parity(source, 0 /* even */, length / 2));
    bit_lib_copy_bits(target, target_position + 1, length, source, 0);
    bit_lib_set_bit(
        target,
        target_position + length + 1,
        protocol_pyramid_get_parity(source + length / 2, 1 /* odd */, length / 2));
}

// Encode data in the Pyramid format
static void protocol_pyramid_encode(ProtocolPyramid* protocol) {
    memset(protocol->encoded_data, 0, sizeof(protocol->encoded_data));

    if (protocol->is_39_bit_format) {
        // Encode Pyramid Wiegand 39-bit format
        protocol_pyramid_wiegand_encode(protocol);
    } else {
        // Encode Pyramid 26-bit format
        uint8_t pre[16];
        memset(pre, 0, sizeof(pre));

        // Format start bit
        bit_lib_set_bit(pre, 79, 1);

        uint8_t wiegand[3];
        memset(wiegand, 0, sizeof(wiegand));

        // Facility Code
        bit_lib_copy_bits(wiegand, 0, 8, protocol->data, 8);

        // Card Number
        bit_lib_copy_bits(wiegand, 8, 16, protocol->data, 16);

        // Wiegand parity
        protocol_pyramid_add_wiegand_parity(pre, 80, wiegand, 24);

        bit_lib_add_parity(pre, 8, protocol->encoded_data, 8, 102, 8, 1);

        // Add checksum
        uint8_t checksum_buffer[13];
        for (uint8_t i = 0; i < 13; i++) {
            checksum_buffer[i] = bit_lib_get_bits(protocol->encoded_data, 16 + (i * 8), 8);
        }

        uint8_t crc = bit_lib_crc8(checksum_buffer, 13, 0x31, 0x00, true, true, 0x00);
        bit_lib_set_bits(protocol->encoded_data, 120, crc, 8);
    }
}

// Encode data in the Pyramid Wiegand format
static void protocol_pyramid_wiegand_encode(ProtocolPyramid* protocol) {
    memset(protocol->encoded_data, 0, sizeof(protocol->encoded_data));

    uint8_t pre[29];
    memset(pre, 0, sizeof(pre));

    // Format start bit
    bit_lib_set_bit(pre, 36, 1);

    // Facility Code
    bit_lib_copy_bits(pre, 37, 17, protocol->data, 8);

    // Card Number
    bit_lib_copy_bits(pre, 54, 20, protocol->data, 26);

    // Even Parity
    uint8_t even_parity = protocol_pyramid_get_parity(pre, 0, 19);
    bit_lib_set_bit(pre, 38, even_parity);

    // Odd Parity
    uint8_t odd_parity = protocol_pyramid_get_parity(pre + 19, 1, 19);
    bit_lib_set_bit(pre, 39, odd_parity);

    bit_lib_copy_bits(protocol->encoded_data, 0, 39, pre, 0);
}

// Start the encoder
bool protocol_pyramid_encoder_start(ProtocolPyramid* protocol) {
    protocol->encoder.encoded_index = 0;
    protocol->encoder.pulse = 0;
    protocol_pyramid_encode(protocol);
    return true;
}

// Yield the level and duration for encoding
LevelDuration protocol_pyramid_encoder_yield(ProtocolPyramid* protocol) {
    bool level = 0;
    uint32_t duration = 0;

    // If the pulse is zero, we need to output high, otherwise we need to output low
    if (protocol->encoder.pulse == 0) {
        // Get bit
        uint8_t bit = bit_lib_get_bit(protocol->encoded_data, protocol->encoder.encoded_index);

        // Get pulse from oscillator
        bool advance = fsk_osc_next(protocol->encoder.fsk_osc, bit, &duration);

        if (advance) {
            bit_lib_increment_index(protocol->encoder.encoded_index, PYRAMID_ENCODED_BIT_SIZE);
        }

        // Duration divided by 2 because we need to output high and low
        duration = duration / 2;
        protocol->encoder.pulse = duration;
        level = true;
    } else {
        // Output low half and reset pulse
        duration = protocol->encoder.pulse;
        protocol->encoder.pulse = 0;
        level = false;
    }

    return level_duration_make(level, duration);
}

// Write data to the encoder
bool protocol_pyramid_write_data(ProtocolPyramid* protocol, void* data) {
    LFRFIDWriteRequest* request = (LFRFIDWriteRequest*)data;
    bool result = false;

    // Correct protocol data by redecoding
    protocol_pyramid_encode(protocol);
    bit_lib_remove_bit_every_nth(protocol->encoded_data, 8, 15 * 8, 8);
    protocol_pyramid_decode(protocol);

    protocol_pyramid_encoder_start(protocol);

    if (request->write_type == LFRFIDWriteTypeT5577) {
        request->t5577.block[0] = LFRFID_T5577_MODULATION_FSK2a | LFRFID_T5577_BITRATE_RF_50 |
                                (4 << LFRFID_T5577_MAXBLOCK_SHIFT);
        request->t5577.block[1] = bit_lib_get_bits_32(protocol->encoded_data, 0, 32);
        request->t5577.block[2] = bit_lib_get_bits_32(protocol->encoded_data, 32, 32);
        request->t5577.block[3] = bit_lib_get_bits_32(protocol->encoded_data, 64, 32);
        request->t5577.block[4] = bit_lib_get_bits_32(protocol->encoded_data, 96, 32);
        request->t5577.blocks_to_write = 5;
        result = true;
    }
    return result;
}

// Render data to a string
void protocol_pyramid_render_data(ProtocolPyramid* protocol, FuriString* result) {
    uint8_t* decoded_data = protocol->data;
    uint8_t format_length = decoded_data[0];

    furi_string_cat_printf(result, "Format: %d\r\n", format_length);

    if (format_length == PYRAMID_FORMAT_26) {
        uint8_t facility;
        bit_lib_copy_bits(&facility, 0, 8, decoded_data, 8);

        uint16_t card_id;
        bit_lib_copy_bits((uint8_t*)&card_id, 8, 8, decoded_data, 16);
        bit_lib_copy_bits((uint8_t*)&card_id, 0, 8, decoded_data, 24);
        furi_string_cat_printf(
            result, "FC: %lu, Card: %lu", (unsigned long)facility, (unsigned long)card_id);
    } else if (format_length == PYRAMID_FORMAT_39) {
        // Handle Pyramid Wiegand 39-bit format
        uint32_t facility = bit_lib_get_bits_32(decoded_data, 8, 17);
        uint32_t card_id = bit_lib_get_bits_32(decoded_data, 26, 20);
        furi_string_cat_printf(
            result, "FC: %lu, Card: %lu", (unsigned long)facility, (unsigned long)card_id);
    } else {
        furi_string_cat_printf(result, "Data: unknown");
    }
}
// Place this function definition near the ProtocolBase structure definition.
void set_data_size(ProtocolBase *protocol, uint8_t format) {
    if (format == PYRAMID_FORMAT_26) {
        protocol->data_size = PYRAMID_DECODED_DATA_SIZE; // 26-bit format
    } else if (format == PYRAMID_FORMAT_39) {
        protocol->data_size = PYRAMID_WIEGAND_DECODED_DATA_SIZE; // 39-bit format
    } else {
        // Handle unsupported formats or default to one of them
        protocol->data_size = PYRAMID_DECODED_DATA_SIZE;
    }
}

// Initialize your ProtocolBase and set the data size based on the format in your protocol initialization function.
ProtocolPyramid* protocol_pyramid_alloc(void) {
    ProtocolPyramid* protocol = malloc(sizeof(ProtocolPyramid));
    protocol->decoder.fsk_demod = fsk_demod_alloc(MIN_TIME, 6, MAX_TIME, 5);
    protocol->encoder.fsk_osc = fsk_osc_alloc(8, 10, 50);
    
    // Example usage to set the format and data size (You can change the format as needed)
    uint8_t format = PYRAMID_FORMAT_26; // Change this to PYRAMID_FORMAT_39 for 39-bit format
    set_data_size((ProtocolBase*)&protocol_pyramid, format);
    
    return protocol;
}

const ProtocolBase protocol_pyramid = {
    .name = "Pyramid",
    .manufacturer = "Farpointe",
    .features = LFRFIDFeatureASK,
    .validate_count = 3,
    .alloc = (ProtocolAlloc)protocol_pyramid_alloc,
    .free = (ProtocolFree)protocol_pyramid_free,
    .get_data = (ProtocolGetData)protocol_pyramid_get_data,
    .decoder =
        {
            .start = (ProtocolDecoderStart)protocol_pyramid_decoder_start,
            .feed = (ProtocolDecoderFeed)protocol_pyramid_decoder_feed,
        },
    .encoder =
        {
            .start = (ProtocolEncoderStart)protocol_pyramid_encoder_start,
            .yield = (ProtocolEncoderYield)protocol_pyramid_encoder_yield,
        },
    .render_data = (ProtocolRenderData)protocol_pyramid_render_data,
    .render_brief_data = (ProtocolRenderData)protocol_pyramid_render_data,
    .write_data = (ProtocolWriteData)protocol_pyramid_write_data,
};

void set_data_size(ProtocolBase* protocol, uint8_t format) {
    if (format == PYRAMID_FORMAT_26) {
        protocol->data_size = PYRAMID_DECODED_DATA_SIZE; // 26-bit format
    } else if (format == PYRAMID_FORMAT_39) {
        protocol->data_size = PYRAMID_WIEGAND_DECODED_DATA_SIZE; // 39-bit format
    } else {
        // Handle unsupported formats or default to one of them
        protocol->data_size = PYRAMID_DECODED_DATA_SIZE;
    }
}

