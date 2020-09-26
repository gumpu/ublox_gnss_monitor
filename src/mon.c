#include <sys/types.h>
#include <sys/stat.h>
#include <stdint.h>
#include <fcntl.h>
#include <termios.h>
#include <ctype.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <assert.h>

#define RATE_FAST   (1)
#define RATE_NORMAL (2)
#define RATE_SLOW   (3)

char* rate_string[4] = {
    "none",
    "fast",
    "normal",
    "slow"
};

typedef struct NMEA_Info {
    char name[4];
    uint8_t class;
    uint8_t id;
} NMEA_Info;

#define DTM 0
#define GBS 1
#define GGA 2
#define GLL 3
#define GLQ 4
#define GNQ 5
#define GNS 6
#define GPQ 7
#define GRS 8
#define GSA 9
#define GST 10
#define GSV 11
#define RMC 12
#define TXT 13
#define VTG 14
#define ZDA 15
#define LOOKUP_TABLE_SIZE 16

NMEA_Info nmea_lookup_table[LOOKUP_TABLE_SIZE] = {
    { "DTM",0xF0,0x0A }, /* Datum Reference */
    { "GBS",0xF0,0x09 }, /* GNSS Satellite Fault Detection */
    { "GGA",0xF0,0x00 }, /* Global positioning system fix data */
    { "GLL",0xF0,0x01 }, /* Latitude and longitude, with time of position fix and status */
    { "GLQ",0xF0,0x43 }, /* Poll a standard message (if the current Talker ID is GL) */
    { "GNQ",0xF0,0x42 }, /* Poll a standard message (if the current Talker ID is GN) */
    { "GNS",0xF0,0x0D }, /* GNSS fix data */
    { "GPQ",0xF0,0x40 }, /* Poll a standard message (if the current Talker ID is GP) */
    { "GRS",0xF0,0x06 }, /* GNSS Range Residuals */
    { "GSA",0xF0,0x02 }, /* GNSS DOP and Active Satellites */
    { "GST",0xF0,0x07 }, /* GNSS Pseudo Range Error Statistics */
    { "GSV",0xF0,0x03 }, /* GNSS Satellites in View */
    { "RMC",0xF0,0x04 }, /* Recommended Minimum data */
    { "TXT",0xF0,0x41 }, /* Text Transmission */
    { "VTG",0xF0,0x05 }, /* Course over ground and Ground speed */
    { "ZDA",0xF0,0x08 }, /* Time and Date */
};

#define BAUDRATE B230400
/* #define MODEMDEVICE "/dev/usbch1" */
#define MODEMDEVICE "/dev/ttyACM0"
#define _POSIX_SOURCE 1 /* POSIX compliant source */
#define FALSE 0
#define TRUE 1

// Buffer big enough for about half an hour's worth of data
// This way we write only every half hour to disk, and
// can use an SD card (raspberrypi) to store the data
#define LOG_BUFFER_SIZE (80*8*1800)

#define INPUT_BUFFER_SIZE (1024)
#define SENTENCE_BUFFER_SIZE (1024)

FILE* g_log_file = NULL;
enum MessageKind {
    NMEA = 1,
    UBX =  2,
    Undefined = 5
};

enum MessageState {
    empty = 1,
    waiting_for_more = 2,
    complete = 3,
};

#define MAX_UBX_DATA_LENGTH (1024)

typedef struct UBX_Message {
    uint8_t sync_char1;
    uint8_t sync_char2;
    uint8_t class;
    uint8_t id;
    uint16_t length;
    uint8_t body[MAX_UBX_DATA_LENGTH];
} UBX_Message;

typedef struct Message {
    enum MessageKind kind;
    enum MessageState state;
    char buffer[SENTENCE_BUFFER_SIZE];
    uint16_t current_position;
    uint16_t expected_length;
} Message;

typedef struct CFG_MSG_Body {
    uint8_t msgClass;
    uint8_t msgID;
    uint8_t rate;
} CFG_MSG_Body;

typedef struct CFG_RATE_Body {
    uint16_t measRate;
    uint16_t navRate;
    uint16_t timeRef;
} CFG_RATE_Body;

typedef struct CFG_PRT_Poll_Body {
    uint8_t PortID;
} CFG_PRT_Poll_Body;

typedef struct CFG_PRT_Body {
    uint8_t portID;
    uint8_t reserved0;
    uint16_t txReady;
    uint32_t mode;
    uint32_t baudRate;
    uint16_t inProtoMask;
    uint16_t outProtoMask;
    uint16_t flags;
    uint16_t reserved5;
} CFG_PRT_Body;


/* --------------------------------------------------------------------*/

typedef struct Stack_Element {
    UBX_Message message;
    int size;
} Stack_Element;

#define MAX_UBX_STACK_SIZE (100)
typedef struct UBX_Message_Stack {
    uint16_t n;
    Stack_Element elements[MAX_UBX_STACK_SIZE];
} UBX_Message_Stack;

/* --------------------------------------------------------------------*/

volatile int STOP=FALSE;
void signal_handler(int dummy) {
    STOP=TRUE;
}


/* --------------------------------------------------------------------*/

void log_nmea_string(char* nmea_string)
{
    fprintf(g_log_file, "%s", nmea_string);
}

void dump_prt_config(CFG_PRT_Body* prt)
{
    printf("Port %d ", prt->portID);
    printf("txReady %d ", prt->txReady);
    printf("Mode %x ", prt->mode);
    printf("Baud %d ", prt->baudRate);
    if (prt->inProtoMask & 0x01) {
        printf("inUBX ");
    }
    if (prt->inProtoMask & 0x02) {
        printf("inNmea ");
    }
    if (prt->inProtoMask & 0x03) {
        printf("inRtcm ");
    }
    if (prt->outProtoMask & 0x01) {
        printf("outUBX ");
    }
    if (prt->outProtoMask & 0x02) {
        printf("outNmea ");
    }
    printf("Flags %x ", prt->mode);
    printf("\n");
}

void log_ubx_message(UBX_Message* m)
{
    int i;
    fprintf(g_log_file, "%d %d ", m->class, m->id);
    fprintf(g_log_file, "%d: ", m->length);
    for (i = 0; i < m->length; ++i) {
        fprintf(g_log_file, "%02x ", m->body[i]);
    }
    fprintf(g_log_file, "\n");
}

void parse_ubx(UBX_Message* m)
{
    log_ubx_message(m);
    switch (m->class) {
        case 0x05:
            {
                switch(m->id) {
                    case 0x00:
                        printf("ack nak\n");
                        break;
                    case 0x01:
                        printf("ack ack\n");
                        break;
                    default:
                        break;
                }
            }
        case 0x06:
            {
                switch(m->id) {
                    case 0x00:
                        printf("CFG-PRT\n");
                        dump_prt_config((void*)(&(m->body[0])));
                    case 0x3E:
                        printf("CFG-GNSS\n");
                        break;
                    case 0x24:
                        printf("Navigation engine settings\n");
                        break;
                    case 0x23:
                        printf("Navigation engine expert settings\n");
                        printf("ppp %d\n", m->body[26]);
                        break;
                    case 0x06:
                        printf("DAT Settings\n");
                        break;
                    case 0x08:
                        printf("Rate Settings\n");
                        break;
                    default:
                        break;
                }
            }
        default:
            break;
    }
}


void compute_checksum(const uint8_t* buffer, uint16_t n, uint8_t *ck_a, uint8_t* ck_b)
{
    uint16_t i;
    uint8_t CK_A = 0;
    uint8_t CK_B = 0;
    for (i = 0; i < n; i++)
    {
        CK_A = CK_A + buffer[i];
        CK_B = CK_B + CK_A;
    }
    *ck_a = CK_A;
    *ck_b = CK_B;
}

/* Return the number of bytes needed to send the full message */
uint16_t create_ubx_message(
        UBX_Message* m, uint8_t class, uint8_t id, char* body, uint16_t length)
{
    m->sync_char1 = 0xB5;
    m->sync_char2 = 0x62;
    m->class = class;
    m->id = id;
    m->length = length;
    if (length > 0) {
        memcpy(m->body, body, length);
    }
    uint8_t* data = (uint8_t*)m;
    /* Checksum is over body plus class, id and length */
    compute_checksum(&(data[2]), length + 4, &(m->body[length]), &(m->body[length+1]));
    /* total length = |Header| + |body| + |checksum| */
    return (6U + length + 2U);
}

void init_message(Message* m)
{
    m->kind = Undefined;
    m->state = empty;
    m->buffer[0] = 0;
    m->current_position = 0U;
    m->expected_length = 0U;
}


/* --------------------------------------------------------------------*/

void init_ubx_message_stack(UBX_Message_Stack* ubx_messages)
{
    uint32_t i;
    ubx_messages->n = 0;
    for (i = 0; i < MAX_UBX_STACK_SIZE; i++) {
        bzero(&(ubx_messages->elements[i]), sizeof(Stack_Element));
    }
}

void ubx_message_push(
        UBX_Message_Stack* ubx_messages,
        uint8_t class, uint8_t id, char* body, uint16_t length,
        char* label)
{
    assert(ubx_messages->n < MAX_UBX_STACK_SIZE);
    uint16_t i;
    uint16_t n = ubx_messages->n;
    uint8_t* p;

    ubx_messages->elements[n].size = create_ubx_message(
            &(ubx_messages->elements[n].message), class, id, body, length);

    p = (uint8_t*)(&(ubx_messages->elements[n].message));
    printf("%s ", label);
    for (i = 0; i < ubx_messages->elements[n].size; i++) {
        printf("%02x ", p[i]);
    }
    printf("\n");

    ubx_messages->n = n + 1;
}

Stack_Element* ubx_message_pop(UBX_Message_Stack* ubx_messages)
{
    Stack_Element* result = NULL;
    if (ubx_messages->n > 0) {
        uint16_t n = ubx_messages->n;
        n--;
        result = &(ubx_messages->elements[n]);
        ubx_messages->n = n;
    }
    return result;
}

void queue_messages(UBX_Message_Stack* messages, int rate)
{
    CFG_RATE_Body cfg_rate;
    CFG_MSG_Body  msg;
    CFG_PRT_Poll_Body prt;
    CFG_PRT_Body prt_config;
    uint8_t rmc_rate = 1;
    uint8_t gsv_rate = 1;
    uint8_t other_rate = 1;

    ubx_message_push(messages, 0x06, 0x08, NULL, 0, "poll_rate"); /* CFG-RATE */

    switch(rate) {
        case RATE_NORMAL:
            cfg_rate.measRate = 1000; /* ms */
            other_rate = 1;
            gsv_rate = 1;
            break;
        case RATE_FAST:
            cfg_rate.measRate = 200; /* ms */
            gsv_rate = 0;
            other_rate = 0;
            break;
        case RATE_SLOW:
            cfg_rate.measRate = 5000; /* ms */
            other_rate = 1;
            gsv_rate = 1;
            break;
        default:
            assert(0);
    }
    cfg_rate.navRate  = 1; /* Always 1 */
    cfg_rate.timeRef  = 0; /* UTC */
    ubx_message_push(messages, 0x06, 0x08, (char*)(&cfg_rate), sizeof(CFG_RATE_Body), "set_rate");

    msg.msgClass = nmea_lookup_table[RMC].class;
    msg.msgID    = nmea_lookup_table[RMC].id;
    msg.rate     = rmc_rate;
    ubx_message_push(messages, 0x06, 0x01, (char*)(&msg), sizeof(CFG_MSG_Body), "set_rmc_rate");
    msg.msgClass = nmea_lookup_table[GSV].class;
    msg.msgID    = nmea_lookup_table[GSV].id;
    msg.rate     = gsv_rate;
    ubx_message_push(messages, 0x06, 0x01, (char*)(&msg), sizeof(CFG_MSG_Body), "set_gsv_rate");
    msg.msgClass = nmea_lookup_table[GGA].class;
    msg.msgID    = nmea_lookup_table[GGA].id;
    msg.rate     = rmc_rate;
    ubx_message_push(messages, 0x06, 0x01, (char*)(&msg), sizeof(CFG_MSG_Body), "set_gga_rate");
    msg.msgClass = nmea_lookup_table[GSA].class;
    msg.msgID    = nmea_lookup_table[GSA].id;
    msg.rate     = other_rate;
    ubx_message_push(messages, 0x06, 0x01, (char*)(&msg), sizeof(CFG_MSG_Body), "set_gsa_rate");
    msg.msgClass = nmea_lookup_table[VTG].class;
    msg.msgID    = nmea_lookup_table[VTG].id;
    msg.rate     = other_rate;
    ubx_message_push(messages, 0x06, 0x01, (char*)(&msg), sizeof(CFG_MSG_Body), "set_vtg_rate");
    msg.msgClass = nmea_lookup_table[GLL].class;
    msg.msgID    = nmea_lookup_table[GLL].id;
    msg.rate     = other_rate;
    ubx_message_push(messages, 0x06, 0x01, (char*)(&msg), sizeof(CFG_MSG_Body), "set_gll_rate");

    ubx_message_push(messages, 0x06, 0x24, NULL, 0, "poll_nav5"); /* CFG-NAV5 */
    ubx_message_push(messages, 0x06, 0x23, NULL, 0, "poll_navx5"); /* CFG-NAVX5 */
    ubx_message_push(messages, 0x06, 0x3E, NULL, 0, "poll_gnss"); /* CFG-GNSS */
    ubx_message_push(messages, 0x06, 0x06, NULL, 0, "poll_dat"); /* CFG-DAT */

    prt.PortID = 0;
    ubx_message_push(messages, 0x06, 0x00, (char*)(&prt), sizeof(CFG_PRT_Poll_Body), "poll_port_0");
    prt.PortID = 1;
    ubx_message_push(messages, 0x06, 0x00, (char*)(&prt), sizeof(CFG_PRT_Poll_Body), "poll_port_1");
    prt.PortID = 3;
    ubx_message_push(messages, 0x06, 0x00, (char*)(&prt), sizeof(CFG_PRT_Poll_Body), "poll_port_3");
    prt.PortID = 4;
    ubx_message_push(messages, 0x06, 0x00, (char*)(&prt), sizeof(CFG_PRT_Poll_Body), "poll_port_4");

    prt_config.portID = 1;
    prt_config.txReady = 0;
    prt_config.mode = 0x8c0;
    prt_config.baudRate = 9600;
    prt_config.inProtoMask = 0;
    prt_config.outProtoMask = 0;
    prt_config.flags = 0x8c0;
    ubx_message_push(messages, 0x06, 0x00, (char*)(&prt_config), sizeof(CFG_PRT_Body), "config_port_1");

    prt_config.portID = 0;
    prt_config.txReady = 0;
    prt_config.mode = 0x0;
    prt_config.baudRate = 0;
    prt_config.inProtoMask = 0;
    prt_config.outProtoMask = 0;
    prt_config.flags = 0x0;
    ubx_message_push(messages, 0x06, 0x00, (char*)(&prt_config), sizeof(CFG_PRT_Body), "config_port_0");

    prt_config.portID = 4;
    prt_config.txReady = 0;
    prt_config.mode = 0x0;
    prt_config.baudRate = 0;
    prt_config.inProtoMask = 0;
    prt_config.outProtoMask = 0;
    prt_config.flags = 0x0;
    ubx_message_push(messages, 0x06, 0x00, (char*)(&prt_config), sizeof(CFG_PRT_Body), "config_port_4");

    cfg_rate.measRate = 1000; /* Every 1 seconds */
    cfg_rate.navRate  = 1;
    cfg_rate.timeRef  = 0; /* UTC */
    ubx_message_push(messages, 0x06, 0x08, (char*)(&cfg_rate), sizeof(CFG_RATE_Body), "set_rate_to_1");

    msg.msgClass = nmea_lookup_table[RMC].class;
    msg.msgID    = nmea_lookup_table[RMC].id;
    msg.rate     = 1;
    ubx_message_push(messages, 0x06, 0x01, (char*)(&msg), sizeof(CFG_MSG_Body), "set_rmc_rate_to_1");
}

/* --------------------------------------------------------------------*/

void parse(uint8_t* input_buffer, int n, Message* message)
{
    int i;

    for (i = 0; i < n; ++i) {
        uint8_t c = input_buffer[i];
        if (message->current_position > SENTENCE_BUFFER_SIZE - 10) {
            printf("Communication error 3\n");
            fprintf(g_log_file, "Err3:");
            init_message(message);
        }
        if (message->state == waiting_for_more) {
            if (message->kind == NMEA) {
                if (c == 0x0A) {
                    message->buffer[message->current_position] = c;
                    (message->current_position)++;
                    message->buffer[message->current_position] = 0;

                    /* Deal with full message */
                    log_nmea_string(message->buffer);

                    /* Now we are ready for the next one */
                    init_message(message);
                } else if (c == (uint8_t)'$') {
                    uint32_t i;
                    printf("Communication error 1\n");
                    fprintf(g_log_file, "Err1:");
                    for (i = 0; i < message->current_position; ++i) {
                        fprintf(g_log_file, "%02x ", message->buffer[i]);
                    }
                    fprintf(g_log_file, "\n");
                    /* Reset */
                    init_message(message);
                } else {
                    message->buffer[message->current_position] = c;
                    (message->current_position)++;
                }
            } else if (message->kind == UBX) {
                message->buffer[message->current_position] = c;
                (message->current_position)++;
                if (message->current_position == 2) {
                    if (message->buffer[1] != 'b') {
                        printf("Communication error 2\n");
                        fprintf(g_log_file, "Err2:");
                        init_message(message);
                    }
                } else if (message->current_position == 6) {
                    uint16_t* length;
                    /* Bytes 4 and 5 contain the length in little endian
                     * format */
                    length = (uint16_t*)(&(message->buffer[4]));
                    message->expected_length = (*length) + 8U;
                    // printf("UBX Length %d\n", message->expected_length);
                }
                if (message->current_position == message->expected_length) {
                    // printf("Got a full UBX message\n");
                    UBX_Message* ubx_message = (UBX_Message*)(&(message->buffer[0]));
                    parse_ubx(ubx_message);
                    /* Reset for the next message */
                    init_message(message);
                }
            } else {
                assert(message->state == empty);
            }
        } else if (message->state == empty) {
            if (c == 0xB5U) {
                // printf("Got start of an UBX message %d\n", i);
                message->kind = UBX;
                message->buffer[message->current_position] = c;
                (message->current_position)++;
                message->state = waiting_for_more;
            } else if (c == (uint8_t)'$') {
                message->kind = NMEA;
                message->buffer[message->current_position] = c;
                (message->current_position)++;
                message->state = waiting_for_more;
            } else {
                /* Skip the bytes of a message for which we missed the begining */
            }
        }
    }
}

void communcation_loop(
        int fd, int number_of_samples, int do_flush,
        UBX_Message_Stack* ubx_messages)
{
    int n;
    static uint8_t input_buffer[INPUT_BUFFER_SIZE];
    Message message;
    init_message(&message);
    int k = 0;
    int x = 2;

    while (STOP==FALSE) {       /* loop for input */
        /* returns after at least 5 chars have been input */
        n = read(fd, input_buffer, INPUT_BUFFER_SIZE);
        input_buffer[n] = 0;               /* so we can printf... */

        parse(input_buffer, n, &message);
        if (k == number_of_samples) {
            STOP=TRUE;
        }
        if (x == 0) {
            Stack_Element* e = ubx_message_pop(ubx_messages);
            if (e != NULL) {
                write(fd, (char*)(&(e->message)), e->size);
            }
            x = 2;
        }
        k++;
        x--;
    }
}

void self_test(void)
{
    static uint8_t bytes[4] = { 0xEF, 0xBE, 0xAD, 0xDE };
    void *p = bytes;
    uint32_t test_integer = *((uint32_t*)p);
    assert(test_integer == 0xDEADBEEF);
}

int get_index(void)
{
    FILE* f;
    int next_index = 0;
    int index = 0;

    f = fopen("current_index.txt", "r");
    if (f == NULL) {
        next_index = 1;
    } else {
        int n = 0;
        n = fscanf(f, "%d", &index);
        assert(n > 0);
        next_index = index + 1;
        fclose(f);
    }

    f = fopen("current_index.txt", "w");
    assert(f != NULL);
    fprintf(f, "%d", next_index);
    fclose(f);

    return index;
}

int create_log_file(char* buffer)
{
    int ok = 0;
    char   namestr[130];
    int index;

    index = get_index();

    sprintf(namestr, "./experiment_%05d.txt", index);
    g_log_file = fopen( namestr, "w" );
    if (g_log_file) {
        if (setvbuf(g_log_file, buffer, _IOFBF, LOG_BUFFER_SIZE) == 0) {
            ok = 1;
        } else {
            perror( "setvbuf()" );
        }
    } else {
        perror( "fopen()" );
    }

    return ok;
}

int open_gnss(
        struct termios* oldtio,
        struct termios* newtio)
{
    int fd;

    fd = open(MODEMDEVICE, O_RDWR | O_NOCTTY );
    if (fd < 0) {
        perror(MODEMDEVICE);
    } else {
        tcgetattr(fd, oldtio); /* save current port settings */

        bzero(newtio, sizeof(struct termios));
        /* CS8 - 8 bit characters
         * CLOCAL - Ignore modem control lines
         * CREAD -  Enable receiver
         */
        newtio->c_cflag = BAUDRATE | CS8 | CLOCAL | CREAD;
        newtio->c_iflag = IGNPAR;
        newtio->c_oflag = 0;

        /* set input mode (non-canonical, no echo,...) */
        newtio->c_lflag = 0;
        newtio->c_cc[VTIME] = 0;   /* inter-character timer unused */
        newtio->c_cc[VMIN]  = 5;   /* blocking read until 5 chars received */

        tcflush(fd, TCIFLUSH);
        tcsetattr(fd, TCSANOW, newtio);
    }

    return fd;
}

void usage(void)
{
    printf( "mon:  gnss monitor\n" );
    printf( "Usage:\n");
    printf( "./mon [-f] [-n NUM]\n" );
    printf( "\n");
    printf( "-n NUM  -- minimum number of samples to get.\n" );
    printf( "-f      -- immediately flush a message to the logfile.\n" );
    printf( "-x      -- navigation rate 5Hz.\n" );
    printf( "-z      -- navigation rate 0.2Hz.\n" );
}

int main(int argc, char** argv)
{
    int fd;
    struct termios oldtio, newtio;
    int opt;
    int do_flush = 0;
    int number_of_samples = 0;
    int result = EXIT_FAILURE;
    int rate = RATE_NORMAL;

    while ((opt = getopt(argc,argv, "n:hfxz" )) != -1) {
        switch( opt ) {
            case 'n':
                number_of_samples = atoi(optarg);
                break;
            case 'f':
                do_flush = 1;
                break;
            case 'x':
                rate = RATE_FAST;
                break;
            case 'z':
                rate = RATE_SLOW;
                break;
            case 'h':
                usage();
                result = EXIT_SUCCESS;
            default:
                printf( "unknown option %c\n", opt );
        }
    }

    if (number_of_samples == 0) {
        /* Nothing to do */
    } else {
        UBX_Message_Stack ubx_messages;

        signal(SIGINT, signal_handler);
        self_test();
        init_ubx_message_stack(&ubx_messages);
        queue_messages(&ubx_messages, rate);

        fd = open_gnss(&oldtio, &newtio);
        if (fd >= 0) {
            char* log_buffer = malloc(LOG_BUFFER_SIZE + 10);
            if (log_buffer == NULL) {
                perror("malloc");
            } else {
                int ok;
                ok = create_log_file(log_buffer);
                if (ok) {
                    fprintf(g_log_file, "mon: rate %s version: %s\n", rate_string[rate], "V0.1.0");
                    communcation_loop(fd, number_of_samples, do_flush, &ubx_messages);
                    // Flush any unsaved logging to disk
                    fflush(g_log_file);
                    fclose(g_log_file);
                    printf("Stopped\n");
                    result = EXIT_SUCCESS;
                }
            }
            /* Restore old terminal settings */
            tcsetattr(fd, TCSANOW, &oldtio);
            close(fd);
        }
    }

    return result;
}

