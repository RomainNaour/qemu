/*
 * QEMU ARCnet COM90C66 emulation
 */

#include "arcnet.h"

/* name of the device (debug usage) */
#define COM90C65_DEVICE "com90c65"

#ifdef COM90C65_DEBUG
/* set it to 1 to print configuration changes */
# define DEBUG_CONFIG 1
/* set it to 1 to print send/receive events */
# define DEBUG_RXTX   1
/* set it to 1 to print sent/received packets */
# define DEBUG_DUMP   0
/* set it to 1 to print interrupt related events */
# define DEBUG_IRQ    0
/* set it to 1 to print read/write of registers */
# define DEBUG_IO_RW  0
/* set it to 1 to print read/write of memory mapped */
# define DEBUG_MM_RW  0
#else
# define DEBUG_CONFIG 0
# define DEBUG_RXTX   0
# define DEBUG_DUMP   0
# define DEBUG_IRQ    0
# define DEBUG_IO_RW  0
# define DEBUG_MM_RW  0
#endif

#define eprintf(fmt, args...) fprintf(stderr, COM90C65_DEVICE ": " fmt, ## args)
#define not_implemented0(fmt, args...) eprintf(fmt ": not implemented\n", ## args)
#ifdef COM90C65_DEBUG
# define dprintf0(fmt, args...) fprintf(stderr, fmt, ## args)
# define not_implemented(fmt, args...) not_implemented0(fmt, ## args)
#else
# define dprintf0(fmt, args...) do {} while (0)
# define not_implemented(fmt, args...) {                          \
        static int once = 0;                                      \
        if (! once) { once = 1; not_implemented0(fmt, ## args); } \
    }
#endif
#define dprintf(fmt, args...) dprintf0(COM90C65_DEVICE ": " fmt, ## args)
#define BOOL_TEXT(state) (state ? "enable" : "disable")

/* known byte to check the device at initialization */
#define RESET_MAGIC_NUMBER 0xD1

/* the 2 kB internal memory is partitioned in 4 blocks */
#define PAGE_SIZE ARCNET_MAX_SIZE
#define MEM_SIZE (PAGE_SIZE * 4)

/* addresses can be read/write or can have different uses in read and write */
enum register_offsets {
    OFFSET_STATUS       = 0x0, /* R   */
    OFFSET_IRQ_MASK     = 0x0, /*   W */
    OFFSET_DIAG_STATUS  = 0x1, /* R   */
    OFFSET_COMMAND      = 0x1, /*   W */
    OFFSET_CONFIG       = 0x2, /* R/W */
    OFFSET_IO_SELECT    = 0x3, /* R   */
    OFFSET_MEM_SELECT   = 0x4, /* R   */
    OFFSET_NODE_ID      = 0x5, /* R/W */
    OFFSET_EXTERNAL     = 0x7, /*   W */
    OFFSET_RESET_BEGIN  = 0x8, /* R/W */
    /*     RESET          0x9     R/W */
    /*     RESET          0xA     R/W */
    OFFSET_RESET_END    = 0xB, /* R/W */
    OFFSET_DATA_LOW     = 0xC, /* R/W */
    OFFSET_DATA_HIGH    = 0xD, /* R/W */
    OFFSET_ADDR_LOW     = 0xE, /* R/W */
    OFFSET_ADDR_HIGH    = 0xF, /* R/W */
    REGISTERS_SIZE
};

enum status_bits {
    BIT_TA    = 1 << 0,
    BIT_TMA   = 1 << 1,
    BIT_RECON = 1 << 2,
    BIT_TEST  = 1 << 3,
    BIT_POR   = 1 << 4,
    BIT_RI    = 1 << 7
};
/* RECON should not be set by default (see status register in datasheet) but is needed by Linux */
#define DEFAULT_STATUS (BIT_POR | BIT_RECON | BIT_RI | BIT_TA)
/* these status bits can be "masked" to generate an IRQ */
#define IRQ_MASK (BIT_TA | BIT_RECON | BIT_RI)

enum diag_status_bits {
    BIT_TOKEN   = 1 << 4,
    BIT_RCVACT  = 1 << 5,
    BIT_MYRECON = 1 << 7
};

/* commands can be distinguished by their first 3 bits */
#define COMMAND_MASK 7
enum commands {
    COMMAND_CLEAR_IRQ    = 0,
    COMMMAND_DISABLE_TX  = 1,
    COMMMAND_DISABLE_RX  = 2,
    COMMMAND_ENABLE_TX   = 3,
    COMMMAND_ENABLE_RX   = 4,
    COMMMAND_CONFIG      = 5,
    COMMMAND_CLEAR_FLAGS = 6
};
enum command_bits {
    BIT_CLEAR_IRQ      = 1 << 3,
    BIT_RX_BROADCAST   = 1 << 7,
    BIT_TX_LONG_PACKET = 1 << 3,
    BIT_CLEAR_POR      = 1 << 3,
    BIT_CLEAR_RECON    = 1 << 4
};
/* bits 3-4 of rx/tx commands define a page number */
#define PAGE(command) ((command & 0x18) >> 3)

enum config_bits {
    BIT_TXOFF     = 1 << 0,
    BIT_IO_ACCESS = 1 << 1,
    BIT_WAIT      = 1 << 2,
    BIT_ET2       = 1 << 3,
    BIT_ET1       = 1 << 4,
    BIT_DECODE    = 1 << 5,
    BIT_CCHEN     = 1 << 6,
    BIT_16EN      = 1 << 7
};
#define DEFAULT_CONFIG (BIT_ET1 | BIT_ET2 | BIT_WAIT)

/* addresses are 11 bits wide to access to the 2 kB of the internal memory */
#define ADDRESS_MASK 0x7ff
enum address_bits {
    BIT_AUTO_INC = 1 << 14
};

/* THE object */
typedef struct COM90C65State {

    /* internal storage */
    MemoryRegion io;
    MemoryRegion mmio;
    uint8_t mem[MEM_SIZE];
    uint8_t long_packet_enabled;
    uint8_t broadcast_enabled;
    uint8_t reception_page;

    /* registers */
    uint8_t status;
    uint8_t irq_mask;
    uint8_t diag_status;
    uint8_t config;
    uint8_t io_select;
    uint8_t mem_select;
    uint8_t node_id;
    uint16_t data;
    uint16_t address;

    /* registers access by arrays */
    struct {
        uint8_t *read[REGISTERS_SIZE];
        uint8_t *write[REGISTERS_SIZE];
    } registers;

    /* qemu handlers */
    qemu_irq irq;
    NICState *nic;
    NICConf c;

} COM90C65State;

void com90c65_setup_io(COM90C65State *s, unsigned size);
void com90c65_setup_mmio(COM90C65State *s, unsigned size);
void com90c65_registers_access_init(COM90C65State *s);
void com90c65_reset(void *user_data);
int com90c65_can_receive(VLANClientState *nc);
ssize_t com90c65_receive(VLANClientState *nc, const uint8_t *buffer, size_t size_);
