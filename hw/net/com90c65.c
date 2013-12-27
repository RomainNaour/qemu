/*
 * QEMU ARCnet COM90C66 emulation
 *
 * This is a hardware NIC (Network Interface Card)
 * emulation of the COM90C66 device family.
 *
 * It works on an (emulated) ISA bus. The code of this file should be generic
 * however. ISA dependant code is in the source file com90c65-isa.c.
 * Access can be 8 or 16 bits, in I/O or memory mapped mode.
 * There is a burst mode called command chaining (not implemented).
 * The datasheet is available at:
 * http://www.smsc.com/main/tools/discontinued/90c66.pdf
 *
 * The device communicates with its vlan.
 * An arcnet layer must be connected to the same vlan.
 * A send comes from the driver to the vlan.
 * A receive comes from the vlan to the driver.
 */

#include "hw/hw.h"
#include "hw/isa/isa.h"
#include "hw/i386/pc.h"
#include "net/net.h"
#include "hw/net/com90c65.h"

/* map the register variables with 2 arrays for read and write access */
void com90c65_registers_access_init(COM90C65State *s) {
    /* FIXME: what happens if WORDS_BIGENDIAN != TARGET_WORDS_BIGENDIAN ? */
#if __BYTE_ORDER == __LITTLE_ENDIAN
# define LOW_BYTE_OFFSET  0
# define HIGH_BYTE_OFFSET 1
#elif __BYTE_ORDER == __BIG_ENDIAN
# define LOW_BYTE_OFFSET  1
# define HIGH_BYTE_OFFSET 0
#endif

    /* dummy is used for registers whose written value is not used */
    static uint8_t dummy;
    /* structure is initialized with 0, so the unreferenced registers are NULL */
    s->registers.read [OFFSET_STATUS]      = &s->status;
    s->registers.write[OFFSET_IRQ_MASK]    = &s->irq_mask;
    s->registers.read [OFFSET_DIAG_STATUS] = &s->diag_status;
    s->registers.write[OFFSET_COMMAND]     = &dummy;
    s->registers.read [OFFSET_CONFIG]      = &s->config;
    s->registers.write[OFFSET_CONFIG]      = &s->config;
    s->registers.read [OFFSET_IO_SELECT]   = &s->io_select;
    s->registers.read [OFFSET_MEM_SELECT]  = &s->mem_select;
    s->registers.read [OFFSET_NODE_ID]     = &s->node_id;
    s->registers.write[OFFSET_NODE_ID]     = &s->node_id;
    s->registers.write[OFFSET_EXTERNAL]    = &dummy;
    s->registers.read [OFFSET_DATA_LOW]    = (uint8_t*)&s->data + LOW_BYTE_OFFSET;
    s->registers.write[OFFSET_DATA_LOW]    = (uint8_t*)&s->data + LOW_BYTE_OFFSET;
    s->registers.read [OFFSET_DATA_HIGH]   = (uint8_t*)&s->data + HIGH_BYTE_OFFSET;
    s->registers.write[OFFSET_DATA_HIGH]   = (uint8_t*)&s->data + HIGH_BYTE_OFFSET;
    s->registers.read [OFFSET_ADDR_LOW]    = (uint8_t*)&s->address + LOW_BYTE_OFFSET;
    s->registers.write[OFFSET_ADDR_LOW]    = (uint8_t*)&s->address + LOW_BYTE_OFFSET;
    s->registers.read [OFFSET_ADDR_HIGH]   = (uint8_t*)&s->address + HIGH_BYTE_OFFSET;
    s->registers.write[OFFSET_ADDR_HIGH]   = (uint8_t*)&s->address + HIGH_BYTE_OFFSET;
}

/* print device configuration (for debug use) of bits set in mask */
static void com90c65_print_config(uint8_t config, uint8_t mask) {
    if (DEBUG_CONFIG) {
        if (mask & BIT_TXOFF)
            dprintf("%s tx\n", BOOL_TEXT(~ config & BIT_TXOFF));
        if (mask & BIT_IO_ACCESS)
            dprintf("%s buffer access\n", config & BIT_IO_ACCESS ? "I/O" : "memory");
        if (mask & BIT_WAIT)
            dprintf("%s zero wait state\n", BOOL_TEXT(~ config & BIT_WAIT));
        if (mask & (BIT_ET1 | BIT_ET2))
            dprintf("extended timeout = %d%d\n", (config & BIT_ET1) / BIT_ET1, (config & BIT_ET2) / BIT_ET2);
        if (mask & (BIT_DECODE | BIT_IO_ACCESS))
            dprintf("%dK ROM\n", config & BIT_IO_ACCESS ? (config & BIT_DECODE ? 16 : 8) : (config & BIT_DECODE ? 2 : 128));
        if (mask & BIT_CCHEN)
            dprintf("%s command chaining\n", BOOL_TEXT(config & BIT_CCHEN));
        if (mask & BIT_16EN)
            dprintf("%d-bit\n", config & BIT_16EN ? 16 : 8);
    }
    if (config & mask & BIT_CCHEN)
        not_implemented("command chaining");
}

/* generate an IRQ if a masked status bit becomes 1 */
static void com90c65_update_irq(COM90C65State *s, int generate) {
    static uint8_t old_isr = 0;
    uint8_t isr = s->status & s->irq_mask & IRQ_MASK;
    int trigger = (isr & ~old_isr) != 0;
    old_isr = isr;
    if (generate) {
        if (DEBUG_IRQ) {
            dprintf ("status=0x%x", s->status);
            if (trigger)
                dprintf0(" => IRQ\n");
            else
                dprintf0("\n");
        }
        qemu_set_irq(s->irq, trigger);
    }
}

/* reset the device with the default values,
 * stands for hard and soft reset */
void com90c65_reset(void *user_data) {
    COM90C65State *s = user_data;

    dprintf("reset\n");

    s->status = DEFAULT_STATUS;
    s->irq_mask = 0;
    s->diag_status = 0;
    s->config = DEFAULT_CONFIG;

    /* an arbitrary number is set and can be check for a valid reset */
    s->mem[0] = RESET_MAGIC_NUMBER;
    /* the node id can be read after a reset */
    s->mem[1] = s->node_id;

    s->long_packet_enabled = 0;
    s->broadcast_enabled = 0;

    com90c65_update_irq(s, 0);
}

static uint32_t com90c65_reset_on_read(void *user_data, uint32_t address) {
    com90c65_reset(user_data);
    return 0;
}

static void com90c65_reset_on_write(void *user_data, uint32_t address, uint32_t value) {
    com90c65_reset(user_data);
}

/* send the packet of the specified memory page to the vlan */
static void com90c65_send(COM90C65State *s, int page) {
    const uint8_t *buffer = &s->mem[PAGE_SIZE * page];
    int size;

    if (s->long_packet_enabled)
        size = ARCNET_SIZE(buffer);
    else
        size = ARCNET_SHORT_SIZE;

    if (DEBUG_RXTX)
        dprintf("send %d bytes\n", size);

    /* send to other clients of the vlan,
     * it should be received by an arcnet layer */
    qemu_send_packet(qemu_get_queue(s->nic), buffer, size);
}

/* receive a packet from the vlan to the specified memory page */
ssize_t com90c65_receive(NetClientState *nc, const uint8_t *buffer, size_t size_)
{
    COM90C65State *s = DO_UPCAST(NICState, nc, nc)->opaque;
    int size = size_;

    if (buffer == NULL || size <= 0)
        return -1;

    /* a broadcasted packet has a recipient 0 and can be disabled by a command */
    if (buffer [ARCNET_DESTINATION] == 0 && ! s->broadcast_enabled) {
        if (DEBUG_RXTX)
            dprintf("ignore broadcasted packet\n");
        return -1;
    }

    if (DEBUG_RXTX)
        dprintf("receive %d bytes\n", size);

    /* copy the packet into the internal memory page specified on rx start */
    memcpy(&s->mem[PAGE_SIZE * (s->reception_page & 3)], buffer, size);

    /* an IRQ signals the packet availability */
    s->status |= BIT_RI;
    com90c65_update_irq(s, 1);
    return size_;
}

/* the arcnet layer send a packet to the vlan only when the device is ready to receive */
int com90c65_can_receive(NetClientState *nc)
{
    COM90C65State *s = DO_UPCAST(NICState, nc, nc)->opaque;
    return (s->status & BIT_RI) == 0;
}

/* various actions must be done after a write in the command register */
static void com90c65_command(COM90C65State *s, uint32_t command) {
    switch (command & COMMAND_MASK) {
        /* command chaining is not implemented */
        case COMMAND_CLEAR_IRQ:
            if ((command & BIT_CLEAR_IRQ) == 0) {
                if (DEBUG_IRQ)
                    dprintf("command chaining: clear tx interrupt\n");
                /* TODO (not used in Linux) */
            } else {
                if (DEBUG_IRQ)
                    dprintf("command chaining: clear rx interrupt\n");
                /* TODO (not used in Linux) */
            }
            break;
        /* the emulation has no pending transmit, so cancel does nothing */
        case COMMMAND_DISABLE_TX:
            if (DEBUG_RXTX)
                dprintf("cancel tx\n");
            s->status |= BIT_TA;
            break;
        /* cancel reception if it has not begun */
        case COMMMAND_DISABLE_RX:
            if (DEBUG_RXTX)
                dprintf("cancel rx\n");
            s->status |= BIT_RI;
            /* save the bit state without generate an IRQ */
            com90c65_update_irq(s, 0);
            break;
        /* send the packet of the specified page */
        case COMMMAND_ENABLE_TX:
            if (DEBUG_RXTX)
                dprintf("start tx from page %d\n", PAGE(command));
            s->status &= ~(BIT_TA | BIT_TMA);
            /* save the resetted bits states */
            com90c65_update_irq(s, 0);
            com90c65_send(s, PAGE(command));
            if (1) { /* on acknowledgement (assume it is ok) */
                s->status |= BIT_TMA;
            }
            if (1) { /* on transmit completion (assume it is ok) */
                s->status |= BIT_TA;
                /* generate an IRQ */
                com90c65_update_irq(s, 1);
            }
            break;
        /* prepare to receive a packet in the specified page */
        case COMMMAND_ENABLE_RX:
            s->broadcast_enabled = command & BIT_RX_BROADCAST;
            s->reception_page = PAGE(command);
            if (DEBUG_RXTX)
                dprintf("start rx to page %d (broadcast %sd)\n", s->reception_page, BOOL_TEXT(s->broadcast_enabled));
            s->status &= ~ BIT_RI;
            /* save the cleared bit state (IRQ is generated on reception) */
            com90c65_update_irq(s, 0);
            break;
        /* set max size of packet to send */
        case COMMMAND_CONFIG:
            s->long_packet_enabled = command & BIT_TX_LONG_PACKET;
            if (DEBUG_CONFIG)
                dprintf("%s long packet\n", BOOL_TEXT(s->long_packet_enabled));
            break;
        /* clear status bits */
        case COMMMAND_CLEAR_FLAGS:
            if (command & BIT_CLEAR_POR) {
                if (DEBUG_IO_RW)
                    dprintf("clear flag POR\n");
                s->status &= ~ BIT_POR;
            }
            if (command & BIT_CLEAR_RECON) {
                if (DEBUG_IO_RW)
                    dprintf("clear flag RECON\n");
                s->status &= ~ BIT_RECON;
                /* save the cleared bit state */
                com90c65_update_irq(s, 0);
            }
            break;
        default:
            dprintf("an invalid command (0x%x) is received\n", command);
    }
}

/* read data in internal memory at the address written in the address registers
 * and set it in the data registers */
static void com90c65_update_data(COM90C65State *s) {
    s->data = s->mem[s->address & ADDRESS_MASK] | s->mem[(s->address & ADDRESS_MASK) + 1] << 8;
}

/* increment address registers if needed and update data registers,
 * auto_increment can be 1 or 2 for 8 or 16 bits access */
static void com90c65_auto_increment(COM90C65State *s, int auto_increment) {
    if (s->address & BIT_AUTO_INC)
        s->address = ((s->address & ADDRESS_MASK) + auto_increment) | BIT_AUTO_INC;
    com90c65_update_data(s);
}

/* return the value of the register */
static uint32_t com90c65_ioport_read(COM90C65State *s, uint32_t address, int width) {
    uint8_t *reg;
    uint32_t value;
    int auto_increment = 1;

    address &= 0xf;

    if (width == 16 && s->config & BIT_16EN && address == OFFSET_DATA_LOW) {
        /* 16 bits: read the word of the data register */
        value = s->data;
        auto_increment = 2;
    } else {
        /* 8 bits: read the byte of the specified register */
        reg = s->registers.read[address];
        if (reg == NULL) {
            dprintf("an invalid register (0x%X) is read\n", address);
            return 0xaa;
        }
        value = *reg;
    }
    if (DEBUG_IO_RW)
        dprintf("register 0x%X is read (0x%x)\n", address, value);

    /* do specific actions */
    switch (address) {
        case OFFSET_DIAG_STATUS:
            /* reset diag_status after a read */
            s->diag_status = 0;
            break;
        case OFFSET_DATA_LOW:
            /* load next data if the auto increment bit is set */
            com90c65_auto_increment(s, auto_increment);
            break;
    }

    return value;
}

/* a register read can be 8-bit wide (inb syscall) */
static uint32_t com90c65_ioport_read8(void *user_data, uint32_t address) {
    COM90C65State *s = user_data;
    return com90c65_ioport_read(s, address, 8);
}

/* a register read can be 16-bit wide (inw syscall) */
static uint32_t com90c65_ioport_read16(void *user_data, uint32_t address) {
    COM90C65State *s = user_data;
    return com90c65_ioport_read(s, address, 16);
}

/* write value into the specified register */
static void com90c65_ioport_write(COM90C65State *s, uint32_t address, uint32_t value, int width) {
    uint8_t *reg;
    uint8_t old_value = 0;
    int auto_increment = 1;

    address &= 0xf;

    if (width == 16 && s->config & BIT_16EN && address == OFFSET_DATA_LOW) {
        /* 16 bits: write the word into the data register */
        value &= 0xffff;
        s->data = value;
        auto_increment = 2;
    } else {
        /* 8 bit: write the byte into the specified register */
        value &= 0xff;
        reg = s->registers.write[address];
        if (reg == NULL) {
            dprintf("an invalid register (0x%X) is written\n", address);
            return;
        }
        old_value = *reg;
        *reg = value;
    }
    if (DEBUG_IO_RW)
        dprintf("register 0x%X is written (0x%x)\n", address, value);

    /* do specific actions */
    switch (address) {
        case OFFSET_IRQ_MASK:
            if (DEBUG_IRQ)
                dprintf("mask=0x%x\n", s->irq_mask);
            /* generate an IRQ if it unmaks a bit set at 1 */
            com90c65_update_irq(s, 1);
            break;
        case OFFSET_COMMAND:
            com90c65_command(s, value);
            break;
        case OFFSET_CONFIG:
            /* print the configuration changes (debug only) */
            com90c65_print_config(s->config, s->config ^ old_value);
            break;
        case OFFSET_DATA_LOW:
            /* write the data in the internal memory */
            s->mem[s->address & ADDRESS_MASK] = s->data & 0xff;
            if (auto_increment == 2) {
                /* write the upper byte of the 16-bit data */
                s->mem[(s->address + 1) & ADDRESS_MASK] = s->data >> 8;
            }
            /* increment address for the next data if the auto increment bit is set */
            com90c65_auto_increment(s, auto_increment);
            break;
        case OFFSET_ADDR_LOW:
        case OFFSET_ADDR_HIGH:
            /* load data of the specified address for a read */
            com90c65_update_data(s);
            break;
    }
}

/* a register write can be 8-bit wide (outb syscall) */
static void com90c65_ioport_write8(void *user_data, uint32_t address, uint32_t value) {
    COM90C65State *s = user_data;
    com90c65_ioport_write(s, address, value, 8);
}

/* a register write can be 16-bit wide (outw syscall) */
static void com90c65_ioport_write16(void *user_data, uint32_t address, uint32_t value) {
    COM90C65State *s = user_data;
    com90c65_ioport_write(s, address, value, 16);
}

/* read from memory mapped */
static uint64_t com90c65_mmio_read(void *opaque, hwaddr addr, unsigned size) {
    COM90C65State *s = opaque;
    uint32_t value = 0;
    int offset;
    for (offset = 0; offset < size; offset ++)
        value |= s->mem[addr + offset] << (offset << 3);
    if (DEBUG_MM_RW)
        dprintf("read %d bytes from memory 0x" TARGET_FMT_plx " (0x%x)\n",
                size, addr, value);
    return value;
}

/* write to memory mapped */
static void com90c65_mmio_write(void *opaque, hwaddr addr, uint64_t data, unsigned size) {
    COM90C65State *s = opaque;
    uint32_t value = data;
    int offset;
    if (DEBUG_MM_RW)
        dprintf("write %d bytes to memory: 0x" TARGET_FMT_plx " (0x%x)\n",
                 size, addr, value);
    for (offset = 0; offset < size; offset ++)
        s->mem[addr + offset] = value >> (offset << 3) & 0xff;
}

static uint64_t com90c65_read(void *opaque, hwaddr addr,
                              unsigned size)
{
    COM90C65State *s = opaque;

    if (addr == OFFSET_DATA_LOW && size == 2) {
        return com90c65_ioport_read16(s, addr);
    } else if (addr >= OFFSET_RESET_BEGIN &&
               addr <= OFFSET_RESET_END &&
               size == 1) {
        return com90c65_reset_on_read(s, addr);
    } else if (addr < REGISTERS_SIZE && size == 1) {
        return com90c65_ioport_read8(s, addr);
    }
    return ((uint64_t)1 << (size * 8)) - 1;
}

static void com90c65_write(void *opaque, hwaddr addr,
                           uint64_t data, unsigned size)
{
    COM90C65State *s = opaque;

    if (addr == OFFSET_DATA_LOW && size == 2) {
        return com90c65_ioport_write16(s, addr, data);
    } else if (addr >= OFFSET_RESET_BEGIN &&
               addr <= OFFSET_RESET_END &&
               size == 1) {
        return com90c65_reset_on_write(s, addr, data);
    } else if (addr < REGISTERS_SIZE && size == 1) {
        return com90c65_ioport_write8(s, addr, data);
    }
}

static const MemoryRegionOps com90c65_io_ops = {
    .read = com90c65_read,
    .write = com90c65_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

void com90c65_setup_io(COM90C65State *s, unsigned size)
{
    memory_region_init_io(&s->io, &com90c65_io_ops, s, "com90c65-io", size);
}

static const MemoryRegionOps com90c65_mmio_ops = {
    .read = com90c65_mmio_read,
    .write = com90c65_mmio_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

void com90c65_setup_mmio(COM90C65State *s, unsigned size)
{
    memory_region_init_io(&s->mmio, &com90c65_mmio_ops, s, "com90c65-mmio", size);
}
