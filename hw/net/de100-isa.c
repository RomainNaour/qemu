/*
 * QEMU DE100 emulation
 *
 * The DE100 is an ISA DECnet network card. It is also known as DEC EtherWORKS
 * LC Ethernet Controller.
 */

#include "hw/isa/isa.h"
#include "net/net.h"
#include "hw/net/pcnet.h"
#include "qemu-common.h"

#include "hw/loader.h"

#define DE100_DEVICE "de100"

#ifdef DEBUG_DE100
# define DPRINTF(fmt, args...) fprintf(stderr, DE100_DEVICE ": " fmt, ## args)
#else
# define DPRINTF(fmt, args...) do {} while (0)
#endif /* !DEBUG_DE100 */

#ifdef DEBUG_DE100_INTERNALS
# define DPRINTF_INT(fmt, args...) fprintf(stderr, DE100_DEVICE ": " fmt, ## args)
#else
# define DPRINTF_INT(fmt, args...) do {} while (0)
#endif /* !DEBUG_DE100 */

#ifdef DEBUG_DE100_IO
# define DPRINTF_IO(fmt, args...) fprintf(stderr, DE100_DEVICE ": " fmt, ## args)
#else
# define DPRINTF_IO(fmt, args...) do {} while (0)
#endif /* !DEBUG_DE100_IO */

#ifdef DEBUG_DE100_MMIO
# define DPRINTF_MMIO(fmt, args...) fprintf(stderr, DE100_DEVICE ": " fmt, ## args)
#else
# define DPRINTF_MMIO(fmt, args...) do {} while (0)
#endif /* !DEBUG_DE100_MMIO */

#ifdef DEBUG_DE100_ROM
# define DPRINTF_ROM(fmt, args...) fprintf(stderr, DE100_DEVICE ": " fmt, ## args)
#else
# define DPRINTF_ROM(fmt, args...) do {} while (0)
#endif /* !DEBUG_DE100_ROM */

/**
 * NETWORK INTERFACE CSR (NI_CSR) bit definitions
 * The NICSR is a register of the DE100 card.
 */
#define NICSR_TO        0x0100	/* Time Out for remote boot */
#define NICSR_SHE       0x0080  /* SHadow memory Enable */
#define NICSR_BS        0x0040  /* Bank Select */
#define NICSR_BUF       0x0020	/* BUFfer size (1->32k, 0->64k) */
#define NICSR_RBE       0x0010	/* Remote Boot Enable (1->net boot) */
#define NICSR_AAC       0x0008  /* Address ROM Address Counter (1->enable) */
#define NICSR_IM        0x0004	/* Interrupt Mask (1->mask) */
#define NICSR_IEN       0x0002	/* Interrupt tristate ENable (1->enable) */
#define NICSR_LED       0x0001	/* LED control */

#define LA_MASK 0x0000ffff	/* LANCE address mask for mapping network RAM
				   to LANCE memory address space */

#define DE100_ROMFILE   "de100.bin"
#define ROM_SIZE	(16 * 1024)
#define RAM_SIZE	(32 * 1024)
#define RAM_OFFSET	(RAM_SIZE)

/* Region aliasing allows the bank selection process to be quite simple
 * between swaping and exchanging buffers.
 * But, these regions cannot be executable, and thus does not permit IO
 * debugging. Reducing them will make the process more complexe, but
 * make a debugging region (de100-mmio-debug) visible.
 * The bank selection process must then be managed in this region too.
 */
#ifdef DEBUG_DE100_MMIO
/* Note: All these size must be 0x1000 aligned (minimum MMU page size) */
# define ALIAS_START	0x1000
/* # define ALIAS_END	0x2000 */
# define ALIAS_END	(RAM_SIZE)
#else /* DEBUG_DE100_MMIO */
# define ALIAS_START	0
# define ALIAS_END	(RAM_SIZE)
#endif /* !DEBUG_DE100_MMIO */

#define ALIAS_SIZE	((ALIAS_END) - (ALIAS_START))

/**
 * Device signature returned by the DE100 ISA card. This signature is checked by
 * the host driver to recognise the device.
 */
/* #define DE100_SIGNATURE 0xAA5500FFUL */
/**
 * If the device signature is read further 16 bytes, the hardware MAC address,
 * and, its CRC, is found.
 */
#define ROM_SIGNATURE_SIZE 16
unsigned char	rom_signature[ROM_SIGNATURE_SIZE] = {
    0xFF, 0x00, 0x55, 0xAA, 0xFF, 0x00, 0x55, 0xAA
};

typedef struct ISADE100State {
    ISADevice dev;
    PCNetState state;           /**< state of the Am7990 chip */

    uint32_t iobase;            /**< Base address of I/O ports */
    MemoryRegion io;

    uint32_t mmiobase;          /**< Base address of I/O memory */
    MemoryRegion mmio;
    MemoryRegion mmio_bank1;
    MemoryRegion mmio_bank2;

    uint32_t mmio_zone1_addr;
    uint32_t mmio_zone2_addr;

    uint8_t mmiosize;           /**< Size of I/O mem (32 or 64 KB) */
#if defined(DEBUG_DE100_MMIO) || defined(DEBUG_DE100_ROM)
    MemoryRegion mmio_dbg;
#endif /* DEBUG_DE100_MMIO || DEBUG_DE100_ROM */

    uint32_t rombase;           /**< Base address of rom memory */
    MemoryRegion rom;
    char* romfile;
    int romshadowed;
    int ramselected;

    uint32_t isairq;            /**< IRQ of the board */

    uint16_t nicsr;             /**< Network interface CSR */
    uint signature_pos;         /**< signature position used when reading the
                                   signature of the card */
    uint eth_addr_pos;          /**< prom position used when reading the MAC
                                   address */
    qemu_irq irq;
} ISADE100State;

/**
 * Update the ROM memory mapping depending on current state and
 * NICSR value.
 * Note: with DEBUG_DE100_ROM set, this function does almost nothing.
 */
static void de100_ram_shadow_update(ISADE100State *isa)
{
    if(!(isa->romshadowed) && (isa->nicsr & NICSR_SHE)) {
        // not shadowed but should be: remove rom subregion form memory map
#ifndef DEBUG_DE100_ROM
        memory_region_del_subregion(isa_address_space(&isa->dev), &isa->rom);
#endif /* !DEBUG_DE100_ROM */
        isa->romshadowed = 1;
	DPRINTF_INT("ROM is now shadowed\n");
    }
    if((isa->romshadowed) && !(isa->nicsr & NICSR_SHE)) {
        // shadowed but should not be: add subregion to memory map
#ifndef DEBUG_DE100_ROM
        memory_region_add_subregion_overlap(isa_address_space(&isa->dev),
                                            isa->rombase, &isa->rom, 3);
#endif /* !DEBUG_DE100_ROM */
        isa->romshadowed = 0;
	DPRINTF_INT("ROM shadowing disabled\n");
    }
}

/**
 * Reset the MAC address ROM counter when NICSR_AAC is set.
 */
static void de100_rom_reset(ISADE100State* isa)
{
    if (isa->nicsr & NICSR_AAC)
	isa->signature_pos = 0;
}

/**
 * Update the memory bank mapping:
 * Set 32k region to bank1.
 * see: de100_ram_select
 */
static void de100_ram_select32(ISADE100State* isa)
{
    memory_region_del_subregion(isa_address_space(&isa->dev),
				&isa->mmio_bank2);
    memory_region_add_subregion_overlap(isa_address_space(&isa->dev),
					isa->mmio_zone2_addr + ALIAS_START,
					&isa->mmio_bank1,
					2);

    DPRINTF_INT("Bank1 mapped on 0x%X:0x%X\n",
	    isa->mmio_zone2_addr, RAM_SIZE);
}

/**
 * Update the memory bank mapping:
 * Set 64k region to bank1 followed by bank2.
 * see: de100_ram_select
 */
static void de100_ram_select64(ISADE100State* isa)
{
    memory_region_del_subregion(isa_address_space(&isa->dev),
				&isa->mmio_bank2);
    memory_region_del_subregion(isa_address_space(&isa->dev),
				&isa->mmio_bank1);

    memory_region_add_subregion_overlap(isa_address_space(&isa->dev),
					isa->mmio_zone2_addr + ALIAS_START,
					&isa->mmio_bank1, 2);

    memory_region_add_subregion_overlap(isa_address_space(&isa->dev),
					isa->mmio_zone1_addr + ALIAS_START,
					&isa->mmio_bank2, 2);

    DPRINTF_INT("Bank1 mapped on 0x%X:0x%X\n",
	    isa->mmio_zone2_addr, RAM_SIZE);
    DPRINTF_INT("Bank2 mapped on 0x%X:0x%X\n",
	    isa->mmio_zone1_addr, RAM_SIZE);
}

/**
 * Update the memory bank mapping:
 * Set 32k region to bank2.
 * see: de100_ram_select
 */
static void de100_ram_deselect32(ISADE100State* isa)
{
    memory_region_del_subregion(isa_address_space(&isa->dev),
				&isa->mmio_bank1);

    memory_region_add_subregion_overlap(isa_address_space(&isa->dev),
					isa->mmio_zone2_addr + ALIAS_START,
					&isa->mmio_bank2, 2);
    DPRINTF_INT("Bank2 mapped on 0x%X:0x%X\n",
	    isa->mmio_zone2_addr, RAM_SIZE);
}

/**
 * Update the memory bank mapping:
 * Set 64k region to bank2 followed by bank1.
 * see: de100_ram_select
 */
static void de100_ram_deselect64(ISADE100State* isa)
{
    memory_region_del_subregion(isa_address_space(&isa->dev),
				&isa->mmio_bank1);
    memory_region_del_subregion(isa_address_space(&isa->dev),
				&isa->mmio_bank2);

    memory_region_add_subregion_overlap(isa_address_space(&isa->dev),
					isa->mmio_zone1_addr + ALIAS_START,
					&isa->mmio_bank1, 2);

    memory_region_add_subregion_overlap(isa_address_space(&isa->dev),
					isa->mmio_zone2_addr + ALIAS_START,
					&isa->mmio_bank2, 2);
}

/**
 * Bank selection function:
 * Update the memory mapping depending on NICSR_BS and mmiosize.
 * If the bank select changed, depending on the network buffer size
 * configuration, we have to swap buffers (32KB), or to exchange
 * upper and lower 32KB buffers (64KB).
 */
static void de100_ram_select(ISADE100State* isa)
{
    if ((isa->nicsr & NICSR_BS) && !isa->ramselected)
    {
	if (isa->mmiosize == 32)
	    de100_ram_select32(isa);
	else
	    de100_ram_select64(isa);
	isa->ramselected = 1;
    }

    if (!(isa->nicsr & NICSR_BS) && isa->ramselected)
    {
	if (isa->mmiosize == 32)
	    de100_ram_deselect32(isa);
	else
	    de100_ram_deselect64(isa);
	isa->ramselected = 0;
    }
}

/**
 * Write data to an I/O port of the DE100 device
 *
 * @param opaque Device state (ISADE100State)
 * @param addr address of the I/O port
 * @param val value to write
 * @param size size of the val (should be 2 bytes for this device)
 */
static void de100_io_write(void *opaque, target_phys_addr_t addr,
                           uint64_t val, unsigned size)
{
    ISADE100State *isa = opaque;
    static short data = 0;

    DPRINTF_IO("I/O write: addr=0x%" PRIx64 ", val=0x%" PRIx64 ", size=%i\n", addr, val, size);
    switch (addr) {
        case 0x00:
            isa->nicsr = val & (0xffff & ~NICSR_BUF & ~NICSR_RBE);
            de100_ram_shadow_update(isa);
	    de100_rom_reset(isa);
	    de100_ram_select(isa);
            break;
        case 0x04:
        case 0x06:
            if (size < 2)
	    {
		    data = val;
		    return;
	    }

	    pcnet_ioport_writew(&isa->state, addr - 0x04, val);
	    break;
        case 0x05:
        case 0x07:
	    // only works if consecutive access of 8bit registers
            pcnet_ioport_writew(&isa->state, addr - 0x05, val << 8 | data);
	    data = 0;
            break;
        default:
            break;
    }
}

/**
 * Return the MAC address and the checksum byte by byte
 *
 * @param isa Device state
 *
 * @return a part of the MAC address or checksum
 */
static uint8_t de100_get_hw_addr(ISADE100State *isa)
{
    static int chksum = 0;
    uint8_t val = 0;

    if (isa->eth_addr_pos == 0)
        chksum = 0;
    if (isa->eth_addr_pos < 6) {
        val = isa->state.prom[isa->eth_addr_pos];
        chksum = chksum + (val << ((isa->eth_addr_pos % 2) * 8));
        if ((isa->eth_addr_pos == 1) || (isa->eth_addr_pos == 3))
            chksum <<= 1;
        if (chksum > 0xffff)
            chksum -= 0xffff;
    } else if (isa->eth_addr_pos == 6) {
        val = chksum & 0xff;
    } else if (isa->eth_addr_pos == 7) {
        val = (chksum >> 8) & 0xff;
    }
    isa->eth_addr_pos = (isa->eth_addr_pos + 1) % 8;
    return val;
}

/**
 * Read data from an I/O port of the DE100 device
 *
 * @param opaque Device state (ISADE100State)
 * @param addr address of the I/O port
 * @param size size of the return value (should be 2 bytes for this device)
 *
 * @return read value
 */
static uint64_t de100_io_read(void *opaque, target_phys_addr_t addr,
                              unsigned size)
{
    ISADE100State *isa = opaque;
    uint64_t val = -1;

    switch (addr) {
        case 0x00:
            val = isa->nicsr |
		    ((isa->mmiosize == 32) ? NICSR_BUF : 0);
            break;
        case 0x04:
        case 0x06:
	     val = pcnet_ioport_readw(&isa->state, addr - 0x04);
             break;
        case 0x05:
        case 0x07:
	     val = pcnet_ioport_readw(&isa->state, addr - 0x05) >> 8;
	     break;
        case 0x0c:
        case 0x0d:
            val = rom_signature[isa->signature_pos % ROM_SIGNATURE_SIZE];
            ++isa->signature_pos;
            break;
        default:
            break;
    }
    val &= ((1 << (size * 8)) - 1);
    DPRINTF_IO("I/O read: addr=0x%" PRIx64 ", val=0x%" PRIx64 ", size=%i\n",
	       addr, val, size);
    return val;
}

static const MemoryRegionOps de100_io_ops = {
    .read = de100_io_read,
    .write = de100_io_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = {
        .min_access_size = 2,
        .max_access_size = 2,
    },
};

#if defined(DEBUG_DE100_MMIO) || defined(DEBUG_DE100_ROM)
/**
 * Write data to the memory of the DE100 device with traces
 *
 * @param opaque Device state (ISADE100State)
 * @param addr address of the memory
 * @param val value to write
 * @param size size of the val
 */
static void de100_mmio_write(void *opaque, target_phys_addr_t addr,
                           uint64_t val, unsigned size)
{
    ISADE100State *isa = opaque;
    uint8_t *ptr = memory_region_get_ram_ptr(&isa->mmio);

    // ONLY WORKS IF rombase is placed over the end of mmio as in actual card.
    if ((!isa->romshadowed) && (addr >= (isa->rombase - isa->mmiobase)))
    {
        ptr = memory_region_get_ram_ptr(&isa->rom);
        addr -= (isa->rombase - isa->mmiobase);

        DPRINTF_ROM("ROM IGNORE write: addr=0x%" PRIx64 ", val=0x%" PRIx64 ", size=%d\n", addr, val, size);
        return;
    }

    if (isa->mmiosize == 32)
	    addr += RAM_SIZE;

    if (isa->nicsr & NICSR_BS)
	    addr = (addr + RAM_SIZE) % (2 * RAM_SIZE);

    DPRINTF_MMIO("MMIO write: addr=0x%" PRIx64 ", val=0x%" PRIx64 ", size=%d (%d)\n", addr, val, size, !!(isa->nicsr & NICSR_BS));

    switch(size) {
    case 4:
        /* 32 bit write access */
        *((uint32_t*)(ptr+addr))= val;
        break;
    case 2:
        /* 16 bit write access */
        *((uint16_t*)(ptr+addr))= val;
        break;
    case 1:
        /* 8 bit write access */
        *((uint8_t*)(ptr+addr))= val;
        break;
    default:
        hw_error(DE100_DEVICE ": unexpected %s with size = %u", __func__, size);
        break;
    }
}

/**
 * Read data from the memory of the DE100 device with traces
 *
 * @param opaque Device state (ISADE100State)
 * @param addr address of the memory
 * @param size size of the return value
 *
 * @return read value
 */
static uint64_t de100_mmio_read(void *opaque, target_phys_addr_t addr,
                              unsigned size)
{
    ISADE100State *isa = opaque;
    uint64_t val = -1;
    uint8_t *ptr = memory_region_get_ram_ptr(&isa->mmio);
    int isromaccess = 0;
    // ONLY WORKS IF rombase is placed over the end of mmio as in actual card.
    if ((!isa->romshadowed) && (addr >= (isa->rombase - isa->mmiobase)))
    {
        ptr = memory_region_get_ram_ptr(&isa->rom);
        addr -= (isa->rombase - isa->mmiobase);
        isromaccess = 1;
    }
    if (!isromaccess)
    {
	    if (isa->mmiosize == 32)
		    addr += RAM_SIZE;

	    if (isa->nicsr & NICSR_BS)
		    addr = (addr + RAM_SIZE) % (2 * RAM_SIZE);
    }

    switch(size) {
    case 4:
        /* 32 bit read access */
        val = *((uint32_t*)(ptr+addr));
        break;
    case 2:
        /* 16 bit read access */
        val = *((uint16_t*)(ptr+addr));
        break;
    case 1:
        /* 8 bit read access */
        val = *((uint8_t*)(ptr+addr));
        break;
    default:
        hw_error(DE100_DEVICE ": unexpected %s with size = %u", __func__, size);
        break;
    }
    if (isromaccess) {
        DPRINTF_ROM("ROM  read: addr=0x%" PRIx64 ", val=0x%" PRIx64 ", size=%d\n", addr, val, size);
    } else {
        DPRINTF_MMIO("MMIO read: addr=0x%" PRIx64 ", val=0x%" PRIx64 ", size=%d (%d)\n", addr, val, size, !!(isa->nicsr & NICSR_BS));
    }
    return val;
}
static const MemoryRegionOps de100_mmio_ops = {
    .read = de100_mmio_read,
    .write = de100_mmio_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
};
#endif /* DEBUG_DE100_MMIO || DEBUG_DE100_ROM */

/**
 * cleanup function, registered in net_de100_isa_info structure.
 * only used to call pcnet cleanup function: pcnet_common_cleanup.
 */
static void de100_isa_cleanup(VLANClientState *nc)
{
    PCNetState *d = DO_UPCAST(NICState, nc, nc)->opaque;

    pcnet_common_cleanup(d);
}

static NetClientInfo net_de100_isa_info = {
    .type = NET_CLIENT_OPTIONS_KIND_NIC,
    .size = sizeof(NICState),
    .can_receive = pcnet_can_receive,
    .receive = pcnet_receive,
    .link_status_changed = pcnet_set_link_status,
    .cleanup = de100_isa_cleanup,
};

static const VMStateDescription de100_isa_vmstate = {
    .name = "pcnet",
    .version_id = 3,
    .minimum_version_id = 2,
    .minimum_version_id_old = 2,
    .fields      = (VMStateField []) {
        VMSTATE_STRUCT(state, ISADE100State, 0, vmstate_pcnet, PCNetState),
        VMSTATE_END_OF_LIST()
    }
};

/**
 * Write data into the memory of the device
 * Used for access by pcnet emulation (lance) not CPU.
 *
 * @param dma_opaque Device state (ISADE100State)
 * @param addr relative address of the data to write
 * @param buf data to write
 * @param len length of the data
 * @param do_bswap not used
 */
static void isa_physical_memory_write(void *dma_opaque, target_phys_addr_t addr,
                                      uint8_t *buf, int len, int do_bswap)
{
    ISADE100State* isa = (ISADE100State*) dma_opaque;
    uint8_t *ptr = memory_region_get_ram_ptr(&isa->mmio);
    target_phys_addr_t addr2=addr&LA_MASK;
    if ((addr2+len)>(2*RAM_SIZE))
    {
        fprintf(stderr, "de100: LANCE reading out of memory boundaries addr=0x%" PRIx64 " len=%d\n", addr2, len);
        exit(1);
    }

    memcpy(ptr+addr2, buf, len);
    memory_region_set_dirty(&isa->mmio, addr2, len);

#ifdef DEBUG_DE100_MMIO
#ifndef DEBUG_DE100_MMIO_LANCE_VERBOSE
    DPRINTF_MMIO("lance mmio write: addr=0x%" PRIx64 " len=%d\n", addr2, len);
#else
    {
    int i;
        DPRINTF_MMIO("lance mmio write: addr=0x%" PRIx64 " len=%d data", addr2, len);
        for(i=0; i<len; i++){
            fprintf(stderr,",0x%02x",buf[i]);
        }
        fprintf(stderr,"\n");
    }
#endif
#endif

}

/**
 * Read data from the memory of the device
 * Used for access by pcnet emulation (lance) not CPU.
 *
 * @param dma_opaque Device state (ISADE100State)
 * @param addr relative address of the data to read
 * @param buf data to read
 * @param len length of the data
 * @param do_bswap not used
 */
static void isa_physical_memory_read(void *dma_opaque, target_phys_addr_t addr,
                                     uint8_t *buf, int len, int do_bswap)
{
    ISADE100State* isa = (ISADE100State*) dma_opaque;
    uint8_t *ptr = memory_region_get_ram_ptr(&isa->mmio);
    target_phys_addr_t addr2=addr&LA_MASK;
    if ((addr2+len)>(2*RAM_SIZE))
    {
        fprintf(stderr, "de100: LANCE reading out of memory boundaries addr=0x%" PRIx64 " len=%d\n", addr2, len);
        exit(1);
    }

    memcpy(buf, ptr+addr2, len);

#ifdef DEBUG_DE100_MMIO
#ifndef DEBUG_DE100_MMIO_LANCE_VERBOSE
    DPRINTF_MMIO("lance mmio read: addr=0x%" PRIx64 " len=%d\n", addr2, len);
#else
    {
        int i;
        DPRINTF_MMIO("lance mmio read: addr=0x%" PRIx64 " len=%d data", addr2, len);
        for(i=0; i<len; i++){
            fprintf(stderr,",0x%02x",buf[i]);
        }
        fprintf(stderr,"\n");
    }
#endif
#endif
}

/**
 * DE100 IRQ handler, for masking interrupt if required by the NICSR
 * register
 */
static void de100_isa_request_irq(void *opaque, int irq, int level)
{
    ISADE100State *isa = opaque;

    DPRINTF_INT("de100: IRQ line update. NICSR=0x%X, level=%i\n",
	    isa->nicsr, level);

    /* Is interrupt inactive */
    if (!(isa->nicsr & NICSR_IEN))
	    return;

    /* Is interrupt line forced inactive */
    if (isa->nicsr & NICSR_IM)
	    level = 0;

    qemu_set_irq(isa->irq, level);
}

/**
 * Initialize the device
 *
 * @param dev ISA Device
 *
 * @return 0 on success
 */
static int de100_isa_init(ISADevice *dev)
{
    ISADE100State *isa = DO_UPCAST(ISADE100State, dev, dev);
    PCNetState *s = &isa->state;
    qemu_irq* irq = NULL;

    if (!isa->romfile) {
        isa->romfile = (char*)DE100_ROMFILE;
    }

    if (s->conf.vlan) {
        DPRINTF("initialization (DE100 in vlan%d)\n",
                s->conf.vlan->id);
        DPRINTF("iobase=0x%x, irq=%d, mmiobase=0x%x, mmiosize=%d, rombase=0x%x, romfile='%s'\n",
                isa->iobase, isa->isairq, isa->mmiobase, isa->mmiosize,
		isa->rombase, isa->romfile);
    }

    /* Handler for memory-mapped I/O */
    memory_region_init_io(&isa->io, &de100_io_ops, isa, "de100-io", 0x10);
    isa_register_ioport(dev, &isa->io, isa->iobase);

    /* Allocate the total DE100 memory of two 32KB RAM. */
    memory_region_init_ram(&isa->mmio, "de100-mmio", 2 * RAM_SIZE);
    vmstate_register_ram_global(&isa->mmio);


    if (isa->mmiobase == 32)
    {
    	isa->mmio_zone2_addr = isa->mmiobase + RAM_OFFSET;
    	isa->mmio_zone1_addr = isa->mmiobase;
    }
    else
    {
    	isa->mmio_zone2_addr = isa->mmiobase;
    	isa->mmio_zone1_addr = isa->mmiobase + RAM_OFFSET;
    }

#ifdef DEBUG_DE100_MMIO
    memory_region_init_io(&isa->mmio_dbg, &de100_mmio_ops, isa,
			  "de100-mmio-dbg", isa->mmiosize * 1024);

    memory_region_add_subregion_overlap(isa_address_space(dev),
                                        isa->mmiobase, &isa->mmio_dbg, 2);
#endif /* DEBUG_DE100_MMIO */

    isa->ramselected = 0;
    memory_region_init_alias(&isa->mmio_bank1, "mmio-bank1", &isa->mmio,
			     ALIAS_START, ALIAS_SIZE);

    memory_region_init_alias(&isa->mmio_bank2, "mmio-bank2", &isa->mmio,
			     RAM_OFFSET + ALIAS_START, ALIAS_SIZE);


    /* If we are in a 32 KB configuration, we only have the upper 32KB */
    /* mapped, when BS=0, and the lower ones when BS=1. */
    memory_region_add_subregion_overlap(isa_address_space(dev),
    					isa->mmio_zone2_addr +
					ALIAS_START,
					&isa->mmio_bank2, 2);
    if (isa->mmiosize == 64)
	memory_region_add_subregion_overlap(isa_address_space(dev),
					    isa->mmio_zone1_addr +
					    ALIAS_START,
					    &isa->mmio_bank1, 2);

    /* Handler for rom */
    memory_region_init_ram(&isa->rom, "de100-rom", 16 * 1024);
    vmstate_register_ram_global(&isa->rom);
    isa->romshadowed = 0;
    memory_region_add_subregion_overlap(isa_address_space(dev),
                                        isa->rombase, &isa->rom, 3);

    char *filename=qemu_find_file(QEMU_FILE_TYPE_BIOS, isa->romfile);
    int ret = load_image_targphys(filename, isa->rombase, ROM_SIZE);
    if (ret <= 0) {
        fprintf(stderr, "de100: could not load romfile '%s'\n", isa->romfile);
        exit(1);
    }
    memory_region_set_readonly(&isa->rom, true);
#ifdef DEBUG_DE100_ROM
    /* When debugging we don't want to map the ROM except during init for
       QEMU to load its content! */
    isa->romshadowed = 2;
#endif /* DEBUG_DE100_ROM */

    /* Give the Lance our interrupt handler before passing it to this isa
     * bus, if required, accordingly to the NICSR register.
     */
    isa_init_irq(dev, &isa->irq, isa->isairq);
    irq = qemu_allocate_irqs(de100_isa_request_irq, isa, 1);
    s->irq = irq[0];

    s->phys_mem_read = isa_physical_memory_read;
    s->phys_mem_write = isa_physical_memory_write;
    s->dma_opaque = isa;

    isa->nicsr = ((isa->mmiosize == 32) ? NICSR_BUF : 0);
    isa->signature_pos = 0;
    isa->eth_addr_pos = 0;

    return pcnet_common_init(&dev->qdev, s, &net_de100_isa_info);
}

/**
 * Reset function, complete the initialization.
 */
static void de100_isa_reset(DeviceState *dev)
{
    unsigned int i = 0;
    ISADE100State *isa = DO_UPCAST(ISADE100State, dev.qdev, dev);

    pcnet_h_reset(&isa->state);
#ifdef DEBUG_DE100_ROM
    /* At reset the ROM has been loaded by QEMU, in debug mode we can
     * unmap it. */
    if(isa->romshadowed == 2)
    {
        DPRINTF_ROM("ROM init end.\n");
        memory_region_del_subregion(isa_address_space(&isa->dev), &isa->rom);
        isa->romshadowed = 0;
    }
#endif /* DEBUG_DE100_ROM */
    isa->nicsr = (isa->mmiosize == 32) ? NICSR_BUF : 0;
    de100_ram_shadow_update(isa);
    isa->signature_pos = 0;

    /* After signature add MAC addr */
    for (i = 8; i < ROM_SIGNATURE_SIZE; ++i)
	rom_signature[i] = de100_get_hw_addr(isa);
    isa->eth_addr_pos = 0;
}

static Property de100_isa_properties[] = {
    DEFINE_PROP_HEX32("iobase", ISADE100State, iobase, 0x300),
    DEFINE_PROP_UINT32("irq", ISADE100State, isairq, 2),
    DEFINE_PROP_HEX32("mmiobase", ISADE100State, mmiobase, 0xd8000),
    DEFINE_PROP_UINT8("mmiosize", ISADE100State, mmiosize, 32),
    DEFINE_PROP_HEX32("rombase", ISADE100State, rombase, 0xdc000),
    DEFINE_PROP_STRING("romfile", ISADE100State, romfile),
    DEFINE_NIC_PROPERTIES(ISADE100State, state.conf),
    DEFINE_PROP_END_OF_LIST(),
};

static void de100_isa_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ISADeviceClass *ic = ISA_DEVICE_CLASS(klass);

    ic->init = de100_isa_init;
    dc->reset = de100_isa_reset;
    dc->vmsd = &de100_isa_vmstate;
    dc->props = de100_isa_properties;
}

static TypeInfo de100_isa_info = {
    .name = "de100_isa",
    .parent = TYPE_ISA_DEVICE,
    .instance_size = sizeof(ISADE100State),
    .class_init = de100_isa_class_init,
};

static void de100_isa_register_types(void)
{
    type_register_static(&de100_isa_info);
}

type_init(de100_isa_register_types)
