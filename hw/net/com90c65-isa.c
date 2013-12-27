/*
 * QEMU ARCnet COM90C66 emulation
 *
 * This source file contains the ISA declaration of the network card. To work,
 * it uses the generic code of the file com90c65.c.
 */

#include "hw/hw.h"
#include "hw/isa/isa.h"
#include "hw/i386/pc.h"
#include "monitor/qdev.h"
#include "net/net.h"
#include "hw/net/com90c65.h"

/* default I/O addresses */
#define IO_BASES_MAX 8
static int io_bases[IO_BASES_MAX] = {0x260, 0x290, 0x2e0, 0x2f0, 0x300, 0x350, 0x380, 0x3e0};

typedef struct ISACOM90C65State {
    ISADevice dev;
    uint32_t iobase;
    uint32_t isairq;
    uint32_t mmiobase;
    uint32_t nodeid;
    COM90C65State com90c65;
} ISACOM90C65State;

static void isa_com90c65_cleanup(NetClientState *nc)
{
    COM90C65State *s = DO_UPCAST(NICState, nc, nc)->opaque;

    s->nic = NULL;
}

static NetClientInfo net_com90c65_isa_info = {
    .type = NET_CLIENT_OPTIONS_KIND_NIC,
    .size = sizeof (NICState),
    .can_receive = com90c65_can_receive,
    .receive = com90c65_receive,
    .cleanup = isa_com90c65_cleanup,
};

/* convert I/O address to an index value in the dedicated register */
static int com90c65_set_io_select(COM90C65State *s, int io_base)
{
    int index ;
    for (index = 0 ; index < IO_BASES_MAX ; index ++) {
        if (io_base == io_bases[index]) {
            s->io_select = index ;
            return 0 ;
        }
    }
    return -1 ;
}

static int isa_com90c65_initfn(ISADevice *dev) {
    ISACOM90C65State *isa = DO_UPCAST(ISACOM90C65State, dev, dev);
    COM90C65State *s = &isa->com90c65;

    if (s->c.vlan) {
        dprintf("initialization (" COM90C65_DEVICE " in vlan%d)\n",
                s->c.vlan->id);
        dprintf("iobase=0x%x, irq=%d, mmiobase=0x%x, nodeid=%d\n",
                isa->iobase, isa->isairq, isa->mmiobase, isa->nodeid);
    }

    if (com90c65_set_io_select(s, isa->iobase) != 0) {
        eprintf("bad I/O address: %X\n", isa->iobase);
        return -1;
    }

    com90c65_setup_io(s, REGISTERS_SIZE);
    isa_register_ioport(dev, &s->io, isa->iobase);
    com90c65_setup_mmio(s, MEM_SIZE);
    memory_region_add_subregion_overlap(isa_address_space(dev),
                                        isa->mmiobase, &s->mmio, 2);

    isa_init_irq(dev, &s->irq, isa->isairq);

    if (isa->nodeid > UCHAR_MAX)
        qemu_macaddr_default_if_unset(&s->c.macaddr);
    else {
        memset(s->c.macaddr.a, 0, 5);
        s->c.macaddr.a[5] = isa->nodeid;
    }
    s->mem_select = 0 ; /* not used */
    s->node_id = s->c.macaddr.a[5];
    com90c65_reset(s);

    /* initialize read/write accesses to the registers for easy implementation of the read/write functions */
    com90c65_registers_access_init(s);

    /* register a hard reset used by qemu */
    qemu_register_reset(com90c65_reset, s);

    s->nic = qemu_new_nic(&net_com90c65_isa_info, &s->c,
                          object_get_typename(OBJECT(dev)), dev->qdev.id, s);

    return 0;
}

static Property com90c65_isa_properties[] = {
    DEFINE_PROP_HEX32("iobase", ISACOM90C65State, iobase, 0x2e0),
    DEFINE_PROP_UINT32("irq", ISACOM90C65State, isairq, 5),
    DEFINE_PROP_HEX32("mmiobase", ISACOM90C65State, mmiobase, 0xd0000),
    DEFINE_PROP_UINT32("nodeid", ISACOM90C65State, nodeid, UCHAR_MAX + 1),
    DEFINE_NIC_PROPERTIES(ISACOM90C65State, com90c65.c),
    DEFINE_PROP_END_OF_LIST(),
};

static void isa_com90c65_class_initfn(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ISADeviceClass *ic = ISA_DEVICE_CLASS(klass);
    ic->init = isa_com90c65_initfn;
    dc->props = com90c65_isa_properties;
}

static TypeInfo com90c65_isa_info = {
    .name          = "com90c65_isa",
    .parent        = TYPE_ISA_DEVICE,
    .instance_size = sizeof (ISACOM90C65State),
    .class_init    = isa_com90c65_class_initfn,
};

static void com90c65_isa_register_types(void)
{
    type_register_static(&com90c65_isa_info);
}

type_init(com90c65_isa_register_types)
