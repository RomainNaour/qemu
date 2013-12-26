/*
 * QEMU ARCNet user mode
 *
 * This is a layer/tunnel which connects
 * an emulated arcnet device and a Linux arcnet interface of the host.
 *
 * It communicates with the upper layer (the emulated device) by a Qemu vlan
 * and with the bottom layer (the real device) by a raw socket.
 * It converts packets between the real format and the Linux one.
 * A send comes from the vlan to the socket.
 * A receive comes from the socket to the vlan.
 */

#include <err.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <net/if_arp.h>
#include <linux/if_arcnet.h>

#include "qemu-char.h"

#include "user-arcnet.h"
#include "arcnet.h"

#ifdef USER_ARCNET_DEBUG
/* set it to 1 to print send/receive events */
# define USER_ARCNET_DEBUG_RXTX 0
#else
# define USER_ARCNET_DEBUG_RXTX 0
#endif

#ifdef USER_ARCNET_DEBUG
# define dprintf0(fmt, args...) fprintf(stderr, fmt, ## args)
#else
# define dprintf0(fmt, args...) do {} while (0)
#endif
#define dprintf(fmt, args...) dprintf0("arcnet: " fmt, ## args)
#define dperror(fmt, args...) fprintf(stderr, "%s: " fmt "ERROR %d: %s\n", __FUNCTION__, ## args, errno, strerror(errno))

typedef struct archdr ArcnetPacket;

/* arcnet connection between the emulator and the host interface */
typedef struct UserArcnetState {
    VLANClientState nc;
    int fd;
    struct sockaddr address;
} UserArcnetState;

/* send a packet from the emulator vlan to the host */
static ssize_t net_user_arcnet_receive(VLANClientState *nc, const uint8_t *buf, size_t size)
{
    UserArcnetState *s = DO_UPCAST(UserArcnetState, nc, nc);
    ArcnetPacket *linux_buf = (ArcnetPacket*)buf;
    int data_length, length;
    ssize_t sent;

    /* ignore bad packets */
    if (buf == NULL || size <= 0 || size != ARCNET_SIZE(buf)) {
        return -1;
    }

    /* convert from hardware to linux format (see linux/if_arcnet.h),
     * move data to the beginning of the buffer */
    data_length = ARCNET_DATA_LENGTH(buf);
    memmove(linux_buf->soft.raw, ARCNET_DATA(buf), data_length);
    length = data_length + (linux_buf->soft.raw - (uint8_t*) linux_buf);

    if (USER_ARCNET_DEBUG_RXTX)
        dprintf("send %d bytes\n", length);

    /* send raw packet to the predefined interface */
    sent = sendto(s->fd, buf, length, 0, &s->address, sizeof (s->address));
    if (sent != length) {
        dperror();
        return -1;
    }
    return sent;
}

/* cannot receive packet if the device is not ready to receive */
static int net_user_arcnet_can_send(void *opaque)
{
    UserArcnetState *s = opaque;
    return qemu_can_send_packet(&s->nc);
}

/* receive a packet from the host to the emulator vlan */
static void net_user_arcnet_send(void *opaque)
{
    UserArcnetState *s = opaque;
    uint8_t buf[ARCNET_MAX_SIZE];
    ArcnetPacket *linux_buf = (ArcnetPacket*)buf;
    ssize_t received;
    int data_length, size;

    /* receive raw packet from the predefined interface */
    received = qemu_recv(s->fd, buf, ARCNET_MAX_SIZE, 0);
    if (received <= 0) {
        dperror();
        return;
    }

    if (USER_ARCNET_DEBUG_RXTX)
        dprintf("receive %d bytes\n", received);

    /* get data length */
    data_length = received - (linux_buf->soft.raw - (uint8_t*) linux_buf);
    if (data_length <= ARCNET_SHORT_DATA_MAX_SIZE) {
        size = ARCNET_SHORT_SIZE;
    } else if (data_length < ARCNET_LONG_DATA_MIN_SIZE) { /* should not happen */
        size = ARCNET_LONG_SIZE;
        /* data length between short and long => add padding to make a long packet */
        data_length = ARCNET_LONG_DATA_MIN_SIZE;
    } else if (data_length <= ARCNET_LONG_DATA_MAX_SIZE) {
        size = ARCNET_LONG_SIZE;
    } else { /* message too long */
        errno = EMSGSIZE;
        dperror();
        return;
    }
    /* write data length in the header */
    buf[ARCNET_SHORT_START] = 0;
    buf[ARCNET_LONG_START] = 0;
    buf[ARCNET_START(size)] = size - data_length;
    /* convert from linux to hardware format (see linux/if_arcnet.h),
     * move data to the end of the buffer */
    memmove(ARCNET_DATA(buf), linux_buf->soft.raw, data_length);

    /* send packet to the vlan */
    qemu_send_packet(&s->nc, buf, size);
}

static void net_user_arcnet_cleanup(VLANClientState *nc)
{
    UserArcnetState *s = DO_UPCAST(UserArcnetState, nc, nc);
    qemu_set_fd_handler(s->fd, NULL, NULL, NULL);
    close(s->fd);
}

static NetClientInfo net_user_arcnet_info = {
    .type = NET_CLIENT_OPTIONS_KIND_USER_ARCNET,
    .size = sizeof (UserArcnetState),
    .receive = net_user_arcnet_receive,
    .cleanup = net_user_arcnet_cleanup,
};

int net_init_user_arcnet(const NetClientOptions *opts, const char *name,
                         VLANClientState *peer)
{
    VLANClientState *nc;
    UserArcnetState *s;
    const NetdevUserArcnetOptions *userArcnet;
    int id;
    int ret;

    assert(opts->kind == NET_CLIENT_OPTIONS_KIND_USER_ARCNET);
    userArcnet = opts->userArcnet;
    
    assert(peer);

    ret = net_hub_id_for_client(peer, &id);
    assert(ret == 0); /* peer must be on a hub */

    dprintf("initialization (%s in vlan%d)\n", userArcnet->ifname, id);

    nc = qemu_new_net_client(&net_user_arcnet_info, NULL, peer, "user-arcnet", name);

    s = DO_UPCAST(UserArcnetState, nc, nc);

    /* open a raw socket */
    s->fd = socket(PF_PACKET, SOCK_PACKET, htons(ETH_P_ARCNET));
    if (s->fd < 0) {
        dperror();
        return -1;
    }
    /* store address of the specified interface */
    s->address.sa_family = ARPHRD_ARCNET;
    strcpy(s->address.sa_data, userArcnet->ifname);

    /* listen the specified interface */
    if (bind(s->fd, &s->address, sizeof (s->address))) {
        dperror("bind on %s: ", userArcnet->ifname);
        return -1;
    }

    /* set a handler to receive data from the ARCnet network */
    qemu_set_fd_handler2(s->fd, net_user_arcnet_can_send,
                         net_user_arcnet_send, NULL, s);

    snprintf(nc->info_str, sizeof (nc->info_str), "arcnet user mode");
    dprintf("vlan=%d, interface=%s, fd=%d\n", id, userArcnet->ifname, s->fd);

    return 0;
}
