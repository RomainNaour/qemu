/*
 * QEMU ARCNet user mode
 */

#ifndef QEMU_NET_USER_ARCNET_H
#define QEMU_NET_USER_ARCNET_H

#include "net.h"
#include "qemu-common.h"

int net_init_user_arcnet(QemuOpts *opts, const char *name, VLANState *vlan);

#endif /* QEMU_NET_USER_ARCNET_H */
