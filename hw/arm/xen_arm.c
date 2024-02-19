/*
 * QEMU ARM Xen PVH Machine
 *
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "qemu/osdep.h"
#include "qemu/error-report.h"
#include "qapi/qapi-commands-migration.h"
#include "qapi/visitor.h"
#include "hw/boards.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "sysemu/block-backend.h"
#include "sysemu/tpm_backend.h"
#include "sysemu/sysemu.h"
#include "hw/xen/xen-hvm-common.h"
#include "sysemu/tpm.h"
#include "hw/xen/arch_hvm.h"
#include "trace.h"
#include "hw/pci-host/gpex.h"

#define TYPE_XEN_ARM  MACHINE_TYPE_NAME("xenpvh")
OBJECT_DECLARE_SIMPLE_TYPE(XenArmState, XEN_ARM)

static const MemoryListener xen_memory_listener = {
    .region_add = xen_region_add,
    .region_del = xen_region_del,
    .log_start = NULL,
    .log_stop = NULL,
    .log_sync = NULL,
    .log_global_start = NULL,
    .log_global_stop = NULL,
    .priority = MEMORY_LISTENER_PRIORITY_ACCEL,
};

struct XenArmState {
    /*< private >*/
    MachineState parent;

    XenIOState *state;

    struct {
        uint64_t tpm_base_addr;
        MemMapEntry pcie_mmio;
        MemMapEntry pcie_ecam;
        MemMapEntry pcie_mmio_high;
    } cfg;
};

static MemoryRegion ram_lo, ram_hi;

/*
 * VIRTIO_MMIO_DEV_SIZE is imported from tools/libs/light/libxl_arm.c under Xen
 * repository.
 *
 * Origin: git://xenbits.xen.org/xen.git 2128143c114c
 */
#define VIRTIO_MMIO_DEV_SIZE   0x200

#define NR_VIRTIO_MMIO_DEVICES   \
   (GUEST_VIRTIO_MMIO_SPI_LAST - GUEST_VIRTIO_MMIO_SPI_FIRST)

static void xen_set_irq(void *opaque, int irq, int level)
{
    if (xendevicemodel_set_irq_level(xen_dmod, xen_domid, irq, level)) {
        error_report("xendevicemodel_set_irq_level failed");
    }
}

static void xen_create_virtio_mmio_devices(XenArmState *xam)
{
    int i;

    for (i = 0; i < NR_VIRTIO_MMIO_DEVICES; i++) {
        hwaddr base = GUEST_VIRTIO_MMIO_BASE + i * VIRTIO_MMIO_DEV_SIZE;
        qemu_irq irq = qemu_allocate_irq(xen_set_irq, NULL,
                                         GUEST_VIRTIO_MMIO_SPI_FIRST + i);

        sysbus_create_simple("virtio-mmio", base, irq);

        trace_xen_create_virtio_mmio_devices(i,
                                             GUEST_VIRTIO_MMIO_SPI_FIRST + i,
                                             base);
    }
}

static void xen_init_ram(MachineState *machine)
{
    MemoryRegion *sysmem = get_system_memory();
    ram_addr_t block_len, ram_size[GUEST_RAM_BANKS];

    trace_xen_init_ram(machine->ram_size);
    if (machine->ram_size <= GUEST_RAM0_SIZE) {
        ram_size[0] = machine->ram_size;
        ram_size[1] = 0;
        block_len = GUEST_RAM0_BASE + ram_size[0];
    } else {
        ram_size[0] = GUEST_RAM0_SIZE;
        ram_size[1] = machine->ram_size - GUEST_RAM0_SIZE;
        block_len = GUEST_RAM1_BASE + ram_size[1];
    }

    memory_region_init_ram(&ram_memory, NULL, "xen.ram", block_len,
                           &error_fatal);

    memory_region_init_alias(&ram_lo, NULL, "xen.ram.lo", &ram_memory,
                             GUEST_RAM0_BASE, ram_size[0]);
    memory_region_add_subregion(sysmem, GUEST_RAM0_BASE, &ram_lo);
    if (ram_size[1] > 0) {
        memory_region_init_alias(&ram_hi, NULL, "xen.ram.hi", &ram_memory,
                                 GUEST_RAM1_BASE, ram_size[1]);
        memory_region_add_subregion(sysmem, GUEST_RAM1_BASE, &ram_hi);
    }

    DPRINTF("init grant ram mapping for XEN\n");
    ram_grants = *xen_init_grant_ram();
}

static bool xen_validate_pcie_config(XenArmState *xam)
{
    if (xam->cfg.pcie_ecam.base == 0 &&
        xam->cfg.pcie_ecam.size == 0 &&
        xam->cfg.pcie_mmio.base == 0 &&
        xam->cfg.pcie_mmio.size == 0 &&
        xam->cfg.pcie_mmio_high.base == 0 &&
        xam->cfg.pcie_mmio_high.size == 0) {
        /* It's okay, user just don't want PCIe brige */
        return false;
    }

    if (xam->cfg.pcie_ecam.base == 0 ||
        xam->cfg.pcie_ecam.size == 0 ||
        xam->cfg.pcie_mmio.base == 0 ||
        xam->cfg.pcie_mmio.size == 0 ||
        xam->cfg.pcie_mmio_high.base == 0 ||
        xam->cfg.pcie_mmio_high.size == 0) {
        /* User provided some PCIe options, but not all of them */
        error_printf("Incomplete PCIe bridge configuration\n");
        exit(1);
    }

    return true;
}

static void xen_create_pcie(XenArmState *xam)
{
    MemoryRegion *mmio_alias, *mmio_alias_high, *mmio_reg;
    MemoryRegion *ecam_alias, *ecam_reg;
    DeviceState *dev;
    int i;

    dev = qdev_new(TYPE_GPEX_HOST);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);

    /* Map ECAM space */
    ecam_alias = g_new0(MemoryRegion, 1);
    ecam_reg = sysbus_mmio_get_region(SYS_BUS_DEVICE(dev), 0);
    memory_region_init_alias(ecam_alias, OBJECT(dev), "pcie-ecam",
                             ecam_reg, 0, xam->cfg.pcie_ecam.size);
    memory_region_add_subregion(get_system_memory(), xam->cfg.pcie_ecam.base,
                                ecam_alias);

    /* Map the MMIO space */
    mmio_alias = g_new0(MemoryRegion, 1);
    mmio_reg = sysbus_mmio_get_region(SYS_BUS_DEVICE(dev), 1);
    memory_region_init_alias(mmio_alias, OBJECT(dev), "pcie-mmio",
                             mmio_reg,
                             xam->cfg.pcie_mmio.base,
                             xam->cfg.pcie_mmio.size);
    memory_region_add_subregion(get_system_memory(), xam->cfg.pcie_mmio.base,
                                mmio_alias);

    /* Map the MMIO_HIGH space */
    mmio_alias_high = g_new0(MemoryRegion, 1);
    memory_region_init_alias(mmio_alias_high, OBJECT(dev), "pcie-mmio-high",
                             mmio_reg,
                             xam->cfg.pcie_mmio_high.base,
                             xam->cfg.pcie_mmio_high.size);
    memory_region_add_subregion(get_system_memory(),
                                xam->cfg.pcie_mmio_high.base,
                                mmio_alias_high);

    /* Legacy PCI interrupts (#INTA - #INTD) */
    for (i = 0; i < GPEX_NUM_IRQS; i++) {
        qemu_irq irq = qemu_allocate_irq(xen_set_irq, NULL,
                                         GUEST_VIRTIO_PCI_SPI_FIRST + i);

        sysbus_connect_irq(SYS_BUS_DEVICE(dev), i, irq);
        gpex_set_irq_num(GPEX_HOST(dev), i, GUEST_VIRTIO_PCI_SPI_FIRST + i);
    }
}

void arch_handle_ioreq(XenIOState *state, ioreq_t *req)
{
    hw_error("Invalid ioreq type 0x%x\n", req->type);

    return;
}

void arch_xen_set_memory(XenIOState *state, MemoryRegionSection *section,
                         bool add)
{
}

void xen_hvm_modified_memory(ram_addr_t start, ram_addr_t length)
{
}

void qmp_xen_set_global_dirty_log(bool enable, Error **errp)
{
}

#ifdef CONFIG_TPM
static void xen_enable_tpm(XenArmState *xam)
{
    Error *errp = NULL;
    DeviceState *dev;
    SysBusDevice *busdev;

    TPMBackend *be = qemu_find_tpm_be("tpm0");
    if (be == NULL) {
        error_report("Couldn't find tmp0 backend");
        return;
    }
    dev = qdev_new(TYPE_TPM_TIS_SYSBUS);
    object_property_set_link(OBJECT(dev), "tpmdev", OBJECT(be), &errp);
    object_property_set_str(OBJECT(dev), "tpmdev", be->id, &errp);
    busdev = SYS_BUS_DEVICE(dev);
    sysbus_realize_and_unref(busdev, &error_fatal);
    sysbus_mmio_map(busdev, 0, xam->cfg.tpm_base_addr);

    trace_xen_enable_tpm(xam->cfg.tpm_base_addr);
}
#endif

static void xen_arm_init(MachineState *machine)
{
    XenArmState *xam = XEN_ARM(machine);

    xam->state =  g_new0(XenIOState, 1);

    if (machine->ram_size == 0) {
        warn_report("%s non-zero ram size not specified. QEMU machine started"
                    " without IOREQ (no emulated devices including virtio)",
                    MACHINE_CLASS(object_get_class(OBJECT(machine)))->desc);
        return;
    }

    xen_init_ram(machine);

    xen_register_ioreq(xam->state, machine->smp.cpus, &xen_memory_listener);

    xen_create_virtio_mmio_devices(xam);

    if (xen_validate_pcie_config(xam)) {
        xen_create_pcie(xam);
    } else {
        DPRINTF("PCIe host bridge is not configured,"
                " only virtio-mmio can be used\n");
    }

#ifdef CONFIG_TPM
    if (xam->cfg.tpm_base_addr) {
        xen_enable_tpm(xam);
    } else {
        warn_report("tpm-base-addr is not provided. TPM will not be enabled");
    }
#endif
}

#ifdef CONFIG_TPM
static void xen_arm_get_tpm_base_addr(Object *obj, Visitor *v,
                                      const char *name, void *opaque,
                                      Error **errp)
{
    XenArmState *xam = XEN_ARM(obj);
    uint64_t value = xam->cfg.tpm_base_addr;

    visit_type_uint64(v, name, &value, errp);
}

static void xen_arm_set_tpm_base_addr(Object *obj, Visitor *v,
                                      const char *name, void *opaque,
                                      Error **errp)
{
    XenArmState *xam = XEN_ARM(obj);
    uint64_t value;

    if (!visit_type_uint64(v, name, &value, errp)) {
        return;
    }

    xam->cfg.tpm_base_addr = value;
}
#endif

static void xen_arm_get_pcie_ecam_base_addr(Object *obj, Visitor *v,
                                           const char *name, void *opaque,
                                           Error **errp)
{
    XenArmState *xam = XEN_ARM(obj);
    uint64_t value = xam->cfg.pcie_ecam.base;

    visit_type_uint64(v, name, &value, errp);
}

static void xen_arm_set_pcie_ecam_base_addr(Object *obj, Visitor *v,
                                           const char *name, void *opaque,
                                           Error **errp)
{
    XenArmState *xam = XEN_ARM(obj);
    uint64_t value;

    if (!visit_type_uint64(v, name, &value, errp)) {
        return;
    }

    xam->cfg.pcie_ecam.base = value;
}

static void xen_arm_get_pcie_ecam_size(Object *obj, Visitor *v,
                                      const char *name, void *opaque,
                                      Error **errp)
{
    XenArmState *xam = XEN_ARM(obj);
    uint64_t value = xam->cfg.pcie_ecam.size;

    visit_type_uint64(v, name, &value, errp);
}

static void xen_arm_set_pcie_ecam_size(Object *obj, Visitor *v,
                                      const char *name, void *opaque,
                                      Error **errp)
{
    XenArmState *xam = XEN_ARM(obj);
    uint64_t value;

    if (!visit_type_uint64(v, name, &value, errp)) {
        return;
    }

    xam->cfg.pcie_ecam.size = value;
}

static void xen_arm_get_pcie_mmio_base_addr(Object *obj, Visitor *v,
                                           const char *name, void *opaque,
                                           Error **errp)
{
    XenArmState *xam = XEN_ARM(obj);
    uint64_t value = xam->cfg.pcie_mmio.base;

    visit_type_uint64(v, name, &value, errp);
}

static void xen_arm_set_pcie_mmio_base_addr(Object *obj, Visitor *v,
                                           const char *name, void *opaque,
                                           Error **errp)
{
    XenArmState *xam = XEN_ARM(obj);
    uint64_t value;

    if (!visit_type_uint64(v, name, &value, errp)) {
        return;
    }

    xam->cfg.pcie_mmio.base = value;
}

static void xen_arm_get_pcie_mmio_size(Object *obj, Visitor *v,
                                      const char *name, void *opaque,
                                      Error **errp)
{
    XenArmState *xam = XEN_ARM(obj);
    uint64_t value = xam->cfg.pcie_mmio.size;

    visit_type_uint64(v, name, &value, errp);
}

static void xen_arm_set_pcie_mmio_size(Object *obj, Visitor *v,
                                      const char *name, void *opaque,
                                      Error **errp)
{
    XenArmState *xam = XEN_ARM(obj);
    uint64_t value;

    if (!visit_type_uint64(v, name, &value, errp)) {
        return;
    }

    xam->cfg.pcie_mmio.size = value;
}

static void xen_arm_get_pcie_prefetch_base_addr(Object *obj, Visitor *v,
                                               const char *name, void *opaque,
                                               Error **errp)
{
    XenArmState *xam = XEN_ARM(obj);
    uint64_t value = xam->cfg.pcie_mmio_high.base;

    visit_type_uint64(v, name, &value, errp);
}

static void xen_arm_set_pcie_prefetch_base_addr(Object *obj, Visitor *v,
                                               const char *name, void *opaque,
                                               Error **errp)
{
    XenArmState *xam = XEN_ARM(obj);
    uint64_t value;

    if (!visit_type_uint64(v, name, &value, errp)) {
        return;
    }

    xam->cfg.pcie_mmio_high.base = value;
}

static void xen_arm_get_pcie_prefetch_size(Object *obj, Visitor *v,
                                          const char *name, void *opaque,
                                          Error **errp)
{
    XenArmState *xam = XEN_ARM(obj);
    uint64_t value = xam->cfg.pcie_mmio_high.size;

    visit_type_uint64(v, name, &value, errp);
}

static void xen_arm_set_pcie_prefetch_size(Object *obj, Visitor *v,
                                          const char *name, void *opaque,
                                          Error **errp)
{
    XenArmState *xam = XEN_ARM(obj);
    uint64_t value;

    if (!visit_type_uint64(v, name, &value, errp)) {
        return;
    }

    xam->cfg.pcie_mmio_high.size = value;
}

static void xen_arm_machine_class_init(ObjectClass *oc, void *data)
{

    MachineClass *mc = MACHINE_CLASS(oc);
    mc->desc = "Xen Para-virtualized PC";
    mc->init = xen_arm_init;
    mc->max_cpus = 1;
    mc->default_machine_opts = "accel=xen";
    /* Set explicitly here to make sure that real ram_size is passed */
    mc->default_ram_size = 0;

#ifdef CONFIG_TPM
    object_class_property_add(oc, "tpm-base-addr", "uint64_t",
                              xen_arm_get_tpm_base_addr,
                              xen_arm_set_tpm_base_addr,
                              NULL, NULL);
    object_class_property_set_description(oc, "tpm-base-addr",
                                          "Set Base address for TPM device.");

    machine_class_allow_dynamic_sysbus_dev(mc, TYPE_TPM_TIS_SYSBUS);
#endif

    object_class_property_add(oc, "pci-ecam-base-addr", "uint64_t",
                              xen_arm_get_pcie_ecam_base_addr,
                              xen_arm_set_pcie_ecam_base_addr,
                              NULL, NULL);
    object_class_property_set_description(oc, "pci-ecam-base-addr",
                                          "Set Base address for PCI ECAM.");

    object_class_property_add(oc, "pci-ecam-size", "uint64_t",
                              xen_arm_get_pcie_ecam_size,
                              xen_arm_set_pcie_ecam_size,
                              NULL, NULL);
    object_class_property_set_description(oc, "pci-ecam-size",
                                          "Set Size for PCI ECAM.");

    object_class_property_add(oc, "pci-mmio-base-addr", "uint64_t",
                              xen_arm_get_pcie_mmio_base_addr,
                              xen_arm_set_pcie_mmio_base_addr,
                              NULL, NULL);
    object_class_property_set_description(oc, "pci-mmio-base-addr",
                                          "Set Base address for PCI MMIO.");

    object_class_property_add(oc, "pci-mmio-size", "uint64_t",
                              xen_arm_get_pcie_mmio_size,
                              xen_arm_set_pcie_mmio_size,
                              NULL, NULL);
    object_class_property_set_description(oc, "pci-mmio-size",
                                          "Set size for PCI MMIO.");

    object_class_property_add(oc, "pci-prefetch-base-addr", "uint64_t",
                              xen_arm_get_pcie_prefetch_base_addr,
                              xen_arm_set_pcie_prefetch_base_addr,
                              NULL, NULL);
    object_class_property_set_description(oc, "pci-prefetch-base-addr",
                                          "Set Prefetch Base address for PCI.");

    object_class_property_add(oc, "pci-prefetch-size", "uint64_t",
                              xen_arm_get_pcie_prefetch_size,
                              xen_arm_set_pcie_prefetch_size,
                              NULL, NULL);
    object_class_property_set_description(oc, "pci-prefetch-size",
                                          "Set Prefetch size for PCI.");
}

static const TypeInfo xen_arm_machine_type = {
    .name = TYPE_XEN_ARM,
    .parent = TYPE_MACHINE,
    .class_init = xen_arm_machine_class_init,
    .instance_size = sizeof(XenArmState),
};

static void xen_arm_machine_register_types(void)
{
    type_register_static(&xen_arm_machine_type);
}

type_init(xen_arm_machine_register_types)
