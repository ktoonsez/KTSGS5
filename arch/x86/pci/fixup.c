
/*
 * Twinhead H12Y needs us to block out a region otherwise we map devices
 * there and any access kills the box.
 *
 *   See: https://bugzilla.kernel.org/show_bug.cgi?id=10231
 *
 * Match off the LPC and svid/sdid (older kernels lose the bridge subvendor)
 */
static void __devinit twinhead_reserve_killing_zone(struct pci_dev *dev)
{
        if (dev->subsystem_vendor == 0x14FF && dev->subsystem_device == 0xA003) {
                pr_info("Reserving memory on Twinhead H12Y\n");
                request_mem_region(0xFFB00000, 0x100000, "twinhead");
        }
}
DECLARE_PCI_FIXUP_HEADER(PCI_VENDOR_ID_INTEL, 0x27B9, twinhead_reserve_killing_zone);
