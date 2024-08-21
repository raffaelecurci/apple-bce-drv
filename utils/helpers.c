#include <linux/kernel.h>
#include <linux/usb.h>
#include <linux/usb/hcd.h>

#include <uapi/asm-generic/errno.h>

#include "helpers.h"
#include "../vhci/vhci.h"

const char *get_error_name2(int num)
{
    static const char *error_names[] = {
        "EPERM\0", "ENOENT\0", "ESRCH\0", "EINTR\0", "EIO\0", "ENXIO\0", "E2BIG\0", "ENOEXEC\0", "EBADF\0", "ECHILD\0",
        "EAGAIN\0", "ENOMEM\0", "EACCES\0", "EFAULT\0", "ENOTBLK\0", "EBUSY\0", "EEXIST\0", "EXDEV\0", "ENODEV\0", "ENOTDIR\0",
        "EISDIR\0", "EINVAL\0", "ENFILE\0", "EMFILE\0", "ENOTTY\0", "ETXTBSY\0", "EFBIG\0", "ENOSPC\0", "ESPIPE\0", "EROFS\0",
        "EMLINK\0", "EPIPE\0", "EDOM\0", "ERANGE\0"};

    static char buffer[16];

    int abs_num = abs(num);
    if (abs_num < 1 || abs_num > 34)
    {
        snprintf(buffer, sizeof(buffer), "unknown ERROR %d, no error ??", num);
        return buffer;
    }

    return error_names[abs_num - 1];
}

const char *get_error_name(int num) {
    static const struct {
        int num;
        const char *name;
    } error_map[] = {
        {URB_SHORT_NOT_OK, "URB_SHORT_NOT_OK"},
        {URB_ISO_ASAP, "URB_ISO_ASAP"},
        {URB_NO_TRANSFER_DMA_MAP, "URB_NO_TRANSFER_DMA_MAP"},
        {URB_NO_INTERRUPT, "URB_NO_INTERRUPT"},
        {URB_FREE_BUFFER, "URB_FREE_BUFFER"},
        {URB_ZERO_PACKET, "URB_ZERO_PACKET"},
        // USB-specific error codes
        {-ENOMEM, "ENOMEM"}, // Out of memory
        {-ENODEV, "ENODEV"}, // No such device
        {-EPIPE, "EPIPE"}, // Stalled endpoint
        {-EAGAIN, "EAGAIN"}, // Too many queued ISO transfers
        {-EFBIG, "EFBIG"}, // Too many requested ISO frames
        {-EINVAL, "EINVAL"}, // Invalid argument
        {-EOVERFLOW, "EOVERFLOW"}, // Overflow
        {-ECONNRESET, "ECONNRESET"}, // Connection reset
        {-ESHUTDOWN, "ESHUTDOWN"}, // Device or host controller has been disabled
        {-EPROTO, "EPROTO"}, // Protocol error
        {-ETIME, "ETIME"}, // Timeout
        {-EILSEQ, "EILSEQ"}, // Illegal byte sequence
        {-ENOSR, "ENOSR"}, // Out of stream resources
        {-EHOSTUNREACH, "EHOSTUNREACH"}, // Host unreachable
        {-EREMOTEIO, "EREMOTEIO"}, // Remote I/O error
        {-ENOMEDIUM, "ENOMEDIUM"}, // No medium found
        {-EMEDIUMTYPE, "EMEDIUMTYPE"}, // Wrong medium type
        {-EINPROGRESS, "EINPROGRESS"}, // Operation in progress
        {-EBUSY, "EBUSY"}, // Device or resource busy
        {-EIDRM, "EIDRM"}, // Identifier removed
        {-EPERM, "EPERM"},
        {-ENOENT, "ENOENT"},
        {-ESRCH, "ESRCH"},
        {-EINTR, "EINTR"},
        {-EIO, "EIO"},
        {-ENXIO, "ENXIO"},
        {-E2BIG, "E2BIG"},
        {-ENOEXEC, "ENOEXEC"},
        {-EBADF, "EBADF"},
        {-ECHILD, "ECHILD"},
        {-EAGAIN, "EAGAIN"},
        {-ENOMEM, "ENOMEM"},
        {-EACCES, "EACCES"},
        {-EFAULT, "EFAULT"},
        {-ENOTBLK, "ENOTBLK"},
        {-EBUSY, "EBUSY"},
        {-EEXIST, "EEXIST"},
        {-EXDEV, "EXDEV"},
        {-ENODEV, "ENODEV"},
        {-ENOTDIR, "ENOTDIR"},
        {-EISDIR, "EISDIR"},
        {-EINVAL, "EINVAL"},
        {-ENFILE, "ENFILE"},
        {-EMFILE, "EMFILE"},
        {-ENOTTY, "ENOTTY"},
        {-ETXTBSY, "ETXTBSY"},
        {-EFBIG, "EFBIG"},
        {-ENOSPC, "ENOSPC"},
        {-ESPIPE, "ESPIPE"},
        {-EROFS, "EROFS"},
        {-EMLINK, "EMLINK"},
        {-EPIPE, "EPIPE"},
        {-EDOM, "EDOM"},
        {-ERANGE, "ERANGE"},
        // Additional error codes
        {0, "NO ERROR"}, // No error
        {516, "URB_NO_TRANSFER_DMA_MAP | URB_ZERO_PACKET"} // Combined transfer flags
    };

    static char buffer[32];
    size_t i;

    for (i = 0; i < sizeof(error_map) / sizeof(error_map[0]); i++) {
        if (error_map[i].num == num) {
            return error_map[i].name;
        }
    }

    snprintf(buffer, sizeof(buffer), "UNKNOWN ERROR %d", num);
    return buffer;
}


void print_hcd(struct usb_hcd *hcd)
{
    if (!hcd)
    {
        PRINT_FIELD("usb_hcd is NULL\n");
        return;
    }

    PRINT_FIELD("self: %p", &hcd->self);
    PRINT_FIELD("kref: %p", &hcd->kref);
    PRINT_FIELD("product_desc: %s", hcd->product_desc ? hcd->product_desc : "NULL");
    PRINT_FIELD("speed: %d", hcd->speed);
    PRINT_FIELD("irq_descr: %s", hcd->irq_descr);
    PRINT_FIELD("rh_timer: %p", &hcd->rh_timer);
    PRINT_FIELD("status_urb: %p", hcd->status_urb);
#ifdef CONFIG_PM
    PRINT_FIELD("wakeup_work: %p", &hcd->wakeup_work);
#endif
    PRINT_FIELD("died_work: %p", &hcd->died_work);
    PRINT_FIELD("driver: %p", hcd->driver);
    PRINT_FIELD("usb_phy: %p", hcd->usb_phy);
    PRINT_FIELD("phy_roothub: %p", hcd->phy_roothub);
    PRINT_FIELD("dev_policy: %d", hcd->dev_policy);
    PRINT_FIELD("rh_registered: %u", hcd->rh_registered);
    PRINT_FIELD("rh_pollable: %u", hcd->rh_pollable);
    PRINT_FIELD("msix_enabled: %u", hcd->msix_enabled);
    PRINT_FIELD("msi_enabled: %u", hcd->msi_enabled);
    PRINT_FIELD("skip_phy_initialization: %u", hcd->skip_phy_initialization);
    PRINT_FIELD("uses_new_polling: %u", hcd->uses_new_polling);
    PRINT_FIELD("has_tt: %u", hcd->has_tt);
    PRINT_FIELD("amd_resume_bug: %u", hcd->amd_resume_bug);
    PRINT_FIELD("can_do_streams: %u", hcd->can_do_streams);
    PRINT_FIELD("tpl_support: %u", hcd->tpl_support);
    PRINT_FIELD("cant_recv_wakeups: %u", hcd->cant_recv_wakeups);
    PRINT_FIELD("irq: %u", hcd->irq);
    PRINT_FIELD("regs: %p", hcd->regs);
    PRINT_FIELD("rsrc_start: %llu", hcd->rsrc_start);
    PRINT_FIELD("rsrc_len: %llu", hcd->rsrc_len);
    PRINT_FIELD("power_budget: %u", hcd->power_budget);
    PRINT_FIELD("high_prio_bh: %p", &hcd->high_prio_bh);
    PRINT_FIELD("low_prio_bh: %p", &hcd->low_prio_bh);
    PRINT_FIELD("address0_mutex: %p", hcd->address0_mutex);
    PRINT_FIELD("bandwidth_mutex: %p", hcd->bandwidth_mutex);
    PRINT_FIELD("shared_hcd: %p", hcd->shared_hcd);
    PRINT_FIELD("primary_hcd: %p", hcd->primary_hcd);
    for (int i = 0; i < HCD_BUFFER_POOLS; i++)
    {
        PRINT_FIELD("pool[%d]: %p", i, hcd->pool[i]);
    }
    PRINT_FIELD("state: %d", hcd->state);
    PRINT_FIELD("localmem_pool: %p", hcd->localmem_pool);
}

char * get_bce_vhci_urb_state(int num) {
    switch (num) {
        case BCE_VHCI_URB_INIT_PENDING:
            return "BCE_VHCI_URB_INIT_PENDING";
        case BCE_VHCI_URB_WAITING_FOR_TRANSFER_REQUEST:
            return "BCE_VHCI_URB_WAITING_FOR_TRANSFER_REQUEST";
        case BCE_VHCI_URB_WAITING_FOR_COMPLETION:
            return "BCE_VHCI_URB_WAITING_FOR_COMPLETION";
        case BCE_VHCI_URB_DATA_TRANSFER_COMPLETE:
            return "BCE_VHCI_URB_DATA_TRANSFER_COMPLETE";
        case BCE_VHCI_URB_CONTROL_WAITING_FOR_SETUP_REQUEST:
            return "BCE_VHCI_URB_CONTROL_WAITING_FOR_SETUP_REQUEST";
        case BCE_VHCI_URB_CONTROL_WAITING_FOR_SETUP_COMPLETION:
            return "BCE_VHCI_URB_CONTROL_WAITING_FOR_SETUP_COMPLETION";
        case BCE_VHCI_URB_CONTROL_COMPLETE:
            return "BCE_VHCI_URB_CONTROL_COMPLETE";
        default:
            return "Unknown state";
    }
}

int print_state(struct bce_vhci_urb *vurb, struct bce_vhci_transfer_queue *q, struct urb *urb, int status, int ret)
{
    PRINT_FIELD("bce-vhci: vurb->state %d, %s", vurb->state, get_bce_vhci_urb_state(vurb->state));
    PRINT_FIELD("bce-vhci: status %d, %s", status, get_error_name(status));
    // PRINT_FIELD("bce-vhci: usb_hcd_check_unlink_urb(q->vhci->hcd, urb, status) = %s",
    //             ret == 0 ? "0\0" : ret == -EBUSY ? "-EBUSY\0"
    //                            : ret == -EIDRM   ? "-EIDRM\0"
    //                                              : "unknow value\0");
    PRINT_FIELD("bce-vhci: usb_hcd_check_unlink_urb(q->vhci->hcd, urb, status): %d, %s",ret, get_error_name(ret));
    
    PRINT_FIELD("bce-vhci: vurb->received_status  %d, %s",vurb->received_status, get_error_name(vurb->received_status));
    PRINT_FIELD("bce-vhci: urb->unlinked  %d, %s",urb->unlinked, get_error_name(urb->unlinked));
    PRINT_FIELD("bce-vhci: urb->hcpriv  %p", urb->hcpriv);
    PRINT_FIELD("bce-vhci: urb->urb_list  %p", &urb->urb_list);
    PRINT_FIELD("bce-vhci: urb->transfer_flags %d, %s",urb->transfer_flags, get_error_name(urb->transfer_flags));
    PRINT_FIELD("bce-vhci: q->state  %d", q->state);

    switch (vurb->state)
    {
    case BCE_VHCI_URB_INIT_PENDING:
        PRINT_FIELD("bce-vhci: Handle BCE_VHCI_URB_INIT_PENDING state");
        break;

    case BCE_VHCI_URB_WAITING_FOR_TRANSFER_REQUEST:
        PRINT_FIELD("bce-vhci: Handle BCE_VHCI_URB_WAITING_FOR_TRANSFER_REQUEST state");
        break;

    case BCE_VHCI_URB_WAITING_FOR_COMPLETION:
        PRINT_FIELD("bce-vhci: Handle BCE_VHCI_URB_WAITING_FOR_COMPLETION state");
        break;

    case BCE_VHCI_URB_DATA_TRANSFER_COMPLETE:
        PRINT_FIELD("bce-vhci: Handle BCE_VHCI_URB_DATA_TRANSFER_COMPLETE state");
        break;

    case BCE_VHCI_URB_CONTROL_WAITING_FOR_SETUP_REQUEST:
        PRINT_FIELD("bce-vhci: Handle BCE_VHCI_URB_CONTROL_WAITING_FOR_SETUP_REQUEST state");
        break;

    case BCE_VHCI_URB_CONTROL_WAITING_FOR_SETUP_COMPLETION:
        PRINT_FIELD("bce-vhci: Handle BCE_VHCI_URB_CONTROL_WAITING_FOR_SETUP_COMPLETION state");
        break;

    case BCE_VHCI_URB_CONTROL_COMPLETE:
        PRINT_FIELD("bce-vhci: Handle BCE_VHCI_URB_CONTROL_COMPLETE state");
        break;

    default:
        PRINT_FIELD("bce-vhci: Handle unknown state");
        break;
    }

    return 0;
}

int print_urb(struct urb *u, char *file)
{
    if(!u)
        return -ENOENT;
    PRINT_FIELD("bce-vhci:\t\tPRINTING URB");
    PRINT_FIELD("bce-vhci:called from %s for device %p", file, u->dev);
    PRINT_FIELD("bce-vhci:1: kref = %p", &u->kref);
    PRINT_FIELD("bce-vhci:2: unlinked = %d", u->unlinked);
    PRINT_FIELD("bce-vhci:3: hcpriv = %p", u->hcpriv);
    PRINT_FIELD("bce-vhci:4: use_count = %d", atomic_read(&u->use_count));
    PRINT_FIELD("bce-vhci:5: reject = %d", atomic_read(&u->reject));
    PRINT_FIELD("bce-vhci:6: urb_list = %p", &u->urb_list);
    PRINT_FIELD("bce-vhci:7: anchor_list = %p", &u->anchor_list);
    PRINT_FIELD("bce-vhci:8: anchor = %p", u->anchor);
    PRINT_FIELD("bce-vhci:9: dev = %p", u->dev);
    PRINT_FIELD("bce-vhci:10: ep = %p", u->ep);
    PRINT_FIELD("bce-vhci:11: pipe = %u", u->pipe);
    PRINT_FIELD("bce-vhci:12: stream_id = %u", u->stream_id);
    PRINT_FIELD("bce-vhci:13: status = %d", u->status);
    PRINT_FIELD("bce-vhci:14: transfer_flags = %u", u->transfer_flags);
    PRINT_FIELD("bce-vhci:15: transfer_buffer = %p", u->transfer_buffer);
    PRINT_FIELD("bce-vhci:16: transfer_dma = %llu", (unsigned long long)u->transfer_dma);
    PRINT_FIELD("bce-vhci:17: sg = %p", u->sg);
    PRINT_FIELD("bce-vhci:18: num_mapped_sgs = %d", u->num_mapped_sgs);
    PRINT_FIELD("bce-vhci:19: num_sgs = %d", u->num_sgs);
    PRINT_FIELD("bce-vhci:20: transfer_buffer_length = %u", u->transfer_buffer_length);
    PRINT_FIELD("bce-vhci:21: actual_length = %u", u->actual_length);
    PRINT_FIELD("bce-vhci:22: setup_packet = %p", u->setup_packet);
    PRINT_FIELD("bce-vhci:23: setup_dma = %llu", (unsigned long long)u->setup_dma);
    PRINT_FIELD("bce-vhci:24: start_frame = %d", u->start_frame);
    PRINT_FIELD("bce-vhci:25: number_of_packets = %d", u->number_of_packets);
    PRINT_FIELD("bce-vhci:26: interval = %d", u->interval);
    PRINT_FIELD("bce-vhci:27: error_count = %d", u->error_count);
    PRINT_FIELD("bce-vhci:28: context = %p", u->context);
    PRINT_FIELD("bce-vhci:29: complete = %p", u->complete);
    PRINT_FIELD("bce-vhci:30: iso_frame_desc = %p", u->iso_frame_desc);

    return 0;
}

int print_transfer_queue(struct bce_vhci_transfer_queue *s, char *file)
{
    PRINT_FIELD("bce-vhci: helper.c: \t\tPRINTING TRNSFER QUEUE\n");
    PRINT_FIELD("bce-vhci: helper.c called from %s for device %u\n", file, s->dev_addr);
    PRINT_FIELD("bce-vhci: helper.c 1: vhci = %p\n", s->vhci);
    PRINT_FIELD("bce-vhci: helper.c 2: endp = %p\n", s->endp);
    PRINT_FIELD("bce-vhci: helper.c 3: state = %d\n", s->state);
    PRINT_FIELD("bce-vhci: helper.c 4: max_active_requests = %u\n", s->max_active_requests);
    PRINT_FIELD("bce-vhci: helper.c 5: remaining_active_requests = %u\n", s->remaining_active_requests);
    PRINT_FIELD("bce-vhci: helper.c 6: active = %d\n", s->active);
    PRINT_FIELD("bce-vhci: helper.c 7: stalled = %d\n", s->stalled);
    PRINT_FIELD("bce-vhci: helper.c 8: paused_by = %u\n", s->paused_by);
    PRINT_FIELD("bce-vhci: helper.c 9: dev_addr = %u\n", s->dev_addr);
    PRINT_FIELD("bce-vhci: helper.c 10: endp_addr = %u\n", s->endp_addr);
    PRINT_FIELD("bce-vhci: helper.c 11: cq = %p\n", s->cq);
    PRINT_FIELD("bce-vhci: helper.c 12: sq_in = %p\n", s->sq_in);
    PRINT_FIELD("bce-vhci: helper.c 13: sq_out = %p\n", s->sq_out);
    PRINT_FIELD("bce-vhci: helper.c 14: evq = %p\n", &s->evq);
    PRINT_FIELD("bce-vhci: helper.c 15: urb_lock = %p\n", &s->urb_lock);
    PRINT_FIELD("bce-vhci: helper.c 16: pause_lock = %p\n", &s->pause_lock);
    PRINT_FIELD("bce-vhci: helper.c 17: giveback_urb_list = %p\n", &s->giveback_urb_list);
    PRINT_FIELD("bce-vhci: helper.c 18: w_reset = %p\n", &s->w_reset);
    return 0;
}

const char* get_usb_state_name(int state) {
    switch (state) {
        case 0: return "USB_STATE_NOTATTACHED";
        case 1: return "USB_STATE_ATTACHED";
        case 2: return "USB_STATE_POWERED";
        case 3: return "USB_STATE_RECONNECTING";
        case 4: return "USB_STATE_UNAUTHENTICATED";
        case 5: return "USB_STATE_DEFAULT";
        case 6: return "USB_STATE_ADDRESS";
        case 7: return "USB_STATE_CONFIGURED";
        case 8: return "USB_STATE_SUSPENDED";
        default: return "UNKNOWN_STATE";
    }
}

void print_ep_from_urb(struct urb *urb){
    struct usb_device *dev = urb->dev;
    pr_info("USB Device Information:\n");
    pr_info("  Device Number: %d\n", dev->devnum);
    pr_info("  Speed: %d\n", dev->speed);
    pr_info("  Device Path: %s\n", dev->devpath);
    pr_info("  Bus Number: %d\n", dev->bus->busnum);
    pr_info("  Parent Device: %p\n", dev->parent);
    pr_info("  Number of Children: %d\n", dev->maxchild);
    pr_info("  Manufacturer: %s\n", dev->manufacturer ? dev->manufacturer : "Unknown");
    pr_info("  Product: %s\n", dev->product ? dev->product : "Unknown");
    pr_info("  Serial Number: %s\n", dev->serial ? dev->serial : "Unknown");
    pr_info("  Number of Configurations: %d\n", dev->descriptor.bNumConfigurations);
    pr_info("  Active Configuration: %p\n", dev->actconfig);
    pr_info("  Device State: %d, %s \n", dev->state, get_usb_state_name(dev->state));
    
    for (int i = 0; i < 16; i++) {
        if (dev->ep_in[i]) {
            pr_info("  Endpoint IN[%d]: %p\n", i, dev->ep_in[i]);
        }
        if (dev->ep_out[i]) {
            pr_info("  Endpoint OUT[%d]: %p\n", i, dev->ep_out[i]);
        }
    }
}

void dump_bce_queue_sq(struct bce_queue_sq *sq) {
    if(!sq){
        pr_info("the sq in input is NULL");
        return;
    }
    PRINT_FIELD("qid: %d", sq->qid);
    PRINT_FIELD("type: %d", sq->type);
    PRINT_FIELD("el_size: %u", sq->el_size);
    PRINT_FIELD("el_count: %u", sq->el_count);
    PRINT_FIELD("dma_handle: %pad", &sq->dma_handle);
    PRINT_FIELD("data: %p", sq->data);
    PRINT_FIELD("userdata: %p", sq->userdata);
    PRINT_FIELD("reg_mem_dma: %p", sq->reg_mem_dma);

    PRINT_FIELD("available_commands: %d", atomic_read(&sq->available_commands));
    PRINT_FIELD("available_command_completion_waiting_count: %d", atomic_read(&sq->available_command_completion_waiting_count));
    PRINT_FIELD("head: %u", sq->head);
    PRINT_FIELD("tail: %u", sq->tail);

    PRINT_FIELD("completion_cidx: %u", sq->completion_cidx);
    PRINT_FIELD("completion_tail: %u", sq->completion_tail);
    PRINT_FIELD("completion_data: %p", sq->completion_data);
    PRINT_FIELD("has_pending_completions: %d", sq->has_pending_completions);
    PRINT_FIELD("completion: %p", &sq->completion);
}

