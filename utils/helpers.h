#ifndef _H_HELPERS
#define _H_HELPERS

#define LOG_LEVEL_INFO 1
#define LOG_LEVEL_DEBUG 2
#define LOG_LEVEL_ERR 3
#define LOG_LEVEL_NONE 4

// Define the log level here
#define LOG_LEVEL LOG_LEVEL_INFO

#if LOG_LEVEL == LOG_LEVEL_INFO
    #define PRINT_FIELD(format, ...) pr_info(format "\n", ##__VA_ARGS__)
#elif LOG_LEVEL == LOG_LEVEL_DEBUG
    #define PRINT_FIELD(format, ...) pr_debug(format "\n", ##__VA_ARGS__)
#elif LOG_LEVEL == LOG_LEVEL_ERR
    #define PRINT_FIELD(format, ...) pr_err(format "\n", ##__VA_ARGS__)
#elif LOG_LEVEL == LOG_LEVEL_NONE
    #define PRINT_FIELD(format, ...) /* no print */
#else
    #error "Invalid log level"
#endif

struct bce_vhci_transfer_queue; struct urb ; struct bce_vhci_urb; struct bce_queue_sq;

const char* get_error_name(int num);
void print_hcd(struct usb_hcd *hcd);
int print_transfer_queue(struct bce_vhci_transfer_queue *s, char *file);
int print_urb(struct urb *u, char *file);
int print_state(struct bce_vhci_urb *vurb, struct bce_vhci_transfer_queue *q, struct urb *urb, int status, int ret);
void print_ep_from_urb(struct urb *urb);

void dump_bce_queue_sq(struct bce_queue_sq *sq);

// void dump_bce_mailbox(struct bce_mailbox *mb);
// pid_t get_os_pid(void);
// void print_irq_info(int irq, struct pci_dev *pdev, irq_handler_t handler);
// void enumerate_device_node(struct device *dev);

#endif