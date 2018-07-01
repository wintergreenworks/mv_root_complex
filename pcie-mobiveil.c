/*
 * PCIe host controller driver for Mobiveil PCIe Host controller
 *
 * Based on the Altera PCIe driver
 *
 * Bits taken from Altera/Xilinx Host controller driver and
 * ARM PCI Host generic driver.
 * Copyright mobiveil Corporation (C) 2013-2015. All rights reserved
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/irq.h>
#include <linux/of_pci.h>
#include <linux/pci.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/kthread.h>

//#include <linux/mv.h>
//#include <linux/mv_conf.h>

#define RESET_SOC_PCIE
//#define MVRC_DEBUG
//#define mv_dbg DLOG
#ifdef MVRC_DEBUG
#define DLOG printk
#define DPRINTF(err, fmt, args...)  printk("ERR#%04x: ", err); printk(fmt, ## args)
#else
#define DLOG(...)
#define DPRINTF(...)
#endif

#define NEW_INTR_METHOD

#define GPEX_INTA_POS   			(5)
#define OP_READ         			(0)
#define OP_WRITE        			(1)

/*
   VENDOR ID: 0x1C18
   DEVICE ID: 0x2324
*/

/*Register Offsets and Bit Positions*/

#define BAR_MAP_BASE        			0x10000000
#define BAR_MAP_DISTANCE    			0x00000000
#define IO_BAR_MAP_BASE     			0x20000000
#define IO_BAR_MAP_DISTANCE 			0x10000000
#define WIN_NUM_0           			0
#define WIN_NUM_1           			1
#define WIN_NUM_2           			2
#define WIN_NUM_3           			3

/* ltssm_state_status*/
#define LTSSM_STATE_STATUS_REG  		0x0404
#define LTSSM_STATUS_CODE_MASK  		0x3F
#define LTSSM_STATUS_CODE       		0x2D	/* LTSSM Status Code L0 */

#define PAB_CAP_REG                 		(0x0804)
#define PAB_CTRL_REG                		(0x0808)
#define  AMBA_PIO_ENABLE_BIT    		0
#define  PEX_PIO_ENABLE_BIT     		1

#define PAB_AXI_PIO_CTRL_REG(win_num) 		(0x0840 + 0x10*win_num)
#define  PIO_ENABLE_BIT             		0
#define  MEM_WINDOW_ENABLE_BIT      		1
#define  IO_WINDOW_ENABLE_BIT       		2
#define  CONFIG_WINDOW_ENABLE_BIT   		3

#define PAB_PEX_PIO_CTRL_REG        		0x08C0
#define PAB_PEX_PIO_STAT_REG        		0x08C4
#define PAB_INTP_AMBA_MISC_ENB      		0x0B0C
#define PAB_INTP_AMBA_MISC_STAT     		0x0B1C
#define  PAB_INTP_INTX_MASK         		0x1E0 /* INTx(A-D)*/
#define  PAB_INTP_MSI_MASK          		0x8

#define PAB_MSI_IB_STAT            		0x0B60
#define PAB_MSI_IB_FIFO_DW1         		0x0B64
#define PAB_MSI_IB_FIFO_DW2         		0x0B68
#define PAB_MSI_IB_FIFO_DW3         		0x0B6C
#define PAB_MSI_IB_FIFO_DW4         		0x0B70
#define PAB_MSI_IB_FIFO_DW5         		0x0B74

#define PAB_AXI_AMAP_CTRL_REG(win_num) 		(0x0BA0 + 0x10*(win_num))
#define  ENABLE_BIT             		0
#define  TYPE_BIT               		1		/*2..1*/
#define  AXI_WINDOW_SIZE_BIT    		10		/*31..10*/

#define PAB_AXI_AMAP_AXI_WIN_REG(win_num) 	(0x0BA4 + 0x10*(win_num))
#define  AXI_WINDOW_BASE_BIT    		2		/*31..2*/

#define PAB_AXI_AMAP_PEX_WIN_L_REG(win_num) 	(0x0BA8 + 0x10*(win_num))
#define  BUS_BIT                		24		/*31..24*/
#define  DEVICE_BIT             		19		/*23..19*/
#define  FUNCTION_BIT           		16		/*18..16*/
#define  REGISTER_BIT           		0		/*15..0*/

#define PAB_AXI_AMAP_PEX_WIN_H_REG(win_num) 	(0x0BAC + 0x10*(win_num))
#define PAB_INTP_AXI_PIO_ENB_REG        	(0x0B00)
#define PAB_INTP_AXI_PIO_STAT_REG       	(0x0B10)
#define PAB_INTP_AXI_PIO_VENID_REG      	(0x470)
#define PAB_INTP_AXI_PIO_DEVID_REG     		(0x472)
#define PAB_INTP_AXI_PIO_CLASS_REG      	(0x474)

#define PAB_EXT_PEX_AMAP_SIZEN(win_num)		(0xBEF0 + (0x10*(win_num)))
#define PAB_AXI_PIO_STAT(win_num)       	(0x844  + (0x10*(win_num)))
#define PAB_PEX_AMAP_CTRL(win_num)      	(0x4BA0 + (0x10*(win_num)))
#define PAB_PEX_AMAP_AXI_WIN(win_num)   	(0x4BA4 + (0x10*(win_num)))
#define PAB_PEX_AMAP_PEX_WIN_L(win_num) 	(0x4BA8 + (0x10*(win_num)))
#define PAB_PEX_AMAP_PEX_WIN_H(win_num) 	(0x4BAC + (0x10*(win_num)))

#define MSI_MSG_BASE_ADDR			(0x60011000)
#define INTX_NUM                        	(4)
#define MOBIVEIL_NUM_MSI_IRQS           	(16)

/* Other Registers */
#define MV_PAB_SOC_CSR				(0x75800000)
#define DEFAULT_PAB_INTP_CLASS_VAL		(0x06040200)

/* local prototypes */

/**
    PCIe port information
*/
struct mobiveil_pcie
{
	struct platform_device *pdev;   /** platform device pointer*/
	phys_addr_t phy_gpex_csr;       /** CSR Base physical address */
	phys_addr_t phy_apio_base;      /** APIO Base Physical address */
	void __iomem *config_apio_base;	/** IO mapped register base for endpoint config access */
	void __iomem *gpex_csr_base;   	/** IO mapped register base for root port config access */
	int ob_wins_configured; 	/** Configured outbound windows */
	int ib_wins_configured; 	/** Configured inbound windows */
	int irq;                        /** IRQ number of the root port */
	u8 root_bus_nr;                 /** Root port bus number */
	struct irq_domain *irq_domain;  /** irq domain associated with root port */
	struct resource bus_range;      /** bus range resource */
	struct list_head resources;     /** all the enumerated resources */
	unsigned long msi_pages;        /** Physical address of the MSI data */
};

/**
  union csr_reg_offset_t - paged register offsets union

  */
typedef union __attribute__ ((packed))
{
    unsigned int dw;
    struct __attribute__ ((packed))
    {
        unsigned int offset  :10;
        unsigned int pg_sel  :4;
    };
}csr_reg_offset_t;

/**
  union pab_pex_amap_ctrl_t - PAB_PEX_AMAP register bitfields
  */
typedef union __attribute__ ((packed))
{
    int dw;
    struct __attribute__ ((packed))
    {
        int enable      :1;
        int type        :2;
        int no_snoop_ov_en  :1;
        int no_snoop    :1;
        int rsvd        :5;
        int size        :22;
    };
}pab_pex_amap_ctrl_t;

/**
  union pab_ctrl_t - PAB_CTRL register bitfields
  */
typedef union __attribute__ ((packed))
{
    int dw;
    struct __attribute__ ((packed))
    {
        int amba_pio    :1;
        int pex_pio     :1;
        int wdma        :1;
        int rdma        :1;
        int axi_max_burst_len        :2;
        int rsvd        :1;
        int dma_pio_arb        :1;
        int prefetch_depth     :2;
        int mrrs        :3;
        int pg_sel      :6;
    };
}pab_ctrl_t;


/* global variables  */

struct  mobiveil_pcie *g_pcie = NULL;
u32     msi_ints = 0, 
        msi_msgs = 0;
static  DECLARE_BITMAP(msi_irq_in_use, MOBIVEIL_NUM_MSI_IRQS);

/**
    get_reg_offset  - Routine to get register offset of the paged register

    @pcie   :  pointer to root port
    @reg    :   full paged address of the register
*/
unsigned int get_reg_offset(struct mobiveil_pcie *pcie, const u32 reg)
{
#if 0
    if(0x400 > reg)
    {
        printk("get_reg_offset: reg: %x\n", reg);
        return reg;
    }
    else
#endif
    {
        pab_ctrl_t pab_ctrl;
        csr_reg_offset_t reg_offset;
        reg_offset.dw = reg;
        pab_ctrl.dw = readl_relaxed (pcie->gpex_csr_base + PAB_CTRL_REG);
        pab_ctrl.pg_sel = reg_offset.pg_sel;
        printk("get_reg_offset: reg: %x, pg_sel:%x, offset: %x \n",
                reg, pab_ctrl.pg_sel, reg_offset.offset);
        writel_relaxed (pab_ctrl.dw, pcie->gpex_csr_base + PAB_CTRL_REG);
        return reg_offset.offset;
    }
}

/**
  csr_writel - routine to write one DWORD to memory mapper register

  @pcie :   pointer to root port
  @value:   value to be written to register
  @reg  :   register offset
  */
static inline void csr_writel (struct mobiveil_pcie *pcie, const u32 value, const u32 reg)
{
    writel_relaxed (value, pcie->gpex_csr_base + reg);
    DLOG ("wrote 0x%x @ 0x%p\n", value, pcie->gpex_csr_base + reg);
}

/**
  csr_readl - routine to read one DWORD from memory mapper register

  @pcie :    pointer to root port
  @reg  :    register offset
  */

static inline u32 csr_readl (struct mobiveil_pcie *pcie, const u32 reg) 
{     
    u32 value;

    value = readl_relaxed (pcie->gpex_csr_base + reg);
    DLOG ("read 0x%x @ 0x%p\n", value, pcie->gpex_csr_base + reg);
    return value;
}

/**
  mobiveil_pcie_link_is_up - routine to check if PCIe link is up

  @pcie :    pointer to root port
  */
 
static bool mobiveil_pcie_link_is_up (struct mobiveil_pcie *pcie) 
{    
    u32 link;
    /* NOTE: since link detection happens in micro-seconds, no infinite polling here*/
    do {
    	link = csr_readl (pcie, LTSSM_STATE_STATUS_REG) & LTSSM_STATUS_CODE_MASK;
    	//pr_info ("%s: Link Status LTSSM_STATE_STATUS_REG[0x%x] : 0x%x\n", __FUNCTION__,
            //LTSSM_STATE_STATUS_REG, csr_readl (pcie, LTSSM_STATE_STATUS_REG));
    }while( link != LTSSM_STATUS_CODE );
    
    return true;
}

/**
  mobiveil_pcie_valid_device - routine to check if a valid device/function is present on the bus

  @pcie :    pointer to root port
  */
static bool mobiveil_pcie_valid_device (struct pci_bus *bus, unsigned int devfn) 
{    
    struct mobiveil_pcie *pcie = bus->sysdata;

    /* Check if link is up when trying to access downstream ports */
    if (bus->number != pcie->root_bus_nr)
        if (!mobiveil_pcie_link_is_up (pcie))
            return false;

    /* Only one device down on each root port */
    if (bus->number == pcie->root_bus_nr && devfn > 0)
        return false;

    /* Do not read more than one device on the bus directly attached to RC */
    if (bus->primary == pcie->root_bus_nr && devfn > 0)
        return false;

    return true;
}

/**
  mobiveil_pcie_map_bus - routine to get the configuration base of either root port or endpoint

  @bus  :   pointer to local bus
  @devfn:   variable containing the device and function numbers
  @where:   register offset
  */
static void __iomem * mobiveil_pcie_map_bus (struct pci_bus *bus, unsigned int devfn, int where) 
{    
    struct mobiveil_pcie *pcie = bus->sysdata;
    void __iomem *addr = NULL;

    //printk("%s: Device Function :%d , Where %d \n",  __FUNCTION__, devfn, where);
    if (!mobiveil_pcie_valid_device (bus, devfn)) {
    	//printk("%s: Not Valid Device  or Device function ERROR \n",  __FUNCTION__);
        return NULL;
    }	
    if (bus->number == pcie->root_bus_nr)
    {
        /* RC config access (in CSR space)*/
        addr = pcie->gpex_csr_base + where;
    	//printk("%s: RC Bus %d : Addr = %p, Value 0x%X  \n",  __FUNCTION__,bus->number, addr,  ioread8 (addr));
    }
    else
    {	
        /* EP config access (in Config/APIO space)*/
        u32 value;

        /* Program PEX Address base (31..16 bits) with appropriate value (BDF) in PAB_AXI_AMAP_PEX_WIN_L0 Register*/
        value = csr_readl (pcie, PAB_AXI_AMAP_PEX_WIN_L_REG (0));
        /* TODO: Use standard macros instead of numbers 3 and 7*/
        csr_writel (pcie, bus->number << BUS_BIT | (devfn >> 3) << DEVICE_BIT | (devfn & 7) << FUNCTION_BIT, 
                PAB_AXI_AMAP_PEX_WIN_L_REG (0));
        addr = pcie->config_apio_base + where;
    //	printk("%s: Endpoint Bus %d :  access Addr = %p\n",  __FUNCTION__, bus->number, addr);
    }
    return addr;
}

/* PCIe operations */
static struct pci_ops mobiveil_pcie_ops = {
    .map_bus = mobiveil_pcie_map_bus,
    .read = pci_generic_config_read,
    .write = pci_generic_config_write,
};

/**
    dump_resource - routine to dump a single resource 

    @r    : pointer to resource structure
  */

void dump_resource (struct resource *r)
{
    printk("%s: Resource, name: %s, start: %x, end: %x, type: 0x%lx(%s), flags: 0x%lx\n",
                 __FUNCTION__, r->name, (unsigned int) r->start, (unsigned int )r->end,
		resource_type (r), (resource_type (r) == IORESOURCE_MEM)?"MEM":"IO",  r->flags);

}

/**
    dump_resources - routine to dump resources associated with the platform device

    @dev    : pointer to platform device 
  */
void dump_resources (struct platform_device *dev)
{
#ifdef  MVRC_DEBUG
    int i;

    for (i = 0; i < dev->num_resources; i++)
    {
        struct resource *r = &dev->resource[i];

        if (r->name != NULL)
        {
            dump_resource(r);
            if (r->parent)
            {
                DLOG("r->parent \n");
                dump_resource(r->parent)
            }
            else
                DLOG ("r->parent=NULL\n");

        }

    }
#endif
}
/**
  access_paged_register - routine to access paged register of root complex

  registers of RC are paged, with pg_sel field of the PAB_CTRL_REG register needs to be 
  updated with page of the register, before accessing least significant 10 bits offset.
  This routine does the PAB_CTRL_REG updation and split access of the offset.

  @pcie   : pointer to rootport
  @write  : type of operation flag
  @val    : value to be written to the register
  @offset : full offset of the register address
 */
unsigned int access_paged_register(struct mobiveil_pcie *pcie, unsigned int write, unsigned int val, unsigned int offset)
{
	pab_ctrl_t pab_ctrl;
	unsigned int off = (offset & 0x3FF) + 0xC00; 
	pab_ctrl.dw = csr_readl (pcie, PAB_CTRL_REG);
	pab_ctrl.pg_sel = (offset >> 10) & 0x3F;
	csr_writel (pcie, pab_ctrl.dw, PAB_CTRL_REG);

	printk(KERN_ERR "access_paged_register: op: %x, offset: %x, val: %x, pg: %x, off: %x, pab_ctrl: %x \n",
			write, offset, val, pab_ctrl.pg_sel, off, csr_readl (pcie, PAB_CTRL_REG) );
	if(write == OP_WRITE)
		csr_writel (pcie, val, off);
	else
		return csr_readl (pcie, off);
	return 0;
}

/**
Just for DEBUGGING, Please remove this after porting - Karthikeyan Mitran
*/
void dump_csr_register(struct mobiveil_pcie *pcie)
{
int window = WIN_NUM_0;
#if 0
	unsigned int u32_start_addr, u32_end_addr, curr_reg;
	printk(KERN_WARNING "AXi interrupt Register:");
	u32_start_addr = 0x0B00;
	u32_end_addr = 0x0B1C;
	for (curr_reg = u32_start_addr; curr_reg <= u32_end_addr ; curr_reg +=4)
	{
		printk(KERN_WARNING "REGISTER 0x%x Value [0x%x]",curr_reg, csr_readl(pcie, curr_reg));
	}
	printk(KERN_WARNING "AXi AMAP window 0 Register:");
	u32_start_addr = 0x0BA0;
	u32_end_addr = 0x0BAC;
	for (curr_reg = u32_start_addr; curr_reg <= u32_end_addr ; )
	{
		printk(KERN_WARNING "REGISTER 0x%x Value [0x%x]",curr_reg, csr_readl(pcie, curr_reg));
		curr_reg += 4;
	}
	printk(KERN_WARNING "AXi AMAP window 1 Register:");
	u32_start_addr = 0x0BB0;
	u32_end_addr = 0x0BBC;
	for (curr_reg = u32_start_addr; curr_reg <= u32_end_addr ; )
	{
		printk(KERN_WARNING "REGISTER 0x%x Value [0x%x]",curr_reg, csr_readl(pcie, curr_reg));
		curr_reg += 4;
	}
	printk(KERN_WARNING "AXi AMAP window 2 Register:");
	u32_start_addr = 0x0BC0;
	u32_end_addr = 0x0BCC;
	for (curr_reg = u32_start_addr; curr_reg <= u32_end_addr ; )
	{
		printk(KERN_WARNING "REGISTER 0x%x Value [0x%x]",curr_reg, csr_readl(pcie, curr_reg));
		curr_reg += 4;
	}
	printk(KERN_WARNING "PEX AMAP window 0 Register:");
	u32_start_addr = 0x4BA0;
	u32_end_addr = 0x4BAC;
	for (curr_reg = u32_start_addr; curr_reg <= u32_end_addr ; )
	{
		printk(KERN_WARNING "REGISTER 0x%x Value [0x%x]",curr_reg, csr_readl(pcie, curr_reg));
		curr_reg += 4;
	}
	printk(KERN_WARNING "PEX AMAP window 1 Register:");
	u32_start_addr = 0x4BB0;
	u32_end_addr = 0x4BBC;
	for (curr_reg = u32_start_addr; curr_reg <= u32_end_addr ; )
	{
		printk(KERN_WARNING "REGISTER 0x%x Value [0x%x]",curr_reg, csr_readl(pcie, curr_reg));
		curr_reg += 4;
	}

	
	printk(KERN_WARNING "EP's Config space dump");
	u32_start_addr = 0x0;
	u32_end_addr = 0x200;
	for (curr_reg = u32_start_addr; curr_reg <= u32_end_addr ; )
	{
		printk(KERN_WARNING "[EP's]APIO REGISTER 0x%x Value [0x%x]",curr_reg, ioread32(pcie->config_apio_base + curr_reg));

		curr_reg += 4;
	}


    pr_info ("%s: PAB_PEX_PIO_CTRL_REG register[0x%x] : 0x%x\n",
            __FUNCTION__, PAB_PEX_PIO_CTRL_REG, csr_readl (pcie, PAB_PEX_PIO_CTRL_REG));
    pr_info ("%s: PAB_PEX_AMAP_CTRL register[0x%x] : 0x%x\n", __FUNCTION__, PAB_PEX_AMAP_CTRL(window),
            access_paged_register(pcie, OP_READ, 0, PAB_PEX_AMAP_CTRL(window)));
    pr_info ("%s: PAB_PEX_AMAP_AXI_WIN register[0x%x] : 0x%x\n",
            __FUNCTION__, PAB_PEX_AMAP_AXI_WIN(window), access_paged_register (pcie, OP_READ, 0, PAB_PEX_AMAP_AXI_WIN(window)));
    pr_info ("%s: PAB_PEX_AMAP_PEX_WIN_L register[0x%x] : 0x%x\n",
            __FUNCTION__, PAB_PEX_AMAP_PEX_WIN_L(window), access_paged_register (pcie, OP_READ, 0, PAB_PEX_AMAP_PEX_WIN_L(window)));
    pr_info ("%s: PAB_PEX_AMAP_PEX_WIN_H register[0x%x] : 0x%x\n",
            __FUNCTION__, PAB_PEX_AMAP_PEX_WIN_H(window), access_paged_register (pcie, OP_READ, 0, PAB_PEX_AMAP_PEX_WIN_H(window)));
	window = 1;

    pr_info ("%s: PAB_PEX_PIO_CTRL_REG register[0x%x] : 0x%x\n",
            __FUNCTION__, PAB_PEX_PIO_CTRL_REG, csr_readl (pcie, PAB_PEX_PIO_CTRL_REG));
    pr_info ("%s: PAB_PEX_AMAP_CTRL register[0x%x] : 0x%x\n", __FUNCTION__, PAB_PEX_AMAP_CTRL(window),
            access_paged_register(pcie, OP_READ, 0, PAB_PEX_AMAP_CTRL(window)));
    pr_info ("%s: PAB_PEX_AMAP_AXI_WIN register[0x%x] : 0x%x\n",
            __FUNCTION__, PAB_PEX_AMAP_AXI_WIN(window), access_paged_register (pcie, OP_READ, 0, PAB_PEX_AMAP_AXI_WIN(window)));
    pr_info ("%s: PAB_PEX_AMAP_PEX_WIN_L register[0x%x] : 0x%x\n",
            __FUNCTION__, PAB_PEX_AMAP_PEX_WIN_L(window), access_paged_register (pcie, OP_READ, 0, PAB_PEX_AMAP_PEX_WIN_L(window)));
    pr_info ("%s: PAB_PEX_AMAP_PEX_WIN_H register[0x%x] : 0x%x\n",
            __FUNCTION__, PAB_PEX_AMAP_PEX_WIN_H(window), access_paged_register (pcie, OP_READ, 0, PAB_PEX_AMAP_PEX_WIN_H(window)));
#endif
}
/**
  mobiveil_pcie_isr - interrupt handler for root complex

  @irq    : IRQ number 
  @data   : pointer to driver specific data
 */
static irqreturn_t mobiveil_pcie_isr (int irq, void *data)
{
    struct irq_chip *chip = data;
    struct mobiveil_pcie *pcie;
    unsigned long status, shifted_status;
    u32  bit1 = 0, virq = 0;
    u32 val, mask;

    pcie = (struct mobiveil_pcie *)data;
    val = csr_readl(pcie, PAB_INTP_AMBA_MISC_STAT);
    mask = csr_readl(pcie, PAB_INTP_AMBA_MISC_ENB);
    status = val & mask;
    if(!status)
    {
        return IRQ_NONE;
    }
    if(status & PAB_INTP_INTX_MASK)
    {
        while ((shifted_status =  (csr_readl(pcie, PAB_INTP_AMBA_MISC_STAT) >> GPEX_INTA_POS)) != 0)
        {
            for_each_set_bit (bit1, &shifted_status, INTX_NUM)
            {
                /* clear interrupts */
                //csr_writel(pcie, shifted_status << GPEX_INTA_POS , PAB_INTP_AMBA_MISC_STAT);

//                virq = irq_find_mapping (pcie->legacy_irq_domain, bit1 + 1);
		virq=2;
                DLOG ("%s: misc shifted status: %x, virq: %x, bit1: %x \n", __FUNCTION__, shifted_status, virq, bit1);
                if (virq)
                    generic_handle_irq (virq);
                else
                    dev_err (&pcie->pdev->dev, "unexpected IRQ, INT%d\n", bit1);

            }
            shifted_status = 0;
        }
    }

    if(status & PAB_INTP_MSI_MASK)
    {
        u32 msi_data = 0;
        u32 messages = 0;
        do
        {
            u32 ib_stat = csr_readl(pcie, PAB_MSI_IB_STAT);
            messages = (ib_stat >> 1) & 0xf;
            if(1 == ib_stat)
            {
                printk(KERN_WARNING "num messages are zero ! problem call !  \n");
                BUG();
                return IRQ_NONE;
            }
            if(messages > 1)
                DLOG("msi message!, ib_stat: %x, messages: %x \n", ib_stat, messages);
            msi_data = csr_readl(pcie, PAB_MSI_IB_FIFO_DW5);
            /* Handle MSI */
            generic_handle_irq (msi_data);
            ib_stat = csr_readl(pcie, PAB_MSI_IB_STAT);
            messages = (ib_stat >> 1) & 0xf;
            csr_writel(pcie, status, PAB_INTP_AMBA_MISC_STAT);
        }while(messages);
    }

}

void mobi_softreset_pcie(struct mobiveil_pcie *pcie)
{
    struct platform_device *pdev = pcie->pdev;
    unsigned int soc_csr_value = 0x00;
    unsigned long soc_csr_base = MV_PAB_SOC_CSR;
    unsigned long *soc_csr_base_va = (unsigned long *) ioremap(soc_csr_base, 0x1000);

    dev_err (&pdev->dev, "Soft Resetting PCIe RC IP at soc_csr_base : %lx\n", soc_csr_base);
    soc_csr_value = ioread32(soc_csr_base_va);

    soc_csr_value &= ~0x2;
    iowrite32(soc_csr_value,soc_csr_base_va);
    wmb();
    
    udelay(100);
    soc_csr_value = ioread32(soc_csr_base_va);
    soc_csr_value |= 0x2;
    iowrite32(soc_csr_value,soc_csr_base_va);
    wmb();

    soc_csr_value = ioread32(soc_csr_base_va);
    dev_err (&pdev->dev, "Soft Resetting PCIe RC IP at soc_csr_base complete : %lx\n\n\n", soc_csr_base);
}


/**
  mobiveil_pcie_parse_dt - routine to parse the device tree structure and extract the resource info

  @pcie    pointer to root port
  @return  -ENODEV/ if no "gpex_csr_base" or "config_apio_base" is set in the DTB,  
  */
static int mobiveil_pcie_parse_dt (struct mobiveil_pcie *pcie) 
{    
    struct resource *gpex_csr_base, *config_apio_base;
    struct platform_device *pdev = pcie->pdev;
    struct device *dev = &pcie->pdev->dev;
    struct device_node *node = dev->of_node;
    u32 ret = 0;

    /* read csr resource */
    gpex_csr_base = platform_get_resource_byname (pdev, IORESOURCE_MEM, "gpex_csr_base");
    if (!gpex_csr_base)
    {
        dev_err (&pdev->dev, "no gpex_csr_base memory resource defined\n");
        return -ENODEV;
    }

    pcie->phy_gpex_csr = gpex_csr_base->start;           /* CSR Base physical address */
    dump_resource(gpex_csr_base);
    /* remap csr resource*/
    pcie->gpex_csr_base = devm_ioremap_resource (&pdev->dev, gpex_csr_base);
    if (IS_ERR (pcie->gpex_csr_base))
    {
        dev_err (&pdev->dev, "failed to map gpex_csr_base memory\n");
        return PTR_ERR (pcie->gpex_csr_base);
    }

    pr_info("%s: gpex_csr_base : 0x%p, *(gpex_csr_base) : 0x%x\n",
            __FUNCTION__, pcie->gpex_csr_base, ioread32 (pcie->gpex_csr_base));

    /* read config resource*/
    config_apio_base = platform_get_resource_byname (pdev, IORESOURCE_MEM, "config_apio_base");
    if (!config_apio_base)
    {
        dev_err (&pdev->dev, "no config_apio_base memory resource defined\n");
        return -ENODEV;
    }
    pcie->phy_apio_base = config_apio_base->start;          /* APIO Base Physical address */

    dump_resource(config_apio_base);
    /* remap config resource*/
    pcie->config_apio_base = devm_ioremap_resource (&pdev->dev, config_apio_base);
    if (IS_ERR (pcie->config_apio_base))
    {
        dev_err (&pdev->dev, "failed to map config_apio_base memory\n");
        return PTR_ERR (pcie->config_apio_base);
    }
    pr_info("%s: config_apio_base : 0x%p\n",  __FUNCTION__, pcie->config_apio_base);

    pcie->irq = irq_of_parse_and_map(node, 0);
    if (pcie->irq <= 0)
    {
        dev_err (&pdev->dev, "failed to get IRQ: %d\n", pcie->irq);
        return -EINVAL;
    }

    ret = devm_request_irq(&pdev->dev, pcie->irq, mobiveil_pcie_isr,
            IRQF_SHARED | IRQF_NO_THREAD,
            "mobiveil-pcie", pcie);
    if (ret) 
    {
        dev_err(&pdev->dev, "unable to request irq %d\n", pcie->irq);
        return ret;
    }

    return ret;
}

/**
  mobi_devm_request_pci_bus_resources - routine to map IO and memory resources of the root port

  @dev          : pointer to device structure
  @resources    : pointer to list head of the resources
  */
int mobi_devm_request_pci_bus_resources (struct device *dev,
        struct list_head *resources)
{
    struct resource_entry *win;
    struct resource *parent, *res;
    int err;

    resource_list_for_each_entry (win, resources)
    {
        res = win->res;
        dump_resource (res);
        switch (resource_type (res))
        {
            case IORESOURCE_IO:
                parent = &ioport_resource;
                break;
            case IORESOURCE_MEM:
                parent = &iomem_resource;
                break;
            default:
                continue;
        }
        err = devm_request_resource (dev, parent, res);
        if (err)
            return err;
    }

    return 0;
}
/**
  mobiveil_pcie_parse_request_of_pci_ranges - routine to parse and map the PCI ranges defined in device tree

  @pcie :    pointer to root port
  */
static int mobiveil_pcie_parse_request_of_pci_ranges (struct mobiveil_pcie *pcie) 
{ 
    int err, res_valid = 1;
    struct device *dev = &pcie->pdev->dev;
    struct device_node *np = dev->of_node;
    struct resource_entry *win, *tmp;
    resource_size_t iobase;

    err = of_pci_get_host_bridge_resources (np, 0, 0xff, &pcie->resources,
            &iobase);
    if (err)
	    return err;

    err = mobi_devm_request_pci_bus_resources (dev, &pcie->resources);
    if (err)
	    goto out_release_res;

    resource_list_for_each_entry_safe (win, tmp, &pcie->resources)
    {
	    struct resource *res = win->res;
	    printk("*_*_*_*_*_*_*_*_*_*_*_*_*_*_*_*_*_*_*_*_*_*_*_*_*_*_*_*_*");
	    dump_resource (res);
	    if (resource_type (res) == IORESOURCE_MEM)
		    res_valid |= !(res->flags & IORESOURCE_PREFETCH);
	    if (resource_type (res) == IORESOURCE_IO) {
		    err = pci_remap_iospace(res, iobase);
		    if(err) { 
		    	    printk("______IO RESOURCE REMAP FAILED [ERROR!!!]____*");
			    printk("**************BABABABABBABABABABABABBABABABABBAB******************");
			    dev_warn(dev,"Error in IO space mapping\n");
			    resource_list_destroy_entry(win);
		    }
	    }
	    printk("*_*_*_*_*_*_*_*_*_*_*_*_*_*_*_*_*_*_*_*_*_*_*_*_*_*_*_*_*");
    }

    if (res_valid)
	    return 0;

    dev_err (dev, "non-prefetchable memory resource required\n");
    err = -EINVAL;

out_release_res:
    pci_free_resource_list (&pcie->resources);
    return err;
}
/**
  w
  program_ib_windows - routine to program the inbound windows of RC

  @pcie   : pointer to rootport
 */
void program_ib_windows(struct mobiveil_pcie *pcie)
{
	int value;
	int window = WIN_NUM_1;
	int ib_start = 0x00000000;//0x62000000;
	pab_pex_amap_ctrl_t amap_ctrl;

	pr_info ("program_ib_windows: ib_window: %x, \n", window);

	value = csr_readl (pcie, PAB_PEX_PIO_CTRL_REG);
	csr_writel (pcie, value|0x01, PAB_PEX_PIO_CTRL_REG);

	amap_ctrl.dw = access_paged_register (pcie, OP_READ, 0, PAB_PEX_AMAP_CTRL(window));
	amap_ctrl.enable = 1;
	amap_ctrl.type = 3; /* 0: interrupt, 2: prefetchable memory, 3-Non-prefetable*/
	amap_ctrl.size = 0x3fffff; /* *2GB*/
	
	access_paged_register (pcie, OP_WRITE, amap_ctrl.dw, PAB_PEX_AMAP_CTRL(window));
	access_paged_register (pcie, OP_WRITE, 0x62000000, PAB_PEX_AMAP_AXI_WIN(window));
	access_paged_register (pcie, OP_WRITE, 0x80000000, PAB_PEX_AMAP_PEX_WIN_L(window));
	access_paged_register (pcie, OP_WRITE, 0x00000000, PAB_PEX_AMAP_PEX_WIN_H(window));

	pr_info("______________________IB_WINDOWS___________________________________ \n");
	pr_info ("%s: PAB_PEX_PIO_CTRL_REG register[1x%x] : 0x%x\n",  __FUNCTION__, PAB_PEX_PIO_CTRL_REG, csr_readl (pcie, PAB_PEX_PIO_CTRL_REG));
	pr_info ("%s: PAB_PEX_AMAP_CTRL register[0x%x] : 0x%x\n", __FUNCTION__, PAB_PEX_AMAP_CTRL(window), access_paged_register(pcie, OP_READ, 0, PAB_PEX_AMAP_CTRL(window)));
	pr_info ("%s: PAB_PEX_AMAP_AXI_WIN register[0x%x] : 0x%x\n", 
			__FUNCTION__, PAB_PEX_AMAP_AXI_WIN(window), access_paged_register (pcie, OP_READ, 0, PAB_PEX_AMAP_AXI_WIN(window)));
	pr_info ("%s: PAB_PEX_AMAP_PEX_WIN_L register[0x%x] : 0x%x\n", 
			__FUNCTION__, PAB_PEX_AMAP_PEX_WIN_L(window), access_paged_register (pcie, OP_READ, 0, PAB_PEX_AMAP_PEX_WIN_L(window)));
	pr_info ("%s: PAB_PEX_AMAP_PEX_WIN_H(window) register[0x%x] : 0x%x\n", 
			__FUNCTION__, PAB_PEX_AMAP_PEX_WIN_H(window), access_paged_register (pcie, OP_READ, 0, PAB_PEX_AMAP_PEX_WIN_H(window)));
}

/**
  program_ob_windows - routine to program the outbound windows of RC

  @pcie                 : pointer to rootport
  @win_num              : window number
  @pci_axi_window_base  : AXI window base
  @pex_addr_base_lower  : PCI window base
  @config_io_bit        : flag bit to indecate memory or IO type
  @size_kb              : window size requested
*/

void program_ob_windows ( struct mobiveil_pcie *pcie, int win_num,
		int pci_axi_window_base, int pex_addr_base_lower,
		int config_io_bit, int size_kb)
{
	u32 value, type, i;

	if(pcie->ob_wins_configured > win_num)
	{
		pr_info("%s already configured !!!\n",__func__);
	}

	pr_info ("program_ob_windows: ob_window: %x, \n", win_num);

	/* Program PIO Enable Bit to 1 and Config Window Enable Bit to 1 in PAB_AXI_PIO_CTRL Register*/

	/* Program Enable Bit to 1, Type Bit to (00) base 2, AXI Window Size Bit to 4 KB in PAB_AXI_AMAP_CTRL Register*/
	type = (MEM_WINDOW_ENABLE_BIT == config_io_bit) ? 2 : (IO_WINDOW_ENABLE_BIT == config_io_bit) ? 1 : 0;
	printk("ob_window type = %d", type);
	value = csr_readl (pcie, PAB_AXI_AMAP_CTRL_REG (win_num));
	csr_writel (pcie, 1 << ENABLE_BIT | type << TYPE_BIT | size_kb << AXI_WINDOW_SIZE_BIT, PAB_AXI_AMAP_CTRL_REG (win_num));


	/* Program AXI window base with appropriate value in PAB_AXI_AMAP_AXI_WIN0 Register*/
	value = csr_readl (pcie, PAB_AXI_AMAP_AXI_WIN_REG (win_num));
	csr_writel (pcie, pci_axi_window_base /*PCI_AXI_WINDOW_BASE */  ,
			PAB_AXI_AMAP_AXI_WIN_REG (win_num));

	value = csr_readl (pcie, PAB_AXI_AMAP_PEX_WIN_H_REG (win_num));
	csr_writel (pcie, pex_addr_base_lower, PAB_AXI_AMAP_PEX_WIN_L_REG (win_num));
	csr_writel (pcie, 0, PAB_AXI_AMAP_PEX_WIN_H_REG (win_num));

	for( i =0; i < 0x40; i++ ) {
		csr_writel (pcie, 0xFFFFFFFF, PAB_AXI_PIO_STAT(win_num));
	}

	pr_info ("%s: PAB_AXI_PIO_CTRL register[0x%x] : 0x%x\n", __FUNCTION__,
			PAB_AXI_PIO_CTRL_REG (0), csr_readl (pcie, PAB_AXI_PIO_CTRL_REG (0)));
	pr_info ("%s: PAB_AXI_AMAP_CTRL_REG[%x] : 0x%x\n", __FUNCTION__,
			PAB_AXI_AMAP_CTRL_REG (win_num), csr_readl (pcie, PAB_AXI_AMAP_CTRL_REG(win_num)));
	pr_info ("%s: PAB_AXI_AMAP_AXI_WIN_REG[0x%x] : 0x%x\n", __FUNCTION__,
			PAB_AXI_AMAP_AXI_WIN_REG (win_num), csr_readl (pcie, PAB_AXI_AMAP_AXI_WIN_REG(win_num)));
	pr_info ("%s: PAB_AXI_AMAP_PEX_WIN_L_REG[0x%x] : 0x%x\n", __FUNCTION__,
			PAB_AXI_AMAP_PEX_WIN_H_REG (win_num), csr_readl (pcie, PAB_AXI_AMAP_PEX_WIN_L_REG(win_num)));
	pr_info("Device & vendor ID 0x%x\r\n\n", ioread32(pcie->config_apio_base));
	pr_info("Cmmand and status 0x%x\r\n\n",  ioread32(pcie->config_apio_base +4));
	pr_info("Init ID & class code 0x%x\r\n\n",  ioread32(pcie->config_apio_base + 8));
	pr_info("cache line & header type 0x%x\r\n\n",  ioread32(pcie->config_apio_base + 12));

	pr_info("PAB_AXI_PIO_STAT 0x%x\r\n\n",csr_readl(pcie, 0x844));
	pr_info("PCI base address + 848 : 0x%x\r\n", csr_readl(pcie, 0x848));
	pr_info("PCI base address + 84C : 0x%x\r\n", csr_readl(pcie, 0x84C));

	pcie->ob_wins_configured++;
}

/*
   mobiveil_pcie_setup_csr_for_config_access - routine to prepare the RC for config access

   @pcie                 : pointer to rootport

 */
void mobiveil_pcie_setup_csr_for_config_access (struct mobiveil_pcie *pcie)
{
	u32 value, type  = 0;
    	struct resource_entry *win, *tmp;
	pab_ctrl_t pab_ctrl;

	/* TODO: Replace numbers with macros*/
	mobiveil_pcie_link_is_up(pcie);

	/* Read Bridge Capability Register to figure out the mode and the number of engine supported by bridge*/
	value = csr_readl (pcie, PAB_CAP_REG);
	pr_info("%s: PAB_CAP register: 0x%x\n", __FUNCTION__, value);

	/* Program Bus Master Enable Bit in Command Register in GPEX Config Space*/
	value = csr_readl (pcie, PCI_COMMAND);
	csr_writel (pcie,value | PCI_COMMAND_IO | PCI_COMMAND_MEMORY | PCI_COMMAND_MASTER, PCI_COMMAND);
	//csr_writel (pcie,value |  PCI_COMMAND_MEMORY | PCI_COMMAND_MASTER, PCI_COMMAND);
	pr_info("%s: PCI_COMMAND 0x%x\n", __FUNCTION__, csr_readl (pcie, PCI_COMMAND));

	/* Program PIO Enable Bit to 1 (and PEX PIO Enable to 1) in PAB_CTRL Register*/
	pab_ctrl.dw = csr_readl (pcie, PAB_CTRL_REG);
	pab_ctrl.amba_pio = 1;
	pab_ctrl.pex_pio = 1;
	pab_ctrl.axi_max_burst_len = 1;
	csr_writel(pcie, 0x1f, PAB_CTRL_REG);
	pr_info ("%s: PAB_CTRL_REG     register[0x%x] : 0x%x\n", __FUNCTION__,
			PAB_CTRL_REG , csr_readl (pcie, PAB_CTRL_REG ));

	csr_writel(pcie, 0x1e8, PAB_INTP_AMBA_MISC_ENB); /* MSI, INTA-INTB*/
	/*Enable AXI -> PEX transfer and windows*/
	value = csr_readl(pcie, PAB_AXI_PIO_CTRL_REG (0));
	csr_writel(pcie, 0xF, PAB_AXI_PIO_CTRL_REG (0));
	/*Enable PEX -> AXI transfer*/
    	value = csr_readl (pcie, PAB_PEX_PIO_CTRL_REG);
	csr_writel(pcie, value|0x01, PAB_PEX_PIO_CTRL_REG);

	//centreno program ob win
	program_ob_windows(pcie, WIN_NUM_0, pcie->phy_apio_base, 0, CONFIG_WINDOW_ENABLE_BIT, 4/*4KB/1024*/);
	program_ib_windows(pcie);

	resource_list_for_each_entry_safe(win, tmp, &pcie->resources) {
		type = 0;
		if (resource_type(win->res) == IORESOURCE_MEM)
			type = MEM_WINDOW_ENABLE_BIT;
		if (resource_type(win->res) == IORESOURCE_IO)
			type = IO_WINDOW_ENABLE_BIT;
		if (type) {
			pr_info("<<<<<<<<<<<<< CONFIGURING WINDOWS >>>>>>>>>>>>>");
			dump_resource(win->res);
			/* configure outbound translation window */
			program_ob_windows(pcie, pcie->ob_wins_configured,
					win->res->start, 0, type,
					resource_size(win->res));
		}
	}



}




/**
  mobiveil_pcie_intx_map - routine to setup the INTx delated data structures

  @domain   : pointer to IRQ domain
  @irq      : IRQ number
  @hwirq    : hardware IRQ number

 */
static int mobiveil_pcie_intx_map (struct irq_domain *domain, unsigned int irq, irq_hw_number_t hwirq) 
{
	printk("%s Domain %p, irq, %d, hwirq %d", __func__, domain, irq, hwirq);
	irq_set_chip_and_handler (irq, &dummy_irq_chip, handle_simple_irq);
	irq_set_chip_data (irq, domain->host_data);
	return 0;
}

/*  INTx domain opeartions structure */
static const struct irq_domain_ops intx_domain_ops = {
	.map = mobiveil_pcie_intx_map,
};

/**
 * mobiveil_pcie_destroy_msi - Free MSI number
 * @irq: IRQ to be freed
 */
static void mobiveil_pcie_destroy_msi(unsigned int irq)
{
	struct msi_desc *msi;
	struct mobiveil_pcie *pcie;

	if (!test_bit(irq, msi_irq_in_use)) 
	{
		msi = irq_get_msi_desc(irq);
		pcie = msi_desc_to_pci_sysdata(msi);
		printk("Trying to free unused MSI#%d\n", irq);
	} else {
		clear_bit(irq, msi_irq_in_use);
	}
}

/**
 * mobiveil_pcie_assign_msi - Allocate MSI number
 * @pcie: PCIe port structure
 *
 * Return: A valid IRQ on success and error value on failure.
 */
static int mobiveil_pcie_assign_msi(struct mobiveil_pcie *pcie)
{
	int pos;

	pos = find_first_zero_bit(msi_irq_in_use, MOBIVEIL_NUM_MSI_IRQS);
	if (pos < MOBIVEIL_NUM_MSI_IRQS)
		set_bit(pos, msi_irq_in_use);
	else
		return -ENOSPC;

	return pos;
}

/**
 * mobiveil_msi_teardown_irq - Destroy the MSI
 * @chip: MSI Chip descriptor
 * @irq: MSI IRQ to destroy
 */
static void mobiveil_msi_teardown_irq(struct msi_controller *chip,
		unsigned int irq)
{
	mobiveil_pcie_destroy_msi(irq);
}

/**
  mobiveil_pcie_msi_setup_irq - routine to compose MSI message descriptor

  @chip   : pointer to MSI controller structure
  @pdev   : pointer to PCI device
  @desc   : MSI descriptor to be setup
 */
static int mobiveil_pcie_msi_setup_irq( struct msi_controller *chip,
		struct pci_dev *pdev,
		struct msi_desc *desc)
{
	int hwirq;
	unsigned int irq;
	struct msi_msg msg;
	u32 val, i;
	phys_addr_t msg_addr;
	struct mobiveil_pcie *pcie = pdev->bus->sysdata;

	hwirq = mobiveil_pcie_assign_msi(pcie);
	if (hwirq < 0)
		return hwirq;

	irq = irq_create_mapping(pcie->irq_domain, hwirq);
	if (!irq)
		return -EINVAL;

	/* Not Supported*/ 
	irq_set_msi_desc(irq, desc);

    msg_addr = ((void *)pcie->msi_pages);

    msg.address_hi = 0;
    msg.address_lo = msg_addr;
    msg.data = irq;
    pr_info("%s: msg.address_lo : 0x%x, data:%x,\n", __FUNCTION__, msg.address_lo,  msg.data);
    pci_write_msi_msg(irq, &msg);
//  pci_read_config_dword(pdev, 0x50, &val);
//  val = val | 0x1 << 16; 
//  pr_info("\n!!!!!!!!!!!!!!!!!VAL = %x!!!!!!!!!!!!!!!!!!!!", val);
//  pci_write_config_dword(pdev, 0x50, &val);
//  pr_info("\n!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
    return 0;
}

/* MSI Chip Descriptor */
static struct msi_controller mobiveil_pcie_msi_chip = {
    .setup_irq = mobiveil_pcie_msi_setup_irq,
    .teardown_irq = mobiveil_msi_teardown_irq,
};

/* HW Interrupt Chip Descriptor */
static struct irq_chip mobiveil_msi_irq_chip = {
    .name = "Mobiveil PCIe MSI",
    .irq_enable = pci_msi_unmask_irq,
    .irq_disable = pci_msi_mask_irq,
    .irq_mask = pci_msi_mask_irq,
    .irq_unmask = pci_msi_unmask_irq,
};

/**
    mobiveil_pcie_msi_map - routine to initialize MSI data structures

    @domain :   pointer IRQ domain
    @irq    :   IRQ number
    @hwirq  :   hardware IRQ number
*/
static int mobiveil_pcie_msi_map(struct irq_domain *domain, unsigned int irq,
        irq_hw_number_t hwirq)
{
    irq_set_chip_and_handler(irq, &mobiveil_msi_irq_chip, handle_simple_irq);
    irq_set_chip_data(irq, domain->host_data);
    return 0;
}

/* MSI IRQ Domain operations */
static const struct irq_domain_ops msi_domain_ops = {
    .map = mobiveil_pcie_msi_map,
};
/**
 * mobiveil_pcie_enable_msi - Enable MSI support
 * @pcie: PCIe port information
 */
static void mobiveil_pcie_enable_msi(struct mobiveil_pcie *pcie)
{
    phys_addr_t msg_addr;
    phys_addr_t pex_start_addr;

    int value;
    int window = WIN_NUM_0;
    pab_pex_amap_ctrl_t amap_ctrl;
    u32 i, rc_stat; 
    pcie->msi_pages = __get_free_pages(GFP_KERNEL, 0);
    msg_addr = virt_to_phys((void *)pcie->msi_pages);
    pcie->msi_pages = (phys_addr_t)msg_addr;
    pex_start_addr = msg_addr;//0x62000000;//msg_addr / 2;
    pr_info ("$$$$$ mobiveil_pcie_enable_msi: win: %x, msg_Addr:%x , PEX_start:%x\n", window, msg_addr, pex_start_addr);

    amap_ctrl.dw = access_paged_register (pcie, OP_READ, 0, PAB_PEX_AMAP_CTRL(window));
    amap_ctrl.enable = 1;
    amap_ctrl.type = 0; /* 0: interrupt, 2: prefetchable memory*/
    amap_ctrl.size = 0x4;//0x3ffff; /* MOBIVEIL_NUM_MSI_IRQS*5DWs*/
    access_paged_register (pcie, OP_WRITE, amap_ctrl.dw, PAB_PEX_AMAP_CTRL(window));

    access_paged_register (pcie, OP_WRITE, pex_start_addr, PAB_PEX_AMAP_AXI_WIN(window));
    access_paged_register (pcie, OP_WRITE, pex_start_addr, PAB_PEX_AMAP_PEX_WIN_L(window));
    access_paged_register (pcie, OP_WRITE, 0x00000000, PAB_PEX_AMAP_PEX_WIN_H(window));
    pr_info ("%s: msi_pages : 0x%lx, msg_addr:%llx\n", __FUNCTION__, pcie->msi_pages, msg_addr );
    pr_info ("%s: PAB_PEX_PIO_CTRL_REG register[0x%x] : 0x%x\n",
            __FUNCTION__, PAB_PEX_PIO_CTRL_REG, csr_readl (pcie, PAB_PEX_PIO_CTRL_REG));
    pr_info ("%s: PAB_PEX_AMAP_CTRL register[0x%x] : 0x%x\n", __FUNCTION__, PAB_PEX_AMAP_CTRL(window),
            access_paged_register(pcie, OP_READ, 0, PAB_PEX_AMAP_CTRL(window)));
    pr_info ("%s: PAB_PEX_AMAP_AXI_WIN register[0x%x] : 0x%x\n",
            __FUNCTION__, PAB_PEX_AMAP_AXI_WIN(window), access_paged_register (pcie, OP_READ, 0, PAB_PEX_AMAP_AXI_WIN(window)));
    pr_info ("%s: PAB_PEX_AMAP_PEX_WIN_L register[0x%x] : 0x%x\n",
            __FUNCTION__, PAB_PEX_AMAP_PEX_WIN_L(window), access_paged_register (pcie, OP_READ, 0, PAB_PEX_AMAP_PEX_WIN_L(window)));
    pr_info ("%s: PAB_PEX_AMAP_PEX_WIN_H register[0x%x] : 0x%x\n",
            __FUNCTION__, PAB_PEX_AMAP_PEX_WIN_H(window), access_paged_register (pcie, OP_READ, 0, PAB_PEX_AMAP_PEX_WIN_H(window)));
}

/**
  mobiveil_pcie_init_irq_domain - routine to setup IRQ domains for both INTx and MSI interrupts 

  @pcie : pointer to pci root port 
*/
static int mobiveil_pcie_init_irq_domain (struct mobiveil_pcie *pcie)
{
    struct device *dev = &pcie->pdev->dev;
    struct device_node *node = dev->of_node;

    /* Setup INTx */
    pcie->irq_domain = irq_domain_add_linear (node, INTX_NUM + 1, &intx_domain_ops, pcie);

    pr_info("mobiveil_pcie_init_irq_domain: domain: %lx \n", (unsigned long )pcie->irq_domain );
    if (!pcie->irq_domain)
    {
        dev_err (dev, "Failed to get a INTx IRQ domain\n");
        return -ENOMEM;
    }
#if 0 
    /* Setup MSI */
    if (IS_ENABLED(CONFIG_PCI_MSI)) 
    {
        pcie->irq_domain = irq_domain_add_linear(node,
                MOBIVEIL_NUM_MSI_IRQS,
                &msi_domain_ops,
                &mobiveil_pcie_msi_chip);
        if (!pcie->irq_domain) 
        {
            dev_err(dev, "Failed to get a MSI IRQ domain\n");
            return PTR_ERR(pcie->irq_domain);
        }

        mobiveil_pcie_enable_msi(pcie);
    }
#endif

    return 0;
}

static int msi_poll_thread (void * p_pcie)
{
	struct mobiveil_pcie *pcie = p_pcie;
	unsigned value,mask, status;
	unsigned mvic_stat;
    	unsigned long mvic_csr_base = 0x71800000;
    	unsigned long *mv_base_va = (unsigned long *) ioremap(mvic_csr_base, 0x1000);
	while(!kthread_should_stop())
	{
		mvic_stat = ioread32(mv_base_va);
		value = csr_readl(pcie, PAB_INTP_AMBA_MISC_STAT);
		mask = csr_readl(pcie, PAB_INTP_AMBA_MISC_ENB);
		status = value & mask;
		printk(KERN_WARNING"%s: misc stat %x  val: %x, mask: %x \n", __FUNCTION__, 
			status, value, mask );
		printk(KERN_WARNING" mvic stat %x\n", mvic_stat); 
		value = csr_readl(pcie,0x080c);
		printk(KERN_WARNING" PAB_BR_STAT value 0x%x", value);
		
		value = csr_readl(pcie, PAB_PEX_PIO_STAT_REG);
		printk(KERN_WARNING" PAB_PEX_PIO_STAT_REG value 0x%x", value);
		value = csr_readl(pcie, PAB_MSI_IB_STAT);
		printk(KERN_WARNING" MSI IB_STAT value 0x%x", value);
		value = csr_readl(pcie, 0x8c8);
		printk(KERN_WARNING" PAX MASTER STATUS value 0x%x", value);
                
		u32 msi_dw[5], dw;
#if 0
                for(dw = 1; dw < 5; dw++ )
		{
			//Only run upto 4 DW reading the last[5th] clears the Status
			msi_dw[dw-1] = csr_readl(pcie, PAB_MSI_IB_FIFO_DW1 + ((dw-1)*4));
			mv_dbg("msi dw[%x]: %x\n", dw, msi_dw[dw-1]);
		}

#endif
			msleep(200);
	return 0;

	}
	pr_err("Kthread stopping\n");
	do_exit(0);
	return 0;
}

/**
  mobiveil_pcie_probe  - probe routine which will get called by kernel once the RC is discovered 

  @pdev :  pointer to platform device
*/
static int mobiveil_pcie_probe (struct platform_device *pdev) 
{ 
    struct  mobiveil_pcie *pcie;
    struct  pci_bus *bus;
    int     ret;
    int     max_bus;
    struct  device_node *np = pdev->dev.of_node;
    unsigned int vendId  = 0;
    unsigned int ClassId = 0; 

    resource_size_t iobase = 0;
    LIST_HEAD (res);

    /* allocate the PCIe port */
    pcie = devm_kzalloc (&pdev->dev, sizeof (*pcie), GFP_KERNEL);
    if (!pcie)
        return -ENOMEM;

    pcie->pdev = pdev;

    mobi_softreset_pcie(pcie);

    /* parse the device tree */
    ret = mobiveil_pcie_parse_dt (pcie);
    if (ret)
    {
        dev_err (&pdev->dev, "Parsing DT failed\n");
        return ret;
    }

    INIT_LIST_HEAD (&pcie->resources);

    /* parse the device tree for PCI ranges and map them */
    ret = mobiveil_pcie_parse_request_of_pci_ranges (pcie);
    if (ret)
    {
        dev_err (&pdev->dev, "Failed add resources\n");
        return ret;
    }
#if 1
    /* initialize the IRQ domains */ 
    ret = mobiveil_pcie_init_irq_domain (pcie);
    if (ret)
    {
        dev_err (&pdev->dev, "Failed creating IRQ Domain\n");
        return ret;
    }
#endif
    /* configure all inbound and outbound windows and prepare teh RC for config access */
    mobiveil_pcie_setup_csr_for_config_access (pcie);

    /* fixup for PCIe config space register data */
    csr_writel (pcie, DEFAULT_PAB_INTP_CLASS_VAL, PAB_INTP_AXI_PIO_CLASS_REG);

    vendId = csr_readl(pcie,PAB_INTP_AXI_PIO_VENID_REG);
    ClassId = csr_readl(pcie,PAB_INTP_AXI_PIO_CLASS_REG);

    dev_err (&pdev->dev, "Vendor ID : 0x%X , Class Code : 0x%X\n", vendId,ClassId);
    dev_err (&pdev->dev, "Vendor ID of EP using APIO Address : 0x%X \n", ioread32(pcie->config_apio_base));

    ret = of_pci_get_host_bridge_resources (np, 0, 0xff, &res, &iobase);
    if (ret)
    {
        printk ( "Getting bridge resources failed\n");
        return ret;
    }

    /* create the PCIe root bus */
    bus = pci_create_root_bus (&pdev->dev, 0, &mobiveil_pcie_ops, pcie, &res);
    if (!bus)
        return -ENOMEM;
#if 0
    /* setup MSI, if enabled */
    if (IS_ENABLED(CONFIG_PCI_MSI))
    {
    	printk ( " %s CONFIG_PCI_MSI Enabled\n", __FUNCTION__);
        mobiveil_pcie_msi_chip.dev = &pcie->pdev->dev;
        bus->msi = &mobiveil_pcie_msi_chip;
    }
#endif

    /* setup the kernel resources for the newly added PCIe root bus */
    printk ( " %s PCI Bus Scan from Probe Started \n", __FUNCTION__);
    max_bus  = pci_scan_child_bus (bus);
    printk ( " %s Max child Bus : %d\n", __FUNCTION__, max_bus);

    pci_fixup_irqs (pci_common_swizzle, of_irq_parse_and_map_pci);
    pci_assign_unassigned_bus_resources (bus);
    pci_bus_add_devices (bus);
    platform_set_drvdata (pdev, pcie);
    g_pcie = pcie;
    kthread_run(msi_poll_thread, pcie, "MSI POll Validation thread");
    printk ( "Mobiveil RC Probe Success\n");
    return ret;
}

/* device id structure mentioning the compatible string to search for in the device tree */
static const struct of_device_id mobiveil_pcie_of_match[] = {
    {.compatible = "mobiveil,mobiveil-gpex-1.0",},
    {},
};
MODULE_DEVICE_TABLE (of, mobiveil_pcie_of_match);

/* platform driver structure */
static struct platform_driver mobiveil_pcie_driver = {
    .probe = mobiveil_pcie_probe,
    .driver = {
        .name = "mobiveil-pcie",
        .of_match_table = mobiveil_pcie_of_match,
        .suppress_bind_attrs = true,
    },
};


/**
    mobiveil_pcie_init -  init routine for the driver module
*/
static int mobiveil_pcie_init (void) 
{     
    int ret = 0;
    ret = platform_driver_register (&mobiveil_pcie_driver);
    if (ret == 0)
        printk ("%s: platform driver registered\n", __FUNCTION__);
    else
        printk ("%s: platform driver not registered\n", __FUNCTION__);

    return ret;
}
module_init (mobiveil_pcie_init);

/**
    mobiveil_pcie_exit -  exit routine for the driver module
*/
static void mobiveil_pcie_exit (void) 
{
    platform_driver_unregister (&mobiveil_pcie_driver);
    printk ("%s: platform driver unregistered\n", __FUNCTION__);
}
module_exit (mobiveil_pcie_exit);


MODULE_AUTHOR ("Karthikeyan Mitran <m.karthikeyan@mobiveil.co.in>");
MODULE_DESCRIPTION ("Mobiveil PCIe host controller driver");
MODULE_LICENSE ("GPL v2");

