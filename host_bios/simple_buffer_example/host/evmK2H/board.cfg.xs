var CpIntc =   xdc.useModule('ti.sysbios.family.arm.a15.tci66xx.CpIntc');

var devType = "k2h"

/* Load the OSAL package */ 
var osType = "tirtos"
var Osal = xdc.useModule('ti.osal.Settings');
Osal.osType = osType;
Osal.socType = devType;

/*use CSL package*/
var Csl = xdc.loadPackage('ti.csl');
Csl.Settings.deviceType = devType;

/* Load the uart package */
var Uart = xdc.loadPackage('ti.drv.uart');
Uart.Settings.socType = devType;

/* Load the I2C package */
var I2c                         = xdc.loadPackage('ti.drv.i2c');
I2c.Settings.socType = devType;

/* Load the Board package and set the board name */
var Board = xdc.loadPackage('ti.board');
Board.Settings.boardName = "evmK2H";

/* ================ Cache configuration ================ */
var Cache  = xdc.useModule('ti.sysbios.family.arm.a15.Cache');

/* Enable the cache */
Cache.enableCache = true;

/* ================ MMU configuration ================ */
var Mmu = xdc.useModule('ti.sysbios.family.arm.a15.Mmu');

/* Enable the MMU (Required for L1/L2 data caching) */
Mmu.enableMMU = true;

/* descriptor attribute structure */
var peripheralAttrs = new Mmu.DescriptorAttrs();

Mmu.initDescAttrsMeta(peripheralAttrs);

peripheralAttrs.type = Mmu.DescriptorType_BLOCK;  // BLOCK descriptor
peripheralAttrs.noExecute = true;                 // not executable
peripheralAttrs.accPerm = 0;                      // read/write at PL1
peripheralAttrs.attrIndx = 1;                     // MAIR0 Byte1 describes
                                                      // memory attributes for
                                                      // each BLOCK MMU entry

/* Define the base address of the 2 MB page
   the peripheral resides in.
*/
var peripheralBaseAddrs = [ 
  { base: 0x4ae00000, size: 0x00100000 },  // PRM
  { base: 0x02530C00, size: 0x00000400 },  // UART 0 regs
  { base: 0x02531000, size: 0x00000400 }   // UART 1 regs
];

/* Configure the corresponding MMU page descriptor accordingly */
for (var i =0; i < peripheralBaseAddrs.length; i++)
{
  for (var j = 0; j < peripheralBaseAddrs[i].size; j += 0x200000)
  {
      var addr = peripheralBaseAddrs[i].base + j;
      Mmu.setSecondLevelDescMeta(addr, addr, peripheralAttrs);
  }
}
