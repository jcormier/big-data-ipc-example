
var socType = "am572x";

/*use CSL package*/
var Csl = xdc.loadPackage('ti.csl');
Csl.Settings.deviceType = socType;

/* Load the osal package -- required by board & interrupt example */
var osType = "tirtos"
var Osal = xdc.loadPackage('ti.osal');
Osal.Settings.osType = osType;

/* Load the I2C package  - required by board */
var I2c = xdc.loadPackage('ti.drv.i2c');
I2c.Settings.socType = socType;

/* Load the uart package -- required by board */
var Uart = xdc.loadPackage('ti.drv.uart');
Uart.Settings.socType = socType;

/* Load the Board package and set the board name */
var Board = xdc.loadPackage('ti.board');
Board.Settings.boardName = "evmAM572x";

/* ================ Cache configuration ================ */
var Cache  = xdc.useModule('ti.sysbios.family.arm.a15.Cache');

/* Enable the cache */
Cache.enableCache = true;

/* ================ MMU configuration ================ */
var Mmu = xdc.useModule('ti.sysbios.family.arm.a15.Mmu');

/* Enable the MMU (Required for L1 data caching) */
Mmu.enableMMU = true;

var attrs = new Mmu.DescriptorAttrs();
Mmu.initDescAttrsMeta(attrs);
attrs.type = Mmu.DescriptorType_BLOCK;
attrs.noExecute = true;
attrs.accPerm = 0;       // R/W at PL1
attrs.attrIndx = 3;       // Use MAIR0 Byte2
Mmu.setMAIRMeta(3, 0x04);
Mmu.setSecondLevelDescMeta(0x43200000, 0x43200000, attrs);

/* Set IO Delay configuration areas as non-cache */
attrs.attrIndx = 1;
Mmu.setSecondLevelDescMeta(0x4844a000, 0x4844a000, attrs);
Mmu.setSecondLevelDescMeta(0x4ae07d00, 0x4ae07d00, attrs);

/* ================ Memory sections configuration ================ */
Program.sectMap["BOARD_IO_DELAY_DATA"] = "OCMC_RAM1";
Program.sectMap["BOARD_IO_DELAY_CODE"] = "OCMC_RAM1";

