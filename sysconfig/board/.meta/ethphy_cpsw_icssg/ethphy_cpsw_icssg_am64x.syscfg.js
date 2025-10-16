
let common = system.getScript("/common");
let device = common.getDeviceName();

const cpswPhyExtendedConfig = new Map([
	 
	['DP83869',{extConfig:
`/* Extended PHY configuration for DP83869 */
.txClkShiftEn         = false,
.rxClkShiftEn         = true,
.txDelayInPs          = 500U, /* Value in pecosec. Refer to DLL_RX_DELAY_CTRL_SL field in ANA_RGMII_DLL_CTRL register of DP83869 PHY datasheet */
.rxDelayInPs          = 500U, /* Value in pecosec. Refer to DLL_TX_DELAY_CTRL_SL field in ANA_RGMII_DLL_CTRL register of DP83869 PHY datasheet */
.txFifoDepth          = 4U,
.impedanceInMilliOhms = 35000,  /* 35 ohms */
.idleCntThresh        = 4U,     /* Improves short cable performance */
.gpio0Mode            = DP83869_GPIO0_RX_SFD,
.gpio1Mode            = DP83869_GPIO1_COL, /* Unused */
.ledMode              =
{
	DP83869_LED_LINKED,         /* Unused */
	DP83869_LED_LINKED_100BTX,
	DP83869_LED_RXTXACT,
	DP83869_LED_LINKED_1000BT,
},`}],

	['DP83867',{extConfig:
`/* Extended PHY configuration for DP83867 */
.txClkShiftEn         = true,
.rxClkShiftEn         = true,
.txDelayInPs          = 250U,   /* 0.25 ns */
.rxDelayInPs          = 2000U,  /* 2.00 ns */
.txFifoDepth          = 4U,
.impedanceInMilliOhms = 35000,  /* 35 ohms */
.idleCntThresh        = 4U,     /* Improves short cable performance */
.gpio0Mode            = DP83867_GPIO0_LED3,
.gpio1Mode            = DP83867_GPIO1_COL, /* Unused */
.ledMode              =
{
	DP83867_LED_LINKED,         /* Unused */
	DP83867_LED_LINKED_100BTX,
	DP83867_LED_RXTXACT,
	DP83867_LED_LINKED_1000BT,
},`}],

	['DP83822',{extConfig:""}],

	['DP83826',{extConfig:""}],

	['DP83TC812',{extConfig:""}],

	['DP83TG720',{extConfig:
`.txClkShiftEn = true,
.rxClkShiftEn = true,
.interruptEn = false,
.sgmiiAutoNegEn = true,
.MasterSlaveMode = DP83TG720_MASTER_SLAVE_STRAP,`}],

	['DP83TG721',{extConfig:""}],

	['CUSTOM',{extConfig:""}],

	['NO-PHY',{extConfig:""}],
],
);

const icssPhyExtendedConfig = new Map([

['DP83869',{extConfig:
`.txClkShiftEn         = false,
.rxClkShiftEn         = true,
.txDelayInPs          = 750U, /* Value in pecosec. Refer to DLL_RX_DELAY_CTRL_SL field in ANA_RGMII_DLL_CTRL register of DP83869 PHY datasheet */
.rxDelayInPs          = 2000U, /* Value in pecosec. Refer to DLL_TX_DELAY_CTRL_SL field in ANA_RGMII_DLL_CTRL register of DP83869 PHY datasheet */
.txFifoDepth          = 4U,
.impedanceInMilliOhms = 35000,  /* 35 ohms */
.idleCntThresh        = 4U,     /* Improves short cable performance */
.gpio0Mode            = DP83869_GPIO0_RX_SFD,
.gpio1Mode            = DP83869_GPIO1_COL, /* Unused */
.ledMode              =
{
	DP83869_LED_LINKED,         /* Unused */
	DP83869_LED_LINKED_100BTX,
	DP83869_LED_RXTXACT,
	DP83869_LED_LINKED_1000BT,
},`}],

['DP83867',{extConfig:
`.txClkShiftEn         = true,
.rxClkShiftEn         = true,
.txDelayInPs          = 250U,   /* 0.25 ns */
.rxDelayInPs          = 2000U,  /* 2.00 ns */
.txFifoDepth          = 4U,
.impedanceInMilliOhms = 35000,  /* 35 ohms */
.idleCntThresh        = 4U,     /* Improves short cable performance */
.gpio0Mode            = DP83867_GPIO0_LED3,
.gpio1Mode            = DP83867_GPIO1_COL, /* Unused */
.ledMode              =
{
	DP83867_LED_LINKED,         /* Unused */
	DP83867_LED_LINKED_100BTX,
	DP83867_LED_RXTXACT,
	DP83867_LED_LINKED_1000BT,
},`}],

	['DP83822',{extConfig:""}],

	['DP83826',{extConfig:""}],

	['DP83TC812',{extConfig:""}],

	['DP83TG720',{extConfig:
`.txClkShiftEn = true,
.rxClkShiftEn = true,
.interruptEn = false,
.sgmiiAutoNegEn = true,
.MasterSlaveMode = DP83TG720_MASTER_SLAVE_STRAP,`}],

	['DP83TG721',{extConfig:""}],

	['CUSTOM',{extConfig:""}],

	['NO-PHY',{extConfig:""}],
],
);


function getPhyInfo(peripheral)
{
    const cpswPhyInfoMap = 
		[
			{deviceName: "am64x-evm", defaultPhy1:"DP83867", defaultPhyAddr1: 0, defaultPhy2:"DP83869", defaultPhyAddr2: 3},
		];
	const icssgPhyInfoMap = 
		[
			{deviceName:"am64x-evm", instance:"ICSSG1", defaultPhy1:"DP83869", defaultPhyAddr1: 15, defaultPhy2:"DP83869", defaultPhyAddr2: 3},
			{deviceName:"am64x-evm", instance:"ICSSG0", defaultPhy1:"DP83826", defaultPhyAddr1: 1, defaultPhy2:"DP83826", defaultPhyAddr2: 7},
		];

	if (peripheral.includes("ICSS")){
		return icssgPhyInfoMap.find(element => element.deviceName === device
            						&& peripheral.includes(element.instance));
	}
    else {
		return cpswPhyInfoMap.find(element => element.deviceName === device);
	}
}

function getExternalConfig(phyUsed, peripheral)
{
	if(peripheral.includes("CPSW")){
		return cpswPhyExtendedConfig.get(phyUsed).extConfig;
	}
	else {
		return icssPhyExtendedConfig.get(phyUsed).extConfig;
	}
}

function getMaxInstanceCount()
{
	return 5;
}

exports = {
	getExternalConfig,
	getPhyInfo,
	getMaxInstanceCount,
};
