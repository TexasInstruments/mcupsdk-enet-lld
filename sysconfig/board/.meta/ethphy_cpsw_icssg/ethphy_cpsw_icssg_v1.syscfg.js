
let common = system.getScript("/common");
let device = common.getDeviceName();

const cpswPhyExtendedConfig = new Map([

['DP83869',{extConfig:
`.txClkShiftEn         = false,
.rxClkShiftEn         = false,
.txDelayInPs          = 2000U,  /* 2.00 ns */
.rxDelayInPs          = 2000U,  /* 2.00 ns */
.txFifoDepth          = 4U,     /* 4 bytes/nibbles */
.impedanceInMilliOhms = 50000U, /* 50 ohms */
.idleCntThresh        = 5U,
.gpio0Mode            = DP83869_GPIO0_RXERR,
.gpio1Mode            = DP83869_GPIO1_COL,
.ledMode              =
{
    DP83869_LED_LINKED,
    DP83869_LED_LINKED_1000BT,
    DP83869_LED_RXTXACT,
    DP83869_LED_LINKED_100BTX,
},`}],

['DP83867',{extConfig:
`/* The delay values are set based on trial and error and not tuned per port of the evm */
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

function getPhyInfo(peripheral)
{
    const cpswPhyInfoMap =
        [
            {deviceName: "am62x-sk", defaultPhy1:"DP83867", defaultPhyAddr1: 0, defaultPhy2:"DP83867", defaultPhyAddr2: 1},
			{deviceName: "am62lx-evm", defaultPhy1:"DP83867", defaultPhyAddr1: 0, defaultPhy2:"DP83867", defaultPhyAddr2: 1},
            {deviceName: "am62x-sk-sip", defaultPhy1:"DP83867", defaultPhyAddr1: 0, defaultPhy2:"DP83867", defaultPhyAddr2: 1},
            {deviceName: "am62ax-sk", defaultPhy1:"DP83867", defaultPhyAddr1: 0, defaultPhy2:"DP83867", defaultPhyAddr2: 1},
            {deviceName: "am62dx-evm", defaultPhy1:"DP83867", defaultPhyAddr1: 0, defaultPhy2:"DP83867", defaultPhyAddr2: 3},
            {deviceName: "am62d-evm (DP83TG721 PHY)", defaultPhy1:"DP83TG721", defaultPhyAddr1: 0, defaultPhy2:"DP83TG721", defaultPhyAddr2: 3},
            {deviceName: "am62px-sk", defaultPhy1:"DP83867", defaultPhyAddr1: 0, defaultPhy2:"DP83867", defaultPhyAddr2: 1},
        ];
    if(peripheral === "am62d-evm (DP83TG721 PHY)"){
        return cpswPhyInfoMap.find(element => element.deviceName === peripheral);
    }
    else{
        return cpswPhyInfoMap.find(element => element.deviceName === device);
    }
}

function getExternalConfig(phyUsed, peripheral)
{
    return cpswPhyExtendedConfig.get(phyUsed).extConfig;
}

function getMaxInstanceCount()
{
    return 2;
}

exports = {
    getExternalConfig,
    getPhyInfo,
    getMaxInstanceCount,
};
