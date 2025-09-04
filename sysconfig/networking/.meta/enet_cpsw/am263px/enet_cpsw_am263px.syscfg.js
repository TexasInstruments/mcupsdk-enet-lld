"use strict";

let common = system.getScript("/common");
let pinmux = system.getScript("/drivers/pinmux/pinmux");

let soc = system.getScript(`/networking/soc/networking_${common.getSocName()}`);
//Get packet pool configuration script
const pktPoolScript = system.getScript("./enet_pkt_pool_config");
//Get ALE configuration script
const aleScript = system.getScript("./enet_cpsw_ale_config");
//Get MDIO configuration script
const mdioScript = system.getScript("./enet_cpsw_mdio_config");
//Get Host Port configuration script
const hostportScript = system.getScript("./enet_cpsw_hostport_config");
//Get CPTS configuration script
const cptsScript = system.getScript("./enet_cpsw_cpts_config");
//Get MAC Port configuration script
const macportScript = system.getScript("./enet_cpsw_macport_config");
const utilsScript = system.getScript("./../../common/enet_cpsw_utils");
const pinMuxScript = system.getScript("./enet_cpsw_am263px_pinmux");

const enet_cpsw_pinmux_config = {
    name: "pinmuxConfig",
    displayName: "Pinmux config",
	longDescription: "Configuration of pinmux for CPSW",
    collapsed:true,
    config: [

    ],
};

const enet_cpsw_cpdma_channel_config = {
    name: "cpdmaChConfig",
    displayName: "DMA channel config",
	longDescription: "Configuration of Tx/Rx DMA channels",
    collapsed:true,
    config: [

    ],
};

const enet_cpsw_lwipIf_config = {
    name: "lwipIfConfig",
    displayName: "LWIP Interface config",
	longDescription: "Configuration of LWIP Interface",
    collapsed:true,
    config: [

    ],
};

const enet_cpsw_system_config = {
    name: "cpswSystemConfig",
    displayName: "System integration config",
    longDescription: "System integration related configuration",
    collapsed:true,
    config: [
        {
            name: "McmEnable",
            description: "Flag to enable multi-client manager. Required for multi-core, multiple Enet client use cases",
            displayName: "Mcm Enable",
            default: false,
        },
        {
            name: "ExternalPhyMgmtEnable",
            description: "Flag to enable phy management in application. The enet driver internal phy functions including phy state machine is bypassed in this mode",
            displayName: "External Phy Management Enable",
            default: false,
            onChange:function (inst, ui) {
                if(inst.ExternalPhyMgmtEnable == true) {
                    ui.macport1LinkSpeed.hidden = true;
                    ui.macport1LinkDuplexity.hidden = true;
                    ui.macport2LinkSpeed.hidden = true;
                    ui.macport2LinkDuplexity.hidden = true;
                }
                else{
                    ui.macport1LinkSpeed.hidden = false;
                    ui.macport1LinkDuplexity.hidden = false;
                    ui.macport2LinkSpeed.hidden = false;
                    ui.macport2LinkDuplexity.hidden = false;
                }
            }
        },
        {
            name: "RtosVariant",
            description: "Select FreeRTOS or No RTOS",
            displayName: "RTOS Variant",
            default: "FreeRTOS",
            options: [
                {
                    name: "FreeRTOS",
                    displayName: "FreeRTOS",
                },
                {
                    name: "NoRTOS",
                    displayName: "No RTOS (Bare Metal)",
                },
            ],
        },
        {
            name: "macAddrConfig",
            description: "MAC address to set in the driver. 'Auto Assign shall select the address automatiically from EEPROM and/or EFUSES. 'Manual Entry' will allow to input MAC address",
            displayName: "MAC Address Assignment Method",
            onChange:function (inst, ui) {
                if(inst.macAddrConfig === "Auto Assign") {
                    ui.macAddrList.hidden = true;
                } else {
                    ui.macAddrList.hidden = false;
                }
            },
            options: [
                {
                    name: "Auto Assign",
                },
                {
                    name: "Manual Entry",
                },
            ],
            default: "Auto Assign",
        },
        {
            name: "macAddrList",
            description: "MAC address to set in the driver. Enter MAC address. Seperate multiple MAC address with comma. Eg.: aa:bb:bb:cc:dd:ee,01:22:33:aa:bb:ee",
            displayName: "MAC Address List",
            default: "70:ff:76:1d:ec:f2,70:ff:76:1d:ec:e3",
            hidden: true,
        },
        {
            name: "AppLinkUpPortMask",
            description: "Application config to determine which macPorts should be polled for linkup to indicate link is up.Applicable in multi port scenario only",
            displayName: "AppLinkUpPortMask Config",
            default: "ANY_PORT",
            options: [
                {
                    name: "ALL_PORTS",
                },
                {
                    name: "ANY_PORT",
                },
            ],
        },
    ],
};

const enet_cpsw_board_config = {
    name: "cpswBoardConfig",
    displayName: "Board Config",
    longDescription: "Board specific configuration",
    collapsed:true,
    config: [
        {
            name: "customBoardEnable",
            description: "Enable Custom Board Configuration",
            displayName: "Custom Board",
            longDescription: "Configuration for custom board that are not supported out of box in MCU+ SDK",
            default: false,
        },
    ],
};

function getInterfaceNameList(inst) {
    return pinMuxScript.getInterfaceNameList(inst);
}

function pinmuxRequirements(inst) {
    return pinMuxScript.pinmuxRequirements(inst);
}

function getPeripheralPinNames(inst)
{
    return pinMuxScript.getPeripheralPinNames(inst);
}

const enet_clock_config =
    {
        clockIds        : [ "SOC_RcmPeripheralId_CPTS" ],
        clockFrequencies: [
            {
                moduleId: "SOC_RcmPeripheralId_CPTS",
                clkId   : "SOC_RcmPeripheralClockSource_SYS_CLK",
                clkRate : 200000000,
            },
        ],
    };

function getClockEnableIds(instance) {
    let instConfig = enet_clock_config;
    return instConfig.clockIds;
}

function getClockFrequencies(inst) {
    let instConfig = enet_clock_config;
    return instConfig.clockFrequencies;
}

function getDmaInterface(instance) {
    let cpswInstInfo = getCpswInstInfo(instance);
    return cpswInstInfo.dmaIf;
}

function getInstIdTable(instances) {
    let tbl = '{ '
    for (var i = 0; i < instances.length; i++)
    {
        tbl += '{';
        var matchedInst = getCpswInstInfo(instances[i])
        tbl += i + ', ' + matchedInst.enetType + ', ' +  matchedInst.instId
        tbl += '}, '
    }
    tbl += '}'
    return tbl;
}

function getCpswInstInfo(instance) {
    const cpswInstInfoMap = new Map(
                               [
                                 ['am263px',{enetType: 'ENET_CPSW_3G', numMacPorts: '2', instId: '0', dmaIf:'ENET_SOC_HOSTPORT_DMA_TYPE_CPDMA', macPortList:['ENET_MAC_PORT_1', 'ENET_MAC_PORT_2']}],
                               ],
                             );
    let instInfo =  cpswInstInfoMap.get(common.getSocName());
    instInfo.macPortList = instInfo.macPortList.filter(function(macPort, index,arr){
        let includeEntry = true;
        if ((macPort === 'ENET_MAC_PORT_1') && (instance.DisableMacPort1 === true))
        {
            includeEntry = false;
        }
        if ((macPort === 'ENET_MAC_PORT_2') && (instance.DisableMacPort2 === true))
        {
            includeEntry = false;
        }
        return includeEntry;
    });
    instInfo.numMacPorts = instInfo.macPortList.length;
    return instInfo;
}

function getBoardConfigTemplateInfo() {
    const boardConfigTemplate = new Map(
                               [
                                 ['am263px',{Cfile: "/board/ethphy_cpsw_icssg/templates/am263px/enet_board_cfg.c.xdt",
                                 Header: "/board/ethphy_cpsw_icssg/templates/am263px/enet_board_cfg.h.xdt"}],
                               ],
                             );
    return boardConfigTemplate.get(common.getSocName());
}


function getSocConfigTemplateInfo() {
    const socConfigTemplate = new Map(
                               [
                                 ['am263px',{Cfile: "/networking/enet_cpsw/templates/am263px/enet_soc_cfg.c.xdt"}],
                               ],
                             );
    return socConfigTemplate.get(common.getSocName());
}

function getPacketsCount(instance, channelType) {
    let totalNumPackets = 0;
    let driverVer = soc.getDriverVer("enet_cpsw");
    let dma_ch_instances;
    let module_dma_ch;

    if (channelType === "TX")
    {
        dma_ch_instances = instance.txDmaChannel;
        module_dma_ch = system.modules[`/networking/enet_cpsw/${driverVer}/enet_cpsw_${driverVer}_tx_channel`];
    }
    else
    {
        dma_ch_instances = instance.rxDmaChannel;
        module_dma_ch = system.modules[`/networking/enet_cpsw/${driverVer}/enet_cpsw_${driverVer}_rx_channel`];
    }

    for(let ch = 0; ch < dma_ch_instances.length; ch++) {
        let ch_instance = dma_ch_instances[ch];
        let ch_config = module_dma_ch.getInstanceConfig(ch_instance);
        totalNumPackets += ch_config.PacketsCount;
    }
    return totalNumPackets;
}

function getChannelCount(instance, channelType) {
    let totalNumChannels = 0;
    let driverVer = soc.getDriverVer("enet_cpsw");
    let dma_ch_instances;
    let module_dma_ch;

    if (channelType === "TX")
    {
        dma_ch_instances = instance.txDmaChannel;
        module_dma_ch = system.modules[`/networking/enet_cpsw/${driverVer}/enet_cpsw_${driverVer}_tx_channel`];
    }
    else
    {
        dma_ch_instances = instance.rxDmaChannel;
        module_dma_ch = system.modules[`/networking/enet_cpsw/${driverVer}/enet_cpsw_${driverVer}_rx_channel`];
    }

    for(let ch = 0; ch < dma_ch_instances.length; ch++) {
        let ch_instance = dma_ch_instances[ch];
        let ch_config = module_dma_ch.getInstanceConfig(ch_instance);
        totalNumChannels++;
    }
    return totalNumChannels;
}

function getTxPacketsCount(instance) {
    return getPacketsCount(instance, "TX");
}

function getRxPacketsCount(instance) {
    return getPacketsCount(instance, "RX");
}

function getTxChannelCount(instance) {
    return getChannelCount(instance, "TX");
}

function getRxChannelCount(instance) {
    return getChannelCount(instance, "RX");
}

function getNumCpdmaDesc(instance) {
    /* Tx packet requires Two Tx scatter gather segments + 1 csum offload descriptor */
    const txDesc2PacketScalingFactor = 3;
    /* Rx packet requires only one desc per packet till scatter gather is supported.
     * Csum info is at end of packet and no cpdma desc is used */
    const rxDesc2PacketScalingFactor = 1;
    let cpdmaNumDesc = (rxDesc2PacketScalingFactor * getRxPacketsCount(instance)) + (txDesc2PacketScalingFactor * getTxPacketsCount(instance));
    return  cpdmaNumDesc;
}

function getChannelConfig(instance, channelType, chTypeInstNum) {
    let driverVer = soc.getDriverVer("enet_cpsw");
    let dma_ch_instances;
    let module_dma_ch;

    if (channelType === "TX")
    {
        dma_ch_instances = instance.txDmaChannel;
        module_dma_ch = system.modules[`/networking/enet_cpsw/${driverVer}/enet_cpsw_${driverVer}_tx_channel`];
    }
    else
    {
        dma_ch_instances = instance.rxDmaChannel;
        module_dma_ch = system.modules[`/networking/enet_cpsw/${driverVer}/enet_cpsw_${driverVer}_rx_channel`];
    }
    let channelCfgArray = new Array();


    for(let ch = 0; ch < dma_ch_instances.length; ch++) {
        let ch_instance = dma_ch_instances[ch];
        let ch_config = module_dma_ch.getInstanceConfig(ch_instance);
        channelCfgArray.push(ch_config);
    }
    return channelCfgArray[chTypeInstNum];
}

function getDefaultPacketCount(channelType) {
    let driverVer = soc.getDriverVer("enet_cpsw");
    let module_dma_ch;

    if (channelType === "TX")
    {
        module_dma_ch = system.modules[`/networking/enet_cpsw/${driverVer}/enet_cpsw_${driverVer}_tx_channel`];
    }
    else
    {
        module_dma_ch = system.modules[`/networking/enet_cpsw/${driverVer}/enet_cpsw_${driverVer}_rx_channel`];
    }
    return (module_dma_ch.config.filter(o => o.name === 'PacketsCount'))[0].default;
}


function getNetifCount(instance) {
    let driverVer = soc.getDriverVer("enet_cpsw");
    let totalNumNetifs = 0;
    let instances;
    let module;

    instances = instance.netifInstance;
    module = system.modules[`/networking/enet_cpsw/${driverVer}/enet_cpsw_lwipif_netif`];

    for(let num = 0; num < instances.length; num++) {
        let num_instance = instances[num];
        totalNumNetifs++;
    }
    return totalNumNetifs;
}

function getNetifConfig(instance, InstNum) {
    let driverVer = soc.getDriverVer("enet_cpsw");
    let instances;
    let module;

    instances = instance.netifInstance;
    module = system.modules[`/networking/enet_cpsw/${driverVer}/enet_cpsw_lwipif_netif`];

    let cfgArray = new Array();


    for(let num = 0; num < instances.length; num++) {
        let num_instance = instances[num];
        let num_config = module.getInstanceConfig(num_instance)[`moduleInstance`];
        cfgArray.push(num_config);
    }
    return cfgArray[InstNum];
}

function getNetifPacketDequeueMode(instance){
    let enableTimerBasedPoll = (getNetifConfig(instance, 0).packetDequeueMode === "TimerBasedPolling") ? 1 : 0;
    return enableTimerBasedPoll;     
}

function getNetifEtherringSupport(instance){
    let etherringLwipSupport = getNetifConfig(instance, 0).etherRingLwipSupport;
    return etherringLwipSupport;     
}

function verifyNetifPacketDequeueMode(instance){
    let timerEnabledNetifcount = 0;
    let firstNetifMode = getNetifConfig(instance, 0).packetDequeueMode;

    for (let Idx = 1; Idx < getNetifCount(instance); Idx++)
    {
        if(getNetifConfig(instance, Idx).packetDequeueMode !==firstNetifMode){
            return false;
        }
    }
    return true;
}

function getDefaultNetifCount(instance)
{
    let defaultNetifCount = 0;

    for (let Idx = 0; Idx < getNetifCount(instance); Idx++)
    {
        defaultNetifCount += (getNetifConfig(instance, Idx).isDefault === true) ? 1 : 0;
    }
    return defaultNetifCount;

}

function getDefaultNetifIdx(instance)
{
    let defaultNetifIdx = -1;

    for (let Idx = 0; Idx < getNetifCount(instance); Idx++)
    {
        if(getNetifConfig(instance, Idx).isDefault === true)
        {
            defaultNetifIdx = Idx;
            break;
        }
    }
    return defaultNetifIdx;
}

function getCpuID() {
    return system.getScript(`/drivers/soc/drivers_${common.getSocName()}`).getCpuID();
}

function getMiiConfig(instance) {
    const cpswMiiConfigMap = new Map(
    [
        ["RGMII",{layerType:"ENET_MAC_LAYER_GMII", variantType:"ENET_MAC_VARIANT_FORCED", sublayerType:"ENET_MAC_SUBLAYER_REDUCED"}],
        ["RMII", {layerType:"ENET_MAC_LAYER_MII", variantType:"ENET_MAC_VARIANT_NONE", sublayerType:"ENET_MAC_SUBLAYER_REDUCED"}],
        ["MII", {layerType:"ENET_MAC_LAYER_MII", variantType:"ENET_MAC_VARIANT_NONE", sublayerType:"ENET_MAC_SUBLAYER_STANDARD"}],
    ],)
    return cpswMiiConfigMap.get(instance.phyToMacInterfaceMode);
}

function validate(instance, report) {
    pktPoolScript.validate(instance, report);
    aleScript.validate(instance, report);
    mdioScript.validate(instance, report);
    macportScript.validate(instance, report);
    hostportScript.validate(instance, report);

    if ((instance.BoardType === "am263px-cc") || (instance.BoardType === "am263px-cc-addon-ind") || (instance.BoardType === "am263px-cc-addon-auto"))
    {
        if (instance.DisableMacPort1 === false)
        {
            report.logError(`Port1 is unavailable on the AM263Px-CC Board`, instance);
        }
    }

    if (getNetifCount(instance) > 0)
    {
        if (getDefaultNetifCount(instance) !=1)
        {
            report.logError(`Only one netif can be set as default`, instance, "netifInstance");
        }
        if(verifyNetifPacketDequeueMode(instance) === false){
            report.logError(`Both the Netif should be in same PacketDeque Mode`, instance,);
        }

        if (getNetifCount(instance) === 2)
        {
            if ((instance.DisableMacPort1 === true) || (instance.DisableMacPort2 === true))
            {
                report.logError("Both MAC ports in MAC PORT Config should be enabled to support two NetIfs", instance);
            }

            if ((instance.macOnlyEn_hostPort === false) || (instance.macOnlyEn_macPort1 === false) || (instance.macOnlyEn_macPort2 === false))
            {
                report.logError("All Ports in 'ALE Config -> ALE Port Config -> MAC-only mode config' should be in MAC-only mode in case of two NetIfs", instance);
            }
        }
    }
    if (/^([0-9a-fA-F]{2}[:-]){5}[0-9a-fA-F]{2}(,([0-9a-fA-F]{2}[:-]){5}[0-9a-fA-F]{2})+/.test(instance.macAddrList) == false)
    {
        report.logError(`Invalid macAddrList Entry`, instance, "macAddrList");
    }
}

function moduleInstances(instance) {

    let Instances  = new Array();
    let driverVer = soc.getDriverVer("enet_cpsw");
    let maxCh     = 8;
    let maxNetif  = 2;

    Instances.push({
        name: "txDmaChannel",
        displayName: "ENET tx dma channel",
        moduleName: `/networking/enet_cpsw/${driverVer}/enet_cpsw_${driverVer}_tx_channel`,
        useArray: true,
        minInstanceCount: 1,
        maxInstanceCount: maxCh,
        defaultInstanceCount: 1,
        collapsed:false,
        group: "cpdmaChConfig",
    });

    Instances.push({
        name: "rxDmaChannel",
        displayName: "ENET rx dma channel",
        moduleName: `/networking/enet_cpsw/${driverVer}/enet_cpsw_${driverVer}_rx_channel`,
        useArray: true,
        minInstanceCount: 1,
        maxInstanceCount: maxCh,
        defaultInstanceCount: 1,
        collapsed:false,
        group: "cpdmaChConfig",
    });

    Instances.push({
        name: "netifInstance",
        displayName: "NETIF instance",
        moduleName: `/networking/enet_cpsw/${driverVer}/enet_cpsw_lwipif_netif`,
        useArray: true,
        minInstanceCount: 0,
        maxInstanceCount: maxNetif,
        defaultInstanceCount: 0,
        collapsed:false,
        group: "lwipIfConfig",
    });

    return (Instances);
}

function addSharedModuleInstances(instance) {
    let Instances = new Array();

    if(instance.DisableMacPort1 === false && instance.customBoardEnable === false){
        Instances.push({
            name: "ethphy1",
            displayName: "Port 1 PHY Configuration",
            moduleName: "/board/ethphy_cpsw_icssg/ethphy_cpsw_icssg",
            requiredArgs: {
                boardType: instance.BoardType,
                peripheral: "CPSW_MAC_PORT_1",
                enableCustomBoard: instance.customBoardEnable,
            },
            group: "macPort1Cfg",
            });
    }
    
    if(instance.DisableMacPort2 == false && instance.customBoardEnable === false){
        Instances.push({
            name: "ethphy2",
            displayName: "Port 2 PHY Configuration",
            moduleName: "/board/ethphy_cpsw_icssg/ethphy_cpsw_icssg",
            requiredArgs: {
                boardType: instance.BoardType,
                peripheral: "CPSW_MAC_PORT_2",
                enableCustomBoard: instance.customBoardEnable,
            },
            group: "macPort2Cfg",
            });
    }

    return Instances;
}

function getCpuInfo() {
	const cpuInfo = new Map(
                               [
                                 ['CSL_CORE_ID_R5FSS0_0',{subsystem: "R5FSS",
                                  clusternum: "0", core: "0"}],
                                 ['CSL_CORE_ID_R5FSS0_1',{subsystem: "R5FSS",
                                  clusternum: "0", core: "1"}],
                                 ['CSL_CORE_ID_R5FSS1_0',{subsystem: "R5FSS",
                                  clusternum: "1", core: "0"}],
                                 ['CSL_CORE_ID_R5FSS1_1', {subsystem: "R5FSS",
                                  clusternum: "1", core: "1"}],
                               ],
                             );
	return cpuInfo.get(getCpuID());
}

function getEnetResPartInfoNumCores() {
    let resPartInfo = getEnetResPartInfo();
    return resPartInfo.numCores;
}

function getEnetCoreResInfoNumRxCh(idx) {
    let resPartInfo = getEnetResPartInfo();
    return resPartInfo.coreResInfo[idx].numRxCh;
}

function getEnetCoreResInfoNumMac(idx) {
    let resPartInfo = getEnetResPartInfo();
    return resPartInfo.coreResInfo[idx].numMacAddress;
}

function getEnetCoreResInfoNumHwPush(idx) {
    let resPartInfo = getEnetResPartInfo();
    return resPartInfo.coreResInfo[idx].numHwPush;
}

function getEnetResPartInfoIsStatTxChAlloc() {
    let resPartInfo = getEnetResPartInfo();
    return resPartInfo.isStaticTxChanAllocated;
}

function getEnetResPartInfo() {
    const ResPartInfoMap = new Map(
                               [
                                 ['am263px',{numCores: 1, coreResInfo: [{txCh: {}, numRxCh: 1, numRxFlows: 1, numMacAddress: 4, numHwPush: 0}], isStaticTxChanAllocated: false}],
                            ],
                             );
    let instInfo =  ResPartInfoMap.get(common.getSocName());
    return instInfo;
}

let enet_cpsw_module_name = "/networking/enet_cpsw/enet_cpsw";

let enet_cpsw_module = {

    displayName: "Enet (CPSW)",
    longDescription: "Driver for Common Port SWitch (CPSW). Support MAC, Switch and used in auto and industrial Ethernet to run TCP/IP, AVB etc. applications. TSN is supported via CPSW",
    templates: {
        "/drivers/pinmux/pinmux_config.c.xdt": {
            moduleName: enet_cpsw_module_name,
        },
        "/drivers/system/system_config.h.xdt": {
            driver_config: "/networking/enet_cpsw/templates/enet_cpsw_v3.h.xdt",
            moduleName: enet_cpsw_module_name,
        },
        "/drivers/system/power_clock_config.c.xdt": {
            moduleName: enet_cpsw_module_name,
        },
        "/board/board/board_config.h.xdt": {
            board_config: getBoardConfigTemplateInfo().Header,
            moduleName: enet_cpsw_module_name,
        },
        "/board/board/board_config.c.xdt": {
            board_config: getBoardConfigTemplateInfo().Cfile,
            moduleName: enet_cpsw_module_name,
        },
        "/networking/common/enet_config.c.xdt": {
            enet_mem_config: "/networking/enet_cpsw/templates/enet_app_memutils_cfg_cpdma.c.xdt",
            enet_syscfg_info: "/networking/enet_cpsw/templates/enet_app_syscfg_info.c.xdt",
            moduleName: enet_cpsw_module_name,
        },
        "/networking/common/enet_config.h.xdt": {
            enet_config: "/networking/enet_cpsw/templates/enet_syscfg.h.xdt",
            moduleName: enet_cpsw_module_name,
        },
        "/networking/common/enet_init.c.xdt": {
            enet_init: "/networking/enet_cpsw/templates/cpsw_init_config.c.xdt",
            moduleName: enet_cpsw_module_name,
        },
        "/networking/common/dma_init.h.xdt": {
            dma_init: "/networking/enet_cpsw/templates/dma_init_config.h.xdt",
            moduleName: enet_cpsw_module_name,
        },
        "/networking/common/dma_init.c.xdt": {
            dma_init: "/networking/enet_cpsw/templates/dma_init_config.c.xdt",
            moduleName: enet_cpsw_module_name,
        },
        "/networking/common/enet_open.c.xdt": {
            enet_open: "/networking/enet_cpsw/templates/enet_init.c.xdt",
            enet_init_config: "/networking/enet_cpsw/templates/enet_app_cpsw_cfg.c.xdt",
            moduleName: enet_cpsw_module_name,
        },
        "/networking/common/enet_open.h.xdt": {
            enet_open: "/networking/enet_cpsw/templates/enet_init.h.xdt",
            moduleName: enet_cpsw_module_name,
        },
        "/networking/common/enet_soc.c.xdt": {
            enet_soc: getSocConfigTemplateInfo().Cfile,
            moduleName: enet_cpsw_module_name,
        },
        "/networking/common/enet_lwipif.c.xdt": {
            enet_lwipif: "/networking/enet_cpsw/templates/enet_lwipif.c.xdt",
            moduleName: enet_cpsw_module_name,
        },
        "/networking/common/enet_lwipif.h.xdt": {
            enet_lwipif: "/networking/enet_cpsw/templates/enet_lwipif.h.xdt",
            moduleName: enet_cpsw_module_name,
        },
    },
    defaultInstanceName: "CONFIG_ENET_CPSW",
    config: [
        {
            name: "BoardType",
            description: "Board selection for AM263Px",
            displayName: "BoardType",
            default: "am263px-cc",
            options: [
                {
                    name: "am263px-cc",
                },
                {
                    name: "am263px-cc-addon-auto",
                },
                {
                    name: "am263px-cc-addon-ind",
                },
                {
                    name: "am263px-lp",
                },
            ],
        },
        enet_cpsw_system_config,
        enet_cpsw_cpdma_channel_config,
        pktPoolScript.config,
        enet_cpsw_lwipIf_config,
        aleScript.config,
        mdioScript.config,
        hostportScript.config,
        macportScript.config,
        cptsScript.config,
        enet_cpsw_board_config,
        pinMuxScript.config,
    ],
    moduleStatic: {
        modules: function(inst) {
            return [{
                name: "system_common",
                moduleName: "/system_common",
            }]
        },
    },
    moduleInstances: moduleInstances,
    sharedModuleInstances: addSharedModuleInstances,
    utils: utilsScript,
    pinmuxRequirements,
    getInterfaceNameList,
    getPeripheralPinNames,
    getClockEnableIds,
    getClockFrequencies,
    getDmaInterface,
    getInstIdTable,
    getCpswInstInfo,
    getBoardConfigTemplateInfo,
    getCpuID,
    getCpuInfo,
    getSocConfigTemplateInfo,
    getTxPacketsCount,
    getRxPacketsCount,
    getRxChannelCount,
    getTxChannelCount,
    getNumCpdmaDesc,
    getChannelConfig,
    getDefaultPacketCount,
    getNetifCount,
    getNetifConfig,
    getNetifPacketDequeueMode,
    getNetifEtherringSupport,
    getDefaultNetifIdx,
    getMiiConfig,
    getEnetResPartInfo,
    getEnetResPartInfoNumCores,
    getEnetCoreResInfoNumRxCh,
    getEnetCoreResInfoNumMac,
    getEnetCoreResInfoNumHwPush,
    getEnetResPartInfoIsStatTxChAlloc,
    validate: validate,
};

exports = enet_cpsw_module;
