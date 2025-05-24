    
let common = system.getScript("/common");
let netxduo_module = system.modules["/networking/netxduo/netxduo"];


function getInstanceConfig(moduleInstance) {
    return {
        ...moduleInstance,
    };
};

function getDefaultIfCount()
{
    let defaultIfCount = 0;
    for(let i = 0; i < netxduo_module.$instances.length; i++) {
        let instance = netxduo_module.$instances[i];
        for (let Idx = 0; Idx < netxduo_module.getIfCount(instance); Idx++)
        {
            defaultIfCount += (netxduo_module.getIfConfig(instance, Idx).isDefault === true) ? 1 : 0;
        }
    }
    return defaultIfCount;
}


function getTotalIfCount()
{
    let count = 0;
    for(let i = 0; i < netxduo_module.$instances.length; i++) {
        let instance = netxduo_module.$instances[i];
        count += netxduo_module.getIfCount(instance);
    }
    return count;
}

function getDefaultIfIndex()
{
    let dfltIdx = 0;
    for (let i = 0; i < netxduo_module.$instances.length; i++) {
        let instance = netxduo_module.$instances[i];
        for (let Idx = 0; Idx < netxduo_module.getIfCount(instance); Idx++)
        {
            if (netxduo_module.getIfConfig(instance, Idx).isDefault === true)
            {
                dfltIdx = Idx;
            }
        }
    }
    return dfltIdx;
}


function getIfEnetType(instance_name)
{
    let enet_icss_module = system.modules["/networking/enet_icss/enet_icss"];
    let enet_cpsw_module = system.modules["/networking/enet_cpsw/enet_cpsw"];

    if (enet_icss_module) {
        let enet_instance = enet_icss_module.$instances.find(obj => { return obj.$name === instance_name});
        if (enet_instance) {
            return ('ICSSG');
        }
    }
    if (enet_cpsw_module) {
        let enet_instance = enet_cpsw_module.$instances.find(obj => { return obj.$name === instance_name});
        if (enet_instance) {
            return ('CPSW');
        }
    }
    return ('UNKNOWN');
}

function validate(instance, report)
{
    let enet_module;
    let enet_icss_module = system.modules["/networking/enet_icss/enet_icss"];
    let enet_cpsw_module = system.modules["/networking/enet_cpsw/enet_cpsw"];

    if (enet_icss_module) {
        enet_module = enet_icss_module;
    } else {
        enet_module = enet_cpsw_module;
    }

    if (!enet_module) {
         report.logError(`This interface must be tied to a valid Enet driver instance.`, instance);
         return;
    }

    if (instance.mode === "SWITCH")
    {
        if (getDefaultIfCount() > 1)
        {
            report.logError(`Switch case should have only one default netif`, instance);
        }
    }
    if (instance.mode === "DUAL MAC")
    {
        if (getDefaultIfCount() > 1)
        {
            report.logError(`DUAL MAC case should have only one default netif`, instance);
        }
    }
    for(let i = 0; i < netxduo_module.$instances.length; i++)
     {
        let default_found = false;
        let netx_instance = netxduo_module.$instances[i];
        for (let Idx = 0; Idx < netxduo_module.getIfCount(netx_instance); Idx++)
        {
            let ifCfg = netxduo_module.getIfConfig(netx_instance,Idx);
            let rxCh = ifCfg.rxDmaChNum;
            let enet_instance = enet_module.$instances.find(obj => { return obj.$name === ifCfg.enet_instance_name});

            if (ifCfg.isDefault) {
                if (default_found) {
                    report.logError(`Only one default interface is allowed.`, instance);
                }
                default_found = true;
            }

            if (enet_instance === undefined) {
                report.logError(ifCfg.enet_instance_name + " not found.", instance);
                return;
            }

            for (let j = 0; j < rxCh.length; j++)
            {
               if( rxCh[j] >= enet_module.getRxChannelCount(enet_instance) )
               {
                    report.logError(`Incorrect Rx channel`, instance);
               }
            }
            
            let txCh = netxduo_module.getIfConfig(netx_instance,Idx).txDmaChNum;
            for (let j = 0; j < txCh.length; j++)
            {
               if( txCh[j] >= enet_module.getTxChannelCount(enet_instance) )
               {
                    report.logError(`Incorrect Tx channel`, instance);
               }
            }
        }
     }
}


function getIfRxCh()
{
    var ret = '{';
    for(let i = 0; i < netxduo_module.$instances.length; i++)
     {
        let netx_instance = netxduo_module.$instances[i];

        ret += '{';
        for (let Idx = 0; Idx < netxduo_module.getIfCount(netx_instance); Idx++)
        {
            let ifCfg = netxduo_module.getIfConfig(netx_instance,Idx);
            let rxCh = ifCfg.rxDmaChNum;

            let enet_module = null;
            if (getIfEnetType(ifCfg.enet_instance_name) === 'CPSW') {
                enet_module = system.modules["/networking/enet_cpsw/enet_cpsw"];
            }
            if (getIfEnetType(ifCfg.enet_instance_name) === 'ICSSG') {
                enet_module = system.modules["/networking/enet_icss/enet_icss"];
            }

            let enet_instance = enet_module.$instances.find(obj => { return obj.$name === ifCfg.enet_instance_name});

            ret += '{' + enet_module.getChannelConfig(enet_instance, "RX", rxCh[0]).$name.toUpperCase() + ', ';
            if (rxCh.length == 2)
            {
                ret += enet_module.getChannelConfig(enet_instance, "RX", rxCh[1]).$name.toUpperCase() + ',' + '},';
            }
            else
            {
                ret += -1 + ',' + '},'
            }
        }
        ret += '}, ';
     }
    ret += '}';
    return ret;
}


function getIfTxCh()
{
    var ret = '{';
    for(let i = 0; i < netxduo_module.$instances.length; i++)
     {
        let netx_instance = netxduo_module.$instances[i];
        
        ret += '{';
        for (let Idx = 0; Idx < netxduo_module.getIfCount(netx_instance); Idx++)
        {
            let ifCfg = netxduo_module.getIfConfig(netx_instance,Idx);
            let txCh = ifCfg.txDmaChNum

            let enet_module = null;
            if (getIfEnetType(ifCfg.enet_instance_name) === 'CPSW') {
                enet_module = system.modules["/networking/enet_cpsw/enet_cpsw"];
            }
            if (getIfEnetType(ifCfg.enet_instance_name) === 'ICSSG') {
                enet_module = system.modules["/networking/enet_icss/enet_icss"];
            }

            let enet_instance = enet_module.$instances.find(obj => { return obj.$name === ifCfg.enet_instance_name});
            ret += '{' + enet_module.getChannelConfig(enet_instance, "TX", txCh[0]).$name.toUpperCase() + '},';
        }
        ret += '},';
     }
    ret += '}';
    return ret;
}

function uniqueChannels(value, index, array) {
    return (array.indexOf(value) === index);
}

function getAllRxChannels() {

    let ret;
    let chArray = new Array();

    for(let i = 0; i < netxduo_module.$instances.length; i++)
     {
        let netx_instance = netxduo_module.$instances[i];

        for (let Idx = 0; Idx < netxduo_module.getIfCount(netx_instance); Idx++)
        {
            let ifCfg = netxduo_module.getIfConfig(netx_instance,Idx);
            let rxCh = ifCfg.rxDmaChNum;

            let enet_module = null;
            if (getIfEnetType(ifCfg.enet_instance_name) === 'CPSW') {
                enet_module = system.modules["/networking/enet_cpsw/enet_cpsw"];
            }
            if (getIfEnetType(ifCfg.enet_instance_name) === 'ICSSG') {
                enet_module = system.modules["/networking/enet_icss/enet_icss"];
            }

            let enet_instance = enet_module.$instances.find(obj => { return obj.$name === ifCfg.enet_instance_name});

            for (let k = 0; k < rxCh.length; k++) {
                chArray.push(enet_module.getChannelConfig(enet_instance, "RX", rxCh[k]).$name.toUpperCase());
            }
        }
     }

    chArray = chArray.filter(uniqueChannels);
    
    ret = '{';
    for(let i = 0; i < chArray.length; i++) {
        ret += chArray[i];
        if (i < chArray.length - 1) {
            ret += ','   
        }
    }
    ret += '}';

    return ret;
}


function getAllTxChannels() {

    let ret;
    let chArray = new Array();

    for(let i = 0; i < netxduo_module.$instances.length; i++)
     {
        let netx_instance = netxduo_module.$instances[i];

        for (let Idx = 0; Idx < netxduo_module.getIfCount(netx_instance); Idx++)
        {
            let ifCfg = netxduo_module.getIfConfig(netx_instance,Idx);
            let txCh = ifCfg.txDmaChNum;

            let enet_module = null;
            if (getIfEnetType(ifCfg.enet_instance_name) === 'CPSW') {
                enet_module = system.modules["/networking/enet_cpsw/enet_cpsw"];
            }
            if (getIfEnetType(ifCfg.enet_instance_name) === 'ICSSG') {
                enet_module = system.modules["/networking/enet_icss/enet_icss"];
            }

            let enet_instance = enet_module.$instances.find(obj => { return obj.$name === ifCfg.enet_instance_name});

            for (let k = 0; k < txCh.length; k++) {
                chArray.push(enet_module.getChannelConfig(enet_instance, "TX", txCh[k]).$name.toUpperCase());
            }
        }
     }

    chArray = chArray.filter(uniqueChannels);
    
    ret = '{';
    for(let i = 0; i < chArray.length; i++) {
        ret += chArray[i];
        if (i < chArray.length - 1) {
            ret += ','   
        }
    }
    ret += '}';

    return ret;
}


function getIfMacPorts()
{
    let enet_icss_module = system.modules["/networking/enet_icss/enet_icss"];
    let enet_cpsw_module = system.modules["/networking/enet_cpsw/enet_cpsw"];
    let netxduo_interface_module = system.modules["/networking/netxduo/netxduo_interface"];

    var ret = '{';
    for(let i = 0; i < netxduo_module.$instances.length; i++)
     {
        ret += '{';
        let netx_instance = netxduo_module.$instances[i];
        for (let Idx = 0; Idx < netxduo_module.getIfCount(netx_instance); Idx++)
        {
            let ifCfg = netxduo_module.getIfConfig(netx_instance,Idx);
            let txCh = ifCfg.txDmaChNum
            let enet_instance = null;
         
            let enet_module = null;
            if (getIfEnetType(ifCfg.enet_instance_name) === 'CPSW') {
                if (netxduo_interface_module.$instances.length == 2) {
                    ret += Idx === 0 ? 'ENET_MAC_PORT_1' : 'ENET_MAC_PORT_2'; // Mac shared channel.
                } else {
                    ret += 'ENET_MAC_PORT_INV'; // Switch
                }
            }
            if (getIfEnetType(ifCfg.enet_instance_name) === 'ICSSG') {
                enet_module = system.modules["/networking/enet_icss/enet_icss"];
                enet_instance = enet_module.$instances.find(obj => { return obj.$name === ifCfg.enet_instance_name});
                if (enet_instance.dualMacPortSelected) {
                    ret += enet_instance.dualMacPortSelected; // Dual mac.
                } else {
                    ret += 'ENET_MAC_PORT_INV'; // Switch
                }
            }
            ret += ', ';
        }
        ret += '},';
     }
    ret += '}';
    return ret;
}


function getIfIdx2EnetMap()
{
    let enet_module;
    let enet_icss_module = system.modules["/networking/enet_icss/enet_icss"];
    let enet_cpsw_module = system.modules["/networking/enet_cpsw/enet_cpsw"];

    if (enet_icss_module) {
        enet_module = enet_icss_module;
    } else {
        enet_module = enet_cpsw_module;
    }

    var idx = 0;
    var ret = '{';
    for(let i = 0; i < netxduo_module.$instances.length; i++) {
        ret += '{';
        let netx_instance = netxduo_module.$instances[i];
        for (let Idx = 0; Idx < netxduo_module.getIfCount(netx_instance); Idx++)
        {
            let ifCfg = netxduo_module.getIfConfig(netx_instance,Idx);
            let enet_instance = enet_module.$instances.find(obj => { return obj.$name === ifCfg.enet_instance_name});
            let matchedEntry;
            if (enet_icss_module) {
                matchedEntry = enet_module.getInstId(enet_instance);
            } else {
                matchedEntry = enet_module.getCpswInstInfo(enet_instance);
            }
            ret += '{' + matchedEntry.enetType + ', ' +  matchedEntry.instId + '},';
        }
        ret += '}';
    }
    ret += '}'
    return ret;
}


let netxduo_interface_module = {
    displayName: "NetxDuo Interface configuration",
    longDescription: "This adds and configures a NetxDuo Interface.",
    alwaysShowLongDescription: false,
    defaultInstanceName: "CONFIG_NETX_IF",
    config: [
        {
            name: "isDefault",
            description: "Set this netif to default Netif",
            displayName: "Set As Default Netif",
            default: false,
        },
        {
            name: "enet_instance_name",
            description: "Enet driver instance tied to this interface.",
            displayName: "Enet Instance Name",
            default: "CONFIG_ENET_CPSW0",
        },
        {
            name: "rxDmaChNum",
            description: "Rx DMA used by this interface",
            displayName: "Rx DMA Used By This Interface",
            default: Array.from(Array(1).keys()).map(String),
            minSelections: 0,
            options: _.keys(Array(16)).map((index)=>({name: index})),
        },
        {
            name: "txDmaChNum",
            description: "Tx DMA used by this interface",
            displayName: "Tx DMA Used By This Interface",
            default: Array.from(Array(1).keys()).map(String),
            minSelections: 0,
            options: _.keys(Array(16)).map((index)=>({name: index})),
        },
    ],
    getInstanceConfig,
    getDefaultIfIndex,
    getIfIdx2EnetMap,
    getTotalIfCount,
    getIfMacPorts,
    getIfRxCh,
    getIfTxCh,
    getAllRxChannels,
    getAllTxChannels,
    validate: validate,
};


exports = netxduo_interface_module;
