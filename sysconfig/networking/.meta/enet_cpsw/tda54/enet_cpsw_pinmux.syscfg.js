
let common = system.getScript("/common");
let pinmux = system.getScript("/drivers/pinmux/pinmux");
let device = common.getDeviceName();
let soc = system.getScript(`/networking/soc/networking_${common.getSocName()}`);

function getInstanceConfig(moduleInstance) {
    return {
        ...moduleInstance,
    };
};

function getPeripheralRequirements(inst, peripheralName, name)
{
    let interfaceName = getInterfaceName(inst, peripheralName);
    let resources = [];
    let pinResource = {};

    if(peripheralName == "RGMII")
    {
        pinResource = pinmux.getPinRequirements(interfaceName, "RD0", "RD0");
        pinmux.setConfigurableDefault( pinResource, "rx", true );
        resources.push( pinResource);
        pinResource = pinmux.getPinRequirements(interfaceName, "RD1", "RD1");
        pinmux.setConfigurableDefault( pinResource, "rx", true );
        resources.push( pinResource);
        pinResource = pinmux.getPinRequirements(interfaceName, "RD2", "RD2");
        pinmux.setConfigurableDefault( pinResource, "rx", true );
        resources.push( pinResource);
        pinResource = pinmux.getPinRequirements(interfaceName, "RD3", "RD3");
        pinmux.setConfigurableDefault( pinResource, "rx", true );
        resources.push( pinResource);
        pinResource = pinmux.getPinRequirements(interfaceName, "RX_CTL", "RX_CTL");
        pinmux.setConfigurableDefault( pinResource, "rx", true );
        resources.push( pinResource);
        pinResource = pinmux.getPinRequirements(interfaceName, "RXC", "RX_RXC");
        pinmux.setConfigurableDefault( pinResource, "rx", true );
        resources.push( pinResource);
        pinResource = pinmux.getPinRequirements(interfaceName, "TD0", "TD0");
        pinmux.setConfigurableDefault( pinResource, "rx", true );
        resources.push( pinResource);
        pinResource = pinmux.getPinRequirements(interfaceName, "TD1", "TD1");
        pinmux.setConfigurableDefault( pinResource, "rx", true );
        resources.push( pinResource);
        pinResource = pinmux.getPinRequirements(interfaceName, "TD2", "TD2");
        pinmux.setConfigurableDefault( pinResource, "rx", true );
        resources.push( pinResource);
        pinResource = pinmux.getPinRequirements(interfaceName, "TD3", "TD3");
        pinmux.setConfigurableDefault( pinResource, "rx", true );
        resources.push( pinResource);
        pinResource = pinmux.getPinRequirements(interfaceName, "TX_CTL", "TX_CTL");
        pinmux.setConfigurableDefault( pinResource, "rx", true );
        resources.push( pinResource);
        pinResource = pinmux.getPinRequirements(interfaceName, "TXC", "TX_RXC");
        pinmux.setConfigurableDefault( pinResource, "rx", true );
        resources.push( pinResource);
    }
    else if (name == "CPSW_CPTS")
    {
        pinResource = pinmux.getPinRequirements(interfaceName, "TS_SYNC", "TS_SYNC");
        pinmux.setConfigurableDefault( pinResource, "rx", false );
        resources.push( pinResource);
    }
    else
    {
        let pinList = getInterfacePinList(inst, interfaceName);
        for(let pin of pinList)
        {
            pinResource = pinmux.getPinRequirements(interfaceName, pin);

            /* make all pins as "rx" and then override to make "rx" as false as needed  */
            pinmux.setConfigurableDefault( pinResource, "rx", true );

            resources.push( pinResource );
        }
    }

    let peripheralRequirements = {
        name: name,
        displayName: name,
        interfaceName: interfaceName,
        resources: resources,
    };

    return peripheralRequirements;
}

function getInterfaceName(inst, peripheralName)
{
    return `${peripheralName}`;
}

function getInterfacePinList(inst, peripheralName)
{
    let interfaceName = getInterfaceName(inst, peripheralName);
    let pinList = [];

    pinList = pinmux.getInterfacePinList(interfaceName);

    return pinList;
}


function pinmuxRequirements(inst) {

    let perRequirements = [];

    if (inst.enableTsOut === true)
    {
        let cptsTsSync = getPeripheralRequirements(inst, "CPSW3G", "CPSW_CPTS");
        pinmux.setPeripheralPinConfigurableDefault( cptsTsSync, "CPSW3G", "rx", false);
        perRequirements.push(cptsTsSync);
    }

    let mdio = getPeripheralRequirements(inst, "MDIO", "MDIO");

    /* set default values for "rx" for different pins, based on use case */
    pinmux.setPeripheralPinConfigurableDefault( mdio, "MDC", "rx", false);
    perRequirements.push(mdio);

    if( inst.phyToMacInterfaceMode === "MII")
    {
        let mii1 = getPeripheralRequirements(inst, "MII", "MII1");
        let mii2 = getPeripheralRequirements(inst, "MII", "MII2");

        return [mdio, mii1, mii2];
    }
    else if( inst.phyToMacInterfaceMode === "RMII")
    {
        let rmii1 = getPeripheralRequirements(inst, "RMII", "RMII1");
        let rmii2 = getPeripheralRequirements(inst, "RMII", "RMII2");

        pinmux.setPeripheralPinConfigurableDefault( rmii1, "TXD0", "rx", false);
        pinmux.setPeripheralPinConfigurableDefault( rmii1, "TXD1", "rx", false);
        pinmux.setPeripheralPinConfigurableDefault( rmii1, "TX_EN", "rx", false);
        pinmux.setPeripheralPinConfigurableDefault( rmii2, "TXD0", "rx", false);
        pinmux.setPeripheralPinConfigurableDefault( rmii2, "TXD1", "rx", false);
        pinmux.setPeripheralPinConfigurableDefault( rmii2, "TX_EN", "rx", false);
        perRequirements.push(rmii1);
        perRequirements.push(rmii2);
    }
    else
    {
        let rgmii1 = getPeripheralRequirements(inst, "RGMII", "RGMII1");
        let rgmii2 = getPeripheralRequirements(inst, "RGMII", "RGMII2");

        perRequirements.push(rgmii1);
        perRequirements.push(rgmii2);
    }
        return perRequirements;
}
function getInterfaceNameList(inst) {
    let interfaceNameList = []
    interfaceNameList.push(getInterfaceName(inst, "MDIO"))

    if (inst.enableTsOut === true)
    {
        interfaceNameList.push("CPSW_CPTS")
    }
    if (inst.phyToMacInterfaceMode === "MII")
    {
        interfaceNameList.push(getInterfaceName(inst, "MII1"));
        interfaceNameList.push(getInterfaceName(inst, "MII2"));
    }
    else if (inst.phyToMacInterfaceMode === "RMII")
    {
        interfaceNameList.push(getInterfaceName(inst, "RMII2"));
        interfaceNameList.push(getInterfaceName(inst, "RMII2"));
    }
    else
    {
        interfaceNameList.push(getInterfaceName(inst, "RGMII1"));
        interfaceNameList.push(getInterfaceName(inst, "RGMII2"));
    }
    return interfaceNameList;
}

function getPeripheralPinNames(inst)
{
    let pinList = [];
    if (inst.enableTsOut === true)
    {
      pinList = pinList.concat( "CPTS0_TS_SYNC");
    }

    if(inst.phyToMacInterfaceMode === "MII")
    {
        pinList = pinList.concat( getInterfacePinList(inst, "MDIO"),
                        getInterfacePinList(inst, "MII" )
        );
    }
    else if(inst.phyToMacInterfaceMode === "RMII")
    {
        pinList = pinList.concat(getInterfacePinList(inst, "MDIO"),
                        getInterfacePinList(inst, "RMII" ));
    }
    else
    {
        pinList = pinList.concat( getInterfacePinList(inst, "MDIO"),
                        getInterfacePinList(inst, "RGMII" ));
    }
    return pinList;
}

let enet_cpsw_pinmux_module = {
    displayName: "CPSW pinmux config",
    longDescription: `This configures CPSW module pinmux`,
    config: [
        {
            name: "phyToMacInterfaceMode",
            displayName: "RMII/RGMII",
            default: "RGMII",
            options: [
                {
                    name: "RMII",
                },
                {
                    name: "RGMII",
                },
            ],
        },
        {
            name: "enableTsOut",
            displayName: "Enable CPTS TS Output",
            default: false,
            hidden: false,
        },
        {
            name: "enablexmii1",
            description: "Enable port 1 specific RGMII/RMII/MII.Change 'Disable MAC Port' status in 'MAC Port Config' to change this",
            displayName: "Enable Port1 R/G/MII For CPSW",
            default: true,
            hidden: false,
            readOnly: true,
        },
        {
            name: "enablexmii2",
            description: "Enable port 2 specific RGMII/RMII/MII.Change 'Disable MAC Port' status in 'MAC Port Config' to change this",
            displayName: "Enable Port1 R/G/MII For CPSW",
            default: true,
            hidden: false,
            readOnly: true,
        },
    ],
    collapsed: false,
};

exports = {
    config: enet_cpsw_pinmux_module,
    pinmuxRequirements,
    getInterfaceNameList,
    getPeripheralPinNames,
};
