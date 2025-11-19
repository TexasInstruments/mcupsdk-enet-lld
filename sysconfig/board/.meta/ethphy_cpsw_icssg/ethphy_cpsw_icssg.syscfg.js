
let common = system.getScript("/common");
let device = common.getDeviceName();
let soc = system.getScript(`/board/soc/board_${common.getSocName()}`);

let driverVer = soc.getDriverVer("ethphy");
let ethphy = system.getScript(`/board/ethphy_cpsw_icssg/ethphy_cpsw_icssg_${driverVer}`);

const longDescriptionExtendedConfig =
`This field is optional.
	If Extended configuration is needed for the PHY, follow the format below to add the features: \r\n
		.propertyName1 = propertyValue1; \r\n		.propertyName2 = propertyValue2; \r\n		.propertyName3 = propertyValue3;
		.propertyName4 =
		{
			propertyValue4_1,
			propertyValue4_2,
		},
`;

const ethphy_devices = [
    {
        name: "DP83869",
    },
	{
        name: "DP83867",
    },
	{
        name: "DP83822",
    },
    {
        name: "DP83826",
    },
    {
        name: "DP83TC812",
    },
    {
        name: "DP83TG720",
    },
    {
        name: "DP83TG721",
    },
    {
        name: "CUSTOM",
        description: "Use this option to use a custom ETHPHY device with the name entered in the text box below"
    },
    {
        name: "NO-PHY",
        description: "Use this option to disable ETHPHY driver code generation and enable MAC-to-MAC mode"
    },
];

function getValidInstances(module) {
    let validInstances = [];

    for(let i = 0; i < module.$instances.length; i++) {
        let instance = module.$instances[i];
        if(instance.phySelect != "NO-PHY")
        {
            validInstances.push(i);
        }
    }
    return validInstances;
}

function getLinkedInstances(module) {
    let instances = module.$instances;
    let linkedInstances = [];

    for (let idx of getValidInstances(module)) {
        if(instances[idx].$sharedBy.length !== 0) {
            linkedInstances.push(idx);
        }
    }
    return linkedInstances;
};

function getUniqueLinkedEthphy(module) {
    let instances = module.$instances;
    let uniq = [];

    for(let i of getLinkedInstances(module)) {
        let ethphyDevice = instances[i].phySelect;
        if (ethphyDevice === "CUSTOM") {
            ethphyDevice = instances[i].customDeviceName;
        }
        if(!(uniq.includes(ethphyDevice))) {
            uniq.push(ethphyDevice);
        }
    }
    return uniq;
};

function peripheralSelectionOnChange (inst, ui) {
    ui.phyAddr.hidden = false;
    ui.customDeviceName.hidden = true;

    let peripheral = inst.peripheral;
    if(inst.boardType != "NONE") {
        peripheral = inst.boardType;
    }
    if((inst.peripheral).includes("PORT_1")) {
        inst.phySelect = ethphy.getPhyInfo(peripheral).defaultPhy1;
        inst.phyAddr = ethphy.getPhyInfo(peripheral).defaultPhyAddr1;
    }
    else {
        inst.phySelect = ethphy.getPhyInfo(peripheral).defaultPhy2;
        inst.phyAddr = ethphy.getPhyInfo(peripheral).defaultPhyAddr2;
    }

    inst.extendedConfig = ethphy.getExternalConfig(inst.phySelect, inst.peripheral);
}

function customBoardSelectionOnChange (inst, ui) {
    let hideConfig = false;
    if(inst.enableCustomBoard === true)	{
        hideConfig = true;
    }

    ui.phySelect.hidden = hideConfig;
    ui.phyAddr.hidden = hideConfig;
    ui.isStrappedPhy.hidden = hideConfig;
    ui.skipExtendedConfig.hidden = hideConfig;
    ui.extendedConfig.hidden = hideConfig;
    ui.peripheral.hidden = hideConfig;
}

function checkInstanceLinkage(inst, report) {
    if(inst.$sharedBy.length === 0) {
        report.logWarning(`Warning : Ethphy device is not linked to a network peripheral`, inst);
    }
}

let ethphy_module_name = "/board/ethphy_cpsw_icssg/ethphy_cpsw_icssg";

let ethphy_module = {
    displayName: "ETHPHY (Enet CPSW/ICSS)",
	longDescription: "ETHPHY module for Enet (CPSW) and Enet (ICSS) drivers",
	alwaysShowLongDescription: true,
    defaultInstanceName: "CONFIG_ENET_ETHPHY",
	config: [
        {
            name: "peripheral",
            description: "Selected network peripheral",
            displayName: "Peripheral",
            default: "NONE",
            hidden: true,
			onChange: peripheralSelectionOnChange,
		},
        {
            name: "boardType",
            description: "Board Type",
            displayName: "Board Type",
            default: "NONE",
            hidden: true,
            onChange: peripheralSelectionOnChange,
		},
		{
            name: "enableCustomBoard",
            description: "Enable Custom Board Configuration",
            displayName: "Custom Board",
            longDescription: "Configuration for custom board that are not supported out of box in MCU+ SDK",
            default: false,
            hidden: true,
            onChange:customBoardSelectionOnChange,
        },
        {
            name: "phySelect",
            description: "Select the required PHY",
            displayName: "ETHPHY Device",
            default: "DP83869",
            options: ethphy_devices,
            onChange: function(inst, ui) {
                if(inst.phySelect == "NO-PHY"){
                    ui.customDeviceName.hidden = true;
                    inst.phyAddr = (~0);
                    ui.phyAddr.hidden = true;
                }
                else {
                    let defaultAddr = 0;
                    if (inst.peripheral != "NONE"){
                        let peripheral = (inst.boardType == "NONE")? inst.peripheral:inst.boardType;
                        defaultAddr = (inst.peripheral).includes("PORT_1")?ethphy.getPhyInfo(peripheral).defaultPhyAddr1:ethphy.getPhyInfo(peripheral).defaultPhyAddr2;
                    }

                    ui.customDeviceName.hidden = (inst.phySelect == "CUSTOM")? false:true;
                    inst.phyAddr = defaultAddr;
                    ui.phyAddr.hidden = false;
                }

                if(inst.skipExtendedConfig == false){
                    inst.extendedConfig = ethphy.getExternalConfig(inst.phySelect, inst.peripheral);
                }
            }
        },
        {
            name: "customDeviceName",
            displayName: "Custom Device Name",
            default: "myethphy",
            description: "Name of the custom ETHPHY device (in lowercase)",
            hidden: true,
        },
        {
            name: "phyAddr",
            description: "Address of the PHY. Value MUST be between 0 .. 31",
            displayName: "Phy Address",
            default: 0,
            displayFormat: "dec",
            isInteger: true,
            range: [-1, 31],
        },
        {
            name: "isStrappedPhy",
            longDescription: "Set if PHY SOC has auto-negotiation disabled strap setting (Strapped mode).",
            displayName: "Strapped Mode",
            default: false,
        },
        {
            name: "skipExtendedConfig",
            displayName: "Skip Extended Configuration",
            description: "Set if extended configuation is needed for the PHY" ,
            default:false,
            onChange:function (inst, ui) {
                if(inst.skipExtendedConfig == true) {
                    ui.extendedConfig.hidden = true;
                }
                else {
                    ui.extendedConfig.hidden = false;
                }
            },
        },
        {
            name: "extendedConfig",
            longDescription: longDescriptionExtendedConfig,
            displayName: "Extended Configuration",
            multiline: true,
            default: "",
        },
	],
    maxInstances: ethphy.getMaxInstanceCount(),
    validate: validate,
    moduleStatic: {
        modules: function(inst) {
            return [{
                name: "system_common",
                moduleName: "/system_common",
            }]
        },
    },
    getValidInstances,
    getLinkedInstances,
    getUniqueLinkedEthphy,
};

function checkConfigFormat(instance, report, property){
    /*TODO: Add proper check for extended configuration input format*/
	const pattern = /\s*/;///^(\s*\.\w+\s*=\s*[^,]*,\s*)+$/;
	const inputConfig = instance[property];

	const lines = inputConfig => inputConfig.trim().split(/\r?\n/);

	for (var i = 0; i < lines.length; i++)
	{
		// lines[i] = lines[i].trim();
		if (pattern.test(lines[i]) == false)
		{
			report.logError("Invalid format. Make sure each entry starts with '.' and ends with ','", instance, property);
		}
	}
}

function validate(inst, report) {

    if(inst.phySelect == "CUSTOM") {
        report.logInfo(`Name of the custom ETHPHY device (in lowercase).
        Follow the below guidelines for naming the required file:  \r\n
        Header file: Name - ${inst.customDeviceName}.h
        Source file: Name - ${inst.customDeviceName}.c
        Private header file (if needed): Name - ${inst.customDeviceName}_priv.h`, inst, "customDeviceName");
    }

    if(inst.customDeviceName != inst.customDeviceName.toLowerCase()){
        report.logError(`Device name should be in lowercase`, inst, "customDeviceName");
    }

    common.validate.checkValidCName(inst, report, "customDeviceName");

    checkConfigFormat (inst, report, "extendedConfig");
    checkInstanceLinkage(inst, report);
}

exports = ethphy_module;
