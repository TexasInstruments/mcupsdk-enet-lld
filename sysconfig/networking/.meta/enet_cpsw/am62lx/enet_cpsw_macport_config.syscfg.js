"use strict";
const utilsScript = system.getScript("./../../common/enet_cpsw_utils");


function enet_cpsw_macport_validate(inst, report) {

    if ((inst.DisableMacPort1 == true) && (inst.DisableMacPort2 == true))
    {
        report.logError("Atleast one MAC port should be enabled", inst);
    }

    if (inst.macport1LinkSpeed != "ENET_SPEED_AUTO")
    {
        if (inst.macport1LinkDuplexity == "ENET_DUPLEX_AUTO")
        {
            report.logError("MAC port 1 Duplexity should be AUTO if Speed is set so", inst);
        }
    }
    if (inst.macport1LinkDuplexity != "ENET_DUPLEX_AUTO")
    {
        if (inst.macport1LinkSpeed == "ENET_SPEED_AUTO")
        {
            report.logError("MAC port 1 Speed should be AUTO if Duplexity is set so", inst);
        }
    }

    if (inst.macport2LinkSpeed != "ENET_SPEED_AUTO")
    {
        if (inst.macport2LinkDuplexity == "ENET_DUPLEX_AUTO")
        {
            report.logError("MAC port 2 Duplexity should be AUTO if Speed is set so", inst);
        }
    }
    if (inst.macport2LinkDuplexity != "ENET_DUPLEX_AUTO")
    {
        if (inst.macport2LinkSpeed == "ENET_SPEED_AUTO")
        {
            report.logError("MAC port 2 Speed should be AUTO if Duplexity is set so", inst);
        }
    }

    if (inst.macport1LinkSpeed == "ENET_SPEED_1GBIT")
    {
        if (inst.macport1LinkDuplexity == "ENET_DUPLEX_HALF")
        {
            report.logError("Link capabilty cannot support with 1G speed with half duplex", inst);
        }
    }
    if (inst.macport2LinkSpeed == "ENET_SPEED_1GBIT")
    {
        if (inst.macport2LinkDuplexity == "ENET_DUPLEX_HALF")
        {
            report.logError("Link capabilty cannot support with 1G speed with half duplex", inst);
        }
    }

    if (inst.macport1LoopbackMode == "LOOPBACK_MODE_PHY")
    {
        if ((inst.macport1LinkSpeed == "ENET_SPEED_AUTO") || (inst.macport1LinkDuplexity == "ENET_DUPLEX_AUTO"))
        {
            report.logError("PHY LOOP needs fixed speed and duplex setting (AUTO not supported)", inst);
        }
    }
    if (inst.macport2LoopbackMode == "LOOPBACK_MODE_PHY")
    {
        if ((inst.macport2LinkSpeed == "ENET_SPEED_AUTO") || (inst.macport2LinkDuplexity == "ENET_DUPLEX_AUTO"))
        {
            report.logError("PHY LOOP needs fixed speed and duplex setting (AUTO not supported)", inst);
        }
    }
}

const enet_cpsw_macport_config = {
    name: "macPort#Cfg",
    displayName: "MAC Port # Config",
	longDescription: "Configuration of CPSW MAC PORT #",
    config: [
        {
            name: "DisableMacPort#",
            description: "Flag to selectively disable MACport#. For CPSW3G both external mac ports are enabled by default. Application may selectively choose to disable some external ports",
            displayName: "Disable Mac Port #",
            default: false,
            onChange: function(inst, ui) {
                utilsScript.hideGroup(utilsScript.getGroupHierarchyByName(inst.$module.config, "macPortCfg/macPort1Cfg"), false, ui);
                utilsScript.hideGroup(utilsScript.getGroupHierarchyByName(inst.$module.config, "macPortCfg/macPort2Cfg"), false, ui);
                inst.enablexmii1 = true;
                inst.enablexmii2 = true;
                if (inst.ExternalPhyMgmtEnable == true)
                {
                    ui.macport1LinkSpeed.hidden = true;
                    ui.macport1LinkDuplexity.hidden = true;
                    ui.macport2LinkSpeed.hidden = true;
                    ui.macport2LinkDuplexity.hidden = true;
                }
                if (inst.DisableMacPort1 === true)
                {
                    utilsScript.hideGroup(utilsScript.getGroupHierarchyByName(inst.$module.config, "macPortCfg/macPort1Cfg"), true, ui);
                    inst.enablexmii1 = false;
                }
                if (inst.DisableMacPort2 === true)
                {
                    utilsScript.hideGroup(utilsScript.getGroupHierarchyByName(inst.$module.config, "macPortCfg/macPort2Cfg"), true, ui);
                    inst.enablexmii2 = false;
                }
                ui.DisableMacPort1.hidden = false;
                ui.DisableMacPort2.hidden = false;
            },
        },
        {
            name: "macport#LoopbackMode",
            description: "Loop-back operation mode to configure. Set to NONE for normal operation",
            displayName: "Loop-back Mode",
            default: "LOOPBACK_MODE_NONE",
            options: [
                {
                    name: "LOOPBACK_MODE_MAC",
                },
                {
                    name: "LOOPBACK_MODE_PHY",
                },
                {
                    name: "LOOPBACK_MODE_NONE",
                },
            ],
            hidden: false,
        },
        {
            name: "macport#LinkSpeed",
            description: "Link Speed capability configuration to PHY during auto-negotiation. Check with PHY datasheet to see the capabilites. Set to AUTO if not-sure",
            displayName: "Link Speed Capability",
            default: "ENET_SPEED_AUTO",
            options: [
                {
                    name: "ENET_SPEED_AUTO",
                },
                {
                    name: "ENET_SPEED_10MBIT",
                },
                {
                    name: "ENET_SPEED_100MBIT",
                },
                {
                    name: "ENET_SPEED_1GBIT",
                },
            ],
            hidden: false,
        },
        {
            name: "macport#LinkDuplexity",
            description: "Link Duplexity capbility configuration to PHY during auto-negotiation. Check with PHY datasheet to see the capabilites. Set to AUTO if not-sure",
            displayName: "Link Duplexity Capabilty",
            default: "ENET_DUPLEX_AUTO",
            options: [
                {
                    name: "ENET_DUPLEX_AUTO",
                },
                {
                    name: "ENET_DUPLEX_HALF",
                },
                {
                    name: "ENET_DUPLEX_FULL",
                },
            ],
            hidden: false,
        },
        {
            name: "macport#CrcType",
            description: "Type of CRC",
            displayName: "CRC Type",
            default: "ENET_CRC_ETHERNET",
            options: [
                {
                    name: "ENET_CRC_ETHERNET",
                },
                {
                    name: "ENET_CRC_CASTAGNOLI",
                },
            ],
            hidden: false,
        },
        {
            name: "macport#RxMtu",
            description: "Max length of a received frame on ingress. This max length includes VLAN",
            displayName: "Rx MTU",
            default: 1518,
            isInteger: true,
            range: [0, 1522],
            hidden: false,
        },
        {
            name: "macport#PassPriorityTaggedUnchanged",
            description: "Whether priority tagged packets should be passed unchanged (if set to true) or replaced with port's VID (if set to false)",
            displayName: "Un-Change Priority Tagged Packets",
            default: false,
            hidden: false,
        },
        {
            name: "macport#TxPriorityType",
            description: "Egress Priority type: Fixed or Escalate",
            displayName: "Egress Priority Type",
            default: "ENET_EGRESS_PRI_TYPE_FIXED",
            options: [
                {
                    name: "ENET_EGRESS_PRI_TYPE_FIXED",
                },
                {
                    name: "ENET_EGRESS_PRI_TYPE_ESCALATE",
                },
            ],
            hidden: false,
        },
        {
            name: "vlanCfg",
            description: "Port VLAN configuration",
            longDescription: "Port VLAN configuration. Configuration are taken from taken from 'ALE Config' -> 'Port Default Vlan Config')",
            config : [
                {
                    name: "macport#PortVID",
                    description: "Port VLAN ID. '0': frame does not carry a VLAN ID",
                    displayName: "Port VLAN ID",
                    default: 0,
                    readOnly: true,
                    getValue: function(inst) { return inst.vlanId_macPort1 },
                    isInteger: true,
                    range: [0, 4094],
                    displayFormat: "hex",
                    hidden: false,
                },
                {
                    name: "macport#PortPri",
                    description: "Port VLAN priority Value",
                    displayName: "Port VLAN Priority",
                    default: 0,
                    readOnly: true,
                    getValue: function(inst) { return inst.vlanPrio_macPort1 },
                    isInteger: true,
                    range: [0, 7],
                    hidden: false,
                },
                {
                    name: "macport#PortCfi",
                    description: "Port CFI bit",
                    displayName: "Set Port CFI Bit",
                    default: false,
                    readOnly: true,
                    getValue: function(inst) { return inst.vlanCfiBit_macPort1 },
                    hidden: false,
                },
            ],
            collapsed:true,
        },
    ],
    collapsed:true,
};

const enet_cpsw_macport_topConfig = {
    name: "macPortCfg",
    displayName: "MAC Port Config",
    longDescription: "Configuration of CPSW MAC PORTS",
    config: [
        utilsScript.getPortSpecificConfig(enet_cpsw_macport_config, "#", "1"),
        utilsScript.getPortSpecificConfig(enet_cpsw_macport_config, "#", "2"),
    ],
    collapsed: true,
};

exports =
{
    config: enet_cpsw_macport_topConfig,
    validate: enet_cpsw_macport_validate
};
