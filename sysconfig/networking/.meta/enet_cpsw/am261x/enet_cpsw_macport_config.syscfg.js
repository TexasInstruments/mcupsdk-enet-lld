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
        if (inst.macport1LinkDuplexity == "ENET_SPEED_HALF")
        {
            report.logError("Link capabilty cannot support with 1G speed with half duplex", inst);
        }
    }
    if (inst.macport2LinkSpeed == "ENET_SPEED_1GBIT")
    {
        if (inst.macport2LinkDuplexity == "ENET_SPEED_HALF")
        {
            report.logError("Link capabilty cannot support with 1G speed with half duplex", inst);
        }
    }
    if(inst.macport2LinkSpeed == "ENET_SPEED_1GBIT")
    {
        if(inst.phyToMacInterfaceMode == "RMII")
        {
            report.logError("RMII Interface can not support with 1Gbps", inst);
        }
    }
    if(inst.macport1LinkSpeed == "ENET_SPEED_1GBIT")
    {
        if(inst.phyToMacInterfaceMode == "RMII")
        {
            report.logError("RMII Interface can not support with 1Gbps", inst);
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
                }
                if (inst.DisableMacPort2 === true)
                {
                    utilsScript.hideGroup(utilsScript.getGroupHierarchyByName(inst.$module.config, "macPortCfg/macPort2Cfg"), true, ui);
                }
                if (inst.DisableMacPort1 === false && inst.macport1EnableIET == false)
                {
                    ui.macport1IetMinFrag.hidden = true;
                    ui.macport1IetMacVerifyEnable.hidden = true;
                    ui.macport1IetQueue0.hidden = true;
                    ui.macport1IetQueue1.hidden = true;
                    ui.macport1IetQueue2.hidden = true;
                    ui.macport1IetQueue3.hidden = true;
                    ui.macport1IetQueue4.hidden = true;
                    ui.macport1IetQueue5.hidden = true;
                    ui.macport1IetQueue6.hidden = true;
                    ui.macport1IetQueue7.hidden = true;
                }
                if (inst.DisableMacPort2 === false && inst.macport2EnableIET == false)
                {
                    ui.macport2IetMinFrag.hidden = true;
                    ui.macport2IetMacVerifyEnable.hidden = true;
                    ui.macport2IetQueue0.hidden = true;
                    ui.macport2IetQueue1.hidden = true;
                    ui.macport2IetQueue2.hidden = true;
                    ui.macport2IetQueue3.hidden = true;
                    ui.macport2IetQueue4.hidden = true;
                    ui.macport2IetQueue5.hidden = true;
                    ui.macport2IetQueue6.hidden = true;
                    ui.macport2IetQueue7.hidden = true;
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
            onChange: function(inst, ui) {
                
            },
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
            name: "enableRgmiiIntDelay#",
            description: "Set for Enabling RGMII port internal delay mode",
            displayName: "Disable RGMII Internal Delay",
            default: false,
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
                    getValue: function(inst) {
                        const portNumber = this.name.match(/\d+/)[0];
                        return inst["vlanId_macPort" + portNumber];
                    },
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
                    getValue: function(inst) {
                        const portNumber = this.name.match(/\d+/)[0];
                        return inst["vlanPrio_macPort" + portNumber];
                    },
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
                    getValue: function(inst) {
                        const portNumber = this.name.match(/\d+/)[0];
                        return inst["vlanCfiBit_macPort" + portNumber]
                    },
                    hidden: false,
                },
            ],
            collapsed:true,
        },
        {
            name: "ietEnable",
            description: "IET configuration",
            longDescription: "IET Frame Pre-emption configuration.",
            config : [
                {
                    name: "macport#EnableIET",
                    description: "Enable IET",
                    displayName: "Enable IET",
                    default: false,
                    onChange: function(inst, ui) {
                        if(inst.macport1EnableIET == true && inst.DisableMacPort1 == false)
                        {
                            ui.macport1IetMinFrag.hidden = false;
                            ui.macport1IetMacVerifyEnable.hidden = false;
                            ui.macport1IetQueue0.hidden = false;
                            ui.macport1IetQueue1.hidden = false;
                            ui.macport1IetQueue2.hidden = false;
                            ui.macport1IetQueue3.hidden = false;
                            ui.macport1IetQueue4.hidden = false;
                            ui.macport1IetQueue5.hidden = false;
                            ui.macport1IetQueue6.hidden = false;
                            ui.macport1IetQueue7.hidden = false;
                        }
                        else if(inst.macport1EnableIET == false || (inst.macport1EnableIET == true && inst.DisableMacPort1 == true))
                        {
                            ui.macport1IetMinFrag.hidden = true;
                            ui.macport1IetMacVerifyEnable.hidden = true;
                            ui.macport1IetQueue0.hidden = true;
                            ui.macport1IetQueue1.hidden = true;
                            ui.macport1IetQueue2.hidden = true;
                            ui.macport1IetQueue3.hidden = true;
                            ui.macport1IetQueue4.hidden = true;
                            ui.macport1IetQueue5.hidden = true;
                            ui.macport1IetQueue6.hidden = true;
                            ui.macport1IetQueue7.hidden = true;
                        }
                        if(inst.macport2EnableIET == true && inst.DisableMacPort2 == false)
                        {
                            ui.macport2IetMinFrag.hidden = false;
                            ui.macport2IetMacVerifyEnable.hidden = false;
                            ui.macport2IetQueue0.hidden = false;
                            ui.macport2IetQueue1.hidden = false;
                            ui.macport2IetQueue2.hidden = false;
                            ui.macport2IetQueue3.hidden = false;
                            ui.macport2IetQueue4.hidden = false;
                            ui.macport2IetQueue5.hidden = false;
                            ui.macport2IetQueue6.hidden = false;
                            ui.macport2IetQueue7.hidden = false;
                        }
                        else if(inst.macport2EnableIET == false || (inst.macport2EnableIET == true && inst.DisableMacPort2 == true))
                        {
                            ui.macport2IetMinFrag.hidden = true;
                            ui.macport2IetMacVerifyEnable.hidden = true;
                            ui.macport2IetQueue0.hidden = true;
                            ui.macport2IetQueue1.hidden = true;
                            ui.macport2IetQueue2.hidden = true;
                            ui.macport2IetQueue3.hidden = true;
                            ui.macport2IetQueue4.hidden = true;
                            ui.macport2IetQueue5.hidden = true;
                            ui.macport2IetQueue6.hidden = true;
                            ui.macport2IetQueue7.hidden = true;
                        }
                    },
                },
                {
                    name: "macport#IetMinFrag",
                    description: "Minimum fragment for IET frame Pre-emption",
                    longDescription: "Minimum fragment size in bytes for IET frame Pre-emption in multiples of 64: 0=64; 1=128; 2=192 and so on",
                    displayName: "IET Minimum Fragments",
                    default: 1,
                    range: [0, 7],
                    isInteger: true,
                    hidden: true,
                },
                {
                    name: "macport#IetMacVerifyEnable",
                    description: "IET Enable Macport verification",
                    displayName: "Enable IET Macport Verification",
                    default: false,
                    hidden: true,
                },
                {
                    name: "macport#IetQueueMode",
                    displayName: "IET Queue Fragments",
                    config: [
                        {
                            name: "macport#IetQueue0",
                            displayName: "IET Queue Mode 0",
                            longDescription: "Traffic configuration for Queue 0",
                            default: "ENET_MAC_QUEUE_PREEMPT_MODE_PREEMPT",
                            hidden: true,
                            options: [
                                {
                                    name: "ENET_MAC_QUEUE_PREEMPT_MODE_PREEMPT",
                                },
                                {
                                    name: "ENET_MAC_QUEUE_PREEMPT_MODE_EXPRESS",
                                },
                            ]
                        },
                        {
                            name: "macport#IetQueue1",
                            displayName: "IET Queue Mode 1",
                            longDescription: "Traffic configuration for Queue 1",
                            default: "ENET_MAC_QUEUE_PREEMPT_MODE_PREEMPT",
                            hidden: true,
                            options: [
                                {
                                    name: "ENET_MAC_QUEUE_PREEMPT_MODE_PREEMPT",
                                },
                                {
                                    name: "ENET_MAC_QUEUE_PREEMPT_MODE_EXPRESS",
                                },
                            ]
                        },
                        {
                            name: "macport#IetQueue2",
                            displayName: "IET Queue Mode 2",
                            longDescription: "Traffic configuration for Queue 2",
                            default: "ENET_MAC_QUEUE_PREEMPT_MODE_PREEMPT",
                            hidden: true,
                            options: [
                                {
                                    name: "ENET_MAC_QUEUE_PREEMPT_MODE_PREEMPT",
                                },
                                {
                                    name: "ENET_MAC_QUEUE_PREEMPT_MODE_EXPRESS",
                                },
                            ]
                        },
                        {
                            name: "macport#IetQueue3",
                            displayName: "IET Queue Mode 3",
                            longDescription: "Traffic configuration for Queue 3",
                            default: "ENET_MAC_QUEUE_PREEMPT_MODE_PREEMPT",
                            hidden: true,
                            options: [
                                {
                                    name: "ENET_MAC_QUEUE_PREEMPT_MODE_PREEMPT",
                                },
                                {
                                    name: "ENET_MAC_QUEUE_PREEMPT_MODE_EXPRESS",
                                },
                            ]
                        },
                        {
                            name: "macport#IetQueue4",
                            displayName: "IET Queue Mode 4",
                            longDescription: "Traffic configuration for Queue 4",
                            default: "ENET_MAC_QUEUE_PREEMPT_MODE_PREEMPT",
                            hidden: true,
                            options: [
                                {
                                    name: "ENET_MAC_QUEUE_PREEMPT_MODE_PREEMPT",
                                },
                                {
                                    name: "ENET_MAC_QUEUE_PREEMPT_MODE_EXPRESS",
                                },
                            ]
                        },
                        {
                            name: "macport#IetQueue5",
                            displayName: "IET Queue Mode 5",
                            longDescription: "Traffic configuration for Queue 5",
                            default: "ENET_MAC_QUEUE_PREEMPT_MODE_PREEMPT",
                            hidden: true,
                            options: [
                                {
                                    name: "ENET_MAC_QUEUE_PREEMPT_MODE_PREEMPT",
                                },
                                {
                                    name: "ENET_MAC_QUEUE_PREEMPT_MODE_EXPRESS",
                                },
                            ]
                        },
                        {
                            name: "macport#IetQueue6",
                            displayName: "IET Queue Mode 6",
                            longDescription: "Traffic configuration for Queue 6",
                            default: "ENET_MAC_QUEUE_PREEMPT_MODE_PREEMPT",
                            hidden: true,
                            options: [
                                {
                                    name: "ENET_MAC_QUEUE_PREEMPT_MODE_PREEMPT",
                                },
                                {
                                    name: "ENET_MAC_QUEUE_PREEMPT_MODE_EXPRESS",
                                },
                            ]
                        },
                        {
                            name: "macport#IetQueue7",
                            displayName: "IET Queue Mode 7",
                            longDescription: "Traffic configuration for Queue 7",
                            default: "ENET_MAC_QUEUE_PREEMPT_MODE_PREEMPT",
                            hidden: true,
                            options: [
                                {
                                    name: "ENET_MAC_QUEUE_PREEMPT_MODE_PREEMPT",
                                },
                                {
                                    name: "ENET_MAC_QUEUE_PREEMPT_MODE_EXPRESS",
                                },
                            ]
                        },
                        
                        
                    ]
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
