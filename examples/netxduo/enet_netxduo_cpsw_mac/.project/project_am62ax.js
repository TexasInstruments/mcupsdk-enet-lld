let path = require('path');

let device = "am62ax";

const files = {
    common: [
        "netxduo_cpsw.c",
        "enet_cpsw_link.c",
        "main.c",
    ],
};

/* Relative to where the makefile will be generated
 * Typically at <example_folder>/<BOARD>/<core_os_combo>/<compiler>
 */
const filedirs = {
    common: [
        "..",       /* core_os_combo base */
        "../../..", /* Example base */
    ],
};


const libdirs_a53 = {
    common: [
        "${MCU_PLUS_SDK_PATH}/source/networking/netxduo/lib",
        "${MCU_PLUS_SDK_PATH}/source/kernel/threadx/lib",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/lib",
        "${MCU_PLUS_SDK_PATH}/source/drivers/lib",
        "${MCU_PLUS_SDK_PATH}/source/board/lib",
    ],
};

const includes_a53 = {
    common: [
        "${MCU_PLUS_SDK_PATH}/source/board/ethphy/enet/rtos_drivers/include",
        "${MCU_PLUS_SDK_PATH}/source/board/ethphy/port",
        "${MCU_PLUS_SDK_PATH}/source/kernel/threadx/threadx_src/common/inc",
        "${MCU_PLUS_SDK_PATH}/source/kernel/threadx/ports/ti_arm_gcc_clang_cortex_a53/inc",
        "${MCU_PLUS_SDK_PATH}/source/networking/netxduo/netxduo_src/common/inc",
        "${MCU_PLUS_SDK_PATH}/source/networking/netxduo/netxduo_enet",
        "${MCU_PLUS_SDK_PATH}/source/networking/netxduo/netxduo_src/ports/cortex_a5/gnu/inc/",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/utils/include",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/utils/V3",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/core",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/core/include",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/core/include/phy",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/core/include/core",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/core/utils/include",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/soc/k3/am62ax",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/hw_include",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/hw_include/mdio/V4",
        "${MCU_PLUS_SDK_PATH}/source/networking/netxduo/netxduo_src/addons/auto_ip",
        "${MCU_PLUS_SDK_PATH}/source/networking/netxduo/netxduo_src/addons/azure_iot",
        "${MCU_PLUS_SDK_PATH}/source/networking/netxduo/netxduo_src/addons/BSD",
        "${MCU_PLUS_SDK_PATH}/source/networking/netxduo/netxduo_src/addons/cloud",
        "${MCU_PLUS_SDK_PATH}/source/networking/netxduo/netxduo_src/addons/dhcp",
        "${MCU_PLUS_SDK_PATH}/source/networking/netxduo/netxduo_src/addons/dns",
        "${MCU_PLUS_SDK_PATH}/source/networking/netxduo/netxduo_src/addons/ftp",
        "${MCU_PLUS_SDK_PATH}/source/networking/netxduo/netxduo_src/addons/http",
        "${MCU_PLUS_SDK_PATH}/source/networking/netxduo/netxduo_src/addons/mdns",
        "${MCU_PLUS_SDK_PATH}/source/networking/netxduo/netxduo_src/addons/mqtt",
        "${MCU_PLUS_SDK_PATH}/source/networking/netxduo/netxduo_src/addons/nat",
        "${MCU_PLUS_SDK_PATH}/source/networking/netxduo/netxduo_src/addons/pop3",
        "${MCU_PLUS_SDK_PATH}/source/networking/netxduo/netxduo_src/addons/ppp",
        "${MCU_PLUS_SDK_PATH}/source/networking/netxduo/netxduo_src/addons/pppoe",
        "${MCU_PLUS_SDK_PATH}/source/networking/netxduo/netxduo_src/addons/ptp",
        "${MCU_PLUS_SDK_PATH}/source/networking/netxduo/netxduo_src/addons/rtp",
        "${MCU_PLUS_SDK_PATH}/source/networking/netxduo/netxduo_src/addons/rtsp",
        "${MCU_PLUS_SDK_PATH}/source/networking/netxduo/netxduo_src/addons/smtp",
        "${MCU_PLUS_SDK_PATH}/source/networking/netxduo/netxduo_src/addons/snmp",
        "${MCU_PLUS_SDK_PATH}/source/networking/netxduo/netxduo_src/addons/sntp",
        "${MCU_PLUS_SDK_PATH}/source/networking/netxduo/netxduo_src/addons/telnet",
        "${MCU_PLUS_SDK_PATH}/source/networking/netxduo/netxduo_src/addons/tftp",
        "${MCU_PLUS_SDK_PATH}/source/networking/netxduo/netxduo_src/addons/web",
        "${MCU_PLUS_SDK_PATH}/source/networking/netxduo/netxduo_src/addons/websocket",
    ],
};


const libs_a53 = {
    common: [
        "netxduo.am62ax.a53.gcc-aarch64.${ConfigName}.lib",
        "threadx.am62ax.a53.gcc-aarch64.${ConfigName}.lib",
        "enet-cpsw.am62ax.a53.gcc-aarch64.${ConfigName}.lib",
        "drivers.am62ax.a53.gcc-aarch64.${ConfigName}.lib",
        "board.am62ax.a53.gcc-aarch64.${ConfigName}.lib",
    ],
};



const defines_a53 = {
    common: [
        "NX_INCLUDE_USER_DEFINE_FILE",
    ],
};


const cflags_a53 = {
    common: [
        "-Wno-unused-function",
        "-Wno-format",
    ],
    release: [
        "-flto",
    ],
};

const lflags_a53 = {
    common: [
    ],
};


const loptflags_a53 = {
    release: [
        "-flto"
    ],
};

const lnkfiles = {
    common: [
        "linker.cmd",
    ]
};

const syscfgfile = "../example.syscfg";

const readmeDoxygenPageTag = "EXAMPLES_ECLIPSE_THREADX_NETXDUO_CPSW_MAC";

const templates_a53 =
[
    {
        input: ".project/templates/am62ax/common/linker_a53.cmd.xdt",
        output: "linker.cmd",
        options: {
            enableDMARegion: true,
            dmaHeapSize: 0x20000,
        }
    },
    {
        input: ".project/templates/am62ax/threadx/main_threadx.c.xdt",
        output: "../main.c",
        options: {
            entryFunction: "netxduo_cpsw_main",
            stackSize : "8192",
        },
    }
];

const buildOptionCombos = [
    { device: device, cpu: "a53ss0-0", cgt: "gcc-aarch64", board: "am62ax-sk", os: "threadx"},
];

function getComponentProperty() {
    let property = {};

    property.dirPath = path.resolve(__dirname, "..");
    property.type = "executable";
    property.name = "enet_netxduo_cpsw_mac";
    property.tirexResourceSubClass = [ "example.gettingstarted" ];
    property.description = "A simple example for NetxDuo with CPSW in MAC mode."
    property.buildOptionCombos = buildOptionCombos;

    return property;
}

function getComponentBuildProperty(buildOption) {
    let build_property = {};

    build_property.files = files;
    build_property.filedirs = filedirs;
    build_property.lnkfiles = lnkfiles;
    build_property.syscfgfile = syscfgfile;
    build_property.projecspecFileAction = "link";
    build_property.readmeDoxygenPageTag = readmeDoxygenPageTag;

    if(buildOption.cpu.match(/a53*/)) {
        build_property.includes = includes_a53;
        build_property.libdirs = libdirs_a53;
        build_property.libs = libs_a53;
        build_property.templates = templates_a53;
        build_property.defines = defines_a53;
        build_property.cflags = cflags_a53;
        build_property.lflags = lflags_a53;
        build_property.loptflags = loptflags_a53;
    }

    return build_property;
}

module.exports = {
    getComponentProperty,
    getComponentBuildProperty,
};
