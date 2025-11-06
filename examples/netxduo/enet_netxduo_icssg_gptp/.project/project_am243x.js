let path = require('path');

let device = "am243x";

const files = {
    common: [
        "gptp_init.c",
        "default_flow_cfg.c",
        "tsninit.c",
        "debug_log.c",
        "netxduo_gptp_icssg_main.c",
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

const libdirs = {
    common: [
        "generated",
        "${MCU_PLUS_SDK_PATH}/source/networking/netxduo/lib",
        "${MCU_PLUS_SDK_PATH}/source/kernel/threadx/lib",
        "${MCU_PLUS_SDK_PATH}/source/fs/filex/lib",
        "${MCU_PLUS_SDK_PATH}/source/drivers/lib",
        "${MCU_PLUS_SDK_PATH}/source/board/lib",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/lib",
        "${MCU_PLUS_SDK_PATH}/source/networking/tsn/lib",
    ],
};

const includes = {
    common: [
        "${MCU_PLUS_SDK_PATH}/source/board/ethphy/enet/rtos_drivers/include",
        "${MCU_PLUS_SDK_PATH}/source/board/ethphy/port",
        "${MCU_PLUS_SDK_PATH}/source/kernel/threadx/threadx_src/common/inc",
        "${MCU_PLUS_SDK_PATH}/source/kernel/threadx/ports/ti_arm_gcc_clang_cortex_r5/inc",
        "${MCU_PLUS_SDK_PATH}/source/fs/filex/filex_src/common/inc",
        "${MCU_PLUS_SDK_PATH}/source/fs/filex/filex_src/ports/generic/inc",
        "${MCU_PLUS_SDK_PATH}/source/networking/netxduo/netxduo_src/common/inc",
        "${MCU_PLUS_SDK_PATH}/source/networking/netxduo/netxduo_enet",
        "${MCU_PLUS_SDK_PATH}/source/networking/netxduo/netxduo_enet/crypto_hw/sa2ul",
        "${MCU_PLUS_SDK_PATH}/source/networking/netxduo/netxduo_src/ports/cortex_r5/gnu/inc/",
        "${MCU_PLUS_SDK_PATH}/source/networking/netxduo/netxduo_src/nx_secure/inc",
        "${MCU_PLUS_SDK_PATH}/source/networking/netxduo/netxduo_src/nx_secure/ports",
        "${MCU_PLUS_SDK_PATH}/source/networking/netxduo/netxduo_src/crypto_libraries/inc",
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
        "${MCU_PLUS_SDK_PATH}/source/networking/enet",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/core/utils",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/core/utils/include",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/core/utils/V3",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/core",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/core/include",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/core/include/phy",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/core/include/core",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/soc/k3/am64x_am243x",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/hw_include",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/hw_include/mdio/V4",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/core/examples/tsn",
        "${MCU_PLUS_SDK_PATH}/source/networking/tsn/tsn-stack",
        "${MCU_PLUS_SDK_PATH}/source/networking/tsn/tsn-stack/tsn_gptp",
        "${MCU_PLUS_SDK_PATH}/source/networking/tsn/tsn-stack/tsn_gptp/tilld",
        "${MCU_PLUS_SDK_PATH}/source/networking/tsn/tsn-stack/tsn_combase/tilld/sitara",
        "${MCU_PLUS_SDK_PATH}/source/networking/tsn/tsn-stack/tsn_gptp/gptpconf",
        "${MCU_PLUS_SDK_PATH}/source/networking/tsn/tsn-stack/tsn_uniconf",
        "${MCU_PLUS_SDK_PATH}/source/networking/tsn/tsn-stack/tsn_uniconf/yangs",
    ],
};

const libs = {
    common: [
        "drivers.am243x.r5f.ti-arm-clang.${ConfigName}.lib",
        "enet-icssg.am243x.r5f.ti-arm-clang.${ConfigName}.lib",
        "board.am243x.r5f.ti-arm-clang.${ConfigName}.lib",
        "libc.a",
        "libsysbm.a",
        "tsn_icssg_combase-freertos.am243x.r5f.ti-arm-clang.${ConfigName}.lib",
        "tsn_unibase-freertos.am243x.r5f.ti-arm-clang.${ConfigName}.lib",
        "tsn_gptp-freertos.am243x.r5f.ti-arm-clang.${ConfigName}.lib",
        "tsn_uniconf-freertos.am243x.r5f.ti-arm-clang.${ConfigName}.lib",
        "netxduo.am243x.r5f.ti-arm-clang.${ConfigName}.lib",
        "threadx.am243x.r5f.ti-arm-clang.${ConfigName}.lib",
        "filex.am243x.r5f.ti-arm-clang.${ConfigName}.lib",
    ],
};

const linker_includePath = {
    common: [
        "${PROJECT_BUILD_DIR}/syscfg",

    ],
};

const defines = {
    common: [
        "NX_INCLUDE_USER_DEFINE_FILE",
        "ENET_ENABLE_PER_ICSSG=1",
        'PRINT_FORMAT_NO_WARNING',
        'SITARA',
        'GPTP_ENABLED=1',
    ],
};

const cflags = {
    common: [
        "--include tsn_buildconf/sitara_buildconf.h",
    ],
    release: [
        "-Oz",
        "-flto",
    ],
    debug: [
        "-Oz",
        "-flto",
    ],
};

const lflags = {
    common: [
        "--zero_init=on",
        "--use_memset=fast",
        "--use_memcpy=fast"
    ],
};

const loptflags = {
    release: [
        "-mcpu=cortex-r5",
        "-mfloat-abi=hard",
        "-mfpu=vfpv3-d16",
        "-mthumb",
        "-Oz",
        "-flto"
    ],
};

const lnkfiles = {
    common: [
        "../linker.cmd",
    ]
};

const syscfgfile = "../example.syscfg";

const readmeDoxygenPageTag = "EXAMPLES_ECLIPSE_THREADX_NETXDUO_ICSSG_TSN_GPTP";

const templates =
[
    {
        input: ".project/templates/am243x/threadx/main_threadx.c.xdt",
        output: "../main.c",
        options: {
            entryFunction: "netxduo_icssg_main",
        },
    },
];

const buildOptionCombos = [
    { device: device, cpu: "r5fss0-0", cgt: "ti-arm-clang", board: "am243x-evm", os: "threadx"},
    { device: device, cpu: "r5fss0-0", cgt: "ti-arm-clang", board: "am243x-lp", os: "threadx"},
];

function getComponentProperty() {
    let property = {};

    property.dirPath = path.resolve(__dirname, "..");
    property.type = "executable";
    property.name = "enet_netxduo_icssg_netxduo_gptp";
    property.isInternal = false;
    property.buildOptionCombos = buildOptionCombos;

    return property;
}

function getComponentBuildProperty(buildOption) {
    let build_property = {};

    build_property.files = files;
    build_property.filedirs = filedirs;
    build_property.libdirs = libdirs;
    build_property.lnkfiles = lnkfiles;
    build_property.syscfgfile = syscfgfile;
    build_property.readmeDoxygenPageTag = readmeDoxygenPageTag;
    build_property.projecspecFileAction = "link";
    build_property.includes = includes;
    build_property.templates = templates;
    build_property.libs = libs;
    build_property.cflags = cflags;
    build_property.lflags = lflags;
    build_property.loptflags = loptflags;
    build_property.projectspecLnkPath = linker_includePath;
    build_property.defines = defines;

    return build_property;
}

module.exports = {
    getComponentProperty,
    getComponentBuildProperty,
};

