let path = require('path');


const files = {
    common: [
        "gptp_init.c",
        "default_flow_cfg.c",
        "tsninit.c",
        "debug_log.c",
        "netxduo_cpsw_main.c",
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
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/lib",
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
        "${MCU_PLUS_SDK_PATH}/source/kernel/threadx/ports/ti_arm_gcc_clang_cortex_a53/inc",
        "${MCU_PLUS_SDK_PATH}/source/fs/filex/filex_src/common/inc",
        "${MCU_PLUS_SDK_PATH}/source/fs/filex/filex_src/ports/generic/inc",
        "${MCU_PLUS_SDK_PATH}/source/networking/netxduo/netxduo_src/common/inc",
        "${MCU_PLUS_SDK_PATH}/source/networking/netxduo/netxduo_enet",
        "${MCU_PLUS_SDK_PATH}/source/networking/netxduo/netxduo_src/ports/cortex_a5/gnu/inc/",
        "${MCU_PLUS_SDK_PATH}/source/networking/netxduo/netxduo_src/nx_secure/inc",
        "${MCU_PLUS_SDK_PATH}/source/networking/netxduo/netxduo_src/nx_secure/ports",
        "${MCU_PLUS_SDK_PATH}/source/networking/netxduo/netxduo_src/crypto_libraries/inc",
        "${MCU_PLUS_SDK_PATH}/source/networking/netxduo/netxduo_src/addons/dhcp",
        "${MCU_PLUS_SDK_PATH}/source/networking/netxduo/netxduo_src/addons/http",
        "${MCU_PLUS_SDK_PATH}/source/networking/netxduo/netxduo_src/addons/web",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/core/utils",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/core/utils/include",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/core/utils/V3",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/core",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/core/include",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/core/include/phy",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/core/include/core",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/soc/k3/am62ax",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/hw_include",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/hw_include/mdio/V4",
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
        "drivers.am62ax.a53.gcc-aarch64.${ConfigName}.lib",
        "enet-cpsw.am62ax.a53.gcc-aarch64.${ConfigName}.lib",
        "board.am62ax.a53.gcc-aarch64.${ConfigName}.lib",
        "tsn_combase-freertos.am62ax.a53.gcc-aarch64.${ConfigName}.lib",
        "tsn_unibase-freertos.am62ax.a53.gcc-aarch64.${ConfigName}.lib",
        "tsn_gptp-freertos.am62ax.a53.gcc-aarch64.${ConfigName}.lib",
        "tsn_uniconf-freertos.am62ax.a53.gcc-aarch64.${ConfigName}.lib",
        "netxduo.am62ax.a53.gcc-aarch64.${ConfigName}.lib",
        "threadx.am62ax.a53.gcc-aarch64.${ConfigName}.lib",
        "filex.am62ax.a53.gcc-aarch64.${ConfigName}.lib",
    ],
};


const defines = {
    common: [
        "NX_INCLUDE_USER_DEFINE_FILE",
        "ENET_ENABLE_PER_CPSW=1",
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
        "-Wno-error"
    ],
};

const cflags_a53 = {
    common: [
        "--include tsn_buildconf/sitara_buildconf.h",
    ],
    release: [
        "-Os",
        "-flto",
    ],
    debug: [
        "-Wno-error"
    ],
};

const lflags = {
    common: [
        "-mno-fix-cortex-a53-835769",
        "-mno-fix-cortex-a53-843419",
    ],
};

const loptflags = {
    release: [
        "-flto"
    ],
};

const lnkfiles = {
    common: [
        "./linker.cmd",
    ]
};

const syscfgfile = "../example.syscfg";

const readmeDoxygenPageTag = "EXAMPLES_ECLIPSE_THREADX_NETXDUO_CPSW_TSN_GPTP";

const templates =
[
    {
        input: ".project/templates/am62ax/common/linker_a53.cmd.xdt",
        output: "linker.cmd",
        options: {
            enableDMARegion: true,
            dmaHeapSize: 0x40000,
        }
    },
    {
        input: ".project/templates/am62ax/threadx/main_threadx.c.xdt",
        output: "../main.c",
        options: {
            entryFunction: "netxduo_cpsw_main",
        },
    },
];

const buildOptionCombos = [
    { device: "am62ax", cpu: "a53ss0-0", cgt: "gcc-aarch64", board: "am62ax-sk", os: "threadx"},
];

function getComponentProperty() {
    let property = {};

    property.dirPath = path.resolve(__dirname, "..");
    property.type = "executable";
    property.name = "enet_netxduo_cpsw_netxduo_gptp";
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
if(buildOption.cpu.match(/a53*/)) {
    build_property.cflags = cflags_a53;
} else {
    build_property.cflags = cflags;    
}
    build_property.lflags = lflags;
    build_property.loptflags = loptflags;
    build_property.defines = defines;

    return build_property;
}

module.exports = {
    getComponentProperty,
    getComponentBuildProperty,
};
