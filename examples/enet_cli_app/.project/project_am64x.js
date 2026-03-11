let path = require('path');

let device = "am64x";

const files = {
    common: [
            "enet_cli_wrapper.c",
            "enet_cli.c",
            "cli_ale_unicast.c",
            "cli_ale_mcast.c",
            "cli_ale_vlan.c",
            "lwip_shell.c",
            "cli_phy_phymode.c",
            "cli_lwip.c",
            "cli_gptp_log.c",
            "cli_gptp_app.c",
            "enet_cli_config.c",
            "enet_cli_debug.c",
            "enet_cli_layer2_datapath.c",
            "enet_cli_phy.c",
            "enet_cli_utils.c",
            "enet_cli_main.c",
            "main.c",
    ],
};

const incfiles = {
    common: [
        "enet_cli.h",
        "enet_cli_layer2_datapath.h",
        "enet_cli_wrapper.h",
        "cli_ale_unicast.h",
        "cli_ale_vlan.h",
        "cli_lwip.h",
        "lwip_shell.h",
        "cli_gptp_app.h",
        "cli_gptp_log.h",
    ],
};

/* Relative to where the makefile will be generated
 * Typically at <example_folder>/<BOARD>/<core_os_combo>/<compiler>
 */
const filedirs = {
    common: [
        "..",       /* core_os_combo base */
        "../../..", /* Example base */
        "../../../cli_gptp", /* Example base */
        "../../../cli_lwip", /* Example base */
        "../../../enet_cli_wrapper", /* Example base */
        "../../../cli_ale", /* Example base */
        "../../../cli_phy", /* Example base */
    ],
};

const libdirs_freertos = {
    common: [
        "generated",
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/lib",
        "${MCU_PLUS_SDK_PATH}/source/drivers/lib",
        "${MCU_PLUS_SDK_PATH}/source/board/lib",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/lib",
        "${MCU_PLUS_SDK_PATH}/source/networking/tsn/lib",
        "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lib",
        "${MCU_PLUS_SDK_PATH}/source/networking/tsn/tsn-stack/license_lib",
    ],
};

const includes_freertos_r5f = {
    common: [
        "${MCU_PLUS_SDK_PATH}/source/board/ethphy/enet/rtos_drivers/include",
        "${MCU_PLUS_SDK_PATH}/source/board/ethphy/port",
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/FreeRTOS-Kernel/include",
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/FreeRTOS-Plus-CLI",
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/portable/TI_ARM_CLANG/ARM_CR5F",
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/config/am64x/r5f",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/core/utils",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/core/utils/include",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/core/utils/V3",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/core",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/core/include",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/core/include/phy",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/core/include/per",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/core/include/core",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/soc/k3/am64x_am243x",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/hw_include",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/hw_include/mdio/V4",
        "${MCU_PLUS_SDK_PATH}/source/networking/tsn/tsn-stack",
        "${MCU_PLUS_SDK_PATH}/source/networking/tsn/tsn-stack/tsn_gptp",
        //"${MCU_PLUS_SDK_PATH}/source/networking/tsn/tsn-stack/tsn_unibase",
        "${MCU_PLUS_SDK_PATH}/source/networking/tsn/tsn-stack/tsn_gptp/tilld",
        "${MCU_PLUS_SDK_PATH}/source/networking/tsn/tsn-stack/tsn_combase/tilld/sitara",
        "${MCU_PLUS_SDK_PATH}/source/networking/tsn/tsn-stack/tsn_gptp/gptpconf",
        "${MCU_PLUS_SDK_PATH}/source/networking/tsn/tsn-stack/tsn_uniconf",
        "${MCU_PLUS_SDK_PATH}/source/networking/tsn/tsn-stack/tsn_uniconf/yangs",
        "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-stack/src/include",
        "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-port/include",
        "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-port/freertos/include",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/core/lwipif/inc",
        "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-stack/contrib",
        "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-config/am64x",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/core/examples/enet_cli_app/",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/core/examples/enet_cli_app/cli_gptp",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/core/examples/enet_cli_app/cli_lwip",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/core/examples/enet_cli_app/cli_ale",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/core/examples/enet_cli_app/cli_phy",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/core/examples/enet_cli_app/enet_cli_wrapper",
    ],
};

const includes_freertos_a53 = {
    common: [
        "${MCU_PLUS_SDK_PATH}/source/board/ethphy/enet/rtos_drivers/include",
        "${MCU_PLUS_SDK_PATH}/source/board/ethphy/port",
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/FreeRTOS-Kernel/include",
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/FreeRTOS-Plus-CLI",
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/portable/GCC/ARM_CA53",
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/config/am64x/a53",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/core/utils",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/core/utils/include",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/core/utils/V3",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/core",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/core/include",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/core/include/phy",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/core/include/per",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/core/include/core",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/soc/k3/am64x_am243x",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/hw_include",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/hw_include/mdio/V4",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/core/examples/tsn",
        "${MCU_PLUS_SDK_PATH}/source/networking/tsn/tsn-stack",
        "${MCU_PLUS_SDK_PATH}/source/networking/tsn/tsn-stack/tsn_gptp",
        //"${MCU_PLUS_SDK_PATH}/source/networking/tsn/tsn-stack/tsn_unibase",
        "${MCU_PLUS_SDK_PATH}/source/networking/tsn/tsn-stack/tsn_gptp/tilld",
        "${MCU_PLUS_SDK_PATH}/source/networking/tsn/tsn-stack/tsn_combase/tilld/sitara",
        "${MCU_PLUS_SDK_PATH}/source/networking/tsn/tsn-stack/tsn_gptp/gptpconf",
        "${MCU_PLUS_SDK_PATH}/source/networking/tsn/tsn-stack/tsn_uniconf",
        "${MCU_PLUS_SDK_PATH}/source/networking/tsn/tsn-stack/tsn_uniconf/yangs",
        "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-stack/src/include",
        "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-port/include",
        "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-port/freertos/include",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/core/lwipif/inc",
        "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-stack/contrib",
        "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-config/am64x",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/core/examples/enet_cli_app/",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/core/examples/enet_cli_app/cli_gptp",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/core/examples/enet_cli_app/cli_lwip",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/core/examples/enet_cli_app/cli_ale",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/core/examples/enet_cli_app/cli_phy",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/core/examples/enet_cli_app/enet_cli_wrapper",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/core/examples/enet_cli_app/FreeRTOS-Plus-CLI",
    ],
};

const libs_freertos_r5f = {
    common: [
        "freertos.am64x.r5f.ti-arm-clang.${ConfigName}.lib",
        "drivers.am64x.r5f.ti-arm-clang.${ConfigName}.lib",
        "enet-cpsw.am64x.r5f.ti-arm-clang.${ConfigName}.lib",
        "board.am64x.r5f.ti-arm-clang.${ConfigName}.lib",
        "libc.a",
        "libsysbm.a",
        "tsn_combase-freertos.am64x.r5f.ti-arm-clang.${ConfigName}.lib",
        "tsn_unibase-freertos.am64x.r5f.ti-arm-clang.${ConfigName}.lib",
        "tsn_gptp-freertos.am64x.r5f.ti-arm-clang.${ConfigName}.lib",
        "tsn_uniconf-freertos.am64x.r5f.ti-arm-clang.${ConfigName}.lib",
        "lwipif-cpsw-freertos.am64x.r5f.ti-arm-clang.${ConfigName}.lib",
        "lwip-freertos.am64x.r5f.ti-arm-clang.${ConfigName}.lib",
        "lwip-contrib-freertos.am64x.r5f.ti-arm-clang.${ConfigName}.lib",
        "yangemb-freertos.am64x.r5f.ti-arm-clang.lib",
    ],
};

const libs_freertos_a53 = {
    common: [

        "freertos.am64x.a53.gcc-aarch64.${ConfigName}.lib",
        "drivers.am64x.a53.gcc-aarch64.${ConfigName}.lib",
        "enet-cpsw.am64x.a53.gcc-aarch64.${ConfigName}.lib",
        "board.am64x.a53.gcc-aarch64.${ConfigName}.lib",
        "tsn_combase-freertos.am64x.a53.gcc-aarch64.${ConfigName}.lib",
        "tsn_unibase-freertos.am64x.a53.gcc-aarch64.${ConfigName}.lib",
        "tsn_gptp-freertos.am64x.a53.gcc-aarch64.${ConfigName}.lib",
        "tsn_uniconf-freertos.am64x.a53.gcc-aarch64.${ConfigName}.lib",
        "lwipif-cpsw-freertos.am64x.a53.gcc-aarch64.${ConfigName}.lib",
        "lwip-freertos.am64x.a53.gcc-aarch64.${ConfigName}.lib",
        "lwip-contrib-freertos.am64x.a53.gcc-aarch64.${ConfigName}.lib",
        "yangemb-freertos.am64x.a53.gcc-aarch64.lib"
    ],
};


const linker_includePath_freertos = {
    common: [
        "${PROJECT_BUILD_DIR}/syscfg",

    ],
};

const defines_r5f = {
    common: [
        "ENET_ENABLE_PER_CPSW=1",
        'PRINT_FORMAT_NO_WARNING',
        'SITARA',
        'GPTP_ENABLED=1',
    ],
};

const defines_a53 = {
    common: [
        "ENET_ENABLE_PER_CPSW=1",
        'PRINT_FORMAT_NO_WARNING',
        'SITARA',
        'GPTP_ENABLED=1',
    ],
};

const cflags_r5f = {
    common: [
        "--include tsn_buildconf/sitara_buildconf.h",
    ],
    release: [
        "-Oz",
        "-flto",
    ],
};

const cflags_a53 = {
    common: [
        "--include tsn_buildconf/sitara_buildconf.h",
        "-Wno-unused-function",
    ],
    release: [
       "-flto",
    ],
};

const lflags_r5f = {
    common: [
        "--zero_init=on",
        "--use_memset=fast",
        "--use_memcpy=fast"
    ],
};

const lflags_a53 = {
    common: [

    ],
};

const lnkfiles = {
    common: [
        "linker.cmd",
    ]
};

const lnkfiles_a53 = {
    common: [
        "linker.cmd",
    ]
};

const loptflags_r5f = {
    release: [
        "-mcpu=cortex-r5",
        "-mfloat-abi=hard",
        "-mfpu=vfpv3-d16",
        "-mthumb",
        "-Oz",
        "-flto"
    ],
};

const loptflags_a53 = {
    release: [
        "-flto"
    ],
};

const syscfgfile = "../example.syscfg";

const readmeDoxygenPageTag = "EXAMPLES_ENET_CLI_APP";

const templates_freertos_r5f =
[
    {
        input: "source/networking/enet/core/sysconfig/.project/templates/freertos/main_freertos.c.xdt",
        output: "../main.c",
        options: {
            entryFunction: "EnetCLI_mainTask",
            taskPri : "2",
            stackSize : "16384",
        },
    },
];

const templates_freertos_a53 =
[
    {
        input: ".project/templates/am64x/common/linker_a53.cmd.xdt",
        output: "linker.cmd",
        options: {
            enableDMARegion: true,
            dmaHeapSize: 0x20000,
        }
    },
    {
        input: "source/networking/enet/core/sysconfig/.project/templates/freertos/main_freertos.c.xdt",
        output: "../main.c",
        options: {
            entryFunction: "EnetCLI_mainTask",
            taskPri : "2",
            stackSize : "16384",
        },
    }
];

const buildOptionCombos = [
    { device: device, cpu: "r5fss0-0", cgt: "ti-arm-clang", board: "am64x-evm", os: "freertos"},
    { device: device, cpu: "a53ss0-0", cgt: "gcc-aarch64", board: "am64x-evm", os: "freertos"},
    { device: device, cpu: "r5fss0-0", cgt: "ti-arm-clang", board: "am64x-sk", os: "freertos"},
    { device: device, cpu: "a53ss0-0", cgt: "gcc-aarch64", board: "am64x-sk", os: "freertos"},
];

function getComponentProperty() {
    let property = {};

    property.dirPath = path.resolve(__dirname, "..");
    property.type = "executable";
    property.name = "enet_cli_app";
    property.isInternal = false;
    property.buildOptionCombos = buildOptionCombos;

    return property;
}

function getComponentBuildProperty(buildOption) {
    let build_property = {};

    build_property.files = files;
    build_property.incfiles = incfiles;
    build_property.filedirs = filedirs;
    build_property.syscfgfile = syscfgfile;
        build_property.readmeDoxygenPageTag = readmeDoxygenPageTag;

    if(buildOption.cpu.match(/r5f*/)) {
        build_property.lnkfiles = lnkfiles;
        if(buildOption.os.match(/freertos*/) )
        {
            const _ = require('lodash');
            let libdirs_freertos_cpy = _.cloneDeep(libdirs_freertos);
            /* Logic to remove generated/ from libdirs_freertos, it generates warning for ccs build */
            if (buildOption.isProjectSpecBuild === true)
            {
                var delIndex = libdirs_freertos_cpy.common.indexOf('generated');
                if (delIndex !== -1) {
                    libdirs_freertos_cpy.common.splice(delIndex, 1);
                }
            }
            build_property.includes = includes_freertos_r5f;
            build_property.libdirs = libdirs_freertos_cpy;
            build_property.libs = libs_freertos_r5f;
            build_property.templates = templates_freertos_r5f;
            build_property.defines = defines_r5f;
            build_property.cflags = cflags_r5f;
            build_property.lflags = lflags_r5f;
            build_property.projectspecLnkPath = linker_includePath_freertos;
            build_property.loptflags = loptflags_r5f;
        }
    }

    if(buildOption.cpu.match(/a53*/)){
        build_property.lnkfiles = lnkfiles_a53;
        if(buildOption.os.match(/freertos*/) )
            {
                const _ = require('lodash');
                let libdirs_freertos_cpy = _.cloneDeep(libdirs_freertos);
                /* Logic to remove generated/ from libdirs_freertos, it generates warning for ccs build */
                if (buildOption.isProjectSpecBuild === true)
                {
                    var delIndex = libdirs_freertos_cpy.common.indexOf('generated');
                    if (delIndex !== -1) {
                        libdirs_freertos_cpy.common.splice(delIndex, 1);
                    }

                }
                build_property.includes = includes_freertos_a53;
                build_property.libdirs = libdirs_freertos_cpy;
                build_property.libs = libs_freertos_a53;
                build_property.templates = templates_freertos_a53;
                build_property.defines = defines_a53;
                build_property.cflags = cflags_a53;
                build_property.lflags = lflags_a53;
                build_property.loptflags = loptflags_a53;
            }

    }

    return build_property;
}

module.exports = {
    getComponentProperty,
    getComponentBuildProperty,
};
