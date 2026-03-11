let path = require('path');

let device = "am243x";

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
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/config/am243x/r5f",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/core/utils",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/core/utils/include",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/core/utils/V3",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/core",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/core/include",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/core/include/phy",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/core/include/per",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/core/include/core",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/core/lwipif/inc",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/hw_include",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/soc/k3/am64x_am243x",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/hw_include",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/hw_include/mdio/V4",
        "${MCU_PLUS_SDK_PATH}/source/networking/tsn/tsn-stack",
        "${MCU_PLUS_SDK_PATH}/source/networking/tsn/tsn-stack/tsn_gptp",
        "${MCU_PLUS_SDK_PATH}/source/networking/tsn/tsn-stack/tsn_unibase",
        "${MCU_PLUS_SDK_PATH}/source/networking/tsn/tsn-stack/tsn_gptp/tilld",
        "${MCU_PLUS_SDK_PATH}/source/networking/tsn/tsn-stack/tsn_combase/tilld/sitara",
        "${MCU_PLUS_SDK_PATH}/source/networking/tsn/tsn-stack/tsn_gptp/gptpconf",
        "${MCU_PLUS_SDK_PATH}/source/networking/tsn/tsn-stack/tsn_uniconf",
        "${MCU_PLUS_SDK_PATH}/source/networking/tsn/tsn-stack/tsn_uniconf/yangs",
        "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-stack/src/include",
        "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-port/include",
        "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-port/freertos/include",
        "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-stack/contrib",
        "${MCU_PLUS_SDK_PATH}/source/networking/lwip/lwip-config/am243x",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/core/examples/enet_cli_app/",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/core/examples/enet_cli_app/cli_gptp",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/core/examples/enet_cli_app/cli_lwip",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/core/examples/enet_cli_app/cli_ale",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/core/examples/enet_cli_app/cli_phy",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/core/examples/enet_cli_app/enet_cli_wrapper",
    ],
};

const libs_freertos_r5f = {
    common: [
        "freertos.am243x.r5f.ti-arm-clang.${ConfigName}.lib",
        "drivers.am243x.r5f.ti-arm-clang.${ConfigName}.lib",
        "enet-cpsw.am243x.r5f.ti-arm-clang.${ConfigName}.lib",
        "board.am243x.r5f.ti-arm-clang.${ConfigName}.lib",
        "libc.a",
        "libsysbm.a",
        "tsn_combase-freertos.am243x.r5f.ti-arm-clang.${ConfigName}.lib",
        "tsn_unibase-freertos.am243x.r5f.ti-arm-clang.${ConfigName}.lib",
        "tsn_gptp-freertos.am243x.r5f.ti-arm-clang.${ConfigName}.lib",
        "tsn_uniconf-freertos.am243x.r5f.ti-arm-clang.${ConfigName}.lib",
        "lwipif-cpsw-freertos.am243x.r5f.ti-arm-clang.${ConfigName}.lib",
        "lwip-freertos.am243x.r5f.ti-arm-clang.${ConfigName}.lib",
        "lwip-contrib-freertos.am243x.r5f.ti-arm-clang.${ConfigName}.lib",
        "yangemb-freertos.am243x.r5f.ti-arm-clang.lib",
    ],
};

const libs_freertos_r5f_gcc = {
    common: [
        "freertos.am243x.r5f.gcc-armv7.${ConfigName}.lib",
        "drivers.am243x.r5f.gcc-armv7.${ConfigName}.lib",
        "enet-cpsw.am243x.r5f.gcc-armv7.${ConfigName}.lib",
        "board.am243x.r5f.gcc-armv7.${ConfigName}.lib",
        "libc.a",
        "libsysbm.a",
        "tsn_combase-freertos.am243x.r5f.ti-arm-clang.${ConfigName}.lib",
        "tsn_unibase-freertos.am243x.r5f.ti-arm-clang.${ConfigName}.lib",
        "tsn_gptp-freertos.am243x.r5f.ti-arm-clang.${ConfigName}.lib",
        "tsn_uniconf-freertos.am243x.r5f.ti-arm-clang.${ConfigName}.lib",
        "lwipif-cpsw-freertos.am243x.r5f.gcc-armv7.${ConfigName}.lib",
        "lwip-freertos.am243x.r5f.gcc-armv7.${ConfigName}.lib",
        "lwip-contrib-freertos.am243x.r5f.gcc-armv7.${ConfigName}.lib",
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

const cflags_r5f_gcc = {
    common: [
        "--include tsn_buildconf/sitara_buildconf.h",
    ],
    release: [
        "-flto",
    ]
}

const lflags_r5f = {
    common: [
        "--zero_init=on",
        "--use_memset=fast",
        "--use_memcpy=fast"
    ],
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

const lnkfiles = {
    common: [
        "linker.cmd",
    ]
};

const lnkpreprocessor_gcc = {
    common: [
        "linker_preprocessor.cmd",
    ]
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
            stackSize : "8192",
        },
    }
];


const buildOptionCombos = [
    { device: device, cpu: "r5fss0-0", cgt: "ti-arm-clang", board: "am243x-lp", os: "freertos"},
    { device: device, cpu: "r5fss0-0", cgt: "ti-arm-clang", board: "am243x-evm", os: "freertos"},
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
    build_property.lnkfiles = lnkfiles;
    build_property.syscfgfile = syscfgfile;
        build_property.readmeDoxygenPageTag = readmeDoxygenPageTag;
    if(buildOption.cpu.match(/r5f*/)) {
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
            if(buildOption.cgt.match(/gcc*/))
            {
                build_property.libs = libs_freertos_r5f_gcc;
                build_property.cflags = cflags_r5f_gcc;
                build_property.lnkpreprocessor_gcc = lnkpreprocessor_gcc;
            }
            else
            {
                build_property.libs = libs_freertos_r5f;
                build_property.cflags = cflags_r5f;
                build_property.loptflags = loptflags_r5f;
                build_property.lflags = lflags_r5f;
            }
            build_property.templates = templates_freertos_r5f;
            build_property.defines = defines_r5f;
            build_property.projectspecLnkPath = linker_includePath_freertos;
        }
    }
    return build_property;
}

module.exports = {
    getComponentProperty,
    getComponentBuildProperty,
};
