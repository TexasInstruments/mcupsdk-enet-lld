let path = require('path');

let device = "tda54";

const files = {
    common: [
            "npac_test.c",
            "npac_main.c",
            "enet_custom_board_config.c",
            "main.c",
    ],
};

/* Relative to where the makefile will be generated
 * Typically at <example_folder>/<BOARD>/<core_os_combo>/<compiler>
 */
const filedirs = {
    common: [
        "..",       /* core_os_combo base */
        "../../../../", /* Example base */
    ],
};

const libdirs_freertos = {
    common: [
        "generated",
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/lib",
        "${MCU_PLUS_SDK_PATH}/source/drivers/lib",
        "${MCU_PLUS_SDK_PATH}/source/board/lib",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/lib",
    ],
};

const includes_freertos_mcu0 = {
    common: [
        "${MCU_PLUS_SDK_PATH}/source/board/ethphy/enet/rtos_drivers/include",
        "${MCU_PLUS_SDK_PATH}/source/board/ethphy/port",
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/FreeRTOS-Kernel/include",
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/portable/TI_ARM_CLANG/ARM_CM55",
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/config/tda54/m55",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/utils",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/core/utils/include",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/core/utils/V3",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/core",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/core/include",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/core/include/phy",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/core/include/core",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/soc/k3/tda54",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/hw_include",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/hw_include/mdio/V4"
    ],
};


const libs_freertos_mcu0 = {
    common: [
        "freertos.tda54.mcu.ti-arm-clang.${ConfigName}.lib",
        "drivers.tda54.mcu.ti-arm-clang.${ConfigName}.lib",
        // "board.tda54.m55.ti-arm-clang.${ConfigName}.lib",
        "enet-cpsw.tda54.mcu.ti-arm-clang.${ConfigName}.lib",
    ],
};


const linker_includePath_freertos = {
    common: [
        "${PROJECT_BUILD_DIR}/syscfg",

    ],
};

const defines_mcu0 = {
    common: [
    ],
};

const cflags_mcu0 = {
    common: [
    ],
    release: [
    ],
};

const cflags_a53 = {
    common: [
        "-Wno-unused-function",
        "-Wno-unused-variable",
    ],
    release: [
        "-flto",
    ],
};

const lflags_mcu0 = {
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

const loptflags_mcu0 = {
    release: [
        "-mcpu=cortex-m55",
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

const readmeDoxygenPageTag = "EXAMPLES_ENET_NPAC_DATAPATH";

// const templates_freertos_mcu0 =
// [
//     {
//         input: "source/networking/enet/core/sysconfig/.project/templates/freertos/main_freertos.c.xdt",
//         output: "../main.c",
//         options: {
//             entryFunction: "EnetLpbk_mainTask",
//             taskPri : "2",
//             stackSize : "8192",
//         },
//     }
// ];

const templates_freertos_mcu0 =
[
    {
        input: ".project/templates/tda54/freertos/main_freertos.c.xdt",
        output: "../main.c",
        options: {
        entryFunction: "Npac_mainTask",
        },
    }
];

const buildOptionCombos = [
   { device: device, cpu: "mcu0", cgt: "ti-arm-clang", board: "tda54-evm", os: "freertos", isPartOfSystemProject: false},
];

function getComponentProperty() {
    let property = {};

    property.dirPath = path.resolve(__dirname, "..");
    property.type = "executable";
    property.name = "basic_npac_datapath";
    property.isInternal = false;
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

    if(buildOption.cpu.match(/mcu0*/)) {
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
            build_property.includes = includes_freertos_mcu0;
            build_property.libdirs = libdirs_freertos_cpy;
            build_property.libs = libs_freertos_mcu0;
            build_property.templates = templates_freertos_mcu0;
            build_property.defines = defines_mcu0;
            build_property.cflags = cflags_mcu0;
            build_property.lflags = lflags_mcu0;
            build_property.projectspecLnkPath = linker_includePath_freertos;
            build_property.loptflags = loptflags_mcu0;
        }
    }

    return build_property;
}

module.exports = {
    getComponentProperty,
    getComponentBuildProperty,
};
