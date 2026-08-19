let path = require('path');

let device = "am263x";

const files = {
    common: [
        /* this example */
        "main.c",
        "tsndemo_listener.c",
        /* shared demo source (examples/tsn_demo/common) */
        "tsndemo_enet.c",
        "tsndemo_dataflow.c",
        "tsndemo_feature.c",
        /* shared TSN framework (examples/tsn) */
        "tsninit.c",
        "gptp_init.c",
        "debug_log.c",
    ],
};

const incfiles = {
    common: [
        "tsndemo.h",
        /* shared TSN framework headers */
        "common.h",
        "debug_log.h",
        "tsnapp_porting.h",
        "tsninit.h",
    ],
};

/* Relative to where the makefile will be generated
 * Typically at <example_folder>/<BOARD>/<core_os_combo>/<compiler>
 */
const filedirs = {
    common: [
        "..",                          /* core_os_combo base */
        "../../..",                    /* example base (tsn_demo/listener) */
        "../../../../common", /* shared demo source */
        "../../../../../tsn",             /* shared TSN framework */
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
        "${MCU_PLUS_SDK_PATH}/source/networking/tsn/tsn-stack/license_lib",
    ],
};

const includes_freertos_r5f = {
    common: [
        "${MCU_PLUS_SDK_PATH}/source/board/ethphy/enet/rtos_drivers/include",
        "${MCU_PLUS_SDK_PATH}/source/board/ethphy/port",
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/FreeRTOS-Kernel/include",
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/portable/TI_ARM_CLANG/ARM_CR5F",
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/config/am263x/r5f",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/core/utils",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/core/utils/include",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/core/utils/V3",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/core",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/core/include",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/core/include/phy",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/core/include/core",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/soc/am263x",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/hw_include",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/hw_include/mdio/V4",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/core/examples/tsn",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/core/examples/tsn_demo/common",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/core/examples/tsn_demo/listener",
        "${MCU_PLUS_SDK_PATH}/source/networking/tsn/tsn-stack",
        "${MCU_PLUS_SDK_PATH}/source/networking/tsn/tsn-stack/tsn_gptp",
        "${MCU_PLUS_SDK_PATH}/source/networking/tsn/tsn-stack/tsn_unibase",
        "${MCU_PLUS_SDK_PATH}/source/networking/tsn/tsn-stack/tsn_gptp/tilld",
        "${MCU_PLUS_SDK_PATH}/source/networking/tsn/tsn-stack/tsn_combase/tilld/sitara",
        "${MCU_PLUS_SDK_PATH}/source/networking/tsn/tsn-stack/tsn_gptp/gptpconf",
        "${MCU_PLUS_SDK_PATH}/source/networking/tsn/tsn-stack/tsn_uniconf",
        "${MCU_PLUS_SDK_PATH}/source/networking/tsn/tsn-stack/tsn_uniconf/yangs",
    ],
};

const libs_freertos_r5f = {
    common: [
        "freertos.am263x.r5f.ti-arm-clang.${ConfigName}.lib",
        "drivers.am263x.r5f.ti-arm-clang.${ConfigName}.lib",
        "enet-cpsw.am263x.r5f.ti-arm-clang.${ConfigName}.lib",
        "board.am263x.r5f.ti-arm-clang.${ConfigName}.lib",
        "libc.a",
        "libsysbm.a",
        "tsn_combase-freertos.am263x.r5f.ti-arm-clang.${ConfigName}.lib",
        "tsn_unibase-freertos.am263x.r5f.ti-arm-clang.${ConfigName}.lib",
        "tsn_gptp-freertos.am263x.r5f.ti-arm-clang.${ConfigName}.lib",
        "tsn_uniconf-freertos.am263x.r5f.ti-arm-clang.${ConfigName}.lib",
        "yangemb-freertos.am263x.r5f.ti-arm-clang.lib",
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
        /* GPTP_ENABLED is the ONLY TSN framework module enabled; do NOT add
         * EST_APP_ENABLED/AVTP_ENABLED/etc. - EST is configured via direct
         * TAS IOCTLs in tsndemo_feature.c */
        'GPTP_ENABLED=1',
        'TSNDEMO_ROLE_LISTENER=1',
        /* required by tsn_gptp/gptpclock.h's UB_LOG macros, must match the
         * category used by the tsn_gptp library build */
        'UB_LOGCAT=3',
        'UB_LOGTSTYPE=UB_CLOCK_REALTIME',
        'TSNAPP_LOGLEVEL=\\\"4,ubase:45,cbase:45,uconf:45,gptp:33,lldp:45,avtp:45,nconf:45\\\"',
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
        "./linker.cmd",
    ]
};

const syscfgfile = "../example.syscfg";

const readmeDoxygenPageTag = "EXAMPLES_ENET_CPSW_TSN_DEMO_LISTENER";

const templates_freertos_r5f =
[
    {
        input: "source/networking/enet/core/sysconfig/.project/templates/freertos/main_freertos.c.xdt",
        output: "../main.c",
        options: {
            entryFunction: "TsnDemoListener_mainTask",
            taskPri : "17",
            stackSize : "16384",
        },
    },
];

const buildOptionCombos = [
    { device: device, cpu: "r5fss0-0", cgt: "ti-arm-clang", board: "am263x-lp", os: "freertos"},
    { device: device, cpu: "r5fss0-0", cgt: "ti-arm-clang", board: "am263x-cc", os: "freertos"},
];

function getComponentProperty() {
    let property = {};

    property.dirPath = path.resolve(__dirname, "..");
    property.type = "executable";
    property.name = "tsn_demo_listener";
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
            build_property.libs = libs_freertos_r5f;
            build_property.templates = templates_freertos_r5f;
            build_property.defines = defines_r5f;
            build_property.cflags = cflags_r5f;
            build_property.lflags = lflags_r5f;
            build_property.projectspecLnkPath = linker_includePath_freertos;
            build_property.loptflags = loptflags_r5f;
        }
    }

    return build_property;
}

module.exports = {
    getComponentProperty,
    getComponentBuildProperty,
};
