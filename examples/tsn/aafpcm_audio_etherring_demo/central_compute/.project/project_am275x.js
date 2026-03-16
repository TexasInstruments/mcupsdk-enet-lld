let path = require('path');

let device = "am275x";

const main_files = {
    common: [
        "gptp_config.c",
        "avtp_init.c",
        "default_flow_cfg.c",
        "tsninit.c",
        "debug_log.c",
        "enetapp_cpsw.c",
        "tsn_audioapp_cpsw_main.c",
        "background_traffic.c",
        "main.c",
        "board.c",
        "crf_hw_config.c",
        "crf_app.c",
        "central_aaf_app.c",
        "shm_cirbuf.c",
        "ts_mcasp_config.c",
        "gpio_sm.c",
        "control_data.c",
        "ether_ring.c",
        "avtpcf.c",
        "timer_pwm.c",
        "cbs_config.c",
    ],
};
 
const main_incfiles = {
    common: [
        "Enetapp_common.h",
        "avtpcf.h",
        "common.h",
        "crf_app.h",
        "crf_hw_config.h",
        "debug_log.h",
        "enetapp_cpsw.h",
        "gpio_sm.h",
        "sample_audio.h",
        "shm_cirbuf.h",
        "tsnapp_porting.h",
        "tsninit.h",
    ],
};

const remote_files = {
    common: [
        "main.c",
        "remote_mcasp_playback.c",
        "shm_cirbuf.c",
        "gpio_sm.c",
    ],
};

const remote_incfiles = {
    common: [
        "gpio_sm.h",
        "shm_cirbuf.h",
    ],
};

/* Relative to where the makefile will be generated
 * Typically at <example_folder>/<BOARD>/<core_os_combo>/<compiler>
 */
const main_filedirs = {
    common: [
        "..",       /* core_os_combo base */
        "$(MCU_PLUS_SDK_PATH)/source/networking/enet/core/examples/tsn/aafpcm_audio_etherring_demo/central_compute", /* Example base */
        "$(MCU_PLUS_SDK_PATH)/source/networking/enet/core/examples/tsn/aafpcm_audio_etherring_demo", /* Example base */
        "$(MCU_PLUS_SDK_PATH)/source/networking/enet/core/examples/tsn/aafpcm_audio_etherring_demo/common_files", /* Example base */
        "$(MCU_PLUS_SDK_PATH)/source/networking/enet/core/examples/tsn", /* Example base */
        // "$(MCU_PLUS_SDK_PATH)/source/networking/enet/core/examples/tsn/nrt_flow", /* Example base */
        "$(MCU_PLUS_SDK_PATH)/source/networking/enet/core/examples/tsn/aafpcm_app", /* Example base */
        "$(MCU_PLUS_SDK_PATH)/source/networking/enet/core/ether_ring/src/",
        ],
};
""

const remote_filedirs = {
    common: [
        "..",       /* core_os_combo base */
        "$(MCU_PLUS_SDK_PATH)/source/networking/enet/core/examples/tsn/aafpcm_audio_etherring_demo/central_compute/remote_core", /* Example base */
        "$(MCU_PLUS_SDK_PATH)/source/networking/enet/core/examples/tsn/aafpcm_audio_etherring_demo/common_files", /* Example base */
        "$(MCU_PLUS_SDK_PATH)/source/networking/enet/core/examples/tsn/aafpcm_audio_etherring_demo", /* Example base */
    ],
};

const libdirs_freertos = {
    common: [
        "generated",
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/lib",
        "${MCU_PLUS_SDK_PATH}/source/drivers/lib",
        "${MCU_PLUS_SDK_PATH}/source/board/lib",
        "${MCU_PLUS_SDK_PATH}/source/drivers/udma/lib",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/lib",
        "${MCU_PLUS_SDK_PATH}/source/networking/tsn/lib",
        "${MCU_PLUS_SDK_PATH}/source/networking/tsn/tsn-stack/eval_lib",
        "${MCU_PLUS_SDK_PATH}/source/networking/tsn/tsn-stack/license_lib",
    ],
};

const includes_freertos_r5f = {
    common: [
        "$(MCU_PLUS_SDK_PATH)/source/networking/enet/core/examples/tsn", /* Example base */
        "$(MCU_PLUS_SDK_PATH)/source/networking/enet/core/examples/tsn/aafpcm_audio_etherring_demo/common_files", /* Example base */
        "${MCU_PLUS_SDK_PATH}/source/board/ethphy/enet/rtos_drivers/include",
        "${MCU_PLUS_SDK_PATH}/source/board/ethphy/port",
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/FreeRTOS-Kernel/include",
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/portable/TI_ARM_CLANG/ARM_CR5F",
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/config/am275x/r5f",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/utils",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/core/utils/include",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/core/utils/V3",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/core",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/core/ether_ring/inc/",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/core/include",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/core/include/phy",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/core/include/core",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/soc/k3/am275x",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/hw_include",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/hw_include/mdio/V4",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/core/examples/tsn",
        "${MCU_PLUS_SDK_PATH}/source/networking/tsn/tsn-stack",
        "${MCU_PLUS_SDK_PATH}/source/networking/tsn/tsn-stack/eval_inc",
        "${MCU_PLUS_SDK_PATH}/source/networking/tsn/tsn-stack/eval_inc/tsn_l2",
        "${MCU_PLUS_SDK_PATH}/source/networking/tsn/tsn-stack/tsn_unibase",
        "${MCU_PLUS_SDK_PATH}/source/networking/tsn/tsn-stack/tsn_gptp",
        "${MCU_PLUS_SDK_PATH}/source/networking/tsn/tsn-stack/tsn_gptp/tilld",
        "${MCU_PLUS_SDK_PATH}/source/networking/tsn/tsn-stack/tsn_combase/tilld/sitara",
        "${MCU_PLUS_SDK_PATH}/source/networking/tsn/tsn-stack/tsn_gptp/gptpconf",
        "${MCU_PLUS_SDK_PATH}/source/networking/tsn/tsn-stack/tsn_uniconf",
        "${MCU_PLUS_SDK_PATH}/source/networking/tsn/tsn-stack/tsn_uniconf/yangs",
    ],
};

const includes_freertos_c75 = {
    common: [
        "$(MCU_PLUS_SDK_PATH)/source/networking/enet/core/examples/tsn/aafpcm_audio_demo/common_files", /* Example base */
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/FreeRTOS-Kernel/include",
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/portable/TI_CGT/DSP_C75X",
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/config/am275x/c75x",
        "${MCU_PLUS_SDK_PATH}/source/networking/enet/core/examples/tsn/avb_cpsw_app",
    ],
};

const libs_freertos_r5f = {
    common: [
        "freertos.am275x.r5f.ti-arm-clang.${ConfigName}.lib",
        "drivers.am275x.r5f.ti-arm-clang.${ConfigName}.lib",
        "enet-cpsw.am275x.r5f.ti-arm-clang.${ConfigName}.lib",
        "board.am275x.r5f.ti-arm-clang.${ConfigName}.lib",
        "libc.a",
        "libsysbm.a",
        "tsn_combase-freertos.am275x.r5f.ti-arm-clang.${ConfigName}.lib",
        "tsn_unibase-freertos.am275x.r5f.ti-arm-clang.${ConfigName}.lib",
        "tsn_gptp-freertos.am275x.r5f.ti-arm-clang.${ConfigName}.lib",
        "tsn_uniconf-freertos.am275x.r5f.ti-arm-clang.${ConfigName}.lib",
        "tsn_l2-freertos.am275x.r5f.ti-arm-clang.lib",
        "yangemb-freertos.am275x.r5f.ti-arm-clang.lib"
    ],
};
const libs_freertos_c75 = {
    common: [
        "freertos.am275x.c75x.ti-c7000.${ConfigName}.lib",
        "drivers.am275x.c75x.ti-c7000.${ConfigName}.lib",
        "udma.am275x.c75x.ti-c7000.${ConfigName}.lib",
    ],
};

const linker_includePath_freertos = {
    common: [
        "${PROJECT_BUILD_DIR}/syscfg",

    ],
};

const defines_c75 = {
    common: [
        "SOC_AM275X",
    ],
};

const defines_r5f = {
    common: [
        "SOC_AM275X",
        "CORE_R5F",
        "ENET_ENABLE_PER_CPSW=1",
        'PRINT_FORMAT_NO_WARNING',
        'SITARA',
        'NO_GETOPT_LONG=1',
        'UB_LOGCAT=5',
        'TSNAPP_LOGLEVEL=\\\"4,ubase:45,cbase:45,uconf:45,gptp:45,lldp:45,avtp:45,nconf:45\\\"',
        'AVTP_ENABLED=1',
        'AVTP_HAVE_NO_SIGNAL=1',
        'GPTP_ENABLED=1',
        'GPTP_MASTER=1',
        'AVTP_DIRECT_MODE=1',
        'MCASP_PLAYBACK=1',
        'AVTP_CRF_TALKER=1',
        'DEF_NODE_CENTRAL=1',
        'CBS_APP_ENABLED',
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
        "linker.cmd",
    ]
};

const syscfgfile = "../example.syscfg";

const readmeDoxygenPageTag = "EXAMPLES_ENET_AVB_AUDIO_ETHERRING_DEMO";

const templates_freertos_r5f =
[
    {
        input: "source/networking/enet/core/sysconfig/.project/templates/freertos/main_freertos.c.xdt",
        output: "../main.c",
        options: {
            entryFunction: "EnetApp_mainTask",
            taskPri : "2",
            stackSize : "16384",
        },
    },
];

const templates_freertos_c75 =
[
    {
        input: ".project/templates/am275x/freertos/main_freertos.c.xdt",
        output: "../main.c",
        options: {
            entryFunction: "mcasp_playback_main",
            stackSize: 64*1024,
        },
    }
];

const buildOptionCombos = [
    { device: device, cpu: "r5fss0-0", cgt: "ti-arm-clang", board: "am275x-evm-dp83867/am275x-evm", os: "freertos"},
    { device: device, cpu: "c75ss0-0", cgt: "ti-c7000",     board: "am275x-evm-dp83867/am275x-evm", os: "freertos"},
];

function getComponentProperty() {
    let property = {};

    property.dirPath = path.resolve(__dirname, "..");
    property.type = "executable";
    property.name = "central_ring_audio_app";
    property.isInternal = false;
    property.buildOptionCombos = buildOptionCombos;

    return property;
}

function getComponentBuildProperty(buildOption) {
    let build_property = {};

    if(buildOption.cpu.match(/r5f*/))
    {
        build_property.files = main_files;
        build_property.filedirs = main_filedirs;
    }
    else if(buildOption.cpu.match(/c75*/))
    {
        build_property.files = remote_files;
        build_property.filedirs = remote_filedirs;
    }

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
            build_property.incfiles = main_incfiles;
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
    else if(buildOption.cpu.match(/c75*/))
    {
        build_property.incfiles = remote_incfiles;
        build_property.defines = defines_c75;
        build_property.includes = includes_freertos_c75;
        build_property.libdirs = libdirs_freertos;
        build_property.libs = libs_freertos_c75;
        build_property.templates = templates_freertos_c75;
    }

    return build_property;
}

module.exports = {
    getComponentProperty,
    getComponentBuildProperty,
};
