
let common = system.getScript("/common");
let soc = system.getScript(`/networking/soc/networking_${common.getSocName()}`);

exports = common.getSelfSysCfgCoreName().includes('pru') ? {} : {
    displayName: "TI Networking",
    templates: [
        {
            name: "/networking/common/enet_config.c.xdt",
            outputPath: "ti_enet_config.c",
            alwaysRun: true,
        },
        {
            name: "/networking/common/enet_config.h.xdt",
            outputPath: "ti_enet_config.h",
            alwaysRun: true,
        },
        {
            name: "/networking/common/enet_init.c.xdt",
            outputPath: "ti_enet_init.c",
            alwaysRun: true,
        },
        {
            name: "/networking/common/dma_init.h.xdt",
            outputPath: "ti_enet_dma_init.h",
            alwaysRun: true,
        },
        {
            name: "/networking/common/dma_init.c.xdt",
            outputPath: "ti_enet_dma_init.c",
            alwaysRun: true,
        },
        {
            name: "/networking/common/enet_open.c.xdt",
            outputPath: "ti_enet_open_close.c",
            alwaysRun: true,
        },
        {
            name: "/networking/common/enet_open.h.xdt",
            outputPath: "ti_enet_open_close.h",
            alwaysRun: true,
        },
        {
            name: "/networking/common/enet_soc.c.xdt",
            outputPath: "ti_enet_soc.c",
            alwaysRun: true,
        },
        {
            name: "/networking/common/enet_lwipif.c.xdt",
            outputPath: "ti_enet_lwipif.c",
            alwaysRun: true,
        },
        {
            name: "/networking/common/enet_lwipif.h.xdt",
            outputPath: "ti_enet_lwipif.h",
            alwaysRun: true,
        },
    ],
    topModules: soc.getTopModules(),
};
