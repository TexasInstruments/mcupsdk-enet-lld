
let common = system.getScript("/common");

const topModules_main = [
    "/networking/enet_cpsw/enet_cpsw",
];

const topModules_mcu = [
    "/networking/enet_cpsw/enet_cpsw",
];


const driverVer = {
    "enet_cpsw": {
        version: "j722s",
    },
}

exports = {
    getTopModules: function() {

        let topModules = topModules_main;

        if(common.getSelfSysCfgCoreName().includes("m4f")) {
            topModules = topModules_mcu;
        }

        return topModules;
    },
    getDriverVer: function(driverName) {
        return driverVer[driverName].version;
    },
    getDirName: function(driverName) {
        return driverVer[driverName].version;
    },
};
