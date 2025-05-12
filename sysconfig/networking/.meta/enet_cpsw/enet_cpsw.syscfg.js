
let common = system.getScript("/common");
let soc = system.getScript(`/networking/soc/networking_${common.getSocName()}`);

function getModule() {

    let driverVer = soc.getDriverVer("enet_cpsw");

    if (driverVer == "am62dx_am62ax_am62px")
    {
        return system.getScript(`/networking/enet_cpsw/${driverVer}/enet_cpsw_top`);
    }
    else if(driverVer == "am62lx"){
        
        return system.getScript(`/networking/enet_cpsw/${driverVer}/enet_cpsw_top`);
    }
    else
    {
        return system.getScript(`/networking/enet_cpsw/${driverVer}/enet_cpsw_${driverVer}`);
    }

}

exports = getModule();
