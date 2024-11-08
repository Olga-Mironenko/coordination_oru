package se.oru.coordination.coordination_oru.code;

import java.util.HashMap;

public class VehiclesHashMap {
    private static HashMap<Integer, AbstractVehicle> map = new HashMap<>();
    
    private VehiclesHashMap() {}

    public static HashMap<Integer, AbstractVehicle> getList() {
        return map;
    }

    public static AbstractVehicle getVehicle(int key) {
        return getList().get(key);
    }

    public static boolean isHuman(int key) {
        return getVehicle(key) instanceof HumanDrivenVehicle;
    }

    public static HumanDrivenVehicle getTheHuman() {
        HumanDrivenVehicle humanDrivenVehicle = null;
        for (AbstractVehicle vehicle : map.values()) {
            if (vehicle instanceof HumanDrivenVehicle) {
                assert humanDrivenVehicle == null;
                humanDrivenVehicle = (HumanDrivenVehicle) vehicle;
            }
        }
        assert humanDrivenVehicle != null;
        return humanDrivenVehicle;
    }
}

