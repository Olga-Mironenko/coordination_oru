package se.oru.coordination.coordination_oru.code;

import java.util.Collection;
import java.util.HashMap;

public class VehiclesHashMap {
    private static final HashMap<Integer, AbstractVehicle> map = new HashMap<>();
    
    private VehiclesHashMap() {}

    public static HashMap<Integer, AbstractVehicle> getList() {
        return map;
    }

    public static AbstractVehicle getVehicle(int key) {
        return map.get(key);
    }

    public static boolean isHuman(int key) {
//        return getVehicle(key) instanceof HumanDrivenVehicle;
        return key == 0; // to speed up
    }

    public static Collection<AbstractVehicle> getVehicles() {
        return map.values();
    }

    public static HumanDrivenVehicle getTheHuman() {
        HumanDrivenVehicle humanDrivenVehicle = null;
        for (AbstractVehicle vehicle : getVehicles()) {
            if (vehicle instanceof HumanDrivenVehicle) {
                assert humanDrivenVehicle == null;
                humanDrivenVehicle = (HumanDrivenVehicle) vehicle;
            }
        }
        assert humanDrivenVehicle != null;
        return humanDrivenVehicle;
    }
}

