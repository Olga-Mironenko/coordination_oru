package se.oru.coordination.coordination_oru.code;

import java.util.HashMap;

public class VehiclesHashMap {
    private static final Object lock = new Object();
    private static VehiclesHashMap instance;
    private static HashMap<Integer, AbstractVehicle> list = new HashMap<>();

    private VehiclesHashMap() {
    }

    public static VehiclesHashMap getInstance() {
        if (instance == null) {
            synchronized (lock) {
                if (instance == null) {
                    instance = new VehiclesHashMap();
                }
            }
        }
        return instance;
    }

    public static AbstractVehicle getVehicle(int key) {
        if (instance == null) {
            synchronized (lock) {
                if (instance == null) {
                    instance = new VehiclesHashMap();
                }
            }
        }
        return instance.getList().get(key);
    }

    public synchronized static HashMap<Integer, AbstractVehicle> getList() {
        return list;
    }
}

