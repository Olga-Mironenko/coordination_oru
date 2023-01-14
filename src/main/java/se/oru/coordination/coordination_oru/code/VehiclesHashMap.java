package se.oru.coordination.coordination_oru.code;

import java.util.HashMap;

public class VehiclesHashMap {
    private static VehiclesHashMap instance;
    private int key;
    private static HashMap<Integer, AbstractVehicle> list = new HashMap<>();
    private static final Object lock = new Object();

    private VehiclesHashMap() {}

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



    public HashMap<Integer, AbstractVehicle> getList() {
        return list;
    }
}

