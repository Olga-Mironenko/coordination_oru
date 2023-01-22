import java.util.HashMap;

public class VehiclesHashMap<K, V> {
    private static VehiclesHaspMap instance;
    private HashMap<K, V> map;

    private VehiclesHaspMap() {
        map = new HashMap<>();
    }

    public static VehiclesHaspMap getInstance() {
        if (instance == null) {
            instance = new VehiclesHaspMap();
        }
        return instance;
    }

    public void put(K key, V value) {
        map.put(key, value);
    }

    public V get(K key) {
        return map.get(key);
    }
}
