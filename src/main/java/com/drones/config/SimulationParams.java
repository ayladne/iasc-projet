package com.drones.config;

public class SimulationParams {
    // Dimensions de la grille
    public static final int GRID_WIDTH = 50;
    public static final int GRID_HEIGHT = 50;
    
    // Durée de simulation (ms)
    public static final int TICK_DURATION_MS = 200;
    
    // Paramètres des drones
    public static final int NUM_DRONES = 7;
    public static final double DRONE_SPEED = 2.0; // cellules par tick
    public static final int DRONE_AUTONOMY_MS = 30 * 60 * 1000; // 30 minutes
    public static final int DRONE_RECHARGE_MS = 10 * 60 * 1000; // 10 minutes
    public static final int MEASUREMENT_DURATION_MS = 10 * 1000; // 10 secondes
    
    // Paramètres des anomalies
    public static final double ANOMALY_SPAWN_PROBABILITY = 0.05; // 5% par tick
    public static final double ANOMALY_DIFFUSION_FACTOR = 0.1; // propagé aux voisins
    public static final double ANOMALY_DECAY_RATE = 0.95; // intensité *= 0.95 par tick
    public static final double ANOMALY_DETECTION_THRESHOLD = 0.3;
    
    // Échelle UI
    public static final int CELL_SIZE_PX = 12; // pixels par cellule
    
    private SimulationParams() {
        // Pas d'instanciation
    }
}
