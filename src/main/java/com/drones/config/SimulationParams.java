package com.drones.config;

public class SimulationParams {
    // Grid dimensions
    public static final int GRID_WIDTH = 50;
    public static final int GRID_HEIGHT = 50;
    
    // Simulation timing (ms)
    public static final int TICK_DURATION_MS = 200;
    
    // Drone parameters
    public static final int NUM_DRONES = 7;
    public static final double DRONE_SPEED = 2.0; // cells per tick
    public static final int DRONE_AUTONOMY_MS = 30 * 60 * 1000; // 30 minutes
    public static final int DRONE_RECHARGE_MS = 10 * 60 * 1000; // 10 minutes
    public static final int MEASUREMENT_DURATION_MS = 10 * 1000; // 10 seconds
    
    // Anomaly parameters
    public static final double ANOMALY_SPAWN_PROBABILITY = 0.05; // 5% per tick
    public static final double ANOMALY_DIFFUSION_FACTOR = 0.1; // spread to neighbors
    public static final double ANOMALY_DECAY_RATE = 0.95; // intensity *= 0.95 per tick
    public static final double ANOMALY_DETECTION_THRESHOLD = 0.3;
    
    // UI scaling
    public static final int CELL_SIZE_PX = 12; // pixels per cell
    
    private SimulationParams() {
        // No instantiation
    }
}
