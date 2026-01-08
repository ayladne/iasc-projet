package com.drones.model;

import com.drones.config.SimulationParams;
import java.util.*;

public class Environment {
    private int width, height;
    private double[][] anomalyIntensity; // grid of anomaly intensity
    private List<Anomaly> anomalies;
    private Random random;
    private long elapsedTime;
    
    public Environment(int width, int height) {
        this.width = width;
        this.height = height;
        this.anomalyIntensity = new double[height][width];
        this.anomalies = new ArrayList<>();
        this.random = new Random(System.currentTimeMillis());
        this.elapsedTime = 0;
    }
    
    public int getWidth() { return width; }
    public int getHeight() { return height; }
    public double[][] getAnomalyIntensity() { return anomalyIntensity; }
    public List<Anomaly> getAnomalies() { return anomalies; }
    public long getElapsedTime() { return elapsedTime; }
    
    // Get anomaly intensity at position (with interpolation)
    public double getAnomalyAt(double x, double y) {
        int ix = (int) Math.floor(x);
        int iy = (int) Math.floor(y);
        
        if (ix < 0 || ix >= width || iy < 0 || iy >= height) {
            return 0;
        }
        
        return anomalyIntensity[iy][ix];
    }
    
    // Update environment (spawn, diffuse, decay)
    public void update(long tickDurationMs) {
        elapsedTime += tickDurationMs;
        
        // Step 1: Spawn new anomalies randomly
        spawnAnomalies();
        
        // Step 2: Decay and diffuse anomalies
        decayAndDiffuse();
        
        // Step 3: Remove dead anomalies
        anomalies.removeIf(a -> !a.isAlive());
    }
    
    private void spawnAnomalies() {
        if (random.nextDouble() < SimulationParams.ANOMALY_SPAWN_PROBABILITY) {
            int x = random.nextInt(width);
            int y = random.nextInt(height);
            double intensity = 0.5 + random.nextDouble() * 0.5; // 0.5-1.0
            anomalies.add(new Anomaly(x, y, intensity, elapsedTime));
        }
    }
    
    private void decayAndDiffuse() {
        // Clear grid
        for (int i = 0; i < height; i++) {
            for (int j = 0; j < width; j++) {
                anomalyIntensity[i][j] = 0;
            }
        }
        
        // Rebuild grid from anomalies
        for (Anomaly a : anomalies) {
            int ix = (int) Math.round(a.getX());
            int iy = (int) Math.round(a.getY());
            
            if (ix >= 0 && ix < width && iy >= 0 && iy < height) {
                anomalyIntensity[iy][ix] += a.getIntensity();
            }
            
            // Decay
            a.decay(SimulationParams.ANOMALY_DECAY_RATE);
        }
        
        // Diffuse to neighbors
        double[][] newIntensity = new double[height][width];
        for (int i = 0; i < height; i++) {
            for (int j = 0; j < width; j++) {
                newIntensity[i][j] = anomalyIntensity[i][j];
            }
        }
        
        double diffusion = SimulationParams.ANOMALY_DIFFUSION_FACTOR;
        for (int i = 0; i < height; i++) {
            for (int j = 0; j < width; j++) {
                if (anomalyIntensity[i][j] > 0) {
                    // Spread to neighbors
                    for (int di = -1; di <= 1; di++) {
                        for (int dj = -1; dj <= 1; dj++) {
                            if (di == 0 && dj == 0) continue;
                            int ni = i + di;
                            int nj = j + dj;
                            if (ni >= 0 && ni < height && nj >= 0 && nj < width) {
                                double spread = anomalyIntensity[i][j] * diffusion / 8.0;
                                newIntensity[ni][nj] += spread;
                            }
                        }
                    }
                }
            }
        }
        
        // Clamp and copy back
        for (int i = 0; i < height; i++) {
            for (int j = 0; j < width; j++) {
                anomalyIntensity[i][j] = Math.min(1.0, newIntensity[i][j]);
            }
        }
    }
    
    // Clear environment
    public void reset() {
        anomalies.clear();
        for (int i = 0; i < height; i++) {
            for (int j = 0; j < width; j++) {
                anomalyIntensity[i][j] = 0;
            }
        }
        elapsedTime = 0;
    }
}
