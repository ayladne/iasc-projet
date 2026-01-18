package com.drones.model;
import com.drones.config.SimulationParams;
import java.util.*;
public class Environment {
    private int width, height;
    private double[][] anomalyIntensity; // grille d'intensité des anomalies
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
    
    // Obtenir l'intensité des anomalies à une position (avec interpolation)
    public double getAnomalyAt(double x, double y) {
        int ix = (int) Math.floor(x);
        int iy = (int) Math.floor(y);
        
        if (ix < 0 || ix >= width || iy < 0 || iy >= height) {
            return 0;
        }
        
        return anomalyIntensity[iy][ix];
    }
    
    // Mettre à jour l'environnement (apparition, diffusion, décomposition)
    public void update(long tickDurationMs) {
        elapsedTime += tickDurationMs;
        
        // Étape 1: Créer de nouvelles anomalies aléatoirement
        spawnAnomalies();
        
        // Étape 2: Décomposer et diffuser les anomalies
        decayAndDiffuse();
        
        // Étape 3: Supprimer les anomalies mortes
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
        // Effacer la grille
        for (int i = 0; i < height; i++) {
            for (int j = 0; j < width; j++) {
                anomalyIntensity[i][j] = 0;
            }
        }
        
        // Reconstruire la grille à partir des anomalies
        for (Anomaly a : anomalies) {
            int ix = (int) Math.round(a.getX());
            int iy = (int) Math.round(a.getY());
            
            if (ix >= 0 && ix < width && iy >= 0 && iy < height) {
                anomalyIntensity[iy][ix] += a.getIntensity();
            }
            
            // Décomposer
            a.decay(SimulationParams.ANOMALY_DECAY_RATE);
        }
        
        // Diffuser vers les voisins
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
                    // Propager aux voisins
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
        
        // Restreindre et copier
        for (int i = 0; i < height; i++) {
            for (int j = 0; j < width; j++) {
                anomalyIntensity[i][j] = Math.min(1.0, newIntensity[i][j]);
            }
        }
    }
    
    // Effacer l'environnement
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
