package com.drones.model;

public class Anomaly {
    private double x, y;
    private double intensity; // 0 to 1
    private double maxIntensity;
    private long creationTime;
    
    public Anomaly(double x, double y, double intensity, long creationTime) {
        this.x = x;
        this.y = y;
        this.intensity = intensity;
        this.maxIntensity = intensity;
        this.creationTime = creationTime;
    }
    
    public double getX() { return x; }
    public double getY() { return y; }
    public double getIntensity() { return intensity; }
    public long getCreationTime() { return creationTime; }
    public double getMaxIntensity() { return maxIntensity; }
    
    public void setIntensity(double intensity) {
        this.intensity = Math.max(0, Math.min(1, intensity));
    }
    
    public void decay(double decayRate) {
        intensity *= decayRate;
    }
    
    public boolean isAlive() {
        return intensity > 0.01;
    }
}
