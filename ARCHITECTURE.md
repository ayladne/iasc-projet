# Architecture Technique - Système Autonome de Drones

## 1. Vue d'Ensemble

```
┌─────────────────────────────────────────┐
│        Application JavaFX (UI)          │
│  MainApp - Canvas + Controls + Logging  │
└──────────────┬──────────────────────────┘
               │
        ┌──────▼──────┐
        │  Simulation │  (SimulationEngine)
        │   Engine    │
        └──────┬──────┘
               │
      ┌────────┴────────┐
      │                 │
 ┌────▼────┐      ┌────▼─────┐
 │   Model │      │  Control  │
 │ Classes │      │ (Logic)   │
 └─────────┘      └───────────┘
      │                 │
   Drone, Env,     Coordinator,
  Anomaly, etc.   PathPlanning
```

## 2. Packages et Classes

### `com.drones.model`
Entités et états du système

- **`Drone`** (classe)
  - Attributs : id, x, y, state, autonomyRemaining, measurements[], waypoints
  - Méthodes : update(), startMeasurement(), addMeasurement(), setWaypoints()
  - **États** : ACTIVE, MEASURING, RETURNING, CHARGING

- **`DroneState`** (enum)
  - ACTIVE : exploration en cours
  - MEASURING : prise de mesure (10 s)
  - RETURNING : retour à la base
  - CHARGING : recharge à la base (10 min)

- **`Environment`** (classe)
  - Attributs : width, height, anomalyIntensity[][], anomalies[]
  - Méthodes : update(), getAnomalyAt(), spawnAnomalies(), decayAndDiffuse(), reset()
  - Gère la grille, les anomalies, la dynamique

- **`Anomaly`** (classe)
  - Attributs : x, y, intensity, maxIntensity, creationTime
  - Méthodes : decay(), isAlive(), getIntensity()
  - Représente une source d'anomalie unique

- **`Drone.Measurement`** (classe interne)
  - Enregistrement : intensity, timestamp, x, y
  - Données collectées lors d'une mesure

### `com.drones.control`
Logique et orchestration de la simulation

- **`SimulationEngine`** (classe, modèle State)
  - Attributs : environment, drones[], simulationTime, running, metrics, coordinator
  - Méthodes : tick(), start(), stop(), reset(), getEnvironment(), getDrones()
  - **Boucle principale** : tick() met à jour env + drones chaque 200 ms
  - **Métriques** : coveragePercentage, anomaliesDetected, activeDrones, etc.

- **`Coordinator`** (classe)
  - Attributs : visitedCells, droneWaypoints
  - Méthodes : generateCoveragePlan(), adaptiveRetasking()
  - Gère la stratégie de couverture et la réaffectation d'urgence

### `com.drones.config`
Configuration et paramètres

- **`SimulationParams`** (classe utilitaire)
  - Constantes : GRID_WIDTH, GRID_HEIGHT, TICK_DURATION_MS, NUM_DRONES
  - Autonomie : DRONE_AUTONOMY_MS, DRONE_RECHARGE_MS, MEASUREMENT_DURATION_MS
  - Anomalies : ANOMALY_SPAWN_PROBABILITY, ANOMALY_DIFFUSION_FACTOR, ANOMALY_DECAY_RATE
  - Détection : ANOMALY_DETECTION_THRESHOLD

- **`SimulationScenario`** (enum)
  - Scénarios pré-configurés : NO_ANOMALIES, SPARSE, NORMAL, HEAVY, RAPIDLY_SPREADING
  - Chacun avec ses paramètres

### `com.drones.metrics`
Collecte et export de données

- **`MetricsCollector`** (classe)
  - Crée des snapshots toutes les 5 secondes
  - **MetricsSnapshot** : time, coverage, anomalies, activeDrones, etc.

- **`ExportUtils`** (classe utilitaire)
  - exportMetricsToCSV() : export time-series pour graphes
  - exportMeasurementsToCSV() : export des mesures par drone

### `com.drones.ui`
Interface utilisateur

- **`MainApp`** (classe Application JavaFX)
  - Layout : BorderPane (center=Canvas, right=Controls, bottom=Log)
  - Render : affiche grille, heatmap, drones
  - Controls : Start, Stop, Reset buttons
  - Animation : AnimationTimer chaque 200 ms pour render
  - Log : TextArea pour événements

## 3. Cycle de Simulation (Tick)

```
Chaque tick (200 ms) :

1. Environment.update(tickDuration)
   ├─ Spawn aléatoire anomalies
   ├─ Decay intensité anomalies * 0.95
   └─ Diffuse vers voisins

2. Pour chaque Drone :
   a. Drone.update(tickDuration)
      ├─ Switch state
      ├─ Move vers waypoint
      ├─ Consume autonomy
      └─ Check return-to-base condition
   
   b. Si state == ACTIVE et anomalie > threshold :
      └─ Drone.addMeasurement()

3. Coordinator.adaptiveRetasking() (chaque 30 ticks)
   └─ Réassigner drones vers hotspots

4. Metrics.update()
   └─ Calculer couverture, count anomalies, etc.

5. UI.render()
   ├─ Draw grid + heatmap
   ├─ Draw drones + labels
   └─ Update metricsLabel
```

## 4. État d'un Drone - Machine d'État

```
                    ┌──────────────────────┐
                    │      ACTIVE          │
                    │  (En exploration)    │
                    └──────┬───────────────┘
                           │
        ┌──────────────────┴──────────────────┐
        │                                     │
     Autonomy                          Waypoint
     épuisée?                         atteint?
        │                                     │
        │ OUI                          NO → Continue
        │                                    mouvement
        │
    ┌───▼──────────────────┐
    │    RETURNING         │
    │ (Retour base x,y=0) │
    └───┬──────────────────┘
        │
   Arrivé à base?
        │ OUI
        │
    ┌───▼──────────────────┐
    │    CHARGING          │
    │  (10 min recharge)   │
    └───┬──────────────────┘
        │
  Timer recharge ≤ 0?
        │ OUI
        │
     ┌──▼──────────────────────┐
     │ MEASURING (optionnel)    │
     │ (10 sec measurement)     │
     └───┬──────────────────────┘
         │
    Timer ≤ 0? → Retour à ACTIVE
```

**Transitions Implicites** :
- ACTIVE → MEASURING : si anomalie > 0.3, appel `startMeasurement()`
- MEASURING → ACTIVE : après 10 s

## 5. Modèle d'Anomalies

### Apparition
```
À chaque tick :
  if random() < ANOMALY_SPAWN_PROBABILITY (0.05) :
    créer nouvelle Anomaly(x_aléatoire, y_aléatoire, intensity_aléatoire)
```

### Évolution
```
Pour chaque cell (x, y) :
  1. Décroissance : intensity *= ANOMALY_DECAY_RATE (0.95)
  2. Diffusion : spread vers voisins
     nouvelle_intensity = sum(neighbors * diffusionFactor / 8)
  3. Clamp : intensity = min(1.0, intensity)
```

### Détection
```
Drone detect anomaly si :
  intensity_at_position > ANOMALY_DETECTION_THRESHOLD (0.3)
  
Mesure avec bruit : measured = real_intensity + gauss(-0.05, 0.05)
```

## 6. Stratégie de Couverture

### Pattern Initial (Lawnmower)
1. Partitionner grille 50×50 en N régions (N = NUM_DRONES = 7)
2. Chaque drone assigé une région
3. Pattern : scan horizontal alternant (left-to-right, right-to-left)
4. Puis retour à base (0,0)

```
Drone 0      Drone 1      Drone 2
┌──────┐    ┌──────┐    ┌──────┐
│1→ 2→ │    │1→ 2→ │    │1→ 2→ │
│← ← ← │    │← ← ← │    │← ← ← │
│3→ 4→ │    │3→ 4→ │    │3→ 4→ │
└──────┘    └──────┘    └──────┘
```

### Réaffectation Adaptative
Toutes les 6 secondes (30 ticks) :
1. Identifier hotspots : cells avec anomaly > 0.7
2. Pour chaque hotspot sans drone proche (< 5 cells) :
3. Assigner drone ACTIVE le plus proche
4. Créer waypoints d'urgence vers hotspot
5. Puis retour à base

## 7. Communication

### Uplink (Drone → Centre)
- **Toujours possible** : mesure uploadée en temps réel
- **Données** : intensity, timestamp, x, y
- Latency : instantanée (simulation)

### Downlink (Centre → Drone)
- **Uniquement à la base** : téléchargement carte globale
- Permet re-planification après recharge
- Latency : instantanée

### Broadcast
- État global à tous les drones : non implémenté (extension)
- Coordination locale : via `Coordinator` centralisé

## 8. Performance & Complexité

### Complexité Temporelle
- **tick()** : O(NUM_DRONES + GRID_WIDTH * GRID_HEIGHT)
  - Update drones : O(NUM_DRONES) = O(7)
  - Update environment : O(WIDTH * HEIGHT) = O(2500) + diffusion locale
  - Coordinator retasking : O(NUM_DRONES)
  - **Total : O(2500)** par tick (acceptable pour 200 ms)

### Mémoire
- Grille 50×50 floats : ~10 KB
- 7 drones × ~5 KB : ~35 KB
- Measurements buffer : ~1-10 MB (selon durée simulation)

### Scalabilité
- ✅ Jusqu'à 50×50 grille sans problème
- ⚠️ 100×100 : à tester
- ❌ Temps réel sur mille drones : non

## 9. Extensibilité

### Points de Variation
1. **Stratégie pathfinding** : remplacer `Drone.moveToward()` par RRT/A*
2. **Modèle anomalies** : ajouter types, propagation, décay variés
3. **Détection** : modèle probabiliste, apprentissage
4. **Communication** : délai réseau, perte de messages, broadcast
5. **Obstacles** : marquer cells non-passables, A* planning
6. **Visualisation 3D** : JavaFX 3D ou OpenGL

### Dépendances Minimales
- Aucune dépendance externe (hors JavaFX)
- Pas de framework lourd
- Code modulaire : facile à tester unitairement

## 10. Limites et Améliorations

### Limitations Actuelles
- ❌ Pas d'anticollision
- ❌ Pathfinding naïf (waypoints pré-calculés)
- ❌ Pas de communication inter-drone
- ❌ Pattern de couverture statique
- ❌ Pas d'obstacles

### Améliorations Recommandées
- ✅ Détection de collision simple (réserver cellule/tick)
- ✅ Pathfinding dynamique (A* vers hotspots)
- ✅ Échange d'info locale (drones proches)
- ✅ Apprentissage : ajuster pattern selon anomalies
- ✅ Simulation d'obstacles (zones interdites, bâtiments)

---

## Diagramme de Dépendances

```
┌─────────────────┐
│      UI         │
│   MainApp       │ (dépend)
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│  SimulationEngine│ (orchestrage)
└────┬────────┬───┘
     │        │
     ▼        ▼
┌────────┐ ┌─────────────┐
│ Model  │ │  Coordinator│ (stratégie)
│classes │ │  + Metrics  │
└────────┘ └─────────────┘
     │
     └─► Config, Utils

Flot de dépendance : Config ← Model ← Control ← UI
                     └─────────► Metrics ◄─────┘
```

---

Voir `README.md` pour le guide utilisateur.
Voir `GUIDE_UTILISATION.md` pour les scénarios de test.
