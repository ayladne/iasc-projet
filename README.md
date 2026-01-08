# Autonomous Drone Swarm Simulator

Système de simulation autonome pour un essaim de 7 drones coopératifs effectuant la surveillance d'environnements sensibles.

## Architecture

- **Modèles** (`com.drones.model`) :
  - `Drone` : agent autonome avec état (actif, mesure, retour base, recharge), énergie limitée
  - `Environment` : grille 2D modélisant les anomalies (apparition, diffusion, décroissance)
  - `Anomaly` : source d'anomalie avec intensité variable

- **Contrôle** (`com.drones.control`) :
  - `SimulationEngine` : boucle principale de simulation, gestion des états drones et environnement
  - Stratégie de couverture : balayage en raster (lawnmower) partitionné

- **UI** (`com.drones.ui`) :
  - `MainApp` : interface JavaFX avec canvas pour visualiser la grille, positions drones, heatmap d'anomalies
  - Panneau de contrôle (Start/Stop/Reset)
  - Zone de log en temps réel
  - Métriques : couverture, anomalies détectées, drones actifs/recharge

- **Métriques** (`com.drones.metrics`) :
  - `MetricsCollector` : snapshots périodiques de l'état de la simulation

## Paramètres

Tous les paramètres sont définis dans `SimulationParams` :
- Grille : 50×50 cellules
- Nombre de drones : 7
- Autonomie drone : 30 minutes
- Recharge à la base : 10 minutes
- Durée mesure : 10 secondes
- Vitesse tick : 200 ms

## Compilation et Exécution

### Compilation
```bash
mvn clean compile
```

### Exécution (JavaFX)
```bash
mvn javafx:run
```

### Compilation standalone (JAR)
```bash
mvn clean package
java -jar target/swarm-simulator-1.0-SNAPSHOT.jar
```

## Utilisation

1. Cliquer **Start** pour lancer la simulation
2. Observer :
   - Grille avec heatmap rouge/orange/jaune (anomalies)
   - Drones colorés selon l'état :
     - **Vert** : Actif (en exploration)
     - **Bleu** : En mesure (10 s)
     - **Orange** : Retour à la base
     - **Rouge** : Recharge
   - Base à l'origine (0,0) en vert foncé
3. Métriques mises à jour en temps réel
4. Cliquer **Stop** pour pausé, **Reset** pour recommencer

## Scénarios à Tester

1. **Couverture complète** : Observer comment les 7 drones couvrent la zone en balayage systématique
2. **Anomalies sporadiques** : Vérifier que les drones détectent et rapportent les anomalies
3. **Gestion énergétique** : Observer les retours à la base et recharge, impact sur la couverture
4. **Dynamique anomalies** : Voir la propagation et décroissance des anomalies au fil du temps

## Améliorations Futures

- [ ] Stratégies de couverture adaptatives (allocation dynamique en fonction des anomalies)
- [ ] Communication entre drones (partage d'informations locales)
- [ ] Obstacles et zones interdites
- [ ] Visualisation 3D
- [ ] Export de métriques (CSV/graphes)
- [ ] Tests unitaires complets
- [ ] Optimisation avec algorithmes de planification (RRT, PRM)

## Structure des Packages

```
com.drones/
├── config/        SimulationParams
├── model/         Drone, Environment, Anomaly, DroneState
├── control/       SimulationEngine
├── metrics/       MetricsCollector
└── ui/            MainApp
```

## Notes

- La simulation est synchronisée à 200 ms par tick
- Les anomalies se propagent par diffusion locale et décroissent exponentiellement
- Les drones suivent un plan de waypoints déterministe au démarrage (pattern lawnmower)
- L'énergie est consommée pendant l'exploration et la mesure, pas pendant le déplacement (simplification)
- La communication drone-base est instantanée mais limitée (downlink seulement à la base)
