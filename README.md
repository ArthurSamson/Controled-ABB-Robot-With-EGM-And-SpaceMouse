# Contrôle simple ABB EGM (PC C++ + RAPID)

Ce dépôt contient un exemple minimal pour piloter la tête d'outil (TCP) d'un ABB via EGM:

- **PC C++**: réception `EgmRobot` + envoi `EgmSensor` sur UDP (`egm_simple_move.cpp`)
- **Contrôleur RAPID**: activation EGM mode pose sur IRB 1300 / C30 (`rapid/EGM_IRB1300_C30.mod`)

---

## 1) Configuration RobotStudio (déjà fournie)

Communication Device `UDPUC`:

- `Name`: `EGM_1`
- `Type`: `UDPUC`
- `Remote Address`: `127.0.0.1`
- `Remote Port Number`: `6511`
- `Local Port Number`: `0`

> Avec cette config, le **PC doit écouter sur le port UDP 6511**.

---

## 2) Côté RAPID (IRB 1300 / C30)

Fichier: `rapid/EGM_IRB1300_C30.mod`

Ce module:

1. met le robot en position de départ,
2. lance `EGMSetupUC` sur le device `EGM_1`,
3. active `EGMActPose`,
4. exécute `EGMRunPose` pendant une durée donnée,
5. fait `EGMReset` à la fin.

⚠️ Selon votre version exacte RobotWare/C30, certains paramètres d'`EGMActPose`/`EGMSetupUC` peuvent légèrement varier. Gardez la structure et adaptez la signature si RobotStudio demande une variante.

---

## 3) Côté PC C++ (`egm_simple_move.cpp`)

Le programme:

1. ouvre un socket UDP,
2. attend les paquets EGM du contrôleur,
3. lit la pose cartésienne actuelle,
4. renvoie une pose corrigée (offset XYZ + orientation RPY).

### Compilation (exemple Linux)

```bash
g++ -std=c++17 -O2 egm_simple_move.cpp egm.pb.cc -lprotobuf -lpthread -o egm_simple_move
```

### Exécution avec votre config UDPUC

```bash
./egm_simple_move 6511
```

Exemple avec offset:

```bash
./egm_simple_move 6511 10 0 0 0 0 5
```

Interprétation:

- `6511` = port UDP local (aligné avec `Remote Port Number` du contrôleur)
- `10 0 0` = +10 mm en X
- `0 0 5` = roll=0°, pitch=0°, yaw=5°

---

## 4) Comment obtenir `egm.pb.h` / `egm.pb.cc` (partie PC)

Il faut générer les fichiers C++ à partir de `egm.proto` avec `protoc`.

### Étapes

1. Installer protobuf compiler et dev C++.
2. Récupérer `egm.proto` (depuis vos exemples ABB EGM / package EGM ABB installé).
3. Générer les sources C++:

```bash
protoc --cpp_out=. egm.proto
```

Cela crée:

- `egm.pb.h`
- `egm.pb.cc`

4. Compiler votre app avec ces fichiers:

```bash
g++ -std=c++17 -O2 egm_simple_move.cpp egm.pb.cc -lprotobuf -lpthread -o egm_simple_move
```

### Installation rapide protobuf (Ubuntu/Debian)

```bash
sudo apt update
sudo apt install -y protobuf-compiler libprotobuf-dev
```

Vérification:

```bash
protoc --version
```

---

## 5) Sécurité / production

Cet exemple est volontairement minimal. Pour une cellule réelle, ajouter:

- limites workspace strictes,
- limitation vitesse/accélération,
- watchdog communication,
- validation état robot/automate,
- stratégie de repli sûre en cas de perte réseau.
