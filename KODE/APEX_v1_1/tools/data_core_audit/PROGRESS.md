# Cœur de données (circular_buffer / data_topic / data_packet) : suivi des correctifs de l'audit

> **Fichier de reprise.** Si la session s'interrompt, une nouvelle session lit ce fichier en entier,
> puis reprend à la première case non cochée de la section « Plan ». Le journal, en bas, date chaque étape.
> Règle : on ne coche une case qu'avec une **preuve** (sortie hôte, log cible dans `logs/`, mutant tué).

## 0. Statut final (2026-10-08 21:30) : ✅ toutes les phases closes

- Tous les constats de l'audit du 05/10 sont traités, et quatre nouveaux constats (N1 à N4) ont été trouvés et corrigés. Chacun a d'abord été prouvé par un test rouge, puis validé vert sur cible.
- Sur cible : SEQ_CB 22/22, SEQ_DT 30/30, SEQ_DP 7/7, RTOS_DT_USB 77/77, RTOS_BMI088_USB 23/23, et le stress ×50 avec 10 min d'endurance PASS. Le mutant sans verrou fait tomber tous les cas de concurrence. Sur hôte, 13 mutants de gardes sont tous tués (`logs/2026-10-08_host_final_mutants.txt`), et toute la logique pure passe (`logs/2026-10-08_host_final.txt`, seuls les 6 cas ISR échouent, ce qui est attendu sur PC).
- Rien n'est commité : les changements sont dans l'arbre de travail de la branche `Config`. Liste des fichiers : `git status`.
- Points ouverts (décision de l'utilisateur) :
  1. Coût de notification du registre (+2,5 à 4 µs par publish en -O0). Optimisation possible.
  2. Les missions RTOS ne compilent pas (API W25Q, framework de tâches), mais leurs detach sont corrigés.
  3. `configCHECK_FOR_STACK_OVERFLOW` est toujours désactivé (déjà signalé le 04/10, hors périmètre).

## 1. Contexte

- **Demande (2026-10-08)** : appliquer tous les correctifs que l'audit a fait ressortir, signaler et traiter
  tout autre problème trouvé en route, suivre un plan strict et tracer l'avancement ici.
  « C'est le cœur de la gestion de données du firmware de vol : une erreur de la lib signe la mort de la mission. »
- **L'audit** a été fait le 2026-10-05 (session Claude `5892265a…`, rapport « Audit des tests circular_buffer /
  data_topic », statut proposé 🟡). Ses constats : B1 à B4, les manques de couverture §2 à §5 et le plan de tests P1 à P3.
- **Première passe de correctifs** : même session, le 2026-10-05. Elle a été validée sur cible puis commitée dans
  `e01e8e0`. Elle s'est arrêtée sur une *limite de session*, au milieu d'un durcissement de la liste des abonnés.
  Ce durcissement est commité, mais il n'a **jamais été validé sur cible** (voir §3).

## 2. Outillage (ce dossier)

| Fichier | Rôle |
|---|---|
| `run_target.sh <PROJET> <timeout_s> <log>` | Sélectionne le projet dans la CMake, compile dans `build/audit`, flashe en SWD et lit le rapport USB sur COM3 → `logs/<log>.txt`. Si COM3 est tenu par un terminal, il bascule sur `../APEX_v0_2/COM3_*.txt`. `MARK=END_OF_REPORT` pour le stress. |
| `run_mutant.sh <PROJET> <timeout_s> <log>` | Vide `cb_critical_enter/exit` (mutant « sans verrou »), lance le projet, puis restaure le fichier quoi qu'il arrive. Tous les cas de concurrence doivent **échouer**. |
| `select_project.sh <PROJET>` | Active un seul projet dans `cmake/stm32cubemx/CMakeLists.txt`. |
| `host/build.sh [dt_src.c] [exe]` | Compile et exécute CB/DT/DP seq sur PC (gcc). Les cas ISR TIM5 échouent forcément sur hôte (« ISR TIM5 : 0 appels »). |
| `logs/` | Preuves : rapports cible horodatés, sorties hôte. `2026-10-05_stress_full_baseline.txt` = référence DWT. |

Matériel vérifié le 2026-10-08 : la carte répond en SWD (ST-LINK `B55B5A1A…`, STM32F411xE), et COM3 est libre.

## 3. État des constats de l'audit (au début de cette session)

| Id | Constat | État au 2026-10-08 | Preuve |
|---|---|---|---|
| B1 | Un publish avant `osKernelStart` bloque la carte (assert PRIGROUP dans `vPortValidateInterruptPriority`) | ✅ Corrigé : `dt_notify` sort si le scheduler n'a pas démarré. Hook `setup_pre_kernel()` ajouté, T27 RTOS | Vert sur cible le 10-05, stress ×50 |
| B2 | data_packet construit le paquet en place, dans le slot lisible | ✅ Corrigé : buffer `staging` + lectures par copie. Suite DP de 6 cas | Rouge prouvé sur l'ancien code (525/1495 lectures en retard), vert sur cible |
| B3 | `attach` ne valide pas `mode` | ✅ Corrigé (DT seq T22) | Vert sur cible |
| B4 | Message de T15 RTOS trompeur | ✅ (`dt0` séparé de `dt`) | Relu dans le code |
| §2 | CB : cap 1, plusieurs écrasements, sortie de l'état plein, canaris, `_ptr`, extrêmes de `cb_wrap`, NULL, push en place, test aléatoire, ISR PRIMASK | ✅ CB T11 à T21 (22 cas) | 22/22 sur cible |
| §2 mineur | Tests CB : lecture directe de `cb.count`, codes de retour non vérifiés (T5 à T8) | ❌ À faire (P2.3) | — |
| §3 | DT seq : `lag == cap+1`, perte via `peek(idx>0)` et `_ptr`, peek pendant un retard, detach au milieu, wrap de `pub_seq`, FROM_OLDEST après plusieurs tours, cap 1, canaris | ✅ DT T14 à T22 | 25/25 sur cible |
| §4 | RTOS : mutation, T17 avec ISR, churn face à une ISR, `wait(osWaitForever)`, pré-noyau, répétition, endurance, DWT | ✅ T17 renforcé, T27 à T30, RTOS_DT_STRESS | 70/70, mutant rouge, stress PASS (`logs/2026-10-05_stress_full_baseline.txt`) |
| §5 | Build séquentiel sans ISR concurrente | ✅ CB T20/T21, DT T23/T24 (PRIMASK) | Vert sur cible, mutant rouge |
| Rappel | Relancer la suite RTOS BMI088 (`DT_DATA_LOSS` remonte maintenant réellement) | ❌ À faire (P4.1) | — |
| Session | Les tâches mission ne détachent pas leurs abonnés de pile | ❌ À traiter (P4.2). Ces projets ne compilent plus, pour d'autres raisons (API W25Q, ancien framework de tâches) | grep du 10-05 |
| Session | Durcissement liste : `list_faults`, `dt_sub_sane/member`, safe-unlink, T31 RTOS, T25 seq | ⚠️ Commité, **jamais passé sur cible**. Mutant `next_prev_check` survivant. T26 prévu mais pas écrit | Transcript du 10-05 |

## 4. Nouveaux constats (cette session)

| Id | Gravité | Constat | Décision |
|---|---|---|---|
| N1 | 🟠 | **`sub_count` peut descendre sous le nombre réel d'abonnés** (code de durcissement en vol). Scénario d'erreur d'usage : topic ré-initialisé avec deux abonnés encore attachés (O2→O1), puis attache d'un nouvel abonné F. Le detach de O1 avant O2 trouve `prev=O2` membre avec `O2->next==O1`, donc `in_list=true` et `sub_count--` passe à 0 alors que F est dans la liste. Ensuite `dt_linked_locked` voit `n=1 > sub_count=0` et **refuse toute nouvelle attache** sur ce topic. Cause : « est dans la liste » est décidé localement (liens du voisin), pas par joignabilité depuis la tête. | Passer à un unlink **par parcours depuis la tête** (liste simplement chaînée). On n'écrit que dans le prédécesseur trouvé (membre) ou dans la tête, jamais dans un successeur, et on supprime `prev`. `sub_count` ne baisse que si le nœud a été trouvé. Test de régression dans les deux ordres. |
| N2 | 🟡 | `dt_notify` parcourt la liste **sans borne** (scheduler suspendu, ou en ISR). La terminaison repose sur un raisonnement d'acyclicité, sans borne explicite (règle « toute boucle bornée »). | Borner tous les parcours par `sub_count`. Après N1, l'invariant `sub_count ≥ longueur joignable` tient, donc la borne ne saute jamais un abonné légitime. |
| N3 | 🟡 | **Repli de `pub_seq` (2³²)** : un abonné qui ne lit pas pendant ≥ 2³² publications voit un `lag` replié. Il peut alors lire des données dans le désordre avec DT_OK au lieu de DT_DATA_LOSS (1 kHz : 49,7 jours ; 20 kHz : 2,5 jours). | Contrôle de cohérence du curseur dans `dt_access`. Invariant : `sub->tail == (head − lag) mod cap`. Une incohérence est traitée comme une perte (recalage + DT_DATA_LOSS). La limite résiduelle est documentée (cap puissance de 2 : seul le drapeau de perte manque, l'ordre est bon). |
| N4 | ⚪ | `data_packer_check` : la boucle de rejet des échantillons trop vieux n'a pas de borne fixe. Exemple : un publieur à horodatage figé + une ISR rapide. | Borner (au plus `capacity + 1` rejets par appel). |

### Décision D1 (2026-10-08) : registre d'abonnés à taille fixe au lieu de la liste chaînée intrusive

N1 est **prouvé rouge sur hôte** : le nouveau cas DT seq T26 sort `sub_count=0 attendu 1 (fresh seul)`.
Le défaut est structurel. Dans une liste intrusive, les liens vivent dans la mémoire des abonnés. Un seul abonné
fautif (remis à zéro, mémoire réutilisée, rattaché ailleurs, topic ré-initialisé) coupe donc la chaîne pour tous
ceux qui le suivent. Chaque garde ajoutée (sane/member/back-pointer/in_list) multiplie les cas, et le code en vol
en avait déjà raté un.

**Remède : `data_sub_t *subs[DATA_TOPIC_MAX_SUBS]` dans le topic** (8 par défaut, redéfinissable). Les tests
utilisent au plus environ 6 abonnés par topic. Propriétés obtenues par construction :
- Un abonné incohérent n'affecte que lui-même. Il est ignoré, puis son slot est récupéré et compté une seule fois
  dans `list_faults`. Les autres abonnés sont toujours notifiés.
- Une opération sur le topic T n'écrit jamais dans un abonné qui n'est pas validement attaché à T. Seule exception :
  l'abonné passé explicitement par son propriétaire (attach/detach).
- Toutes les boucles ont une borne fixe (`DATA_TOPIC_MAX_SUBS`). Aucun cycle n'est possible et `sub_count` est
  exact (= slots occupés).
- Un abonné remis à zéro encore inscrit qui se ré-attache reprend son slot : pas de doublon, anomalie comptée.
  L'ancien comportement « refus » (T22) devient « reprise ».
- `prev`/`next` disparaissent de `data_sub_t` (8 o de moins par abonné), et le topic grossit de 32 o.
- Nouveau statut `DT_NO_SLOT` (registre plein), ajouté en fin d'enum : les valeurs existantes ne changent pas.
- Coût : la notification parcourt 8 slots (quelques dizaines de cycles), à vérifier au DWT en P3.4.

## 5. Plan (strict, dans l'ordre ; une phase n'est close que si son critère de sortie est prouvé)

### P1 : Registre des abonnés (remplace le durcissement en vol, voir D1)

- [x] P1.0 Preuve rouge de N1 : DT seq T26 « re-init fautive, detach dans l'ordre inverse » échoue sur `e01e8e0` (hôte).
- [x] P1.1 `data_topic.{h,c}` : registre fixe `subs[DATA_TOPIC_MAX_SUBS]`, `DT_NO_SLOT`, `prev`/`next` supprimés, slots invalides récupérés (attach, notify) et comptés une seule fois. La doc de l'en-tête suit. `data_packet.c` passe de `topic.subs != NULL` à `sub_count`.
- [x] P1.2 Tests adaptés au registre : helper seq `dts_list_is` (devient indépendant de l'ordre), seq T18/T22, RTOS T18/T19/T26/T29/T30/T31, et le contrôle d'invariants de `dt_rtos_stress.c`.
- [x] P1.3 Nouveaux cas seq : T27 « abonné remis à zéro et rattaché à un autre topic : aucun des deux registres abîmé, slot récupéré » et T28 « registre plein → DT_NO_SLOT, puis slot libéré réutilisable ».
- [x] P1.4 Hôte : suites seq PASS (hors cas ISR), T26 vert, et chaque garde du registre tuée par au moins un test (mutation hôte).
- **Sortie P1** : hôte vert, aucun mutant de garde survivant. ✅ (2026-10-08) Preuves : `logs/2026-10-08_host_P1_registry.txt` (tout PASS hors 5 cas ISR) et `logs/2026-10-08_host_final_mutants.txt` (outil `host/mutate_registry.py` : 13/13 mutants tués sur le code final, dont `cursor_unchecked`). La compilation cible RTOS_DT_USB est propre. Les gardes propres au RTOS (sémaphore, libération en notification tâche/ISR) seront validées sur cible par T31 (P3.2).

### P2 : Autres constats

- [x] P2.1 N3 : contrôle de cohérence du curseur dans `dt_access` + doc de la limite 2³². Test seq T29 (curseur incohérent → DT_DATA_LOSS + recalage, données correctes).
- [x] P2.2 N4 : borne de la boucle de rejet de `data_packer_check` + cas DP (horodatages figés : l'appel rend la main).
- [x] P2.3 Audit mineur : tests CB via `cb_count()`, et tous les codes de retour vérifiés.
- **Sortie P2** : hôte vert. ✅ (2026-10-08) `logs/2026-10-08_host_P2.txt` : tout PASS sauf 6 cas ISR (CB T20/T21, DT T23/T24, DP T5/T6), attendu sur PC.
  - N3 prouvé sur hôte : sans le contrôle, T29 lit **DT_OK sur un slot jamais écrit** (`out=0`). Le mutant `cursor_unchecked` est ajouté à l'outil de mutation.
  - N4 : la preuve rouge (blocage sans borne) se fera sur cible avec DP T6, à cause de l'ISR (P3.1).

### P3 : Validation sur cible (preuves dans `logs/`)

- [x] P3.1 SEQ_CB_USB, SEQ_DT_USB et SEQ_DP_USB : 100 % PASS.
- [x] P3.2 RTOS_DT_USB (suite RTOS, T31 compris, plus le rejeu des 3 suites seq) : 100 % PASS.
- [x] P3.3 Mutant sans verrou sur RTOS_DT_USB, SEQ_CB_USB, SEQ_DT_USB et SEQ_DP_USB : chaque cas de concurrence échoue.
  - Preuves P3.1 : `logs/2026-10-08_SEQ_CB_USB.txt` 22/22 (repassé après le bornage des vidanges de T20), `logs/2026-10-08_SEQ_DT_USB.txt` 30/30, `logs/2026-10-08_SEQ_DP_USB.txt` 7/7.
  - Preuve rouge N4 : `logs/2026-10-08_SEQ_DP_USB_nobound_variant.txt`. Sans la borne, DP T6 ne rend jamais la main (livelock ISR/rejet vu au PC).
  - Preuve P3.2 : `logs/2026-10-08_RTOS_DT_USB.txt` 77/77 (RTOS 18, DT 30, CB 22, DP 7). RTOS T22 et T23 ont été modifiés après ce passage (vidanges bornées, voir P3.3) : leur vert final est apporté par P3.4 (×50).
  - Preuves P3.3 : `logs/2026-10-08_mutant_*.txt`. Sur RTOS tombent T17, T20, T22, T23, **T29** (nouveau, grâce à `list_faults`) et T30, plus DT T23, CB T20/T21 et DP T5 rejoués. Sur les builds seq tombent CB T20/T21, DT T23 et DP T5. DT T24 (comptabilité) et DP T6 (borne) ne sont pas des détecteurs de verrou, comme le 05/10. Au premier passage, le mutant bloquait RTOS T23 (vidange sans borne sur `count` corrompu) : les vidanges des tests RTOS T23/T22 et CB T20 sont maintenant bornées par la capacité, ce qui est neutre avec le code réel.
- [x] P3.4 RTOS_DT_STRESS complet (×50, 10 min d'endurance, DWT) : PASS. Temps DWT comparés à la référence du 10-05.
  - Preuve P3.4 : `logs/2026-10-08_RTOS_DT_STRESS.txt`. Suites ×50 (609 s) à 0 échec (DT RTOS 18, DT seq 30, CB 22, DP 7). Endurance de 600 s PASS : 600 contrôles d'invariants, 0 violé, 0 corrompu, comptes lus + sautés exacts, rafale d'attache de 12000 cycles à 0 échec, data_packet à 0 incohérent. Piles ≥ 660 o libres sur 1024.
  - DWT (-O0) comparé au 05/10 : temps masqué de `data_sub_read` +0,45 µs (contrôle du curseur, modulo), max 17,6 µs à 256 o. `publish` médian 8,7→11,2 µs (0 abonné), 11,6→14,5 µs (1 abonné), 19,2→23,4 µs (4 abonnés) : la notification parcourt les 8 slots. Coût accepté. Optimisation possible si besoin (arrêt après `sub_count` slots occupés), non faite pour ne pas ajouter de risque.
- **Sortie P3** : tout vert, mutants rouges, stress PASS. ✅ (2026-10-08 21:20)

### P4 : Restes de l'audit et de la session précédente

- [x] P4.1 RTOS_BMI088_USB relancé (rappel de l'audit).
- [x] P4.2 Missions RTOS : `data_sub_detach` avant chaque sortie de tâche qui a attaché un abonné de pile. Le problème est signalé : ces projets ne compilent plus pour d'autres raisons.
- [x] P4.3 Fins de ligne : pas de fichier mixte CRLF/LF parmi les fichiers touchés.
- [x] P4.4 CLAUDE.md à jour (règles data_topic).
  - P4.2 : 10 `data_sub_detach(&sub)` ajoutés avant les `osThreadExit_Cstm()` qui suivent une attache, dans RTOS_FLIGHT (4), RTOS_LAUNCH_DETECTION (4) et RTOS_BMI088_W25Q_1 (2). **Non compilable** pour l'instant : ces missions ne compilent plus pour d'autres raisons (anciennes API W25Q et framework de tâches, constaté le 05/10). Les autres utilisateurs (BMI088 test/bench, SEQ_SEQ_UART) détachent déjà correctement.
  - P4.3 : aucun fichier mixte. Les missions restent en CRLF, le reste en LF (comme HEAD). `.gitignore` ajouté pour les artefacts de l'outillage.
  - P4.4 : CLAUDE.md décrit le registre (`DATA_TOPIC_MAX_SUBS`, `DT_NO_SLOT`, `list_faults`), le harnais hôte et les outils de mutation et de capture.
- [x] P4.5 Sélection CMake restaurée sur `RTOS_BMI088_USB`.
- [x] P4.6 Mémoire Claude à jour + rapport final à l'utilisateur.
  - P4.1 : `logs/2026-10-08_RTOS_BMI088_USB.txt` 23/23 PASS. Aucune perte signalée sur les topics acc/gyr/temp (T13, T15 à T17, T19 et T20 vérifient `!loss`).
  - P4.5 : la CMake active RTOS_BMI088_USB (`build/audit` sert aux essais, `build/Debug` de l'utilisateur n'a pas été touché).

## 6. Journal

- **2026-10-08 20:05** : reprise. Audit et transcript du 10-05 relus. État reconstruit (§3). Rapport du stress du 10-05 lu : PASS, sur le code *antérieur* au durcissement. Les trois librairies ont été relues en entière. Constats N1 à N4 ajoutés. Outillage installé dans ce dossier. Carte et COM3 vérifiés.
- **2026-10-08 ~20:20** : ligne de base hôte sur `e01e8e0` : tout PASS sauf les 5 cas ISR (attendu sur PC). T26 seq ajouté : **rouge** (`sub_count=0 attendu 1`), donc N1 est prouvé. Décision D1 (registre fixe). Plan P1 réécrit en conséquence.
- **2026-10-08 ~20:31** : P1 terminé.
  - `data_topic` passe au registre fixe. `attach` est simplifié : la branche « reprise de son propre slot » était redondante (mutant équivalent), la libération des slots invalides couvre ce cas.
  - Tests : seq T18/T22 (+ écrasement partiel)/T25 adaptés, T27 (abonné déplacé, free avec slot périmé) et T28 (registre plein) ajoutés. RTOS T19 (ordre des slots), T26/T29/T30 (registre exact et `list_faults==0`), T31 réécrit (2 fantômes, tâche + ISR). Contrôle d'invariants du stress adapté.
  - Outil de mutation hôte ajouté. Piège à connaître : sous Windows, `subprocess` lance le bash de WSL, il faut passer le chemin de Git Bash. Un plantage compte comme mutant tué.
- **2026-10-08 ~20:36** : P2 terminé.
  - `dt_access` vérifie l'invariant du curseur, la limite 2³² est documentée (« Limites » dans l'en-tête), test DT seq T29.
  - `data_packer_check` : au plus `capacity` rejets par appel, test DP T6 (flot ISR d'échantillons trop vieux, 100 kHz en séquentiel, 40 kHz sous RTOS).
  - Tests CB : `cb_count()` partout (sauf l'invariant ISR en boîte blanche), codes de retour T5 à T8 vérifiés.
  - Commentaire orphelin de l'ancien helper supprimé.
- **2026-10-08 ~21:00** : P3.1 à P3.3 faits.
  - Lecteur COM3 fiabilisé. Le rapport n'est émis qu'une fois, à DTR, et `TEST_usb_print` abandonne en ~100 ms : `run_target.sh` ouvre donc le port d'abord, puis fait un reset SWD, et le marqueur de fin est cherché sans les codes VT100.
  - Les heures du journal avant 21:00 sont approximatives.
- **2026-10-08 21:00** : stress lancé (flash 21:00:13, fin vers 21:25). En parallèle : P4.2, P4.3 et P4.4 faits (sources hors projet compilé).
- **2026-10-08 21:20** : stress PASS (P3.4). P3 clos.
- **2026-10-08 21:30** : RTOS_BMI088_USB 23/23 (P4.1), CMake sur RTOS_BMI088_USB (P4.5), mémoire à jour (P4.6). Toutes les phases sont closes.
