// Fichier "StationMeteo.h : Defines du Projet Station Météo.
// Pour IDE Arduino version 1.8.5

// ************************************
// *  Configuration de l'IDE Arduino  *
// ************************************

// Copier "Arduino_STM32" (Archive fournie) dans "C:/Program Files (x86)/Arduino/hardware".
// Board Manager : STM32 Cores by STMicroelectronics version 1.4.0 (https://github.com/stm32duino/BoardManagerFiles/raw/master/STM32/package_stm_index.json)
// (Ajouter cette URL dans "File/Preferences/Settings/Additional Boards Manager URLs")
// puis installer en allant dans "Tools/Board/Boards Manager..."
// NOTE : Cette implémentation du STM32 pour Arduino "Roger Clark Melbourne Maple Mini" est basée sur LibMaple...
//        Spécifique. Mais on n'utilise extrèmement peu cette librairie (2 ou 3 fonctions), pour le NVIC (Interruptions) et le DMA, trop "galère" sinon.

// Board : "Maple Mini" dans "Tools/Boards/STM32F1 Boards (STM32duino.com)". CPU Speed "72MHz (Normal)".
// Bootloader version : "Bootloader 2.0 (20k RAM/120k Flash)" (Bootloader à programmer par ST-Link avec une carte Nucleo via le connecteur ICSP - Câble spécifique Tag-Connect)
// Une fois le Bootloader programmé, la programmation du logiciel "StationMétéo.ino" s'effectue directement dans l'IDE Arduino, par le BootLoader, sur le port COM "Maple Mini". Simple et efficace.

// Librairies utilisées : "MapleFreeRTOS900" (FreeRTOS Kernel pour STM32 version 9.0.0), et Port Série Virtuel sur USB (Librairie intégrée au Système Arduino).
// Compilateur utilisé : gcc 7-2017-q4-major. Le compilateur installé par défaut est la version 6-2017-q2-update.
// Les compilateurs de versions supérieures à 7-2017-q4-major ne sont pas compatibles avec ce framework STM32 (testé).

// Procédure à suivre pour installer la version 7-2017-q4-major du compilateur C/C++ :

// Télécharger la version 7-2017-q4-major du Compilateur ici : (https://developer.arm.com/open-source/gnu-toolchain/gnu-rm/downloads). Choisir "Gnu Arm Embedded Toolchain:7-2017-q4-major (December 18, 2017)"
// (Télécharger l'archive (Windows ZIP), la décompacter dans le dossier "Uilisateurs/xxx/AppData/Local/Arduino15/packages/STM32/tools/arm-none-eabi-gcc/7-2017-q4-major" (dossier à créer)
// Modifier le fichier "Uilisateurs/<votre Nom>/AppData/Local/Arduino15/packages/STM32/hardware/stm32/1.4.0/platform.txt" :
// Trouver "compiler.path={runtime.tools.arm-none-eabi-gcc-6-2017-q2-update.path}/bin/" (ligne 22)
// Remplacer par "compiler.path={runtime.tools.arm-none-eabi-gcc-7-2017-q4-major.path}/bin/"


// ************************************
// *  Utilisation Flash/RAM Minimale  *
// ************************************

// Un programme "minimal" (crée une tâche qui fait clignoter une Led) prend environ 16K de Flash et 11K de RAM.
// Ceci est dû aux librairies C, à la gestion du port série virtuel sur USB et à l'OS Temps Réel FreeRTOS.
// On dispose sur ce MCU de 120K de flash programme (+ 8K réservés au Bootloader), et de 20K de RAM (8880 octets libres avec le programme "minimal").


// ************************************
// *  Note sur la Documentation       *
// ************************************

// "Reference Manual" est le document "STM32F103 Reference Manual.pdf" RM0008, DocID13902 Rev 16, de STMicro (fourni)
// "Famille STM32F103" est le document "STM32F103CBT6 (72MHz-128K-20K-QFN48-USB).pdf" DocID13587 Rev 17, de STMicro (fourni)
// "Doc FRAM" est le document "FRAM SPI FM25V05 512Kbit.pdf" pour la FRAM FM25V05 de Cypress (fourni)
// "Doc RTC" est le document "Horloge RTC I2C DS3231MZ avec TCMO.pdf" (fourni)
// "Doc BMP180" est le document "Baromètre I2C BMP180.pdf" (fourni)
// "Doc Luminosité" est le document "Capteur Luminosité Analogique GA1A1S202WP.pdf" (fourni)
// "Doc Temp/Hum" est le document "Capteur Temp & RH Analogique SHT31-ARP.pdf" (fourni)


#ifndef STATION_METEO_H
#define STATION_METEO_H


#define STATION_METEO_VERSION   "0.24"

/*  
  HISTORIQUE DES REVISIONS
  ------------------------
  
  05/01/2019 :  Version 0.05
                Début de gestion du bus SPI1, pour la FRAM. Plante.
                
  06/01/2019 :  Version 0.06 (17096 Flash / 11616 RAM.
                Gestion du bus SPI1 niveau matériel, pour la FRAM. Câblage FRAM sur carte prototype.
                
  07/01/2019 :  Version 0.07 (17808 Flash / 11624 RAM).
                Gestion complète de la FRAM, Lecture, Ecriture, détection densité, gestion adresses sur 16 et 24 bits selon densité. Optimisation.

  08/01/2019 :  Version 0.08 (17840 Flash / 11624 RAM).
                Vérification fréquence horloge SPI1 (36MHz), paramétrage du Mode SPI (0...3), gestion de la mise en veille de la FRAM (5µA de consommation).
                Début de gestion du bus I2C1 pour RTC.

  09/01/2019 :  Version 0.09 (18116 Flash / 11624 RAM). [.ino (31K), .h (17K).]
                Gestion du bus I2C... Normalement fini l'initialisation hardware. A tester. Utilisable pour I2C1 et I2C2, bus à 100KHz et 400KHz gérés.

  10/01/2019 :  Version 0.10
                Câblage de la RTC et du BMP180 sur I2C1 & 2. Fin des routines "de base" pour l'I2C. Plante toujours...

  11/01/2019 :  Version 0.11 (20324 Flash / 11648 RAM)
                L'I2C fonctionne "à peu près", 100KHz et 400KHz (vérifié). Revoir l'agencement des routines bas niveau.
                On lit Minutes/Secondes dans la RTC, mais les minutes s'incrémentent de 2. Il faut 120µs pour la transaction complète de lecture des 2 registres, 120-140µs avec taskYIELD().
                Il y a encore des "glitches"...
                
  12/01/2019 :  Version 0.12 (21032 Flash / 11656 RAM) [.ino (43K), .h (24K).]
                L'I2C fonctionne parfaitement (ne PAS utiliser taskYIELD() dans les routines bas niveau I2C).
                Introduction de Mutex FreeRTOS. Lecture de la date/heure complète, et de la température.
                Lecture/Ecriture d'un registre spécifique sur RTC.
                Optimisation/rationalisation des routines bas niveau I2C, ça baigne.
                Indication du jour de la semaine en clair et du mois en cours en clair.
                Tests de durée de lecture DateTimeRec sur I2C avec oscilloscope. 260µs d'occupation du bus I2C à 400KHz, pour lire les données du DateTimeRec.
                Revu à la baisse l'affichage sur port série virtuel. Demain, on attaque le Baromètre...

  13/01/2019 :  Version 0.13 (22552 Flash, 11784 RAM) [.ino (61K), .h (25K).]
                Portage des routines BMP180 du PIC vers STM32.
                Bus I2C2 plante... Pas de résistances pullup sur le PCB du BMP180... Donc demain soudure... (Ca plante dès le Start) : Pb de bus I2C.
                
  14/01/2019 :  Version 0.14 (28280 Flash, 11784 RAM) [.ino (62K), .h (25K).]
                Ajout des résistances "pullup" pour SDA et SCL sur bus I2C2.
                Fait le Reset des bus SPI1, I2C1 et I2C2, le bus I2C2 fonctionne. Le reset d'I2C1 devait être fait par le soft Arduino...
                Le calcul de la Température et de la Pression Barométrique "Météo" sont achevés. Tout fonctionne bien.
                Avec cette version, toutes les interfaces "Digitales" sont opérationnelles : GPIO, SPI1, I2C1 et I2C2.
                L'utilisation de powf() au lieu du pow() d'Arduino permet de gagner 2K de Flash, et d'économiser 200µs de temps de calcul, pour un résultat identique ici...
                
  15/01/2019 :  Version 0.15
                Câblage VBAT Monitor. Défaut de conception : lorsque Q4 (NFet) est désactivé, la batterie se décharge dans le port ADC du STM32, au-travers de la résistance de 22K.
                A remplacer par un PFet, modifier le PCB.

  16/01/2017 :  Version 0.15 (28472 Flash, 11784 RAM)
                Câblage du PFet pour VBAT Monitor. Problème d'injection tension batterie dans port ADC IN3 quand NFET désactivé.
                Il faudrait remplacer par un PFet + NFet sur sa Gate... (PFet seul ne fonctionne pas, car HIGH = 3V3, VSource = 4V2 => Vgs = -0.9V...
                Ecrit routine d'initialisation de l'ADC1 pour VBATMonitor, mais la conversion A/D ne démarre pas...
                Par contre, ça a l'air de fonctionner avec adc_read()... Donc à voir...
                
  17/01/2019 :  Version 0.16 (29096 Flash, 11816 RAM) [.ino (76K), .h (32K)]
                Finition de la Conversion A/D par IRQ, et écriture de la Conversion A/D de VBAT par DMA, 10 mesures, tri à bulles, moyenne sur 8 valeurs.
                Le "Span" passe de 85-107 à 8-10. Excellent.
                Ecriture de la Tâche VBATMonitorTask(), Gestion du démarrage ADC & DMA dans la Tâche, et conversion ADC -> Tension Batterie.
                Ajout du coefficient de conversion ADC->Tension VBAT dans les Prefs, pour étalonnage facile.
                Fabrication de la planche en bois, support du Prototype.

  18/01/2019 :  Version 0.17 (29456 Flash, 11816 RAM)
                Re-modification du PCB du Prototype pour lire valeur correcte de VBAT (le PFet cessant progressivement de conduire au fur et à mesure que VBAT décroit).
                J'ai donc supprimé le Mosfet Q4, et mis R6 = 220K, R7 = 510K, C7 = 1nF (10nF c'est trop).
                On a quand même un temps de charge de C7 de 4-5 secondes, à cause de la haute impédance... Corrigé par Soft.
                Descendu l'horloge des ADC à 9MHz au lieu de 12MHz, pour laisser plus de temps à l'intégration du signal Analogique.
                Essayé aussi condensateur 22µF en // avec Cout du chargeur, pour voir si Span ADC de VBAT est meilleur... Non, donc abandon de l'idée.
                Simplification des flags. Un peu de "nettoyage" du code Source.
                Recherche nouvelle valeur Coeff pour étalonner la mesure de VBAT (nouveau coefficient de division pour nouvelles valeurs de résistances et Condensateur, R6, R7 et C7).
                Lancement du monitor de décharge batterie pendant la nuit (sans charge USB), pour faire un tableau Tension -> % Charge Batterie.
                Début de structure (sur papier) de l'interfaçage en mode Console avec puTTY (VT220).
                Et réception des PCB d'OSH Park...
                Demain, on s'attaque au Capteur de Luminosité (dans Dôme en plastique blanc)...
                Pour respecter le cahier des charges du Prof, il va faloir déclencher les Acq ADC2 (Capteurs) par des Timers... A explorer...

  19/01/2019 :  Cablage capteur Luminosité (sur dôme blanc) et connecteur sur Prototype. Découpe et peinture plaque Acrylique 1mm.
  
  20/01/2019 :  Version 0.18 (34488 Flash, 11832 RAM)
                Gestion d'ADC2 pour Luminosité, sans Timers. Conversion A/D en continu, et conversion Valeur ADC => Lux, avec étalonnage en Prefs, selon mon LuxMètre.
                Correction ADC VBAT (EOCIE doit être mis à "0" quand on utilise le DMA).
                Calcul des valeurs de Timers 2, 3, 4 (Prescaler et Période) pour générer une interruption à 1Hz...20Hz (au pas de 1Hz), comme demandé.
                Initialisation des Pins ADC pour Température et Humidité.
                
  21/01/2019 :  Version 0.19 (30144 Flash, 11888 RAM) [.ino (95K), .h (46K)]
                Gestion de TIMER2 et des IRQ pour déclencher les Conversions de Luminosité. Ca fonctionne.
                Ecriture du mécanisme permettant d'avoir 3 fréquences de Conversion différentes pour les 3 capteurs (1Hz à 20Hz).
                Pour la Luminosité, on fait 20 Conversions, et on en moyenne 12 "centrales", pour plus de stabilité.
                Ecriture de la Tâche AnalogCapteursTask() qui gère les 3 capteurs (partie Luminosité seulement).

  22/01/2019 : Version 0.20 (29688 Flash, 11944 RAM) [.ino (105K), .h (50K)]
               Gestion des Conversions Température et Humidité dans AnalogCapteursTask() - A tester et finir les calculs.
               Modification des Prefs, pour tenir compte des nouvelles options.
               Création de la structure SalveDATARec et définition des Modes de Travail de la Station.
               Gestion des 3 Timers TIM2, TIM3 et TIM4. A basse fréquence (1-3Hz), ça fonctionne. Testé à 20Hz chaque (60 acq/s), ça a l'air correct.

  23/01/2019 : Soudure composants sur PCB principal avec grosse Hot Plate + fer à souder pour les connecteurs.

  24/01/2019 : Problèmes de RTC qui retarde beaucoup. Essai de plusieurs IC, batteries, toujours le problème.

  25/01/2019 : 3 pcs IC DS3231MZ défectueux... Celui qui fonctionne sur la carte Proto fonctionne aussi sur le PCB...
               Remplacé batterie rechargeable RTC par pile.
               Fait face avant du boîtier, câblage PCB Panel, Leds, Bouton, Connecteurs, trous pour trépied et Chargeur Aux...
               Reste découpe USB à faire.
               Pb du PortA pin 10, clignote, 1Hz, "ON" pendant 20ms... Après recherches, c'est UpdateBarometerTask() le coupable...
               Donc la durée d'acquisition de la Pression Atmosphérique dure 20 ms... Bon à savoir.
               Mis Bouton tactile 5mm + petit cache silicone YZXStudio, avec entretoises laiton 6mm, parfait.
               Footprints des 2 Leds sur PCB Panel complètement pourris...
               On avait un problème de switch ON/OFF, j'ai ressoudé R3 (300K), et là ça fonctionne normalement...
               Soudé fils des Leds Chargeur sur PCB.

  26/01/2019 : Réception Commande Amazon Capteur Température/Humidité. Perçage coffret 5.5mm pour cache du capteur. Perçage pour USB. Renfort des collages.
               Le câble pour Capteur Temp/Humidité n'est pas assez long, rajouter 65mm.
               Ajout d'une variable d'étalonnage pour le Baromètre dans les Prefs, jusqu'à maintenant on jouait sur l'altitude... Pas bon.
               Ajout de la tension sortie Régulateur, utilisé pour les calculs de Température et Humidité.
               Refonte des Etalonnages, plus simples. VBAT, Temp et BARO sont OK. Reste Luminosité et Humidité.
               Fin de construction du Prototype n° 1. Photos des divers éléments.

  27/01/2019 : Version 0.21
               Etalonnages, revu l'affichage en "MODE_BASIC". Finitions du boitier Board#1.
               Le Jour de Semaine passe bien de 7 à 1... (Dimanche suivi de Lundi)...

  29/01/2019 : Version 0.22 (32712 Flash, 11952 RAM) [StationMeteo.ino (115K), StationMeteo.h (55K), InterfacePC.ino (7K), InterfacePC.h (3K)] = 180K
               Création du fichier source "InterfacePC.ino". Laisser l'extension en ".ino" pour que l'IDE s'y retrouve. Si on met le fichier en ".c", rien ne fonctionne...
               Gestion des Commandes : 1) Esc!Modex et 2) Esc!Datax. Gestion "MODE_BASIC" avec Séquences Escape Clear Page & Home. C'est plus lisible à l'écran sur puTTY.
               Ajout de taskYIELD() dans ProgPrincipalTask() et suppression des taskYIELD() et SLEEPTASK() dans InterfacePCTask(). A voir, pour optimiser...
               Dans ProgPrincipalTask() on essaye d'avoir les 3 mesures affichées... (Lum/Temp/Hum)
               A partir de cette version, on doit utiliser puTTY comme Terminal, pour avoir l'émulation VT100/VT220 et les caractères de Contrôle au clavier.
               
  30/01/2019 : Version 0.23(33920 Flash, 11952 RAM).
               Restauration de SLEEPTASK() dans ProgPrincipalTask() et InterfacePCTask(), sinon ça bloque...
               Meilleure synchro affichage en MODE_BASIC.
               Gestion de l'envoi de toutes les Mesures en mode "MODE_TEMPS_REEL". Gestion du TimeStamp (Unix) (dans les 2 sens : Date2secs() et Secs2Date()).
               En gros, le mode "MODE_TEMPS_REEL" est fait, pour l'envoi des données.
               
  31/01/2019 : Version 0.24 (35184 Flash, 12608 RAM) [StationMeteo.ino (113K), StationMeteo.h (57K), InterfacePC.ino (24K), InterfacePC.h (7K)] = 201K
               Gestion complètement revue de la réception des Commandes par liaison Série.
               Correction des variables (signed short) et des affichages pour les Températures Négatives...
               Ecriture des petites routines d'affichage des Mesures, pour réemploi.
               Ecriture de la gestion du Mode n° 2 (Salves).
               Définition de nouvelles Commandes, pas encore gérées mais "définies".
               La version suivante devra gérer le mode de Travail "MODE_ONDEMAND", avec écriture des Mesures dans un Buffer Circulaire en FRAM...

 01/02/2019 : Version 0.25

 
*/

// Gestion des différents Prototype (pour les Etalonnages par défaut, en fonction des tolérances des Composants et des Capteurs)

#define   PROTOTYPE     1
#define   BOARD1        2
#define   BOARD2        3
#define   BOARD3        4

#define   BOARD_USED    PROTOTYPE
//#define   BOARD_USED    BOARD1


#include <MapleFreeRTOS900.h>           // On utilise la version 9.0.0 de FreeRTOS pour STM32, compatible Maple Mini. La version 10 n'est pas encore (Janvier 2019) portée sur cette plateforme.
#include <libmaple/stm32.h>
#include <libmaple/rcc.h>
#include <libmaple/gpio.h>
#include <libmaple/spi.h>
#include <libmaple/i2c.h>
#include <libmaple/adc.h>
#include <libmaple/nvic.h>
#include <libmaple/dma.h>
#include <libmaple/timer.h>

// Les Structures et Defines concernant le Hardware du STM32 sont dans le dossier "Program Files(x86)/Arduino/hardware/Arduino_STM32/STM32F1/system/libmaple/stm32f1/include/series"

// ******************************************************************
// *  Defines des ports GPIO Digitaux STM32 utilisés par le projet  *
// ******************************************************************

#define RCC_APB2_IOPAEN_BIT     2           // Bit 2 pour activer l'hologe du Port A, dans registre RCC_APB2ENR
#define RCC_APB2_IOPBEN_BIT     3           // Bit 3 pour activer l'hologe du Port B, dans registre RCC_APB2ENR

// On définit ces fonctions "inline" ici, pour pouvoior les utiliser dans les macros

static inline void ClearGPIO(gpio_dev* Port, unsigned short Pin);
static inline void SetGPIO(gpio_dev* Port, unsigned short Pin);

// *************************************
// * Entrées/Sorties sur le port GPIOA *
// *************************************

#define MEM_CS_GPIO             GPIOA
#define MEM_CS_PIN              4           // Output - Chip Select de FRAM sur le Port A, Pin 4 (actif au niveau "0")
#define MEM_SELECT()            ClearGPIO(MEM_CS_GPIO,MEM_CS_PIN)
#define MEM_DESELECT()          SetGPIO(MEM_CS_GPIO,MEM_CS_PIN)

#define USER_LED_RED_GPIO       GPIOA
#define USER_LED_RED_PIN        9           // Output - La Led "User" rouge est sur le Port A, Pin 9
#define USER_LED_RED_ON()       ClearGPIO(USER_LED_RED_GPIO,USER_LED_RED_PIN) // Led Anode Commune, allumée au niveau "0"
#define USER_LED_RED_OFF()      SetGPIO(USER_LED_RED_GPIO,USER_LED_RED_PIN)   // Led Anode Commune, éteinte au niveau "1"

#define USER_LED_GREEN_GPIO     GPIOA
#define USER_LED_GREEN_PIN      10          // Output - La Led "User" verte est sur le Port A, Pin 10
#define USER_LED_GREEN_ON()     ClearGPIO(USER_LED_GREEN_GPIO,USER_LED_GREEN_PIN) // Led Anode Commune, allumée au niveau "0"
#define USER_LED_GREEN_OFF()    SetGPIO(USER_LED_GREEN_GPIO,USER_LED_GREEN_PIN)   // Led Anode Commune, éteinte au niveau "1"

#define EN_BATMON_GPIO          GPIOA
#define EN_BATMON_PIN           15           // Output - La sortie pour valider la mesure tension batterie est sur le Port A, Pin 15 (PA15)
#define BATMON_ENABLE()         SetGPIO(EN_BATMON_GPIO,EN_BATMON_PIN) // NFET, Gate à "1" pour activer
#define BATMON_DISABLE()        SetGPIO(EN_BATMON_GPIO,EN_BATMON_PIN) // ClearGPIO(EN_BATMON_GPIO,EN_BATMON_PIN) normalement, mais on a désactivé cette fonction.

// *************************************
// * Entrées/Sorties sur le port GPIOB *
// *************************************

#define USB_DET_GPIO            GPIOB
#define USB_DET_PIN             0           // Input - La détection d'USB connecté est sur le port B, pin 0 (PB0)

#define MAPLE_LED_GPIO          GPIOB
#define MAPLE_LED_PIN           1           // Output - La Led du Maple Mini (sur carte MCU) est sur le Port B, Pin 1 (PB1)
#define MAPLE_LED               PB1         // Pour accéder à la Led au-travers du Framework Arduino, pour tester, si besoin...
#define MAPLE_LED_ON()          SetGPIO(MAPLE_LED_GPIO,MAPLE_LED_PIN)
#define MAPLE_LED_OFF()         ClearGPIO(MAPLE_LED_GPIO,MAPLE_LED_PIN)

#define EN_CAPTEURS_GPIO        GPIOB
#define EN_CAPTEURS_PIN         3           // Output - La sortie pour alimenter les capteurs analogiques est sur le Port B, Pin 3 (PB3)
#define CAPTEURS_POWERON()      ClearGPIO(EN_CAPTEURS_GPIO,EN_CAPTEURS_PIN) // PFET, Gate à "0" pour activer
#define CAPTEURS_POWEROFF()     SetGPIO(EN_CAPTEURS_GPIO,EN_CAPTEURS_PIN)

#define SWOUT_GPIO              GPIOB
#define SWOUT_PIN               4           // Input / Output - L'indicateur Marche/Arrêt & Auto-OFF est sur le Port B, Pin 4 (PB4)
                                            // ATTENTION !! Cette Pin doit être "Compatible 5V", car elle est reliée à la tension VIN (4.2V batterie ou 5V USB).
                                            // La doc "Famille STM32F103" indique page 32 que PB4 remplit cette condition.
#define RTCINT_GPIO             GPIOB
#define RTCINT_PIN              5           // Input - L'interruption 1Hz générée par l'horloge RTC (avec Pullup, car Open-Drain)


// *************************************************************
// *  Defines du bus SPI1 et de la FRAM (externe) du projet    *
// *************************************************************

// FM25V05 FRAM (512Kbit : 64K x 8bit) Opcodes
// On envoie toujours <OpCode 8 bits><Adresse 16 bits><Data 8 bits>...<Data 8 bits>... pour Lecture/Ecriture

#define FRAM_WRITE              0x02        // Pour écrire dans la FRAM - Doc FRAM Page 6 et 9
#define FRAM_READ               0x03        // Pour lire la FRAM - Doc FRAM Page 6 et 9
#define FRAM_WRDI               0x04        // Reset du latch d'écriture, écriture dans la FRAM interdite
#define FRAM_WREN               0x06        // Active le Latch pour autoriser l'écriture dans la FRAM
#define FRAM_RDID               0x9F        // Pour lire les 9 octets du Device ID - Doc FRAM Page 6 et 11
#define FRAM_SLEEP              0xB9        // La FRAM est mise en mode Veille - Doc FRAM Page 6 et 11

// La FRAM est sur le bus SPI1, bus interne APB2 du STM32F103. Le STM32F103 est "Maitre", la FRAM est "Esclave".

#define RCC_APB2_SPI1EN_BIT     12           // Bit 12 pour activer l'hologe du bus SPI1, dans registre RCC_APB2ENR
#define RCC_APB2RSTR_SPI1_MASK  0x00000800   // Bit 12 pour le Reset de SPI1, dans le registre RCC_APB2RSTR, Reference Manual pages 105-106.

#define FRAM_SPI                SPI1         // La FRAM est sur le bus SPI1.

#define SPI1_SCK_PORT           GPIOA
#define SPI1_SCK_PIN            5            // Output - Le SCK  de SPI1 est sur PA5 (Clock)
#define SPI1_MISO_PORT          GPIOA
#define SPI1_MISO_PIN           6            // Input -  Le MISO de SPI1 est sur PA6 (Master Input Slave Output, lecture des données de la FRAM)
#define SPI1_MOSI_PORT          GPIOA
#define SPI1_MOSI_PIN           7            // Output - Le MOSI de SPI1 est sur PA7 (Master Output Slave Input, Ecriture des données dans la FRAM)

// Defines pour le registre CR1, Reference Manual page 745-746

#define SPI_2LINESMODE          (0U << 15)
#define SPI_DATAFRAME_8BITS     (0U << 11)
#define SPI_DATAFRAME_16BITS    (1U << 11)
#define SPI_FULLDUPLEX          (0U << 10)
#define SPI_SOFTWARE_CS         (1U << 9) // SSM = 1, Reference Manual Page 706
#define SPI_SSIENABLE           (1U << 8) // SSI = 1
#define SPI_MSBFIRST            (0U << 7)
#define SPI_LSBFIRST            (1U << 7)
#define SPI_ENABLE              (1U << 6)
#define SPI_DISABLE             (0U << 6)

#define I2C_NAK_POS_CURRENT     (0xF7FF)
#define I2C_NAK_POS_NEXT        (0x0800)

#define SPI_CLK_DIV2            (0x0 << 3)   // 36MHz sur APB2 (SPI1), 18MHz sur APB1 (SPI2)
#define SPI_CLK_DIV4            (0x1 << 3)   // 18MHz sur APB2 (SPI1), 9MHz sur APB1 (SPI2)
#define SPI_CLK_DIV8            (0x2 << 3)   // 9MHz sur APB2 (SPI1), 4.5MHz sur APB1 (SPI2)
#define SPI_CLK_DIV16           (0x3 << 3)   // 4.5MHz sur APB2 (SPI1), 2.25MHz sur APB1 (SPI2)
#define SPI_CLK_DIV32           (0x4 << 3)   // 2.25MHz sur APB2 (SPI1), 1125KHz sur APB1 (SPI2)
#define SPI_CLK_DIV64           (0x5 << 3)   // 1125KHz sur APB2 (SPI1), 562KHz sur APB1 (SPI2)
#define SPI_CLK_DIV128          (0x6 << 3)   // 562KHz sur APB2 (SPI1), 281KHz sur APB1 (SPI2)
#define SPI_CLK_DIV256          (0x7 << 3)   // 281KHz sur APB2 (SPI1), 140KHz sur APB1 (SPI2)

#define FRAM_SPI_CLOCK          SPI_CLK_DIV2 // On choisit de configurer l'horloge SPI à 36MHz pour la FRAM. Modifier ici et re-compiler le Projet en cas de changement.

#define SPI_MASTERMODE          (1U << 2)
#define SPI_SLAVEMODE           (0U << 2)
#define SPI_CPOL_LOW            (0U << 1)    // Clock polarity Low when idle
#define SPI_CPOL_HIGH           (1U << 1)    // Clock polarity High when idle
#define SPI_CPHA_1EDGE          (0U << 0)    // First clock transition is first data capture edge
#define SPI_CPHA_2EDGE          (1U << 0)    // Second clock transition is first data capture edge

#define SPI_MODE_0              (SPI_CPOL_LOW | SPI_CPHA_1EDGE)
#define SPI_MODE_1              (SPI_CPOL_LOW | SPI_CPHA_2EDGE)
#define SPI_MODE_2              (SPI_CPOL_HIGH | SPI_CPHA_1EDGE)
#define SPI_MODE_3              (SPI_CPOL_HIGH | SPI_CPHA_2EDGE)

// Defines pour connaître la densité de la FRAM (FRAM Cypress FM25Vxx (128K à 2Mbit) et CY15B104Q (4Mbit)), bits 0...4 octet 8 sur 9 du Device ID.

#define FRAM128K                1            // FM25V01-A
#define FRAM256K                2            // FM25V02-A
#define FRAM512K                3            // FM25V05-G
#define FRAM1024K               4            // FM25V10-G
#define FRAM2048K               5            // FM25V20-G
#define FRAM4096K               6            // CY15B104Q


// **********************************
// *  Defines des bus I2C1 et I2C2  *
// **********************************

// Fréquence maximale du bus I2C du STM32F103 : 400KHz

#define RCC_APB1_I2C1EN_BIT     21           // Bit 21 pour activer l'hologe du bus I2C1, dans registre RCC_APB1ENR, Reference Manual page 114.
#define RCC_APB1_I2C2EN_BIT     22           // Bit 22 pour activer l'hologe du bus I2C2, dans registre RCC_APB1ENR, Reference Manual page 114.
#define RCC_APB1RSTR_I2C1_RST   21           // Bit 21 pour faire le Reset du bus I2C1, dans le registre RCC_APB1RSTR, Reference Manual page 108.
#define RCC_APB1RSTR_I2C2_RST   22           // Bit 22 pour faire le Reset du bus I2C2, dans le registre RCC_APB1RSTR, Reference Manual page 108.
#define RCC_APB1RSTR_I2C1_MASK  0x00200000   // Mask pour Reset de I2C1 dans le registre RCC_APB1RSTR.
#define RCC_APB1RSTR_I2C2_MASK  0x00400000   // Mask pour Reset de I2C2 dans le registre RCC_APB1RSTR.


#define I2C_DUTY_CYCLE_2        1            // SCL tLow/tHigh = 2, pour Fast I2C > 100KHz
#define I2C_DUTY_CYCLE_16_9     2            // SCL tLow/tHigh = 16/9, pour Fast I2C, > 100KHz

// Defines pour le registre CR1, Reference Manual page 777

#define I2C_SWRST               (1U << 15)    // Reset Logiciel du périphérique I2C
#define I2C_ACK_ENABLE          (1U << 10)    // Active le Acknowledge
#define I2C_STOP                (1U << 9)     // Génère un signal Stop
#define I2C_START               (1U << 8)     // Génère un signal Start
#define I2C_NOSTRETCH           (1U << 7)     // Désactive le Clock stretching
#define I2C_ENGC                (1U << 6)     // Active le General call
#define I2C_MODEI2C             (0U << 1)     // Mode I2C
#define I2C_ENABLE              (1U << 0)     // Active le périphérique I2C

#define ENABLE                  1
#define DISABLE                 0
#define ACK_POS_CURRENT         1
#define ACK_POS_NEXT            2

#define I2C1_SCL_PORT           GPIOB
#define I2C1_SCL_PIN            6            // Output Open Drain - Le SCL de I2C1 est sur PB6 (Clock)
#define I2C1_SDA_PORT           GPIOB
#define I2C1_SDA_PIN            7            // Output Open Drain - Le SDA de I2C1 est sur PB7 (Data)

#define I2C2_SCL_PORT           GPIOB
#define I2C2_SCL_PIN            10           // Output Open Drain - Le SCL de I2C2 est sur PB10 (Clock)
#define I2C2_SDA_PORT           GPIOB
#define I2C2_SDA_PIN            11           // Output Open Drain - Le SDA de I2C2 est sur PB11 (Data)

#define STM32_I2C_ADDR_RTC      0x01         // STM32 Master I2C "Own Address" sur I2C1
#define STM32_I2C_ADDR_BARO     0x00         // STM32 Master I2C "Own Address" sur I2C2



// ***************************************************
// *  Defines de l'Horloge RTC DS3231MZ et du BMP180 *
// ***************************************************

// Circuit d'horloge Real Time Clock DS3231MZ.
// Oscillateur MEMS compensé en température, stabilité 5ppm.
// Sauvegardé par batterie Lithium 3V rechargeable.

// La RTC est sur le bus I2C1, bus interne APB1 du STM32F103.

#define RTC_I2C                 1            // La RTC est sur le bus I2C1.
#define RTC_I2C_BUS             I2C1
#define RTC_BUS_SPEED           400000       // On accède à la RTC par un bus I2C à 400Kbit/s (Maximum toléré par RTC : 400Kbit/s).
                                             // La fréquence Système du STM32 doit être multiple de 10MHz pour atteindre 400KHz. On utilise 72MHz.
                                             
#define RTC_I2C_DUTYCYCLE       I2C_DUTY_CYCLE_2

#define RTC_WRITE_ADDR          0xD0         // Adresse I2C pour écrire dans la RTC
#define RTC_READ_ADDR           0xD1         // Adresse I2C pour lire la RTC

#define RTC_DAYOFWEEK_REG       0x03


// Le Baromètre BMP180 est sur le bus I2C2, bus interne APB1 du STM32F103.

#define BMP180_I2C              2
#define BMP180_I2C_BUS          I2C2
#define BMP180_BUS_SPEED        400000       // On accède au BMP180 par un bus I2C à 400Kbit/s (Maximum toléré par BMP180 : 2Mbit/s)
                                             // La fréquence Système du STM32 doit être multiple de 10MHz pour atteindre 400KHz. On utilise 72MHz.

#define BMP180_I2C_DUTYCYCLE    I2C_DUTY_CYCLE_2

#define BMP180_WRITE_ADDR       0xEE         // Adresse I2C pour écrire dans le BMP180
#define BMP180_READ_ADDR        0xEF         // Adresse I2C pour lire le BMP180

#define BAROMAXTEMPERATUREACQTIME   5       // 5 ms pour 4.5 ms max, temps de Conversion de la Température

// Définition de la résolution du BMP180, Doc BMP180 page 12.
// Resolution = 0 : 1 échantillon,  tconvmax = 5ms (basse consommation)
// Resolution = 1 : 2 échantillons, tconvmax = 8ms (standard)
// Resolution = 2 : 4 échantillons, tconvmax = 14ms (haute résolution)
// Resolution = 3 : 8 échantillons, tconvmax = 26ms (ultra haute résolution)

#define BARORESOLUTION              2

#if   (BARORESOLUTION == 0) 
    #define BAROMAXCONVERSIONTIME   5  //10
#elif (BARORESOLUTION == 1)
    #define BAROMAXCONVERSIONTIME   8  //15
#elif (BARORESOLUTION == 2)
    #define BAROMAXCONVERSIONTIME   14  //20
#elif (BARORESOLUTION == 3)
    #define BAROMAXCONVERSIONTIME   26  //30
#else
    #define BAROMAXCONVERSIONTIME   26  //30
#endif


// *****************************************************************
// *  Defines pour la Conversion Analogique/Digital (ADC1 & ADC2)  *
// *****************************************************************

// Temps d'échantillonnage pour chaque Canal A/D (de 0 à 9), Registre ADC_SMPR2, Reference Manual page 249.
// Pour les Canaux A/D de 10 à 17, on doit utiliser le Registre ADC_SMPR1 (non disponibles sur STM103RBT6).

#define SAMPLE_TIME_1_5         0           // Temps d'échantillonnage : 1.5 cycles d'horloge ADC
#define SAMPLE_TIME_7_5         1           // Temps d'échantillonnage : 7.5 cycles d'horloge ADC
#define SAMPLE_TIME_13_5        2           // Temps d'échantillonnage : 13.5 cycles d'horloge ADC
#define SAMPLE_TIME_28_5        3           // Temps d'échantillonnage : 28.5 cycles d'horloge ADC
#define SAMPLE_TIME_41_5        4           // Temps d'échantillonnage : 41.5 cycles d'horloge ADC
#define SAMPLE_TIME_55_5        5           // Temps d'échantillonnage : 55.5 cycles d'horloge ADC
#define SAMPLE_TIME_71_5        6           // Temps d'échantillonnage : 71.5 cycles d'horloge ADC
#define SAMPLE_TIME_239_5       7           // Temps d'échantillonnage : 239.5 cycles d'horloge ADC

// ADC_SMPR2
#define ADC_SAMPLE_TIME0(x)     (x << 0)     // Décalage pour Canal A/D numéro 0
#define ADC_SAMPLE_TIME1(x)     (x << 3)     // Décalage pour Canal A/D numéro 1
#define ADC_SAMPLE_TIME2(x)     (x << 6)     // Décalage pour Canal A/D numéro 2
#define ADC_SAMPLE_TIME3(x)     (x << 9)     // Décalage pour Canal A/D numéro 3
#define ADC_SAMPLE_TIME4(x)     (x << 12)    // Décalage pour Canal A/D numéro 4
#define ADC_SAMPLE_TIME5(x)     (x << 15)    // Décalage pour Canal A/D numéro 5
#define ADC_SAMPLE_TIME6(x)     (x << 18)    // Décalage pour Canal A/D numéro 6
#define ADC_SAMPLE_TIME7(x)     (x << 21)    // Décalage pour Canal A/D numéro 7
#define ADC_SAMPLE_TIME8(x)     (x << 24)    // Décalage pour Canal A/D numéro 8
#define ADC_SAMPLE_TIME9(x)     (x << 27)    // Décalage pour Canal A/D numéro 9

// ADC_SMPR1
#define ADC_SAMPLE_TIME10(x)    (x << 0)     // Décalage pour Canal A/D numéro 10
#define ADC_SAMPLE_TIME11(x)    (x << 3)     // Décalage pour Canal A/D numéro 11
#define ADC_SAMPLE_TIME12(x)    (x << 6)     // Décalage pour Canal A/D numéro 12
#define ADC_SAMPLE_TIME13(x)    (x << 9)     // Décalage pour Canal A/D numéro 13
#define ADC_SAMPLE_TIME14(x)    (x << 12)    // Décalage pour Canal A/D numéro 14
#define ADC_SAMPLE_TIME15(x)    (x << 15)    // Décalage pour Canal A/D numéro 15
#define ADC_SAMPLE_TIME16(x)    (x << 18)    // Décalage pour Canal A/D numéro 16
#define ADC_SAMPLE_TIME17(x)    (x << 21)    // Décalage pour Canal A/D numéro 17

// Définitions des Canaux A/D à utiliser pour la Séquence de Conversions (16 Conversions maximum dans 1 Séquence).

// SQR3
#define ADC_CONV1(Canal)  (Canal << 0)       // Indique quel Canal A/D utiliser pour la Conversion n° 01 dans la séquence.
#define ADC_CONV2(Canal)  (Canal << 5)       // Indique quel Canal A/D utiliser pour la Conversion n° 02 dans la séquence.
#define ADC_CONV3(Canal)  (Canal << 10)      // Indique quel Canal A/D utiliser pour la Conversion n° 03 dans la séquence.
#define ADC_CONV4(Canal)  (Canal << 15)      // Indique quel Canal A/D utiliser pour la Conversion n° 04 dans la séquence.
#define ADC_CONV5(Canal)  (Canal << 20)      // Indique quel Canal A/D utiliser pour la Conversion n° 05 dans la séquence.
#define ADC_CONV6(Canal)  (Canal << 25)      // Indique quel Canal A/D utiliser pour la Conversion n° 06 dans la séquence.

// SQR2
#define ADC_CONV7(Canal)  (Canal << 0)       // Indique quel Canal A/D utiliser pour la Conversion n° 07 dans la séquence.
#define ADC_CONV8(Canal)  (Canal << 5)       // Indique quel Canal A/D utiliser pour la Conversion n° 08 dans la séquence.
#define ADC_CONV9(Canal)  (Canal << 10)      // Indique quel Canal A/D utiliser pour la Conversion n° 09 dans la séquence.
#define ADC_CONV10(Canal) (Canal << 15)      // Indique quel Canal A/D utiliser pour la Conversion n° 10 dans la séquence.
#define ADC_CONV11(Canal) (Canal << 20)      // Indique quel Canal A/D utiliser pour la Conversion n° 11 dans la séquence.
#define ADC_CONV12(Canal) (Canal << 25)      // Indique quel Canal A/D utiliser pour la Conversion n° 12 dans la séquence.

// SQR1
#define ADC_CONV13(Canal) (Canal << 0)       // Indique quel Canal A/D utiliser pour la Conversion n° 13 dans la séquence.
#define ADC_CONV14(Canal) (Canal << 5)       // Indique quel Canal A/D utiliser pour la Conversion n° 14 dans la séquence.
#define ADC_CONV15(Canal) (Canal << 10)      // Indique quel Canal A/D utiliser pour la Conversion n° 15 dans la séquence.
#define ADC_CONV16(Canal) (Canal << 15)      // Indique quel Canal A/D utiliser pour la Conversion n° 16 dans la séquence.


// ********************************************************************
// *  Defines pour la Conversion A/D du Moniteur de Tension Batterie  *
// ********************************************************************

#define VBMON_ADC               ADC1         // VBATMonitor utilise l'ADC1 du STM32F103

#define RCC_APB2_ADC1EN_BIT     9            // Bit 9 pour activer l'hologe de l'ADC1 dans le Registre RCC_APB2ENR
#define RCC_APB2RSTR_ADC1_MASK  0x00000200   // Mask pour Reset de ADC1 dans le registre RCC_APB2RSTR.

#define RCC_CFGR_ADC_CLOCK      ADC_PRE_PCLK2_DIV_8   // 72MHz du bus APB2 / 8 = 9MHz fréquence Horloge ADC1 & ADC2 (Max = 14MHz), Reference Manual page 100.
                                                      // On a ADC_PRE_PCLK2_DIV_2, ADC_PRE_PCLK2_DIV_4, ADC_PRE_PCLK2_DIV_6, ADC_PRE_PCLK2_DIV_8
                                                      
#define VBATMON_AN3_GPIO        GPIOA
#define VBATMON_AN3_PIN         3                     // Entrée Analogique Canal 3, sur le Port A, Pin 3

#define VBATMON_ADC_CHANEL      3                     // PA3 = IN3 = Canal ADC numéro 3.
#define VBATMON_SAMPLE_TIME     SAMPLE_TIME_239_5     // Le Pont Diviseur sur VBAT est à haute impédance, donc on doit utiliser le temps d'intégration le plus long.
#define VBATMON_DMA_PRIORITY    DMA_CCR_PL_MEDIUM     // Options : DMA_CCR_PL_LOW, DMA_CCR_PL_MEDIUM, DMA_CCR_PL_HIGH ou DMA_CCR_PL_VERY_HIGH

#define VBATMON_START_CONV()    (VBMON_ADC->regs->CR2 |= ADC_CR2_SWSTART)


// *************************************************************************
// *  Defines pour la Conversion A/D des 3 capteurs Analogiques du Projet  *
// *************************************************************************

// Les 3 Capteurs Analogiques du projet sont :
// -----------------------------------------

// 1) Capteur de Luminosité : GA1A1S202WP, sensible à la longueur d'onde 555nm (jaune), forte immunité face aux Infrarouges (chaleur), plage de mesures de 3 à 55000 Lux
//                            Ce capteur fournit un courant de sortie (Io) proportionel au logarithme de l'éclairage exprimé en Lux (Ev) : Io (µA) = 10xLog(Ev), Doc Luminosité page 7
//                            Une résistance de 68 K Ohms est placée entre la sortie de ce Capteur et la Masse, pour convertir ce courant en Tension, suivant la Loi d'Ohms U = R x I.
//                            En étudiant la droite du courant de sortie page 10 de la Doc Luminosité, on lit les valeurs suivantes :
//                            Ev = 3 Lux      => Io = 5µA     => Vout = (5 x 10^-6)     x 68000  = 0.34V
//                            Ev = 55000 Lux  => Io = 47.5µA  => Vout = (47.5 x 10^-6)  x 68000  = 3.23V

#define CAPTEURS_ADC            ADC2                // Les 3 capteurs analogiques sont lus par l' ADC2 du STM32F103

#define RCC_APB2_ADC2EN_BIT     10                  // Bit 10 pour activer l'hologe de l'ADC2 dans le Registre RCC_APB2ENR
#define RCC_APB2RSTR_ADC2_MASK  0x00000400          // Mask pour Reset de ADC2 dans le registre RCC_APB2RSTR.

#define ADC_START_ACQ(dev)      {dev->regs->CR2 |= ADC_CR2_ADON; dev->regs->CR2 |= ADC_CR2_SWSTART;}

#define CAPTEURS_SAMPLE_TIME    SAMPLE_TIME_239_5   // On a le temps, on utilise donc le temps d'intégration le plus long.

// Capteur de Luminosité

#define LUM_AN0_GPIO            GPIOA
#define LUM_AN0_PIN             0                   // Entrée Analogique Canal 0, sur le Port A, Pin 0
#define LUM_ADC_CANAL           0                   // PA0 = IN0 = Canal ADC numéro 0.
#define LUM_TIMER               TIM2                // Timer TIM2 utilisé pour déclencher les Conversions de Luminosité, à la fréquence choisie.

// Capteur de Température

#define TEMP_AN1_GPIO           GPIOA
#define TEMP_AN1_PIN            1                   // Entrée Analogique Canal 1, sur le Port A, Pin 1
#define TEMP_ADC_CANAL          1                   // PA1 = IN1 = Canal ADC numéro 1.
#define TEMP_TIMER              TIM3                // Timer TIM3 utilisé pour déclencher les Conversions de Température, à la fréquence choisie.

// Capteur d'Humidité

#define HUM_AN2_GPIO            GPIOA
#define HUM_AN2_PIN             2                   // Entrée Analogique Canal 0, sur le Port A, Pin 2
#define HUM_ADC_CANAL           2                   // PA2 = IN2 = Canal ADC numéro 2.
#define HUM_TIMER               TIM4                // Timer TIM2 utilisé pour déclencher les Conversions d'Humidité, à la fréquence choisie.


// Constantes pour les Timers TIM2, TIM3 et TIM4, utilisés pour démarrer les Conversions de nos 3 Capteurs à 3 intervalles indépendants (de 1Hz à 20Hz).

#define TIMER_2                   2
#define TIMER_3                   3
#define TIMER_4                   4

#define RCC_APB1RSTR_TIMER2_MASK  0x00000001        // Mask pour Reset de TIMER2 dans le registre RCC_APB1RSTR.
#define RCC_APB1RSTR_TIMER3_MASK  0x00000002        // Mask pour Reset de TIMER3 dans le registre RCC_APB1RSTR.
#define RCC_APB1RSTR_TIMER4_MASK  0x00000004        // Mask pour Reset de TIMER4 dans le registre RCC_APB1RSTR.

#define TIMER_COUNTERMODE_UP      (unsigned short) 0x0000
#define TIMER_COUNTERMODE_DOWN    (unsigned short) 0x0010

#define START_TIMER(dev)          (((dev->regs).gen)->CR1  |= TIMER_CR1_CEN)

#define TIMER_PRESCALER           7200              // 72MHz / 7200 = 10000Hz
const unsigned short              TimerPeriode[20] = {10000, 5000, 3333, 2500, 2000, 1666, 1428, 1250, 1111, 1000, 909, 833, 769, 714, 666, 625, 588, 555, 526, 500};

// Pour Timer  1Hz :                                // (72MHz / 7200) / 10000 =  1Hz
// Pour Timer  2Hz :                                // (72MHz / 7200) /  5000 =  2Hz
// Pour Timer  3Hz :                                // (72MHz / 7200) /  3333 =  3Hz
// Pour Timer  4Hz :                                // (72MHz / 7200) /  2500 =  4Hz
// Pour Timer  5Hz :                                // (72MHz / 7200) /  2000 =  5Hz
// Pour Timer  6Hz :                                // (72MHz / 7200) /  1666 =  6Hz
// Pour Timer  7Hz :                                // (72MHz / 7200) /  1428 =  7Hz
// Pour Timer  8Hz :                                // (72MHz / 7200) /  1250 =  8Hz
// Pour Timer  9Hz :                                // (72MHz / 7200) /  1111 =  9Hz
// Pour Timer 10Hz :                                // (72MHz / 7200) /  1000 = 10Hz
// Pour Timer 11Hz :                                // (72MHz / 7200) /   909 = 11Hz
// Pour Timer 12Hz :                                // (72MHz / 7200) /   833 = 12Hz
// Pour Timer 13Hz :                                // (72MHz / 7200) /   769 = 13Hz
// Pour Timer 14Hz :                                // (72MHz / 7200) /   714 = 14Hz
// Pour Timer 15Hz :                                // (72MHz / 7200) /   666 = 15Hz
// Pour Timer 16Hz :                                // (72MHz / 7200) /   625 = 16Hz
// Pour Timer 17Hz :                                // (72MHz / 7200) /   588 = 17Hz
// Pour Timer 18Hz :                                // (72MHz / 7200) /   555 = 18Hz
// Pour Timer 19Hz :                                // (72MHz / 7200) /   526 = 19Hz
// Pour Timer 20Hz :                                // (72MHz / 7200) /   500 = 20Hz


// *******************************************************
// *  Defines pour les Commandes PC et séquences Escape  *
// *******************************************************

#define TAB                       0x09
#define ESC                       0x1B
#define BELL                      0x07
#define ESCAPE_INTRO              '!'

#define CLS()                     Serial.write(ESC); Serial.write("["); Serial.write("2"); Serial.write("J"); Serial.write(ESC); Serial.write("[");Serial.write("H");

// Commandes reconnues : ("Esc" représente sur 1 octet unique le code ASCII 0x1B, ou CHAR(27) décimal.

// Esc!Modex (1 <= x <= 7)        Changement de Mode de Travail de la Station (voir Modes de Travail ci-dessous)
//                                Esc!Mode1 : Bascule en "MODE_BASIC"
//                                Esc!Mode2 : Bascule en "MODE_TEMPS_REEL"
//                                Esc!Mode3 : Bascule en "MODE_SALVES"
//                                Esc!Mode4 : Bascule en "MODE_ONDEMAND"
//                                Esc!Mode5 : Bascule en "MODE_EVENEMENTIEL"
//                                Esc!Mode6 : Bascule en "MODE_CONSOLE"
//                                Esc!Mode7 : Bascule en "MODE_METEO"


// *********************************
// *   Définition des Structures   *
// *********************************

// Structure pour la RTC

typedef struct {
  unsigned char   Secondes;
  unsigned char   Minutes;
  unsigned char   Heure;
  unsigned char   JourSem;      // Valeur : 1-7 (1 = Lundi, 2 = Mardi ... 7 = Dimanche).
  unsigned char   Date;         // Valeur : 1-31
  unsigned char   Mois;         // Valeur : 1-12
  unsigned char   Annee;        // Valeur : 0-99
  unsigned char   Padding;      // Pour aligner le 32bit suivant sur 4 octets
  unsigned long   TimeStamp;    // Nombre de secondes depuis le 01/01/1970 0h00 (pour Horodatage) (Temps Unix).
} DateTimeRec;

// Structures pour le Baromètre

typedef struct
{
  // Coefficients de calibrage du BMP180, stockés dans son EEPROM (11 x 16 bits), Doc BMP180 page 13
  
  int16_t           AC1;
  int16_t           AC2; 
  int16_t           AC3; 
  uint16_t          AC4;
  uint16_t          AC5;
  uint16_t          AC6;
  int16_t           _B1; 
  int16_t           _B2;
  int16_t           MB;
  int16_t           MC;
  int16_t           MD;
  
  int16_t           _B5;            // Calculé après avoir lu UT
  int16_t           Altitude;       // Altitude réelle du lieu, en mètres, entrée manuellement

  // Valeurs internes et Finales

  unsigned long     UT;             // Uncompensated Temperature read from BMP085 (was signed long)
  signed short      Temperature;    // Real Temperature in 1/10 °C => 285 = 28.5 °C
  unsigned long     UP;
  unsigned long     AbsPressure;    // Pression Atmosphérique Absolue, en Pa (Pascal, 1/100e mBar) => 97525 = 975.25 hPa = 975.25 mBar
  unsigned long     MeteoPressure;  // Pression Atmosphérique compatible "Météo" (Niveau de la mer): Pression compensée en fonction de l'altitude du lieu.

  unsigned short    OSS;            // Niveau de sur-échantillonnage (Oversampling) pour mesurer la Pression (0...3), Doc BMP180 page 12.
} BARORec,*BAROPtr;

typedef struct
{
  // Coefficients de calibrage du BMP180, stockés dans son EEPROM (11 x 16 bits), Doc BMP180 page 13
  
  int16_t           AC1;
  int16_t           AC2; 
  int16_t           AC3; 
  uint16_t          AC4;
  uint16_t          AC5;
  uint16_t          AC6;
  int16_t           _B1; 
  int16_t           _B2;
  int16_t           MB;
  int16_t           MC;
  int16_t           MD;
  int16_t          _B5;            // Calculé après avoir lu UT
  int16_t          Altitude;       // Altitude réelle du lieu, en mètres, entrée manuellement
} BAROCalib,*BAROCalibPtr;

// Structure de nos Préférences, sauvegardées en FRAM

typedef struct
{
  unsigned char     Emplacement[14];// Nom du l'emplacement qui est à cette altitude (Longueur 13 max + 0x00 final)
  signed short      Altitude;       // Altitude réelle de l'emplacement (en mètres). Le BMP180 demande un signed short...
} AltitudeData, *AltitudePtr;

typedef struct
{
  unsigned long     StartTimeStamp;   // TimeStamp du début de bufférisation de ces mesures en mode "SALVE"
  unsigned char     NbSec;            // Nombre de secondes d'enregistrement ici (1 à 5).
  unsigned char     FreqLum;          // Fréquence des Conversions Luminosité utilisée ici.
  unsigned char     FreqTemp;         // Fréquence des Conversions Température utilisée ici.
  unsigned char     FreqHum;          // Fréquence des Conversions Humidité utilisée ici.
  unsigned short    DataLum[20][5];   // Mesures de Luminosité, max 20Hz pendant Max 5 sec.
  signed short      DataTemp[20][5];  // Mesures de Température, max 20Hz pendant Max 5 sec.
  unsigned short    DataHum[20][5];   // Mesures d'Humidité, max 20Hz pendant Max 5 sec.

  unsigned short    DataVBAT[5];      // Option : Mesures de tension Batterie à 1Hz, Max 5 sec.
  unsigned long     DataBARO[5];      // Option : Mesures Pression Atmosphérique à 1Hz, Max 5 sec.
  signed short      DataTempInt[5];   // Option : Mesures de Température Interne de la Station à 1Hz, Max 5 sec.
} SalveDATARec, *SalveDATAPtr;        // sizeof(SalveDATARec) = 648 octets (à DataTempInt) - En RAM

typedef struct
{
  unsigned char     ModeTravail;
  boolean           SendLum;
  boolean           SendTempExt;
  boolean           SendHum;
  boolean           SendBaro;
  boolean           SendVBAT;
  boolean           SendTempInt;
  boolean           SendTimeStamp;
} PrivateDataRec, *PrivateDataPtr;

typedef enum mode_travail {
    MODE_BASIC        = 1,    // Envoi de toutes les mesures disponibles sur le port COM, sans interaction avec l'Utilisateur
    MODE_TEMPS_REEL   = 2,    // Envoi des mesures demandées par le logiciel PC, réception des modifications de réglages
    MODE_SALVES       = 3,    // Bufférisation des mesures en RAM (1 à 5 secondes), puis envoi au logiciel PC
    MODE_ONDEMAND     = 4,    // Bufférisation des mesures en FRAM (temps selon capacité FRAM), puis envoi au logiciel PC
    MODE_EVENEMENTIEL = 5,    // Option du Groupe : Envoi des mesures uniquement si en-dessous d'un seuil paramétrable
    MODE_CONSOLE      = 6,    // Interface Utilisateur avec logiciel "puTTY" sur PC, en émulation VT220.
    MODE_METEO        = 7     // Conversions à intervalles beaucoup plus longs, pour mémoriser plusieurs jours de mesures en FRAM
} mode_travail;

typedef struct
{
  unsigned short    PrefsVersion;   // Version du Record des Préférences
  AltitudeData      GeoPosition[4]; // On peut enregistrer 4 emplacements différents
  unsigned char     GeoPosSelect;   // Emplacement actif (0...3)
  unsigned short    EtalonnageBARO; // Etalonnage du Baromètre.
  unsigned short    VCCvoltage;     // Valeur de la tension en sortie de régulateur 3.3V (en mV) - 3325 = 3.325V (utilisé pour calcul Température et Humidité).
  unsigned short    EtalonnageVBAT; // Etalonnage de la conversion ADC->Tension pour VBAT.
  unsigned short    EtalonnageLum;  // Variable servant à étalonner la conversion ADC->Lux pour Capteur de Luminosité.
  unsigned char     FrequenceLum;   // Fréquence des Conversions Luminosité (1 à 20Hz) pour Mode Temps Réel
  unsigned short    EtalonnageTemp; // Variable servant à étalonner la conversion ADC->°C pour Capteur de Température.
  unsigned char     FrequenceTemp;  // Fréquence des Conversions Température (1 à 20Hz) pour Mode Temps Réel
  unsigned short    EtalonnageHum;  // Variable servant à étalonner la conversion ADC->%RH pour Capteur d'Humidité.
  unsigned char     FrequenceHum;   // Fréquence des Conversions Humidité (1 à 20Hz) pour Mode Temps Réel
  unsigned char     ModeTravail;    // Mode de travail de la Station (voir mode_travail ci-dessus)
  unsigned char     DureeSalve;     // Durée de bufférisation en RAM des données en mode "SALVE" (1 à 5 secondes)
  unsigned short    ConsigneLum;    // Seuil de Luminosité en-dessous duquel on envoie les mesures Luminosité vers le PC, en mode "EVENEMENTIEL"
  unsigned short    ConsigneTemp;   // Seuil de Température en-dessous duquel on envoie les mesures Température vers le PC, en mode "EVENEMENTIEL"
  unsigned short    ConsigneHum;    // Seuil d'Humidité en-dessous duquel on envoie les mesures Humidité vers le PC, en mode "EVENEMENTIEL"
}PrefsRec, *PrefsPtr;               // sizeof(PrefsRec) = 90 octets (à ConsigneHum) - En FRAM

#define FRAM_PREFS_BASEADDR     0x0000    // Nos Prefs sont stockées en début de FRAM, à l'adresse 0x0000
#define FRAM_DATA_BASEADDR      0x0100    // Les mesures sont stockées en FRAM à partir de l'adresse 0x0100, on réserve donc 256 octets pour les Prefs.



// *************************************************************
// *   Variables Globales et Macros des Tâches FreeRTOS        *
// *************************************************************

// Variables Globales Générales

unsigned long             FRAM_AdresseMax = 0L;  // Nécessaire : Dernière adresse valide dans la FRAM, selon sa capacité
unsigned long             FRAM_Densite = 0L;     // Nécessaire : Densité de la FRAM en Kbit

DateTimeRec               Horloge;
const char                JourdelaSemaine[7][10] = {"Lundi", "Mardi", "Mercredi", "Jeudi", "Vendredi","Samedi","Dimanche"};
const char                Mois[12][10] = {"Janvier", "Février", "Mars", "Avril", "Mai", "Juin", "Juillet", "Août", "Septembre", "Octobre", "Novembre", "Décembre"};
boolean                   HorlogeGood;

BARORec                   Barometer;
boolean                   BarometreOK;

PrefsRec                  MyPrefs;

SalveDATARec              Salve;

volatile unsigned short   DMA_VBAT_Buffer[10] __attribute__((aligned (16)));    // Buffer pour 10 Conversions A/D - Mesure de tension Batterie - rempli par DMA
unsigned short            TensionBatterie;        // Tension de la batterie, en 1/100è de Volt (396 = 3.96V)
volatile boolean          VbatDMAComplete;        // Flag indiquant que les conversions A/D pour la tension Batterie sont terminées, et transférées par DMA.
boolean                   VbatGood;               // Flag indiquand que la variable TensionBatterie contient une valeur correcte.

volatile unsigned short   ValeurCapteur;          // Mis à jour par l'ISR d'ADC2, contient le résultat de la Conversion A/D du Capteur sélectionné
volatile boolean          ConversionComplete;     // Flag mis à jour par l'ISR d'ADC2 pour indiquer qu'une nouvelle Conversion A/D est disponible
unsigned char             CapteurSelect;          // Capteur en cours de Conversion : 0 = Aucun, 1 = Luminosité, 2 = Température, 3 = Humidité

volatile boolean          MustStartLum;           // Flag mis à jour par l'ISR de TIM2, indiquant qu'on doit lancer les acquisitions de Luminosité
unsigned short            LumBuff[20];            // Buffer pour "Moyenner" (10 effectuées - 2 utilisées) = 8 Conversions de Luminosité
unsigned short            C_LumLux;               // Valeur de Luminosité, moyenne de 8 Conversions, calculée en Lux par AnalogCapteursTask().
boolean                   NewLuminositeDispo;     // Flag pour nouvelle valeur de Luminosité disponible (acquise et calculée)

volatile boolean          MustStartTemper;        // Flag mis à jour par l'ISR de TIM3, indiquant qu'on doit lancer les acquisitions de Température
unsigned short            TemperBuff[10];         // Buffer pour "Moyenner" (10 effectuées - 2 utilisées) = 8 Conversions de Température
short                     C_Temperature;          // Valeur de Température Extérieure, moyenne de 8 Conversions (signé pour T° négatives)
boolean                   NewTemperatureDispo;    // Flag pour nouvelle valeur de Température disponible (acquise et calculée)

volatile boolean          MustStartHum;           // Flag mis à jour par l'ISR de TIM4, indiquant qu'on doit lancer les acquisitions d'Humidité
unsigned short            HumBuff[10];            // Buffer pour "Moyenner" (10 effectuées - 2 utilisées) = 8 Conversions d'Humidité
unsigned short            C_Humidite;             // Valeur d'Humidité, moyenne de 8 Conversions
boolean                   NewHumiditeDispo;       // Flag pour nouvelle valeur d'Humidité disponible (acquise et calculée)

unsigned char             ModeTravail;            // Mode de travail (lu depuis les Prefs au boot) : MODE_BASIC, MODE_TEMPS_REEL, MODE_SALVES, MODE_ONDEMAND, MODE_EVENEMENTIEL, MODE_CONSOLE ou MODE_METEO
boolean                   ModeSalvesInited = false; // Flag indiquant qu'il faut réinitialiser le SalveDataRec "Salve", car on vient de "basculer" en mode "par Salves". 


// Variables Globales pour FreeRTOS :

TaskHandle_t              BlinkTask,HorlogeTask,BarometreTask,BatteryMonTask,CapteursTask,MainTask;
TaskHandle_t              PCInterfaceTask;
SemaphoreHandle_t         Mutex_Horloge   = NULL;
SemaphoreHandle_t         Mutex_I2C1      = NULL;
SemaphoreHandle_t         Mutex_Barometre = NULL;
SemaphoreHandle_t         Mutex_I2C2      = NULL;


// Macros pour FreeRTOS :

#define SLEEPTASK(Delay)    vTaskDelay(((unsigned long) Delay * configTICK_RATE_HZ) / 1000L)
#define TAKE_MUTEX(Mutex)   while (!xSemaphoreTake(Mutex,(TickType_t) 10)) {taskYIELD();}
#define GIVE_MUTEX(Mutex)   xSemaphoreGive(Mutex)


// *************************************************************
// *             Déclaration des routines externes             *
// *************************************************************

// Les routines externes en C doivent être déclarées comme [extern "C"] pour que le Sketch Arduino (utilisant le compilateur C++) les retrouve.
// ("Name-mangling"), voir : https://forum.arduino.cc/index.php?topic=45003.0
// Recherche Google : "arduino ide undefined reference" en cas de problème...


#ifdef __cplusplus
extern "C" {
#endif

extern void InterfacePCTask(void *pvParameters);

#ifdef __cplusplus
}
#endif




#endif  // (#ifndef STATION_METEO_H)


