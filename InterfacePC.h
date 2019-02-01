// Fichier "InterfacePC.h : Defines pour l'interfaçage de la Station Météo avec le PC.
// Pour IDE Arduino version 1.8.5



#define SECONDS_PER_DAY             86400L
#define SECONDS_FROM_1970_TO_2000   946684800

const unsigned char daysInMonth [] = { 31,28,31,30,31,30,31,31,30,31,30,31 };

// Commandes reconnues :
// --------------------

// *********************************************************************************
// * Commandes concernant le Mode de Travail et la sélection des Mesures à envoyer *
// *********************************************************************************

// 1)  Esc!Modex (1 <= x <= 7)     Changement de Mode de Travail de la Station (voir Modes de Travail ci-dessous)
//                                 Esc!Mode1 : Bascule en "MODE_BASIC"
//                                 Esc!Mode2 : Bascule en "MODE_TEMPS_REEL"
//                                 Esc!Mode3 : Bascule en "MODE_SALVES"
//                                 Esc!Mode4 : Bascule en "MODE_ONDEMAND"
//                                 Esc!Mode5 : Bascule en "MODE_EVENEMENTIEL"
//                                 Esc!Mode6 : Bascule en "MODE_CONSOLE"
//                                 Esc!Mode7 : Bascule en "MODE_METEO"

// 2)  Esc!Datax (x est codé sur 7 bits, pour sélectionner les Mesures à envoyer), en "MODE_TEMPS_REEL"
//                                 Si bit 0 à "1", envoi des Mesures de Luminosité             (à une fréquence de 1 à 20Hz)
//                                 Si bit 1 à "1", envoi des Mesures de Température Externe    (à une fréquence de 1 à 20Hz)
//                                 Si bit 2 à "1", envoi des Mesures d'Humidité                (à une fréquence de 1 à 20Hz)
//                                 Si bit 3 à "1", envoi des Mesures de Pression Atmosphérique (à une fréquence de 1Hz)
//                                 Si bit 4 à "1", envoi des Mesures de Tension Batterie       (à une fréquence de 1Hz)
//                                 Si bit 5 à "1", envoi des Mesures de Température Intérieure (à une fréquence de 1Hz)
//                                 Si bit 6 à "1", envoi de l'Horodatage                       (à une fréquence de 1Hz)

// Format de l'envoi des Mesures au PC, après réception de cette Commande :

// En-tête si Horodatage activé  : @:1234567890<TAB> (envoyé avant chacune des 3 mesures du Projet : Luminosité, Température Extérieure, Humidité).

// Mesure Luminosité             : L:xxxxx<CR><LF>  si Horodatage activé le PC recevra : @:1234567890<TAB>L:xxxxx<CR><LF> (xxxx représente la Luminosité en Lux)
// Mesure Température Extérieure : T:xx.x<CR><LF>   si Horodatage activé le PC recevra : @:1234567890<TAB>T:xx.x<CR><LF>  (xx.x représente la Température en °C)
// Mesure Humidité               : H:xx<CR><LF>     si Horodatage activé le PC recevra : @:1234567890<TAB>Hxx<CR><LF>     (xx   représente l'Humidité en %)
// Mesure Pression Atmosphérique : P:xxxx.x<CR><LF> (xxxx.x représente la Pression Atmosphérique en hPa)
// Mesure Tension Batterie       : V:x.xx<CR><LF>   (x.xx   représente la Tension Batterie en Volts)
// Mesure Température Intérieure : I:xx.x<CR><LF>   (xx.x   représente la Température Interne en °C)

// >>> ATTENTION : Les Températures peuvent être NEGATIVES <<<

// Exemple : si la Température Extérieure est de -5.8°C, le PC recevra la Chaîne de caractères suivante :
// T:-5.8<CR><LF>   Si Horodatage activé, le PC recevra : @:1234567890<TAB>T:-5.8<CR><LF>
// C'est pareil pour la Température Externe et pour la Température Intérieure.

// ****************************************************************************************
// * Commandes concernant les Réglages de Fréquence d'échantillonnage et de Bufférisation *
// ****************************************************************************************

// 3)  Esc!FLxx (xx = 01 à 20, Fréquence d'Echantilonnage de la Luminosité). Enregistré dans le Record des Prefs, mais pas encore sauvegardé en FRAM.
//                                Exemple : Esc!FL05 définit une fréquence d'échantillonnage Luminosité à 5Hz.

// 4)  Esc!FTxx (xx = 01 à 20, Fréquence d'Echantilonnage de la Température). Enregistré dans le Record des Prefs, mais pas encore sauvegardé en FRAM.
//                                Exemple : Esc!FT10 définit une fréquence d'échantillonnage Température à 10Hz.

// 5)  Esc!FHxx (xx = 01 à 20, Fréquence d'Echantilonnage de l'Humidité). Enregistré dans le Record des Prefs, mais pas encore sauvegardé en FRAM.
//                                Exemple : Esc!FH15 définit une fréquence d'échantillonnage Humidité à 15Hz.

// 6)  Esc!Sx   (x = 1 à 5,    Durée de Bufférisation des Mesures pour le Mode "Salve"). Enregistré dans le Record des Prefs, mais pas encore sauvegardé en FRAM.
//                                Exemple : Esc!S5 définit une durée de Bufférisation de 5 secondes.

// ********************************
// * Commandes concernant la FRAM *
// ********************************

// 7)  Esc!PW                  (Prefs Write). Sauvegarde des Prefs en FRAM.

// 8)  Esc!PR                  (Prefs Reset). Réinitialise les Prefs en FRAM et en RAM aux valeurs par défaut.

// 9)  Esc!SI                  (Storage Init). Réinitialise le Buffer Circulaire de stockage des Mesures en FRAM (toutes les Mesures sont perdues).

// 10) Esc!SC                  (Storage Capacity). Demande la Capacité maximale (en Nb de Mesures) du Buffer Circulaire en FRAM. Dépend de la taille de la FRAM utilisée.

// 11) Esc!SN                  (Storage Number). Demande le Nombre de Mesures stockées dans le Buffer Circulaire en FRAM.

// 12) Esc!SS                  (Storage Send). Pour recevoir toutes les Mesures stockées dans le Buffer Circulaire en FRAM. Ne fait pas de RAZ du buffer, pour celà il faut envoyer Esc!SI si désiré.

// *********************************
// * Commandes Générales (courtes) *
// *********************************

// 13) EscDS                   (Data Stop). Arrête l'envoi des Mesures, quel que soit le Mode de travail.

// 14) EscDG                   (Data Go). Redémarre l'envoi des Mesures (stoppé par EscDS), quel que soit le mode de travail.

// 15) EscMx                   (Mesure x). Demande l'envoi de la Mesure sélectionnée, immédiatement (sans Horodatage, mais au même format que d'habitude). La valeur la plus récente est envoyée.
//                             Si x = '1', Luminosité, si x = '2' Température Extérieure, si x = '3' Humidité, si x = '4' Pression Atmosphérique, si x = '5' Tension Batterie, si x = '6' Température Interne.

// 16) EscV                    Demande l'envoi de la Version du Firmware de la Station Météo. Retourne une chaîne comme 1.01<CR><LF> (pour la Version 1.01)


// *************************
// * Commandes pour la RTC *
// *************************


// **************************
// * Commandes d'Etalonnage *
// **************************



