// Fichier "StationMeteo.ino : Sketch (programme principal) du Projet Station Météo. Infres 9-3°A ED-2018-2019
// Pour IDE Arduino version 1.8.5

// "Board" à utiliser dans Arduino IDE : Maple Mini, Bootloader 2.0 (20k RAM, 120k Flash), 72MHz(Normal), Smallest(default) on COMx
// "MapleMini" se trouve dans "STM32F1 Boards (STM32duino.com)"

// ATTENTION : Pour que le PC puisse communiquer avec ce projet, pour mettre à jour le programme en Flash par le Driver DFU (Device Firmware Upgrade) sur USB
//             ou pour recevoir des données sur un logiciel PC adapté, par port COMx virtuel sur USB, il FAUT installer les Drivers USB fournis (pour Windows 7 et supérieurs).
//             Ces Drivers se trouvent dans le Dossier "Installation IDE Arduino", fichiers fournis.

// Ce logiciel utilise la librairie Arduino "Serial", pour communiquer avec le PC au-travers d'un port série virtuel sur USB.
// L'implémentation "personnelle" d'un tel outil dépasse le cadre de ce Projet.
// On utilise également la librairie "FreeRTOS" version 9.0.0 "MapleFreeRTOS900".

// Cette implémentation du STM32F103 sur Framework Arduino repose généralement sur la librairie "libMaple".
// Les seules fonctions utilisées dans cette librairie sont nvic_irq_enable() [Interruptions] et powf() [Maths].
// On utilise également les fonctions "delay()", "millis()" et "delayMicroseconds()" du Framework Arduino pour gérer de petits délais.

#include "StationMeteo.h"


// Ici se trouve la gestion des Modes de travail suivants :
// -------------------------------------------------------

//  MODE_BASIC        : Envoi de toutes les mesures disponibles sur le port COM, sans interaction avec l'Utilisateur


// La procédure setup() est appelée au démarrage du MCU, une seule fois. On place ici toutes les initialisations.

void setup() {
  
  Serial.begin(2000000);                        // Ouverture du Port Série Virtuel sur USB : 8bits, 1stop, sans parité, 2000000 bauds.
 
  // Création des Mutex de FreeRTOS

  Mutex_Horloge   = xSemaphoreCreateMutex();    // On crée un Mutex pour accéder à l'Horloge RTC à partir de différentes tâches
  HorlogeGood = false;                          // On n'a pas encore mis à jour le DateTimeRec en variable Globale...

  Mutex_Barometre = xSemaphoreCreateMutex();    // On crée un Mutex pour accéder au Baromètre à partir de différentes tâches
  BarometreOK = false;                          // Le Baromètre n'a pas encore été initialisé

  VbatGood = false;                             // La détermination de la Tension Batterie n'est pas encore effectuée.

  NewLuminositeDispo  = false;                  // Pour le moment, aucune valeur de Luminosité n'est disponible.
  NewTemperatureDispo = false;                  // Pour le moment, aucune valeur de Température n'est disponible.
  NewHumiditeDispo    = false;                  // Pour le moment, aucune valeur d'Humidité n'est disponible.
  
  // Initialisations Hardware bas niveau
  
  GPIO_Setup();                                       // Initialisation des Ports d'Entrée/Sortie du MCU.
  CAPTEURS_POWERON();                                 // Les capteurs Analogiques sont alimentés par défaut. A optimiser pour une plus faible consommation électrique.
  USER_LED_RED_ON();
  USER_LED_GREEN_ON();
  
  FRAM_Setup();                                       // Initialisation de la FRAM sur bus SPI1.

  //SetDefaultPrefs();                                  // Enregistre nos Préférences par défaut dans la FRAM (A activer en cas de changement de structure des Prefs).
  
  ReadMyPrefs();                                      // Lit nos Préférences dans la FRAM
  if (MyPrefs.PrefsVersion == 0) SetDefaultPrefs();   // Enregistre nos Préférences par défaut dans la FRAM si elle est vierge.
                                                      // Les données d'une FRAM vierge sont à 0x00, celles d'une Flash vierge sont à 0xFF...
  ModeTravail = MyPrefs.ModeTravail;
  
  RTC_Setup();                                        // Initialisation de l'Horloge RTC sur bus I2C1.
  //RTC_WriteRegister(RTC_DAYOFWEEK_REG,3);           // Enregistre le jour de la semaine (1 = Lundi, 7 = Dimanche) - A utiliser pour mise à jour Horloge...
  
  
  BARO_Setup();                                       // Initialisation du Baromètre BMP180 sur bus I2C2.
  VBATMon_Setup();                                    // Initialisation du Convertisseur A/D ADC1 pour mesurer la tension Batterie, transfert des Conversions par DMA.
  Capteurs_Setup();                                   // Initialisation du Convertisseur A/D ADC2 pour mesurer les valeurs de nos 3 Capteurs analogiques (Luminosité, Température et Humidité).

  
  // Création des Tâches de FreeRTOS (6 Tâches)
  // Une tâche de priorité supérieure bloquée bloque les tâches de priorité inférieure, mais pas celles de priorité >=
  
  xTaskCreate(LEDFlashTask,     // On crée la tâche qui fait clignoter la Led du Maple Mini
    "LedFlash",
    configMINIMAL_STACK_SIZE,
    NULL,
    tskIDLE_PRIORITY + 1,
    &BlinkTask);

  xTaskCreate(UpdateHorlogeTask,     // On crée la tâche qui met à jour l'Horloge
    "Horloge", 
    configMINIMAL_STACK_SIZE,
    NULL,
    tskIDLE_PRIORITY + 1,
    &HorlogeTask);

  xTaskCreate(UpdateBarometerTask,   // On crée la tâche qui met à jour le Baromètre
    "Barometre", 
    configMINIMAL_STACK_SIZE,
    NULL,
    tskIDLE_PRIORITY + 1,
    &BarometreTask);

  xTaskCreate(VBATMonitorTask,       // On crée la tâche qui surveille la tension Batterie
    "VBATMonitor", 
    configMINIMAL_STACK_SIZE,
    NULL,
    tskIDLE_PRIORITY + 1,
    &BatteryMonTask);

  xTaskCreate(AnalogCapteursTask,       // On crée la tâche qui gère les 3 Capteurs Analogiques
    "AnalogCapteurs", 
    configMINIMAL_STACK_SIZE + 128,
    NULL,
    tskIDLE_PRIORITY + 1,
    &CapteursTask);

  xTaskCreate(InterfacePCTask,       // On crée la tâche qui gère l'interface avec le logiciel spécifique sur PC
    "InterfacePC", 
    configMINIMAL_STACK_SIZE + 128,
    NULL,
    tskIDLE_PRIORITY + 1,
    &PCInterfaceTask);

  xTaskCreate(ProgPrincipalTask,    // On crée la tâche du Corps Principal du Projet (l'équivalent de Main() en C).
    "MainProg",
    configMINIMAL_STACK_SIZE + 128,
    NULL,
    tskIDLE_PRIORITY + 1,           // On lui affecte une Priorité [un peu supérieure].
    &MainTask);

  vTaskStartScheduler();            // On démarre le Scheduler de FreeRTOS : les tâches définies commencent à  s'exécuter maintenant...
}

// La procédure Arduino loop() n'est pas appelée sous FreeRTOS. Seules les Tâches créées sont exécutées...

void loop() {
}


// ProgPrincipalTask() : Corps principal [Main()] du Projet, sous forme de tâche FreeRTOS.

void ProgPrincipalTask(void *pvParameters)
{
int             Compteur,i;
unsigned char   buff[9];
unsigned long   AdresseCompteur,HectoPascals;
boolean         Success,HeaderDisplayed;
unsigned char   ValeurRegistre;
short           Temperature,temp;
unsigned long   StartTime,CurTime,NextDisplayTime;
unsigned short  TemperatureBARO;
unsigned char   Displayed,MaxDisplay;

  #define UPDATE_DELAY_MAIN   1000            // 1 seconde entre chaque affichage
  
  StartTime = millis() + 3000L;               // 3 secondes, le temps de démarrer la Console et de charger le Condensateur tampon de mesure de VBAT...
  while (millis() < StartTime) taskYIELD();   // Ce temps CPU "donné" permet aux autres Tâches FreeRTOS de démarrer  
  Compteur = 0;
  CurTime = millis();
  NextDisplayTime = CurTime + UPDATE_DELAY_MAIN;           // 1 affichage par seconde, pour l'entête
  HeaderDisplayed = false;
  MaxDisplay = MyPrefs.FrequenceLum + MyPrefs.FrequenceTemp + MyPrefs.FrequenceHum; // Nombre de Mesures à afficher chaque seconde, en fonction des Prefs (3...60)
  
  Displayed = MaxDisplay;   // On initialise à MaxDisplay pour que l'entête s'affiche la première fois.
  
  while (1) {
      if (ModeTravail == MODE_BASIC) {            // On gère ici le Mode "MODE_BASIC"...
          CurTime = millis();
          if ((CurTime >= NextDisplayTime) && (Displayed >= MaxDisplay)) {       // Chaque seconde, on affiche l'entête, en effacant l'écran, si nos 3 Mesures ont été affichées...
              while (! NewLuminositeDispo) taskYIELD();  // Attente Nouvelles Mesures Dispo (Synchronisation avec les Timers)
              
              HeaderDisplayed = true;
              Displayed = 0;  // On efface l'écran, donc on n'a aucune des 3 Mesures affichées
              CLS();
              
              Serial.print("Bonjour ! n° ");
              Serial.print(Compteur++);
              Serial.print(" Firmware version ");
              Serial.println(STATION_METEO_VERSION);
     
              if (HorlogeGood == true) { // Le DateTimeRec a été initialisé par la tâche UpdateHorlogeTask()
                  TAKE_MUTEX(Mutex_Horloge);
                  
                    Serial.print("Nous sommes le ");
                    Serial.print(JourdelaSemaine[Horloge.JourSem - 1]); // JourSem va de 1 à 7 dans la RTC, notre tableau des jours va de 0 à 7.
                    Serial.print(" ");
                    if (Horloge.Date < 10) Serial.print("0");
                    Serial.print(Horloge.Date);
                    Serial.print(" ");
                    Serial.print(Mois[Horloge.Mois - 1]);
                    Serial.print(" ");
                    if (Horloge.Annee < 2000) Serial.print("2");
                    if (Horloge.Annee < 100) Serial.print("0");
                    if (Horloge.Annee < 10) Serial.print("0");
                    Serial.print(Horloge.Annee);
                    Serial.print(", il est ");
                    if (Horloge.Heure < 10) Serial.print("0");
                    Serial.print(Horloge.Heure);
                    Serial.print(":");
                    if (Horloge.Minutes < 10) Serial.print("0");
                    Serial.print(Horloge.Minutes);
                    Serial.print(":");
                    if (Horloge.Secondes < 10) Serial.print("0");
                    Serial.println(Horloge.Secondes);
                    
                  GIVE_MUTEX(Mutex_Horloge);
              }
    
              if (BarometreOK) {
                TAKE_MUTEX(Mutex_Barometre);
                
                    if (Barometer.MeteoPressure != 0L) {
                      Serial.print("Station Météo de ");
                      Serial.print((char*) MyPrefs.GeoPosition[MyPrefs.GeoPosSelect].Emplacement);
                      Serial.print(" (Altitude : ");
                      Serial.print(MyPrefs.GeoPosition[MyPrefs.GeoPosSelect].Altitude);
                      Serial.println(" m)");
          
                      Temperature = Barometer.Temperature;
                      Serial.print("La Température interne de la Station Météo est de ");
                      Serial.print(Temperature / 10);
                      Serial.print(".");
                      if (Temperature < 0) Temperature = -(Temperature);
                      Serial.print(Temperature % 10);
                      Serial.println("°C\n\r");
          
                      if (VbatGood) {  // VbatGood est activé par la Tache VBATMonitorTask()
                        Serial.print("Tension Batterie Principale  : ");
                        Serial.print(TensionBatterie / 100);
                        Serial.print(".");
                        temp = (TensionBatterie % 100);
                        if (temp < 10) Serial.print("0");
                        Serial.print(temp);
                        Serial.println(" V");
                      }
          
                      HectoPascals = Barometer.MeteoPressure / 10;
                      Serial.print("Pression Atmosphérique       : ");
                      Serial.print(HectoPascals / 10);
                      Serial.print(".");
                      Serial.print(HectoPascals % 10);
                      Serial.println(" hPa");
                      Serial.println();      
                    }
              
                GIVE_MUTEX(Mutex_Barometre);
              }
              NextDisplayTime = CurTime + UPDATE_DELAY_MAIN;   // 1 affichage par seconde, pour l'entête
          }

          if (HeaderDisplayed) {
              if (NewLuminositeDispo && (Displayed < MaxDisplay)) {   // On doit envoyer une nouvelle valeur de Luminosité
                if (HorlogeGood == true) { // Le DateTimeRec a été initialisé par la tâche UpdateHorlogeTask()
                  TAKE_MUTEX(Mutex_Horloge);
                    if (Horloge.Heure < 10) Serial.print("0");
                    Serial.print(Horloge.Heure);
                    Serial.print(":");
                    if (Horloge.Minutes < 10) Serial.print("0");
                    Serial.print(Horloge.Minutes);
                    Serial.print(":");
                    if (Horloge.Secondes < 10) Serial.print("0");
                    Serial.print(Horloge.Secondes);
                    Serial.print(" ");
                  GIVE_MUTEX(Mutex_Horloge);
                }
                Serial.print("Luminosité          : ");
                Serial.print(C_LumLux);
                Serial.println(" Lux");
                NewLuminositeDispo = false;
                Displayed++;  // 1 Mesure de plus d'affichée
              }
        
              if (NewTemperatureDispo && (Displayed < MaxDisplay)) {   // On doit envoyer une nouvelle valeur de Température
                if (HorlogeGood == true) { // Le DateTimeRec a été initialisé par la tâche UpdateHorlogeTask()
                  TAKE_MUTEX(Mutex_Horloge);
                    if (Horloge.Heure < 10) Serial.print("0");
                    Serial.print(Horloge.Heure);
                    Serial.print(":");
                    if (Horloge.Minutes < 10) Serial.print("0");
                    Serial.print(Horloge.Minutes);
                    Serial.print(":");
                    if (Horloge.Secondes < 10) Serial.print("0");
                    Serial.print(Horloge.Secondes);
                    Serial.print(" ");
                  GIVE_MUTEX(Mutex_Horloge);
                }
                Temperature  = C_Temperature;
                Serial.print("Température Externe : ");
                Serial.print(Temperature / 10);
                Serial.print(".");
                if (Temperature < 0) Temperature = -(Temperature);
                Serial.print(Temperature % 10);
                Serial.println("°C");
               NewTemperatureDispo = false;
               Displayed++;  // 1 Mesure de plus d'affichée
              }
        
              if (NewHumiditeDispo && (Displayed < MaxDisplay)) {   // On doit envoyer une nouvelle valeur d'Humidité
                if (HorlogeGood == true) { // Le DateTimeRec a été initialisé par la tâche UpdateHorlogeTask()
                  TAKE_MUTEX(Mutex_Horloge);
                    if (Horloge.Heure < 10) Serial.print("0");
                    Serial.print(Horloge.Heure);
                    Serial.print(":");
                    if (Horloge.Minutes < 10) Serial.print("0");
                    Serial.print(Horloge.Minutes);
                    Serial.print(":");
                    if (Horloge.Secondes < 10) Serial.print("0");
                    Serial.print(Horloge.Secondes);
                    Serial.print(" ");
                  GIVE_MUTEX(Mutex_Horloge);
                }
                Serial.print("Humidité            : ");
                Serial.print(C_Humidite);
                Serial.println("%");
                NewHumiditeDispo = false;
                Displayed++;  // 1 Mesure de plus d'affichée
              }
          }
          
        if (Displayed == MaxDisplay) while (! NewTemperatureDispo) taskYIELD();
      }

      SLEEPTASK(1);
   }
}


// UpdateHorlogeTask() : Tâche qui met à jour la structure globale (DateTimeRec) Horloge, 2 fois par seconde, priorité basse. Lecture de la RTC sur bus I2C1.

void UpdateHorlogeTask(void *pvParameters)
{
DateTimeRec     localTime;

  // On utilise un Mutex (Mutual Exclusion) lors de la mise à jour du DateTimeRec global.
  // La tâche UpdateHorloge "prend" le Mutex quand elle met à jour le DateTimeRec global, la tâche Principale doit attendre avant d'y accéder.
  // La tâche Principale "prend" le Mutex quand elle accède à l'Horloge, la tâche UpdateHorloge doit attendre avant de la mettre à jour.
  // Indispensable avec un RTOS.
  // Temps de lecture des données pour Mise à Jour du DateTimeRec sur bus I2C : 260µs à 400KHz (équivalent à 18700 instructions, soit 0.026% de temps Processeur @ 72MHz).

  while (1) {
      TAKE_MUTEX(Mutex_Horloge);
          RTC_ReadDateTime(&localTime);
          Horloge.Secondes  = localTime.Secondes;
          Horloge.Minutes   = localTime.Minutes;
          Horloge.Heure     = localTime.Heure;
          Horloge.JourSem   = localTime.JourSem;
          Horloge.Date      = localTime.Date;
          Horloge.Mois      = localTime.Mois;
          Horloge.Annee     = localTime.Annee;
      GIVE_MUTEX(Mutex_Horloge);
      
      HorlogeGood = true;       // C'est bon, le DateTimeRec est à jour...

   SLEEPTASK(500);    // La tâche s'endort pendant 500 ms  - Macro définie dans "StationMeteo.h".
  }
}

// UpdateBarometerTask() : Tâche qui gère la lecture du Baromètre BMP180 sur bus I2C2 (Température et Pression), et effectue les calculs.
//                         Mise à jour : 1 fois par seconde

void UpdateBarometerTask(void *pvParameters)
{
unsigned short    UT;
signed short      RealTemp;
signed long       _B5;
unsigned long     myUP,AbsPress,Press0;
  
  while (1) {  
    TAKE_MUTEX(Mutex_Barometre);
    
      if (BarometreOK == false) {    // On n'a pas encore étalonné le Baromètre
          BARO_Calibrate();
          BarometreOK = true;     
      }

      // On met à jour la température réelle, utilisée pour la Compensation de la Pression Atmosphérique
      // en fonction de la Température (variable _B5).
      // Temps de lecture des données pour Mise à Jour de la Pression sur bus I2C : (5ms Temp + 14 ms (Resol = 2) Pression = 20ms à 400KHz) (équivalent à 1440000 instructions, soit 2% de temps Processeur @ 72MHz).
      
      UT = BARO_ReadUT();
      Barometer.UT = UT;
            
      UT2Temp((BAROCalibPtr)(void*)&Barometer,UT,&RealTemp,&_B5);
      Barometer.Temperature = RealTemp;
      Barometer._B5         = _B5;

      // On lit la Pression Atmosphérique "Brute Capteur".
      
      myUP = BARO_ReadUP();      
      Barometer.UP = myUP;

      // On convertit cette Pression en Pression "Absolue" (réelle) et "Météo" (ramenée au Niveau de la mer).
      // Les calculs (fastidieux) sont fournis par le fabricant du composant BMP180.
      
      UP2Pressure((BAROCalibPtr)(void*)&Barometer,myUP,Barometer.Altitude,Barometer.OSS,&AbsPress,&Press0);
      
      Barometer.AbsPressure = AbsPress; // En Pascals (97525 = 975.25 hPa = 975.25 mBar) : Pression Atmosphérique Absolue.
      Barometer.MeteoPressure = Press0; // En Pascals (97525 = 975.25 hPa = 975.25 mBar) : Pression Atmosphérique rapportée au niveau de la mer (compensée en Altitude).
        
    GIVE_MUTEX(Mutex_Barometre);
    
    SLEEPTASK(1000);
  }
}

// VBATMonitorTask() : Tâche qui gère les Conversions A/D, le DMA, pour calculer la tension Batterie (VBAT). Gère aussi les Leds du voyant Marche/Arrêt.
//                     Mise à jour : 1 fois par seconde

void VBATMonitorTask(void *pvParameters)
{
unsigned char         i,j,n;
unsigned short        temp;
float                 voltage;
dma_channel_reg_map   *DMA1_Channel1;
boolean               VBATValideVu;

  VBATValideVu = false;   // Pour le moment, on n'a pas encore vu de tension batterie valide.
  
  while (1) {
    if (VbatDMAComplete) {  // Le DMA et l'ADC1 sont arrêtés, après avoir effectué 10 Conversions, stockées dans DMA_VBAT_Buffer[] par le Controleur DMA.

        // 1) On trie le tableau DMA_VBAT_Buffer[] (tri à bulles) par order croissant. On répète le tri à bulles ici car il y a un souci sinon comme DMA_VBAT_Buffer est "volatile"...
        
        n = 10; // 10 éléments à trier
        for (i = 0; i < n-1; i++) {
          for (j = 0; j < n-i-1; j++) {
            if (DMA_VBAT_Buffer[j] > DMA_VBAT_Buffer[j+1]) {
              temp = DMA_VBAT_Buffer[j];
              DMA_VBAT_Buffer[j] = DMA_VBAT_Buffer[j+1];
              DMA_VBAT_Buffer[j+1] = temp;
            }
          }
        }
        
        // 2) On fait la moyenne des Conversions 2 à 9 (on écarte la plus petite et la plus grande valeur, provenant du "Bruit Digital".

        temp = 0;
        for (i = 1 ; i < (n - 1) ; i++) temp += DMA_VBAT_Buffer[i];
        temp = temp / (n - 2);      // Moyenne
        voltage = (float) temp;     // On effectue les calculs en virgule flottante pour plus de précision.

        // 3) On convertit la valeur A/D en tension

        // 4.202V (VBAT) -> 2.840V (Sortie Pont Diviseur) => 3556 ADC counts
        // Pour calibrer cette transformation ADC -> Tension, il suffit de mettre dans les Prefs
        // la valeur ADC pour 4.202 Volts (3556 ici...).
        // En ajustant cette variable, on étalonne la conversion.

        voltage *= 4202.0;
        voltage /= 3556.0;

        // Maintenant on ajuste avec la variable d'étalonnage de nos Préférences.

        voltage *= (float) MyPrefs.EtalonnageVBAT;    // Valeur nominale : 1000. Diminuer si Tension Batterie affichée est trop forte.
        voltage /= 10000.0;
        voltage /= 10.0;  // En 1/100è de Volt

        if (voltage >= 250.0) {  // Si voltage < 2.5V, le condensateur "tampon" n'est pas encore fini de charger au démarrage (haute impédance du pont diviseur) : on attend sans valider...
          TensionBatterie = (unsigned short) voltage;
          VBATValideVu = true;        
          VbatGood = true;
        }

        // 4) On surveille la tension, et on éteint la Station Météo si elle est inférieure à 3.2V (pour protéger la batterie Lithium).
        //    On gère aussi les Leds Rouge/Vert du voyant "Sous Tension" en fonction de la tension batterie.

        if (VBATValideVu) {
          if (voltage < 320.0) Auto_PowerOFF();
          if (voltage >= 373.0) {
            USER_LED_GREEN_ON();
            USER_LED_RED_OFF();
          }
          else if (voltage >= 363.0) {
            USER_LED_GREEN_ON();
            USER_LED_RED_ON();
          }
          else {
            USER_LED_GREEN_OFF();
            USER_LED_RED_ON();
          }
        }
        
        // 6) On fait une RAZ du buffer DMA (pour être sûr de bien lire de nouvelles valeurs...).

        for (i = 0 ; i < 10 ; i++) DMA_VBAT_Buffer[i] = 0x0000;

        // 7) On demande 10 nouveaux Transferts au Canal DMA, et on réactive l'ADC1, lui aussi programmé pour une séquence de 10 Conversions.
        //    Comme après on met cette tâche en veille pendant 1000 ms, au final on a 10 Conversions => 1 valeur de tension batterie mise à jour chaque seconde..
        
        // Pour le DMA :
        DMA1_Channel1         = dma_channel_regs(DMA1, DMA_CH1);    // On récupère l'adresse de Base des Registres de notre Canal DMA.
        DMA1_Channel1->CCR   &= ~(DMA_CCR_EN);                      // On désactive la Canal 1 du DMA (indispensable), Reference Manual page 278.
        DMA1_Channel1->CNDTR  = 10;                                 // On demande 10 nouveaux transferts.
        DMA1_Channel1->CCR   |= DMA_CCR_EN;                         // On active la Canal 1 du DMA
        VbatDMAComplete       = false;                              // RAZ du Flag de fin de transfert DMA.

        // Pour l'ADC :
        VBMON_ADC->regs->CR2 |= ADC_CR2_SWSTART;                    // On lance les Conversion A/D, chaque EOC (End Of Conversion) lancera un transfert DMA
    }

    SLEEPTASK(1000);
  }
}

boolean ButtonPressed(void)
{
  if (ReadGPIO(SWOUT_GPIO,SWOUT_PIN)) return (true);
  return (false);
}


// AnalogCapteursTask() : Tâche qui gère les Conversions A/D pour nos 3 Capteurs analogiques (Luminosité, Température et Humidité).
//                        Mise à jour : selon configuration.

void AnalogCapteursTask(void *pvParameters)
{
static unsigned char  NbLum, NbTemper, NbHum,i;
unsigned char         NbConv,NbConvLum;
unsigned short        Moyenne;
float                 logLux,Ev;
float                 RH,Temperature,Vout,VCC;

  NbConv    = 10;  // On effectue 10 Conversions pour chaque Capteur, pour donner une valeur finale moyennée sur 8 valeurs (on exclut la plus faible et la plus forte).
  NbConvLum = 20;  // Mais pour la Luminosité (très fluctuante), on effectue 20 mesures qu'on moyenne sur les 8 valeurs "centrales".
  
  while (1) {
    
    // Gestion pour Luminosité
    
    if (MustStartLum) {   // On doit démarrer les Conversions sur le Capteur de Luminosité
       if (CapteurSelect == 0) {   // Aucune Conversion en cours...
        MustStartLum  = false;
        CapteurSelect = 1;
        NbLum = 0;
        CAPTEURS_ADC->regs->SQR3  = ADC_CONV1(LUM_ADC_CANAL);     // On sélectionne le Canal A/D du Capteur de Luminosité
        ConversionComplete = false;
        ADC_START_ACQ(CAPTEURS_ADC);  // On lance 1 Conversion (macro dans "StationMeteo.h")
      }
    }
     
   if ((CapteurSelect == 1) && (ConversionComplete == true)) { // Conversion A/D pour la Luminosité disponible.
      ConversionComplete = false;
      LumBuff[NbLum++] = ValeurCapteur;
      if (NbLum == NbConvLum) {  // On a fini nos 20 Conversions
        TriABulles(LumBuff,NbConvLum); // On trie le tableau
        Moyenne = 0;                // On fait la moyenne des 8 Conversions "centrales", sans les plus basses et les plus hautes valeurs
        for (i = 6 ; i < (NbConvLum - 6) ; i++) Moyenne += LumBuff[i];
        Moyenne /= (NbConvLum - 12);
        
        // On convertit la valeur Capteur "moyenne" en Lux
        logLux = ((float) Moyenne * 4.7404) / (float) MyPrefs.EtalonnageLum;  // 10^4.7404 = 55004 => 55004 Lux donne Io = 47.4µA, conforme Doc Luminosité.
        Ev = powf(10.0,logLux);
        
        C_LumLux = (unsigned short) Ev;   // On stocke le résultat, et on indique qu'une nouvelle valeur est disponible
        NewLuminositeDispo = true;
        CapteurSelect = 0;    // On a fini avec la Luminosité
      }
      else ADC_START_ACQ(CAPTEURS_ADC);  // On lance 1 Conversion (macro dans "StationMeteo.h")
    }

    // Gestion pour Température

    if (MustStartTemper) {   // On doit démarrer les Conversions sur le Capteur de Température
       if (CapteurSelect == 0) {   // Aucune Conversion en cours...
        MustStartTemper  = false;
        CapteurSelect = 2;
        NbTemper = 0;
        CAPTEURS_ADC->regs->SQR3  = ADC_CONV1(TEMP_ADC_CANAL);    // On sélectionne le Canal A/D du Capteur de Température
        ConversionComplete = false;
        ADC_START_ACQ(CAPTEURS_ADC);  // On lance 1 Conversion (macro dans "StationMeteo.h")
      }
    }

   if ((CapteurSelect == 2) && (ConversionComplete == true)) { // Conversion A/D pour la Température disponible.
      ConversionComplete = false;
      TemperBuff[NbTemper++] = ValeurCapteur;
      if (NbTemper == NbConv) {  // On a fini nos 10 Conversions
        TriABulles(TemperBuff,NbConv); // On trie le tableau
        Moyenne = 0;                // On fait la moyenne des 8 Conversions "centrales", sans la plus basse et la plus haute
        for (i = 1 ; i < (NbConv - 1) ; i++) Moyenne += TemperBuff[i];
        Moyenne /= (NbConv - 2);
        
        // On convertit la valeur Capteur "moyenne" en Température, Doc Temp/Hum page 8
        VCC = (float) MyPrefs.VCCvoltage / 1000.0;
        
        Vout = (Moyenne * VCC) / 4095;  // VCCvoltage c'est la tension de référence du Convertisseur A/D
        Vout = (Vout * (float) MyPrefs.EtalonnageTemp) / 10000.0;        
        Temperature = (-66.875 + 218.75 * (Vout / VCC));

        C_Temperature = (signed short) (Temperature * 10.0);   // On stocke le résultat en 1/10 de Degrés Celsius, et on indique qu'une nouvelle valeur est disponible
        NewTemperatureDispo = true;
        CapteurSelect = 0;    // On a fini avec la Température
      }
      else ADC_START_ACQ(CAPTEURS_ADC);  // On lance 1 Conversion (macro dans "StationMeteo.h")
    }

    // Gestion pour Humidité

    if (MustStartHum) {   // On doit démarrer les Conversions sur le Capteur d'Humidité
       if (CapteurSelect == 0) {   // Aucune Conversion en cours...
        MustStartHum  = false;
        CapteurSelect = 3;
        NbHum = 0;
        CAPTEURS_ADC->regs->SQR3  = ADC_CONV1(HUM_ADC_CANAL);     // On sélectionne le Canal A/D du Capteur d'Humidité
        ConversionComplete = false;
        ADC_START_ACQ(CAPTEURS_ADC);  // On lance 1 Conversion (macro dans "StationMeteo.h")
      }
    }

   if ((CapteurSelect == 3) && (ConversionComplete == true)) { // Conversion A/D pour l'Humidité disponible.
      ConversionComplete = false;
      HumBuff[NbHum++] = ValeurCapteur;
      if (NbHum == NbConv) {  // On a fini nos 10 Conversions
        TriABulles(HumBuff,NbConv); // On trie le tableau
        Moyenne = 0;                // On fait la moyenne des 8 Conversions "centrales", sans la plus basse et la plus haute
        for (i = 1 ; i < (NbConv - 1) ; i++) Moyenne += HumBuff[i];
        Moyenne /= (NbConv - 2);
        
        // On convertit la valeur Capteur "moyenne" en %Humidité, Doc Temp/Hum page 8

        Vout = (Moyenne / (float) MyPrefs.EtalonnageHum) * 3.30;  // 3.30V c'est la tension de référence du Convertisseur A/D
        RH = -12.5 + 125.0 * (Vout / 3.30);
        
        C_Humidite = (unsigned short) RH;   // On stocke le résultat en %RH (capteur précis à 2%), et on indique qu'une nouvelle valeur est disponible
        NewHumiditeDispo = true;
        CapteurSelect = 0;    // On a fini avec la Température
      }
      else ADC_START_ACQ(CAPTEURS_ADC);  // On lance 1 Conversion (macro dans "StationMeteo.h")
    }

    SLEEPTASK(1);
  }
}


// LEDFlashTask() : Tâche qui fait flasher la Led de la Carte Maple Mini à une fréquence de 1Hz (50+950ms = 1000ms), indéfiniment...

void LEDFlashTask(void *pvParameters)
{
  // noInterrupts();
  // interrupts();
  
  while (1) {
    MAPLE_LED_ON();     // On allume la Led sur la carte Maple Mini - Macro définie dans "StationMeteo.h"
    SLEEPTASK(50);      // La tâche s'endort pendant 50 ms - Macro définie dans "StationMeteo.h"
    
    MAPLE_LED_OFF();    // On éteint la Led sur la carte Maple Mini - Macro définie dans "StationMeteo.h"
    SLEEPTASK(950);     // La tâche s'endort pendant 950 ms - Macro définie dans "StationMeteo.h"
  }
}

// Tri à Bulles d'un tableau de "n" entiers 16 bits. Ne fonctionne pas avec les tableaux déclarés "volatile".

void TriABulles(unsigned short *buff,unsigned short n)
{
unsigned short    i,j,temp;

  for (i = 0; i < n-1; i++) {
    for (j = 0; j < n-i-1; j++) {
      if (buff[j] > buff[j+1]) {
        temp = buff[j];
        buff[j] = buff[j+1];
        buff[j+1] = temp;
      }
    }
  }
}


// *************************************************************
// *    Section GPIO : Gestion des ports d'Entrées/Sorties     *
// *************************************************************


void GPIO_Setup(void)
{

// *************************************
// * Entrées/Sorties sur le port GPIOA *
// *************************************
  
  RCC_BASE->APB2ENR |= (unsigned long)(1 << RCC_APB2_IOPAEN_BIT);       // On active l'horloge du GPIOA sur le Bus APB2, Reference Manual pages 111-112.
  RCC_BASE->APB2RSTR |= RCC_APB2RSTR_IOPARST;                           // Reset du GPIOA : PA10 clignote tout seul... On essaye ça...
  RCC_BASE->APB2RSTR &= ~(RCC_APB2RSTR_IOPARST);
  
  // 1) Initialisation de la Led Utilisateur (Rouge), (Port PA9), allumée niveau "0" (Anode commune)

  GPIO_Pinmode(USER_LED_RED_GPIO,USER_LED_RED_PIN, GPIO_OUTPUT_PP);     // On définit la pin 9 du PortA comme Output "Standard", Push-Pull 50MHz.
  USER_LED_RED_OFF();                                                   // La Led rouge est éteinte pour le moment

  // 3) Initialisation de la Led Utilisateur (Verte), (Port PA10), allumée niveau "0" (Anode commune)

  GPIO_Pinmode(USER_LED_GREEN_GPIO,USER_LED_GREEN_PIN, GPIO_OUTPUT_PP); // On définit la pin 10 du PortA comme Output "Standard", Push-Pull 50MHz.
  USER_LED_GREEN_OFF();                                                 // La Led verte est éteinte pour le moment

  // 3) Initialisation de la sortie pilotant le Mosfet Canal N qui valide le pont diviseur de tension utilisé pour mesurer la tension de la batterie
  //    Ceci permet de mettre hors-circuit ce pont diviseur quand le projet est hors tension, pour ne pas "vider" la batterie. Port PA15, actif niveau "1"

  GPIO_Pinmode(EN_BATMON_GPIO,EN_BATMON_PIN, GPIO_OUTPUT_PP);           // On définit la pin 15 du PortA comme Output "Standard", Push-Pull 50MHz.
  BATMON_ENABLE();                                                      // Le pont diviseur est activé, car sinon il injecte du courant dans le convertisseur A/D (défaut de conception).

// *************************************
// * Entrées/Sorties sur le port GPIOB *
// *************************************

  RCC_BASE->APB2ENR |= (unsigned long)(1 << RCC_APB2_IOPBEN_BIT);       // On active l'horloge du GPIOB sur le Bus APB2, Reference Manual pages 111-112.                                                  
                                                                        // Ne pas faire de Reset du Port, sinon ça plante l'USB...
                                                                        // Comme l'USB est sur le GPIOB, le Framework Arduino a déjà initialisé ce port...                                                    

  // 4) Initialisation de l'entrée digitale qui détecte si l'USB est branché, Port PB0

  GPIO_Pinmode(USB_DET_GPIO,USB_DET_PIN, GPIO_INPUT_FLOATING);          // On définit la pin 0 du PortB comme Input "Standard".

  // 5) Initialisation de la Led intégrée sur carte STM32F103, Port PB1

  GPIO_Pinmode(MAPLE_LED_GPIO,MAPLE_LED_PIN, GPIO_OUTPUT_PP);           // On définit la pin 1 du PortB comme Output "Standard", Push-Pull 50MHz.
  MAPLE_LED_OFF();                                                      // La Led "Maple" est éteinte pour le moment

  // 6) Initialisation de la sortie pilotant le Mosfet canal P qui sert à alimenter les capteurs analogiques, Port PB3, actif niveau "0".
  //    Ceci permet de mettre hors-circuit les capteurs, pour diminuer la consommation et minimiser l'échauffement du capteur de température

  GPIO_Pinmode(EN_CAPTEURS_GPIO,EN_CAPTEURS_PIN, GPIO_OUTPUT_PP);       // On définit la pin 3 du PortB comme Output "Standard", Push-Pull 50MHz.

  // 7) Initialisation de l'entrée digitale permettant de surveiller l'appui sur le bouton Marche/Arrêt, Port PB4.
  //    Attention, cette pin doit être compatible 5Volts, car elle reçoit la tension batterie (4,2V) ou USB (5V)

  GPIO_Pinmode(SWOUT_GPIO,SWOUT_PIN, GPIO_INPUT_FLOATING);              // On définit la pin 4 du PortB comme Input "Standard".

  // 8) Initialisation de l'entrée digitale (avec Pullup) recevant l'interruption 1Hz générée par le circuit horloge RTC externe, port PB5

  GPIO_Pinmode(RTCINT_GPIO,RTCINT_PIN, GPIO_INPUT_PU);                  // On définit la pin 5 du PortB comme Input avec Pullup interne.
}

inline void SetGPIO(gpio_dev* Port, unsigned short Pin)                 // Met la Sortie GPIO désirée à  "1"
{
  Port->regs->BSRR = ((unsigned long)1 << Pin);                         // Port Bit Set Reset Register, Reference Manual page 172.
}

inline void ClearGPIO(gpio_dev* Port, unsigned short Pin)               // Met la Sortie GPIO désirée à  "0"
{
  Port->regs->BRR = ((unsigned long)1 << Pin);                          // Port Bit Reset Register, Reference Manual page 173.
}

inline boolean ReadGPIO(gpio_dev* Port, unsigned short Pin)             // Lit la valeur de la Pin du Port, renvoie true si "1", false si "0".
{
unsigned long PortPin;

 PortPin = (unsigned long) 1 << Pin;
 if (Port->regs->IDR & PortPin) return (true);        // Lecture Input Data Register du Port, Reference Manual page 171.
 return (false);
}

// Programme les registres GPIO en fonction du type de Pin (Entrée Digitale, Sortie Digitale, Entrée Analogique, etc...).

void GPIO_Pinmode(gpio_dev *Port, unsigned char Pin, gpio_pin_mode Mode)
{
gpio_reg_map          *regs;
__IO unsigned long    *cr;
unsigned long         shift,temp;

    regs = Port->regs;            // Accès aux Registres Hardware du Port
    cr = &regs->CRL + (Pin >> 3); // Accès à  CRL ou CRH, suivant la Pin. CRL pour GPIO Pins 0...7, CRH pour GPIO Pins 8...15.
    
    shift = (Pin & 0x07) << 2;
    temp = *cr;                   // Lecture de la Config du Port

    temp &= ~(0x0000000F << shift);
    temp |= (Mode == GPIO_INPUT_PU ? GPIO_INPUT_PD : Mode) << shift;
    *cr = temp;                   // Sauvegarde des mofifications Dans le Control Register correspondant au Port GPIO.

    // Configuration du Ouput Data Register : Résistances Pullup et PullDown, Reference Manual page 160 (Table 20).
    
    if (Mode == GPIO_INPUT_PD) regs->ODR &= ~((unsigned long)1 << Pin);         // Active la résistance PullDown du Port (environ 50k), si désiré
    else if (Mode == GPIO_INPUT_PU) regs->ODR |= ((unsigned long)1 << Pin);     // Active la résistance Pullup du Port (environ 50k), si désiré
}

void Auto_PowerOFF(void)   // Mise hors tension du Projet par logiciel
{
  // En mode normal, la Pin SWOUT est une entrée digitale, pour détecter un appui sur le bouton Marche/Arrêt
  // Ici, on place la Pin en mode Sortie Digitale, et on la met au niveau "0". Ca éteint le Projet.
  // (Le circuit Marche/Arrêt est prévu pour).
  
  GPIO_Pinmode(SWOUT_GPIO,SWOUT_PIN, GPIO_OUTPUT_PP);
  ClearGPIO(SWOUT_GPIO, SWOUT_PIN);
  while (1) {}; // On attend, environ 2 secondes....
}


// *************************************************************
// *  Section FRAM : Initialisation bus SPI1 et gestion FRAM   *
// *************************************************************

void FRAM_Setup(void)  // Configuration au niveau "Matériel" du bus SPI de la FRAM
{
unsigned char   buff[9];

  // 1) On active l'horloge pour SPI1 et on Reset le Périphérique SPI1
    
  RCC_BASE->APB2ENR |= (unsigned long)(1 << RCC_APB2_SPI1EN_BIT);       // On active l'horloge du SPI1 sur le Bus APB2, Reference Manual pages 111-112.  
  RCC_BASE->APB2RSTR |= RCC_APB2RSTR_SPI1_MASK;                         // Reset du SPI1
  RCC_BASE->APB2RSTR &= ~(RCC_APB2RSTR_SPI1_MASK);
  
  // 2) On configure le bus SPI1, après l'avoir désactivé : Registre CR1, Reference Manual pages 745 à 747
  
  FRAM_SPI->regs->CR1 &= ~(SPI_ENABLE);               // On désactive le SPI avant de pouvoir modifier sa configuration

  // La FRAM supporte les Modes SPI "Mode 0" et "Mode 3", Doc FRAM pages 5 et 6.
  // Mode 0 (0,0) signifie CPOL = 0 et CPHA = 0
  // Mode 3 (1,1) signifie CPOL = 1 et CPHA = 1
  // Nous travaillons ici en Mode 0, Avec l'horloge au niveau "0" au repos (CPOL = 0), et la validation des données sur 1er front d'horloge (montant) (CPHA = 0)

  FRAM_SPI->regs->CR1 |= (SPI_2LINESMODE |          // On utilise 2 pins distinctes pour MOSI / MISO
                          SPI_DATAFRAME_8BITS |     // On effectue des transferts sur 8 bits
                          SPI_FULLDUPLEX |          // On veut travailler en Full Duplex
                          SPI_SOFTWARE_CS |         // On gère le Chip Select avec un port GPIO
                          SPI_SSIENABLE |           // SSI validé (le Reference Manual n'est pas clair à  ce sujet, mais il faut activer ce bit)
                          SPI_MSBFIRST |            // Les données sont transmises avec le MSB en premier
                          FRAM_SPI_CLOCK |          // On affecte la fréquence sélectionnée pour le Projet au bus SPI1 (voir StationMeteo.h).
                          SPI_MASTERMODE |          // Le STM32F103 est Maitre de ce bus SPI (la FRAM est esclave)
                          SPI_MODE_0);              // SPI Mode 0


  // 3) Maintenant qu'on a configuré le SPI1, on DOIT l'activer
  
  FRAM_SPI->regs->CR1 |= SPI_ENABLE;

  // 4) On active l'horloge du GPIOA (si ce n'est pas déjà  fait).
  
  RCC_BASE->APB2ENR |= (unsigned long)(1 << RCC_APB2_IOPAEN_BIT);       // On active l'horloge du GPIOA sur le Bus APB2, Reference Manual page 112.
                                                                        // CS, SCK, MOSI et MISO sont tous sur GPIOA, donc 1 seule horloge à  activer.

  // 5) On initialise les pins SCK, MISO, MOSI et CS. (Le SPI1 DOIT être activé AVANT pour que le mode "AF" fonctionne) - Reference Manual page 161

  GPIO_Pinmode(SPI1_SCK_PORT,SPI1_SCK_PIN, GPIO_AF_OUTPUT_PP);          // AF car on utilise cette pin de façon "Alternate Function", pas comme I/O standard
  GPIO_Pinmode(SPI1_MISO_PORT,SPI1_MISO_PIN, GPIO_INPUT_FLOATING);
  GPIO_Pinmode(SPI1_MOSI_PORT,SPI1_MOSI_PIN, GPIO_AF_OUTPUT_PP);        // AF car on utilise cette pin de façon "Alternate Function", pas comme I/O standard

  GPIO_Pinmode(MEM_CS_GPIO,MEM_CS_PIN, GPIO_OUTPUT_PP);                 // On définit la pin 4 du PortA comme Output "Standard", Push-Pull 50MHz.
  MEM_DESELECT();                                                       // La mémoire FRAM n'est pas sélectionnée pour le moment

  // 6) On calcule la densité et la plus haute adresse accessible dans la FRAM (utilisé dans certaines routines)
  //    FRAM_Densite et FRAM_AdresseMax sont des variables globales
  
  FRAM_ReadDevID(buff,&FRAM_Densite,&FRAM_AdresseMax);  // On lit les caractéristiques de la FRAM, et on les stocke dans les 2 variables utilisées ici.
  // La FRAM est en mode Sleep...
}

unsigned char FRAM_TransferByte(unsigned char Data)
{
spi_reg_map          *regs;

  regs = FRAM_SPI->regs;

  // 1) Envoi de la donnée
  
  while (!(regs->SR & SPI_SR_TXE)) {};     // (TX buffer Empty) On attend que la dernière écriture soit terminée, Reference Manual page 748
  regs->DR = Data;                         // On écrit dans le SPI Data Register, Reference Manual page 749

  // 2) Réception de la donnée

  while (!(regs->SR & SPI_SR_RXNE)) {};    // (RX buffer Not Empty) On attend qu'une donnée soit présente dans le buffer d'entrée, Reference Manual page 749
  return (regs->DR);                       // On retourne la valeur du Data Register
}

void FRAM_Sleep(void)
{
  MEM_DESELECT();                         // On désactive le Chip Select de la FRAM
  MEM_SELECT();                           // On active le Chip Select de la FRAM
  (void) FRAM_TransferByte(FRAM_SLEEP);   // On met la FRAM en mode "Sleep" (5µA de consommation), doc FRAM page 11
  MEM_DESELECT();                         // On désactive le Chip Select de la FRAM 
}

void FRAM_Awake(void)
{
  MEM_SELECT();
  delayMicroseconds(400);   // 400µs de délai, doc FRAM page 17 (tREC)
  // On laisse la FRAM sélectionnée
}

boolean FRAM_ReadAddress(unsigned long Adresse, unsigned char *buff, unsigned short DataLen)   // On lit "DataLen" octets dans la FRAM, à  partir de l'adresse désirée
{
unsigned short       i;
unsigned char        tempByte;

  if (FRAM_AdresseMax == 0L) return (false);                  // Erreur : FRAM non reconnue !
  if ((Adresse + DataLen) > FRAM_AdresseMax) return (false);  // Erreur : Capacité mémoire dépassée
  if (DataLen == 0) return (false);                           // Erreur : Nombre d'octets à lire = 0
  
  FRAM_Awake();                           // On active le Chip Select de la FRAM et on attend 400Ã‚Âµs

  (void) FRAM_TransferByte(FRAM_READ);    // Pendant cet envoi, "SO" de la FRAM est Haute Impédance (on reçoit 0x00 en retour)

  // On va envoyer l'adresse "de base" pour la lecture : 2 octets (16 bits) si FRAM <= 512K, 3 octets (24 bits) au-delà

  if (FRAM_Densite <= 512) {  // Adresse sur 16 bits, MSB en premier
    tempByte = (unsigned char) (Adresse >> 8);
    (void) FRAM_TransferByte(tempByte); // Envoi des 8 bits de poids fort de l'adresse
    tempByte = (unsigned char) (Adresse & 0x000000FF);
    (void) FRAM_TransferByte(tempByte); // Envoi des 8 bits de poids faible de l'adresse
  }
  else {                      // Adresse sur 24 bits, MSB en premier
    tempByte = (unsigned char) (Adresse >> 16);
    (void) FRAM_TransferByte(tempByte); // Envoi des 8 bits de poids fort de l'adresse
    tempByte = (unsigned char) (Adresse >> 8);
    (void) FRAM_TransferByte(tempByte); // Envoi des 8 bits de poids moyen de l'adresse
    tempByte = (unsigned char) (Adresse & 0x000000FF);
    (void) FRAM_TransferByte(tempByte); // Envoi des 8 bits de poids faible de l'adresse    
  }

  // Maintenant, lecture des données

  for (i = 0 ; i < DataLen ; i++) buff[i] = FRAM_TransferByte(0x00);    // On écrit un octet pour activer l'horloge SPI et donc en recevoir un.

  FRAM_Sleep();                           // On met la FRAM en mode "Sleep"

  return (true);
}

boolean FRAM_WriteAddress(unsigned long Adresse, unsigned char *buff, unsigned short DataLen)   // On écrit "DataLen" octets dans la FRAM, à  partir de l'adresse désirée
{
unsigned short       i;
unsigned char        tempByte;

  if (FRAM_AdresseMax == 0L) return (false);                  // Erreur : FRAM non reconnue !
  if ((Adresse + DataLen) > FRAM_AdresseMax) return (false);  // Erreur : Capacité mémoire dépassée
  if (DataLen == 0) return (false);                           // Erreur : Nombre d'octets à lire = 0

  // On doit d'abord "déverrouiller" la FRAM, pour autoriser l'écriture.
  
  FRAM_Awake();                           // On active le Chip Select de la FRAM et on attend 400µs
  (void) FRAM_TransferByte(FRAM_WREN);    // On autorise l'écriture de données dans la FRAM (Opcode WREN)
  MEM_DESELECT();                         // On désactive le Chip Select de la FRAM (obligatoire)

  // Maintenant, on peut écrire dans la FRAM
  
  MEM_SELECT();                           // On active le Chip Select de la FRAM

  (void) FRAM_TransferByte(FRAM_WRITE);    // Pendant cet envoi, "SO" de la FRAM est Haute Impédance (on reçoit 0x00 en retour)

  // On va envoyer l'adresse "de base" pour l'écriture : 2 octets (16 bits) si FRAM <= 512K, 3 octets (24 bits) au-delà 

  if (FRAM_Densite <= 512) {  // Adresse sur 16 bits, MSB en premier
    tempByte = (unsigned char) (Adresse >> 8);
    (void) FRAM_TransferByte(tempByte); // Envoi des 8 bits de poids fort de l'adresse
    tempByte = (unsigned char) (Adresse & 0x000000FF);
    (void) FRAM_TransferByte(tempByte); // Envoi des 8 bits de poids faible de l'adresse
  }
  else {                      // Adresse sur 24 bits, MSB en premier
    tempByte = (unsigned char) (Adresse >> 16);
    (void) FRAM_TransferByte(tempByte); // Envoi des 8 bits de poids fort de l'adresse
    tempByte = (unsigned char) (Adresse >> 8);
    (void) FRAM_TransferByte(tempByte); // Envoi des 8 bits de poids moyen de l'adresse
    tempByte = (unsigned char) (Adresse & 0x000000FF);
    (void) FRAM_TransferByte(tempByte); // Envoi des 8 bits de poids faible de l'adresse    
  }

  // Maintenant, écriture des données

  for (i = 0 ; i < DataLen ; i++) (void) FRAM_TransferByte(buff[i]);

  MEM_DESELECT();                         // On désactive le Chip Select de la FRAM (obligatoire avant verrouillage)

  // On verrouille la FRAM pour interdire l'écriture (sécurité)
  
  MEM_SELECT();                           // On active le Chip Select de la FRAM
  (void) FRAM_TransferByte(FRAM_WRDI);    // On interdit l'écriture de données dans la FRAM (Opcode WRDI)
  FRAM_Sleep();                           // On met la FRAM en mode "Sleep"

  return (true);
}

unsigned char FRAM_ReadDevID(unsigned char *buff, unsigned long *Densite, unsigned long *TopAdresse)  // On lit les 9 octets d'identification de la FRAM, doc FRAM page 11.
{
unsigned char i,Info;
  
  FRAM_Awake();                           // On active le Chip Select de la FRAM, et on attend 400Ã‚Âµs
  
  (void) FRAM_TransferByte(FRAM_RDID);    // Pendant cet envoi, "SO" de la FRAM est Haute Impédance (on reçoit 0x00 en retour)
  for (i = 0 ; i < 9 ; i++) {             // Le DeviceID de la FRAM comporte 9 octets.
    buff[i] = FRAM_TransferByte(0x00);    // On écrit un octet pour activer l'horloge SPI et donc en recevoir un.
                                          // "SI" de la FRAM est Haute Impédance (donc peu importe la valeur envoyée)
  }
  
  FRAM_Sleep();                           // On met la FRAM en mode "Sleep"
  
  // On détermine la densité de la FRAM, Doc FRAM page 11

  Info = buff[7] & 0x1F;  // On ne garde que les bits 0...4

  // On indique la plus haute adresse disponible dans cette FRAM, selon la densité.

  switch (Info) {
    case FRAM128K:  // 128Kbit = 16K x 8, Top Adresse = 0x0000 3FFF
      *Densite = 128;
      *TopAdresse = 0x00003FFF;
    break;
    case FRAM256K:  // 256Kbit = 32K x 8, Top Adresse = 0x0000 7FFF
      *Densite = 256;
      *TopAdresse = 0x00007FFF;
    break;
    case FRAM512K:  // 512Kbit = 64K x 8, Top Adresse = 0x0000 FFFF
      *Densite = 512;
      *TopAdresse = 0x0000FFFF;
    break;
    case FRAM1024K: // 1024Kbit = 128K x 8, Top Adresse = 0x0001 FFFF
      *Densite = 1024;
      *TopAdresse = 0x0001FFFF;
    break;
    case FRAM2048K: // 2048Kbit = 256K x 8, Top Adresse = 0x0003 FFFF
      *Densite = 2048;
      *TopAdresse = 0x0003FFFF;
    break;
    case FRAM4096K: // 4096Kbit = 512K x 8, Top Adresse = 0x0007 FFFF
      *Densite = 4096;
      *TopAdresse = 0x0007FFFF;
    default:        // Mémoire Inconnue
      *Densite = 0L;
      *TopAdresse = 0L;
      return (0); // Erreur, Mémoire non reconnue
    break;
  }
  return (9);
}


// **********************************************
// *   Section I2C : Gestion du Protocole I2C   *
// **********************************************

/*
  Toute la gestion du Protocole I2C repose, au niveau 0 (le plus bas), sur seulement 5 fonctions de base :

  1) I2C_SendStart()    qui envoie un signal "START" sur le bus I2C (Début de Transaction)
  2) I2C_SendStop()     qui envoie un signal "STOP"  sur le bus I2C (Fin de Transaction)
  3) I2C_SendAddress()  qui envoie l'adresse (en Lecture ou en Ecriture) du Périphérique
  4) I2C_ReadData()     qui lit 8 bits de Données en provenance du Périphérique (l'I2C est un Protocole 8 bits)
  5) I2C_WriteData()    qui écrit 8 bits de Données sur le Périphérique (l'I2C est un Protocole 8 bits)

  Les fonctions I2C_SetACK() et I2C_SetACKPos() sont des "Aides", pour simplifier la lecture du Protocole.
  La fonction I2C_Setup() ne fait pas partie du Protocole I2C, elle initialise les Ports Hardware I2C (I2C1 et I2C2) du MCU STM32.

  L'ensemble du Protocole I2C est donc géré au niveau 0 - "Hardware" par ces 8 fonctions. Elles sont liées au MCU utilisé (dépendantes du Hardware).
  Le Protocole I2C est géré ici en mode "Polling", donc bloquant. Il est néanmoins "préempté" par FreeRTOS, et protégé par des Mutex.
  
  Les fonctions I2C_StdReadReg8(), I2C_StdReadReg16(), I2C_StdReadReg24() et I2C_StdWriteReg8(), I2C_StdWriteReg16(), I2C_StdWriteReg24() sont du niveau 1 - "Driver".
  Les fonctions des niveaux 1 et supérieurs sont indépendantes du Hardware MCU STM32, mais dépendent des Périphériques I2C utilisés : RTC et Baromètre ici..
*/

// Fonctions Niveau 0 (Hardware)

void I2C_Setup(unsigned short BusI2C)   // Cette fonction configure le bus I2C1 ou I2C2 du MCU STM32F103 en fonction des Defines de "StationMeteo.h".
{
i2c_dev*        I2CBus;
rcc_clk_domain  apb1_clk_domain;
unsigned long   RCC_Config,Quartz_Freq,PLLMul,PLL_Multiplier;
unsigned long   AHB_Divider, VCO_InputFreq,SysFreq,APB1_Prescaler,APB1_PRE,APB1_Freq;
unsigned long   tempreg,I2CBusSpeed,DutyCycle,freqrange;
unsigned short  Result;

  if (BusI2C == RTC_I2C) I2CBus = RTC_I2C_BUS;
  else I2CBus = BMP180_I2C_BUS;

  //i2c_init(I2CBus); // Nécessaire !!
  //i2c_bus_reset(I2CBus);
  
  // 1) On active l'horloge du bus I2C, bus interne APB1 du STM32F103, Reference Manual page 114.

  if (BusI2C == RTC_I2C) {
    RCC_BASE->APB1ENR |= (unsigned long)(1 << RCC_APB1_I2C1EN_BIT);   // Activation de l'Horloge du Périphérique I2C1
    RCC_BASE->APB1RSTR |= RCC_APB1RSTR_I2C1_MASK;                     // Reset du Périphérique I2C1
    RCC_BASE->APB1RSTR &= ~(RCC_APB1RSTR_I2C1_MASK);    
  }
  else {
    RCC_BASE->APB1ENR |= (unsigned long)(1 << RCC_APB1_I2C2EN_BIT);   // Activation de l'Horloge du Périphérique I2C2
    RCC_BASE->APB1RSTR |= RCC_APB1RSTR_I2C2_MASK;                     // Reset du Périphérique I2C2
    RCC_BASE->APB1RSTR &= ~(RCC_APB1RSTR_I2C2_MASK);    
  }

  // 2) On désactive le périphérique I2C, pour pouvoir le configurer, Reference Manual page 787.

  I2CBus->regs->CR1 &= ~(I2C_ENABLE);

  // 3) On configure le bus I2C
      
      // 3a) Configuration du registre CR2, Reference Manual pages 779-780.
      //     On recherche la fréquence du Bus interne APB1, sur lequel sont interfacés I2C1 et I2C2 (normalement 36MHz)

      RCC_Config = RCC_BASE->CFGR;            // Registre de configuration de toutes les Horloges du STM32, Reference Manual page 100.

      Quartz_Freq = 8000000L;                 // Le Maple Mini possède un quartz de 8MHz.

      PLLMul = RCC_Config & RCC_CFGR_PLLMUL;
      PLL_Multiplier = (PLLMul >> 18) + 2;    // Normalement, on a un multiplicateur 9x (9 x 8MHz = 72MHz)
            
      if (RCC_Config & RCC_CFGR_PLLXTPRE) Quartz_Freq >>= 1;  // La fréquence du Quartz est-elle divisée par 2 en entrée du PLL ? (pour le Maple Mini, non).
    
      VCO_InputFreq = Quartz_Freq * PLL_Multiplier;           // Fréquence interne du VCO du PLL (ici 9 x 8MHz = 72MHz).
      
      AHB_Divider = RCC_Config & RCC_CFGR_HPRE;               // On recherche par combien la fréquence interne du PLL est divisée
      switch (AHB_Divider) {
        case RCC_AHB_SYSCLK_DIV_1:
          SysFreq = VCO_InputFreq;                            // Pour le Maple Mini, c'est ce qui est utilisé (72MHz / 1 = 72MHz).
        break;
        case RCC_AHB_SYSCLK_DIV_2:
          SysFreq = VCO_InputFreq >> 1;
        break;
        case RCC_AHB_SYSCLK_DIV_4:
          SysFreq = VCO_InputFreq >> 2;
        break;
        case RCC_AHB_SYSCLK_DIV_8:
          SysFreq = VCO_InputFreq >> 3;
        break;
        case RCC_AHB_SYSCLK_DIV_16:
          SysFreq = VCO_InputFreq >> 4;
        break;
        case RCC_AHB_SYSCLK_DIV_32:
          SysFreq = VCO_InputFreq >> 5;
        break;
        case RCC_AHB_SYSCLK_DIV_64:
          SysFreq = VCO_InputFreq >> 6;
        break;
      }
    
      APB1_PRE = RCC_Config & RCC_CFGR_PPRE1;               // On recherche par combien la fréquence Système est divisée pour alimenter le bus interne APB1
      switch (APB1_PRE) {
        case RCC_APB1_HCLK_DIV_1:
          APB1_Prescaler = 1;
        break;
        case RCC_APB1_HCLK_DIV_2:
          APB1_Prescaler = 2;                               // Pour le Maple Mini, c'est ce qui est utilisé (72 MHz / 2 = 36MHz)
        break;
        case RCC_APB1_HCLK_DIV_4:
          APB1_Prescaler = 4;
        break;
        case RCC_APB1_HCLK_DIV_8:
          APB1_Prescaler = 8;
        break;
        case RCC_APB1_HCLK_DIV_16:
          APB1_Prescaler = 16;
        break;
      }
    
      APB1_Freq = SysFreq / APB1_Prescaler;                 // Fréquence de APB1 en Hz  (ex : 36000000)
      freqrange = APB1_Freq / 1000000;                      // Fréquence de APB1 en MHz (ex : 36)
      
      // 3b) Mise à jour du registre CR2

      tempreg = I2CBus->regs->CR2;                          // Lecture du registre CR2 de l'I2C.
      tempreg &= 0xFFC0;                                    // Reset des bits 0...5 de CR2
                                                            // On indique au périphérique I2C du STM32 la fréquence de son bus interne APB1, pour qu'il puisse se synchroniser.
      tempreg |= freqrange;
      I2CBus->regs->CR2 = tempreg;                          // On enregistre les modifications dans le registre CR2 de l'I2C.

      // 3c) On configure les registre CCR (Clock Control Register), Reference Manual pages 787-788 et TRISE, Reference Manual page 788.

      tempreg = 0L;
      if (BusI2C == RTC_I2C) {
        I2CBusSpeed = RTC_BUS_SPEED;
        DutyCycle = RTC_I2C_DUTYCYCLE;
      }
      else {
        I2CBusSpeed = BMP180_BUS_SPEED;
        DutyCycle = BMP180_I2C_DUTYCYCLE;
      }
      
      if (I2CBusSpeed <= 100000) {           // Standard I2C (100KHz Max)
        Result = (unsigned short) (APB1_Freq / (I2CBusSpeed << 1));    // On calcule la vitesse du bus en mode "Standard"
        if ((Result & 0x0FFF) < 0x04) Result = 0x04;                   // Valeur minimale, Reference Manual page 788
        tempreg |= Result;
        I2CBus->regs->TRISE = freqrange + 1;  // Maximum Rise Time, Reference Manual page 788.
      }
      
      else {                                // Fast I2C (400KHz Max)
        if (DutyCycle == I2C_DUTY_CYCLE_2) Result = (unsigned short)(APB1_Freq / (I2CBusSpeed * 3));  // Duty Cycle 2:1 (tLow/tHigh = 2)
        else {
          Result = (unsigned short) (APB1_Freq / (I2CBusSpeed * 25));   // Duty Cycle 16:9 (tLow/tHigh = 16/9)
          Result |= I2C_CCR_DUTY_16_9;                                  // On met le bit Duty (14) à 1.
        }
        if ((Result & 0x0FFF) == 0) Result |= 0x0001;                   // Valeur minimale
        tempreg |= (unsigned short)(Result | I2C_CCR_FS);               // Mise à jour de la vitesse, et activation du mode "Fast I2C"
        // On ajuste le Maximum Rise Time pour le mode "Fast I2C"
        I2CBus->regs->TRISE = (unsigned short)(((freqrange * (unsigned short)300) / (unsigned short)1000) + (unsigned short)1);
      }
      
     I2CBus->regs->CCR = tempreg;                          // On enregistre les modifications dans le registre CCR de l'I2C.
     //I2CBus->regs->TRISE = 0x0025;  // ARDUINO
     
     // 3d) On active le Périphérique I2C

     I2CBus->regs->CR1 |= I2C_ENABLE;

     // 3e) On configure le registre OAR1 (Own Address Register), Reference Manual page 781

  switch (BusI2C) {
    case RTC_I2C:
     I2CBus->regs->OAR1 = 0x4000 | (STM32_I2C_ADDR_RTC << 1); // On garde le bit 14 à "1", comme indiqué dans le Reference Manual. On utilise le mode d'adressage 7 bits.
    break;
    case BMP180_I2C:
     I2CBus->regs->OAR1 = 0x4000 | (STM32_I2C_ADDR_BARO << 1); // On garde le bit 14 à "1", comme indiqué dans le Reference Manual. On utilise le mode d'adressage 7 bits.
    break;
  }
     //I2CBus->regs->OAR1 = (STM32_I2C_ADDR << 1);            // Comme Arduino
     
  // 4) On configure les Pins GPIO de SCL et SDA en AF Output Open Drain, nécessaire sur un bus I2C.

  // I2C1 et I2C2 ont leurs SCL et SDA sur le port GPIOB, donc on n'a qu'une seule horloge à activer

  RCC_BASE->APB2ENR |= (unsigned long)(1 << RCC_APB2_IOPBEN_BIT);       // On active l'horloge du GPIOB sur le Bus APB2, Reference Manual pages 111-112.
  
  switch (BusI2C) {
    case RTC_I2C:
        GPIO_Pinmode(I2C1_SCL_PORT,I2C1_SCL_PIN, GPIO_AF_OUTPUT_OD);    // AF car on utilise cette pin de façon "Alternate Function", pas comme I/O standard
        GPIO_Pinmode(I2C1_SDA_PORT,I2C1_SDA_PIN, GPIO_AF_OUTPUT_OD);    // AF car on utilise cette pin de façon "Alternate Function", pas comme I/O standard
    break;
    case BMP180_I2C:     
        GPIO_Pinmode(I2C2_SCL_PORT,I2C2_SCL_PIN, GPIO_AF_OUTPUT_OD);    // AF car on utilise cette pin de façon "Alternate Function", pas comme I/O standard
        GPIO_Pinmode(I2C2_SDA_PORT,I2C2_SDA_PIN, GPIO_AF_OUTPUT_OD);    // AF car on utilise cette pin de façon "Alternate Function", pas comme I/O standard
    break;
  }
}

void inline I2C_GenerateStart(i2c_dev* I2Cx)
{
unsigned long     cr1;

    while ((cr1 = I2Cx->regs->CR1) & (I2C_CR1_START | I2C_CR1_STOP  | I2C_CR1_PEC)) {};

    I2Cx->regs->CR1 |= I2C_CR1_START;
    while(!(I2Cx->regs->SR1 & I2C_SR1_SB)) {};       // On attend que le Start soit généré, Reference Manual page 785
}

void inline I2C_GenerateStop(i2c_dev* I2Cx)
{
unsigned long     cr1;
    
    while ((cr1 = I2Cx->regs->CR1) & (I2C_CR1_START | I2C_CR1_STOP  | I2C_CR1_PEC)) {};

    I2Cx->regs->CR1 |= I2C_CR1_STOP;
    while ((cr1 = I2Cx->regs->CR1) & (I2C_CR1_START | I2C_CR1_STOP  | I2C_CR1_PEC)) {};
}

void I2C_ToggleACK(i2c_dev* I2Cx,boolean Enable)
{
  if (Enable) I2Cx->regs->CR1 |= I2C_ACK_ENABLE;                        // Registre CR1 : ACK = 1, Reference Manual page 778.
  else        I2Cx->regs->CR1 &= ~(I2C_ACK_ENABLE);                     // Registre CR1 : ACK = 0, Reference Manual page 778.
}

void I2C_SetACKPos(i2c_dev* I2Cx,unsigned char Pos)
{
  if (Pos == ACK_POS_CURRENT) I2Cx->regs->CR1 &= I2C_NAK_POS_CURRENT;   // Registre CR1 : POS = 0, Reference Manual page 778.
  else                        I2Cx->regs->CR1 |= I2C_NAK_POS_NEXT;      // Registre CR1 : POS = 1
}

void I2C_SendData(i2c_dev* I2Cx,unsigned char Data)
{
  I2Cx->regs->DR = Data;                          // On envoie 1 octet de Données
  while (!(I2Cx->regs->SR1 & I2C_SR1_TXE)) {};    // On attend que la donnée soit envoyée et validée
}

unsigned char I2C_ReceiveData(i2c_dev* I2Cx)
{
  while (!(I2Cx->regs->SR1 & I2C_SR1_RXNE)) {};  // Attente de buffer réception plein

  return (unsigned char)I2Cx->regs->DR;
}

unsigned short I2C_StdReadReg16(i2c_dev* I2Cx,unsigned char Register,unsigned char AddressRead, unsigned char AddressWrite)
{
unsigned char   MSB,LSB;
unsigned short  Valeur;

  I2C_GenerateStart(I2Cx);                // On envoie le signal "Start". On passe en mode Master
  I2C_SendAddress(I2Cx,AddressWrite);     // On envoie l'adresse du Périphérique, mode Ecriture
  I2C_SendData(I2Cx,Register);            // On envoie l'Adresse de départ pour la Lecture
  I2C_GenerateStart(I2Cx);                // On envoie le signal "Start" (Repeated Start)
  I2C_SendAddress(I2Cx,AddressRead);      // On envoie l'Adresse du Périphérique, en mode Lecture

  MSB  = I2C_ReceiveData(I2Cx);

  I2C_ToggleACK(I2Cx,DISABLE);            // Disable ACKnowledge
  I2C_SetACKPos(I2Cx,ACK_POS_CURRENT);    // Le NACK sera envoyé pour l'octet en cours de réception

  LSB  = I2C_ReceiveData(I2Cx);

  I2C_GenerateStop(I2Cx);                 // Génère le signal Stop
  I2C_ToggleACK(I2Cx,ENABLE);             // Enable ACKnowledge

  Valeur = ((unsigned short) MSB) << 8;
  Valeur |= LSB;
  return (Valeur);
}

unsigned long I2C_StdReadReg24(i2c_dev* I2Cx,unsigned char Register,unsigned char AddressRead, unsigned char AddressWrite)
{
unsigned char   MSB,LSB,XLSB;
unsigned long  Valeur;

  I2C_GenerateStart(I2Cx);                // On envoie le signal "Start". On passe en mode Master
  I2C_SendAddress(I2Cx,AddressWrite);     // On envoie l'adresse du Périphérique, mode Ecriture
  I2C_SendData(I2Cx,Register);            // On envoie l'Adresse de départ pour la Lecture
  I2C_GenerateStart(I2Cx);                // On envoie le signal "Start" (Repeated Start)
  I2C_SendAddress(I2Cx,AddressRead);      // On envoie l'Adresse du Périphérique, en mode Lecture

  MSB   = I2C_ReceiveData(I2Cx);

  LSB   = I2C_ReceiveData(I2Cx);

  I2C_ToggleACK(I2Cx,DISABLE);            // Disable ACKnowledge
  I2C_SetACKPos(I2Cx,ACK_POS_CURRENT);    // Le NACK sera envoyé pour l'octet en cours de réception

  XLSB  = I2C_ReceiveData(I2Cx);

  I2C_GenerateStop(I2Cx);                 // Génère le signal Stop
  I2C_ToggleACK(I2Cx,ENABLE);             // Enable ACKnowledge

  Valeur  = ((unsigned long) MSB) << 16;
  Valeur |= ((unsigned short) LSB) << 8;
  Valeur |= XLSB;
  
  return (Valeur);
}

void I2C_SendAddress(i2c_dev* I2Cx,unsigned char Address)
{
unsigned long     temp;

  I2Cx->regs->DR = Address;
  while (!(I2Cx->regs->SR1 & I2C_SR1_ADDR)) {};   // On attent que l'adresse soit transmise (EV6), Reference Manual page 766

  temp = I2Cx->regs->SR2;                         // Lecture SR1 puis SR2 = RAZ de ADDR
}


// *************************************************************
// *   Section RTC : Initialisation bus I2C1 et gestion RTC    *
// *************************************************************

void RTC_Setup(void)  // Configuration au niveau "Matériel" du bus I2C de la RTC
{
  I2C_Setup(RTC_I2C);
}

unsigned char dec2bcd(unsigned char dec)   // Conversion Décimal -> BCD (format interne de la RTC) - Source : Internet
{
unsigned char ones = 0;
unsigned char tens = 0;
unsigned char temp = 0;

    ones = dec % 10;
    temp = dec / 10;
    tens = (temp % 10) << 4; //I used modulus here to get rid of the improper display issue
    return (tens + ones);
}

unsigned char bcd2dec(unsigned char bcd)   // Conversion BCD (format interne de la RTC) -> Décimal - Source : Internet
{
unsigned char     dec;

  dec = (bcd >> 4) & 0x07;
  dec = (dec << 3) + (dec << 1) + (bcd & 0x0F);
  return (dec);
}

void RTC_AccessReadRegister(unsigned char BaseRegister)   // Accède à un Registre spécifique de la RTC pour le(s) lire, Doc RTC pages 17-18
{
  // Procédure pour lire N octets, Reference Manual page 768 et Doc RTC page 18

  I2C_ToggleACK(RTC_I2C_BUS,ENABLE);                  // Enable ACKnowledge
  I2C_SetACKPos(RTC_I2C_BUS,ACK_POS_CURRENT);         // Le ACK (ou NAK) sera envoyé pour l'octet en cours de réception
    
  I2C_GenerateStart(RTC_I2C_BUS);                     // On envoie le signal "Start". On passe en mode Master

  I2C_SendAddress(RTC_I2C_BUS,RTC_WRITE_ADDR);        // On envoie l'adresse de la RTC, mode Ecriture
    
  I2C_SendData(RTC_I2C_BUS,BaseRegister);             // On envoie 1 octet de Data : l'Adresse du premier Registre à lire
  
  I2C_GenerateStart(RTC_I2C_BUS);                     // On envoie le signal "Start" (Repeated Start)

  I2C_SendAddress(RTC_I2C_BUS,RTC_READ_ADDR);         // On envoie l'Adresse de la RTC, en mode Lecture (on veut Lire des registres)
};

void RTC_ReadDateTime(DateTimeRec *newtime)    // Remplit le DateTimeRec avec les valeurs courantes lues dans la RTC.
{
unsigned char     RegistreDeBase;

  RegistreDeBase = 0x00;                              // Les données RTC que nous voulons commencent au registre 0, Doc RTC page 11.

  RTC_AccessReadRegister(RegistreDeBase);             // On accède en mode Lecture au registre désiré, dans la RTC.
  
  // La RTC envoie des données tant qu'on n'envoie pas un NAK
  
  newtime->Secondes = bcd2dec(I2C_ReceiveData(RTC_I2C_BUS));
  newtime->Minutes  = bcd2dec(I2C_ReceiveData(RTC_I2C_BUS));
  newtime->Heure    = bcd2dec(I2C_ReceiveData(RTC_I2C_BUS));
  newtime->JourSem  = bcd2dec(I2C_ReceiveData(RTC_I2C_BUS));
  newtime->Date     = bcd2dec(I2C_ReceiveData(RTC_I2C_BUS));
  newtime->Mois     = bcd2dec(I2C_ReceiveData(RTC_I2C_BUS));

  I2C_ToggleACK(RTC_I2C_BUS,DISABLE);                 // Clear ACK, car l'octet suivant sera le dernier qu'on veut recevoir (il recevra un NAK)

  newtime->Annee = bcd2dec(I2C_ReceiveData(RTC_I2C_BUS));

  I2C_GenerateStop(RTC_I2C_BUS);                      // Génère le signal Stop
  I2C_ToggleACK(RTC_I2C_BUS,ENABLE);                  // Enable ACKnowledge
}

void RTC_WriteDateTime(DateTimeRec *newtime)   // Met à jour la RTC à partir du DateTimeRec
{
unsigned char     RegistreDeBase;

  RegistreDeBase = 0x00;                              // Les données RTC que nous voulons commencent au registre 0, Doc RTC page 11.

}

unsigned char RTC_ReadRegister(unsigned char Register)  // Lit un registre spécifique de la RTC, Doc RTC page 18, exemple B).
{
unsigned char   Resultat;

  if (Register > 0x12) return (0x00);                   // La RTC DS3231 a ses registres aux adresses 0x00 ... 0x12
  
  Resultat = 0x00;
  
  if (Mutex_Horloge != NULL) {                          // On utilise le Mutex, parce que d'autres tâches (comme UpdateHorlogeTask()) peuvent vouloir accéder à la RTC en même temps...
    TAKE_MUTEX(Mutex_Horloge);
    
    I2C_GenerateStart(RTC_I2C_BUS);                     // On envoie le signal "Start". On passe en mode Master
    I2C_SendAddress(RTC_I2C_BUS,RTC_WRITE_ADDR);        // On envoie l'adresse de la RTC, mode Ecriture
    I2C_SendData(RTC_I2C_BUS,Register);                 // On envoie 1 octet de Data : l'Adresse du Registre à lire
    I2C_GenerateStart(RTC_I2C_BUS);                     // On envoie le signal "Start" (Repeated Start)
    I2C_SendAddress(RTC_I2C_BUS,RTC_READ_ADDR);         // On envoie l'Adresse de la RTC, en mode Lecture (on veut Lire des registres)

    I2C_ToggleACK(RTC_I2C_BUS,DISABLE);                 // Disable ACKnowledge
    I2C_SetACKPos(RTC_I2C_BUS,ACK_POS_CURRENT);         // Le NACK sera envoyé pour l'octet en cours de réception

    Resultat = I2C_ReceiveData(RTC_I2C_BUS);

    I2C_GenerateStop(RTC_I2C_BUS);                      // Génère le signal Stop
    I2C_ToggleACK(RTC_I2C_BUS,ENABLE);                  // Enable ACKnowledge
    
    GIVE_MUTEX(Mutex_Horloge);
  }
  return (Resultat);
}

void RTC_WriteRegister(unsigned char Register, unsigned char Data)  // Ecrit une valeur dans un registre spécifique de la RTC, Doc RTC page 18, exemple A).
{
  if (Register > 0x12) return;                              // La RTC DS3231 a ses registres aux adresses 0x00 ... 0x12
  
  if (Mutex_Horloge != NULL) {                              // On utilise le Mutex, parce que d'autres tâches (comme UpdateHorlogeTask()) peuvent vouloir accéder à la RTC en même temps...
    TAKE_MUTEX(Mutex_Horloge);
    
        I2C_GenerateStart(RTC_I2C_BUS);                     // On envoie le signal "Start". On passe en mode Master
        I2C_SendAddress(RTC_I2C_BUS,RTC_WRITE_ADDR);        // On envoie l'adresse de la RTC, mode Ecriture
        I2C_SendData(RTC_I2C_BUS,Register);                 // On envoie 1 octet de Data : l'Adresse du Registre à écrire
        I2C_SendData(RTC_I2C_BUS,Data);                     // On envoie 1 octet de Data : la Donnée à écrire dans le Registre
        I2C_GenerateStop(RTC_I2C_BUS);                      // Génère le signal Stop
        I2C_ToggleACK(RTC_I2C_BUS,ENABLE);                  // Enable ACKnowledge
    
    GIVE_MUTEX(Mutex_Horloge);
  }
}

short RTC_ReadTemperature(void)                      // Lit la Température interne de la RTC DS3231 (registres 0x11 (MSB) et 0x12 (LSB))
{                                                           // Résolution : 0.25°C. Donc, après arrondi, on a 19.7°C, 20.0°C, 20.2°C, 20.5°C, 20.7°C ou 21.0°C, par exemple.
short             Temperature;
unsigned char     MSB, LSB;
unsigned char     RegistreDeBase;
unsigned short    temp;

  RegistreDeBase = 0x11;                                    // Les données RTC que nous voulons commencent au registre 0x11, Doc RTC page 11.
  Temperature = 0;
  
  if (Mutex_Horloge != NULL) {                              // On utilise le Mutex, parce que d'autres tâches (comme UpdateHorlogeTask()) peuvent vouloir accéder à la RTC en même temps...
    TAKE_MUTEX(Mutex_Horloge);                              // On s'accapare le bus I2C de la RTC

        RTC_AccessReadRegister(RegistreDeBase);             // On accède en mode Lecture au registre désiré, dans la RTC.
      
        // La RTC envoie des données tant qu'on n'envoie pas un NAK
      
        MSB = I2C_ReceiveData(RTC_I2C_BUS);                 // Lecture des MSB de la Température
    
        I2C_ToggleACK(RTC_I2C_BUS,DISABLE);                 // Clear ACK, car l'octet suivant sera le dernier qu'on veut recevoir (il recevra un NAK)
    
        LSB = I2C_ReceiveData(RTC_I2C_BUS);                 // Lecture des LSB de la Température 
    
        I2C_GenerateStop(RTC_I2C_BUS);                      // Génère le signal Stop
        I2C_ToggleACK(RTC_I2C_BUS,ENABLE);                  // Enable ACKnowledge
    
    GIVE_MUTEX(Mutex_Horloge);                              // On libère le bus I2C de la RTC

    Temperature = (unsigned short) MSB << 8;
    Temperature |= LSB;
    Temperature = (Temperature * 10) >> 8;                  // On renvoie la température x 10, pour accéder au 1/10é de Degré (206 = 20.6°C).
  }

  return (Temperature);
}

void RTC_TuneOSC(char Offset)                               // Permet de compenser en + ou en - la fréquence de l'oscillateur interne de la RTC
{                                                           // Registre 0x10, Doc RTC page 15.
unsigned char   Registre;

  Registre = 0x10;
  RTC_WriteRegister(Registre, Offset);   // 1 LSB = 0.12ppm, Offset Positif (< 128) ralentit l'horloge, Offset négatif (>= 128) accélère l'horloge.
}



// ********************************************************************
// *   Section BMP180 : Initialisation bus I2C2 et gestion Baromètre  *
// ********************************************************************

void BARO_Setup(void)      // Configuration au niveau "Matériel" du bus I2C du baromètre BMP180
{
  I2C_Setup(BMP180_I2C);
  BARO_SetResolution(BARORESOLUTION);
}

void BARO_WriteReg(unsigned char Reg,unsigned char Valeur)  // Ecriture d'une Donnée dans un Registre du BMP180
{
  // ATTENTION : Le Mutex "Mutex_Barometre" DOIT être géré par la fonction qui fait appel à cette routine.

  I2C_GenerateStart(BMP180_I2C_BUS);                        // On envoie le signal "Start". On passe en mode Master
  I2C_SendAddress(BMP180_I2C_BUS,BMP180_WRITE_ADDR);        // On envoie l'adresse du BMP180, mode Ecriture
  I2C_SendData(BMP180_I2C_BUS,Reg);                         // On envoie l'Adresse du Registre à écrire.
  I2C_SendData(BMP180_I2C_BUS,Valeur);                      // On envoie la Valeur à écrire dans ce Registre.
  I2C_GenerateStop(BMP180_I2C_BUS);                         // Génère le signal Stop
  I2C_ToggleACK(BMP180_I2C_BUS,ENABLE);                     // Enable ACKnowledge
}

unsigned char BARO_ReadReg(unsigned char Reg)                // Lecture de la Valeur d'un Registre du BMP180
{
unsigned char   Data;

  // ATTENTION : Le Mutex "Mutex_Barometre" DOIT être géré par la fonction qui fait appel à cette routine.

  I2C_GenerateStart(BMP180_I2C_BUS);                     // On envoie le signal "Start". On passe en mode Master
  I2C_SendAddress(BMP180_I2C_BUS,BMP180_WRITE_ADDR);     // On envoie l'adresse du BMP180, mode Ecriture
  I2C_SendData(BMP180_I2C_BUS,Reg);                      // On envoie l'Adresse du Registre
  I2C_GenerateStart(BMP180_I2C_BUS);                     // On envoie le signal "Start" (Repeated Start)
  I2C_SendAddress(BMP180_I2C_BUS,BMP180_READ_ADDR);      // On envoie l'Adresse du BMP180, en mode Lecture (on veut Lire un Registre)

  I2C_ToggleACK(BMP180_I2C_BUS,DISABLE);                 // Disable ACKnowledge
  I2C_SetACKPos(BMP180_I2C_BUS,ACK_POS_CURRENT);         // Le NACK sera envoyé pour l'octet en cours de réception

  Data  = I2C_ReceiveData(BMP180_I2C_BUS);

  I2C_GenerateStop(BMP180_I2C_BUS);                      // Génère le signal Stop
  I2C_ToggleACK(BMP180_I2C_BUS,ENABLE);                  // Enable ACKnowledge
  
  return (Data);
}

unsigned short BARO_ReadUT(void)   // On lit la Température non-compensée, Doc BMP180 page 15. Sert dans les calculs pour compenser la Pression Atmosphérique.
{
unsigned char   MSB,LSB;
unsigned short  myUT;

  // ATTENTION : Le Mutex "Mutex_Barometre" DOIT être géré par la fonction qui fait appel à cette routine.

  // Procédure à suivre (Doc BMP180 page 15):
  // 1) On écrit 0x2E dans le Registre 0xF4 (Demande de Lecture de la Température - 6 ms)
  // 2) Attente de la Fin de Conversion
  // 3) On lit le résultat dans les Registres 0xF6 et 0xF7

  BARO_WriteReg(0xF4,0x2E);
  delay(BAROMAXTEMPERATUREACQTIME); // 6ms
  myUT = I2C_StdReadReg16(BMP180_I2C_BUS,0xF6,BMP180_READ_ADDR, BMP180_WRITE_ADDR);

  return (myUT);
}

void UT2Temp(BAROCalibPtr BAROCal,unsigned short UT,signed short *RealTemp,signed long *_B5)
{
long        myUT,X1,X2,myB5;
short       myRealTemp;

  myUT    = (long) UT;

  X1      = (myUT - (long)Barometer.AC6) * ((long)Barometer.AC5) >> 15;
  X2      = ((long)Barometer.MC << 11) / (X1+(long)Barometer.MD);
  myB5    = X1 + X2;

  myRealTemp  = ((myB5 + 8) >> 4);  // Real Temperature in 1/10 °C (225 = 22.5 °C)
  
  *RealTemp = myRealTemp;
  *_B5     = myB5;
}

void BARO_Calibrate(void)  // On étalonne le Baromètre avec les données contenues dans son EEPROM interne
{
unsigned char     *p;
unsigned short    UT;
signed short      RealTemp;
signed long       _B5;
unsigned char     Adresse,ChipID;

  // ATTENTION : Le Mutex "Mutex_Barometre" DOIT être géré par la fonction qui fait appel à cette routine.

  ChipID =  BARO_ReadReg(0xD0);   // Cette lecture pour s'assurer que le BMP180 est bien initialisé...

  // Adresse du premier registre d'étalonnage (0xAA), Doc BMP180 page 15.
  
  Adresse = 0xAA;

  Barometer.AC1 = (int16_t) I2C_StdReadReg16(BMP180_I2C_BUS,Adresse,BMP180_READ_ADDR, BMP180_WRITE_ADDR);
  Adresse += 2;
  Barometer.AC2 = (int16_t) I2C_StdReadReg16(BMP180_I2C_BUS,Adresse,BMP180_READ_ADDR, BMP180_WRITE_ADDR);
  Adresse += 2;
  Barometer.AC3 = (int16_t) I2C_StdReadReg16(BMP180_I2C_BUS,Adresse,BMP180_READ_ADDR, BMP180_WRITE_ADDR);
  Adresse += 2;
  Barometer.AC4 = I2C_StdReadReg16(BMP180_I2C_BUS,Adresse,BMP180_READ_ADDR, BMP180_WRITE_ADDR);
  Adresse += 2;
  Barometer.AC5 = I2C_StdReadReg16(BMP180_I2C_BUS,Adresse,BMP180_READ_ADDR, BMP180_WRITE_ADDR);
  Adresse += 2;
  Barometer.AC6 = I2C_StdReadReg16(BMP180_I2C_BUS,Adresse,BMP180_READ_ADDR, BMP180_WRITE_ADDR);
  Adresse += 2;
  Barometer._B1 = (int16_t) I2C_StdReadReg16(BMP180_I2C_BUS,Adresse,BMP180_READ_ADDR, BMP180_WRITE_ADDR);
  Adresse += 2;
  Barometer._B2 = (int16_t) I2C_StdReadReg16(BMP180_I2C_BUS,Adresse,BMP180_READ_ADDR, BMP180_WRITE_ADDR);
  Adresse += 2;
  Barometer.MB  = (int16_t) I2C_StdReadReg16(BMP180_I2C_BUS,Adresse,BMP180_READ_ADDR, BMP180_WRITE_ADDR);
  Adresse += 2;
  Barometer.MC  = (int16_t) I2C_StdReadReg16(BMP180_I2C_BUS,Adresse,BMP180_READ_ADDR, BMP180_WRITE_ADDR);
  Adresse += 2;
  Barometer.MD  = (int16_t) I2C_StdReadReg16(BMP180_I2C_BUS,Adresse,BMP180_READ_ADDR, BMP180_WRITE_ADDR);

  Barometer.Altitude = MyPrefs.GeoPosition[MyPrefs.GeoPosSelect].Altitude;
  Barometer.OSS = BARORESOLUTION;
       
  // On lit la température, pour initialiser le calcul de _B5

  UT = BARO_ReadUT();
  UT2Temp((BAROCalibPtr)(void*)&Barometer,UT,&RealTemp,&_B5);

  Barometer.Temperature = RealTemp;
  Barometer._B5         = _B5;
}

// Calcul de la Pression "Météo" (Niveau de la mer) et de la Pression Absolue à partir de la valeur de Pression retournée par le BMP180 (Pression non-compensée).
// On compense en tenant compte de l'Altitude de l'emplacement.
// On prend aussi en compte le paramètre de sur-échantillonnage utilisé par le BMP180 pour l'acquisition de la Pression (OSS).

void UP2Pressure(BAROCalibPtr BAROCal,unsigned long UP,signed short Altitude,unsigned short OSS,unsigned long *AbsPress,unsigned long *Press0)
{
int32_t     ut = 0, up = 0, compp = 0;
int32_t     x1, x2, b5, b6, x3, b3, p;
uint32_t    b4, b7;
float       AbsolutePress,MyAltitude,SeaLevelPress;

  // Lecture de la Température et de la Pression "brutes"
  
  ut = Barometer.UT;
  up = Barometer.UP;
  
  // b5 est utilisé pour la Compensation en température
  
  b5 = Barometer._B5;

  // Calcul de la Pression Barométrique Absolue, compensée
  
  b6 = b5 - 4000;
  x1 = (Barometer._B2 * ((b6 * b6) >> 12)) >> 11;
  x2 = (Barometer.AC2 * b6) >> 11;
  x3 = x1 + x2;
  b3 = (((((int32_t) Barometer.AC1) * 4 + x3) << Barometer.OSS) + 2) >> 2;
  x1 = (Barometer.AC3 * b6) >> 13;
  x2 = (Barometer._B1 * ((b6 * b6) >> 12)) >> 16;
  x3 = ((x1 + x2) + 2) >> 2;
  b4 = (Barometer.AC4 * (uint32_t) (x3 + 32768)) >> 15;
  b7 = ((uint32_t) (up - b3) * (50000 >> Barometer.OSS));

  if (b7 < 0x80000000) p = (b7 << 1) / b4;
  else p = (b7 / b4) << 1;

  x1 = (p >> 8) * (p >> 8);
  x1 = (x1 * 3038) >> 16;
  x2 = (-7357 * p) >> 16;
  compp = p + ((x1 + x2 + 3791) >> 4);
  
  // Calcul de la Pression Barométrique "Météo" (niveau de la mer)

  AbsolutePress = (float) compp;
  MyAltitude = (float) Barometer.Altitude;
  //SeaLevelPress = AbsolutePress / powf((1.0f - ( MyAltitude / 44330.76067f )), 5.25588f);  
  //SeaLevelPress = AbsolutePress / pow(1.0 - (MyAltitude/44330.0), 5.255);
  SeaLevelPress = AbsolutePress / powf(1.0 - (MyAltitude/44330.76067), 5.25588);  // powf() est préférable au pow() d'Arduino, 2K de Flash en moins et 200µs de gagnées...

  // Compensation avec notre variable d'Etalonnage

  SeaLevelPress += (float) MyPrefs.EtalonnageBARO;
  SeaLevelPress -= 10000.0;
  
  *AbsPress = compp;                     // Pression Barométrique Absolue
  *Press0   = (unsigned long) SeaLevelPress;
}

// BARO_ReadUP lit la valeur de UP (Pression Barométrique non-compensée) dans le BMP180.

unsigned long BARO_ReadUP(void)
{
unsigned char   BMPMode;
unsigned char   p8;
unsigned short  p16;
unsigned long   p32;
unsigned long   StartTime;

  // ATTENTION : Le Mutex "Mutex_Barometre" DOIT être géré par la fonction qui fait appel à cette routine.

  // Procédure à suivre (Doc BMP180 page 15):
  // 1) On écrit 0x34 + (OSS << 6) dans le Registre 0xF4 (Demande de Lecture de la Pression - 27 ms)
  // 2) Attente de la Fin de Conversion
  // 3) On lit le résultat dans les Registres 0xF6, 0xF7 et 0xF8

  BMPMode = (unsigned char) Barometer.OSS;  // Facteur de sur-échantillonnage (0...3)
  BARO_WriteReg(0xF4,0x34 + (BMPMode << 6));

  StartTime = millis();
  while ((millis() - StartTime) < BAROMAXCONVERSIONTIME) {};
  
  p16 = I2C_StdReadReg16(BMP180_I2C_BUS,0xF6,BMP180_READ_ADDR, BMP180_WRITE_ADDR);
  p32 = (uint32_t)p16 << 8;
  p8 = BARO_ReadReg(0xF8);
  p32 += p8;
  p32 >>= (8 - BMPMode);
  
  return (p32);
}

void BARO_SetResolution(unsigned short OSS)
{
  Barometer.OSS = 0;  // Default
  if (OSS > 3) return;
  Barometer.OSS = OSS;
}


// *******************************************************************
// *  Section Prefs : Gestion de nos Préférences (stockées en FRAM)  *
// *******************************************************************

#if (BOARD_USED == BOARD1)

    void SetDefaultPrefs(void) // Initialise nos préférences par défaut et les enregistre en FRAM.
    {
    unsigned short  DataLen;
    unsigned char   *p;
      
      MyPrefs.PrefsVersion    = 0x0100;       // Prefs Version 1.0
      
      p = MyPrefs.GeoPosition[0].Emplacement;
      *p++ = 'B'; *p++ = 'R'; *p++ = 'O'; *p++ = 'N'; *p++ = ' '; *p++ = 'A'; *p++ = 'v'; *p++ = 'i'; *p++ = 'a'; *p++ = 't'; *p++ = 'i'; *p++ = 'o'; *p++ = 'n'; *p++ = 0x00;
      MyPrefs.GeoPosition[0].Altitude = 165;          // 165 mètres.
      
      MyPrefs.GeoPosSelect            = 0;            // "BRON Aviation" sélectionné
      MyPrefs.EtalonnageBARO          = 10265;        // Etalonnage du calcul de la Pression Atmosphérique : 10000 est la valeur centrale. 1 unité = 1Pa. Augmenter si valeur calculée trop faible.
      MyPrefs.VCCvoltage              = 3313;         // Tension de sortie du Régulateur 3.3V, en millivolts). Varie d'un composant à un autre à 1 ou 2%. Mesuré avec Multimètre.
      MyPrefs.EtalonnageVBAT          = 9983;         // Etalonnage du calcul de VBAT : 10000 est la valeur centrale (aucun ajustement). 1 unité = 1mV. Augmenter si valeur calculée trop faible.
      MyPrefs.EtalonnageLum           = 3850;         // Etalonnage du calcul de la Luminosité (en Lux)
      MyPrefs.FrequenceLum            = 3;            // Fréquence des Conversions pour Luminosité = 3Hz
      MyPrefs.EtalonnageTemp          = 10016;        // Etalonnage du calcul de Température (en °C)
      MyPrefs.FrequenceTemp           = 2;            // Fréquence des Conversions pour Température = 2Hz
      MyPrefs.EtalonnageHum           = 4096;         // Etalonnage du calcul d'Humidité (en %RH)
      MyPrefs.FrequenceHum            = 1;            // Fréquence des Conversions pour Humidité = 1Hz
      MyPrefs.ModeTravail             = MODE_BASIC;   // Mode de travail de la Station = BASIC
      MyPrefs.DureeSalve              = 5;            // 5 secondes (c'est le maximum) de mémorisation en RAM des mesures en mode "SALVE"
      MyPrefs.ConsigneLum             = 65535;        // Consigne Luminosité pour envoi mesures au PC, mode "EVENEMENTIEL"
      MyPrefs.ConsigneTemp            = 65535;        // Consigne Luminosité pour envoi mesures au PC, mode "EVENEMENTIEL"
      MyPrefs.ConsigneHum             = 65535;        // Consigne Luminosité pour envoi mesures au PC, mode "EVENEMENTIEL"
      
      // Maintenant on enregistre les Prefs dands la FRAM
    
      p = (unsigned char*) &MyPrefs;
      DataLen = sizeof(PrefsRec);
      FRAM_WriteAddress((unsigned long) FRAM_PREFS_BASEADDR, p, DataLen);
    }

#elif (BOARD_USED == PROTOTYPE)

    void SetDefaultPrefs(void) // Initialise nos préférences par défaut et les enregistre en FRAM.
    {
    unsigned short  DataLen;
    unsigned char   *p;
      
      MyPrefs.PrefsVersion    = 0x0100;       // Prefs Version 1.0
      
      p = MyPrefs.GeoPosition[0].Emplacement;
      *p++ = 'B'; *p++ = 'R'; *p++ = 'O'; *p++ = 'N'; *p++ = ' '; *p++ = 'A'; *p++ = 'v'; *p++ = 'i'; *p++ = 'a'; *p++ = 't'; *p++ = 'i'; *p++ = 'o'; *p++ = 'n'; *p++ = 0x00;
      MyPrefs.GeoPosition[0].Altitude = 165;          // 165 mètres.
      
      MyPrefs.GeoPosSelect            = 0;            // "BRON Aviation" sélectionné
      MyPrefs.EtalonnageBARO          = 10000;        // Etalonnage du calcul de la Pression Atmosphérique : 10000 est la valeur centrale. 1 unité = 1Pa. Augmenter si valeur calculée trop faible.
      MyPrefs.VCCvoltage              = 3300;         // Tension de sortie du Régulateur 3.3V, en millivolts). Varie d'un composant à un autre à 1 ou 2%. Mesuré avec Multimètre.
      MyPrefs.EtalonnageVBAT          = 9983;         // Etalonnage du calcul de VBAT : 10000 est la valeur centrale (aucun ajustement). 1 unité = 1mV. Augmenter si valeur calculée trop faible.
      MyPrefs.EtalonnageLum           = 3670;         // Etalonnage du calcul de la Luminosité (en Lux)
      MyPrefs.FrequenceLum            = 2;            // Fréquence des Conversions pour Luminosité = 2Hz
      MyPrefs.EtalonnageTemp          = 4096;         // Etalonnage du calcul de Température (en °C)
      MyPrefs.FrequenceTemp           = 1;            // Fréquence des Conversions pour Température = 1Hz
      MyPrefs.EtalonnageHum           = 4096;         // Etalonnage du calcul d'Humidité (en %RH)
      MyPrefs.FrequenceHum            = 1;            // Fréquence des Conversions pour Humidité = 1Hz
      MyPrefs.ModeTravail             = MODE_BASIC;   // Mode de travail de la Station = BASIC
      MyPrefs.DureeSalve              = 5;            // 5 secondes (c'est le maximum) de mémorisation en RAM des mesures en mode "SALVE"
      MyPrefs.ConsigneLum             = 65535;        // Consigne Luminosité pour envoi mesures au PC, mode "EVENEMENTIEL"
      MyPrefs.ConsigneTemp            = 65535;        // Consigne Luminosité pour envoi mesures au PC, mode "EVENEMENTIEL"
      MyPrefs.ConsigneHum             = 65535;        // Consigne Luminosité pour envoi mesures au PC, mode "EVENEMENTIEL"
      
      // Maintenant on enregistre les Prefs dands la FRAM
    
      p = (unsigned char*) &MyPrefs;
      DataLen = sizeof(PrefsRec);
      FRAM_WriteAddress((unsigned long) FRAM_PREFS_BASEADDR, p, DataLen);
    }
    
#endif


void ReadMyPrefs(void)   // On lit nos Préférences dans la FRAM
{
unsigned short  DataLen;
unsigned char   *p;

  p = (unsigned char*) &MyPrefs;
  DataLen = sizeof(PrefsRec);
  FRAM_ReadAddress((unsigned long) FRAM_PREFS_BASEADDR, p, DataLen);
}


// ******************************************************************************************************
// *   Section surveillance VBAT (Tension Batterie) avec 10 Conversions par ADC1 et Transfert par DMA   *
// ******************************************************************************************************

void VBATMon_Setup(void)
{
unsigned char         NbConv,i;
unsigned long         dma_mode;
dma_channel_reg_map   *DMA1_Channel1;


  // 1) On active le Prescaler des ADC, l'Horloge d'ADC1 et on reset l'ADC1.

  RCC_BASE->CFGR     &= ~(RCC_CFGR_ADCPRE);                             // RAZ des bits du Prescaler de l'ADC
  RCC_BASE->CFGR     |= RCC_CFGR_ADC_CLOCK;                             // Fréquence ADC1 & ADC2 = 72MHz / 8 = 9MHz, Reference Manual page 100.
   
  RCC_BASE->APB2ENR  |= (unsigned long)(1 << RCC_APB2_ADC1EN_BIT);      // On active l'horloge de l'ADC1 sur le Bus APB2, Reference Manual pages 111-112.
  
  RCC_BASE->APB2RSTR |= RCC_APB2RSTR_ADC1_MASK;                         // Reset de l'ADC1, Manual Reference page 106.
  RCC_BASE->APB2RSTR &= ~(RCC_APB2RSTR_ADC1_MASK);

  // 2) Configuration de l'ADC1, pour effectuer 10 Conversions A/D avec transfert par DMA
   
      // a) On indique le Nombre de Conversions à effectuer en Séquence
      
      NbConv = 10;                                                     // La séquence des Conversions A/D d'ADC1 comporte 10 Conversions
      
      VBMON_ADC->regs->SQR1 &= ~(0x0F << 20);                          // RAZ de NbConv (L[3..0]) Reference Manual page 247
      VBMON_ADC->regs->SQR1 |= (NbConv - 1) << 20;                     // On enregistre le nombre de Conversions dans SQR1
    
      // b) On indique pour chaque Conversion sur quel Canal A/D elle devra s'effectuer (registres SQ3, SQ2 et SQ1)

      // Registre SQR3 pour Conversions 1 à 6

      VBMON_ADC->regs->SQR3 &= 0x30000000;                             // RAZ de tous les Canaux de ce registre, Reference Manual page 249.
      VBMON_ADC->regs->SQR3 |= ADC_CONV1(3);                           // La première  Conversion de la Séquence utilisera le Canal A/D de VBATMON (AN3 - n° 3)
      VBMON_ADC->regs->SQR3 |= ADC_CONV2(3);                           // La deuxième  Conversion de la Séquence utilisera le Canal A/D de VBATMON (AN3 - n° 3)
      VBMON_ADC->regs->SQR3 |= ADC_CONV3(3);                           // La troisième Conversion de la Séquence utilisera le Canal A/D de VBATMON (AN3 - n° 3)
      VBMON_ADC->regs->SQR3 |= ADC_CONV4(3);                           // La quatrième Conversion de la Séquence utilisera le Canal A/D de VBATMON (AN3 - n° 3)
      VBMON_ADC->regs->SQR3 |= ADC_CONV5(3);                           // La cinquième Conversion de la Séquence utilisera le Canal A/D de VBATMON (AN3 - n° 3)
      VBMON_ADC->regs->SQR3 |= ADC_CONV6(3);                           // La sixième   Conversion de la Séquence utilisera le Canal A/D de VBATMON (AN3 - n° 3)

      // Registre SQR2 pour Conversions 7 à 10

      VBMON_ADC->regs->SQR2 &= 0x30000000;                             // RAZ de tous les Canaux de ce registre, Reference Manual page 248.
      VBMON_ADC->regs->SQR2 |= ADC_CONV7(3);                           // La septième  Conversion de la Séquence utilisera le Canal A/D de VBATMON (AN3 - n° 3)
      VBMON_ADC->regs->SQR2 |= ADC_CONV8(3);                           // La huitième  Conversion de la Séquence utilisera le Canal A/D de VBATMON (AN3 - n° 3)
      VBMON_ADC->regs->SQR2 |= ADC_CONV9(3);                           // La neuvième  Conversion de la Séquence utilisera le Canal A/D de VBATMON (AN3 - n° 3)
      VBMON_ADC->regs->SQR2 |= ADC_CONV10(3);                          // La dixième   Conversion de la Séquence utilisera le Canal A/D de VBATMON (AN3 - n° 3)

      // Le registre SQR1 (pour les Conversions n° 13 à 16) n'est pas utilisé ici, puisqu'on n'en fait que 10...
     
      // c) On indique pour chaque Canal A/D le temps d'échantillonnage (en nombre de cycles de l'horloge ADC, que nous avons paramétrée ici à 9MHz
      //    Comme on utilise uniquement le Canal A/D n° 3, on ne configure que lui.
      
      VBMON_ADC->regs->SMPR2 = ADC_SAMPLE_TIME3(VBATMON_SAMPLE_TIME);  // Temps d'échantillonnage de 239.5 cycles d'Horloge ADC (c'est le maximum) pour le Canal A/D n° 3 (TIME3).
                                                                       // L'horloge de l'ADC étant fixée à 9MHz, ça fait 26.6 µs de temps d'échantilonnage.
                                                                       // Notre pont diviseur de VBAT ayant une impédance élevée, on doit utiliser un temps d'échantillonnage long.
                                                                       // Le Temps de Conversion (Tconv) = Tech + 12.5 cycles, Reference Manual page 225.
                                                                       // Avec notre horloge ADC à 9MHz, ça donne (239.5 + 12.5)/(9x10^6) = 28µs pour chaque Conversion A/D.
                                                                       // (On ne peut pas faire plus lent avec la fréquence Processeur à 72MHz).
                                                                       
  // 3) On paramètre le Mode de Fonctionnement de l'ADC1, on l'active et on l'étalonne.

  VBMON_ADC->regs->CR2   |= (ADC_CR2_ADON | ADC_CR2_CONT | ADC_CR2_DMA);  // On active l'ADC1, Conversions en continu avec DMA (ADC_CR2_CONT nécessaire).
  delayMicroseconds(1);                                                   // Attente d'1µs le temps que l'ADC démarre, Reference Manual page 218.
  VBMON_ADC->regs->CR2   |= ADC_CR2_CAL;                                  // On étalonne le convertisseur A/D, Reference Manual page 223 (recommandé).
  while (VBMON_ADC->regs->CR2 & ADC_CR2_CAL) {};                          // Attente de la fin de l'auto-étalonnage.

  VBMON_ADC->regs->CR1   |= ADC_CR1_SCAN;                                 // On active le Mode "SCAN", pour que l'ADC "scanne" les Canaux définis dans SQR3 et SQR2, Reference Manual page 239.

  // 4) Configuration du DMA                                              // Seul l'ADC1 est utilisable avec le DMA, Reference Manual page 227.
                                                                          // Le STM32F103 n'a qu'un seul Controleur DMA (DMA1), qui possède 7 Canaux.
      // a) On active l'horloge de DMA1
    
      RCC_BASE->AHBENR  |= RCC_AHBENR_DMA1EN;
    
      // b) On obtient l'adresse des registres du Canal DMA sélectionné, dans DMA1
      
      DMA1_Channel1 = dma_channel_regs(DMA1, DMA_CH1);                            // On utilise ici le Canal DMA n° 1, le seul pouvant être déclenché par l'ADC1, Reference Manual pages 281-282.
                                                                                  // NOTE : Sur le STM32F103, le "Déclencheur" DMA (Trigger) du Canal DMA N° 1 est partagé avec ces Périphériques :
                                                                                  // TIM2_CH3 et TIM4_CH1, Reference Manual page 282. Ici, on ne peut donc pas utiliser ces périphériques pour le DMA.
                                                                              
      // c) On configure les Adresses Source (ADC1 Data Register) et Destination (Tableau en variable Globale, avec alignement 16 bit).
      
      DMA1_Channel1->CPAR   = (volatile unsigned long) (&(VBMON_ADC->regs->DR));  // Adresse (Source) de Base du Périphérique (ADC1 Data Register). CPAR reste à la valeur assignée.                    
      DMA1_Channel1->CMAR   = (unsigned long) DMA_VBAT_Buffer;                    // Adresse (Desination) de Base en mémoire RAM (tableau en Variable Globale). CMAR reste à la valeur assignée.
      
      // d) On configure le Nombre de Transferts à effectuer
    
      DMA1_Channel1->CNDTR  = NbConv;                                             // Le Controleur DMA doit effectuer 10 transferts, PAS 10 octets... Ce registre est décrémenté après chaque transfert.
                                                                                  // en mode non-circulaire, une fois tous les transferts effectués : on doit recharger ce registre.
                                                                                  // Mais pour celà, il faut que le Canal DMA soit désactivé, Reference Manual page 278.
                                                                                  // Ici, on n'utilise pas le Mode Circulaire, pour pouvoir déclencher de nouveaux Transferts à la demande, et non pas en continu.
                                                                                  // Donc on doit "Recharger" ce registre.
                                                                                  
      // e) On définit la Priorité accordée à ce Canal DMA (DMA_CCR_PL_LOW, DMA_CCR_PL_MEDIUM, DMA_CCR_PL_HIGH ou DMA_CCR_PL_VERY_HIGH)
    
      DMA1_Channel1->CCR    &= 0xFFFF0000;                 // Reset des bits [0..15] du Registre CCR, Reference Manual page 286.
      dma_mode               = 0L;                         // On initialise dma_mode à Zéro.
      dma_mode              |= VBATMON_DMA_PRIORITY;       // On fixe la Priorité du Canal DMA.
      
      // f) On configure le mode de fonctionnement du Canal DMA (le mode Circulaire n'est pas activé, les Transferts DMA s'arrêtent après 10 transferts).
    
      dma_mode              |= DMA_CCR_MINC |              // Incrément automatique de l'adresse Mémoire après chaque Transfert (dans notre Tableau en Variable Globale)
                               DMA_CCR_MSIZE_16BITS |      // Données en Mémoire RAM (tableau en Variable Globale DMA_VBAT_Buffer[]) sur 16 bits
                               DMA_CCR_PSIZE_16BITS |      // Données du Périphérique sur 16 bits (Convertisseur ADC Data Register)
                               DMA_CCR_DIR_FROM_PER |      // Transfert depuis un Périphérique (ADC1) vers la RAM.
                               DMA_CCR_TCIE;               // Le DMA génère une interruption quand TOUS les transferts sont terminés (TCIE = Transfer Complete Interrupt Enable)
                             
      DMA1_Channel1->CCR    |= dma_mode;                   // On programme le registre CCR du Canal DMA n° 1 avec le mode de fonctionnement défini.
      DMA1_Channel1->CCR    |= DMA_CCR_EN;                 // On active le Canal 1 du DMA. Le Canal DMA n+ 1 est déclenché par ADC1 EOC, donc pour le moment rien ne bouge...
    

  // 4) Configuration du Port GPIOA, pour l'Entrée Analogique

  RCC_BASE->APB2ENR |= (unsigned long)(1 << RCC_APB2_IOPAEN_BIT);       // On active l'horloge du GPIOA sur le Bus APB2, Reference Manual pages 111-112.
  GPIO_Pinmode(VBATMON_AN3_GPIO,VBATMON_AN3_PIN, GPIO_INPUT_ANALOG);    // On définit la pin 3 du PortA comme Entrée Analogique.

  // 5) On active les Interruptions DMA
  //    ATTENTION : Lorsqu'on utilise le DMA, on ne DOIT PAS activer l'interruption EOCIE de l'ADC (ADC_CR1_EOCIE)
  //                et donc pour cet ADC1, pas besoin d'activer l'IRQ avec nvic_irq_enable(NVIC_ADC_1_2), sinon ça plante...
  //                Bien sûr, si on utilise ADC2 avec des Interruptions, on peut activer NVIC_ADC_1_2, pas de souci tant qu'ici on n'a PAS mis EOCIE à "1".

  VBMON_ADC->regs->CR1   &= ~(ADC_CR1_EOCIE);   // On met EOCIE à zéro
  nvic_irq_enable(NVIC_DMA_CH1);                // On active les Interruptions pour le Périphérique DMA1, Canal 1
  
  // INFO : Liste les Ints : "Program Files (x86)/Arduino/hardware/Arduino_STM32/STM32F1/system/libmaple/stm32f1/include/series/nvic.h"

  // 6) On initialise nos variables Globales, et on démarre les Conversions
    
  for (i = 0 ; i < 10 ; i++) DMA_VBAT_Buffer[i] = 0x0000; // Tableau pour DMA   (Variable Globale)
  VbatDMAComplete = false;                      // Flag de fin de transfert des 10 Conversions par DMA (Variable Globale)

  ADC_START_ACQ(VBMON_ADC);     // On lance les Conversion A/D, chaque EOC (End Of Conversion) lancera un transfert DMA, sans générer d'IRQ.
  //VBMON_ADC->regs->CR2 |= ADC_CR2_ADON;
  //VBMON_ADC->regs->CR2 |= ADC_CR2_SWSTART;      // On lance les Conversion A/D, chaque EOC (End Of Conversion) lancera un transfert DMA, sans générer d'IRQ.
}


// **********************************************************************
// *   Section pour les Capteurs Analogiques (Lum, Temp, RH) sur ADC2   *
// **********************************************************************

void Capteurs_Setup(void)
{
unsigned char     FrequenceConv;
unsigned long     NbConv;

  // 1) On active le Prescaler des ADC, l'Horloge d'ADC2 et on reset l'ADC2.

  RCC_BASE->CFGR     &= ~(RCC_CFGR_ADCPRE);                             // RAZ des bits du Prescaler de l'ADC
  RCC_BASE->CFGR     |= RCC_CFGR_ADC_CLOCK;                             // Fréquence ADC1 & ADC2 = 72MHz / 6 = 12MHz, Reference Manual page 100.
   
  RCC_BASE->APB2ENR  |= (unsigned long)(1 << RCC_APB2_ADC2EN_BIT);      // On active l'horloge de l'ADC2 sur le Bus APB2, Reference Manual pages 111-112.
  
  RCC_BASE->APB2RSTR |= RCC_APB2RSTR_ADC2_MASK;                         // Reset de l'ADC2, Manual Reference page 106.
  RCC_BASE->APB2RSTR &= ~(RCC_APB2RSTR_ADC2_MASK);

  // 2) Configuration de l'ADC2
   
      // a) On indique le Nombre de Conversions à effectuer en Séquence
      
      NbConv = 1;                                                      // La séquence des Conversions A/D ne comporte qu'une seule Conversion
      CAPTEURS_ADC->regs->SQR1 &= ~(0x0F << 20);                       // Clear NbConv (L[3..0]) Reference Manual page 247
      CAPTEURS_ADC->regs->SQR1 |= (NbConv - 1) << 20;                  // On enregistre le nombre de Conversions dans SQR1
    
      // b) On indique pour chaque Conversion sur quel Canal A/D elle devra s'effectuer (registres SQ3, SQ2 et SQ1)
      
      CAPTEURS_ADC->regs->SQR3  = ADC_CONV1(0);                        // La mesure de Luminosité s'effectue sur le Canal A/D n° 0
    
      // c) On indique pour chaque Canal A/D le temps d'échantillonnage (en nombre de cycles de l'horloge ADC, que nous avons paramétrée ici à 9MHz) :
      
      CAPTEURS_ADC->regs->SMPR2 |= ADC_SAMPLE_TIME0(CAPTEURS_SAMPLE_TIME);  // Temps d'échantillonnage de 239.5 cycles d'Horloge ADC (c'est le maximum) pour le Canal A/D n° 0 (TIME0).
      CAPTEURS_ADC->regs->SMPR2 |= ADC_SAMPLE_TIME1(CAPTEURS_SAMPLE_TIME);  // Temps d'échantillonnage de 239.5 cycles d'Horloge ADC (c'est le maximum) pour le Canal A/D n° 1 (TIME1).
      CAPTEURS_ADC->regs->SMPR2 |= ADC_SAMPLE_TIME2(CAPTEURS_SAMPLE_TIME);  // Temps d'échantillonnage de 239.5 cycles d'Horloge ADC (c'est le maximum) pour le Canal A/D n° 2 (TIME2).
                                                                            // Le Temps de Conversion (Tconv) = Tech + 12.5 cycles, Reference Manual page 225.
                                                                            // Avec notre horloge à ADC 9MHz, ça donne (239.5 + 12.5)/(9x10^6) = 28µs pour chaque Conversion A/D.
                                                                            // (On ne peut pas faire plus lent avec la fréquence Processeur à 72MHz).

  // 3) On paramètre le Mode de Fonctionnement de l'ADC2, on l'active et on l'étalonne.

  CAPTEURS_ADC->regs->CR2   |= (ADC_CR2_ADON);                     // On active l'ADC2, PAS de Conversions en continu.
  CAPTEURS_ADC->regs->CR2   &= ~(ADC_CR2_CONT);
  delayMicroseconds(1);                                            // Attente d'1µs le temps que l'ADC démarre, Reference Manual page 218.
  CAPTEURS_ADC->regs->CR2   |= ADC_CR2_CAL;                        // On étalonne le convertisseur A/D, Reference Manual page 223 (recommandé).
  while (CAPTEURS_ADC->regs->CR2 & ADC_CR2_CAL) {};                // Attente de la fin de l'auto-étalonnage.

  CAPTEURS_ADC->regs->CR1   = ADC_CR1_EOCIE;                       // On active l'Interruption de Fin de Conversion (EOCIE = End Of Conversion Interrupt Enable)

  // 4) Configuration du Port GPIOA, pour les 3 Entrée Analogiques

  RCC_BASE->APB2ENR |= (unsigned long)(1 << RCC_APB2_IOPAEN_BIT);       // On active l'horloge du GPIOA sur le Bus APB2, Reference Manual pages 111-112.
  GPIO_Pinmode(LUM_AN0_GPIO,LUM_AN0_PIN, GPIO_INPUT_ANALOG);            // On définit la pin 0 du PortA comme Entrée Analogique pour la Luminosité.
  GPIO_Pinmode(TEMP_AN1_GPIO,TEMP_AN1_PIN, GPIO_INPUT_ANALOG);          // On définit la pin 1 du PortA comme Entrée Analogique pour la Température.
  GPIO_Pinmode(HUM_AN2_GPIO,HUM_AN2_PIN, GPIO_INPUT_ANALOG);            // On définit la pin 2 du PortA comme Entrée Analogique pour l'Humidité.

  // 5) On configure le timer TIM2 qui nous sert à lancer les Conversions de Luminosité à la fréquence désirée (1Hz à 20Hz)

      // a) On récupère dans nos Préférences la Fréquence des Acquisitions de Luminosité
      
      FrequenceConv = MyPrefs.FrequenceLum;
      //FrequenceConv = 1;    // >>>> TEST<<<<

      // b) On configure le Timer
         
      MaConfigTimer(TIMER_2,FrequenceConv);   // On initialise et configure TIM2 pour notre usage

  // 6) On configure le timer TIM3 qui nous sert à lancer les Conversions de Temperature à la fréquence désirée (1Hz à 20Hz)

      // a) On récupère dans nos Préférences la Fréquence des Acquisitions de Température
      
      FrequenceConv = MyPrefs.FrequenceTemp;
      //FrequenceConv = 1;    // >>>> TEST<<<<
      
      // b) On configure le Timer

      MaConfigTimer(TIMER_3,FrequenceConv);   // On initialise et configure TIM3 pour notre usage

  // 7) On configure le timer TIM4 qui nous sert à lancer les Conversions d'Humidité à la fréquence désirée (1Hz à 20Hz)

      // a) On récupère dans nos Préférences la Fréquence des Acquisitions d'Humidité
      
      FrequenceConv = MyPrefs.FrequenceHum;
      //FrequenceConv = 1;    // >>>> TEST<<<<
      
      // b) On configure le Timer

      MaConfigTimer(TIMER_4,FrequenceConv);   // On initialise et configure TIM4 pour notre usage

  // 8) On active les Interruptions d'ADC2 et des Timers 2,3 et 4.
  
  nvic_irq_enable(NVIC_ADC_1_2);                // On active les Interruptions pour les périphériques ADC1 & ADC2 (celles de ADC1 ne sont pas activées puisqu'on utilise le DMA)
  nvic_irq_enable(NVIC_TIMER2);                 // On active les Interruptions pour le Timer 2. On en recevra 1 à chaque Overflow.
  nvic_irq_enable(NVIC_TIMER3);                 // On active les Interruptions pour le Timer 3. On en recevra 1 à chaque Overflow.
  nvic_irq_enable(NVIC_TIMER4);                 // On active les Interruptions pour le Timer 4. On en recevra 1 à chaque Overflow.
  
  // INFO : Liste les Ints : "Program Files (x86)/Arduino/hardware/Arduino_STM32/STM32F1/system/libmaple/stm32f1/include/series/nvic.h"

  // 7) On initialise nos variables Globales
  
  ValeurCapteur       = 0x0000;                 // Valeur "Brute" du Capteur
  ConversionComplete  = false;                  // Flag de "Fin de Conversion"
  CapteurSelect       = 0;                      // Capteur en cours d'acquisition (0 = Aucun)

  MustStartLum        = false;                  // Pour le moment on ne doit pas démarrer les Conversions de Luminosité
  MustStartTemper     = false;                  // Pour le moment on ne doit pas démarrer les Conversions de Température
  MustStartHum        = false;                  // Pour le moment on ne doit pas démarrer les Conversions d'Humidité
  
  CAPTEURS_ADC->regs->CR2 |= ADC_CR2_ADON;      // On valide la mise en route d'ADC2

  // 8) On démarre les 3 Timers qui vont déclencher les Conversions pour nos 3 capteurs Analogiques.
  
  //    Ici, nous ne voulons PAS synchroniser les Timers.
  //    On les décale de 2 millisecondes, car les 3 Capteurs partagent le même Convertisseur A/D (ADC2).
  //    Notre schéma de fonctionnement étant d'effectuer des Conversions pour chaque Capteur l'un après l'autre, les Conversions doivent donc être décalées.
  //    Le décalage étant très faible, les données des 3 capteurs seront reçues par le PC "simultanément" à nos yeux.
  //    A l'échelle de variation des données mesurées (Luminosité, Température et Humidité), on n'est pas à quelques millisecondes près.
  
  START_TIMER(TIMER2);  // On démarre le Timer "Luminosité", Macro définie dans "StationMeteo.h"
  delay(1);   // Délai de 2 ms, pour finir le travail Luminosité avant de recevoir l'ordre de travail sur Température
  
  START_TIMER(TIMER3);  // On démarre le Timer "Température", Macro définie dans "StationMeteo.h"
  delay(1);   // Délai de 2 ms, pour finir le travail Température avant de recevoir l'ordre de travail sur Humidité

  START_TIMER(TIMER4);  // On démarre le Timer "Humidité", Macro définie dans "StationMeteo.h"
}

// Configuration des Timers TIM2, TIM3 et TIM4

void MaConfigTimer(unsigned char Timer, unsigned char Frequence)  // Timer n°2, 3 ou 4, Frequence en Hz (1 à 20)
{
unsigned short    Periode;
timer_gen_reg_map *regs;

  if ((Frequence < 1) || (Frequence > 20)) Frequence = 5;
  Periode = TimerPeriode[Frequence - 1];

  // a) On active l'Horloge de TIMERx, et on fait son Reset

  switch (Timer) {
    case 2:   // TIM2
      RCC_BASE->APB1ENR  |= RCC_APB1ENR_TIM2EN;           // On active l'Horloge de TIMER2 sur bus APB1, Reference Manual pages 111-112.
      RCC_BASE->APB1RSTR |= RCC_APB1RSTR_TIMER2_MASK;     // Reset de TIMER2, Manual Reference page 108.
      RCC_BASE->APB1RSTR &= ~(RCC_APB1RSTR_TIMER2_MASK);

      regs = (TIMER2->regs).gen;                          // Pointeur sur les Registres de TIMER2 (General Timer = .gen)
    break;
    case 3:   // TIM3
      RCC_BASE->APB1ENR  |= RCC_APB1ENR_TIM3EN;           // On active l'Horloge de TIMER3 sur bus APB1, Reference Manual pages 111-112.     
      RCC_BASE->APB1RSTR |= RCC_APB1RSTR_TIMER3_MASK;     // Reset de TIMER3, Manual Reference page 108.
      RCC_BASE->APB1RSTR &= ~(RCC_APB1RSTR_TIMER3_MASK);
      
      regs = (TIMER3->regs).gen;                          // Pointeur sur les Registres de TIMER3 (General Timer = .gen)
    break;
    case 4:   // TIM4
      RCC_BASE->APB1ENR  |= RCC_APB1ENR_TIM4EN;           // On active l'Horloge de TIMER4 sur bus APB1, Reference Manual pages 111-112. 
      RCC_BASE->APB1RSTR |= RCC_APB1RSTR_TIMER4_MASK;     // Reset de TIMER4, Manual Reference page 108.
      RCC_BASE->APB1RSTR &= ~(RCC_APB1RSTR_TIMER4_MASK);

      regs = (TIMER4->regs).gen;                          // Pointeur sur les Registres de TIMER4 (General Timer = .gen)
    break;
  }
  
  // b) Configuration des Registres CR1, CR2, PSC, ARR et DIER, Reference Manual pages 403 à 423 :

  regs->CR1  &= ~(TIMER_CR1_CKD);       // Reset des bits du Diviseur
  regs->CR1  |= TIMER_CR1_CKD_1TCKINT;  // Clock Division 1:1 for sampling filters (inutilisé ici)
  regs->CR1  |= TIMER_CR1_ARPE;         // Auto-Reload activé, pour que ce Timer fonctionne en continu
  regs->CR1  &= ~(TIMER_CR1_DIR | TIMER_CR1_CKD_CMS);  // RAZ des bits DIR et CMS
  regs->CR1  |= TIMER_COUNTERMODE_UP;   // On fait compter le Timer de 0 à Periode, l'Interruption est générée lorsque Compteur = Periode. Ensuite Compteur est RAZ.
  regs->CR1  &= ~(TIMER_CR1_OPM);       // Le Compteur n'est PAS en mode "One Pulse", il compte indéfiniment
  regs->CR1  |= TIMER_CR1_URS;          // Seul l'Overflow génère une Interruption
  regs->CR1  &= ~(TIMER_CR1_UDIS);      // On autorise les Update Events (génération d'une Interruption)

  regs->CR2  &= ~ (TIMER_CR2_MMS);      // RAZ des bits MMS
  regs->CR2  |= TIMER_CR2_MMS_UPDATE;   // Compteur en mode "Update Events"
  
  regs->PSC   = TIMER_PRESCALER;        // PreScaler (voir "StationMeteo.h")
  regs->ARR   = Periode;                // Période. Avec Prescaler, ces deux valeurs définissent la fréquence des Interruptions "Update"

  regs->DIER |= TIMER_DIER_UIE;         // On demande au Timer de générer une Interruption pour un Update Event (UIE = Update Interrupt Enable).
}


// **************************************************************************
// *  ISR pour les Interruptions générées par le DMA, ADC2 et nos 3 Timers  *
// **************************************************************************

// ISR pour DMA Canal 1, Flag "VbatDMAComplete" utilisé par VBATMonitorTask()

extern "C" void __irq_dma1_channel1(void)
{
  if (DMA1->regs->ISR & DMA_ISR_TCIF1) {          // Le Canal 1 du DMA1 a fini ses transferts
    VBMON_ADC->regs->CR2 &= ~(ADC_CR2_SWSTART);   // On désactive les Conversions par la demande "Software".
    DMA1->regs->IFCR |= DMA_IFCR_CTCIF1;          // RAZ du flag DMA_ISR_TCIF1 dans le Registre ISR de DMA1, Reference Manual page 285.
    VbatDMAComplete = true;
  }
}

// ISR pour ADC2, utilisé pour lire les valeurs des 3 capteurs, Flag "ConversionComplete" utilisé par AnalogCapteursTask().

extern "C" void __irq_adc(void)
{
  if(CAPTEURS_ADC->regs->SR & ADC_SR_EOC) {   // Fin de Conversion
    ValeurCapteur = CAPTEURS_ADC->regs->DR;   // Lecture de la valeur brute de Luminosité, ça remet à zéro le bit EOC de SR (Status Register)
    ConversionComplete = 1;                   // On active le Flag "Fin de Conversion pour Capteur"
  }
}

extern "C" void __irq_tim2(void)  // ISR pour TIM2, timer qui déclenche les Conversions de Luminosité, Flag "MustStartLum" utilisé par AnalogCapteursTask().
{
timer_gen_reg_map   *regs;

  regs = (TIMER2->regs).gen;
  regs->SR = (unsigned short) ~(TIMER_SR_UIF);    // Clear Interrupt Flag "Update", seuls les "0" sont écrits car écrire des "1"s dans SR n'est pas possible...

  MustStartLum = true;    // On demande le démarrage des Conversions A/D pour la Luminosité
}

extern "C" void __irq_tim3(void)  // ISR pour TIM3, timer qui déclenche les Conversions de Température, Flag "MustStartTemper" utilisé par AnalogCapteursTask().
{
timer_gen_reg_map   *regs;

  regs = (TIMER3->regs).gen;
  regs->SR = (unsigned short) ~(TIMER_SR_UIF);    // Clear Interrupt Flag "Update", seuls les "0" sont écrits car écrire des "1"s dans SR n'est pas possible...

  MustStartTemper = true; // On demande le démarrage des Conversions A/D pour la Température
}

extern "C" void __irq_tim4(void)  // ISR pour TIM4, timer qui déclenche les Conversions d'Humidité, Flag "MustStartHum" utilisé par AnalogCapteursTask().
{
timer_gen_reg_map   *regs;

  regs = (TIMER4->regs).gen;
  regs->SR = (unsigned short) ~(TIMER_SR_UIF);    // Clear Interrupt Flag "Update", seuls les "0" sont écrits car écrire des "1"s dans SR n'est pas possible...

  MustStartHum = true;    // On demande le démarrage des Conversions A/D pour l'Humidité
}



