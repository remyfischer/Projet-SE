// Fichier "InterfacePC.ino" : Gestion des commandes reçues du logiciel spécifique PC.


#include "StationMeteo.h"
#include "InterfacePC.h"
#include <limits.h>       // Pour INT_MAX et INT_MIN

// InterfacePCTask() gère les Modes de travail suivants :
// ------------------------------------------------------

//    MODE_TEMPS_REEL   : Envoi des mesures demandées par le logiciel PC, réception des modifications de réglages
//    MODE_SALVES       : Bufférisation des mesures en RAM (1 à 5 secondes), puis envoi au logiciel PC
//    MODE_ONDEMAND     : Bufférisation des mesures en FRAM (temps selon capacité FRAM), puis envoi au logiciel PC
//    MODE_EVENEMENTIEL : Option du Groupe : Envoi des mesures uniquement si en-dessous d'un seuil paramétrable


void InterfacePCTask(void *pvParameters)
{
unsigned long   CurTime,NextDisplayTime;
boolean         SendLum, SendTempExt, SendHum, SendBaro, SendVBAT, SendTempInt, SendTimeStamp;
short           i,j,NbMesTotal,NbMesComplete;
PrivateDataRec  myParams;
unsigned char   TempsLum, TempsTemp, TempsHum, TempsVBAT, TempsBARO, TempsTempInt;
unsigned char   NbMesLum, NbMesTemp, NbMesHum;
unsigned long   timestamp, HectoPascals;
boolean         LumComplete, TempComplete, HumComplete, BaroComplete, VBATComplete, TempIntComplete;
signed short    Temperature;

  SendLum         = myParams.SendLum        = false;
  SendTempExt     = myParams.SendTempExt    = false;
  SendHum         = myParams.SendHum        = false;
  SendBaro        = myParams.SendBaro       = false;
  SendVBAT        = myParams.SendVBAT       = false;
  SendTempInt     = myParams.SendTempInt    = false;
  SendTimeStamp   = myParams.SendTimeStamp  = false;
  myParams.ModeTravail = ModeTravail;
  
  NextDisplayTime = 0L;   // Au départ, on affiche tout de suite les Mesures "Option" (hors Projet)
  
  while (1) {
    CurTime = millis();
    if (My_gestion_input(&myParams)) {   // On a reçu une Commande qui a modifié quelque chose
      SendLum         = myParams.SendLum;
      SendTempExt     = myParams.SendTempExt;
      SendHum         = myParams.SendHum;
      SendBaro        = myParams.SendBaro;
      SendVBAT        = myParams.SendVBAT;
      SendTempInt     = myParams.SendTempInt;
      SendTimeStamp   = myParams.SendTimeStamp;
      ModeTravail     = myParams.ModeTravail;
    }

    // *********************************************
    // *   Gestion de Mode n°1 : MODE_TEMPS_REEL   *
    // *********************************************

    if (ModeTravail == MODE_TEMPS_REEL) {   // Envoi des Mesures à la fréquence paramétrée (1Hz à 20Hz)
      
      // On commence par les Mesures du Projet : Luminosité, Température et Humidité.

      if (SendLum) {          // La Luminosité de notre Capteur Analogique, si demandée.
        if (NewLuminositeDispo) {
          if (SendTimeStamp) timestamp = TimeStamp(); GiveTimeStamp(timestamp);
          GiveLum(C_LumLux);
          NewLuminositeDispo = false;
        }
      }

      if (SendTempExt) {      // La Température de notre Capteur Analogique, si demandée.
        if (NewTemperatureDispo) {
          if (SendTimeStamp) timestamp = TimeStamp(); GiveTimeStamp(timestamp);
          GiveTemp(C_Temperature);
          NewTemperatureDispo = false;
        }        
      }

      if (SendHum) {          // L'Humidité de notre Capteur Analogique, si demandée.
        if (NewHumiditeDispo) {
          if (SendTimeStamp) timestamp = TimeStamp(); GiveTimeStamp(timestamp);
          GiveHum(C_Humidite);
          NewHumiditeDispo = false;
        }
      }

      // Ensuite, les Mesures complémentaires : Pression Atmosphérique, Tension Batterie et Température Interne (1 fois par seconde).

      if (CurTime >= NextDisplayTime) {
        
        if (SendBaro) {                       // La Pression Barométrique (Capteur Digital sur bus I2C), si demandée.
          if (BarometreOK) {
            TAKE_MUTEX(Mutex_Barometre);
              HectoPascals = Barometer.MeteoPressure;
            GIVE_MUTEX(Mutex_Barometre);
            if (HectoPascals != 0L) {
              GiveBaro(HectoPascals);
            }
          }
        }
        
        if (SendVBAT) {                       // La Tension Batterie Principale (Mesure Analogique), si demandée.
          if (VbatGood) {                     // VbatGood est activé par la Tache VBATMonitorTask()
            GiveVBAT(TensionBatterie);
          }
        }
        
        if (SendTempInt)  {                   // La Température Interne de la Station (Capteur Digital sur bus I2C), si demandée.
          if (BarometreOK) {
            TAKE_MUTEX(Mutex_Barometre);
              Temperature = Barometer.Temperature;
            GIVE_MUTEX(Mutex_Barometre);
            GiveTempInt(Temperature);
          }
        }
        
        NextDisplayTime = CurTime + 1000;     // On les renverra dans 1 seconde.
      }
    }

    // *****************************************
    // *   Gestion de Mode n°2 : MODE_SALVES   *
    // *****************************************

    if (ModeTravail == MODE_SALVES) {         // Stockage des Mesures en RAM pendant 1 à 5 secondes, et Envoi des Mesures au PC au bout du délai fixé (paramétrable).

      // On utilise le Record Salve, pour stocker les données. Mais on doit savoir s'il faut l'initialiser (lorsqu'on "bascule" dans ce mode)

      if (! ModeSalvesInited) {               // Non, le Record n'est pas initialisé
        Salve.StartTimeStamp  = TimeStamp();  // On met UN SEUL Horodatage pour TOUTES les Mesures bufférisées (elles auront donc toutes le même).
        Salve.NbSec           = MyPrefs.DureeSalve;
        Salve.FreqLum         = MyPrefs.FrequenceLum;
        Salve.FreqTemp        = MyPrefs.FrequenceTemp;
        Salve.FreqHum         = MyPrefs.FrequenceHum;
        for (i = 0 ; i < 20 ; i++) for (j = 0 ; j < 5 ; j++) Salve.DataLum[i][j] = 0x0000;
        for (i = 0 ; i < 20 ; i++) for (j = 0 ; j < 5 ; j++) Salve.DataTemp[i][j] = 0x0000;
        for (i = 0 ; i < 20 ; i++) for (j = 0 ; j < 5 ; j++) Salve.DataHum[i][j] = 0x0000;
        for (i = 0 ; i < 5 ; i++) Salve.DataVBAT[i] = 0x0000;
        for (i = 0 ; i < 5 ; i++) Salve.DataBARO[i] = 0x0000; 
        for (i = 0 ; i < 5 ; i++) Salve.DataTempInt[i] = 0x0000;

        // Initialisation de nos Compteurs et Flags

        TempsLum          = NbMesLum    = 0;
        TempsTemp         = NbMesTemp   = 0;
        TempsHum          = NbMesHum    = 0;
        TempsVBAT         = 0;
        TempsBARO         = 0;
        TempsTempInt      = 0;

        LumComplete       = false;
        TempComplete      = false;
        HumComplete       = false;
        BaroComplete      = false;
        VBATComplete      = false;
        TempIntComplete   = false;

        NextDisplayTime = 0L;   // Au départ, on bufférise tout de suite les Mesures "Option" (hors Projet)
        ModeSalvesInited = true; // Les initialisations sont terminées... Maintenant on doit recevoir une Commande Esc!Datax pour nous indiquer quelles Mesures traiter...
      }

      // Maintenant on bufférise les Mesures demandées
      // On commence par les Mesures du Projet : Luminosité, Température et Humidité.
      
      if (SendLum && (! LumComplete)) {          // La Luminosité de notre Capteur Analogique, si demandée, et si on n'a pas déjà tout.
        if (NewLuminositeDispo) {
          Salve.DataLum[NbMesLum++][TempsLum] = C_LumLux;
          NewLuminositeDispo = false;
          if (NbMesLum == Salve.FreqLum) {
            NbMesLum = 0;
            TempsLum++;
            if (TempsLum == Salve.NbSec) {
              TempsLum = 0;
              LumComplete = true;   // On a toutes les Mesures de Luminosité
            }
          }
        }
      }

      if (SendTempExt && (! TempComplete)) {          // La Température de notre Capteur Analogique, si demandée, et si on n'a pas déjà tout.
        if (NewTemperatureDispo) {
          Salve.DataTemp[NbMesTemp++][TempsTemp] = C_Temperature;
          NewTemperatureDispo = false;
          if (NbMesTemp == Salve.FreqTemp) {
            NbMesTemp = 0;
            TempsTemp++;
            if (TempsTemp == Salve.NbSec) {
              TempsTemp = 0;
              TempComplete = true;   // On a toutes les Mesures de Température
            }
          }
        }
      }

      if (SendHum && (! HumComplete)) {          // L'Humidité de notre Capteur Analogique, si demandée, et si on n'a pas déjà tout.
        if (NewHumiditeDispo) {
          Salve.DataHum[NbMesHum++][TempsHum] = C_Humidite;
          NewHumiditeDispo = false;
          if (NbMesHum == Salve.FreqHum) {
            NbMesHum = 0;
            TempsHum++;
            if (TempsHum == Salve.NbSec) {
              TempsHum = 0;
              HumComplete = true;   // On a toutes les Mesures d'Humidité
            }
          }
        }
      }

      // Ensuite, les Mesures complémentaires : Pression Atmosphérique, Tension Batterie et Température Interne (1 fois par seconde).

      if (CurTime >= NextDisplayTime) {
        
        if (SendBaro && (!BaroComplete)) {         // La Pression Barométrique (Capteur Digital sur bus I2C), si demandée, et si on n'a pas déjà tout.
            if (BarometreOK) {
              TAKE_MUTEX(Mutex_Barometre);
                if (Barometer.MeteoPressure != 0L) {
                  Salve.DataBARO[TempsBARO++] = Barometer.MeteoPressure;
                  if (TempsBARO == Salve.NbSec) {
                    TempsBARO = 0;
                    BaroComplete = true;
                  }
                }
              GIVE_MUTEX(Mutex_Barometre);
            }
        }

        if (SendVBAT && (!VBATComplete)) {         // La Tension Batterie Principale (Mesure Analogique), si demandée, et si on n'a pas déjà tout.
            if (VbatGood) {       // VbatGood est activé par la Tache VBATMonitorTask()
              Salve.DataVBAT[TempsVBAT++] = TensionBatterie;
              if (TempsVBAT == Salve.NbSec) {
                TempsVBAT = 0;
                VBATComplete = true;
              }
            }
        }

        if (SendTempInt && (!TempIntComplete)) {   // La Température Interne de la Station (Capteur Digital sur bus I2C), si demandée, et si on n'a pas déjà tout.
            if (BarometreOK) {
              TAKE_MUTEX(Mutex_Barometre);
                Salve.DataTempInt[TempsTempInt++] = Barometer.Temperature;
              GIVE_MUTEX(Mutex_Barometre);
              if (TempsTempInt == Salve.NbSec) {
                TempsTempInt = 0;
                TempIntComplete = true;
              }
            }
        }
        
        NextDisplayTime = CurTime + 1000;     // On bufférise à nouveau ces Mesures dans 1 seconde.
      }

      // Maintenant on vérifie si on a toutes les Mesures, pour les envoyer au PC (fin de la bufférisation)

      NbMesTotal = 0;
      if (SendLum) NbMesTotal++;
      if (SendTempExt)  NbMesTotal++;
      if (SendHum)      NbMesTotal++;
      if (SendBaro)     NbMesTotal++;
      if (SendVBAT)     NbMesTotal++;
      if (SendTempInt)  NbMesTotal++;

      NbMesComplete = 0;
      if (SendLum && LumComplete)           NbMesComplete++;
      if (SendTempExt && TempComplete)      NbMesComplete++;
      if (SendHum && HumComplete)           NbMesComplete++;
      if (SendBaro && BaroComplete)         NbMesComplete++;
      if (SendVBAT && VBATComplete)         NbMesComplete++;
      if (SendTempInt && TempIntComplete)   NbMesComplete++;

      if (NbMesComplete == NbMesTotal) {    // On peut envoyer au PC, fin de la Bufférisation
        for (i = 0 ; i < Salve.NbSec ; i++) {
          if (SendLum) {        // On envoie toutes les Mesures de Luminosité de la Seconde en cours
            for (j = 0 ; j < Salve.FreqLum ; j++) {
                if (SendTimeStamp) GiveTimeStamp(Salve.StartTimeStamp);
                GiveLum(Salve.DataLum[j][i]);
            }
          }

          if (SendTempExt) {    // On envoie toutes les Mesures de Température de la Seconde en cours
            for (j = 0 ; j < Salve.FreqTemp ; j++) {
                if (SendTimeStamp) GiveTimeStamp(Salve.StartTimeStamp);
                GiveTemp(Salve.DataTemp[j][i]);
            }
          }

          if (SendHum) {        // On envoie toutes les Mesures d'Humidité de la Seconde en cours
            for (j = 0 ; j < Salve.FreqHum ; j++) {
                if (SendTimeStamp) GiveTimeStamp(Salve.StartTimeStamp);
                GiveHum(Salve.DataHum[j][i]);
            }
          }

          // Maintenant, nos Mesures en Option

          if (SendBaro)     GiveBaro(Salve.DataBARO[i]);
          if (SendVBAT)     GiveVBAT(Salve.DataVBAT[i]);
          if (SendTempInt)  GiveTempInt(Salve.DataTempInt[i]);
        }

        // Maintenant on réinitialise nos variables et Flags, pour une nouvelle Bufférisation

        TempsLum          = NbMesLum    = 0;
        TempsTemp         = NbMesTemp   = 0;
        TempsHum          = NbMesHum    = 0;
        TempsVBAT         = 0;
        TempsBARO         = 0;
        TempsTempInt      = 0;

        LumComplete       = false;
        TempComplete      = false;
        HumComplete       = false;
        BaroComplete      = false;
        VBATComplete      = false;
        TempIntComplete   = false;

        Salve.StartTimeStamp  = TimeStamp();  // On met UN SEUL Horodatage pour TOUTES les Mesures bufférisées (elles auront donc toutes le même).
        NextDisplayTime = 0L;   // Au départ, on bufférise tout de suite les Mesures "Option" (hors Projet)        
      }
    }

    // *******************************************
    // *   Gestion de Mode n°3 : MODE_ONDEMAND   *
    // *******************************************

    SLEEPTASK(1);
  }
}


// **************************************************************
// * Routine de gestion des Commandes reçues sur port Série USB *
// **************************************************************

boolean My_gestion_input(PrivateDataRec *DataRec)
{
static unsigned char  Automate = 0;
unsigned char         Car;
unsigned short        NbCarRecus,i;
boolean               Changed;

  NbCarRecus = Serial.available();
  Changed = false;    // Aucun changement pour le moment...
  
  if (NbCarRecus > 0) {    // Réception de donnée(s) sur le port Série
      for (i = 0 ; i < NbCarRecus ; i++) {
        Car = Serial.read();
        switch (Automate) {
          case 0:
            if (Car == ESC) Automate++; // Début de réception de séquence "Escape"
          break;
          case 1:
            if (Car == ESCAPE_INTRO) Automate++;    // Est-ce que c'est une de nos séquences ?
            else Automate = 0;                      // Non, retour à l'état initial
          break;
          case 2:
            if (Car == 'M') Automate = 3;           // Séquence "Esc!Modex" ?
            else if (Car == 'D') Automate = 7;      // Séquence "Esc!Datax" ?
            else Automate = 0;                      // Non, retour à l'état initial
          break;
          case 3:   // Début de tétection de la Séquence "Esc!Modex"
            if (Car == 'o') Automate++;
            else Automate = 0;                      // Non, retour à l'état initial
          break;
          case 4:
            if (Car == 'd') Automate++;
            else Automate = 0;
          break;
          case 5:
            if (Car == 'e') Automate++;
            else Automate = 0;
          break;
          case 6:                                   // Décodage du Mode de Travail demandé.
            Automate = 0;
            Car -= '0';
            if ((Car < MODE_BASIC) || (Car > MODE_METEO)) Serial.write(BELL);   // Erreur : Bip. Mode non-reconnu.       
            else if (Car > MODE_EVENEMENTIEL) Serial.write(BELL);               // Erreur : Bip. Les modes "MODE_CONSOLE" et "MODE_METEO" ne sont pas encore traités...
            else {
              if (DataRec->ModeTravail |= Car) {                                // On change de mode de travail...
                DataRec->SendLum         = false;                               // On réinitialise tous les Flags d'envoi de Mesures
                DataRec->SendTempExt     = false;
                DataRec->SendHum         = false;
                DataRec->SendBaro        = false;
                DataRec->SendVBAT        = false;
                DataRec->SendTempInt     = false;
                DataRec->SendTimeStamp   = false;
              }
              DataRec->ModeTravail = Car;                                       // On enregistre le nouveau mode de travail.
              if (Car == MODE_SALVES) ModeSalvesInited = false;                 // Reset du Flag du Mode "Salves".
              Changed = true;                                                   // On signale qu'il y a eu un changement
            }
          break;
          case 7:  // Début de Détection de la Séquence "Esc!Datax"
            if (Car == 'a') Automate++;
            else Automate = 0;
          break;
          case 8:
            if (Car == 't') Automate++;
            else Automate = 0;
          break;
          case 9:
            if (Car == 'a') Automate++;
            else Automate = 0;
          break;
          case 10:                                  // Décodage des Mesures à envoyer (codées sur 7 bits, chaque bit à "1" indique qu'on doit envoyer un type de Mesure).
            DataRec->SendLum         = false;                // On commence par tout réinitialiser
            DataRec->SendTempExt     = false;
            DataRec->SendHum         = false;
            DataRec->SendBaro        = false;
            DataRec->SendVBAT        = false;
            DataRec->SendTempInt     = false;
            DataRec->SendTimeStamp   = false;
            
            Automate = 0;
            if (Car & 0b00000001) DataRec->SendLum         = true;   // Si bit 0 à "1", envoi des Mesures de Luminosité             (à une fréquence de 1 à 20Hz)
            if (Car & 0b00000010) DataRec->SendTempExt     = true;   // Si bit 1 à "1", envoi des Mesures de Température Externe    (à une fréquence de 1 à 20Hz)
            if (Car & 0b00000100) DataRec->SendHum         = true;   // Si bit 2 à "1", envoi des Mesures d'Humidité                (à une fréquence de 1 à 20Hz)
            if (Car & 0b00001000) DataRec->SendBaro        = true;   // Si bit 3 à "1", envoi des Mesures de Pression Atmosphérique (à une fréquence de 1Hz)
            if (Car & 0b00010000) DataRec->SendVBAT        = true;   // Si bit 4 à "1", envoi des Mesures de Tension Batterie       (à une fréquence de 1Hz)
            if (Car & 0b00100000) DataRec->SendTempInt     = true;   // Si bit 5 à "1", envoi des Mesures de Température Intérieure (à une fréquence de 1Hz)
            if (Car & 0b01000000) DataRec->SendTimeStamp   = true;   // Si bit 6 à "1", envoi de l'Horodatage                       (à une fréquence de 1Hz)

            Changed = true;
          break;
        }
      }
    }
  return (Changed);
}

void GiveTimeStamp(unsigned long timestamp)
{
  Serial.print("@:");
  Serial.print(timestamp);
  Serial.write(TAB);
}

void GiveLum(unsigned short Luminosite)
{
  Serial.print("L:");
  Serial.print(Luminosite);
  Serial.write("\r\n");     // Envoi de <CR><LF>
}

void GiveTemp(signed short Temp)
{
signed short   Temperature;

  Temperature = Temp;
  
  Serial.print("T:");
  Serial.print(Temperature / 10);
  Serial.print(".");
  if (Temperature < 0) Temperature = -(Temperature);
  Serial.print(Temperature % 10);
  Serial.write("\r\n");     // Envoi de <CR><LF>
}

void GiveHum(unsigned short Humidite)
{
  Serial.print("H:");
  Serial.print(Humidite);
  Serial.write("\r\n");     // Envoi de <CR><LF>
}

void GiveBaro(unsigned long HectoPascals)
{
  Serial.print("P:");
  Serial.print(HectoPascals / 10);
  Serial.print(".");
  Serial.print(HectoPascals % 10);
  Serial.write("\r\n");     // Envoi de <CR><LF>
}

void GiveVBAT(unsigned short TensionBat)
{
short     temp;

  Serial.print("V:");
  Serial.print(TensionBat / 100);
  Serial.print(".");
  temp = (TensionBat % 100);
  if (temp < 10) Serial.print("0");
  Serial.print(temp);
  Serial.write("\r\n");     // Envoi de <CR><LF>
}

void GiveTempInt(signed short Temp)
{
signed short   Temperature;

  Temperature = Temp;
  Serial.print("I:");
  Serial.print(Temperature / 10);
  Serial.print(".");
  if (Temperature < 0) Temperature = -(Temperature);
  Serial.print(Temperature % 10);
  Serial.write("\r\n");     // Envoi de <CR><LF>

}

// *******************************************************
// * Routines de calcul sur les Dates, pour l'Horodatage *
// *******************************************************


// Fonction d'Horodatage : retourne le nombre de secondes depuis le 01/01/1970 0h00mn00s (EPOCH, ou "Temps Unix")

unsigned long TimeStamp(void)
{
unsigned long   timestamp;
unsigned short  days;

  TAKE_MUTEX(Mutex_Horloge);
      days = date2days(Horloge.Annee,Horloge.Mois,Horloge.Date);
      timestamp  = time2long(days,Horloge.Heure,Horloge.Minutes,Horloge.Secondes);
  GIVE_MUTEX(Mutex_Horloge);
  
  timestamp += SECONDS_FROM_1970_TO_2000;  // On veut le timestamp "Unix", on ajoute le Nb de secondes entre 01/01/1970 et 01/01/2000

  return (timestamp);
}

#define LEAPOCH (946684800LL + 86400*(31+29))

#define DAYS_PER_400Y (365*400 + 97)
#define DAYS_PER_100Y (365*100 + 24)
#define DAYS_PER_4Y   (365*4   + 1)

// Fonction Inverse : donne la date et l'heure à partir d'un Horodatage (Nombre de secondes)

void Secs2Date(long long t,DateTimeRec *tm)
{
long long           days,secs;
int                 remdays, remsecs, remyears;
int                 qc_cycles,c_cycles,q_cycles;
int                 years,months;
int                 wday,yday,leap;
static const char   days_in_month[] = {31,30,31,30,31,31,30,31,30,31,31,29};

  // Reject time_t values whose year would overflow int
  
  if (t < INT_MIN * 31622400LL || t > INT_MAX * 31622400LL) return;

  secs = t - LEAPOCH;
  days = secs / 86400;
  remsecs = secs % 86400;
  if (remsecs < 0) {
    remsecs += 86400;
    days--;
  }

  wday = (3+days)%7;
  if (wday < 0) wday += 7;

  qc_cycles = days / DAYS_PER_400Y;
  remdays = days % DAYS_PER_400Y;
  if (remdays < 0) {
    remdays += DAYS_PER_400Y;
    qc_cycles--;
  }

  c_cycles = remdays / DAYS_PER_100Y;
  if (c_cycles == 4) c_cycles--;
  remdays -= c_cycles * DAYS_PER_100Y;

  q_cycles = remdays / DAYS_PER_4Y;
  if (q_cycles == 25) q_cycles--;
  remdays -= q_cycles * DAYS_PER_4Y;

  remyears = remdays / 365;
  if (remyears == 4) remyears--;
  remdays -= remyears * 365;

  leap = !remyears && (q_cycles || !c_cycles);
  yday = remdays + 31 + 28 + leap;
  if (yday >= 365+leap) yday -= 365+leap;

  years = remyears + 4*q_cycles + 100*c_cycles + 400*qc_cycles;

  for (months=0; days_in_month[months] <= remdays; months++)
    remdays -= days_in_month[months];

  //if (years+100 > INT_MAX || years+100 < INT_MIN) return;

  //tm->Annee     = years + 100;
  tm->Annee     = years;
  tm->Mois      = months + 3; // Was +2
  if (tm->Mois >= 12) {
    tm->Mois   -= 12;
    tm->Annee++;
  }
  tm->Date      = remdays + 1;
  tm->JourSem   = wday;
  //tm->tm_yday = yday;

  tm->Heure     = remsecs / 3600;
  tm->Minutes   = remsecs / 60 % 60;
  tm->Secondes  = remsecs % 60;
}

// Nombre de jours depuis le 01/01/2000, valide pour 2001..2099

unsigned short date2days(unsigned short y, unsigned char m, unsigned char d)
{
unsigned short    days;
unsigned char     i;

  if (y >= 2000) y -= 2000;
  
  days = d;
  for (i = 1; i < m; ++i) days += daysInMonth[i - 1];
  if (m > 2 && y % 4 == 0) ++days;
  return (days + 365 * y + (y + 3) / 4 - 1);
}

unsigned long time2long(unsigned short days, unsigned char h, unsigned char m, unsigned char s)
{
    return ((days * 24L + h) * 60 + m) * 60 + s;
}


