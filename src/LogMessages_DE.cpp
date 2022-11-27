
#include "settings.h"

#if (LANGUAGE == DE)
    #include "Log.h"

    const char tryConnectMqttS[] PROGMEM = "Versuche Verbindung zu MQTT-Broker aufzubauen";
    const char mqttOk[] PROGMEM = "MQTT-Session aufgebaut.";
    const char sleepTimerEOP[] PROGMEM = "Sleep-Timer: Nach dem letzten Track der Playlist.";
    const char sleepTimerEOT[] PROGMEM = "Sleep-Timer: Nach dem Ende des laufenden Tracks.";
    const char sleepTimerStop[] PROGMEM = "Sleep-Timer wurde deaktiviert.";
    const char sleepTimerEO5[] PROGMEM = "Sleep Timer: Nach Ende des Titels oder, wenn früher, Ende der Playlist";
    const char sleepTimerAlreadyStopped[] PROGMEM = "Sleep-Timer ist bereits deaktiviert.";
    const char sleepTimerSetTo[] PROGMEM = "Sleep-Timer gesetzt auf";
    const char allowButtons[] PROGMEM = "Alle Tasten werden freigegeben.";
    const char lockButtons[] PROGMEM = "Alle Tasten werden gesperrt.";
    const char noPlaylistNotAllowedMqtt[] PROGMEM = "Playmode kann nicht auf 'Keine Playlist' gesetzt werden via MQTT.";
    const char playmodeChangedMQtt[] PROGMEM = "Playmode per MQTT angepasst.";
    const char noPlaymodeChangeIfIdle[] PROGMEM = "Playmode kann nicht verändert werden, wenn keine Playlist aktiv ist.";
    const char noValidTopic[] PROGMEM = "Kein gültiges Topic";
    const char freePtr[] PROGMEM = "Ptr-Freigabe";
    const char freeMemory[] PROGMEM = "Freier Speicher";
    const char writeEntryToNvs[] PROGMEM = "Schreibe Eintrag in NVS";
    const char freeMemoryAfterFree[] PROGMEM = "Freier Speicher nach Aufräumen";
    const char releaseMemoryOfOldPlaylist[] PROGMEM = "Gebe Speicher der alten Playlist frei.";
    const char dirOrFileDoesNotExist[] PROGMEM = "Datei oder Verzeichnis existiert nicht ";
    const char unableToAllocateMemForPlaylist[] PROGMEM = "Speicher für Playlist konnte nicht allokiert werden!";
    const char unableToAllocateMem[] PROGMEM = "Speicher konnte nicht allokiert werden!";
    const char fileModeDetected[] PROGMEM = "Dateimodus erkannt.";
    const char nameOfFileFound[] PROGMEM = "Gefundenes File";
    const char reallocCalled[] PROGMEM = "Speicher reallokiert.";
    const char unableToAllocateMemForLinearPlaylist[] PROGMEM = "Speicher für lineare Playlist konnte nicht allokiert werden!";
    const char numberOfValidFiles[] PROGMEM = "Anzahl gültiger Files/Webstreams";
    const char newLoudnessReceivedQueue[] PROGMEM = "Neue Lautstärke empfangen via Queue";
    const char newCntrlReceivedQueue[] PROGMEM = "Kontroll-Kommando empfangen via Queue";
    const char newPlaylistReceived[] PROGMEM = "Neue Playlist empfangen";
    const char repeatTrackDueToPlaymode[] PROGMEM = "Wiederhole Titel aufgrund von Playmode.";
    const char repeatPlaylistDueToPlaymode[] PROGMEM = "Wiederhole Playlist aufgrund von Playmode.";
    const char cmndStop[] PROGMEM = "Kommando: Stop";
    const char cmndPause[] PROGMEM = "Kommando: Pause";
    const char cmndNextTrack[] PROGMEM = "Kommando: Nächster Titel";
    const char cmndPrevTrack[] PROGMEM = "Kommando: Vorheriger Titel";
    const char cmndFirstTrack[] PROGMEM = "Kommando: Erster Titel von Playlist";
    const char cmndLastTrack[] PROGMEM = "Kommando: Letzter Titel von Playlist";
    const char cmndDoesNotExist[] PROGMEM = "Dieses Kommando existiert nicht.";
    const char lastTrackAlreadyActive[] PROGMEM = "Es wird bereits der letzte Track gespielt.";
    const char trackStartAudiobook[] PROGMEM = "Titel wird im Hörspielmodus von vorne gespielt.";
    const char trackStart[] PROGMEM = "Titel wird von vorne gespielt.";
    const char trackChangeWebstream[] PROGMEM = "Im Webradio-Modus kann nicht an den Anfang gesprungen werden.";
    const char endOfPlaylistReached[] PROGMEM = "Ende der Playlist erreicht.";
    const char trackStartatPos[] PROGMEM = "Titel wird abgespielt ab Position";
    const char waitingForTaskQueues[] PROGMEM = "Task Queue für RFID existiert noch nicht, warte...";
    const char rfidScannerReady[] PROGMEM = "RFID-Tags koennen jetzt gescannt werden...";
    const char rfidTagDetected[] PROGMEM = "RFID-Karte erkannt: ";
    const char rfid15693TagDetected[] PROGMEM = "RFID-Karte (ISO-15693) erkannt: ";
    const char rfidTagReceived[] PROGMEM = "RFID-Karte empfangen";
    const char dontAccepctSameRfid[] PROGMEM = "Aktuelle RFID-Karte erneut aufgelegt - abgelehnt!";
    const char rfidTagUnknownInNvs[] PROGMEM = "RFID-Karte ist im NVS nicht hinterlegt.";
    const char mfrc522HaltFailed[] PROGMEM = "RFID-Leser: Halt Befehl ist fehlgeschlagen";
    const char goToSleepDueToIdle[] PROGMEM = "Gehe in Deep Sleep wegen Inaktivität...";
    const char goToSleepDueToTimer[] PROGMEM = "Gehe in Deep Sleep wegen Sleep Timer...";
    const char goToSleepNow[] PROGMEM = "Gehe jetzt in Deep Sleep!";
    const char maxLoudnessReached[] PROGMEM = "Maximale Lautstärke bereits erreicht!";
    const char minLoudnessReached[] PROGMEM = "Minimale Lautstärke bereits erreicht!";
    const char errorOccured[] PROGMEM = "Fehler aufgetreten!";
    const char noMp3FilesInDir[] PROGMEM = "Verzeichnis beinhaltet keine mp3-Files.";
    const char modeSingleTrack[] PROGMEM = "Modus: Einzelner Track";
    const char modeSingleTrackLoop[] PROGMEM = "Modus: Einzelner Track in Endlosschleife";
    const char modeSingleTrackRandom[] PROGMEM = "Modus: Einzelner Track eines Ordners zufällig";
    const char modeSingleAudiobook[] PROGMEM = "Modus: Hoerspiel";
    const char modeSingleAudiobookLoop[] PROGMEM = "Modus: Hoerspiel in Endlosschleife";
    const char modeAllTrackAlphSorted[] PROGMEM = "Modus: Spiele alle Tracks (alphabetisch sortiert) des Ordners";
    const char modeAllTrackRandom[] PROGMEM = "Modus: Alle Tracks eines Ordners zufällig";
    const char modeAllTrackAlphSortedLoop[] PROGMEM = "Modus: Alle Tracks eines Ordners sortiert (alphabetisch) in Endlosschleife";
    const char modeAllTrackRandomLoop[] PROGMEM = "Modus: Alle Tracks eines Ordners zufällig in Endlosschleife";
    const char modeWebstream[] PROGMEM = "Modus: Webstream";
    const char modeWebstreamM3u[] PROGMEM = "Modus: Webstream (lokale .m3u-Datei)";
    const char webstreamNotAvailable[] PROGMEM = "Aktuell kein Webstream möglich, da keine WLAN-Verbindung vorhanden!";
    const char modeDoesNotExist[] PROGMEM = "Abspielmodus existiert nicht!";
    const char modeRepeatNone[] PROGMEM = "Repeatmodus: Kein Repeat";
    const char modeRepeatTrack[] PROGMEM = "Repeatmodus: Aktueller Titel";
    const char modeRepeatPlaylist[] PROGMEM = "Repeatmodus: Gesamte Playlist";
    const char modeRepeatTracknPlaylist[] PROGMEM = "Repeatmodus: Track und Playlist";
    const char modificatorAllButtonsLocked[] PROGMEM = "Modifikator: Alle Tasten werden per RFID gesperrt.";
    const char modificatorAllButtonsUnlocked[] PROGMEM = "Modifikator: Alle Tasten werden per RFID freigegeben.";
    const char modificatorSleepd[] PROGMEM = "Modifikator: Sleep-Timer wieder deaktiviert.";
    const char modificatorSleepTimer15[] PROGMEM = "Modifikator: Sleep-Timer per RFID aktiviert (15 Minuten).";
    const char modificatorSleepTimer30[] PROGMEM = "Modifikator: Sleep-Timer per RFID aktiviert (30 Minuten).";
    const char modificatorSleepTimer60[] PROGMEM = "Modifikator: Sleep-Timer per RFID aktiviert (60 Minuten).";
    const char modificatorSleepTimer120[] PROGMEM = "Modifikator: Sleep-Timer per RFID aktiviert (2 Stunden).";
    const char ledsDimmedToNightmode[] PROGMEM = "LEDs wurden auf Nachtmodus gedimmt.";
    const char ledsDimmedToInitialValue[] PROGMEM = "LEDs wurden auf initiale Helligkeit gedimmt.";
    const char modificatorNotallowedWhenIdle[] PROGMEM = "Modifikator kann bei nicht aktivierter Playlist nicht angewendet werden.";
    const char modificatorSleepAtEOT[] PROGMEM = "Modifikator: Sleep-Timer am Ende des Titels aktiviert.";
    const char modificatorSleepAtEOTd[] PROGMEM = "Modifikator: Sleep-Timer am Ende des Titels deaktiviert.";
    const char modificatorSleepAtEOP[] PROGMEM = "Modifikator: Sleep-Timer am Ende der Playlist aktiviert.";
    const char modificatorSleepAtEOPd[] PROGMEM = "Modifikator: Sleep-Timer am Ende der Playlist deaktiviert.";
    const char modificatorAllTrackAlphSortedLoop[] PROGMEM = "Modifikator: Alle Titel (alphabetisch sortiert) in Endlosschleife.";
    const char modificatorAllTrackRandomLoop[] PROGMEM = "Modifikator: Alle Titel (zufällige Reihenfolge) in Endlosschleife.";
    const char modificatorCurTrackLoop[] PROGMEM = "Modifikator: Aktueller Titel in Endlosschleife.";
    const char modificatorCurAudiobookLoop[] PROGMEM = "Modifikator: Aktuelles Hörspiel in Endlosschleife.";
    const char modificatorPlaylistLoopActive[] PROGMEM = "Modifikator: Alle Titel in Endlosschleife aktiviert.";
    const char modificatorPlaylistLoopDeactive[] PROGMEM = "Modifikator: Alle Titel in Endlosschleife deaktiviert.";
    const char modificatorTrackActive[] PROGMEM = "Modifikator: Titel in Endlosschleife aktiviert.";
    const char modificatorTrackDeactive[] PROGMEM = "Modifikator: Titel in Endlosschleife deaktiviert.";
    const char modificatorNotAllowed[] PROGMEM = "Modifikator konnte nicht angewendet werden.";
    const char modificatorLoopRev[] PROGMEM = "Modifikator: Endlosschleife beendet.";
    const char modificatorDoesNotExist[] PROGMEM = "Ein Karten-Modifikator existiert nicht vom Typ";
    const char errorOccuredNvs[] PROGMEM = "Es ist ein Fehler aufgetreten beim Lesen aus dem NVS!";
    const char statementsReceivedByServer[] PROGMEM = "Vom Server wurde Folgendes empfangen";
    const char savedSsidInNvs[] PROGMEM = "Speichere SSID in NVS";
    const char savedWifiPwdInNvs[] PROGMEM = "Speichere WLAN-Password in NVS";
    const char apReady[] PROGMEM = "Access-Point geöffnet";
    const char httpReady[] PROGMEM = "HTTP-Server gestartet.";
    const char unableToMountSd[] PROGMEM = "SD-Karte konnte nicht gemountet werden.";
    const char unableToCreateVolQ[] PROGMEM = "Konnte Volume-Queue nicht anlegen.";
    const char unableToCreateRfidQ[] PROGMEM = "Konnte RFID-Queue nicht anlegen.";
    const char unableToCreateMgmtQ[] PROGMEM = "Konnte Play-Management-Queue nicht anlegen.";
    const char unableToCreatePlayQ[] PROGMEM = "Konnte Track-Queue nicht anlegen..";
    const char initialBrightnessfromNvs[] PROGMEM = "Initiale LED-Helligkeit wurde aus NVS geladen";
    const char wroteInitialBrightnessToNvs[] PROGMEM = "Initiale LED-Helligkeit wurde ins NVS geschrieben.";
    const char restoredInitialBrightnessForNmFromNvs[] PROGMEM = "LED-Helligkeit für Nachtmodus wurde aus NVS geladen";
    const char wroteNmBrightnessToNvs[] PROGMEM = "LED-Helligkeit für Nachtmodus wurde ins NVS geschrieben.";
    const char wroteFtpUserToNvs[] PROGMEM = "FTP-User wurde ins NVS geschrieben.";
    const char restoredFtpUserFromNvs[] PROGMEM = "FTP-User wurde aus NVS geladen";
    const char wroteFtpPwdToNvs[] PROGMEM = "FTP-Passwort wurde ins NVS geschrieben.";
    const char restoredFtpPwdFromNvs[] PROGMEM = "FTP-Passwort wurde aus NVS geladen";
    const char restoredMaxInactivityFromNvs[] PROGMEM = "Maximale Inaktivitätszeit wurde aus NVS geladen";
    const char wroteMaxInactivityToNvs[] PROGMEM = "Maximale Inaktivitätszeit wurde ins NVS geschrieben.";
    const char restoredInitialLoudnessFromNvs[] PROGMEM = "Initiale Lautstärke wurde aus NVS geladen";
    const char wroteInitialLoudnessToNvs[] PROGMEM = "Initiale Lautstärke wurde ins NVS geschrieben.";
    const char restoredMaxLoudnessForSpeakerFromNvs[] PROGMEM = "Maximale Lautstärke für Lautsprecher wurde aus NVS geladen";
    const char restoredMaxLoudnessForHeadphoneFromNvs[] PROGMEM = "Maximale Lautstärke für Kopfhörer wurde aus NVS geladen";
    const char wroteMaxLoudnessForSpeakerToNvs[] PROGMEM = "Maximale Lautstärke für Lautsprecher wurde ins NVS geschrieben.";
    const char wroteMaxLoudnessForHeadphoneToNvs[] PROGMEM = "Maximale Lautstärke für Kopfhörer wurde ins NVS geschrieben.";
    const char maxVolumeSet[] PROGMEM = "Maximale Lautstärke wurde gesetzt auf";
    const char wroteMqttFlagToNvs[] PROGMEM = "MQTT-Flag wurde ins NVS geschrieben.";
    const char restoredMqttActiveFromNvs[] PROGMEM = "MQTT-Flag (aktiviert) wurde aus NVS geladen";
    const char restoredMqttDeactiveFromNvs[] PROGMEM = "MQTT-Flag (deaktiviert) wurde aus NVS geladen";
    const char wroteMqttClientIdToNvs[] PROGMEM = "MQTT-ClientId wurde ins NVS geschrieben.";
    const char restoredMqttClientIdFromNvs[] PROGMEM = "MQTT-ClientId wurde aus NVS geladen";
    const char wroteMqttServerToNvs[] PROGMEM = "MQTT-Server wurde ins NVS geschrieben.";
    const char restoredMqttServerFromNvs[] PROGMEM = "MQTT-Server wurde aus NVS geladen";
    const char wroteMqttUserToNvs[] PROGMEM = "MQTT-User wurde ins NVS geschrieben.";
    const char restoredMqttUserFromNvs[] PROGMEM = "MQTT-User wurde aus NVS geladen";
    const char wroteMqttPwdToNvs[] PROGMEM = "MQTT-Passwort wurde ins NVS geschrieben.";
    const char restoredMqttPwdFromNvs[] PROGMEM = "MQTT-Passwort wurde aus NVS geladen";
    const char restoredMqttPortFromNvs[] PROGMEM = "MQTT-Port wurde aus NVS geladen";
    const char mqttWithPwd[] PROGMEM = "Verbinde zu MQTT-Server mit User und Passwort";
    const char mqttWithoutPwd[] PROGMEM = "Verbinde zu MQTT-Server ohne User und Passwort";
    const char ssidNotFoundInNvs[] PROGMEM = "SSID wurde im NVS nicht gefunden.";
    const char wifiPwdNotFoundInNvs[] PROGMEM = "WLAN-Passwort wurde im NVS nicht gefunden.";
    const char wifiStaticIpConfigNotFoundInNvs[] PROGMEM = "Statische WLAN-IP-Konfiguration wurde im NVS nicht gefunden.";
    const char wifiHostnameNotSet[] PROGMEM = "Keine Hostname-Konfiguration im NVS gefunden.";
    const char mqttConnFailed[] PROGMEM = "Verbindung fehlgeschlagen, versuche in Kürze erneut";
    const char restoredHostnameFromNvs[] PROGMEM = "Hostname aus NVS geladen";
    const char currentVoltageMsg[] PROGMEM = "Aktuelle Batteriespannung";
    const char currentChargeMsg[] PROGMEM = "Aktuelle Batterieladung";
    const char batteryCurrentMsg[] PROGMEM = "Stromverbrauch (Batterie)";
    const char batteryTempMsg[] PROGMEM = "Temperatur der Batterie";
    const char batteryCyclesMsg[] PROGMEM = "Gesehene Batteriezyklen";
    const char batteryLowMsg[] PROGMEM = "Batterieladung niedrig";
    const char batteryCriticalMsg[] PROGMEM = "Batterieladung kritisch. Gehe in Deepsleep...";
    const char sdBootFailedDeepsleep[] PROGMEM = "Bootgang wegen SD fehlgeschlagen. Gehe in Deepsleep...";
    const char wifiEnabledAfterRestart[] PROGMEM = "WLAN wird aktiviert.";
    const char wifiDisabledAfterRestart[] PROGMEM = "WLAN wird deaktiviert.";
    const char voltageIndicatorLowFromNVS[] PROGMEM = "Unterer Spannungslevel (Batterie) fuer Neopixel-Anzeige aus NVS geladen";
    const char voltageIndicatorHighFromNVS[] PROGMEM = "Oberer Spannungslevel (Batterie) fuer Neopixel-Anzeige aus NVS geladen";
    const char batteryCheckIntervalFromNVS[] PROGMEM = "Zyklus für Batteriemessung fuer Neopixel-Anzeige aus NVS geladen";
    const char warningLowVoltageFromNVS[] PROGMEM = "Spannungslevel (Batterie) fuer Niedrig-Warnung via Neopixel aus NVS geladen";
    const char warningCriticalVoltageFromNVS[] PROGMEM = "Spannungslevel (Batterie) fuer Kritisch-Warnung via Neopixel aus NVS geladen";
    const char batteryLowFromNVS[] PROGMEM = "Batterieladestand fuer Niedrig-Warnung via Neopixel aus NVS geladen";
    const char batteryCriticalFromNVS[] PROGMEM = "Batterieladestand fuer Kritisch-Warnung via Neopixel aus NVS geladen";
    const char unableToRestoreLastRfidFromNVS[] PROGMEM = "Letzte RFID konnte nicht aus NVS geladen werden";
    const char restoredLastRfidFromNVS[] PROGMEM = "Letzte RFID wurde aus NVS geladen";
    const char failedOpenFileForWrite[] PROGMEM = "Öffnen der Datei für den Schreibvorgang fehlgeschlagen";
    const char fileWritten[] PROGMEM = "Datei geschrieben";
    const char writeFailed[] PROGMEM = "Schreibvorgang fehlgeschlagen";
    const char writingFile[] PROGMEM = "Schreibe Datei";
    const char failedToOpenFileForAppending[] PROGMEM = "Öffnen der Datei zum Schreiben der JSON-Datei fehlgeschlagen";
    const char listingDirectory[] PROGMEM = "Verzeichnisinhalt anzeigen";
    const char failedToOpenDirectory[] PROGMEM = "Öffnen des Verzeichnisses fehlgeschlagen";
    const char notADirectory[] PROGMEM = "Kein Verzeichnis";
    const char sdMountedMmc1BitMode[] PROGMEM = "Versuche SD-Karte wird im SD_MMC-Modus (1 Bit) zu mounten...";
    const char sdMountedSpiMode[] PROGMEM = "Versuche SD-Karte wird im SPI-Modus zu mounten...";
    const char backupRecoveryWebsite[] PROGMEM = "<p>Das Backup-File wird eingespielt...<br />Zur letzten Seite <a href=\"javascript:history.back()\">zur&uuml;ckkehren</a>.</p>";
    const char restartWebsite[] PROGMEM = "<p>Der ESPuino wird neu gestartet...<br />Zur letzten Seite <a href=\"javascript:history.back()\">zur&uuml;ckkehren</a>.</p>";
    const char shutdownWebsite[] PROGMEM = "<p>Der ESPuino wird ausgeschaltet...</p>";
    const char mqttMsgReceived[] PROGMEM = "MQTT-Nachricht empfangen";
    const char trackPausedAtPos[] PROGMEM = "Titel pausiert bei Position";
    const char freeHeapWithoutFtp[] PROGMEM = "Freier Heap-Speicher vor FTP-Instanzierung";
    const char freeHeapWithFtp[] PROGMEM = "Freier Heap-Speicher nach FTP-Instanzierung";
    const char freeHeapAfterSetup[] PROGMEM = "Freier Heap-Speicher nach Setup-Routine";
    const char tryStaticIpConfig[] PROGMEM = "Statische IP-Konfiguration wird durchgeführt...";
    const char staticIPConfigFailed[] PROGMEM = "Statische IP-Konfiguration fehlgeschlagen";
    const char wakeUpRfidNoIso14443[] PROGMEM = "ESP32 wurde vom Kartenleser aus dem Deepsleep aufgeweckt. Allerdings wurde keine ISO-14443-Karte gefunden. Gehe zurück in den Deepsleep...";
    const char lowPowerCardSuccess[] PROGMEM = "Kartenerkennung via 'low power' erfolgreich durchgeführt";
    const char rememberLastVolume[] PROGMEM = "Lautstärke vor dem letzten Shutdown wird wiederhergestellt. Dies überschreibt die Einstellung der initialen Lautstärke aus der GUI.";
    const char unableToStartFtpServer[] PROGMEM = "Der FTP-Server konnte nicht gestartet werden. Entweder weil er ist bereits gestartet oder kein WLAN verfügbar ist.";
    const char unableToTellIpAddress[] PROGMEM = "IP-Adresse kann nicht angesagt werden, da keine WLAN-Verbindung besteht.";
    const char newPlayModeStereo[] PROGMEM = "Neuer Modus: stereo";
    const char newPlayModeMono[] PROGMEM = "Neuer Modus: mono";
    const char portExpanderFound[] PROGMEM = "Port-expander gefunden";
    const char portExpanderNotFound[] PROGMEM = "Port-expander nicht gefunden";
    const char portExpanderInterruptEnabled[] PROGMEM = "Interrupt für Port-Expander aktiviert";
    const char warningRefactoring[] PROGMEM = "!!!!WICHTIG!!!! Beachte bitte https://forum.espuino.de/t/wechsel-zum-refactoring-branch-was-ist-zu-beachten/510 !!!!WICHTIG!!!!";
    const char playlistGenModeUncached[] PROGMEM = "Playlist-Generierung: uncached";
    const char playlistGenModeCached[] PROGMEM = "Playlist-Generierung: cached";
    const char playlistCacheFoundBut0[] PROGMEM = "Playlist-Cache-File gefunden, jedoch 0 Bytes groß";
    const char bootLoopDetected[] PROGMEM = "Bootschleife erkannt! Letzte RFID wird nicht aufgerufen.";
    const char noBootLoopDetected[] PROGMEM = "Keine Bootschleife erkannt. Wunderbar :-)";
    const char importCountNokNvs[] PROGMEM = "Anzahl der ungültigen Import-Einträge";
    const char errorReadingTmpfile[] PROGMEM = "Beim Lesen der temporären Importdatei ist ein Fehler aufgetreten!";
    const char errorWritingTmpfile[] PROGMEM = "Beim Schreiben der temporären Importdatei ist ein Fehler aufgetreten!";
    const char eraseRfidNvsWeb[] PROGMEM = "<p>Die NVS-RFID-Zuweisungen werden gel&ouml;scht...<br />Zur letzten Seite <a href=\"javascript:history.back()\">zur&uuml;ckkehren</a>.</p>";
    const char eraseRfidNvs[] PROGMEM = "NVS-RFID-Zuweisungen werden gelöscht...";
    const char erasePlaylistCachefile[] PROGMEM = "Playlist-Cachefile gelöscht";
    const char fwStart[] PROGMEM = "Starte Firmware-update via OTA...";
    const char fwEnd[] PROGMEM = "Firmware-update beendet";
    const char otaNotSupported[] PROGMEM = "Firmware-update wird von diesem ESPuino nicht unterstuetzt!";
    const char otaNotSupportedWebsite[] PROGMEM = "<p>Firmware-update wird von diesem ESPuino nicht unterstuetzt!<br />Zur letzten Seite <a href=\"javascript:history.back()\">zur&uuml;ckkehren</a>.</p>";
    const char noPlaylist[] PROGMEM = "Keine Playlist aktiv.";
    const char rfidTagRemoved[] PROGMEM = "RFID-Karte wurde entfernt";
    const char rfidTagReapplied[] PROGMEM = "RFID-Karte erneut aufgelegt";
    const char ftpEnableTooLate[] PROGMEM = "FTP kann nur innerhalb der ersten 30s aktiviert werden. Kinderschutz :-)";
    const char syncingViaNtp[] PROGMEM = "Synchronisiere Uhrzeit via NTP...";
    const char sdInfo[] PROGMEM = "SD-Kartengröße / freier Speicherplatz";
    const char paOn[] PROGMEM = "Lautsprecher eingeschaltet";
    const char paOff[] PROGMEM = "Lautsprecher ausgeschaltet";
    const char hpOn[] PROGMEM = "Kopfhörer eingeschaltet";
    const char hpOff[] PROGMEM = "Kopfhörer ausgeschaltet";
    const char webTxCanceled[] PROGMEM = "Der Webtransfer wurde aufgrund von Inaktivität beendet.";
    const char cantConnectToWifi[] PROGMEM = "Verbindung zum WLAN nicht möglich. Nächster Versuch...";
    const char tryToPickRandomDir[] PROGMEM = "Versuche ein zufälliges Unterzeichnis zu finden aus";
    const char pickedRandomDir[] PROGMEM = "Zufällig ausgewähltes Unterverzeichnis";
    const char wrongWakeUpGpio[] PROGMEM = "Der gewählte GPIO ist nicht vom Typ RTC und unterstützt daher das Aufwecken des ESP32 nicht!";
#endif
