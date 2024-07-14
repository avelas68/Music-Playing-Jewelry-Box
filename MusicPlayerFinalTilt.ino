#include "SoftwareSerial.h"
#include "DFRobotDFPlayerMini.h"

// Pin definitions for tilt sensor
const int TILT_SENSOR_PIN = 4;

// Pin for the potentiometer
#define LED_PIN 11
#define POTENTIOMETER_PIN A1

// Use pins 2 and 3 to communicate with DFPlayer Mini
static const uint8_t PIN_MP3_TX = 2; // Connects to module's RX
static const uint8_t PIN_MP3_RX = 3; // Connects to module's TX
SoftwareSerial softwareSerial(PIN_MP3_RX, PIN_MP3_TX);

// Create the Player object
DFRobotDFPlayerMini player;

// Variable to track whether the song is playing or not
bool isSongPlaying = false;

// Variable to store the current song index
int currentSongIndex = 1;

// Variables for managing song timing
unsigned long previousSongTime = 0;
const unsigned long songInterval = 270000; // 4.5 minutes between songs

void setup() {
  // Init USB serial port for debugging
  Serial.begin(9600); // Init serial port for DFPlayer Mini
  softwareSerial.begin(9600); // Start communication with DFPlayer Mini

  if (player.begin(softwareSerial)) {
    Serial.println("OK");
    // Set volume to maximum (0 to 30).
    player.volume(10);
  } else {
    Serial.println("Connecting to DFPlayer Mini failed!");
  }

  pinMode(LED_PIN, OUTPUT);
}

void loop() {
  // Potentiometer
  int potentiometerValue = analogRead(POTENTIOMETER_PIN);
  int brightness = potentiometerValue / 4;
  analogWrite(LED_PIN, brightness);

  // Read the state of the tilt sensor
  int tiltState = digitalRead(TILT_SENSOR_PIN);
  Serial.println(tiltState);

  // Handle song timing
  handleSongTiming();

  // Check if the tilt sensor is HIGH (tilted)
  if (tiltState == HIGH) {
    // If the song is not playing, start playing
    if (!isSongPlaying) {
      playNextSong();
    }
  } else {
    // If the song is playing and the tilt sensor is LOW, stop playing immediately
    if (isSongPlaying) {
      player.stop();
      isSongPlaying = false;
    }
  }
}

// Function to play the next song in the sequence
void playNextSong() {
  // Play the next song
  player.playFolder(1, currentSongIndex);
  isSongPlaying = true;
  previousSongTime = millis();
  currentSongIndex++;

  // Reset song index after playing all songs
  if (currentSongIndex > 3) {
    currentSongIndex = 1;
  }
}

// Function to handle song timing and play the next song when the interval is reached
void handleSongTiming() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousSongTime >= songInterval) {
    playNextSong();
  }
}
