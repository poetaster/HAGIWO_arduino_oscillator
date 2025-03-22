/*   This program is based on the Mozzi examples WavePacket_Double and FMsynth
     Mark Washeim <blueprint@poetaster.de) 2020, public domain.

     Tim Barrass 2013, CC by-nc-sa.
*/

//#include <ADC.h>  // Teensy 3.0/3.1 uncomment this line and install http://github.com/pedvide/ADC
#include <MozziConfigValues.h>
#define MOZZI_AUDIO_MODE MOZZI_OUTPUT_2PIN_PWM
#include <Mozzi.h>
#include <mozzi_analog.h>
#include <WavePacket.h>
// for FMsynth
#include <Oscil.h>
#include <tables/cos2048_int8.h> // table for Oscils to play
#include <mozzi_midi.h>
#include <mozzi_rand.h>
#include <mozzi_fixmath.h>
#include <EventDelay.h>
#include <Smooth.h>
#include <StateVariable.h>


#define MOZZI_AUDIO_RATE 32768
//#define MOZZI_PWM_RATE 32768

bool debug = true;

// analog freq pins
#define FUNDAMENTAL_PIN A0
#define BANDWIDTH_PIN A1
#define CENTREFREQ_PIN A3

// modulation pins analog
#define VOCT A7
#define P1CV A4
#define P2CV A5
#define FLT_PIN 5
#define SW_PIN_1 4
#define SW_PIN_2 5
#define LED_1_PIN 6
#define LED_2_PIN 7
#define LED_3_PIN 8
#define GAIN_CV_PIN A6
#define MODE_CV_PIN A2
// Map Analogue channels
#define CONTROL_RATE 512 // powers of 2 please


// wavetable for oscillator:
#include <tables/sin2048_int8.h>

StateVariable <LOWPASS> svf; // can be LOWPASS, BANDPASS, HIGHPASS or NOTCH


// for fake midi
//EventDelay startNote;
//EventDelay endNote;

WavePacket <DOUBLE>wavey; // <DOUBLE> wavey; // DOUBLE selects 2 overlapping streams

//ADSR <CONTROL_RATE, AUDIO_RATE> envelope;
//unsigned int duration, attack, decay, sustain, release_ms;
int lFreq = 10;

// chordsynth
// harmonics
Oscil<COS8192_NUM_CELLS, MOZZI_AUDIO_RATE> aCos1(COS8192_DATA);
Oscil<COS8192_NUM_CELLS, MOZZI_AUDIO_RATE> aCos2(COS8192_DATA);
Oscil<COS8192_NUM_CELLS, MOZZI_AUDIO_RATE> aCos3(COS8192_DATA);
Oscil<COS8192_NUM_CELLS, MOZZI_AUDIO_RATE> aCos4(COS8192_DATA);


// duplicates but slightly off frequency for adding to originals
Oscil<COS8192_NUM_CELLS, MOZZI_AUDIO_RATE> aCos1b(COS8192_DATA);
Oscil<COS8192_NUM_CELLS, MOZZI_AUDIO_RATE> aCos2b(COS8192_DATA);
Oscil<COS8192_NUM_CELLS, MOZZI_AUDIO_RATE> aCos3b(COS8192_DATA);
Oscil<COS8192_NUM_CELLS, MOZZI_AUDIO_RATE> aCos4b(COS8192_DATA);
// base pitch frequencies in Q16n16 fixed int format (for speed later)
int f1, f2, f3, f4, f5, f6, f7;
int variation()  // changing the return type here enables to easily
// increase or decrease the variation
{
  return  random(16);//mRaw(xorshift96() & 524287UL);
}

// for FMsynth

Oscil<COS2048_NUM_CELLS, MOZZI_AUDIO_RATE> aCarrier(COS2048_DATA);
Oscil<COS2048_NUM_CELLS, MOZZI_AUDIO_RATE> aModulator(COS2048_DATA);
Oscil<COS2048_NUM_CELLS, MOZZI_AUDIO_RATE> aModDepth(COS2048_DATA);
Oscil<COS2048_NUM_CELLS, MOZZI_CONTROL_RATE> kModIndex(COS2048_DATA);
Oscil<SIN2048_NUM_CELLS, MOZZI_AUDIO_RATE> kLfo(SIN2048_DATA);

Q8n8 mod_index;// = float_to_Q8n8(2.0f); // constant version
Q16n16 deviation;
Q16n16 carrier_freq, mod_freq, fundemental, bandwidth, centre;

// FM ratio between oscillator frequencies, stays the same through note range
Q8n8 mod_to_carrier_ratio = float_to_Q8n8(3.f);

EventDelay kNoteChangeDelay;
int gain;

// for note changes
Q7n8 last_note, note_upper_limit, note_lower_limit, note_change_step, smoothed_note;

// Inefficient, but...until there is a better Smooth....
Smooth <int> kSmoothNote(0.95f);


/*
  This sets the MIDI note that corresponds to 0 volts. Typically, this is
  either C0 (MIDI note 12) or C1 (MIDI note 24).
*/
static const uint32_t midi_note_at_zero_volts = 12;
static const float semitones_per_octave = 12.0f;
static const float volts_per_semitone = 1.0f / semitones_per_octave;
static const float a4_frequency = 440.0f;
static const uint32_t a4_midi_note = 69;

/*
  Converts Volts/octave to frequency.

  Just like when converting MIDI notes to frequency, there is an
  exponential relationship between Volts/octave and note frequency.
  Helpfully, the same formula used there can be used here.
*/

float volts_to_frequency(float volts) {
  float semitones = volts * semitones_per_octave;
  float adjusted_semitones = semitones + midi_note_at_zero_volts;
  float semitones_away_from_a4 = adjusted_semitones - (float)(a4_midi_note);
  return powf(2.0f, semitones_away_from_a4 / semitones_per_octave) * a4_frequency;
}
/*
  Converts Volts/octave to its corresponding MIDI note number.

  Similar to the frequncy to MIDI note number conversion, this
  only returns the nearest MIDI note, so any remainder will be
  discarded.
*/
uint32_t volts_to_midi_note(float volts) {
  return ceil(midi_note_at_zero_volts + ceil(volts * semitones_per_octave));
}
/*
   returns voltage for sensor reading with analog read
   resolution is usually 1023.0 but could be 12 bits.
*/
float sensor_to_midi(float sensorValue) {
  float volts = sensorValue * (5.0 / 1023.0);
  return ceil(midi_note_at_zero_volts + ceil(volts * semitones_per_octave));

}

int gain_val = 0;
uint8_t mode = 0;
uint16_t mode_val = 0;


void setup() {
  if (debug) {
    Serial.begin(57600);
    Serial.println("hi there");
  }
  // initialize the pushbutton pin as an input:
  pinMode(13, OUTPUT);
  pinMode(LED_1_PIN, OUTPUT);
  pinMode(LED_2_PIN, OUTPUT);
  pinMode(LED_3_PIN, OUTPUT);
  pinMode(SW_PIN_1, INPUT_PULLUP);
  pinMode(SW_PIN_2, INPUT_PULLUP);
  digitalWrite(SW_PIN_1, HIGH);
  digitalWrite(SW_PIN_2, HIGH);

  // chordsynth
  // select base frequencies using mtof (midi to freq) and fixed-point numbers
  f1 = mtof(48);
  f2 = mtof(74);
  f3 = mtof(64);
  f4 = mtof(77);
  // set Oscils with chosen frequencies
  aCos1.setFreq(f1);
  aCos2.setFreq(f2);
  aCos3.setFreq(f3);
  aCos4.setFreq(f4);
  // set frequencies of duplicate oscillators
  aCos1b.setFreq(f1 + variation());
  aCos2b.setFreq(f2 + variation());
  aCos3b.setFreq(f3 + variation());
  aCos4b.setFreq(f4 + variation());

  // filter setup
  svf.setResonance(48); // 0 to 255, 0 is the "sharp" end
  svf.setCentreFreq(1000);

  // FMsetup
  kNoteChangeDelay.set(768); // ms countdown, taylored to resolution of CONTROL_RATE
  kModIndex.setFreq(.768f); // sync with kNoteChangeDelay
  kLfo.setFreq(lFreq);

  note_change_step = Q7n0_to_Q7n8(3);
  note_upper_limit = Q7n0_to_Q7n8(64);
  note_lower_limit = Q7n0_to_Q7n8(24);
  //note0 = note_lower_limit;
  //note1 = note_lower_limit + Q7n0_to_Q7n8(5);
  //attack = 0;
  //decay = 500;
  // might add back
  //envelope.setADLevels(255, 200);
  //envelope.setTimes(50, 200, 1000, 200); // 10000 is so the note will sustain 10 seconds unless a noteOff comes

  // for the env
  randSeed(); // fresh random
  startMozzi(CONTROL_RATE);
}

// mode 0: FM mode 1: chord, mode 2: add
void check_modes() {
  if (digitalRead(SW_PIN_1)) {
    if (digitalRead(SW_PIN_2)) {
      mode = 1;
    }
    else {
      mode = 2;
    }
  }
  else {
    mode = 0;
  }
  //mode = (mode + mode_val)%3;

  if (mode == 2) {
    digitalWrite(LED_1_PIN, HIGH);
    digitalWrite(LED_2_PIN, LOW);
    digitalWrite(LED_3_PIN, LOW);
    return;
  }
  else if (mode == 1) {
    digitalWrite(LED_2_PIN, HIGH);
    digitalWrite(LED_1_PIN, LOW);
    digitalWrite(LED_3_PIN, LOW);
    return;
  }
  else if (mode == 0) {
    digitalWrite(LED_3_PIN, HIGH);
    digitalWrite(LED_2_PIN, LOW);
    digitalWrite(LED_1_PIN, LOW);
    return;
  }
}
void read_inputs() {
  gain_val = constrain(mozziAnalogRead(GAIN_CV_PIN) / 2, 0 , 255);
  mode_val = constrain(mozziAnalogRead(MODE_CV_PIN), 200, 3000);
  svf.setCentreFreq(mode_val); // add filter:)
  svf.setResonance(constrain(mode_val, 24, 196));
}


void updateControl() {
  read_inputs();
  check_modes();
  if ( mode == 0 ) {
    updateReso();
  } else if ( mode == 1 ) {
    updateFM();
  } else {
    updateWavePacket();
  }
}

void updateFM() {

  //byte cutoff_freq = knob>>4;
  //kAverageF.next( mozziAnalogRead(FUNDAMENTAL_PIN)>>1 ) + kAverageM1.next(mozziAnalogRead(A5)>>1 ) / 2  ,

  int note0 = map(mozziAnalogRead(FUNDAMENTAL_PIN), 0, 1023, 1024, 6144);
  int note1 = map(mozziAnalogRead(VOCT), 0, 1023, 1024, 6144);
  int target_note = note0;

  if (note1 > 1048) {
    target_note = note1 + note0 / 2;
  } else {
    target_note = note0;
  }

  last_note = target_note;
  //Serial.println(target_note);
  int modulate, modI;

  int bw = mozziAnalogRead(BANDWIDTH_PIN) ;
  int mw = mozziAnalogRead(P1CV);
  // make sure we only mix if we have a signal on mod pin
  if ( mw > 1 ) {
    modulate = ( bw + mw );
    modI = map(modulate, 0, 1023, 4, 256);
  } else {
    modI = map(bw, 0, 1023, 4, 256);
  }

  int cw = mozziAnalogRead(CENTREFREQ_PIN) ;
  int cm = mozziAnalogRead(P2CV);
  // make sure we only mix if we have a signal on mod pin
  if ( cw > 1 ) {
    //Serial.println(cw);
    centre = map( ( cw + cm ), 0, 1023, 1, 8);
  } else {
    centre = map(cw, 0, 1023, 1, 8);
  }
  mod_to_carrier_ratio = centre;
  //mod_to_carrier_ratio = (Q8n8)(map( mozziAnalogRead(CENTREFREQ_PIN), 0, 1023, 1,8));
  // vary the modulation index
  mod_index = (Q8n8)modI + kModIndex.next();

  smoothed_note = kSmoothNote.next(target_note);
  setFreqs(target_note);


}

//FM
void setFreqs(Q8n8 midi_note) {

  carrier_freq = Q16n16_mtof(Q8n8_to_Q16n16(midi_note)); // convert midi note to fractional frequency
  mod_freq = ((carrier_freq >> 8) * mod_to_carrier_ratio)  ; // (Q16n16>>8)   Q8n8 = Q16n16, beware of overflow
  deviation = ((mod_freq >> 16) * mod_index); // (Q16n16>>16)   Q8n8 = Q24n8, beware of overflow
  aCarrier.setFreq_Q16n16(carrier_freq);
  aModulator.setFreq_Q16n16(mod_freq);

}
/*
  void updateWavePacket2() {

  int noteA = kAverageF.next(mozziAnalogRead(FUNDAMENTAL_PIN));
  int noteB = kAverageF.next(mozziAnalogRead(VOCT));
  int target_note;
  target_note = noteA;

  if (noteB > 20 ) {
    target_note = noteA + noteB / 2;;
  } else {
    target_note = noteA;
  }

  int bw = kAverageBw.next(mozziAnalogRead(BANDWIDTH_PIN)) ;
  int bm = kAverageM2.next(mozziAnalogRead(P1CV));
  int bandwidth;
  bandwidth = bw + bm / 2; //map(bw,0,1023,10,600);

  int cw = kAverageCf.next(mozziAnalogRead<11>(CENTREFREQ_PIN)) ;
  int cm = kAverageM2.next(mozziAnalogRead<11>(P2CV));
  int center;
  center = cw +cm /2;


    wavey.set(target_note+10, // 10 - 1024
    bandwidth, // (0 -1023)
    center); // 0 - 2047

  }
*/
void updateWavePacket() {

  /*
      wavey.set(kAverageF.next(mozziAnalogRead<10>(FUNDAMENTAL_PIN))+10, // 10 - 1024
      kAverageBw.next(mozziAnalogRead<10>(BANDWIDTH_PIN)), // (0 -1023)
      kAverageCf.next(mozziAnalogRead<11>(CENTREFREQ_PIN))); // 0 - 2047
  */
  int noteA = map(mozziAnalogRead(FUNDAMENTAL_PIN), 0, 1023, 2, 50);
  int noteB = map(mozziAnalogRead(VOCT), 0, 1023, 2, 50);
  int target_note;

  target_note = noteA;

  if (noteB > 5 ) {
    target_note = noteB + noteA / 2;
  } else {
    target_note = noteA;
  }

  int bw = mozziAnalogRead(BANDWIDTH_PIN) ;
  int bm = mozziAnalogRead(P1CV);
  bandwidth = map(bw, 0, 1023, 10, 600);
  // make sure we only mix if we have a signal on mod pin

  if ( bm > bw ) {
    bandwidth = (map(bm, 0, 1023, 10, 400) +  map(bw, 0, 1023, 10, 400)) / 2;
  } else {
    bandwidth = map(bw, 0, 1023, 10, 400);
  }

  int cw = mozziAnalogRead(CENTREFREQ_PIN) ;
  int cm = mozziAnalogRead(P2CV);
  // make sure we only mix if we have a signal on mod pin
  //centre = map(cw,0,1023,60,600);

  if ( cm > cw ) {
    centre = (map(cm, 0, 1023, 60, 400) + map(cw, 0, 1023, 60, 400) ) / 2;
  } else {
    centre = map(cw, 0, 1023, 60, 400);
  }

  wavey.set( target_note, bandwidth, centre );
  //wavey.set( note0,  map(bw, 0, 1023, 128, 1023), map(cw, 0, 1023, 8, 512) );


}

void updateReso() {
  int bw = mozziAnalogRead(BANDWIDTH_PIN) ;
  int bm = mozziAnalogRead(P1CV);
  int variation = map(bw, 0, 1023, 1, 14);
  // make sure we only mix if we have a signal on mod pin

  if ( bm > 4 ) {
    variation = (map(bm, 0, 1023, 1, 14) +  map(bw, 0, 1023, 1, 14) / 2);
  } else {
    variation = map(bw, 0, 1023, 1, 14);
  }

  int cw = map(mozziAnalogRead(CENTREFREQ_PIN), 0, 1023, 0, 5 );
  int cm = mozziAnalogRead(P2CV);
  bandwidth = map(cm, 0, 1023, 0, 5);
  if (cw > 0) {
    bandwidth = random(cw);
  }

  int noteA = sensor_to_midi( (float) mozziAnalogRead(VOCT) ) ; //map(mozziAnalogRead(VOCT), 0, 1023, 36, 64);

  int noteB = map(mozziAnalogRead(FUNDAMENTAL_PIN), 0, 1023, 0, 7);
  int target;

  target = noteA + noteB;
  float target_note = mtof(target);
  float v_note = mtof(target + variation);

  switch (bandwidth) { // 7 is 0111
    case 0:
      aCos1.setFreq(target_note);
      aCos1b.setFreq(target_note + variation);
      break;

    case 1:
      aCos2.setFreq(target_note);
      aCos2b.setFreq(target_note + variation);
      break;

    case 2:
      aCos3.setFreq(target_note);
      aCos3b.setFreq(target_note + variation);
      break;

    case 3:
      aCos4.setFreq(target_note);
      aCos4b.setFreq(target_note + variation);
      break;
  }
}

AudioOutput updateAudio() {
  if ( mode == 0 ) {
    auto asig =
      toSFraction(aCos1.next()) + toSFraction(aCos1b.next()) +
      toSFraction(aCos2.next()) + toSFraction(aCos2b.next()) +
      toSFraction(aCos3.next()) + toSFraction(aCos3b.next()) +
      toSFraction(aCos4.next()) + toSFraction(aCos4b.next()) ;
    return MonoOutput::fromSFix(asig);
  } else if ( mode == 1 ) {
    Q15n16 modulation = deviation * aModulator.next() >> 8;
    int input = aCarrier.phMod(modulation);
    int output = svf.next(input);
    return MonoOutput::fromNBit(10, output);
  } else if ( mode == 2 ) {
    return  MonoOutput::from16Bit( wavey.next() )  ;
  }
  return 0; // should not get here
}

void loop() {
  audioHook(); // required here
}
