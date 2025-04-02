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
#include <tables/triangle_warm8192_int8.h>
#include <mozzi_midi.h>
#include <mozzi_rand.h>
#include <mozzi_fixmath.h>
#include <Smooth.h>
#include <ADSR.h>
#include <Midier.h>

#define MOZZI_AUDIO_RATE 32768
#define MOZZI_CONTROL_RATE 256
//#define MOZZI_PWM_RATE 32768
// Envelopes
// --------------------------------------------------------------------
ADSR <MOZZI_AUDIO_RATE, MOZZI_AUDIO_RATE, unsigned long> envelopeVCO;

bool debug = false;

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


WavePacket <DOUBLE>wavey; // <DOUBLE> wavey; // DOUBLE selects 2 overlapping streams

// chordsynth

const midier::Mode modes[] = {
  midier::Mode::Ionian,
  midier::Mode::Dorian,
  midier::Mode::Phrygian,
  midier::Mode::Lydian,
  midier::Mode::Mixolydian,
  midier::Mode::Aeolian,
  midier::Mode::Locrian,
};

const midier::Note notes[] = {
  midier::Note::C,
  midier::Note::C,
  midier::Note::D,
  midier::Note::D,
  midier::Note::E,
  midier::Note::F,
  midier::Note::F,
  midier::Note::G,
  midier::Note::G,
  midier::Note::A,
  midier::Note::A,
  midier::Note::B,

};
const midier::Quality qualities[] = {
  midier::Quality::major,
  midier::Quality::aug,
  midier::Quality::minor,
  midier::Quality::dim,
  midier::Quality::m7b5,
  midier::Quality::m7,
  midier::Quality::dom7,
  midier::Quality::maj7,
  midier::Quality::aug7,
};
const midier::Degree degrees[] = { 1, 3, 5, 7 };
midier::Quality quality = midier::Quality::major; // boring :)
#include <tables/triangle_warm8192_int8.h>

// harmonics
Oscil<TRIANGLE_WARM8192_NUM_CELLS, MOZZI_AUDIO_RATE> aCos1(TRIANGLE_WARM8192_DATA);
Oscil<TRIANGLE_WARM8192_NUM_CELLS, MOZZI_AUDIO_RATE> aCos2(TRIANGLE_WARM8192_DATA);
Oscil<TRIANGLE_WARM8192_NUM_CELLS, MOZZI_AUDIO_RATE> aCos3(TRIANGLE_WARM8192_DATA);
Oscil<TRIANGLE_WARM8192_NUM_CELLS, MOZZI_AUDIO_RATE> aCos4(TRIANGLE_WARM8192_DATA);
// duplicates but slightly off frequency for adding to originals
Oscil<TRIANGLE_WARM8192_NUM_CELLS, MOZZI_AUDIO_RATE> aCos1b(TRIANGLE_WARM8192_DATA);
Oscil<TRIANGLE_WARM8192_NUM_CELLS, MOZZI_AUDIO_RATE> aCos2b(TRIANGLE_WARM8192_DATA);
Oscil<TRIANGLE_WARM8192_NUM_CELLS, MOZZI_AUDIO_RATE> aCos3b(TRIANGLE_WARM8192_DATA);
Oscil<TRIANGLE_WARM8192_NUM_CELLS, MOZZI_AUDIO_RATE> aCos4b(TRIANGLE_WARM8192_DATA);


// base pitch frequencies in Q16n16 fixed int format (for speed later)
int f1, f2, f3, f4, f5, f6, f7;

// for FMsynth

Oscil<COS2048_NUM_CELLS, MOZZI_AUDIO_RATE> aCarrier(COS2048_DATA);
Oscil<COS2048_NUM_CELLS, MOZZI_AUDIO_RATE> aModulator(COS2048_DATA);
Oscil<COS2048_NUM_CELLS, MOZZI_AUDIO_RATE> aModDepth(COS2048_DATA);
Oscil<COS2048_NUM_CELLS, MOZZI_CONTROL_RATE> kModIndex(COS2048_DATA);
Q8n8 mod_index;// = float_to_Q8n8(2.0f); // constant version
Q16n16 deviation;
Q16n16 carrier_freq, mod_freq, fundemental, bandwidth, centre;

// FM ratio between oscillator frequencies, stays the same through note range
Q8n8 mod_to_carrier_ratio = float_to_Q8n8(3.f);

long fmIntensity; // Carries control info from updateControl to updateAudio
// smoothing for intensity to remove clicks on transitions
float smoothness = 0.95f;
Smooth <long> aSmoothIntensity(smoothness);


int gain;

// for note changes not really using this
Q7n8 last_note, note_upper_limit, note_lower_limit, note_change_step, smoothed_note;


int gain_val = 0;
uint8_t mode = 0;
uint16_t mode_val = 0;
unsigned int notesPlaying;
bool notePlaying = false;
bool noteReset = true;


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
  // wavepacket sample
  //   wavey.setTable(RAVEN_ARH_DATA);
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
  aCos1b.setFreq(f1 + 3);
  aCos2b.setFreq(f2 + 1);
  aCos3b.setFreq(f3 + 5);
  aCos4b.setFreq(f4 + 2);

  // filter setup
  //svf.setResonance(48); // 0 to 255, 0 is the "sharp" end
  //svf.setCentreFreq(1000);

  // FMsetup

  kModIndex.setFreq(.768f); // sync with kNoteChangeDelay

  // might add back

  envelopeVCO.setADLevels(25, 25);
  envelopeVCO.setTimes(10, 100, 6000, 2000); // 10000 is so the note will sustain 10 seconds unless a noteOff comes

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
  gain_val = mozziAnalogRead(GAIN_CV_PIN); // using for trigger
  mode_val = mozziAnalogRead(MODE_CV_PIN); // using for gate

  mode_val = map(mode_val, 0, 4095, 1000, 10000); //let's scale it for weaker signals.
  if (mode_val > 1000) { // only update the envelope if we have a certain amount of signal
    envelopeVCO.setTimes(100, 2000, mode_val, 700);
  }
  if (gain_val > 350 && ! notePlaying ) { // don't retrigger since it's noisy
    envelopeVCO.noteOn();
    notePlaying = true;
  } else if (gain_val < 20 && notePlaying ) {
    envelopeVCO.noteOff();
    notePlaying = false;
  }

  //svf.setCentreFreq(mode_val); // add filter:)
  //svf.setResonance(gain_val);
}


void updateControl() {
  read_inputs();
  check_modes();
  if ( mode == 0 ) {
    updateChords();
  } else if ( mode == 1 ) {
    updateFM();
  } else {
    updateWavePacket();
  }
}

void updateFM() {
  envelopeVCO.setADLevels(25, 25);
  
  //byte cutoff_freq = knob>>4;
  //kAverageF.next( mozziAnalogRead(FUNDAMENTAL_PIN)>>1 ) + kAverageM1.next(mozziAnalogRead(A5)>>1 ) / 2  ,

  int note0 = map(mozziAnalogRead(FUNDAMENTAL_PIN), 0, 1023, 2048, 6144); // offset
  int note1 = map(mozziAnalogRead(VOCT), 0, 1023, 2048, 6144); // note
  int target_note = note0;

  if (note1 > 2048 && note0 > 2048) {
    target_note = note0 + note1 ;
  } else {
    target_note = note1;
  }

  last_note = target_note;

  //Serial.println(target_note);
  int modulate, modI;

  int bw = mozziAnalogRead(BANDWIDTH_PIN) ;
  int mw = mozziAnalogRead(P1CV);
  // make sure we only mix if we have a signal on mod pin
  if ( mw > 1 ) {
    modulate = ( bw + mw );
    modI = map(modulate, 0, 4095, 1, 512);
  } else {
    modI = map(bw, 0, 4095, 1, 512);
  }

  // vary the modulation index
  mod_index = (Q8n8)modI + kModIndex.next();
  float modSpeed = (float)modI;
  aModDepth.setFreq(modSpeed);

  int cw = mozziAnalogRead(CENTREFREQ_PIN) ;
  int cm = mozziAnalogRead(P2CV);
  // make sure we only mix if we have a signal on mod pin
  centre = map( cw, 0, 4095, 1, 512);
  cm = map(cm, 0, 4095, 1, 512);
  if ( cw > 1 ) {
    centre = ( cw + cm ) / 2;
  }

  mod_to_carrier_ratio = (Q8n8)centre;
  fmIntensity = ((long)centre * (aModDepth.next())) >> 8;

  //smoothed_note = kSmoothNote.next(target_note);
  setFreqs(target_note);


}

//FM
void setFreqs(Q8n8 midi_note) {
  carrier_freq = Q16n16_mtof(Q8n8_to_Q16n16(midi_note)); // convert midi note to fractional frequency
  mod_freq = ((carrier_freq >> 8) * mod_to_carrier_ratio)  ; // (Q16n16>>8)   Q8n8 = Q16n16, beware of overflow
  //deviation = ((mod_freq >> 16) * mod_index); // (Q16n16>>16)   Q8n8 = Q24n8, beware of overflow
  aCarrier.setFreq_Q16n16(carrier_freq);
  aModulator.setFreq_Q16n16(mod_freq);

}

void updateWavePacket() {
  /*
      wavey.set(kAverageF.next(mozziAnalogRead<10>(FUNDAMENTAL_PIN))+10, // 10 - 1024
      kAverageBw.next(mozziAnalogRead<10>(BANDWIDTH_PIN)), // (0 -1023)
      kAverageCf.next(mozziAnalogRead<11>(CENTREFREQ_PIN))); // 0 - 2047
  */

  envelopeVCO.setADLevels(70, 70);
  int noteA = map(mozziAnalogRead(FUNDAMENTAL_PIN), 0, 4095, 1, 92);
  int noteB = map(mozziAnalogRead(VOCT), 0, 4095, 1, 396);
  int target_note;
  target_note = noteB + noteA;

  int bw = mozziAnalogRead(BANDWIDTH_PIN) ;
  int bm = mozziAnalogRead(P1CV);
  bandwidth = map(bw, 0, 4095, 1, 256);
  // make sure we only mix if we have a signal on mod pin

  if ( bm > 10 ) {
    bandwidth = (map(bm, 0, 4095, 1, 256) +  bandwidth) / 2;
  }
  if (debug) {
    Serial.print("band: ");
    Serial.println(bandwidth);
  }

  int cw = mozziAnalogRead(CENTREFREQ_PIN) ;
  int cm = mozziAnalogRead(P2CV);
  // make sure we only mix if we have a signal on mod pin
  centre = map(cw, 0, 1023, 1, 512);
  if ( cm > 10 ) {
    centre = (map(cm, 0, 4095, 1, 512) + centre ) / 2 ;
  }
  if (debug) {
    Serial.print("center: ");
    Serial.println(centre);
  }
  wavey.set( target_note, bandwidth, centre );
  //wavey.set( note0,  map(bw, 0, 1023, 128, 1023), map(cw, 0, 1023, 8, 512) );


}

void updateChords() {
  
  envelopeVCO.setADLevels(25, 25);
  
  int bw = mozziAnalogRead(BANDWIDTH_PIN) ;
  int bm = mozziAnalogRead(P1CV);
  int variation ;
  bw = map(bw, 0, 4095, 0, 64);
  bm = map(bm, 0, 4095, 0, 64);
  // make sure we only mix if we have a signal on mod pin
  variation = bw;
  if (bm > 0) variation = bw + bm;

  if (debug) {
    Serial.print("var: ");
    Serial.println(variation);
  }


  int cw = mozziAnalogRead(CENTREFREQ_PIN);
  int cm = mozziAnalogRead(P2CV);
  int bandwidth = map(cw, 0, 4095, 0, 8);
  cm = map(cm, 0, 4095, 0, 3);// offset
  bandwidth = constrain((bandwidth + cm), 0, 8); // can force us to the next types of chord
  midier::Quality quality = qualities[bandwidth];

  if (debug) {
    Serial.print("qual: ");
    Serial.println(bandwidth);
    Serial.println(cw);
  }

  int noteA = mozziAnalogRead(VOCT);
  if (debug) {
    Serial.print("note: ");
    Serial.println(noteA);
  }
  //noteA = sensor_to_midi( (float) mozziAnalogRead(VOCT) ) ; // map(mozziAnalogRead(VOCT), 0, 1023, 21, 81);// A0 = 21
  noteA = map(noteA, 0, 4095, 24, 84);
  if (debug) {
    Serial.print("note: ");
    Serial.println(noteA);
  }

  int noteB = mozziAnalogRead(FUNDAMENTAL_PIN);
  noteB = map(noteB, 0, 4095, 1, 64);

  int octave = ( noteA  / 12 ) + 1;
  int noteIndex = (noteA % 12) ; // offset for A0
  midier::Note root = notes[noteIndex];

  if (debug) {
    Serial.print("index: ");
    Serial.println(noteIndex);
  }
  if (debug) {
    Serial.print("oct: ");
    Serial.println(octave);
    Serial.print("noteB: ");
    Serial.println(noteB);
    Serial.println();
  }
  // iterate over all the degrees
  for (int degree : degrees)
  {
    midier::Interval interval = midier::triad::interval(quality, degree);
    // find out the interval to be added to the root note for this degree and quality
    // we're hacking about with modes for the accidentals :)

    // calculate the note of this degree
    const midier::Note note = root + interval;
    float target_note ;// = mtof(root);
    float v_note = mtof(noteA + variation);
    int midi_number = midier::midi::number(note, octave);

    switch (noteIndex) {
      case 1:
        midi_number = midi_number + 1; //root = midier::Note(root, midier::Accidental::Sharp); //interval = midier::scale::interval(quality, midier::Interval::m2);
        break;
      case 3:
        midi_number = midi_number + 1;
        break;
      case 6:
        midi_number = midi_number + 1;
        break;
      case 8:
        midi_number = midi_number + 1;
        break;
      case 10:
        midi_number = midi_number + 1;
        break;
    }
    target_note = mtof(midi_number);
    switch (degree) { // 7 is 0111
      case 1:
        aCos1.setFreq(target_note);
        aCos1b.setFreq(target_note + random(noteB));
        break;
      case 3:
        aCos2.setFreq(target_note);
        aCos2b.setFreq(target_note + random(noteB));
        break;
      case 5:
        aCos3.setFreq(target_note);
        aCos3b.setFreq(target_note + random(noteB));
        break;
      case 7:
        if ( bandwidth < 4) { // special case for triads
          midier::Interval interval = midier::triad::interval(quality, 1); // just play root.
          midier::Note note = root + interval;
          target_note = mtof(midier::midi::number(note, octave - 1)); //
          aCos4.setFreq(target_note);
          aCos4b.setFreq(target_note + random(noteB));
        } else {
          aCos4.setFreq(target_note);
          aCos4b.setFreq(target_note + random(noteB));
        }
        break;

    }
  }
}


AudioOutput updateAudio() {

  envelopeVCO.update();

  if ( mode == 0 ) {
    
    auto asig = (
                  toSFraction( envelopeVCO.next() *
                               ( aCos1.next() +  aCos1b.next()  +  aCos2.next() + aCos2b.next()  +
                                 aCos3.next() + aCos3b.next()  + aCos4.next() + aCos4b.next() ) ) );
    return MonoOutput::fromSFix(asig);

  } else if ( mode == 1 ) {

    Q15n16 modulation =  aSmoothIntensity.next(fmIntensity) * aModulator.next();
    return MonoOutput::from16Bit( (int)  envelopeVCO.next() * aCarrier.phMod(modulation) );

    //int input = aCarrier.phMod(modulation);
    //int output = svf.next(input);
    //return MonoOutput::fromNBit(10, output);

  } else if ( mode == 2 ) {
      
    //return   MonoOutput::from16Bit( wavey.next() )  ;
    auto asig = (  toSFraction (  wavey.next()) );
    return  MonoOutput::fromSFix(asig * toSFraction(envelopeVCO.next() ) )  ;
  }
  return 0; // should not get here
}

void loop() {
  audioHook(); // required here
}
