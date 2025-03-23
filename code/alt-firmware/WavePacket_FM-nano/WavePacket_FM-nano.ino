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
#include <Midier.h>

#define MOZZI_AUDIO_RATE 32768
//#define MOZZI_PWM_RATE 32768

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
#define CONTROL_RATE 512 // powers of 2 please


// wavetable for oscillator:
#include <tables/sin2048_int8.h>

StateVariable <BANDPASS> svf; // can be LOWPASS, BANDPASS, HIGHPASS or NOTCH


// for fake midi
//EventDelay startNote;
//EventDelay endNote;

WavePacket <DOUBLE>wavey; // <DOUBLE> wavey; // DOUBLE selects 2 overlapping streams

//ADSR <CONTROL_RATE, AUDIO_RATE> envelope;
//unsigned int duration, attack, decay, sustain, release_ms;
int lFreq = 10;

// chordsynth
const static float voctpow[1024] PROGMEM = {
  0,  0.004882, 0.009765, 0.014648, 0.019531, 0.024414, 0.029296, 0.034179, 0.039062, 0.043945, 0.048828, 0.05371,  0.058593, 0.063476, 0.068359, 0.073242, 0.078125, 0.083007, 0.08789,  0.092773, 0.097656, 0.102539, 0.107421, 0.112304, 0.117187, 0.12207,  0.126953, 0.131835, 0.136718, 0.141601, 0.146484, 0.151367, 0.15625,
  0.161132, 0.166015, 0.170898, 0.175781, 0.180664, 0.185546, 0.190429, 0.195312, 0.200195, 0.205078, 0.20996,  0.214843, 0.219726, 0.224609, 0.229492, 0.234375, 0.239257, 0.24414,  0.249023, 0.253906, 0.258789, 0.263671, 0.268554, 0.273437, 0.27832,  0.283203, 0.288085, 0.292968, 0.297851, 0.302734, 0.307617, 0.3125, 0.317382,
  0.322265, 0.327148, 0.332031, 0.336914, 0.341796, 0.346679, 0.351562, 0.356445, 0.361328, 0.36621,  0.371093, 0.375976, 0.380859, 0.385742, 0.390625, 0.395507, 0.40039,  0.405273, 0.410156, 0.415039, 0.419921, 0.424804, 0.429687, 0.43457,  0.439453, 0.444335, 0.449218, 0.454101, 0.458984, 0.463867, 0.46875,  0.473632, 0.478515,
  0.483398, 0.488281, 0.493164, 0.498046, 0.502929, 0.507812, 0.512695, 0.517578, 0.52246,  0.527343, 0.532226, 0.537109, 0.541992, 0.546875, 0.551757, 0.55664,  0.561523, 0.566406, 0.571289, 0.576171, 0.581054, 0.585937, 0.59082,  0.595703, 0.600585, 0.605468, 0.610351, 0.615234, 0.620117, 0.625,  0.629882, 0.634765, 0.639648,
  0.644531, 0.649414, 0.654296, 0.659179, 0.664062, 0.668945, 0.673828, 0.67871,  0.683593, 0.688476, 0.693359, 0.698242, 0.703125, 0.708007, 0.71289,  0.717773, 0.722656, 0.727539, 0.732421, 0.737304, 0.742187, 0.74707,  0.751953, 0.756835, 0.761718, 0.766601, 0.771484, 0.776367, 0.78125,  0.786132, 0.791015, 0.795898, 0.800781, 0.805664, 0.810546,
  0.815429, 0.820312, 0.825195, 0.830078, 0.83496,  0.839843, 0.844726, 0.849609, 0.854492, 0.859375, 0.864257, 0.86914,  0.874023, 0.878906, 0.883789, 0.888671, 0.893554, 0.898437, 0.90332,  0.908203, 0.913085, 0.917968, 0.922851, 0.927734, 0.932617, 0.9375, 0.942382, 0.947265, 0.952148, 0.957031, 0.961914, 0.966796, 0.971679, 0.976562, 0.981445, 0.986328, 0.99121,  0.996093, 1.000976, 1.005859, 1.010742, 1.015625, 1.020507, 1.02539,  1.030273, 1.035156, 1.040039, 1.044921, 1.049804, 1.054687, 1.05957,  1.064453, 1.069335, 1.074218, 1.079101, 1.083984,
  1.088867, 1.09375,  1.098632, 1.103515, 1.108398, 1.113281, 1.118164, 1.123046, 1.127929, 1.132812, 1.137695, 1.142578, 1.14746,  1.152343, 1.157226, 1.162109, 1.166992, 1.171875, 1.176757, 1.18164,  1.186523, 1.191406, 1.196289, 1.201171, 1.206054, 1.210937, 1.21582,  1.220703, 1.225585, 1.230468, 1.235351, 1.240234, 1.245117, 1.25, 1.254882, 1.259765, 1.264648, 1.269531, 1.274414, 1.279296, 1.284179, 1.289062, 1.293945, 1.298828, 1.30371,  1.308593, 1.313476, 1.318359, 1.323242, 1.328125, 1.333007, 1.33789,  1.342773, 1.347656, 1.352539, 1.357421, 1.362304, 1.367187, 1.37207,  1.376953, 1.381835, 1.386718, 1.391601, 1.396484, 1.401367, 1.40625,  1.411132, 1.416015, 1.420898, 1.425781, 1.430664, 1.435546, 1.440429, 1.445312, 1.450195, 1.455078, 1.45996,  1.464843, 1.469726, 1.474609, 1.479492, 1.484375, 1.489257, 1.49414,  1.499023, 1.503906, 1.508789, 1.513671, 1.518554, 1.523437, 1.52832,  1.533203, 1.538085, 1.542968, 1.547851, 1.552734, 1.557617, 1.5625, 1.567382, 1.572265, 1.577148, 1.582031, 1.586914, 1.591796, 1.596679, 1.601562, 1.606445, 1.611328, 1.61621,  1.621093, 1.625976, 1.630859, 1.635742, 1.640625, 1.645507, 1.65039,  1.655273, 1.660156, 1.665039, 1.669921, 1.674804, 1.679687, 1.68457,  1.689453, 1.694335, 1.699218, 1.704101, 1.708984, 1.713867, 1.71875,  1.723632, 1.728515, 1.733398, 1.738281, 1.743164, 1.748046, 1.752929, 1.757812, 1.762695, 1.767578, 1.77246,  1.777343, 1.782226, 1.787109, 1.791992, 1.796875, 1.801757, 1.80664,  1.811523, 1.816406, 1.821289, 1.826171, 1.831054, 1.835937, 1.84082,  1.845703, 1.850585, 1.855468, 1.860351, 1.865234, 1.870117, 1.875,  1.879882, 1.884765, 1.889648, 1.894531, 1.899414, 1.904296, 1.909179, 1.914062, 1.918945, 1.923828, 1.92871,  1.933593, 1.938476, 1.943359, 1.948242, 1.953125, 1.958007, 1.96289,  1.967773, 1.972656, 1.977539, 1.982421, 1.987304, 1.992187, 1.99707,  2.001953, 2.006835, 2.011718, 2.016601, 2.021484, 2.026367, 2.03125,  2.036132, 2.041015, 2.045898, 2.050781, 2.055664, 2.060546, 2.065429, 2.070312, 2.075195, 2.080078, 2.08496,  2.089843, 2.094726, 2.099609, 2.104492, 2.109375, 2.114257, 2.11914,  2.124023, 2.128906, 2.133789, 2.138671, 2.143554, 2.148437, 2.15332,  2.158203, 2.163085, 2.167968, 2.172851, 2.177734, 2.182617, 2.1875, 2.192382, 2.197265, 2.202148, 2.207031, 2.211914, 2.216796, 2.221679, 2.226562, 2.231445, 2.236328, 2.24121,  2.246093, 2.250976, 2.255859, 2.260742, 2.265625, 2.270507, 2.27539,  2.280273, 2.285156, 2.290039, 2.294921, 2.299804, 2.304687, 2.30957,  2.314453, 2.319335, 2.324218, 2.329101, 2.333984, 2.338867, 2.34375,  2.348632, 2.353515, 2.358398, 2.363281, 2.368164, 2.373046, 2.377929, 2.382812, 2.387695, 2.392578, 2.39746,  2.402343, 2.407226, 2.412109, 2.416992, 2.421875, 2.426757, 2.43164,  2.436523, 2.441406, 2.446289, 2.451171, 2.456054, 2.460937, 2.46582,  2.470703, 2.475585, 2.480468, 2.485351, 2.490234, 2.495117, 2.5,  2.504882, 2.509765, 2.514648, 2.519531, 2.524414, 2.529296, 2.534179, 2.539062, 2.543945, 2.548828, 2.55371,  2.558593, 2.563476, 2.568359, 2.573242, 2.578125, 2.583007, 2.58789,  2.592773, 2.597656, 2.602539, 2.607421, 2.612304, 2.617187, 2.62207,  2.626953, 2.631835, 2.636718, 2.641601, 2.646484, 2.651367, 2.65625,  2.661132, 2.666015, 2.670898, 2.675781, 2.680664, 2.685546, 2.690429, 2.695312, 2.700195, 2.705078, 2.70996,  2.714843, 2.719726, 2.724609, 2.729492, 2.734375, 2.739257, 2.74414,  2.749023, 2.753906, 2.758789, 2.763671, 2.768554, 2.773437, 2.77832,  2.783203, 2.788085, 2.792968, 2.797851, 2.802734, 2.807617, 2.8125, 2.817382, 2.822265, 2.827148, 2.832031, 2.836914, 2.841796, 2.846679, 2.851562, 2.856445, 2.861328, 2.86621,  2.871093, 2.875976, 2.880859, 2.885742, 2.890625, 2.895507, 2.90039,  2.905273, 2.910156, 2.915039, 2.919921, 2.924804, 2.929687, 2.93457,  2.939453, 2.944335, 2.949218, 2.954101, 2.958984, 2.963867, 2.96875,  2.973632, 2.978515, 2.983398, 2.988281, 2.993164, 2.998046, 3.002929, 3.007812, 3.012695, 3.017578, 3.02246,  3.027343, 3.032226, 3.037109, 3.041992, 3.046875, 3.051757, 3.05664,  3.061523, 3.066406, 3.071289, 3.076171, 3.081054, 3.085937, 3.09082,  3.095703, 3.100585, 3.105468, 3.110351, 3.115234, 3.120117, 3.125,  3.129882, 3.134765, 3.139648, 3.144531, 3.149414, 3.154296, 3.159179, 3.164062, 3.168945, 3.173828, 3.17871,  3.183593, 3.188476, 3.193359, 3.198242, 3.203125, 3.208007, 3.21289,  3.217773, 3.222656, 3.227539, 3.232421, 3.237304, 3.242187, 3.24707,  3.251953, 3.256835, 3.261718, 3.266601, 3.271484, 3.276367, 3.28125,  3.286132, 3.291015, 3.295898, 3.300781, 3.305664, 3.310546, 3.315429, 3.320312, 3.325195, 3.330078, 3.33496,  3.339843, 3.344726, 3.349609, 3.354492, 3.359375, 3.364257, 3.36914,  3.374023, 3.378906, 3.383789, 3.388671, 3.393554, 3.398437, 3.40332,  3.408203, 3.413085, 3.417968, 3.422851, 3.427734, 3.432617, 3.4375, 3.442382, 3.447265, 3.452148, 3.457031, 3.461914, 3.466796, 3.471679, 3.476562, 3.481445, 3.486328, 3.49121,  3.496093, 3.500976, 3.505859, 3.510742, 3.515625, 3.520507, 3.52539,  3.530273, 3.535156, 3.540039, 3.544921, 3.549804, 3.554687, 3.55957,  3.564453, 3.569335, 3.574218, 3.579101, 3.583984, 3.588867, 3.59375,  3.598632, 3.603515, 3.608398, 3.613281, 3.618164, 3.623046, 3.627929, 3.632812, 3.637695, 3.642578, 3.64746,  3.652343, 3.657226, 3.662109, 3.666992, 3.671875, 3.676757, 3.68164,  3.686523, 3.691406, 3.696289, 3.701171, 3.706054, 3.710937, 3.71582,  3.720703, 3.725585, 3.730468, 3.735351, 3.740234, 3.745117, 3.75, 3.754882, 3.759765, 3.764648, 3.769531, 3.774414, 3.779296, 3.784179, 3.789062, 3.793945, 3.798828, 3.80371,  3.808593, 3.813476, 3.818359, 3.823242, 3.828125, 3.833007, 3.83789,  3.842773, 3.847656, 3.852539, 3.857421, 3.862304, 3.867187, 3.87207,  3.876953, 3.881835, 3.886718, 3.891601, 3.896484, 3.901367, 3.90625,  3.911132, 3.916015, 3.920898, 3.925781, 3.930664, 3.935546, 3.940429, 3.945312, 3.950195, 3.955078, 3.95996,  3.964843, 3.969726, 3.974609, 3.979492, 3.984375, 3.989257, 3.99414,  3.999023, 4.003906, 4.008789, 4.013671, 4.018554, 4.023437, 4.02832,  4.033203, 4.038085, 4.042968, 4.047851, 4.052734, 4.057617, 4.0625, 4.067382, 4.072265, 4.077148, 4.082031, 4.086914, 4.091796, 4.096679, 4.101562, 4.106445, 4.111328, 4.11621,  4.121093, 4.125976, 4.130859, 4.135742, 4.140625, 4.145507, 4.15039,  4.155273, 4.160156, 4.165039, 4.169921, 4.174804, 4.179687, 4.18457,  4.189453, 4.194335, 4.199218, 4.204101, 4.208984, 4.213867, 4.21875,  4.223632, 4.228515, 4.233398, 4.238281, 4.243164, 4.248046, 4.252929, 4.257812, 4.262695, 4.267578, 4.27246,  4.277343, 4.282226, 4.287109, 4.291992, 4.296875, 4.301757, 4.30664,  4.311523, 4.316406, 4.321289, 4.326171, 4.331054, 4.335937, 4.34082,  4.345703, 4.350585, 4.355468, 4.360351, 4.365234, 4.370117, 4.375,  4.379882, 4.384765, 4.389648, 4.394531, 4.399414, 4.404296, 4.409179, 4.414062, 4.418945, 4.423828, 4.42871,  4.433593, 4.438476, 4.443359, 4.448242, 4.453125, 4.458007, 4.46289,  4.467773, 4.472656, 4.477539, 4.482421, 4.487304, 4.492187, 4.49707,  4.501953, 4.506835, 4.511718, 4.516601, 4.521484, 4.526367, 4.53125,  4.536132, 4.541015, 4.545898, 4.550781, 4.555664, 4.560546, 4.565429, 4.570312, 4.575195, 4.580078, 4.58496,  4.589843, 4.594726, 4.599609, 4.604492, 4.609375, 4.614257, 4.61914,  4.624023, 4.628906, 4.633789, 4.638671, 4.643554, 4.648437, 4.65332,  4.658203, 4.663085, 4.667968, 4.672851, 4.677734, 4.682617, 4.6875, 4.692382, 4.697265, 4.702148, 4.707031, 4.711914, 4.716796, 4.721679, 4.726562, 4.731445, 4.736328, 4.74121,  4.746093, 4.750976, 4.755859, 4.760742, 4.765625, 4.770507, 4.77539,  4.780273, 4.785156, 4.790039, 4.794921, 4.799804, 4.804687, 4.80957,  4.814453, 4.819335, 4.824218, 4.829101, 4.833984, 4.838867, 4.84375,  4.848632, 4.853515, 4.858398, 4.863281, 4.868164, 4.873046, 4.877929, 4.882812, 4.887695, 4.892578, 4.89746,  4.902343, 4.907226, 4.912109, 4.916992, 4.921875, 4.926757, 4.93164,  4.936523, 4.941406, 4.946289, 4.951171, 4.956054, 4.960937, 4.96582,  4.970703, 4.975585, 4.980468, 4.985351, 4.990234, 4.995117
};

const midier::Note notes[] = { // just because I'm lazy
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
const midier::Degree fourdegrees[] = { 1, 3, 5, 7 };
const midier::Degree threedegrees[] = { 1, 3, 5, 8 };
midier::Quality quality = midier::Quality::major;

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
#define BASE_NOTE_FREQUENCY  16.3516
// Function converting frequency to offset from BASE_NOTE_FREQUENCY
float FreqToNote(float frequency) {
  float x = (frequency / BASE_NOTE_FREQUENCY);
  float y = 12.0 * log(x) / log(2.0);
  return y;
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
static const uint32_t midi_note_at_zero_volts = 24;
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
   4.096/1024.0 internal voltage ref is 4.09a6
*/
float sensor_to_midi(float sensorValue) {
  float volts = sensorValue * (4.096 / 1024.0);
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
  gain_val = constrain(mozziAnalogRead(GAIN_CV_PIN), 4 , 196);
  mode_val = constrain(mozziAnalogRead(MODE_CV_PIN), 200, 3000);
  svf.setCentreFreq(mode_val); // add filter:)
  svf.setResonance(gain_val);
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

  //byte cutoff_freq = knob>>4;
  //kAverageF.next( mozziAnalogRead(FUNDAMENTAL_PIN)>>1 ) + kAverageM1.next(mozziAnalogRead(A5)>>1 ) / 2  ,

  int note0 = map(mozziAnalogRead(FUNDAMENTAL_PIN), 0, 1023, 2048, 6144);
  int note1 = map(mozziAnalogRead(VOCT), 0, 1023, 2048, 6144);
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
    modI = map(modulate, 0, 1023, 64, 256);
  } else {
    modI = map(bw, 0, 1023, 64, 256);
  }

  // vary the modulation index
  mod_index = (Q8n8)modI + kModIndex.next();

  int cw = mozziAnalogRead(CENTREFREQ_PIN) ;
  int cm = mozziAnalogRead(P2CV);
  // make sure we only mix if we have a signal on mod pin
  centre = map( cw, 0, 1023, 1, 4);
  cm = map(cm, 0, 1023, 1, 4);
  if ( cw > 1 ) {
    centre = ( cw + cm ) / 2;
  }

  mod_to_carrier_ratio = (float)centre;

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
  if (debug) {
    Serial.print("band: ");
    Serial.println(bandwidth);
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
  if (debug) {
    Serial.print("center: ");
    Serial.println(centre);
  }
  wavey.set( target_note, bandwidth, centre );
  //wavey.set( note0,  map(bw, 0, 1023, 128, 1023), map(cw, 0, 1023, 8, 512) );


}

void updateChords() {

  int bw = mozziAnalogRead(BANDWIDTH_PIN) ;
  int bm = mozziAnalogRead(P1CV);
  int variation ;
  bw = map(bw, 0, 1023, 1, 12);
  bm = map(bm, 0, 1023, 1, 12);
  // make sure we only mix if we have a signal on mod pin
  variation = bw;
  if ( bm > 2 ) {
    variation = (bm + bw / 2);
  }

  if (debug) {
    Serial.print("var: ");
    Serial.println(variation);
  }


  int cw = mozziAnalogRead(CENTREFREQ_PIN);
  int cm = mozziAnalogRead(P2CV);
  cw = map(cw, 0, 1023, 0, 8);
  cm = map(cm, 0, 1023, 0, 8);

  if (cm > 0) {
    bandwidth = (cw + cm / 2);
  } else {
    bandwidth = cw;
  }
  quality = qualities[bandwidth];
  if (debug) {
    Serial.print("qual: ");
    Serial.println(bandwidth);
  }

  int noteA = mozziAnalogRead(VOCT);
  noteA = map(noteA, 0, 1023, 24, 72);
  if (debug) {
    Serial.print("note: ");
    Serial.println(noteA);
  }
  //int noteA = sensor_to_midi( (float) mozziAnalogRead(VOCT) ) ; // map(mozziAnalogRead(VOCT), 0, 1023, 21, 81);// A0 = 21
  int noteB = mozziAnalogRead(FUNDAMENTAL_PIN);
  noteB = map(noteB, 0, 1023, 1, 7);

  int octave = ( noteA  / 12 ) - 1;
  int noteIndex = (noteA % 12) ; // offset for A0
  midier::Note root = notes[noteIndex];

  if (debug) {
    Serial.print("index: ");
    Serial.println(noteIndex);
  }
  if (debug) {
    Serial.print("oct: ");
    Serial.println(octave);
    Serial.println();
  }


  float target_note ;// = mtof(root);
  float v_note = mtof(noteA + variation);

  midier::Degree thesedegrees[] = {1,3,5,7};
  if (bandwidth < 4) {
   midier::Degree  thesedegrees[] = {1,3,5,8};
  } 
  // iterate over all the degrees
  for (auto degree : thesedegrees)
  {
    // find out the interval to be added to the root note for this degree and quality
    midier::Interval interval = midier::triad::interval(quality, degree);

    // calculate the note of this degree
    midier::Note note = root + interval;
    target_note = mtof(midier::midi::number(note, octave));

    switch (degree) { // 7 is 0111
      case 1:
        aCos1.setFreq(target_note);
        aCos1b.setFreq(target_note + (float)variation);
        break;
      case 3:
        aCos2.setFreq(target_note);
        aCos2b.setFreq(target_note + (float)variation);
        break;
      case 5:
        aCos3.setFreq(target_note);
        aCos3b.setFreq(target_note + (float)variation);
        break;
      case 7:
        aCos4.setFreq(target_note);
        aCos4b.setFreq(target_note + (float)variation);
        break;
      case 8:
        aCos4.setFreq(target_note);
        aCos4b.setFreq(target_note + (float)variation);
        break;
    }

  }

  // play the note
  //playNote(note);
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
    return MonoOutput::fromNBit(10, aCarrier.phMod(modulation));
    //int input = aCarrier.phMod(modulation);
    //int output = svf.next(input);
    //return MonoOutput::fromNBit(10, output);
  } else if ( mode == 2 ) {
    return  MonoOutput::from16Bit( wavey.next() )  ;
  }
  return 0; // should not get here
}

void loop() {
  audioHook(); // required here
}
