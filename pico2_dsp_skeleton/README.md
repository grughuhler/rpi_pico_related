# Pico 2 I2S Audio DSP Framework

This program is a real-time Audio DSP framework built using the RP2350
(Raspberry Pi Pico 2). It uses PIO to implement 4-wire (send and
receive) i2s.  PIO also implements all needed clocks using side sets.

It is presented in YouTube videos: https://youtu.be/R24kLI5O0y0
and https://youtu.be/IiyGa5ss1Dw

It uses ARM's CMSIS_DSP library.

Note: This README describes the latest version of this software, a
version that may be more recent than any YouTube video.  It may be
best to use this version, but there are tags associated with the
software state that matches video releases.

Git Tags:

video_pico2_dsp_apps matches https://youtu.be/R24kLI5O0y0
video_pico2_dsp_skeleton matches https://youtu.be/IiyGa5ss1Dw

A PCM1808 i2s ADC provides input data and a PCM5102A i2s DAC converts
samples back to analog for output.

A cyclic triple buffer DMA is used.  While software processes one buffer,
the ADC DMA is filling the next and the DAC DMA is sending the previous.
Software sample processing is in-place.

## Algorithms

There are many files with name dsp_XXX.c.  Each implements a different
DSP demonstration, but each provides the same interface to main.c.

    void init_dsp(void);
    void process_buf_dsp(q31_t *buf);

The build process creates an executable for each dsp_XXX.c file,
naming the result XXX.elf.  The file to flash onto the Pico2 is
XXX.uf2.  For example, to run the FIR demontration, flash file
fir.uf2.

See each dsp_XXX.c file for more information, but here is a summary:

dsp_fir.c implements the FIR filter with coefficients from
fir_coeffs.h and applies it to the left channel.

dsp_fft_filter.c implements with FIR filter with coefficients from
fft_filter_coeffs.h using FFT convolution.  This allows a somewhat
larger FIR filter to be done within realtime constraints that stem
from a 64 sample block size.  See the file for constraints on the
number of taps allowed.  The left channel is filtered.  NOTE: With
the default BLOCK_SIZE of 64, you can do a 961 tap filter.  Change
BLOCK_SIZE in dsp_common.h to 384 to do a 3713 tap filter.

dsp_iir.c implements the IIR filter with coefficient from iir_coeffs.h
and applies it to the left channel.

dsp_none.c does not process the signal at all.  It just passes it
though.

dsp_make_sine.c generates sine waves, paying no attention to input.

dsp_mult.c multiplies one of the signals by the other.  You can see
the resulting frequency shifts on the left channel output.

dsp_detect.c uses the Goertzel algorithm to detect a specific tone on
the left channel.  specified by FREQ_TO_DETECT in the file.  It lights
the onboard LED when the tone is seen.

dsp_fft.c computes an FFT of the signal on the left channel and prints
the frequency of the top bin using the Pico 2's USB serial.  This is
not done in realtime and uses core 1 for the non-realtime processing.

dsp_pitch.c implements a pitch detector.  That is it tries to identify
the musical note seen on the left channel.  It cannot detect extremely
low notes or notes higher than ~ 6 KHz.  Pitch detection is a rather
hard problem due to harmonics and noise.  It is not done in realtime
and uses core 1 for the algorithm.

dsp_pitch_fft.c is basically the same as dsp_pitch.c but uses FFTs
to speed the computation rather like dsp_fft_filter.c

dsp_hilbert_fir.c is a time-domain FIR-based Hilbert transform.
Hilbert transforms create an output that is 90 degrees phased shifted
from the input (which may need a delay to align).  It is a kind of
all-pass filter.  Script gen_hilbert_fir.py generates coefficients
based on the desired number of taps.

dsp_hilbert_iir.c is an IIR based Hilbert transform.

dsp_qmix.c demonstrates a digital quadrature mixer using an IIR-based
Hilbert transform.

## Generating Filter Coefficients

Files gen_fir_firwin.py, gen_fir_firwin2.py, and gen_iir.py are python
scripts that use numpy and scipy to generate filter coefficients. You may
hve to install numpy and scipy.

    sudo apt install python3-scipy python3-numpy

All of them have a --help option.  They generate fir_coeffs.h (for fir
and fft_filter) or gen_iir.py (for iir).  Of them, gen_fir_firwin2.py
is the least obvious.  You give it a sequence of frequency gain pairs.
Like this:

    gen_fir_firwin2.py --taps 401 --points 0 1.0 9000 0.1 24414.0625 1

This generates IIR coefficients with a dramatically steep roll off:

    gen_iir.py --order 8 --fc 4000 --btype lp --ftype ellip

## PCM1808 Module Warning

I bought two PCM1808 modules via Amazon.  These are the modules with
the 5 prominent capacitors.  Only one of the two modules worked
properly.  The problem with the bad module was with component values
for the external anti-aliasing filter.  See the applications section
of the PCM1808 data sheet for more on this.  There is a circuit
diagram that shows simple RC low-pass filters being used for both the
right and left inputs.  The bad module used C = 100 nF and R = 10K
Ohms.  This gives a theoretical cut off frequency of around 150 Hz
(200 Hz measured).  This is obviously no good for a component that is
supposed to be dealing with much higher frequencies.

I made my bad module work by removing the offending four components
and bridging across the removed resistor pads.  Doing this with tiny
surface mount components is a hassle.

## Hardware Setup

The PIO heavily utilizes side-set instruction mappings. Therefore
SCK, BCK, and LRCK require contiguous sequential GPIO pin numbers.

### Hardware Device Settings

Both the PCM5102A and the PCM1808 are configured by pins tied low or high.
#### PCM5102A
    FLT: GND (normal latency filter)
    DEMP: GND (no de-ephasis)
    XMST: 3.3V (Don't hard mute)
    FMT: GND (use i2s)
    SCK: GND (Use internal PLL)
    VIN: 5V
#### PCM1808
    FMT: GND (use i2s)
    MD0: GND (slave mode)
    MD1: GND (slave mode)
    SCK: Clock generated by Pico2.
    Power: Needs both 3.3 and 5V.

### Pinout

Look at dsp_common.h.  In addition to pins related to the i2s devices
there are pins named with _DEBUG_.  These are used to measure times
and see if software is meeting realtime constraints.  There are two
for the RX and TX DMAs.  These toggle whenever a DMA completes and is
restarted.  There are also two for software, one for core 0 and one
for core 1.  The idea is that these are high when software is actively
processing a buffer.  In particular if the one for core 0 is high for
a time longer than a DMA, core 0 software is not meeting its realtime
constraint.  I use the ADALM2000's logic analyzer (digital) pins to
measure this.

Pin numbers can be set in the hardware macros inside
dsp_common.h. However, clocks 10, 11, and 12 must be sequential.

## Software Pipeline Execution Details

PIO generates all clock schedules explicitly using simple integer
dividers from a 150MHz core clock to yield an exactly 48828.125 Hz
sample stereo rate without jitter.  Yes, that's a weird sample rate,
and it is hard to change.

## Building

Install the PICO C SDK according to its instructions.  You can see
install_pico_sdk.txt for a summary of what I did, a manual install
without an IDE.

### For this program:

    make a directory someplace and cd to it
    git clone https://github.com/ARM-software/CMSIS-DSP.git
    git clone https://github.com/grughuhler/rpi_pico_related.git
    cd rpi_pico_related/pico2_dsp_skeleton
    # Be sure PICO_SDK_PATH is set to directory of pico-sdk
    mkdir build
    cd build
    cmake -DCMAKE_BUILD_TYPE=Release ..
    make

This will build all of the xxx.uf2 files.

Note: build failed on Fedora 43 with a compiler internal error (by
definition a bug in the compiler).

Tested OK on Ubuntu 24.04 LTS.  This will produce ".uf2" files which
you load onto the Pico2 using BOOTSEL via pressing the button while
powering on (see Pico docs) or using picotool,

    picotool load -f file.uf2 -x
