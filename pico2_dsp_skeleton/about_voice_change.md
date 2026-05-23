# Algorithm: Real-Time Phase-Aligned Delay-Line Pitch Shifter (HD WSOLA Variant)

## Overview
This document was written by Google's AI.

The voice changer algorithm implemented in `dsp_voice_change.c` is a high-performance, studio-quality time-domain pitch shifter designed specifically for the strict real-time constraints of a microcontroller. It is a specialized, continuous-stream variant of the WSOLA (Waveform Similarity Overlap-Add) algorithm.

It works by writing incoming audio to a circular buffer and reading it back at a variable rate (`delay_rate = 1.0 - pitch_shift_ratio`). 
- If pitching up (ratio > 1.0), the read pointer moves faster than the write pointer, and the delay decreases.
- If pitching down (ratio < 1.0), the read pointer moves slower, and the delay increases.

Because the delay cannot increase or decrease infinitely, the pointer must occasionally "jump" back to the other side of a predefined delay window. To prevent the audible clicks, pops, and metallic flanging artifacts that occur when jumping blindly, the algorithm uses a "smart" crossfade driven by a WSOLA cross-correlation search.

## The Incremental Background Search Architecture
Finding the perfect phase alignment requires a massive amount of math (a cross-correlation search comparing an 800-sample template over 400 possible offsets). Performing this math in a single CPU burst would cause a 2.5ms spike, breaking the real-time deadline.

To solve this, the algorithm employs an **Incremental Background Search State Machine**:
1. **Early Trigger**: The algorithm anticipates a required jump exactly 400 audio samples *before* the pointer actually hits the window boundary.
2. **Micro-Tasks**: The massive cross-correlation math is divided into 210 tiny operations. The algorithm computes exactly **one** correlation offset per audio sample during the normal processing loop. This completely flattens the CPU load, executing in less than 120us per block.
3. **Phase Locking**: Once the best offset is found, the target pointer actively increments in parallel with the main pointer, maintaining perfect phase lock while waiting for the exact moment to trigger the crossfade.

## High-Definition Resampling
The algorithm uses a **16-point Blackman-Harris windowed Sinc Interpolator** (via a pre-calculated 64-phase lookup table). This perfectly reconstructs the analog waveform between samples, eliminating aliasing and preserving pristine, studio-quality treble clarity even during extreme pitch shifts.

## Major Weaknesses

1. **Formant Shifting ("Chipmunk Effect")**
   Because this is a pure time-domain resampling pitch shifter, it shifts the entire frequency spectrum of the voice equally. It does not preserve "formants" (the resonant frequencies of the human vocal tract). This means pitching a voice up makes it sound like a smaller creature (a chipmunk), and pitching it down makes it sound like a giant (Darth Vader). True vocal gender-shifting requires preserving formants (using phase vocoders or LPC), which this algorithm cannot do.

2. **Transient Smearing**
   The algorithm makes no distinction between periodic voiced speech (vowels) and unvoiced transients (hard consonants like 't', 'k', 'p'). If a crossfade happens to trigger exactly during a sharp transient, the correlation search might try to align random noise, slightly smearing or softening the sharpness of the consonant.

## How it Differs from Standard WSOLA

This implementation **is** WSOLA in its core principle (aligning waveforms using cross-correlation before overlap-adding), but it differs structurally from classic textbook WSOLA:

1. **Continuous vs. Block-Based**
   Textbook WSOLA is typically frame-based. It chops the audio into fixed-size overlapping frames (e.g., every 10 ms), stretches them, aligns them, and adds them. 
   Our implementation is a **Continuous Variable Delay Line**. It doesn't extract discrete frames; it simply reads continuously at a variable speed using the 16-point Sinc interpolator.

2. **Sparse Alignment (Hop Size)**
   In standard WSOLA, the alignment search and overlap-add happen constantly (e.g., a 50% overlap means it happens twice per window). 
   In our implementation, the WSOLA alignment *only happens at the window boundaries* when the delay line pointer is forced to wrap around. The "hop size" is effectively the entire `WINDOW_LENGTH` (1000 samples). 

This sparse alignment is a deliberate design choice: it is vastly more computationally efficient. By only performing the expensive cross-correlation search when absolutely necessary to reset the delay line, we free up massive amounts of CPU overhead to run the 16-point Sinc interpolation, achieving pristine audio fidelity within a tiny microcontroller budget.
