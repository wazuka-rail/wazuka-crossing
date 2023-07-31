import math
import random
import struct

class Tone:
    def __init__(self, amps):
        self.amps = amps
        self.phs = [0, 0, 0, 0, 0]

    def wave16(self, phs):
        rs = [math.radians(p) for p in phs]
        amps = self.amps
        return [
            amps[0] * math.sin(1.0 * math.pi * t / 8 + rs[0]) + # fundamental harmonic
            amps[1] * math.sin(2.0 * math.pi * t / 8 + rs[1]) + # 2nd harmonic
            amps[2] * math.sin(3.0 * math.pi * t / 8 + rs[2]) + # 3rd harmonic
            amps[3] * math.sin(4.0 * math.pi * t / 8 + rs[3]) + # 4th harmonic
            amps[4] * math.sin(5.0 * math.pi * t / 8 + rs[4])   # 5th harmonic
            for t in range(0, 16)
        ]

    def pp(self, wave):
        return max(wave) - min(wave)

    def optimize(self, phs):
        minpp = self.pp(self.wave16(phs))
        for i in range(10000):
            nphs = [p + int(random.uniform(-10, 10)) for p in phs]
            npp = self.pp(self.wave16(nphs))
            if npp < minpp:
                print(npp)
                phs = nphs
                minpp = npp
        self.phs = phs
        print(phs)

    def get_normalized_waves(self):
        base = self.wave16(self.phs)
        normalized = [(v - min(base)) / self.pp(base) for v in base]
        return [[int(round(198 * 0.85 ** k * v + 1)) for v in normalized] for k in range(16)]

    def write_table(self, filename):
        ws = self.get_normalized_waves()
        with open(filename,'w') as f:
            for k in range(16):
                w = ws[k]
                f.write(f"; Wave table (envelope {k})\n")
                for v in w:
                    f.write(f"    dw          {v}\n")

tone1 = Tone([1.0, 0.7, 0.4, 0.1, 0.0])
tone2 = Tone([1.0, 0.0, 0.9, 0.0, 0.4])

tone1.optimize([-103, -113, 56, 29, 0])
tone2.optimize([-77, 0, 79, 0, -10])

waves1 = tone1.get_normalized_waves()
waves2 = tone2.get_normalized_waves()

tone1.write_table('../crossing.X/wavtable1.inc')
tone2.write_table('../crossing.X/wavtable2.inc')

class Osc:
    fs = 20000 # Hz

    def __init__(self, waves, f1, f2, fm, att1=0, att2=0):
        self.osc1_delta = int(round(16 * 0x100 * f1 / self.fs))
        self.osc2_delta = int(round(16 * 0x100 * f2 / self.fs))
        self.envc = int(round(self.fs / fm / 16))
        self.waves = waves
        self.osc1_acc = 0
        self.osc2_acc = 0
        self.att1 = att1
        self.att2 = att2
        self.count = 0
        self.envelope = 0
        self.data = []

    def get_wave(self, acc, env):
        return self.waves[env][(acc >> 8) & 15]

    def step(self):
        self.osc1_acc += self.osc1_delta
        self.osc2_acc += self.osc2_delta

        osc1_amp = self.get_wave(self.osc1_acc, min(self.envelope + self.att1, 15))
        osc2_amp = self.get_wave(self.osc2_acc, min(self.envelope + self.att2, 15))

        mixed = (osc1_amp + osc2_amp) >> 1

        self.data.append(mixed)

        self.count += 1
        if self.count % self.envc == 0:
            self.envelope = (self.envelope + 1) % 16

    def write_wav(self, filename):
        n = len(self.data)
        header = (
            b"RIFF", #header
            11 * 4 + n - 8, #size of riff
            b"WAVE",
            b"fmt ",
            16, #size of fmt
            1, #format id
            1, #ch
            self.fs, #sampling rate
            self.fs * 1 * 1, #byte / sec
            1, #block size
            8, #bit
            b"data",
            n
        )
        with open(filename, 'wb') as f:
            f.write(struct.pack('<4sL4s4sLHHLLHH4sL', *header))
            f.write(bytearray(self.data))

n = Osc.fs * 4

osc = Osc(waves1, 700, 750, 130 / 60) # JR
for i in range(n):
    osc.step()
osc.write_wav('snd_700_750_130.wav')

osc = Osc(waves2, 600, 650, 100 / 60, att2 = 4) # Tobu
for i in range(n):
    osc.step()
osc.write_wav('snd_600_650_100.wav')

osc = Osc(waves1, 550, 650, 117 / 60, att1 = 4) # Tokyu
for i in range(n):
    osc.step()
osc.write_wav('snd_550_650_117.wav')

osc = Osc(waves1, 523, 660, 100 / 60, att1 = 6) # Seibu
for i in range(n):
    osc.step()
osc.write_wav('snd_523_660_100.wav')

osc = Osc(waves2, 450, 550, 117 / 60) # Odakyu
for i in range(n):
    osc.step()
osc.write_wav('snd_450_550_117.wav')
