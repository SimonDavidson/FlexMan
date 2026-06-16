# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Simon Davidson, University of Manchester
# =============================================================================
# flexman_backend.emulate — bit-accurate model of the FlexMan datapath (as built)
#
# Author: Simon Davidson & Claude
# Created: 2026-06-09
# Last modified: 2026-06-16
#
# Pure-Python integer model of what the RTL actually computes — used to generate
# golden outputs for the closed-loop integration TB. Derived from and validated
# against the RTL (snnAcc/update_state_for_neuron.v, syn_curr_update.v,
# annAcc/ann_update_state_for_neuron.v) and the unit-TB goldens
# (snnAcc/tb_acc_snn_processor.v Test 1/2/3). Models the RTL AS BUILT, including
# the membrane-side bias placement (neuron_model.md) — the golden is what the
# hardware computes, not the training-framework intent.
#
# Per LIF timestep (update_state_for_neuron.v):
#   p_t      = sat32( v_prev + bias + i_t )          # membrane sum, saturating
#   spike    = (p_t >= theta)                        # signed compare, pre-decay
#   syn_next = (i_t * alpha) >> 32                    # stored alpha*i_t (Q0.32 decay)
#   v_next   = 0                if spike & reset-to-zero
#            = (p_t*beta>>32)-theta if spike & subtract-on-fire
#            = (p_t*beta) >> 32  otherwise            # stored beta*p_t
# The MAC (syn_curr_update.v) accumulates i_t = stored(alpha*i_{t-1}) + sum(w*a),
# 32-bit two's-complement wrap (no saturation at the accumulator).
# =============================================================================
from __future__ import annotations
from dataclasses import dataclass

DECAY_BITS = 32   # Q0.32 decay multipliers (syn_curr_decay_mult / pot_decay_mult)


def wrap_signed(x: int, bits: int = 32) -> int:
    """Two's-complement wrap to a signed `bits`-wide value (matches RTL truncation)."""
    m = 1 << bits
    x &= (m - 1)
    return x - m if x & (m >> 1) else x


def sat_signed(x: int, bits: int = 32) -> int:
    """Saturate to the signed `bits`-wide range (matches the membrane-sum clamp)."""
    hi = (1 << (bits - 1)) - 1
    lo = -(1 << (bits - 1))
    return hi if x > hi else lo if x < lo else x


def sat_unsigned(x: int, bits: int) -> int:
    """Saturate to the unsigned `bits`-wide range (annAcc output requant)."""
    hi = (1 << bits) - 1
    return 0 if x < 0 else hi if x > hi else x


def decay(value: int, mult: int, bits: int = DECAY_BITS) -> int:
    """Q0.`bits` decay: high word of signed(value) * unsigned(mult).

    Matches update_state_for_neuron.v: signed product, arithmetic right shift by
    `bits` (Python `>>` floors toward -inf == arithmetic shift). `mult` is the
    Q0.bits unsigned multiplier (e.g. 0x8000_0000 = 0.5 at bits=32).
    """
    return (value * mult) >> bits


# ---------------------------------------------------------------------------
# MAC (spike_processing / syn_curr_update): i_t = stored + sum(w * a)
# ---------------------------------------------------------------------------
def mac_accumulate(stored_syn: int, weights_row, acts, bits: int = 32) -> int:
    """i_t = stored_syn + Σ (signed weight * unsigned activation), `bits`-wide wrap.

    `weights_row` and `acts` are equal-length sequences of ints (weights signed,
    activations unsigned — spikes are 0/1). The accumulation wraps at `bits` (32 =
    the syn_curr width as built; widened only by precision studies).
    """
    acc = stored_syn
    for w, a in zip(weights_row, acts):
        acc = wrap_signed(acc + w * a, bits)
    return acc


# ---------------------------------------------------------------------------
# LIF / LI neuron step (update_state_for_neuron.v)
# ---------------------------------------------------------------------------
@dataclass
class LifResult:
    syn_next: int      # stored alpha*i_t (next timestep's MAC seed)
    v_next:   int      # stored beta*p_t (or reset) -> pot_mem contents
    membrane: int      # p_t, the pre-decay membrane sum (= u_t)
    spike:    int      # 0/1


def lif_step(i_t: int, v_prev: int, bias: int, alpha: int, beta: int,
             theta: int, sub_on_fire: bool = False) -> LifResult:
    """One LIF timestep for one neuron. `i_t` is the post-MAC synaptic current
    (already includes the stored alpha*i_{t-1}); `v_prev` is the stored beta*p_{t-1}."""
    p_t = sat_signed(v_prev + bias + i_t)
    spike = 1 if p_t >= theta else 0
    syn_next = decay(i_t, alpha)
    pot_decayed = decay(p_t, beta)
    if spike:
        v_next = (pot_decayed - theta) if sub_on_fire else 0
    else:
        v_next = pot_decayed
    return LifResult(syn_next=syn_next, v_next=v_next, membrane=p_t, spike=spike)


def li_step(i_t: int, v_prev: int, bias: int, alpha: int, beta: int) -> LifResult:
    """Leaky-integrator readout: threshold never reached (theta = max int32), so
    it never fires and pot_mem holds beta*p_t. The host reads `v_next`."""
    return lif_step(i_t, v_prev, bias, alpha, beta,
                    theta=0x7FFFFFFF, sub_on_fire=False)


# ---------------------------------------------------------------------------
# annAcc feature-extraction neuron (ann_update_state_for_neuron.v + requant §4b)
# ---------------------------------------------------------------------------
def fe_neuron(acc: int, bin_point_syn_curr: int, np_out_bin_point: int,
              out_bits: int, mode: str = "abs") -> int:
    """Feedforward annAcc output activation: ABS|RELU of the accumulator, then
    round -> shift(bin_point_syn_curr - np_out_bin_point) -> unsigned-saturate.

    `mode` in {"abs","relu"}. np_out_bin_point=0 disables requant (legacy low bits).
    """
    if mode == "relu":
        a = acc if acc > 0 else 0
    elif mode == "abs":
        a = -acc if acc < 0 else acc
    else:
        raise ValueError(f"unsupported FE mode {mode!r}")
    if np_out_bin_point == 0:
        return a & ((1 << out_bits) - 1)          # legacy: low bits
    shift = bin_point_syn_curr - np_out_bin_point
    rounded = a + (1 << (shift - 1)) if shift > 0 else a   # round-half-up
    shifted = rounded >> shift if shift > 0 else rounded << (-shift)
    return sat_unsigned(shifted, out_bits)


# =============================================================================
# Gated-nonlinearity + element-wise gated-update primitives.
#
# Model the annAcc LUT activation path and the Hadamard datapath. ADDITIVE and
# unused by the existing spiking-network path — nothing above changes, so the
# bit-validated snnAcc unit-TB goldens are untouched. They model the annAcc RTL
# with three default-off datapath options enabled:
#   (1) LUT input:  scaled + centred + saturated index (not raw low bits)
#   (2) LUT output: signed path for tanh (sigmoid stays unsigned)
#   (3) signed-activation MAC mode: activation treated as signed (sign-extended),
#       so a signed activation can feed the MAC. mac_accumulate() already does a
#       signed multiply, so it models this mode directly (see mac_signed()).
# =============================================================================
import math


def _q_round(value: float, frac: int) -> int:
    """Quantise a real value to `frac` fractional bits, round-half-up."""
    return int(math.floor(value * (1 << frac) + 0.5))


def make_lut(func, in_frac: int, out_frac: int, idx_bits: int, signed: bool):
    """Build a 2**idx_bits-entry activation table for `func` (e.g. math.tanh).

    Entry k corresponds to input value (k - MID)/2**in_frac, MID = 2**(idx_bits-1),
    so index MID is input 0 and the covered domain is [-MID, MID)/2**in_frac
    (e.g. idx_bits=8, in_frac=4 -> domain +/-8 over 256 entries). Outputs are
    fixed-point at `out_frac` fractional bits; sigmoid is unsigned [0,1], tanh is
    signed [-1,1] (the signed-output path).
    """
    n = 1 << idx_bits
    mid = n >> 1
    # Range of the quantised output: tanh in [-1,1] -> +/-2**out_frac needs a
    # signed (out_frac+2)-bit clamp; sigmoid in [0,1] -> [0,2**out_frac] needs an
    # unsigned (out_frac+1)-bit clamp. The clamp only guards the table extremes.
    out_bits = (out_frac + 2) if signed else (out_frac + 1)
    tbl = []
    for k in range(n):
        x = (k - mid) / (1 << in_frac)
        q = _q_round(func(x), out_frac)
        tbl.append(sat_signed(q, out_bits) if signed else sat_unsigned(q, out_bits))
    return tbl


@dataclass
class Lut:
    """A loaded annAcc activation table + its fixed-point metadata."""
    table: list
    in_frac: int        # table input fractional bits (domain scaling)
    out_frac: int       # table output fractional bits (== activation bin point)
    idx_bits: int
    signed: bool        # True for tanh (signed out), False for sigmoid

    @classmethod
    def build(cls, func, in_frac, out_frac, idx_bits, signed):
        return cls(make_lut(func, in_frac, out_frac, idx_bits, signed),
                   in_frac, out_frac, idx_bits, signed)

    def lookup(self, acc: int, bin_point_syn_curr: int) -> int:
        """Indexed lookup: scale the signed accumulator to the table's input
        fractional bits, centre on MID, saturate to the table range, return the
        fixed-point activation (Q`out_frac`; signed for tanh)."""
        n = 1 << self.idx_bits
        mid = n >> 1
        shift = bin_point_syn_curr - self.in_frac
        scaled = (acc >> shift) if shift >= 0 else (acc << (-shift))
        idx = scaled + mid
        idx = 0 if idx < 0 else (n - 1) if idx > n - 1 else idx   # saturate index
        return self.table[idx]


def mac_signed(stored_syn: int, weights_row, acts, bits: int = 32) -> int:
    """Signed-activation MAC: identical to mac_accumulate (the Python multiply is
    already signed), named to make the signed-activation annAcc mode explicit at
    the call site — `acts` may be negative (a signed activation)."""
    return mac_accumulate(stored_syn, weights_row, acts, bits)


def hadamard(a: int, b: int, z: int, z_frac: int, mode: int = 0, r_prev: int = 0) -> int:
    """One Hadamard element: R = round(Z*(A-B) / 2^z_frac) + B + mode*R_prev.

    Z is an unsigned gate in Q`z_frac` (representing [0,1]); A, B share one
    fractional scale and the result is at that same scale. Models hu_compute.v
    ST_TRUNCATE, which applies ROUND-HALF-UP on the down-shift (add 2^(z_frac-1),
    then arithmetic floor) — not a plain floor — so the emulator is bit-exact with
    the RTL Hadamard. Covers both gated-update forms: a plain product Z*A [B=0]
    and a convex combination Z*(A-B)+B.
    """
    prod = z * (a - b)
    if z_frac > 0:
        shifted = (prod + (1 << (z_frac - 1))) >> z_frac   # round-half-up (hu_compute)
    else:
        shifted = prod
    return shifted + b + (r_prev if mode else 0)
