# SPDX-License-Identifier: MIT
# Author: Simon Davidson & Claude
# Created: 2026-06-09
# Last modified: 2026-06-09
# Validates flexman_backend.emulate against the RTL-proven unit-TB goldens in
# snnAcc/tb_acc_snn_processor.v (Test 1/2/3). No torch, no pytest required.
from flexman_backend import emulate as E

HALF = 0x8000_0000   # 0.5 in Q0.32


def test_decay_matches_rtl():
    assert E.decay(20, HALF) == 10            # 20 * 0.5
    assert E.decay(-20, HALF) == -10          # signed arithmetic shift (floor)
    assert E.decay(100, 0xD2E7CC00) == 82     # recurrent alpha=0.823849 -> floor(82.38)
    assert E.decay(0, HALF) == 0


def test_mac_accumulate():
    assert E.mac_accumulate(0, [10, 10], [1, 1]) == 20          # Test 1/2 setup
    assert E.mac_accumulate(5, [3, -2, 4], [1, 0, 1]) == 12      # spike gating (0/1 acts)


def test_smoke_test1_no_spike():
    # i_t=20, v_prev=0, bias=0, alpha=beta=0.5, theta=50 -> syn=10, pot=10, no spike
    r = E.lif_step(i_t=20, v_prev=0, bias=0, alpha=HALF, beta=HALF, theta=50)
    assert r.membrane == 20 and r.spike == 0
    assert r.syn_next == 10        # syn_curr_sram = 20*0.5
    assert r.v_next == 10          # pot_sram     = 20*0.5


def test_smoke_test2_spike_reset():
    # threshold=5 <= 20 -> spike, pot resets to 0, syn decays to 10
    r = E.lif_step(i_t=20, v_prev=0, bias=0, alpha=HALF, beta=HALF, theta=5)
    assert r.spike == 1
    assert r.syn_next == 10
    assert r.v_next == 0           # reset-to-zero on fire


def test_smoke_test3_nonuniform_no_spike():
    # neuron 0: i=20 -> syn=10,pot=10 ; neuron 1: i=10 -> syn=5,pot=5 ; theta=50
    r0 = E.lif_step(i_t=20, v_prev=0, bias=0, alpha=HALF, beta=HALF, theta=50)
    r1 = E.lif_step(i_t=10, v_prev=0, bias=0, alpha=HALF, beta=HALF, theta=50)
    assert (r0.syn_next, r0.v_next, r0.spike) == (10, 10, 0)
    assert (r1.syn_next, r1.v_next, r1.spike) == (5, 5, 0)


def test_subtract_on_fire():
    # spike with subtract-on-fire: v_next = beta*p_t - theta
    r = E.lif_step(i_t=20, v_prev=0, bias=0, alpha=HALF, beta=HALF, theta=5,
                   sub_on_fire=True)
    assert r.spike == 1 and r.v_next == (10 - 5)   # beta*20=10, minus theta


def test_membrane_bias_and_saturation():
    # bias added at the membrane; sum saturates to signed 32-bit
    r = E.lif_step(i_t=10, v_prev=4, bias=6, alpha=HALF, beta=HALF, theta=100)
    assert r.membrane == 20                        # 4 + 6 + 10
    big = E.lif_step(i_t=2**31 - 1, v_prev=2**31 - 1, bias=0,
                     alpha=HALF, beta=HALF, theta=0)
    assert big.membrane == 2**31 - 1               # saturated, not wrapped


def test_li_readout_never_fires():
    r = E.li_step(i_t=10_000_000, v_prev=0, bias=0, alpha=HALF, beta=HALF)
    assert r.spike == 0
    assert r.v_next == 5_000_000                   # pot_mem = beta*p_t


def test_fe_abs_requant():
    # bin_point_syn_curr=16, np_out_bin_point=4 -> shift 12, round-half-up, 8-bit
    assert E.fe_neuron(4096, 16, 4, 8, "abs") == 1       # 4096*2^-16 -> 1 at 2^-4
    assert E.fe_neuron(-8192, 16, 4, 8, "abs") == 2      # ABS then requant
    assert E.fe_neuron(10**9, 16, 4, 8, "abs") == 255    # unsigned saturate
    assert E.fe_neuron(-100, 16, 4, 8, "relu") == 0      # RELU clamps negative
