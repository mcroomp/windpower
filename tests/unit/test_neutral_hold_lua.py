from simulation.rawes_lua_harness import RawesLua


def test_mode_none_holds_disarmed_acro_inputs_at_neutral():
    sim = RawesLua(
        mode=0,
        H_COL_MIN=1342,
        H_COL_MAX=1657,
        RC1_MIN=1100,
        RC1_TRIM=1500,
        RC1_MAX=1900,
        RC2_MIN=1100,
        RC2_TRIM=1500,
        RC2_MAX=1900,
        RC3_MIN=1100,
        RC3_MAX=1900,
        RC3_DZ=10,
    )

    sim.tick()

    assert (sim.ch_out[1], sim.ch_out[2], sim.ch_out[3]) == (1500, 1500, 1506)
    assert sim.fns.neutral_hold_active()


def test_neutral_collective_accounts_for_rc_reversal():
    sim = RawesLua(
        mode=0,
        H_COL_MIN=1342,
        H_COL_MAX=1657,
        RC3_MIN=1100,
        RC3_MAX=1900,
        RC3_DZ=10,
        RC3_REVERSED=1,
    )

    sim.tick()

    assert sim.ch_out[3] == 1494


def test_neutral_hold_clears_when_armed_or_mode_becomes_active():
    sim = RawesLua(mode=0)
    sim.tick()
    assert sim.fns.neutral_hold_active()

    sim.armed = True
    sim.tick()
    assert (sim.ch_out[1], sim.ch_out[2], sim.ch_out[3]) == (0, 0, 0)
    assert not sim.fns.neutral_hold_active()

    sim.armed = False
    sim.tick()
    assert (sim.ch_out[1], sim.ch_out[2], sim.ch_out[3]) == (1500, 1500, 1505)

    sim.set_param("mode", 1)
    sim.tick()
    assert (sim.ch_out[1], sim.ch_out[2], sim.ch_out[3]) == (0, 0, 0)
    assert not sim.fns.neutral_hold_active()


def test_interlock_waits_for_acro_target_reset_and_clears_on_disarm():
    sim = RawesLua(mode=0)
    sim.tick()
    assert sim.ch_out[8] == 1000

    sim.armed = True
    sim.run(0.49)
    assert sim.ch_out[8] == 1000

    sim.run(0.02)
    assert sim.ch_out[8] == 2000

    sim.armed = False
    sim.tick()
    assert sim.ch_out[8] == 1000
