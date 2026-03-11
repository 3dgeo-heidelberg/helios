import pytest

from helios import HookEndOfLegPolicy, HookPayload, HookPoint, Survey, SurveyHook
from helios.settings import ExecutionSettings, OutputFormat


def _make_short_survey(scanner, tripod, scene, legs=1):
    survey = Survey(scanner=scanner, platform=tripod, scene=scene)
    for i in range(legs):
        survey.add_leg(
            x=float(i),
            y=0.0,
            z=0.0,
            pulse_frequency=1000,
            scan_angle="20 deg",
            head_rotation="10 deg/s",
            rotation_start_angle="0 deg",
            rotation_stop_angle="2 deg",
            scan_frequency=120,
            trajectory_time_interval=0.01,
        )
    return survey


def test_leg_start_and_leg_end_order(tls_scanner, tripod, scene):
    survey = _make_short_survey(tls_scanner, tripod, scene, legs=2)
    events = []

    def on_start(ctx, points=None, trajectories=None):
        events.append(("start", ctx.leg_index))

    def on_end(ctx, points=None, trajectories=None):
        events.append(("end", ctx.leg_index))

    survey.run(
        format=OutputFormat.NPY,
        execution_settings=ExecutionSettings(num_threads=1),
        callbacks=(
            SurveyHook(point=HookPoint.LEG_START, callback=on_start),
            SurveyHook(point=HookPoint.LEG_END, callback=on_end),
        ),
    )

    assert events == [("start", 0), ("end", 0), ("start", 1), ("end", 1)]


def test_sim_time_once_fires_once(tls_scanner, tripod, scene):
    survey = _make_short_survey(tls_scanner, tripod, scene)
    fired = []

    def on_once(ctx, points=None, trajectories=None):
        fired.append(ctx.sim_time_s)

    survey.run(
        format=OutputFormat.NPY,
        execution_settings=ExecutionSettings(num_threads=1),
        callbacks=(
            SurveyHook(
                point=HookPoint.SIM_TIME_ONCE,
                callback=on_once,
                sim_time_s=0.02,
            ),
        ),
    )

    assert len(fired) == 1


def test_sim_time_periodic_skips_missed_intervals(tls_scanner, tripod, scene):
    survey = _make_short_survey(tls_scanner, tripod, scene)
    sim_times = []

    def on_periodic(ctx, points=None, trajectories=None):
        sim_times.append(ctx.sim_time_s)

    survey.run(
        format=OutputFormat.NPY,
        execution_settings=ExecutionSettings(num_threads=1),
        callbacks=(
            SurveyHook(
                point=HookPoint.SIM_TIME_PERIODIC,
                callback=on_periodic,
                sim_time_s=0.0,
                period_s=0.0001,
            ),
        ),
    )

    assert len(sim_times) > 0
    theoretical_nominal = int(sim_times[-1] / 0.0001) + 1
    assert len(sim_times) < theoretical_nominal / 2


def test_payload_metadata_only(tls_scanner, tripod, scene):
    survey = _make_short_survey(tls_scanner, tripod, scene)
    seen = []

    def on_end(ctx, points=None, trajectories=None):
        seen.append((points is None, trajectories is None, ctx.payload_points))

    survey.run(
        format=OutputFormat.NPY,
        execution_settings=ExecutionSettings(num_threads=1),
        callbacks=(
            SurveyHook(
                point=HookPoint.LEG_END,
                callback=on_end,
                payload=HookPayload.METADATA_ONLY,
            ),
        ),
    )

    assert seen
    assert all(
        p_none and t_none and payload_points == 0
        for p_none, t_none, payload_points in seen
    )


def test_payload_since_last_non_overlapping_points_and_trajectories(
    tls_scanner, tripod, scene
):
    survey = _make_short_survey(tls_scanner, tripod, scene)
    payload_point_sizes = []
    payload_traj_sizes = []

    def on_periodic(ctx, points=None, trajectories=None):
        payload_point_sizes.append(0 if points is None else len(points))
        payload_traj_sizes.append(0 if trajectories is None else len(trajectories))

    measurements, trajectories = survey.run(
        format=OutputFormat.NPY,
        execution_settings=ExecutionSettings(num_threads=1),
        callbacks=(
            SurveyHook(
                point=HookPoint.SIM_TIME_PERIODIC,
                callback=on_periodic,
                payload=HookPayload.SINCE_LAST,
                sim_time_s=0.0,
                period_s=0.0001,
            ),
        ),
    )

    assert sum(payload_point_sizes) == len(measurements)
    assert sum(payload_traj_sizes) == len(trajectories)


def test_payload_since_last_periodic_tail_flushes_remaining_points(tls_survey):
    payload_point_sizes = []

    def on_periodic(ctx, points=None, trajectories=None):
        payload_point_sizes.append(0 if points is None else len(points))

    measurements, _ = tls_survey.run(
        format=OutputFormat.NPY,
        execution_settings=ExecutionSettings(num_threads=1),
        callbacks=(
            SurveyHook(
                point=HookPoint.SIM_TIME_PERIODIC,
                callback=on_periodic,
                payload=HookPayload.SINCE_LAST,
                sim_time_s=1000.0,
                period_s=1000.0,
                end_of_leg_policy=HookEndOfLegPolicy.FLUSH,
            ),
        ),
    )

    assert len(measurements) > 0
    assert len(payload_point_sizes) == 1
    assert sum(payload_point_sizes) == len(measurements)


def test_periodic_end_of_leg_policy_none_disables_tail_flush(tls_survey):
    payload_point_sizes = []

    def on_periodic(ctx, points=None, trajectories=None):
        payload_point_sizes.append(0 if points is None else len(points))

    measurements, _ = tls_survey.run(
        format=OutputFormat.NPY,
        execution_settings=ExecutionSettings(num_threads=1),
        callbacks=(
            SurveyHook(
                point=HookPoint.SIM_TIME_PERIODIC,
                callback=on_periodic,
                payload=HookPayload.SINCE_LAST,
                sim_time_s=1000.0,
                period_s=1000.0,
                end_of_leg_policy=HookEndOfLegPolicy.NONE,
            ),
        ),
    )

    assert len(measurements) > 0
    assert len(payload_point_sizes) == 0


def test_periodic_end_of_leg_policy_flush_and_reset_resets_timer(
    tls_scanner, tripod, scene
):
    survey = Survey(scanner=tls_scanner, platform=tripod, scene=scene)
    for x in (0.0, 1.0):
        survey.add_leg(
            x=x,
            y=0.0,
            z=0.0,
            pulse_frequency=2000,
            scan_angle="20 deg",
            head_rotation="10 deg/s",
            rotation_start_angle="0 deg",
            rotation_stop_angle="10 deg",
            scan_frequency=120,
            trajectory_time_interval=0.01,
        )

    callbacks = []

    def on_periodic(ctx, points=None, trajectories=None):
        callbacks.append((ctx.leg_index, ctx.scheduled_time_s, ctx.sim_time_s))

    survey.run(
        format=OutputFormat.NPY,
        execution_settings=ExecutionSettings(num_threads=1),
        callbacks=(
            SurveyHook(
                point=HookPoint.SIM_TIME_PERIODIC,
                callback=on_periodic,
                payload=HookPayload.SINCE_LAST,
                sim_time_s=0.0,
                period_s=1.5,
                end_of_leg_policy=HookEndOfLegPolicy.FLUSH_AND_RESET,
            ),
        ),
    )

    leg1_callbacks = [c for c in callbacks if c[0] == 1]
    assert len(leg1_callbacks) == 1
    assert leg1_callbacks[0][1] == pytest.approx(leg1_callbacks[0][2])


def test_payload_all_points_and_trajectories_grows(tls_scanner, tripod, scene):
    survey = _make_short_survey(tls_scanner, tripod, scene)
    point_sizes = []
    traj_sizes = []

    def on_periodic(ctx, points=None, trajectories=None):
        point_sizes.append(0 if points is None else len(points))
        traj_sizes.append(0 if trajectories is None else len(trajectories))

    measurements, trajectories = survey.run(
        format=OutputFormat.NPY,
        execution_settings=ExecutionSettings(num_threads=1),
        callbacks=(
            SurveyHook(
                point=HookPoint.SIM_TIME_PERIODIC,
                callback=on_periodic,
                payload=HookPayload.ALL_POINTS,
                sim_time_s=0.0,
                period_s=0.01,
            ),
        ),
    )

    assert point_sizes
    assert traj_sizes
    assert all(x <= y for x, y in zip(point_sizes, point_sizes[1:]))
    assert all(x <= y for x, y in zip(traj_sizes, traj_sizes[1:]))
    assert point_sizes[-1] == len(measurements)
    assert traj_sizes[-1] == len(trajectories)


def test_barrier_true_includes_pending_points(tls_scanner, tripod, scene):
    survey = _make_short_survey(tls_scanner, tripod, scene)
    payload_sizes = []

    def on_once(ctx, points=None, trajectories=None):
        payload_sizes.append(0 if points is None else len(points))

    survey.run(
        format=OutputFormat.NPY,
        execution_settings=ExecutionSettings(num_threads=4),
        callbacks=(
            SurveyHook(
                point=HookPoint.SIM_TIME_ONCE,
                callback=on_once,
                payload=HookPayload.SINCE_LAST,
                barrier=True,
                sim_time_s=0.05,
            ),
        ),
    )

    assert payload_sizes and payload_sizes[0] > 0


def test_barrier_true_includes_pending_trajectories(tls_scanner, tripod, scene):
    survey = _make_short_survey(tls_scanner, tripod, scene)
    payload_sizes = []

    def on_once(ctx, points=None, trajectories=None):
        payload_sizes.append(0 if trajectories is None else len(trajectories))

    survey.run(
        format=OutputFormat.NPY,
        execution_settings=ExecutionSettings(num_threads=4),
        callbacks=(
            SurveyHook(
                point=HookPoint.SIM_TIME_ONCE,
                callback=on_once,
                payload=HookPayload.SINCE_LAST,
                barrier=True,
                sim_time_s=0.05,
            ),
        ),
    )

    assert payload_sizes and payload_sizes[0] > 0


def test_callback_exception_stops_and_propagates(tls_scanner, tripod, scene):
    survey = _make_short_survey(tls_scanner, tripod, scene)

    def on_once(ctx, points=None, trajectories=None):
        raise RuntimeError("callback boom")

    with pytest.raises(RuntimeError, match="callback boom"):
        survey.run(
            format=OutputFormat.NPY,
            execution_settings=ExecutionSettings(num_threads=1),
            callbacks=(
                SurveyHook(
                    point=HookPoint.SIM_TIME_ONCE,
                    callback=on_once,
                    sim_time_s=0.01,
                ),
            ),
        )
