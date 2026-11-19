"""The plug-in transformers of the chris stack (the plug-in contract: `chris_integrator.mixer.plugin`).

    estimator        vehicle, GNSS, road-model pose, map match, radars, cameras' objects -> `ego_state`, `tracks`, `lead`
    lanes            `road` + `ego_state` (+ `map` prior) -> `lanes`
    egolane          `lanes` + `map` -> ego lane estimate (wiring: `egolane`)
    roadgeom         `lanes` + `map` -> fused road geometry (wiring: `roadgeom`, shown as `view.road`)
    ego_light        a camera's light detections + path + lanes + map -> `lights` (the ego light selected)
    sign_context     a camera's sign detections + lead + lights -> `signs` (context checks), live sign tracks
    rules            `signs`, `env`, `road`, `ego_state` -> `speed_limit`, `rules`
    fused            map + camera decisions -> `fused`
    view             everything above -> `view` (the display channel)
    ride_watch       `camera` -> a new drive resets every plug-in

`chris_integrator.stack.build()` wires them; any of them can be left out, replaced or run alone."""
