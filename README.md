# TDXDepthCamMerger

A TouchDesigner component that merges point clouds from several depth cameras into one coordinate
space. It estimates the rigid transform between cameras with [Open3D](http://www.open3d.org/),
using FPFH features plus RANSAC for the global alignment and ICP for the refinement.

Version 0.4.0. Built and tested on TouchDesigner **2025.33070**.

Supported sources:

- Kinect Azure
- Orbbec
- ZED
- any pair of TOPs you supply yourself (a point cloud TOP and a colour TOP), so recorded clouds,
  other sensors and synthetic data all work

The cameras come in as TOPs, because that is what their TouchDesigner operators produce. Inside the
component they become points: the merged cloud you get back is a POP.

## If you are coming from 0.0.3, read this first

0.4.0 is a breaking change, and the most important part is the install. It follows 0.0.3
directly: 0.2.0 and 0.3.0 were development versions and were never released, and the version
jumped to 0.4.0 so it cannot be misread as 0.0.3.

**Do not put Open3D inside TouchDesigner.** The old README told you to copy it into
`TouchDesigner/bin/Lib/site-packages`, or to pip install it into TD's Python. On current builds
that hard-crashes the process the moment the component imports it. Not an exception you can catch,
the whole of TouchDesigner disappears. Open3D ships its own OpenMP and TBB next to the ones
TouchDesigner already loaded, and the two do not coexist.

So the component runs the registration **outside** TouchDesigner. You point the component at any Python
that has Open3D installed, it dumps the clouds to disk, runs a worker process, and reads back a
4x4 matrix. TouchDesigner itself only ever imports numpy. A nice side effect is that the component
no longer cares which Python TouchDesigner ships.

Parameters that changed:

| 0.0.3 | 0.4.0 |
|---|---|
| `Gather kinects` | `Gather devices` |
| `Use player for calibration` | `Use mask for calibration` |
| `Specify pair x` / `Specify pair y` (XYZW) | `Specify pair`, now two plain integer fields |
| `Current pair`, `Number of pairs`, `As intermediary` | gone, replaced by `Reference device` and the calibration table |
| (none) | `Device type`, `Python exe`, `Check worker`, `Voxel size`, `Refine voxel`, `Max range`, `Use coloured ICP`, `RANSAC seed`, `Preset matrix DAT`, `Rebuild chain`, `Reset calibration`, and the `Last ...` readouts |

The [walkthrough video](https://vimeo.com/501525725) still shows the 0.0.3 interface. The idea is
the same, the parameter names are not.

## Try it without a camera

Open `DemoTDXDepthCamMerger.toe`. It generates three synthetic depth cameras looking at one scene
from different places, coloured red, green and blue, and starts misaligned. Set the Python exe as
below, pulse Gather devices, then calibrate pair 1 and 2 followed by pair 2 and 3, and watch the
clouds come together. The START_HERE DAT in the project has the same steps.

## Installation

1. Drop `TDXDepthCamMerger.tox` into your project.

2. Get a Python 3.11 or newer with Open3D in it. Anything works as long as it is not TouchDesigner's
   own interpreter. Conda is the least painful:

   ```
   conda create -n td python=3.11
   conda activate td
   pip install open3d numpy
   ```

   Or with plain pip, in a virtualenv:

   ```
   python -m venv td-open3d
   td-open3d\Scripts\pip install open3d numpy
   ```

3. On the **Setup** page, set **Python exe** to that interpreter's `python.exe`, then pulse
   **Check worker**. The **Worker status** field should come back with something like
   `OK: open3d 0.19.0 on python 3.11.13 (numpy 2.3.3)`. If it does not, nothing else will work, so
   fix this before going further.

Developed against Open3D 0.19.0. Nothing in the worker is exotic, so nearby versions should be
fine, but that is the one I test.

## Connecting the cameras

The component does not create the camera operators, it reads the ones you make. Put them all in
one base COMP and point **Inputs op** at it.

1. Add one camera TOP per physical camera, inside that base, named after the device type with the
   device number appended: `kinectazure1`, `kinectazure2`, ... or `orbbec1`, `orbbec2`, ... or
   `zed1`, `zed2`, ... The names matter. Each `Device<n>` the component builds looks for the TOP
   whose number matches.

2. Pin each TOP to one camera: **Sensor** on the Kinect Azure TOP, **Device** on the Orbbec TOP,
   **Camera** on the ZED TOP. One TOP per camera, never two on the same serial.

3. Set **Inputs op** on the **Setup** page to that base, then pulse **Gather devices**. The
   component counts the cameras listed by the first TOP's device menu and builds a `Device<n>` for
   each, so **Number of devices** should match what you plugged in. If it comes back empty, or
   short, give the cameras a moment and pulse again: an Orbbec camera can take up to 30 seconds to
   appear after being plugged in.

Everything downstream is the same for every camera. The extra streams each `Device<n>` needs come
from select TOPs the component makes for you.

### Orbbec

Set **Depth Align Mode** on the Orbbec TOP to `Hardware` or `Software`. The colour camera and the
depth camera see the scene from slightly different places, so with alignment disabled the colours
that arrive on the merged cloud belong to the wrong points. On the Kinect the equivalent switch
lives on the select TOP and the component sets it itself; the Orbbec select has no such parameter,
so this one is yours.

If you have a Femto Mega or a Femto Bolt, prefer the **Kinect Azure TOP** with its **Hardware
Type** set to `Orbbec Compatible`, and set **Device type** to `Kinect Azure` here. Those cameras
support body tracking that way, which gets you the player index and therefore masking. Do not use
the Orbbec TOP and the Kinect Azure operators in the same project, TouchDesigner warns that the
combination is unstable.

The Orbbec path is built on TouchDesigner's Orbbec operators and verified inside TouchDesigner
without a camera attached. I have not been able to run it against physical Orbbec hardware yet, so
if something behaves oddly there, please open an issue.

### ZED

Set **Reference Frame** on the ZED TOP to `Camera`, not `World`. In world mode the point cloud is
expressed in the frame the camera started in and moves with the camera's own tracking, which fights
the calibration you are trying to measure. The point cloud itself is metres relative to the colour
camera, the same convention the other two use.

The ZED body mask is not wired up, so **Use mask for calibration** is greyed out. It exists on the
ZED TOP, but it holds body IDs, it only appears once body tracking is running through a ZED CHOP,
and the threshold here was tuned against the Kinect's player index. Ask for it if you want it, and
in the meantime `Custom TOPs` lets you supply any mask you like.

Same caveat as Orbbec: this path is built on TouchDesigner's ZED operators and checked inside
TouchDesigner without a camera attached, not against physical hardware.

## Using it

1. **Setup** page: pick your **Device type**, set **Inputs op** to the base holding your camera
   TOPs (see above), then pulse **Gather devices**. The component builds one `Device<n>` for each
   camera it finds and wires up its source TOPs.

2. **Calibrate** page: set **Specify pair** to the two cameras. The first field is the target,
   which does not move; the second is the source, which gets transformed onto it. The **IDs**
   field shows which physical devices those numbers refer to.

3. Pulse **Calibrate**. With the default **Mode** (on the **Registration** page) the component
   runs the global registration and then seeds ICP with its result, in one pass.

4. Repeat for each remaining pair, chaining outward from the reference camera. Calibrate 1 and 2,
   then 2 and 3, then 3 and 4. Every pairwise result is stored, and the component composes the
   chain so all clouds land in the frame of **Reference device**.

5. Read the merged result from the `out_pop` output.

Pulse **Refine** to run ICP again on an existing calibration. It never falls back to a fresh global
registration, so refine means refine. **Reset calibration** puts every device back at identity.
**Rebuild chain** recomposes the transforms without re-registering anything, which is what you want
after changing the reference device.

### Reading the result

After each calibration you get **Last status**, **Last fitness**, **Last RMSE** and **Last
correspondences**.

- `OK` means it converged and the numbers look sane.
- `WARN` means it converged but the overlap or the error is marginal. Look at the merged cloud
  before trusting it.
- `FAIL` means fitness under 0.05 or fewer than 100 corresponding points. The result is noise.

A confident-looking alignment from a bad global stage is the failure mode to watch for, so when a
chained run grades the global stage as `FAIL` the component skips the ICP rather than polishing a
wrong answer into a convincing one.

## Parameters

### Setup

| Parameter | What it does |
|---|---|
| Device type | `Kinect Azure`, `Orbbec`, `ZED` or `Custom TOPs`. Changing it rebuilds every device's source TOPs |
| Inputs op | The base COMP holding your camera TOPs. Every device type reads it, including `Custom TOPs`, where table cells are looked up inside it |
| Gather devices | Find the connected cameras and build a `Device<n>` for each |
| Number of devices, Devices | Read-only, what was found |
| Python exe | The interpreter that runs the registration. Must have Open3D |
| Check worker | Probe that interpreter and report its versions |
| Worker status | Result of the probe |

### Calibrate

| Parameter | Default | What it does |
|---|---|---|
| Reference device | 1 | Whose frame everything ends up in |
| Specify pair | 1, 2 | Target and source device numbers. The target does not move |
| IDs | | Read-only, the physical devices behind those numbers |
| Calibrate | | Run the current mode on the current pair |
| Refine | | ICP on top of the existing calibration for that pair |
| Rebuild chain | | Recompose all transforms without re-registering |
| Reset calibration | | Forget everything, all devices back to identity |
| Last status, fitness, RMSE, correspondences | | Read-only, how the last run went |

### Registration

The knobs behind a calibration run. The defaults are the ones I use.

| Parameter | Default | What it does |
|---|---|---|
| Mode | Global + ICP refine | `Global + ICP refine` chains both stages in one worker run. `Global registration only` stops after RANSAC so you can inspect it. `From table DAT` takes a 4x4 you produced elsewhere |
| Preset matrix DAT | | A 4x4 table DAT, used when Mode is `From table DAT` |
| Voxel size (m) | 0.05 | Downsampling for the global stage. Bigger is faster and coarser |
| Refine voxel (m) | 0.01 | Downsampling for ICP. 0 uses full resolution |
| Max range (m) | 0 | Drop points beyond this distance. 0 keeps everything |
| Use coloured ICP | off | Coloured ICP. Needs colour on both clouds and wants good overlap |
| Use mask for calibration | off | Register only the masked region. See below |
| RANSAC seed | -1 | Leave it at -1 and RANSAC uses every core, but the same scene can give a slightly different calibration each time. Set it to 0 or more and you get the same answer every run, at the cost of that parallel speedup: Open3D's RANSAC threads race, so reproducibility means running it on one thread |

**Use mask for calibration** restricts registration to a masked part of the cloud, which helps when
the cameras see a lot of static background. It only works where there is a mask to use. The Kinect
Azure select TOP publishes a player index, so it is available there. The Orbbec select TOP does not
publish one, so on Orbbec the toggle is greyed out. With custom sources it becomes available as
soon as at least one row supplies a mask TOP.

## Outputs

The component has one output, `out_pop`. It is a POP: every device's transformed cloud, merged
into a single point set in the reference camera's frame, carrying two attributes.

- `P`, the position
- `Color`, the colour that camera saw at that point

Only points a camera actually returned are in there. The pixels a depth sensor reports as invalid
are deleted inside each device, before the merge, so the count you get is the real one and every
colour belongs to the point it sits on.

Drop it into a geometry COMP to render it, feed it to instancing, or keep processing it with POPs.
You do not need to reach inside the component.

**Coming from 0.2.0:** this replaces the `out_points` and `out_colors` TOP outputs, which tiled the
clouds into two textures. If you were reading those, you now read one POP instead. A `POP to TOP`
gets you back to the old shape if you need it.

## Custom TOP sources

Set **Device type** to `Custom TOPs` and fill in the `customSources` table inside the component,
one row per device:

| name | pointcloud | color | mask |
|---|---|---|---|
| cam1 | `/project1/clouds/cloud1` | `/project1/clouds/color1` | |
| cam2 | `/project1/clouds/cloud2` | `/project1/clouds/color2` | |

Then pulse **Gather devices** and calibrate as usual. The colour TOP does not have to match the
point cloud TOP's resolution: it is sampled per point, not paired pixel by pixel. Leave `mask`
empty if you do not have one.

This is also how you add a camera the component does not know about yet. If your sensor has a
TouchDesigner TOP that can output a point cloud, adding a row to the `deviceTypes` table is enough,
no code involved.

## Limitations

The technique needs real overlap between cameras, seen from broadly similar angles. Cameras placed
at more than about 90 degrees to each other, and certainly cameras facing each other, do not give
RANSAC enough common structure to work with.

You can get around this by calibrating through intermediate positions, and the chain composition
does that part for you: calibrate 1 to 2 and 2 to 3, and camera 3 lands in camera 1's frame
even though 1 and 3 never saw the same thing. Move a camera through a path of overlapping views
and you can reach positions a direct pair could not.

The registration is a one-off calibration step, not a per-frame operation. A global registration on
real cameras takes seconds.

## Development

| Path | What |
|---|---|
| `src/` | the four Python files that become DATs in the component |
| `tests/` | the two suites that need no TouchDesigner |
| `tools/` | build and test scripts that run inside TouchDesigner |

`src/worker.py` is the part that runs outside TouchDesigner. It is a standalone script and you can
drive it by hand:

```
python src/worker.py --probe
python src/worker.py job.json
```

The job format is documented at the top of that file. The matrix it returns satisfies
`x_target = M @ x_source` with homogeneous column vectors.

To run the tests:

```
python tests/test_pipeline.py                    # numpy only, no Open3D, no TouchDesigner
python tests/test_worker.py path/to/python.exe   # drives the worker as a subprocess
```

`test_pipeline.py` deliberately asserts that the TouchDesigner side never imports Open3D, since
that is the thing that crashes the process.

## Support

You can follow me on:

[Instagram](https://www.instagram.com/darien.brito/) |
[Twitter](https://twitter.com/DarienBrito)

If you would like to go one step further with your support, you can subscribe here:
[Patreon](https://www.patreon.com/c/darienbrito)

Best,
Darien

Darien Brito, 2026. MIT licensed, see [LICENSE.md](LICENSE.md).
