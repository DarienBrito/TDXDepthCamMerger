"""
Save DemoTDXDepthCamMerger.toe from the OPEN master, then bring the master back.

The demo IS the master minus TDMCP: the same component, the same Inputs_1 /
Inputs_2 synthetic cameras, the same layout. Only what is true of this machine
is cleared first (the Python exe, the worker probe, the last calibration
readouts), and START_HERE is replaced with instructions for whoever opens the
file.

Run from TD, with the master open and the rig built:

    exec(open(r'<this file>', encoding='utf-8').read())

TDMCP lives outside the project, on this machine only, so a user opening the
demo would get a broken reference. Destroying it also cuts the connection this
call came in on, so the work waits three frames and writes each step to
Log/demo_save.log, which is the only record left once the connection is gone.

Nothing done here reaches the master: it is reloaded from disk at the end.
Expect TouchDesigner to be gone when it finishes. Relaunch it with the master.
"""

import os

REPO = globals().get('REPO') or project.folder.replace(os.sep, '/')
DEMO = REPO + '/DemoTDXDepthCamMerger.toe'
MASTER = REPO + '/TDXDepthCamMerger.0.2.toe'
LOG = REPO + '/Log/demo_save.log'
HOST = op('/ProjectName')
NAME = 'TDXDepthCamMerger'

if not os.path.isdir(REPO + '/Log'):
	os.makedirs(REPO + '/Log')

comp = HOST.op(NAME)
if comp is None:
	raise RuntimeError('no {} in the open project'.format(NAME))

# _______________________________________________ 1. drop what is local to here

comp.par.Pythonexe = ''
for name in ('Open3dstatus', 'Open3dversion', 'Pythonversion',
		'Laststatus', 'Lastfitness', 'Lastrmse'):
	comp.par[name] = ''
comp.par.Lastcorrespondences = 0

# _______________________________________________ 2. instructions for a visitor

readme = HOST.op('START_HERE') or HOST.create(textDAT, 'START_HERE')
readme.par.language = 'python'
readme.text = '''"""
TDXDepthCamMerger demo. Three synthetic depth cameras, no hardware needed.

Inputs_1 generates three overlapping point clouds of one scene, each in its own
camera frame, coloured red, green and blue. They start misaligned. Calibrating
brings them into one coordinate space. Inputs_2 is a second scene from
different poses: point the component's Inputs op at it and pulse Gather devices
to try another rig.

To run it:

  1. Select TDXDepthCamMerger. On the Setup page set Python exe to a python.exe
     that has open3d installed, then pulse Check worker. Worker status should
     report the versions. See the README for the install.

  2. Pulse Gather devices. Three devices appear, one per customSources row.

  3. Calibrate page: set Specify pair to 1 and 2, then pulse Calibrate. Watch
     the green cloud swing onto the red one.

  4. Set Specify pair to 2 and 3 and pulse Calibrate again. Blue joins them,
     composed through camera 2 into camera 1's frame.

The answers to look for are in Inputs_1/syntheticScene: camera 2 sits 25
degrees round and 40 cm to the side of camera 1, camera 3 is 18 degrees the
other way. Last fitness and Last RMSE report how well it did.
"""
'''

print('cleaned: Python exe, worker probe, last readouts, START_HERE')

# _______________________________________________ 3. the save, three frames on
#
# chr(10) instead of a newline escape: this code is a string inside a string,
# and a real newline here would break the generated call apart.

CODE = '''
import os
import traceback

log = open({log!r}, 'w')


def w(msg):
	log.write(msg + chr(10))
	log.flush()


w('--- start')
try:
	tdmcp = op('/ProjectName/TDMCP')
	w('tdmcp: ' + str(tdmcp))
	if tdmcp:
		tdmcp.destroy()
	w('tdmcp destroyed')
	w('project.save -> ' + str(project.save({demo!r})))
	w('demo bytes: ' + str(os.path.getsize({demo!r})))
	w('loading master')
	project.load({master!r})
	w('--- master reloaded')
except Exception:
	w(traceback.format_exc())
log.close()
'''.format(log=LOG, demo=DEMO, master=MASTER)

run(CODE, delayFrames=3)
print('save scheduled in 3 frames. Watch {}'.format(LOG))
print('TouchDesigner is expected to exit; relaunch it with {}'.format(MASTER))
